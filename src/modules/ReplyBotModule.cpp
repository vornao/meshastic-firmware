#include "configuration.h"
#if !MESHTASTIC_EXCLUDE_REPLYBOT
/*
 * ReplyBotModule.cpp
 *
 * This module implements a simple reply bot for the Meshtastic firmware.  It listens for
 * specific text commands ("/ping", "/hello" and "/test") delivered either via a direct
 * message (DM) or a broadcast on the primary channel.  When a supported command is
 * received the bot responds with a short status message that includes the hop count
 * (minimum number of relays), RSSI and SNR of the received packet.  To avoid spamming
 * the network it enforces a per‑sender cooldown between responses.  By default the
 * module is disabled. See the official firmware documentation for guidance on adding modules.
 * To enable this module, set `#undef MESHTASTIC_EXCLUDE_REPLYBOT` in your variant.h file.
 */

#include "Channels.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "ReplyBotModule.h"
#include "mesh/MeshTypes.h"
#include "PowerStatus.h"

#include <Arduino.h>
#include <cctype>
#include <cstring>

//
// Rate limiting data structures
//
// Each sender is tracked in a small ring buffer.  When a message arrives from a
// sender we check the last time we responded to them.  If the difference is
// less than the configured cooldown (different values for DM vs broadcast)
// the message is ignored; otherwise we update the last response time and
// proceed with replying.

struct ReplyBotCooldownEntry {
    uint32_t from = 0;
    uint32_t lastMs = 0;
};

static constexpr uint8_t REPLYBOT_COOLDOWN_SLOTS = 8;          // ring buffer size
static constexpr uint32_t REPLYBOT_DM_COOLDOWN_MS = 15 * 1000; // 15 seconds for DMs
static constexpr uint32_t REPLYBOT_LF_COOLDOWN_MS = 60 * 1000; // 60 seconds for LongFast broadcasts

static ReplyBotCooldownEntry replybotCooldown[REPLYBOT_COOLDOWN_SLOTS];
static uint8_t replybotCooldownIdx = 0;

// Return true if a reply should be rate‑limited for this sender, updating the
// entry table as needed.
static bool replybotRateLimited(uint32_t from, uint32_t cooldownMs)
{
    const uint32_t now = millis();
    for (auto &e : replybotCooldown) {
        if (e.from == from) {
            // Found existing entry; check if cooldown expired
            if ((uint32_t)(now - e.lastMs) < cooldownMs) {
                return true;
            }
            e.lastMs = now;
            return false;
        }
    }
    // No entry found – insert new sender into the ring
    replybotCooldown[replybotCooldownIdx].from = from;
    replybotCooldown[replybotCooldownIdx].lastMs = now;
    replybotCooldownIdx = (replybotCooldownIdx + 1) % REPLYBOT_COOLDOWN_SLOTS;
    return false;
}

// Constructor – registers a single text port and marks the module promiscuous
// so that broadcast messages on the primary channel are visible.
ReplyBotModule::ReplyBotModule() : SinglePortModule("replybot", meshtastic_PortNum_TEXT_MESSAGE_APP)
{
    isPromiscuous = true;
}

void ReplyBotModule::setup()
{
    // In future we may add a protobuf configuration; for now the module is
    // always enabled when compiled in.
}

// Determine whether we want to process this packet.  We only care about
// plain text messages addressed to our port.
bool ReplyBotModule::wantPacket(const meshtastic_MeshPacket *p)
{
    return (p && p->decoded.portnum == ourPortNum);
}

uint32_t getBatteryPercent(const uint32_t voltageMv) {
    // Simple linear mapping for demonstration purposes
    const uint32_t minVoltage = 3300; // 3.3V = 0%
    const uint32_t maxVoltage = 4200; // 4.2V = 100%
    if (voltageMv <= minVoltage) {
        return 0;
    } else if (voltageMv >= maxVoltage) {
        return 100;
    } else {
        return (voltageMv - minVoltage) * 100 / (maxVoltage - minVoltage);
    }
}

ProcessMessage ReplyBotModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    // Accept only direct messages to us or broadcasts on the Primary channel
    // (regardless of modem preset: LongFast, MediumFast, etc).

    const uint32_t ourNode = nodeDB->getNodeNum();
    const bool isDM = (mp.to == ourNode);
    const bool isPrimaryChannel = (mp.channel == channels.getPrimaryIndex()) && isBroadcast(mp.to);
    if (!isDM && !isPrimaryChannel) {
        return ProcessMessage::CONTINUE;
    }

    // Ignore empty payloads
    if (mp.decoded.payload.size == 0) {
        return ProcessMessage::CONTINUE;
    }

    // Copy payload into a null‑terminated buffer
    char buf[260];
    memset(buf, 0, sizeof(buf));
    size_t n = mp.decoded.payload.size;
    if (n > sizeof(buf) - 1)
        n = sizeof(buf) - 1;
    memcpy(buf, mp.decoded.payload.bytes, n);

    // React only to supported slash commands
    if (!isCommand(buf)) {
        return ProcessMessage::CONTINUE;
    }

    // Apply rate limiting per sender depending on DM/broadcast
    const uint32_t cooldownMs = isDM ? REPLYBOT_DM_COOLDOWN_MS : REPLYBOT_LF_COOLDOWN_MS;
    if (replybotRateLimited(mp.from, cooldownMs)) {
        return ProcessMessage::CONTINUE;
    }

    // Compute hop count indicator – if the relay_node is non‑zero we know
    // there was at least one relay.  Some firmware builds support a hop_start
    // field which could be used for more accurate counts, but here we use
    // the available relay_node flag only.
    // int hopsAway = mp.hop_start - mp.hop_limit;
    int hopsAway = getHopsAway(mp);

    // Normalize RSSI: if positive adjust down by 200 to align with typical values
    int rssi = mp.rx_rssi;
    if (rssi > 0) {
        rssi -= 200;
    }
    float snr = mp.rx_snr;

    // Build the reply message and send it back via DM
    char reply[96];
    if (strncmp(buf, "/battery", 8) == 0 || strncmp(buf, "/bat", 4) == 0) {
        // For the battery command, include the battery voltage if available
            uint32_t voltageMv = 0;
            uint32_t batteryPercent = 0;
            if (powerStatus && powerStatus->getHasBattery()) {
                voltageMv = powerStatus->getBatteryVoltageMv();
                batteryPercent = getBatteryPercent(voltageMv);
                snprintf(reply, sizeof(reply), "🔋 %d%% (%dmV) | %d hops | RSSI %d | SNR %.1f",
                        batteryPercent, voltageMv, hopsAway, rssi, snr);
            }
            else {
                snprintf(reply, sizeof(reply), "🔋 Battery info unavailable | %d hops | RSSI %d | SNR %.1f",
                        hopsAway, rssi, snr);
            }    
            
    } else {
        // else for other commands, keep the original reply format
        snprintf(reply, sizeof(reply), "🎙️ Mic Check : %d Hops away | RSSI %d | SNR %.1f", hopsAway, rssi, snr);
    }
    sendDm(mp, reply);
    return ProcessMessage::CONTINUE;
}

// Check if the message starts with one of the supported commands.  Leading
// whitespace is skipped and commands must be followed by end‑of‑string or
// whitespace.
bool ReplyBotModule::isCommand(const char *msg) const
{
    if (!msg)
        return false;
    while (*msg == ' ' || *msg == '\t')
        msg++;
    auto isEndOrSpace = [](char c) { return c == '\0' || std::isspace(static_cast<unsigned char>(c)); };
    if (strncmp(msg, "/ping", 5) == 0 && isEndOrSpace(msg[5]))
        return true;
    if (strncmp(msg, "/hello", 6) == 0 && isEndOrSpace(msg[6]))
        return true;
    if (strncmp(msg, "/test", 5) == 0 && isEndOrSpace(msg[5]))
        return true;
    if (strncmp(msg, "/battery", 8) == 0 && isEndOrSpace(msg[8]))
        return true;
    if (strncmp(msg, "/bat", 4) == 0 && isEndOrSpace(msg[4]))
        return true;
    return false;
}

// Send a direct message back to the originating node.
void ReplyBotModule::sendDm(const meshtastic_MeshPacket &rx, const char *text)
{
    if (!text)
        return;
    meshtastic_MeshPacket *p = allocDataPacket();
    p->to = rx.from;
    p->channel = rx.channel;
    p->want_ack = false;
    p->decoded.want_response = false;
    size_t len = strlen(text);
    if (len > sizeof(p->decoded.payload.bytes)) {
        len = sizeof(p->decoded.payload.bytes);
    }
    p->decoded.payload.size = len;
    memcpy(p->decoded.payload.bytes, text, len);
    service->sendToMesh(p);
}
#endif // MESHTASTIC_EXCLUDE_REPLYBOT