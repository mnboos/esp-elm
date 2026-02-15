// ============================================================
//  M5Stack Atom Lite – ELM327 Emulator for Renault Zoe R240
//  Target: R-Link infotainment on CAN2 (500 kbps)
//  Use with DDT4ALL on Windows @ 115200 baud
// ============================================================

#include <M5Atom.h>
#include "driver/twai.h"

#define CAN_TX_PIN  GPIO_NUM_26
#define CAN_RX_PIN  GPIO_NUM_32
#define SERIAL_BAUD 115200

static bool echo_on     = true;
static bool linefeed_on = false;
static bool headers_on  = true;
static bool spaces_on   = true;
static bool can_running = false;
static int  protocol    = 6;

bool startCAN(twai_timing_config_t *t_cfg) {
    if (can_running) {
        twai_stop();
        twai_driver_uninstall();
        can_running = false;
        delay(50);
    }
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, t_cfg, &f_config) != ESP_OK) return false;
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }
    can_running = true;
    return true;
}

void sendReply(const String &msg) {
    if (msg.length() > 0) {
        Serial.print(msg);
        Serial.print("\r");
    }
    Serial.print(">");
}

String canRequest(uint32_t tx_id, const uint8_t *data, uint8_t dlen,
                  uint32_t rx_id, uint32_t timeout_ms = 300) {
    if (!can_running) return "";

    twai_message_t tx = {};
    tx.identifier       = tx_id;
    tx.data_length_code = 8;
    tx.data[0] = dlen;
    for (int i = 0; i < dlen && i < 7; i++) tx.data[i + 1] = data[i];
    for (int i = dlen + 1; i < 8; i++) tx.data[i] = 0xCC;

    if (twai_transmit(&tx, 0) != ESP_OK) return "CAN ERROR";

    String result       = "";
    unsigned long start = millis();
    bool fc_sent        = false;
    uint8_t sn_expected = 1;

    while (millis() - start < timeout_ms) {
        twai_message_t rx = {};
        if (twai_receive(&rx, pdMS_TO_TICKS(20)) != ESP_OK) continue;

        bool match = (rx.identifier == rx_id) ||
                     (rx.identifier >= 0x7E8 && rx.identifier <= 0x7EF) ||
                     (rx.identifier >= 0x740 && rx.identifier <= 0x7FF);
        if (!match) continue;

        M5.dis.fillpix(0xFFFF00);

        uint8_t pci        = rx.data[0];
        uint8_t frame_type = (pci >> 4) & 0x0F;

        if (frame_type == 0) {
            if (headers_on) {
                char hdr[12];
                snprintf(hdr, sizeof(hdr), "%03X ", (unsigned int)rx.identifier);
                result += hdr;
            }
            for (int i = 0; i < rx.data_length_code; i++) {
                char b[4];
                snprintf(b, sizeof(b), spaces_on ? "%02X " : "%02X", rx.data[i]);
                result += b;
            }
            result.trim();
            result += "\r";
            break;

        } else if (frame_type == 1) {
            if (headers_on) {
                char hdr[12];
                snprintf(hdr, sizeof(hdr), "%03X ", (unsigned int)rx.identifier);
                result += hdr;
            }
            for (int i = 0; i < rx.data_length_code; i++) {
                char b[4];
                snprintf(b, sizeof(b), spaces_on ? "%02X " : "%02X", rx.data[i]);
                result += b;
            }
            result.trim();
            result += "\r";

            if (!fc_sent) {
                twai_message_t fc = {};
                fc.identifier       = tx_id;
                fc.data_length_code = 8;
                fc.data[0] = 0x30;
                fc.data[1] = 0x00;
                fc.data[2] = 0x00;
                for (int i = 3; i < 8; i++) fc.data[i] = 0xCC;
                twai_transmit(&fc, 0);
                fc_sent     = true;
                sn_expected = 1;
                timeout_ms  = 600;
            }

        } else if (frame_type == 2) {
            uint8_t sn = pci & 0x0F;
            if (sn == sn_expected) {
                if (headers_on) {
                    char hdr[12];
                    snprintf(hdr, sizeof(hdr), "%03X ", (unsigned int)rx.identifier);
                    result += hdr;
                }
                for (int i = 0; i < rx.data_length_code; i++) {
                    char b[4];
                    snprintf(b, sizeof(b), spaces_on ? "%02X " : "%02X", rx.data[i]);
                    result += b;
                }
                result.trim();
                result += "\r";
                sn_expected = (sn_expected + 1) & 0x0F;
            }
        }
    }

    return result;
}

String dispatchCANCommand(const String &cmd) {
    if (!can_running) return "CAN ERROR";

    uint32_t tx_id   = 0x744;
    uint32_t rx_id   = 0x74C;
    uint8_t  data[7] = {0};
    uint8_t  dlen    = 0;

    int hash = cmd.indexOf('#');
    if (hash > 0) {
        tx_id = (uint32_t)strtoul(cmd.substring(0, hash).c_str(), nullptr, 16);
        rx_id = tx_id + 8;
        String hex = cmd.substring(hash + 1);
        for (int i = 0; i + 1 < (int)hex.length() && dlen < 7; i += 2) {
            data[dlen++] = (uint8_t)strtoul(hex.substring(i, i + 2).c_str(), nullptr, 16);
        }
    } else {
        for (int i = 0; i + 1 < (int)cmd.length() && dlen < 7; i += 2) {
            data[dlen++] = (uint8_t)strtoul(cmd.substring(i, i + 2).c_str(), nullptr, 16);
        }
        if (data[0] == 0x01 || data[0] == 0x09) {
            tx_id = 0x7DF;
            rx_id = 0x7E8;
        }
    }

    if (dlen == 0) return "?";   // ← ADD THIS before canRequest()

    String resp = canRequest(tx_id, data, dlen, rx_id);
    if (resp.length() == 0) return "NO DATA";
    return resp;
}

String handleAT(const String &cmd) {

    if (cmd.startsWith("ST")) {
        return "?";
    }

    if (cmd == "ATZ" || cmd == "ATWS") {
        echo_on     = true;        // WAS false
        linefeed_on = false;
        headers_on  = true;
        spaces_on   = true;
        protocol    = 6;
        return "ELM327 v1.5";
    }

    if (cmd == "ATI")   return "ELM327 v1.5";
    if (cmd == "AT@1")  return "OBDII to RS232 Interpreter";
    if (cmd == "AT@2")  return "?";
    if (cmd == "ATRV")  return "12.0V";

    if (cmd == "ATE0")  { echo_on = false;     return "OK"; }
    if (cmd == "ATE1")  { echo_on = true;      return "OK"; }

    if (cmd == "ATL0")  { linefeed_on = false; return "OK"; }
    if (cmd == "ATL1")  { linefeed_on = true;  return "OK"; }

    if (cmd == "ATH0")  { headers_on = false;  return "OK"; }
    if (cmd == "ATH1")  { headers_on = true;   return "OK"; }

    if (cmd == "ATS0")  { spaces_on = false;   return "OK"; }
    if (cmd == "ATS1")  { spaces_on = true;    return "OK"; }

    if (cmd == "ATAT0" || cmd == "ATAT1" || cmd == "ATAT2") return "OK";

    if (cmd.startsWith("ATCFC")) return "OK";
    if (cmd.startsWith("ATFC"))  return "OK";
    if (cmd.startsWith("ATCA"))  return "OK";
    if (cmd.startsWith("ATCM"))  return "OK";
    if (cmd.startsWith("ATCF"))  return "OK";
    if (cmd.startsWith("ATCP"))  return "OK";
    if (cmd.startsWith("ATSH"))  return "OK";
    if (cmd.startsWith("ATCEA")) return "OK";
    if (cmd.startsWith("ATST"))  return "OK";

    if (cmd.startsWith("ATSP")) {
        protocol = (int)strtol(cmd.substring(4).c_str(), nullptr, 16);
        twai_timing_config_t t500 = TWAI_TIMING_CONFIG_500KBITS();
        twai_timing_config_t t250 = TWAI_TIMING_CONFIG_250KBITS();
        switch (protocol) {
            case 0:
                if (!startCAN(&t500)) startCAN(&t250);
                break;
            case 6:
            case 7:
                startCAN(&t500);
                break;
            case 8:
            case 9:
                startCAN(&t250);
                break;
            default:
                startCAN(&t500);
                break;
        }
        return "OK";
    }

    if (cmd == "ATDP")  return "ISO 15765-4 (CAN 500/11)";
    if (cmd == "ATDPN") return "A6";
    if (cmd == "ATPC")  return "OK";

    if (cmd == "ATM0" || cmd == "ATM1") return "OK";
    if (cmd == "ATBD" || cmd == "ATLP") return "OK";

    return "OK";
}

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(750);
    Serial.flush();
    while (Serial.available() > 0) Serial.read();

    M5.begin(false, false, true);
    M5.dis.fillpix(0x0000FF);

    twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_500KBITS();
    pinMode(CAN_RX_PIN, INPUT_PULLUP);
    if (!startCAN(&t_cfg)) {
        M5.dis.fillpix(0xFF0000); // Red = CAN init failed
    } else {
        M5.dis.fillpix(0x00FF00);
    }

    // FIX 2: Send the initial prompt so DDT4ALL doesn't time out on first ATZ
    Serial.print(">");
}

void loop() {
    M5.update();

    if (!Serial.available()) return;

    String cmd = Serial.readStringUntil('\r');
    cmd.replace("\n", "");
    cmd.trim();
    
    if (cmd.length() == 0) return;

    if (echo_on) {
        Serial.print(cmd + "\r");  // echo the original mixed-case command
    }

    cmd.toUpperCase();
    M5.dis.fillpix(0x00FFFF);

    String response;

    if (cmd.startsWith("AT")) {
        response = handleAT(cmd);
    } else if (cmd == "?") {
        response = "?";
    } else {
        response = dispatchCANCommand(cmd);
    }

    sendReply(response);
    M5.dis.fillpix(0x00FF00);
}
