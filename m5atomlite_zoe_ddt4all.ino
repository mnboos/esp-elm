// ============================================================
//  M5Stack Atom Lite – ELM327 Emulator for Renault Zoe R240
//  Target: R-Link infotainment on CAN2 (500 kbps)
//  Use with DDT4ALL on Windows @ 115200 baud
// ============================================================

#include <M5Atom.h>

#include "driver/twai.h"

#define CAN_TX_PIN GPIO_NUM_26
#define CAN_RX_PIN GPIO_NUM_32

// State variables matching ELM327 defaults
bool echo_on = true;
bool headers_on = false;
uint32_t current_sh = 0x744;
uint32_t current_rx = 0x74C;

void setup()
{
    M5.begin(true, false, true);
    Serial.begin(115200);

    // Setup CAN for Renault Zoe (500k)
    twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_cfg, &t_cfg, &f_cfg);
    twai_start();

    M5.dis.fillpix(0x00FF00); // Green = Ready
    Serial.print("\r\nELM327 v1.5\r\n>");
}

void loop()
{
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\r');
        cmd.trim();
        if (cmd.length() == 0) {
            Serial.print("\r>");
            return;
        }

        // 1. Handle ECHO (Must be exactly what was sent + \r)
        if (echo_on) {
            Serial.print(cmd + "\r");
        }

        String upCmd = cmd;
        upCmd.toUpperCase();
        upCmd.replace(" ", "");

        // 2. Handle AT Commands
        if (upCmd.startsWith("AT")) {
            if (upCmd == "ATZ" || upCmd == "ATWS") {
                echo_on = true;
                headers_on = false;
                current_sh = 0x744;
                current_rx = 0x74C;
                Serial.print("ELM327 v1.5\r");
            } else if (upCmd == "ATI" || upCmd == "AT@1")
                Serial.print("ELM327 v1.5\r");
            else if (upCmd == "ATRV")
                Serial.print("12.6V\r");
            else if (upCmd == "ATDP")
                Serial.print("ISO 15765-4 (CAN 11/500)\r");
            else if (upCmd == "ATDPN")
                Serial.print("6\r");
            else if (upCmd.startsWith("ATE")) {
                echo_on = (upCmd.endsWith("1"));
                Serial.print("OK\r");
            } else if (upCmd.startsWith("ATH")) {
                headers_on = (upCmd.endsWith("1"));
                Serial.print("OK\r");
            } else if (upCmd.startsWith("ATSH")) {
                current_sh = strtoul(upCmd.substring(4).c_str(), NULL, 16);
                current_rx = current_sh + 8; // Auto-set RX
                Serial.print("OK\r");
            } else if (upCmd.startsWith("ATCRA")) {
                current_rx = strtoul(upCmd.substring(5).c_str(), NULL, 16);
                Serial.print("OK\r");
            } else {
                Serial.print("OK\r"); // Silence all other config AT commands
            }
        }
        // 3. Handle HEX Commands (CAN Requests)
        else {
            uint8_t payload[8];
            int len = 0;
            for (int i = 0; i < upCmd.length() && len < 8; i += 2) {
                payload[len++] = (uint8_t)strtoul(upCmd.substring(i, i + 2).c_str(), NULL, 16);
            }

            if (len > 0) {
                // Send CAN Frame
                twai_message_t tx;
                tx.identifier = current_sh;
                tx.extd = 0;
                tx.data_length_code = 8;
                for (int i = 0; i < 8; i++)
                    tx.data[i] = (i < len) ? payload[i] : 0xAA;
                twai_transmit(&tx, pdMS_TO_TICKS(50));

                // Listen for Response
                unsigned long start = millis();
                bool found = false;
                while (millis() - start < 500) {
                    twai_message_t rx;
                    if (twai_receive(&rx, pdMS_TO_TICKS(10)) == ESP_OK) {
                        if (rx.identifier == current_rx) {
                            M5.dis.fillpix(0xFFFF00); // Flash yellow

                            String response = "";
                            if (headers_on) {
                                char hdr[5];
                                sprintf(hdr, "%03X", rx.identifier);
                                response += String(hdr);
                            }
                            for (int i = 0; i < rx.data_length_code; i++) {
                                char b[4];
                                sprintf(b, "%02X", rx.data[i]);
                                response += String(b);
                                if (i < rx.data_length_code - 1)
                                    response += " ";
                            }
                            Serial.print(response + "\r");
                            found = true;
                            break;
                        }
                    }
                }
                if (!found)
                    Serial.print("NO DATA\r");
            }
        }

        // 4. Always end with the prompt
        Serial.print(">");
        M5.dis.fillpix(0x00FF00);
    }
}                echo_on = (upCmd.endsWith("1"));
                Serial.print("OK\r");
            } else if (upCmd.startsWith("ATH")) {
                headers_on = (upCmd.endsWith("1"));
                Serial.print("OK\r");
            } else if (upCmd.startsWith("ATSH")) {
                current_sh = strtoul(upCmd.substring(4).c_str(), NULL, 16);
                current_rx = current_sh + 8; // Auto-set RX
                Serial.print("OK\r");
            } else if (upCmd.startsWith("ATCRA")) {
                current_rx = strtoul(upCmd.substring(5).c_str(), NULL, 16);
                Serial.print("OK\r");
            } else {
                Serial.print("OK\r"); // Silence all other config AT commands
            }
        }
        // 3. Handle HEX Commands (CAN Requests)
        else {
            uint8_t payload[8];
            int len = 0;
            for (int i = 0; i < upCmd.length() && len < 8; i += 2) {
                payload[len++] = (uint8_t) strtoul(upCmd.substring(i, i + 2).c_str(), NULL, 16);
            }

            if (len > 0) {
                // Send CAN Frame
                twai_message_t tx;
                tx.identifier = current_sh;
                tx.extd = 0;
                tx.data_length_code = 8;
                for (int i = 0; i < 8; i++) tx.data[i] = (i < len) ? payload[i] : 0xAA;
                twai_transmit( & tx, pdMS_TO_TICKS(50));

                // Listen for Response
                unsigned long start = millis();
                bool found = false;
                while (millis() - start < 500) {
                    twai_message_t rx;
                    if (twai_receive( & rx, pdMS_TO_TICKS(10)) == ESP_OK) {
                        if (rx.identifier == current_rx) {
                            M5.dis.fillpix(0xFFFF00); // Flash yellow

                            String response = "";
                            if (headers_on) {
                                char hdr[5];
                                sprintf(hdr, "%03X", rx.identifier);
                                response += String(hdr);
                            }
                            for (int i = 0; i < rx.data_length_code; i++) {
                                char b[4];
                                sprintf(b, "%02X", rx.data[i]);
                                response += String(b);
                                if (i < rx.data_length_code - 1) response += " ";
                            }
                            Serial.print(response + "\r");
                            found = true;
                            break;
                        }
                    }
                }
                if (!found) Serial.print("NO DATA\r");
            }
        }

        // 4. Always end with the prompt
        Serial.print(">");
        M5.dis.fillpix(0x00FF00);
    }
}
