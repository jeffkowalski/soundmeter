#if 0
/**
 * ESP32 I2S UDP Streamer
 *
 * This is influenced by maspetsberger's NoiseLevel at
 * https://github.com/maspetsberger/esp32-i2s-mems/blob/master/examples/NoiseLevel/NoiseLevel.ino
 *
 * @author GrahamM
 * From [[https://gist.github.com/GrahamM/1d5ded26b23f808a80520e8c1510713a][I2S_MIC_UDP.ino]]
 * Modified by Jeff Kowalski for local use
 * Listen with "ffplay -f s32le -ar 44100 -ac 1 udp://localhost:3210"
 */

#include <Arduino.h>
#include <driver/i2s.h>
#include <soc/i2s_reg.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include "credentials.h"

// WiFi network name and password:
const char* ssidName = WIFI_SSID;
const char* ssidPswd = WIFI_PSK;

// UDP Destination
IPAddress udpAddress(192, 168, 7, 60);
const int udpPort = 3210;
// Connection state
boolean connected = false;

//The udp library class
AsyncUDP udp;

const i2s_port_t I2S_PORT = I2S_NUM_0;
const int BLOCK_SIZE = 1024;

void setupWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssidName, ssidPswd);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("WiFi Failed");
        while (1) {
            delay(1000);
        }
    }
}

void I2SSetup() {
    esp_err_t err;

    // The I2S config as per the example
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // We drive clk but only receive
                               // 112000 and below seem to be cleaner
        .sample_rate = 44100, //16000, 32000, 44100, 48000, 96000, 112000, 128000, 144000, 160000, 176000, 192000
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // WS signal must be BCLK/64 - this is how we manage it
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // Left by default
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
        .dma_buf_count = 4,                           // number of buffers
        .dma_buf_len = BLOCK_SIZE,                    // samples per buffer
    };

    // The pin config as per the setup
    const i2s_pin_config_t pin_config = {
        .bck_io_num = 14,   // Bit Clk
        .ws_io_num = 12,    // LR Clk
        .data_out_num = -1, // Data out
        .data_in_num = 32   // Data in
    };

    // Configuring the I2S driver and pins.
    err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf(" ! Failed installing driver: %d\n", err);
        while (true);
    }

    // Alterations for SPH0645 to ensure we receive MSB correctly.
    REG_SET_BIT(I2S_TIMING_REG(I2S_PORT), BIT(9));   // I2S_RX_SD_IN_DELAY
    REG_SET_BIT(I2S_CONF_REG(I2S_PORT), I2S_RX_MSB_SHIFT);  // Phillips I2S - WS changes a cycle earlier

    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        Serial.printf(" ! Failed setting pin: %d\n", err);
        while (true);
    }
    Serial.println(" + I2S driver installed.");
}
void setup() {
    Serial.begin(115200);
    Serial.println(" * Configuring WiFi");
    setupWiFi();
    Serial.println(" * Configuring I2S");
    I2SSetup();
}


int32_t buffer[BLOCK_SIZE];    // Effectively two 1024 byte buffers

void loop() {
    if (!connected) {
        if (udp.connect(udpAddress, udpPort)) {
            connected = true;
            Serial.println(" * Connected to host");
        }
    }
    size_t num_bytes_read;
    i2s_read(I2S_PORT, (char*)buffer, BLOCK_SIZE, &num_bytes_read, portMAX_DELAY);
    udp.write( (uint8_t *)buffer, num_bytes_read);
}
#endif
