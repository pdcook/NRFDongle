// Example script for a dongle receiving data from a host

// for dongle
#define NRF_DONGLE

// if dongle is nrf52, otherwise if it is an nrf24 define NRF24
#define NRF52

#include "nrf_dongle.h"
#include "elapsedMillis.h"

#ifdef NRF52
    #include "nrf_to_nrf.h"

    nrf_to_nrf radio;

    uint8_t data_rate = NRF_2MBPS;
    uint8_t power_level = NRF_PA_HIGH;
#endif // NRF52

#ifdef NRF24
    #include <RF24.h>

    RF24 radio(9, 10); // CE, CSN

    uint8_t data_rate = RF24_2MBPS;
    uint8_t power_level = RF24_PA_HIGH;
#endif // NRF24

uint8_t channel = 90;
uint32_t pair_timeout_millis = 120000; // 2 minutes
uint16_t ping_interval_millis = 1000; // 1 second

// create a dongle object by providing
// the radio object
// a unique device id (unused for dongle)
// the channel
// the ping interval in milliseconds (unused for dongle)
// the pair timeout in milliseconds
// the data rate
// the power level
// as well as two consts for the packet size in bytes
// and buffer size in elements

// for this example, 4 bytes for a float, with a buffer of 2 elements
NRFDongle<4, 2> dongle(radio, 0, channel, ping_interval_millis, pair_timeout_millis, data_rate, power_level);

// timer so that transmission is not too frequent,
// but we can still update the radio every loop
elapsedMillis timer;

// union for converting between float and bytes
union FloatBytes {
    float f;
    uint8_t bytes[4];
};

// function for creating a packet from a float
void create_packet(float f, Packet<4>& packet) {
    FloatBytes fb;
    fb.f = f;

    packet.data[0] = fb.bytes[0];
    packet.data[1] = fb.bytes[1];
    packet.data[2] = fb.bytes[2];
    packet.data[3] = fb.bytes[3];
}

// function for extracting a float from a packet
float extract_packet(Packet<4>& packet) {
    FloatBytes fb;

    fb.bytes[0] = packet.data[0];
    fb.bytes[1] = packet.data[1];
    fb.bytes[2] = packet.data[2];
    fb.bytes[3] = packet.data[3];

    return fb.f;
}

// union for converting between uint64_t and two uint32_t
// since Arduino's Serial.print does not support printing uint64_t
union Uint64Bytes {
    uint64_t u;
    uint32_t bytes[2];
};

void setup() {
    Serial.begin(115200);

    while (!Serial) {
        // wait for serial port to connect. Needed for native USB port only
        // NOTE: on some boards, this will cause the board to hang
        // until a serial connection is established
        delay(10);
    }

    // start the radio
    dongle.begin();

    Serial.println("[DONGLE] Started");

    // start the timer
    timer = 0;
}

bool was_paired = false;

void loop() {
    // update the radio
    dongle.update();

    // get some data on the dongle
    bool enabled = dongle.is_enabled();
    bool paired = dongle.is_paired();
    uint64_t address = dongle.get_address();
    Uint64Bytes ub;
    ub.u = address;

    // if not paired, don't print too often
    if (!paired) {
        if (timer > 1000) {
            if (!enabled) {
                Serial.println("[DONGLE] Not enabled");
            } else {
                Serial.print("[DONGLE] Not paired. Address: ");
                Serial.print(ub.bytes[1], HEX);
                Serial.println(ub.bytes[0], HEX);
            }
            timer = 0;
        }
        return;
    } else if (!was_paired) {
        Serial.print("[DONGLE] Paired with ");
        Serial.print(ub.bytes[1], HEX);
        Serial.println(ub.bytes[0], HEX);
        was_paired = true;
    }

    // once paired, check for data, and print it
    // immediately with some extra info
    if (!dongle.has_data()) {
        return;
    }

    Packet<4> packet;
    dongle.read(packet, true);

    float data = extract_packet(packet);

    Serial.print("[DONGLE] Received ");
    Serial.print(data);
    Serial.print(" from ");
    Serial.print(ub.bytes[1], HEX);
    Serial.println(ub.bytes[0], HEX);

}
