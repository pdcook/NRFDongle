// Example script for a host sending data to a dongle

// for host
#define NRF_HOST

// if host is nrf52, otherwise if it is an nrf24 define NRF24
#define NRF52

#include "nrf_dongle.h"
#include "elapsedMillis.h"

#ifdef NRF52
    #include "nrf_to_nrf.h"

    uint64_t device_id = 0; // will be read from nrf core

    nrf_to_nrf radio;

    uint8_t data_rate = NRF_2MBPS;
    uint8_t power_level = NRF_PA_HIGH;
#endif // NRF52

#ifdef NRF24
    #include <RF24.h>

    uint64_t device_id = 0x12345678AB; // set the device id

    RF24 radio(9, 10); // CE, CSN

    uint8_t data_rate = RF24_2MBPS;
    uint8_t power_level = RF24_PA_HIGH;
#endif // NRF24

uint32_t pair_timeout_millis = 120000; // 2 minutes
uint16_t ping_interval_millis = 2000; // 2 seconds
uint8_t retry_delay = 5; // delay is (x + 1) * 250us, default is 1.5ms
uint8_t retry_count = 15; // 15 retries

// timer so that transmission is not too frequent,
// but we can still update the radio every loop
elapsedMillis timer;

// data to send will be a float that is incremented
float data = 0.0;

// create a dongle object by providing
// the radio object
// a unique device id
// the ping interval in milliseconds
// the pair timeout in milliseconds
// the data rate
// the power level
// the retry delay
// the retry count
// as well as two consts for the packet size in bytes
// and buffer size in elements

// the channel will be determined by the device id

// for this example, the data is a float, with a buffer of 2 elements
NRFDongle<float, 2> dongle(radio, device_id, ping_interval_millis, pair_timeout_millis, data_rate, power_level, retry_delay, retry_count);

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

    #ifdef NRF52
        // read device id from nrf core
        uint32_t deviceIdLow = NRF_FICR->DEVICEID[0];
        uint32_t deviceIdHigh = NRF_FICR->DEVICEID[1];

        device_id = (static_cast<uint64_t>(deviceIdHigh) << 32) | deviceIdLow;

        dongle.set_unique_id(device_id);

    #endif // NRF52

    // start the radio
    dongle.begin();

    Serial.print("[HOST] Device ID: ");
    Uint64Bytes ub;
    ub.u = dongle.get_unique_id();
    Serial.print(ub.bytes[1], HEX);
    Serial.println(ub.bytes[0], HEX);

    // start the timer
    timer = 0;
}

void loop() {
    // update the radio
    dongle.update();


    if (timer < 500) {
        return;
    } else {
        timer = 0;
    }

    // get some data on the dongle
    bool enabled = dongle.is_enabled();
    bool paired = dongle.is_paired();
    uint8_t channel = dongle.get_channel();
    uint64_t address = dongle.get_address();

    if (!enabled) {
        Serial.println("[HOST] Disabled");
        return;
    }

    // try to send some data by adding it to the buffer,
    // not sending it immediately

    bool success = dongle.send(data, false);

    if (success) {
        // increment the data if it was successfully added to the buffer
        data += 1.5;
    }

    // print the information
    Serial.print("[HOST] Paired: ");
    Serial.print(paired);
    Serial.print(", Channel: ");
    Serial.print(channel);
    Serial.print(", Address: ");
    Uint64Bytes ub;
    ub.u = address;
    Serial.print(ub.bytes[1], HEX);
    Serial.print(ub.bytes[0], HEX);
    Serial.print(", Data: ");
    Serial.print(data);
    Serial.print(", Success: ");
    Serial.println(success);
}
