#ifndef NRF_DONGLE_H
#define NRF_DONGLE_H

// ============ NOTE ============
// For host, define NRF_HOST
// For dongle, define NRF_DONGLE
// ==============================

// this library can either use the nrf24 or nrf52
// to use the nrf24, define NRF24
// to use the nrf52, define NRF52

// if neither NRF_HOST or NRF_DONGLE is defined,
// or neither NRF24 or NRF52 is defined,
// or if both from each set are defined,
// throw a compile error

#if !defined(NRF_HOST) && !defined(NRF_DONGLE)
    #error "Either NRF_HOST or NRF_DONGLE must be defined before including nrf_dongle.h"
#endif

#if !defined(NRF24) && !defined(NRF52)
    #error "Either NRF24 or NRF52 must be defined before including nrf_dongle.h"
#endif

#if defined(NRF_HOST) && defined(NRF_DONGLE)
    #error "Only one of NRF_HOST or NRF_DONGLE can be defined before including nrf_dongle.h"
#endif

#if defined(NRF24) && defined(NRF52)
    #error "Only one of NRF24 or NRF52 can be defined before including nrf_dongle.h"
#endif

#ifdef NRF24
    #include <SPI.h>
    // https://github.com/nRF24/RF24
    #include <RF24.h>
    typedef RF24 Radio;
#endif // NRF24

#ifdef NRF52
    // https://github.com/TMRh20/nrf_to_nrf/
    #include "nrf_to_nrf.h"
    typedef nrf_to_nrf Radio;
#endif // NRF52

// https://github.com/rlogiacco/CircularBuffer/
#include <CircularBuffer.hpp>

// https://github.com/pfeerick/elapsedMillis
#include <elapsedMillis.h>

// pairing address, CANNOT be 0
// only 40 bits are used, as addresses are only 5 bytes
const uint64_t _PAIR_ADDRESS_ = 1;

// pairing channel
const uint8_t _PAIR_CHANNEL_ = 0;

// Packet, wraps TData with a ping flag
template <typename TData> struct Packet {
    TData data;
    bool ping = false;
};

// Pairing Packet, contains the unique_id of the host,
// the program_id, which must match the program_id of the dongle,
// and the ping interval in milliseconds
struct PairingPacket {
    uint64_t unique_id;
    uint64_t program_id;
    uint16_t ping_interval_millis;
};

template <typename TData, uint8_t max_packets> class NRFDongle {
    public:
        // take in a reference to the radio, a unique identifier for this radio
        // the ping interval in milliseconds,
        // pairing timeout in milliseconds (0 for no timeout),
        // and settings for the radio:
        //     data rate, power level,
        //     retry delay (x+1)*250us [x = 0-15]
        //     number of retries [0-15]
        #ifdef NRF24
            NRFDongle(
                        Radio &radio,
                        uint64_t unique_id, // unused for dongle
                        uint64_t program_id, // host and dongle must match
                        uint16_t ping_interval_millis, // unused for dongle
                        uint32_t pair_timeout_millis,
                        uint8_t data_rate,
                        uint8_t power_level,
                        uint8_t retry_delay = 5, // (5+1)*250us = 1.5ms
                        uint8_t retry_count = 15,
                        uint8_t ce_pin = 29,
                        uint8_t csn_pin = 5,
                        uint8_t rx_pin = 4,
                        uint8_t sck_pin = 2,
                        uint8_t tx_pin = 3
                    );
        #endif // NRF24
        #ifdef NRF52
            NRFDongle(
                        Radio &radio,
                        uint64_t unique_id, // unused for dongle
                        uint64_t program_id, // host and dongle must match
                        uint16_t ping_interval_millis, // unused for dongle
                        uint32_t pair_timeout_millis,
                        uint8_t data_rate,
                        uint8_t power_level,
                        uint8_t retry_delay = 5, // (5+1)*250us = 1.5ms
                        uint8_t retry_count = 15
                    );
        #endif // NRF52

        void begin();
        void update();
        void end();
        bool unpair();
        bool is_paired();
        bool is_enabled();
        uint64_t get_address();
        uint64_t get_unique_id();
        uint8_t get_channel();
        uint64_t get_program_id();
        void set_unique_id(uint64_t unique_id);

        // method for whether or not there is data in the buffer
        bool has_data();

        Radio &get_radio();

        #ifdef NRF_HOST
            bool send(TData data, bool send_now = false);

            bool ping();
        #endif // NRF_HOST

        #ifdef NRF_DONGLE
            bool read(TData &data, bool pop = true);
        #endif // NRF_DONGLE

    private:
        Radio &radio;
        bool enabled = false;
        bool paired = false;
        uint64_t address;
        uint64_t unique_id;
        uint64_t program_id;
        uint8_t channel;
        uint8_t data_rate;
        uint8_t power_level;
        uint8_t retry_delay;
        uint8_t retry_count;
        CircularBuffer<TData, max_packets> buffer;
        elapsedMillis ping_timer;
        elapsedMillis pair_timer;
        uint16_t ping_interval_millis;
        uint32_t pair_timeout_millis;

        #ifdef NRF24
            uint8_t ce_pin;
            uint8_t csn_pin;
            uint8_t rx_pin;
            uint8_t sck_pin;
            uint8_t tx_pin;
        #endif // NRF24

        bool try_pair();
};

// Implementation

// Constructor
template <typename TData, uint8_t max_packets> NRFDongle<TData, max_packets>::NRFDongle(Radio &radio, uint64_t unique_id, uint64_t program_id, uint16_t ping_interval_millis, uint32_t pair_timeout_millis, uint8_t data_rate, uint8_t power_level, uint8_t retry_delay, uint8_t retry_count
    #ifdef NRF24
        ,
        uint8_t ce_pin,
        uint8_t csn_pin,
        uint8_t rx_pin,
        uint8_t sck_pin,
        uint8_t tx_pin
    #endif // NRF24
        ) : radio(radio) {

    // US law restricts the use of the 2.4 GHz band
    // specifically, frequencies between 2.4-2.473 GHz
    // are generally allowed.
    // This corresponds to nRF channels 0-73
    // (https://github.com/mysensors/MySensors/blob/ee91d5da38a1435cb4b55476447e763f52540c3c/MyConfig.h#L455)
    // 0 -> 2400 MHz
    // 1 -> 2401 MHz
    // n -> 2400 + n MHz
    // Channel 0 is reserved for pairing (not yet implemented)
    // https://en.wikipedia.org/wiki/List_of_WLAN_channels

    // the channel will be determined by the unique_id of
    // the host, specifically
    // channel = (unique_id % 73) + 1
    // so the channel will be between 1 and 73
    // since 0 is reserved for pairing

    // if this is the host, the unique_id will be the address
    // if this is the dongle, the unique_id is unused
    //     and the address will later be the address of the host

    // set enabled in the constructor to prevent powerup sequence on first begin
    this->enabled = true;

    // for now, since we need to pair, the addres is _PAIR_ADDRESS_
    this->address = _PAIR_ADDRESS_;
    this->paired = false;

    // set the unique_id, which will be used later
    this->unique_id = unique_id;

    // set the program_id, which must match the program_id of the host
    this->program_id = program_id;

    // set the channel to the pairing channel
    this->channel = _PAIR_CHANNEL_;

    // set the ping interval
    this->ping_interval_millis = ping_interval_millis;

    // set the pair timeout
    this->pair_timeout_millis = pair_timeout_millis;

    // save radio settings
    this->data_rate = data_rate;
    this->power_level = power_level;
    this->retry_delay = retry_delay;
    this->retry_count = retry_count;

    #ifdef NRF24
        this->ce_pin = ce_pin;
        this->csn_pin = csn_pin;
        this->rx_pin = rx_pin;
        this->sck_pin = sck_pin;
        this->tx_pin = tx_pin;
    #endif // NRF24
}

// Begin
template <typename TData, uint8_t max_packets> void NRFDongle<TData, max_packets>::begin() {

    if (!this->enabled){
        this->enabled = true;
        this->radio.powerUp();
    }

    #ifdef NRF24
        SPI.setRX(this->rx_pin);
        SPI.setTX(this->tx_pin);
        SPI.setSCK(this->sck_pin);
        SPI.setCS(this->csn_pin);
        SPI.begin(true);

        this->radio.begin(&SPI, this->ce_pin, this->csn_pin);
    #endif // NRF24

    #ifdef NRF52
        this->radio.begin();
    #endif // NRF52

    this->channel = _PAIR_CHANNEL_;
    this->address = _PAIR_ADDRESS_;
    this->paired = false;

    // set the channel
    this->radio.setChannel(channel);

    // apply settings
    #ifdef NRF24
        this->radio.setDataRate((rf24_datarate_e)this->data_rate);
        this->radio.setPALevel((rf24_pa_dbm_e)this->power_level);
    #endif // NRF24
    #ifdef NRF52
        this->radio.setDataRate(this->data_rate);
        this->radio.setPALevel(this->power_level);
    #endif // NRF52
    this->radio.setRetries(this->retry_delay, this->retry_count);

    // set transmission size to the size of the pairing packet during pairing
    // later, we will set the payload size to the size of the data packet
    this->radio.setPayloadSize(sizeof(PairingPacket));

    // host opens writing pipe
    // dongle opens reading pipe
    #ifdef NRF_HOST
        this->radio.openWritingPipe(this->address);
    #endif // NRF_HOST
    #ifdef NRF_DONGLE
        this->radio.openReadingPipe(1, this->address);
    #endif // NRF_DONGLE

    // start listening on the dongle
    #ifdef NRF_HOST
        this->radio.stopListening();
    #endif // NRF_HOST
    #ifdef NRF_DONGLE
        this->radio.startListening();
    #endif // NRF_DONGLE

    // start timers
    this->ping_timer = 0;
    this->pair_timer = 0;
}

// Update
template <typename TData, uint8_t max_packets> void NRFDongle<TData, max_packets>::update() {

    if (!this->enabled){
        return;
    }

    // if we are the host, we can send packets
    #ifdef NRF_HOST
        // if we are not paired, try to pair
        if (!this->paired) {
            this->paired = this->try_pair();

            // if we are not paired, and the pair timeout has exceeded
            // then power down the radio
            // unless the timeout is disabled (pair_timeout_millis = 0)
            if (this->pair_timeout_millis > 0 && !this->paired && this->pair_timer > this->pair_timeout_millis) {
                this->end();
            }
        }

        // if we are paired, we can send packets and ping
        else {

            // if we have packets to send
            if (!this->buffer.isEmpty()) {
                // send the packet, and pop it
                bool received = this->send(this->buffer.pop(), true);

                // if the packet was not received, unpair and clear the buffer
                if (!received) {
                    this->unpair();
                    this->buffer.clear();
                } else {
                    // reset the ping timer as there's no need to ping
                    // if data was sent and received
                    this->ping_timer = 0;
                }

            }

            // try to ping
            // if data was just sent, then this will be skipped safely
            if (!this->ping()) {
                // if the ping was sent and not acknowledged, unpair
                this->unpair();
                this->buffer.clear();
            }
        }
    #endif // NRF_HOST

    // if we are the dongle, we can receive packets
    #ifdef NRF_DONGLE
        // if we are not paired, try to pair
        if (!this->paired) {
            this->paired = this->try_pair();

            // if we are not paired, and the pair timeout has exceeded
            // then power down the radio
            // unless the timeout is disabled (pair_timeout_millis = 0)
            if (this->pair_timeout_millis > 0 && !this->paired && this->pair_timer > this->pair_timeout_millis) {
                this->end();
            }
        } else if (this->ping_timer > 2 * this->ping_interval_millis) {
            // if we are paired, but the ping timer has exceeded
            // twice ping_interval_millis, unpair
            this->unpair();
        } else if (this->radio.available()) {
            // if we are paired, we can receive packets
            // if we have a packet

            // create a packet
            Packet<TData> packet;
            // read the packet
            this->radio.read(&packet, sizeof(Packet<TData>));

            // push the packet if it is not a ping packet
            if (!packet.ping) {
                this->buffer.push(packet.data);
            }

            // if we have successfully received a packet
            // reset the ping timer
            this->ping_timer = 0;
        }
    #endif // NRF_DONGLE
}

// End
template <typename TData, uint8_t max_packets> void NRFDongle<TData, max_packets>::end() {
    if (this->enabled){
        this->enabled = false;
        this->radio.powerDown();
    }
}

// Unpair
template <typename TData, uint8_t max_packets> bool NRFDongle<TData, max_packets>::unpair() {
    if (!this->enabled){
        return false;
    }

    if (!this->paired){
        return true;
    }

    this->paired = false;
    this->address = _PAIR_ADDRESS_;
    this->channel = _PAIR_CHANNEL_;

    // set the channel
    this->radio.setChannel(channel);

    // set the payload size to the size of the pairing packet during pairing
    // later, we will set the payload size to the size of the data packet
    this->radio.setPayloadSize(sizeof(PairingPacket));

    // host opens writing pipe
    // dongle opens reading pipe
    #ifdef NRF_HOST
        this->radio.openWritingPipe(this->address);
    #endif // NRF_HOST
    #ifdef NRF_DONGLE
        this->radio.openReadingPipe(1, this->address);
    #endif // NRF_DONGLE

    // start listening on the dongle
    #ifdef NRF_HOST
        this->radio.stopListening();
    #endif // NRF_HOST
    #ifdef NRF_DONGLE
        this->radio.startListening();
    #endif // NRF_DONGLE

    // reset timers
    this->ping_timer = 0;
    this->pair_timer = 0;

    return true;
}

// Is Paired
template <typename TData, uint8_t max_packets> bool NRFDongle<TData, max_packets>::is_paired() {
    return this->paired;
}

// Is Enabled
template <typename TData, uint8_t max_packets> bool NRFDongle<TData, max_packets>::is_enabled() {
    return this->enabled;
}

// Get Address
template <typename TData, uint8_t max_packets> uint64_t NRFDongle<TData, max_packets>::get_address() {
    return this->address;
}

// Get Unique ID
template <typename TData, uint8_t max_packets> uint64_t NRFDongle<TData, max_packets>::get_unique_id() {
    return this->unique_id;
}

// Get Program ID
template <typename TData, uint8_t max_packets> uint64_t NRFDongle<TData, max_packets>::get_program_id() {
    return this->program_id;
}

// Get Channel
template <typename TData, uint8_t max_packets> uint8_t NRFDongle<TData, max_packets>::get_channel() {
    return this->channel;
}

// Set Unique ID
template <typename TData, uint8_t max_packets> void NRFDongle<TData, max_packets>::set_unique_id(uint64_t unique_id) {
    this->unique_id = unique_id;
}

// Has Data
template <typename TData, uint8_t max_packets> bool NRFDongle<TData, max_packets>::has_data() {
    return !this->buffer.isEmpty();
}

// Get Radio
template <typename TData, uint8_t max_packets> Radio &NRFDongle<TData, max_packets>::get_radio() {
    return this->radio;
}

// Try Pair
template <typename TData, uint8_t max_packets> bool NRFDongle<TData, max_packets>::try_pair() {

    if (!this->enabled){
        return false;
    }

    if (this->paired){
        return true;
    }

    // if we are the host, we send our unique_id
    // if the message is market as received, we are paired
    #ifdef NRF_HOST

        // create a pairing packet
        PairingPacket pairing_packet;
        pairing_packet.unique_id = this->unique_id;
        pairing_packet.program_id = this->program_id;
        pairing_packet.ping_interval_millis = this->ping_interval_millis;

        bool report = this->radio.write(&pairing_packet, sizeof(PairingPacket));

        // if the message was received, we are paired
        // and we need to switch our channel, address,
        // and payload size
        if (report) {
            this->paired = true;
            this->address = this->unique_id;
            // the channel is the unique_id modulo 73 plus 1
            this->channel = this->unique_id % 73 + 1;
            this->pair_timer = 0;
            this->ping_timer = 0;

            this->radio.setChannel(this->channel);
            this->radio.openWritingPipe(this->address);
            this->radio.setPayloadSize(sizeof(Packet<TData>));
            this->radio.stopListening();
        }
        return report;
    #endif // NRF_HOST

    // if we are the dongle, we wait to receive the unique_id
    // if we receive a payload that is the right size, we are paired
    #ifdef NRF_DONGLE
        if (this->radio.available()) {
            uint8_t size = this->radio.getPayloadSize();
            // check if the size is the size of the pairing packet
            if (size == sizeof(PairingPacket)) {
                // read the packet
                PairingPacket pairing_packet;
                this->radio.read(&pairing_packet, sizeof(PairingPacket));

                // check if the program_id matches
                if (pairing_packet.program_id != this->program_id) {
                    return false;
                }

                uint64_t unique_id = pairing_packet.unique_id;
                uint16_t ping_interval_millis = pairing_packet.ping_interval_millis;
                this->address = unique_id;
                // the channel is the unique_id modulo 73 plus 1
                this->channel = unique_id % 73 + 1;
                this->radio.setChannel(this->channel);
                this->ping_interval_millis = ping_interval_millis;
                this->radio.openReadingPipe(1, this->address);
                this->radio.setPayloadSize(sizeof(Packet<TData>));
                this->radio.startListening();
                this->paired = true;
                this->pair_timer = 0;
                this->ping_timer = 0;
                return true;
            }
        }
        return false;
    #endif // NRF_DONGLE
}

// Send
#ifdef NRF_HOST
    template <typename TData, uint8_t max_packets> bool NRFDongle<TData, max_packets>::send(TData data, bool send_now) {
        if (!this->enabled){
            return false;
        }

        if (!this->paired){
            return false;
        }

        // if we are not sending now, push the packet
        if (!send_now) {
            this->buffer.push(data);
            return true;
        }

        // if we are sending now, send the packet
        Packet<TData> packet;
        packet.data = data;
        bool report = this->radio.write(&packet, sizeof(Packet<TData>));

        // if the packet was received, reset the ping timer
        if (report) {
            this->ping_timer = 0;
        }

        return report;
    }
#endif // NRF_HOST

// Ping
#ifdef NRF_HOST
    template <typename TData, uint8_t max_packets> bool NRFDongle<TData, max_packets>::ping() {
        // returns true if ping was sent and acknowledged
        // or if there is no need to ping
        // returns false if the ping was sent but not acknowledged
        // therefore signaling that the pair is broken

        if (!this->enabled){
            return true;
        }

        if (!this->paired){
            return true;
        }

        if (this->ping_timer > this->ping_interval_millis) {
            this->ping_timer = 0;

            // create a ping packet

            Packet<TData> ping_packet;
            ping_packet.ping = true;


            bool report = this->radio.write(&ping_packet, sizeof(Packet<TData>));

            return report;
        }

        return true;
    }
#endif // NRF_HOST

// Read
#ifdef NRF_DONGLE
    template <typename TData, uint8_t max_packets> bool NRFDongle<TData, max_packets>::read(TData &data, bool pop) {
        if (!this->enabled){
            return false;
        }

        if (!this->paired){
            return false;
        }

        if (this->buffer.isEmpty()){
            return false;
        }

        if (pop) {
            data = this->buffer.pop();
        } else {
            data = this->buffer.last();
        }

        return true;
    }
#endif // NRF_DONGLE

#endif // NRF_DONGLE_H
