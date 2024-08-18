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
    #error "Either NRF_HOST or NRF_DONGLE must be defined"
#endif

#if !defined(NRF24) && !defined(NRF52)
    #error "Either NRF24 or NRF52 must be defined"
#endif

#if defined(NRF_HOST) && defined(NRF_DONGLE)
    #error "Only one of NRF_HOST or NRF_DONGLE can be defined"
#endif

#if defined(NRF24) && defined(NRF52)
    #error "Only one of NRF24 or NRF52 can be defined"
#endif

#ifdef NRF24
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

// A packet is a struct with a const generic uint8_t size array of uint8_t's
template <uint8_t packet_size> struct Packet {
    bool ping;
    uint8_t data[packet_size];
};

// pairing address, CANNOT be 0x0000000000000000
const uint64_t _PAIR_ADDRESS_ = 0x0A0A0A0A0A0A0A0A;

// pairing packet, sends the unique_id and ping millis
typedef Packet<sizeof(uint64_t) + sizeof(uint32_t)> PairingPacket;

union PairingPacketUnion {
    PairingPacket packet;
    struct {
        uint64_t unique_id;
        uint16_t ping_interval_millis;
    };
};

template <uint8_t packet_size, uint8_t max_packets> class NRFDongle {
    public:
        // take in a reference to the radio, a unique identifier for this radio
        // the channel,
        // the ping interval in milliseconds,
        // pairing timeout in milliseconds,
        // and settings for the radio:
        //     data rate, power level
        NRFDongle(
                    Radio &radio,
                    uint64_t unique_id, // unused for dongle
                    uint8_t channel,
                    uint16_t ping_interval_millis, // unused for dongle
                    uint32_t pair_timeout_millis,
                    uint8_t data_rate,
                    uint8_t power_level
                );

        void begin();
        void update();
        void end();
        bool unpair();
        bool is_paired();
        bool is_enabled();
        uint64_t get_address();
        uint64_t get_unique_id();
        void set_unique_id(uint64_t unique_id);

        // method for whether or not there is data in the buffer
        bool has_data();

        Radio &get_radio();

        #ifdef NRF_HOST
            bool send(Packet<packet_size> packet, bool send_now = false);

            bool ping();
        #endif // NRF_HOST

        #ifdef NRF_DONGLE
            bool read(Packet<packet_size> &packet, bool pop = true);
        #endif // NRF_DONGLE

    private:
        Radio &radio;
        bool enabled = false;
        bool paired = false;
        uint64_t address;
        uint64_t unique_id;
        uint8_t channel;
        uint8_t data_rate;
        uint8_t power_level;
        CircularBuffer<Packet<packet_size>, max_packets> buffer;
        elapsedMillis ping_timer;
        elapsedMillis pair_timer;
        uint16_t ping_interval_millis;
        uint32_t pair_timeout_millis;

        bool try_pair();
        uint8_t get_ping_packet_size();
};

// Implementation

// Constructor
template <uint8_t packet_size, uint8_t max_packets> NRFDongle<packet_size, max_packets>::NRFDongle(Radio &radio, uint64_t unique_id, uint8_t channel, uint16_t ping_interval_millis, uint32_t pair_timeout_millis, uint8_t data_rate, uint8_t power_level) : radio(radio) {

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

    // set the channel
    this->channel = channel;

    // set the ping interval
    this->ping_interval_millis = ping_interval_millis;

    // set the pair timeout
    this->pair_timeout_millis = pair_timeout_millis;

    // save radio settings
    this->data_rate = data_rate;
    this->power_level = power_level;
}

// Begin
template <uint8_t packet_size, uint8_t max_packets> void NRFDongle<packet_size, max_packets>::begin() {

    if (!this->enabled){
        this->enabled = true;
        this->radio.powerUp();
    }

    this->radio.begin();

    this->address = _PAIR_ADDRESS_;
    this->paired = false;

    // set the channel
    this->radio.setChannel(channel);

    // apply settings
    this->radio.setDataRate(this->data_rate);
    this->radio.setPALevel(this->power_level);

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
template <uint8_t packet_size, uint8_t max_packets> void NRFDongle<packet_size, max_packets>::update() {

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
            if (!this->paired && this->pair_timer > this->pair_timeout_millis) {
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
            if (!this->paired && this->pair_timer > this->pair_timeout_millis) {
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
            Packet<packet_size> packet;
            // read the packet
            this->radio.read(&packet, sizeof(Packet<packet_size>));

            // push the packet if it is not a ping packet
            if (!packet.ping) {
                this->buffer.push(packet);
            }

            // if we have successfully received a packet
            // reset the ping timer
            this->ping_timer = 0;
        }
    #endif // NRF_DONGLE
}

// End
template <uint8_t packet_size, uint8_t max_packets> void NRFDongle<packet_size, max_packets>::end() {
    if (this->enabled){
        this->enabled = false;
        this->radio.powerDown();
    }
}

// Unpair
template <uint8_t packet_size, uint8_t max_packets> bool NRFDongle<packet_size, max_packets>::unpair() {
    if (!this->enabled){
        return false;
    }

    if (!this->paired){
        return true;
    }

    this->paired = false;
    this->address = _PAIR_ADDRESS_;

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
template <uint8_t packet_size, uint8_t max_packets> bool NRFDongle<packet_size, max_packets>::is_paired() {
    return this->paired;
}

// Is Enabled
template <uint8_t packet_size, uint8_t max_packets> bool NRFDongle<packet_size, max_packets>::is_enabled() {
    return this->enabled;
}

// Get Address
template <uint8_t packet_size, uint8_t max_packets> uint64_t NRFDongle<packet_size, max_packets>::get_address() {
    return this->address;
}

// Get Unique ID
template <uint8_t packet_size, uint8_t max_packets> uint64_t NRFDongle<packet_size, max_packets>::get_unique_id() {
    return this->unique_id;
}

// Set Unique ID
template <uint8_t packet_size, uint8_t max_packets> void NRFDongle<packet_size, max_packets>::set_unique_id(uint64_t unique_id) {
    this->unique_id = unique_id;
}

// Has Data
template <uint8_t packet_size, uint8_t max_packets> bool NRFDongle<packet_size, max_packets>::has_data() {
    return !this->buffer.isEmpty();
}

// Get Radio
template <uint8_t packet_size, uint8_t max_packets> Radio &NRFDongle<packet_size, max_packets>::get_radio() {
    return this->radio;
}

// Try Pair
template <uint8_t packet_size, uint8_t max_packets> bool NRFDongle<packet_size, max_packets>::try_pair() {

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
        PairingPacketUnion pairing_packet;
        pairing_packet.unique_id = this->unique_id;
        pairing_packet.ping_interval_millis = this->ping_interval_millis;

        bool report = this->radio.write(&pairing_packet.packet, sizeof(PairingPacket));

        // if the message was received, we are paired
        // and we need to switch our address and payload size
        // and change our writing pipe and payload size
        if (report) {
            this->paired = true;
            this->address = this->unique_id;
            this->pair_timer = 0;
            this->ping_timer = 0;
            this->radio.openWritingPipe(this->address);
            this->radio.setPayloadSize(sizeof(Packet<packet_size>));
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
                PairingPacketUnion pairing_packet;
                this->radio.read(&pairing_packet.packet, sizeof(PairingPacket));
                uint64_t unique_id = pairing_packet.unique_id;
                uint16_t ping_interval_millis = pairing_packet.ping_interval_millis;
                this->address = unique_id;
                this->ping_interval_millis = ping_interval_millis;
                this->radio.openReadingPipe(1, this->address);
                this->radio.setPayloadSize(sizeof(Packet<packet_size>));
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
    template <uint8_t packet_size, uint8_t max_packets> bool NRFDongle<packet_size, max_packets>::send(Packet<packet_size> packet, bool send_now) {
        if (!this->enabled){
            return false;
        }

        if (!this->paired){
            return false;
        }

        // if we are not sending now, push the packet
        if (!send_now) {
            this->buffer.push(packet);
            return true;
        }

        // if we are sending now, send the packet
        bool report = this->radio.write(&packet, sizeof(Packet<packet_size>));

        // if the packet was received, reset the ping timer
        if (report) {
            this->ping_timer = 0;
        }

        return report;
    }
#endif // NRF_HOST

// Ping
#ifdef NRF_HOST
    template <uint8_t packet_size, uint8_t max_packets> bool NRFDongle<packet_size, max_packets>::ping() {
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

            Packet<packet_size> ping_packet;
            ping_packet.ping = true;


            bool report = this->radio.write(&ping_packet, sizeof(Packet<packet_size>));

            return report;
        }

        return true;
    }
#endif // NRF_HOST

// Read
#ifdef NRF_DONGLE
    template <uint8_t packet_size, uint8_t max_packets> bool NRFDongle<packet_size, max_packets>::read(Packet<packet_size> &packet, bool pop) {
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
            packet = this->buffer.pop();
        } else {
            packet = this->buffer.last();
        }

        return true;
    }
#endif // NRF_DONGLE

#endif // NRF_DONGLE_H
