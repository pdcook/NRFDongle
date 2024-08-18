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

// A packet is a struct with a const generic uint8_t size array of uint8_t's
template <uint8_t packet_size> struct Packet {
  uint8_t data[size];
};

// pairing address
const uint64_t _PAIR_ADDRESS_ = 0x000000000000;

template <uint8_t packet_size, uint8_t max_packets> class NRFDongle {
    public:
        // take in a reference to the radio, a unique identifier for this radio
        // the channel,
        // and settings for the radio:
        //     data rate, power level
        NRFDongle(
                    Radio &radio,
                    uint64_t unique_id,
                    uint8_t channel,
                    uint8_t data_rate,
                    uint8_t power_level
                );

        void begin();
        void update();
        void end();
        bool unpair();
        bool is_paired();
        uint64_t get_address();

        Radio &get_radio();

        #ifdef NRF_HOST
            bool send(Packet<packet_size> packet, bool send_now = false);
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

        bool try_pair();
};

// Implementation

// Constructor
template <uint8_t packet_size, uint8_t max_packets> NRFDongle<packet_size, max_packets>::NRFDongle(Radio &radio, uint64_t unique_id, uint8_t channel, uint8_t data_rate, uint8_t power_level) : radio(radio) {

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

    // set transmission size to the size of the address during pairing
    // later, we will set the payload size to the size of the packet
    this->radio.setPayloadSize(sizeof(uint64_t));

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
        }

        // if we are paired, we can send packets
        else {
            // if we have packets to send
            if (!this->buffer.isEmpty()) {
                // send the packet, and pop it
                bool received = this->send(this->buffer.pop(), true);

                // if the packet was not received, unpair and clear the buffer
                if (!received) {
                    this->unpair();
                    this->buffer.clear();
                }
            }
        }
    #endif // NRF_HOST

    // if we are the dongle, we can receive packets
    #ifdef NRF_DONGLE
        // if we are not paired, try to pair
        if (!this->paired) {
            this->paired = this->try_pair();
        }

        // if we are paired, we can receive packets
        else {
            // if we have a packet
            if (this->radio.available()) {
                // create a packet
                Packet<packet_size> packet;
                // read the packet
                this->radio.read(&packet, sizeof(Packet<packet_size>));
                // push the packet
                this->buffer.push(packet);
            }
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

    // set the payload size to the size of the address during pairing
    // later, we will set the payload size to the size of the packet
    this->radio.setPayloadSize(sizeof(uint64_t));

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

    return true;
}

// Is Paired
template <uint8_t packet_size, uint8_t max_packets> bool NRFDongle<packet_size, max_packets>::is_paired() {
    return this->paired;
}

// Get Address
template <uint8_t packet_size, uint8_t max_packets> uint64_t NRFDongle<packet_size, max_packets>::get_address() {
    return this->address;
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
        bool report = this->radio.write(&this->unique_id, sizeof(uint64_t));

        // if the message was received, we are paired
        // and we need to switch our address and payload size
        // and change our writing pipe and payload size
        if (report) {
            this->paired = true;
            this->address = this->unique_id;
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
            if (size == sizeof(uint64_t)) {
                uint64_t unique_id;
                this->radio.read(&unique_id, sizeof(uint64_t));
                this->address = unique_id;
                this->radio.openReadingPipe(1, this->address);
                this->radio.setPayloadSize(sizeof(Packet<packet_size>));
                this->paired = true;
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
        return report;
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
