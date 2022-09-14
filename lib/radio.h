#pragma once
/*  File name : radio.h
    Project: Phantom 1
    Autor: Wilhelm Kuckelsberg
    Date: 2022-05-31 (2021.05.24)
    Description: Kommunikation zwischen Drohne und RC

    https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-python-sdk.pdf
    https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf
    https://www.youtube.com/watch?v=V4ziwen24Ps

    // arduino::MbedSPI SPI(16,19,18);
//  Singleton instance of the radio driver
//  RH_NRF24 nrf24(PIN_RADIO_CE, PIN_RADIO_CSN);   // CE, CSN
//  RH_NRF24 nrf24(8, 7); // use this to be electrically compatible with Mirf
//  RH_NRF24 nrf24(8, 10);// For Leonardo, need explicit SS pin
//  RH_NRF24 nrf24(8, 7); // For RFM73 on Anarduino Mini

    SPI SLK     18
        MOSI    19
        MISO    16
*/

#include <Arduino.h>
#include <TaskManager.h>
#include <printf.h>   //funktioniert hier nicht
#include <RF24.h>
#include "myLogger.h"

#define PIN_RADIO_CE 20
#define PIN_RADIO_CSN 17
#define PIN_RADIO_LED 1

typedef struct __attribute__((__packed__))
{
    uint16_t rcThrottle; //!< Get the positions of the rc joysticks
    float rcYaw;
    float rcPitch;
    float rcRoll;
    uint8_t rcSwi1;     // bool ???
    uint8_t rcSwi2;
    uint8_t rcSwi3;
    float checksum;
} payload_t;

typedef struct
{
    bool isconnect;
    payload_t payload;
} interface_t;

class Radio : public Task::Base
{
    bool radioNumber; // 0 uses pipe[0] to transmit, 1 uses pipe[1] to received
    bool role;        // true(>0) = TX role, false(0) = RX role
    const uint64_t pipe[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

protected:
    RF24 *_radio; 
    interface_t *interface; 

public:
    Radio(const String &name)
        : Task::Base(name)
    {}

    virtual ~Radio() {}

    Radio *setModel(interface_t *_model)
    { 
        LOGGER_VERBOSE("Enter....");
        interface = _model;
        LOGGER_VERBOSE("....leave");
        return this;
    }

    virtual void begin() override
    {
        LOGGER_VERBOSE("Enter....");
        pinMode(PIN_RADIO_LED, OUTPUT);
        digitalWrite(PIN_RADIO_LED, LOW);

        radioNumber = 0; // 0 uses pipe[0] to transmit, 1 uses pipe[1] to recieve
        role = true;     // true(>0) = TX role, false(0) = RX role
        
        _radio = new RF24(PIN_RADIO_CE, PIN_RADIO_CSN); // Adresse in Variable speichern -> constuktor

        if (!_radio->begin())
        {
            LOGGER_FATAL("radio hardware is not responding!!");
            while (1)
            {} // hold in infinite loop
        }

        Serial.println(F("RF24/examples/GettingStarted"));
        Serial.println(F("Which radio is this? Enter '0' or '1'. Defaults to '0'"));
        while (!Serial.available()) {
            // wait for user input
        }
        char input = Serial.parseInt();
        radioNumber = input == 1;
        Serial.print(F("radioNumber = "));
        Serial.println((int)radioNumber);
        Serial.println(F("*** PRESS 'T' to begin transmitting to the other node new"));
        //_radio->setChannel(76);
        //_radio->setDataRate(RF24_250KBPS);
        _radio->setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
        _radio->setPayloadSize(sizeof(interface_t)); // float datatype occupies 4 bytes
        _radio->openWritingPipe(pipe[radioNumber]);     // always uses pipe 0
        _radio->openReadingPipe(1, pipe[!radioNumber]); // using pipe 1
  
        if (role) {
            _radio->stopListening();  // put radio in TX mode
        } else {
            _radio->startListening(); // put radio in RX mode
        }

        // uint8_t temp = sizeof(interface_t);
        // Serial.print(F("sizeof ")); Serial.println(temp); 

        // For debugging info
        // printf_begin();             // needed only once for printing details
        // _radio->printDetails();       // (smaller) function that prints raw register values
        // _radio->printPrettyDetails(); // (larger) function that prints human readable data
    delay(5000);
    
        LOGGER_VERBOSE("...leave");
    } /*----------------------------- end of begin ------------------------------------*/

    virtual void update() override
    {
    LOGGER_VERBOSE("Enter....");  
        if (role) {         // This device is the transmitter
           
            unsigned long start_timer = micros();                    // start the timer
            bool report = _radio->write(&interface->payload, sizeof(interface_t));      // transmit & save the report
            unsigned long end_timer = micros();                      // end the timer

            if (report) {
                Serial.print(F("Transmission successful! "));          
                Serial.print(F("Time to transmit = "));
                Serial.print(end_timer - start_timer);                 
                Serial.print(F(" us. Sent: "));
                Serial.println(interface->payload.rcYaw);                               

            } else {
                Serial.println(F("Transmission failed or timed out")); 
            }
        } else {    // This device is the receiver
            
            uint8_t pipe;                                   //???
            if (_radio->available(&pipe)) {             
                uint8_t bytes = _radio->getPayloadSize(); 
                _radio->read(&interface->payload, bytes);    
                
                Serial.print(F("Received "));
                Serial.print(bytes);                 
                Serial.print(F(" bytes on pipe "));
                Serial.print(pipe);                   
                Serial.print(F(": "));
                Serial.println(interface->payload.rcYaw);  
                }
        } // end of role

        if (Serial.available()) {       // change the role via the serial monitor
            
            char c = toupper(Serial.read());
            if (c == 'T' && !role) {
                // Become the TX node
                role = true;
                Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
                _radio->stopListening();

            } else if (c == 'R' && role) {
                // Become the RX node
                role = false;
                Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));
                _radio->startListening();
            }
        }
        LOGGER_VERBOSE("....leave");
    } // ------------------- end of update --------------------------------------------*/
}; /*----------------------------- end of radio.h class -------------------------------*/

#undef _DEBUG_