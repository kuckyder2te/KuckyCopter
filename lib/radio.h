#pragma once
/*  File name : radio.h
    Project: Phantom 1
    Autor: Wilhelm Kuckelsberg
    Date: 2022-05-31 (2021.05.24)
    Description: Kommunikation zwischen Drohne und RC

    https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-python-sdk.pdf
    https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf
    https://www.youtube.com/watch?v=V4ziwen24Ps

    SPI SLK     18
        MOSI    19
        MISO    16
*/

#include <Arduino.h>
#include <TaskManager.h>
// #define SERIAL_DEBUG
// #include <printf.h>
// #include "nRF24L01.h"
#include <RF24.h>
#include "myLogger.h"



typedef struct __attribute__((__packed__))
{
    uint16_t rcThrottle; //!< Get the positions of the rc joysticks
    double rcYaw;
    double rcPitch;
    double rcRoll;
    uint8_t rcSwi1;
    uint8_t rcSwi2;
    uint8_t rcSwi3;
    double checksum;
} payload_t;

typedef struct
{
    bool isconnect;
    payload_t payload;
} interface_t;

// arduino::MbedSPI SPI(16,19,18);
//  Singleton instance of the radio driver
//  RH_NRF24 nrf24(PIN_RADIO_CE, PIN_RADIO_CSN);   // CE, CSN
//  RH_NRF24 nrf24(8, 7); // use this to be electrically compatible with Mirf
//  RH_NRF24 nrf24(8, 10);// For Leonardo, need explicit SS pin
//  RH_NRF24 nrf24(8, 7); // For RFM73 on Anarduino Mini

#define PIN_RADIO_CE 20
#define PIN_RADIO_CSN 17
#define PIN_RADIO_LED 1

//uint8_t pipe[][6] = {"1Node", "2Node"};T
float payload = 0.0;

class Radio : public Task::Base
{
    bool radioNumber; // 0 uses pipe[0] to transmit, 1 uses pipe[1] to received
    bool role;        // true(>0) = TX role, false(0) = RX role
 //   float payload;

public:    
    interface_t *interface;
 //   payload_t payload;
    
protected:
    RF24 *_radio; // CE, CSN        // Pointer auf die Adresse setzen
    
public:
    Radio(const String &name)
        : Task::Base(name)
    {
    }

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
        delay(5000);
        LOGGER_VERBOSE("Enter....");
        pinMode(PIN_RADIO_LED, OUTPUT);
        digitalWrite(PIN_RADIO_LED, LOW);

    //    uint8_t address[][6] = {"1Node", "2Node"};
        const uint64_t address[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

        radioNumber = 0; // 0 uses pipe[0] to transmit, 1 uses pipe[1] to recieve
        role = true;    // true(>0) = TX role, false(0) = RX role
        
        _radio = new RF24(PIN_RADIO_CE, PIN_RADIO_CSN); // Adresse in Variable speichern -> constuktor

        // initialize the transceiver on the SPI bus
        if (!_radio->begin())
        {
            LOGGER_FATAL("radio hardware is not responding!!");
            while (1)
            {
            } // hold in infinite loop
        }
            // print example's introductory prompt
        Serial.println(F("RF24/examples/GettingStarted"));

        // To set the radioNumber via the Serial monitor on startup
        Serial.println(F("Which radio is this? Enter '0' or '1'. Defaults to '0'"));
        while (!Serial.available()) {
            // wait for user input
        }
        char input = Serial.parseInt();
        radioNumber = input == 1;
        Serial.print(F("radioNumber = "));
        Serial.println((int)radioNumber);

        // role variable is hardcoded to RX behavior, inform the user of this
        Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));
        //_radio->setChannel(76);
        //_radio->setDataRate(RF24_250KBPS);
        // Set the PA Level low to try preventing power supply related problems
        // because these examples are likely run with nodes in close proximity to
        // each other.
        _radio->setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

        // save on transmission time by setting the radio to only transmit the
        // number of bytes we need to transmit a float
        _radio->setPayloadSize(sizeof(float)); // float datatype occupies 4 bytes

        // set the TX address of the RX node into the TX pipe
        _radio->openWritingPipe(address[radioNumber]);     // always uses pipe 0

        // set the RX address of the TX node into a RX pipe
        _radio->openReadingPipe(1, address[!radioNumber]); // using pipe 1

        // additional setup specific to the node's role
        if (role) {
            _radio->stopListening();  // put radio in TX mode
        } else {
            _radio->startListening(); // put radio in RX mode
        }

        // For debugging info
        //  printf_begin();             // needed only once for printing details
        //  _radio->printDetails();       // (smaller) function that prints raw register values
        //  _radio->printPrettyDetails(); // (larger) function that prints human readable data



        LOGGER_VERBOSE("...leave");
    } /*----------------------------- end of begin ------------------------------------*/

    virtual void update() override
    {
    LOGGER_VERBOSE("Enter....");  
        if (role) {
            // This device is a TX node

            unsigned long start_timer = micros();                    // start the timer
            bool report = _radio->write(&payload, sizeof(float));      // transmit & save the report
            unsigned long end_timer = micros();                      // end the timer

            if (report) {
                Serial.print(F("Transmission successful! "));          // payload was delivered
                Serial.print(F("Time to transmit = "));
                Serial.print(end_timer - start_timer);                 // print the timer result
                Serial.print(F(" us. Sent: "));
                Serial.println(payload);                               // print payload sent
                payload += 0.01;                                       // increment float payload
            } else {
                Serial.println(F("Transmission failed or timed out")); // payload was not delivered
            }
        } else {
            // This device is a RX node
            uint8_t pipe;
            if (_radio->available(&pipe)) {             // is there a payload? get the pipe number that recieved it
                uint8_t bytes = _radio->getPayloadSize(); // get the size of the payload
                _radio->read(&payload, bytes);            // fetch payload from FIFO

        //        Serial.print("YAW = ");Serial.println(payload);
                
                Serial.print(F("Received "));
                Serial.print(bytes);                    // print the size of the payload
                Serial.print(F(" bytes on pipe "));
                Serial.print(pipe);                     // print the pipe number
                Serial.print(F(": "));
                Serial.println(payload);                // print the payload's value
                }
        } // role

        if (Serial.available()) {
            // change the role via the serial monitor

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