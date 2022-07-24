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
#include <RF24.h>
#include "def.h"
#include "myLogger.h"
#include <printf.h>

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

uint8_t address[][6] = {"1Node", "2Node"};

class Radio : public Task::Base
{
    // uint8_t address[][6];
    bool radioNumber; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
    bool role;        // true = TX role, false = RX role
 //   float payload;

public:    
    interface_t *interface;
    
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
        LOGGER_VERBOSE("Enter....");
        pinMode(PIN_RADIO_LED, OUTPUT);
        digitalWrite(PIN_RADIO_LED, LOW);
        // address[][6] = {"1Node", "2Node"};
        radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
        role = false;    // true = TX role, false = RX role
        //payload = 0.0;
        _radio = new RF24(PIN_RADIO_CE, PIN_RADIO_CSN); // Adresse in Variable speichern -> constuktor

        // initialize the transceiver on the SPI bus
        if (!_radio->begin())
        {
            LOGGER_FATAL("radio hardware is not responding!!");
            while (1)
            {
            } // hold in infinite loop
        }
         LOGGER_WARNING_FMT("RF24 is initialized - RadioNumber = %.i", radioNumber);
        //role variable is hardcoded to RX behavior, inform the user of this
        //LOGGER_NOTICE("*** PRESS 'T' to begin transmitting to the other node");
        _radio->setPALevel(RF24_PA_LOW); // RF24_PA_MAX is default.
        _radio->setPayloadSize(sizeof(payload_t)); // float datatype occupies 4 bytes
        _radio->openWritingPipe(address[radioNumber]); // always uses pipe 0
        _radio->openReadingPipe(1, address[!radioNumber]); // using pipe 1

        if (role)
        {
            _radio->stopListening(); // put radio in TX mode
        }
        else
        {
            _radio->startListening(); // put radio in RX mode
        }

        // For debugging info
        printf_begin(); // needed only once for printing details
        //_radio->printDetails();       // (smaller) function that prints raw register values
        _radio->printPrettyDetails(); // (larger) function that prints human readable data

        LOGGER_VERBOSE("...leave");
    } /*----------------------------- end of begin ------------------------------------*/

    virtual void update() override
    {
        LOGGER_VERBOSE("Enter....");

        if (role)
        {
            // This device is a TX node
            unsigned long start_timer = micros();                 // start the timer
            bool report = _radio->write(&interface->payload, sizeof(payload_t)); // transmit & save the report
            unsigned long end_timer = micros();                   // end the timer

            if (report)
            {
                LOGGER_NOTICE_FMT("Transmission successful! - %i payload %f  ", end_timer - start_timer, payload);                           // payload was delivered
              //   payload += 0.01;                                                      // increment float payload
            }
            else
            {
                LOGGER_FATAL("Transmission failed or timed out"); // payload was not delivered
            }
        }
        else
        {
            // This device is a RX node
            uint8_t pipe;
            if (_radio->available(&pipe))
            { // is there a payload? get the pipe number that recieved it
                digitalWrite(PIN_RADIO_LED, LOW);
                uint8_t bytes = _radio->getPayloadSize(); // get the size of the payload
                _radio->read(&interface->payload, sizeof(payload_t));            // fetch payload from FIFO
                LOGGER_NOTICE_FMT("Throttle = %d Pitch = %d Roll = %d YAW = %d", payload.rcThrottle, payload.rcPitch, payload.rcYaw);
                LOGGER_NOTICE_FMT("Received %d bytes of pipe %d Interface %f", bytes, pipe, interface);
                digitalWrite(PIN_RADIO_LED, HIGH);
            }
        } // end of (if role)

        // if (Serial2.available())                                // ist wohl in diesem speziellen Fall nicht nötig
        // {
        //     // change the role via the serial monitor
        //     char c = toupper(Serial2.read());
        //     if (c == 'T' && !role)
        //     {
        //         // Become the TX node
        //         role = true;
        //         LOGGER_NOTICE("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK");
        //         _radio->stopListening();
        //     }
        //     else if (c == 'R' && role)
        //     {
        //         // Become the RX node
        //         role = false;
        //         LOGGER_NOTICE("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK");
        //         _radio->startListening();
        //     }
        // }
        LOGGER_VERBOSE("....leave");
    } // ------------------- end of update --------------------------------------------*/
}; /*----------------------------- end of radio.h class -------------------------------*/

#undef _DEBUG_