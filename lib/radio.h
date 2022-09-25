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
    bool rcSwi1;     // bool ???
    bool rcSwi2;
    bool rcSwi3;
    float checksum;
} RX_payload_t;

typedef struct __attribute__((__packed__))
{
    float yaw;          // Fluglage via MPU9250
    float pitch;
    float roll;
    uint16_t altitude;      // Höhe via BMP280  
    uint16_t sonic;         // US Sensor
    float temperature;   // MPU9250
    float pressure;     // BMP280
} TX_payload_t;

typedef struct
{
    bool isconnect;
    RX_payload_t RX_payload; 
} rcInterface_t;

class Radio : public Task::Base
{
    bool radioNumber; // 0 uses pipe[0] to transmit, 1 uses pipe[1] to received
    bool role;        // true(>0) = TX role, false(0) = RX role
    const uint64_t pipe[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

protected:
    RF24 *_radio; 
    rcInterface_t *rcInterface; 
    
    TX_payload_t TX_payload;
 //   RX_payload_t *RX_payload; 

public:
    Radio(const String &name)
        : Task::Base(name)
    {}

    virtual ~Radio() {}

    Radio *setModel(rcInterface_t *_model)
    { 
        LOGGER_VERBOSE("Enter....");
        rcInterface = _model;
        LOGGER_VERBOSE("....leave");
        return this;
    }

    virtual void begin() override
    {
        LOGGER_VERBOSE("Enter....");
        pinMode(PIN_RADIO_LED, OUTPUT);
        digitalWrite(PIN_RADIO_LED, LOW);
        _radio = new RF24(PIN_RADIO_CE, PIN_RADIO_CSN); // Adresse in Variable speichern -> constructor

        if (!_radio->begin())
        {
            LOGGER_FATAL("radio hardware is not responding!!");
            while (1)
            {} // hold in infinite loop
        }

        role = true;
        radioNumber = 1; // 1 uses pipe[1] to recieve
        
         //_radio->setChannel(76);
        //_radio->setDataRate(RF24_250KBPS);
        _radio->setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
        _radio->enableDynamicPayloads();  // ACK payloads are dynamically sized
        _radio->enableAckPayload();
        //_radio->setPayloadSize(sizeof(rcInterface_t)); 
        _radio->openWritingPipe(pipe[radioNumber]);     // always uses pipe 0
        _radio->openReadingPipe(1, pipe[!radioNumber]); // using pipe 1
        _radio->stopListening(); 

        TX_payload.yaw = 129.9;
        TX_payload.pitch = -12.8;
        TX_payload.roll = 41.2;
        TX_payload.altitude = 111;
        TX_payload.sonic = 222;
        TX_payload.temperature = 24.5;
        TX_payload.pressure = 12345.0;
  
    delay(100);
    
        LOGGER_VERBOSE("...leave");
    } /*----------------------------- end of begin ------------------------------------*/

    virtual void update() override
    {
    LOGGER_VERBOSE("Enter....");  
    
        if (role) {// This device is a TX node
            unsigned long start_timer = micros(); 
            LOGGER_WARNING_FMT("sizeod RX_payload = %i" ,sizeof(TX_payload_t));                 
            bool report = _radio->write(&TX_payload, sizeof(TX_payload_t)); 
            LOGGER_WARNING_FMT("Report write = %i Number = %i Role = %i",report, radioNumber, role);
            unsigned long end_timer = micros();                    
            if (report) {
               LOGGER_WARNING_FMT("Transmission successful! time to transmit = %u",(end_timer - start_timer)); 
               
                LOGGER_WARNING_FMT("Sent Yaw = %f", (float)TX_payload.yaw);
                LOGGER_WARNING_FMT("Sent Pitch = %f" ,(float)TX_payload.pitch);
                LOGGER_WARNING_FMT("Sent Roll = %f" ,(float)TX_payload.roll);
                LOGGER_WARNING_FMT("Sent Altitude = %f" ,(float)TX_payload.altitude);
                LOGGER_WARNING_FMT("Sent Sonic = %i" ,(int)TX_payload.sonic);
                LOGGER_WARNING_FMT("Sent Temperatur =%f" ,(float)TX_payload.temperature);
                LOGGER_WARNING_FMT("Sent Pressure = %f" ,(float)TX_payload.pressure);

                uint8_t pipe;
                if (_radio->available(&pipe)) {  // is there an ACK payload? grab the pipe number that received it
                    //  PayloadStruct received;
                    _radio->read(&rcInterface->RX_payload, sizeof(rcInterface_t));  // get incoming ACK payload   
                    // hier muss 1 - 1 - 1 kommen
                    LOGGER_WARNING_FMT("Report read = %i Number = %i Role = %i",report, radioNumber, role);
                    LOGGER_WARNING_FMT("Recieved %i bytes on %i ",_radio->getDynamicPayloadSize(), pipe);
                    LOGGER_WARNING_FMT("Read Throttle = %i",(int)rcInterface->RX_payload.rcThrottle);    
                    LOGGER_WARNING_FMT("Read YAW = %f",(float)rcInterface->RX_payload.rcYaw);
                    LOGGER_WARNING_FMT("Read Pitch = %f",(float)rcInterface->RX_payload.rcPitch);
                    LOGGER_WARNING_FMT("Read Roll = %f",(float)rcInterface->RX_payload.rcRoll);
                    LOGGER_WARNING_FMT("Read SW1 = %i",(int)rcInterface->RX_payload.rcSwi1);
                    LOGGER_WARNING_FMT("Read SW2 = %i",(int)rcInterface->RX_payload.rcSwi2);
                    LOGGER_WARNING_FMT("Read SW3 = %i",(int)rcInterface->RX_payload.rcSwi3);
                    LOGGER_WARNING_FMT("Read Check = %f",(float)rcInterface->RX_payload.checksum);
                    
                    _radio->writeAckPayload(1, &TX_payload, sizeof(TX_payload_t));

                } else {
                    LOGGER_FATAL(" Recieved: an empty ACK packet");  // empty ACK packet received
                }
            } else {
                LOGGER_FATAL("Transmission failed or timed out");  // payload was not delivered
            }
            delay(1000);  // slow transmissions down by 1 second
        } 
 //       Serial2.println("role = false");
        LOGGER_VERBOSE("....leave");
    } // ------------------- end of update --------------------------------------------*/
}; /*----------------------------- end of radio.h class -------------------------------*/

//#undef _DEBUG_