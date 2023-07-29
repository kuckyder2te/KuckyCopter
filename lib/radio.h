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

#define LOCAL_DEBUG
#include "myLogger.h"

#define PIN_RADIO_CE  20
#define PIN_RADIO_CSN 17
#define PIN_RADIO_LED  1

#define ACK_PACKAGE_MAX_COUNT 10

typedef struct __attribute__((__packed__))
{
    float checksum;
    uint16_t rcYaw;
    uint8_t rcPitch;
    uint8_t rcRoll;
    int16_t rcThrottle;                 //!< Get the positions of the rc joysticks
    uint16_t rcAltitudeSonicAdj;        // Wert wird über RC Poti eingestellt 0 - 200cm
    uint16_t rcAltitudeBaroAdj;         // Wert wird über RC Poti eingestellt 0 - 10m
    bool rcSwi1;                        // Schaltet in den Programmier-Modus
    bool rcSwi2;                        // autonomes fliegen    
    bool rcSwi3;                        //
} RX_payload_t;

typedef struct __attribute__((__packed__))
{
    float yaw;              // Fluglage via MPU9250
    float pitch;
    float roll;
    uint16_t altitude;      // Höhe via MS5611  
    float temperature;      // MS5611
    float pressure;         
    uint16_t distance_down; // US Sensor
    uint16_t distance_front;
} TX_payload_t;

typedef struct
{
  TX_payload_t  TX_payload;  //Do not change position !!!!! Must be the first entry
  RX_payload_t RX_payload; 
  bool isconnect;
} RC_interface_t;


class Radio : public Task::Base
{
    const uint64_t pipe_TX = 0xF0F0F0F0E1LL;
    const uint64_t pipe_RX = 0xF0F0F0F0D2LL;
    RX_payload_t _RX_payload;
    unsigned long _lastReceivedPacket;
    uint8_t _lostAckPackageCount;

protected:
    RF24 *_radio; 
    RC_interface_t *RC_interface;   

public:
    Radio(const String &name)
        : Task::Base(name){
        _lostAckPackageCount = 0;
        }

    virtual ~Radio() {}

    Radio *setModel(RC_interface_t *_model)
    { 
        LOGGER_VERBOSE("Enter....");
        RC_interface = _model;
        LOGGER_VERBOSE("....leave");
        return this;
    } /*----------------------------- end of setModel ------------------------------------*/

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
            {} 
        }
        
        _radio->setPALevel(RF24_PA_HIGH);  // RF24_PA_MAX is default.
        _radio->enableDynamicPayloads();  // ACK payloads are dynamically sized
        _radio->enableAckPayload();
        _radio->openWritingPipe(pipe_TX);     // always uses pipe 0
        _radio->openReadingPipe(1, pipe_RX); // using pipe 1
        _radio->startListening(); 
  
    delay(100);
    
        LOGGER_VERBOSE("...leave");
    } /*----------------------------- end of begin ------------------------------------*/

    virtual void update() override
    {
        if (_radio->available()) {                     // is there a payload? get the pipe number that recieved it
             uint8_t bytes = _radio->getDynamicPayloadSize();  // get the size of the payload
            Serial.println("rc available");
            _radio->read(&RC_interface->RX_payload, sizeof(RX_payload_t));  // get incoming payload
            _radio->writeAckPayload(1, &RC_interface->TX_payload, sizeof(TX_payload_t));
            #ifdef _SERIAL_STUDIO
                LOGGER_NOTICE_FMT("Reseived %i bytes", bytes);
                LOGGER_NOTICE_FMT("RX_Payload Throttle =  %i ", RC_interface->RX_payload.rcThrottle);
                LOGGER_NOTICE_FMT("RX_Payload Yaw =  %i ", RC_interface->RX_payload.rcYaw);
                LOGGER_NOTICE_FMT("RX_Payload Pitch =  %i ", RC_interface->RX_payload.rcPitch);
                LOGGER_NOTICE_FMT("RX_Payload Roll =  %i ", RC_interface->RX_payload.rcRoll);
            #endif
            _lostAckPackageCount = 0;
            RC_interface->isconnect = true;
            _radio->startListening();
        }else{
            _lostAckPackageCount++;
            if(_lostAckPackageCount>ACK_PACKAGE_MAX_COUNT){
                _lostAckPackageCount = ACK_PACKAGE_MAX_COUNT;   // protect against variable overflow (uint8_t)
                RC_interface->isconnect = false;
            }
        }
        LOGGER_VERBOSE("....leave");
    } // ------------------- end of update --------------------------------------------*/
}; /*----------------------------- end of radio->.h class -------------------------------*/