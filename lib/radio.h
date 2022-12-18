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

//#define LOCAL_DEBUG
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
} RC_interface_t;




class Radio : public Task::Base
{
    bool radioNumber; // 0 uses pipe[0] to transmit, 1 uses pipe[1] to received
    bool role, _role,__role,___role;        // true(>0) = TX role, false(0) = RX role
    const uint64_t pipe[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };
    RX_payload_t _RX_payload;
    unsigned long _lastReceivedPacket;
protected:
    RF24 *_radio; 
    RC_interface_t *RC_interface;   
    TX_payload_t TX_payload, _TX_payload;


public:
    Radio(const String &name)
        : Task::Base(name)
    {}

    virtual ~Radio() {}

    Radio *setModel(RC_interface_t *_model)
    { 
        LOGGER_VERBOSE("Enter....");
        RC_interface = _model;
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
            {} 
        }

        role = false;        // Parameter als Empfänger
        _role = !role;
        __role = _role;
        ___role = _role;
        radioNumber = 1; 
        
        _radio->setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
        _radio->enableDynamicPayloads();  // ACK payloads are dynamically sized
        _radio->enableAckPayload();
        _radio->openWritingPipe(pipe[radioNumber]);     // always uses pipe 0
        _radio->openReadingPipe(1, pipe[!radioNumber]); // using pipe 1
        _radio->startListening(); 

        TX_payload.yaw = 129.9;
        TX_payload.pitch = -12.8;
        TX_payload.roll = 41.2;
        TX_payload.altitude = 111;
        //TX_payload.sonic = 222;
        TX_payload.temperature = 24.5;
        TX_payload.pressure = 12345.0;
  
    delay(100);
    
        LOGGER_VERBOSE("...leave");
    } /*----------------------------- end of begin ------------------------------------*/

    virtual void update() override
    {
    LOGGER_VERBOSE("Enter....");  

        if (!role) {// This device is a TX node
            unsigned long start_timer = micros(); 

            LOGGER_NOTICE_FMT_CHK(role, _role,"sizeof TX_payload = %i" ,sizeof(TX_payload_t));  
            bool report = true;//_radio->write(&TX_payload, sizeof(TX_payload_t)); 
            LOGGER_NOTICE_FMT_CHK(role, __role,"Report write = %i Number = %i Role = %i",report, radioNumber, role);
            unsigned long end_timer = micros();                    
            if (report) {
                //LOGGER_NOTICE_FMT("Transmission successful! time to transmit = %u",(end_timer - start_timer)); 
                if(!_radio->writeAckPayload(1,&TX_payload,sizeof(TX_payload_t))){
                    //LOGGER_NOTICE("ACK Pipe full");
                }else{
                    TX_payload.sonic++;
                    //LOGGER_NOTICE("ACK Pipe empty");
                }
                LOGGER_NOTICE_FMT_CHK(TX_payload.yaw,_TX_payload.yaw,"Sent Yaw = %f", (float)TX_payload.yaw);
                LOGGER_NOTICE_FMT_CHK(TX_payload.pitch,_TX_payload.pitch,"Sent Pitch = %f" ,(float)TX_payload.pitch);
                LOGGER_NOTICE_FMT_CHK(TX_payload.roll,_TX_payload.roll,"Sent Roll = %f" ,(float)TX_payload.roll);
                LOGGER_NOTICE_FMT_CHK(TX_payload.altitude,_TX_payload.altitude,"Sent Altitude = %f" ,(float)TX_payload.altitude);
                LOGGER_NOTICE_FMT_CHK(TX_payload.sonic,_TX_payload.sonic,"Sent Sonic = %i" ,(int)TX_payload.sonic);
                LOGGER_NOTICE_FMT_CHK(TX_payload.temperature,_TX_payload.temperature,"Sent Temperatur =%f" ,(float)TX_payload.temperature);
                LOGGER_NOTICE_FMT_CHK(TX_payload.pressure,_TX_payload.pressure,"Sent Pressure = %f" ,(float)TX_payload.pressure);

                uint8_t pipe;
                if (_radio->available(&pipe)) {  // is there an ACK payload? grab the pipe number that received it
                    _lastReceivedPacket = millis();
                    _radio->read(&RC_interface->RX_payload, sizeof(RC_interface_t));  // get incoming ACK payload   
                    // hier muss 1 - 1 - 1 kommen
                    LOGGER_NOTICE_FMT_CHK(role,___role,"Report read = %i Number = %i Role = %i",report, radioNumber, role);
                    //LOGGER_NOTICE_FMT_CHK("Recieved %i bytes on %i ",_radio->getDynamicPayloadSize(), pipe);
                    LOGGER_NOTICE_FMT_CHK(RC_interface->RX_payload.rcThrottle,_RX_payload.rcThrottle,"Read Throttle = %i",(int)RC_interface->RX_payload.rcThrottle);    
                    LOGGER_NOTICE_FMT_CHK(RC_interface->RX_payload.rcYaw,_RX_payload.rcYaw,"Read YAW = %f",(float)RC_interface->RX_payload.rcYaw);
                    LOGGER_NOTICE_FMT_CHK(RC_interface->RX_payload.rcPitch,_RX_payload.rcPitch,"Read Pitch = %f",(float)RC_interface->RX_payload.rcPitch);
                    LOGGER_NOTICE_FMT_CHK(RC_interface->RX_payload.rcRoll,_RX_payload.rcRoll,"Read Roll = %f",(float)RC_interface->RX_payload.rcRoll);
                    LOGGER_NOTICE_FMT_CHK(RC_interface->RX_payload.rcSwi1,_RX_payload.rcSwi1,"Read SW1 = %i",(int)RC_interface->RX_payload.rcSwi1);
                    LOGGER_NOTICE_FMT_CHK(RC_interface->RX_payload.rcSwi2,_RX_payload.rcSwi2,"Read SW2 = %i",(int)RC_interface->RX_payload.rcSwi2);
                    LOGGER_NOTICE_FMT_CHK(RC_interface->RX_payload.rcSwi3,_RX_payload.rcSwi3,"Read SW3 = %i",(int)RC_interface->RX_payload.rcSwi3);
                    LOGGER_NOTICE_FMT_CHK(RC_interface->RX_payload.checksum,_RX_payload.checksum,"Read Check = %f",(float)RC_interface->RX_payload.checksum);
                    
                    _radio->writeAckPayload(1, &TX_payload, sizeof(TX_payload_t));
                } else {
                    if(millis()-_lastReceivedPacket>10000){
                        static unsigned long _lastmillis = millis();
                        if(millis()-_lastmillis>5000){
                            LOGGER_NOTICE(" Recieved: an empty ACK packet");  // empty ACK packet received
                            _lastmillis = millis();
                        }
                        RC_interface->isconnect = false;
                    }else{
                        RC_interface->isconnect = true;
                    }
                }
            } else {
                LOGGER_FATAL("Transmission failed or timed out");  // payload was not delivered
            }
//            delay(1000);  
        } 
        LOGGER_VERBOSE("....leave");
    } // ------------------- end of update --------------------------------------------*/
}; /*----------------------------- end of radio.h class -------------------------------*/

//#undef _DEBUG_