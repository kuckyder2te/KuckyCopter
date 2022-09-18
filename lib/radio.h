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
} payload_t;

typedef struct __attribute__((__packed__))
{
    uint16_t throttle; // Rotor Geschwindigkeit
    float yaw;          // Fluglage via MPU9250
    float pitch;
    float roll;
    bool altitude;      // Höhe via BMP280  
    bool sonar;         // US Sensor
    bool temperature;   // MPU9250
} txPayload_t;

typedef struct
{
    bool isconnect;
    payload_t payload;
} rcInterface_t;

class Radio : public Task::Base
{
    bool radioNumber; // 0 uses pipe[0] to transmit, 1 uses pipe[1] to received
    bool role;        // true(>0) = TX role, false(0) = RX role
    const uint64_t pipe[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

    float test = 456.78;
protected:
    RF24 *_radio; 
    rcInterface_t *rcInterface; 
 //   payload_t payload;

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

        radioNumber = 1; // 1 uses pipe[1] to recieve
        
        _radio = new RF24(PIN_RADIO_CE, PIN_RADIO_CSN); // Adresse in Variable speichern -> constuktor

        if (!_radio->begin())
        {
            LOGGER_FATAL("radio hardware is not responding!!");
            while (1)
            {} // hold in infinite loop
        }

        role = false;

        Serial.println(F("RF24/examples/GettingStarted"));
         //_radio->setChannel(76);
        //_radio->setDataRate(RF24_250KBPS);
        _radio->setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
        _radio->setPayloadSize(sizeof(rcInterface_t)); 
        _radio->openWritingPipe(pipe[radioNumber]);     // always uses pipe 0
        _radio->openReadingPipe(1, pipe[!radioNumber]); // using pipe 1
  
    delay(100);
    
        LOGGER_VERBOSE("...leave");
    } /*----------------------------- end of begin ------------------------------------*/

    virtual void update() override
    {
    LOGGER_VERBOSE("Enter....");  
    
            // This device is the receiver
            if(role){
                Serial.print(F("Role = "));Serial.println(role);
                uint8_t pipe;                                   //??? Ist aber wohl richtig

                _radio->startListening();
                if (_radio->available(&pipe)) {      
                        
                    uint8_t bytes = _radio->getPayloadSize(); 
                    _radio->read(&rcInterface->payload, bytes);    
                    
                    Serial.print(F("Received "));
                    Serial.print(bytes);                 
                    Serial.print(F(" bytes on pipe "));
                    Serial.println(pipe);                   
                    Serial.print("Throttle = ");Serial.println(rcInterface->payload.rcThrottle);  // Nur zum debuggen
                    Serial.print("YAW =      ");Serial.println(rcInterface->payload.rcYaw); 
                    Serial.print("Pitch =    ");Serial.println(rcInterface->payload.rcPitch);  
                    Serial.print("Roll =     ");Serial.println(rcInterface->payload.rcRoll); 
                    Serial.print("Switch 1 = ");Serial.println(rcInterface->payload.rcSwi1);  
                    Serial.print("Switch 2 = ");Serial.println(rcInterface->payload.rcSwi2); 
                    Serial.print("Switch 3 = ");Serial.println(rcInterface->payload.rcSwi3);  
                    Serial.print("Checksum = ");Serial.println(rcInterface->payload.checksum);  
                    Serial.print("Is Connect = ");Serial.println(rcInterface->isconnect);  
                    role = false;
                } // end of read block
                delay(100);
            }
            else{
                Serial.print(F("Role = "));Serial.println(role);
                _radio->stopListening();
                unsigned long start_timer = micros();
                bool report = _radio->write(&test, sizeof(float)); 
                unsigned long end_timer = micros();                      // end the timer
                Serial.println(end_timer - start_timer);  
                
                if (report) {
                    Serial.print(F("Transmission successful! "));          
                    Serial.print(F("Time to transmit = "));
                                   
                    Serial.print(F(" us. Sent: "));
                    Serial.println(test);                               
                } else {
                    Serial.println(F("Transmission failed or timed out")); 
                }
                role = true;
                delay(100);
            } // end of write block

        LOGGER_VERBOSE("....leave");
    } // ------------------- end of update --------------------------------------------*/
}; /*----------------------------- end of radio.h class -------------------------------*/

//#undef _DEBUG_