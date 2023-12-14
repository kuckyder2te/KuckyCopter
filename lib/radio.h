#pragma once
/*  File name : radio.h
    Project: KuckyCopter 2
    Autors: Stephan Scholz / Wilhelm Kuckelsberg
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
//#include <SPI.h>
#include <RF24.h>

#define LOCAL_DEBUG
#include "myLogger.h"

#include "def.h"

#ifdef _RADIO
    #define ACK_PACKAGE_MAX_COUNT 10000
#else
    #define  ACK_PACKAGE_MAX_COUNT 10
#endif

typedef struct __attribute__((__packed__))
{
    int16_t rcYaw;
    int8_t rcPitch;
    int8_t rcRoll;
    int16_t rcThrottle;          // !< Get the positions of the rc joysticks
    uint16_t rcAltitudeSonicAdj; // Value is set via RC potentiometer 0 - 200cm
    uint16_t rcAltitudeBaroAdj;  // Value is set via RC potentiometer 0 - 10m
    bool rcSwi1;                 // Switches to programming mode, not active
    bool rcSwi2;                 // autonomous flying, not active
    bool rcSwi3;                 // ???
} RX_payload_t;                  // The drone receives data from the RC.

typedef struct __attribute__((__packed__))
{
    int16_t yaw; // MPU9250
    int8_t pitch;
    int8_t roll;
    uint16_t altitude; // MS5611
    float temperature;  // intern temperature
    float pressure;
    uint16_t distance_down; // US Sensor
    uint16_t distance_front;
    uint16_t battery; // State of the battery
} TX_payload_t;       // Transmit data to RC

typedef struct
{
    TX_payload_t TX_payload; // Do not change position !!!!! Must be the first entry
    RX_payload_t RX_payload;
    bool isconnect;
} RC_interface_t;

class Radio : public Task::Base
{
    const uint64_t pipe_TX = 0xF0F0F0E1L;
    const uint64_t pipe_RX = 0xF0F0F0D2L;
    unsigned long _lastReceivedPacket;
    uint16_t _lostAckPackageCount;

protected:
    RF24 *_radio;
    
    RC_interface_t *RC_interface;
    TX_payload_t debugTX_payload;   //Displays only values that differ from the previous value
    RX_payload_t debugRX_payload;

public:
    /// @brief Constructor
    /// @param name Name from Taskmanager
    Radio(const String &name)
        : Task::Base(name)
    {
        _lostAckPackageCount = 0;
    }
    /// @brief
    /// @param _model
    /// @return himself
    Radio *setModel(RC_interface_t *_model)
    {
        LOGGER_VERBOSE("Enter....");
        RC_interface = _model;
        RC_interface->isconnect = false;
        LOGGER_VERBOSE("....leave");
        return this;
    } /*----------------------------- end of setModel ------------------------------------------*/

    virtual void begin() override
    {
        LOGGER_VERBOSE("Enter....");
        pinMode(LED_RADIO, OUTPUT);
        digitalWrite(LED_RADIO, LOW);
        SPI.begin();
        _radio = new RF24(PIN_RADIO_CE, PIN_RADIO_CSN);

        if (!_radio->begin(&SPI))
        {
            LOGGER_FATAL("Radio hardware is not responding!!");
            while (1)
            {
            }
        }
        _radio->setPALevel(RF24_PA_LOW); // RF24_PA_MAX is default.
        _radio->enableDynamicPayloads(); // ACK payloads are dynamically sized
        _radio->enableAckPayload();
        _radio->openWritingPipe(pipe_TX);                                            // always uses pipe 0
        _radio->openReadingPipe(1, pipe_RX);                                         // using pipe 1
        _radio->writeAckPayload(1, &RC_interface->TX_payload, sizeof(TX_payload_t)); // load the first response into the FIFO
        _radio->startListening();
        LOGGER_VERBOSE("...leave");
    } /*----------------------------- end of begin ----------------------------------------------*/

    virtual void update() override
    {
        if (_radio->available())
        {
            LOGGER_NOTICE("radio available");
            digitalWrite(LED_RADIO, LOW);
            _radio->read(&RC_interface->RX_payload, sizeof(RX_payload_t));
            received_data_from_RC();

            _radio->writeAckPayload(1, &RC_interface->TX_payload, sizeof(TX_payload_t));
        //    transmit_data_to_RC();

            _lostAckPackageCount = 0;
            RC_interface->isconnect = true;
            digitalWrite(LED_RADIO, HIGH);
        }
        else
        {
            LOGGER_NOTICE("Transmission fault or time out");
         //   digitalWrite(LED_RADIO, LOW);
            _lostAckPackageCount++;
            if (_lostAckPackageCount > ACK_PACKAGE_MAX_COUNT)
            {
                _lostAckPackageCount = ACK_PACKAGE_MAX_COUNT; // protect against variable overflow (uint8_t)
                RC_interface->isconnect = false;
            }
        }
        LOGGER_VERBOSE("....leave");
    } // ------------------- end of update ------------------------------------------------------*/

    void received_data_from_RC()
    {
         Serial.println(RC_interface->RX_payload.rcThrottle);
        // LOGGER_NOTICE_FMT_CHK(RC_interface->RX_payload.rcThrottle, debugRX_payload.rcThrottle, "Received Thottle = %i", RC_interface->RX_payload.rcThrottle);
        // LOGGER_NOTICE_FMT_CHK(RC_interface->RX_payload.rcYaw, debugRX_payload.rcYaw, "Received Yaw = %i", RC_interface->RX_payload.rcYaw);
        // LOGGER_NOTICE_FMT_CHK(RC_interface->RX_payload.rcPitch, debugRX_payload.rcPitch, "Received Pitch = %i", RC_interface->RX_payload.rcPitch);
        // LOGGER_NOTICE_FMT_CHK(RC_interface->RX_payload.rcRoll, debugRX_payload.rcRoll, "Received Roll = %i", RC_interface->RX_payload.rcRoll);
        // LOGGER_NOTICE_FMT_CHK(RC_interface->RX_payload.rcSwi1, debugRX_payload.rcSwi1, "Received Swi1 = %i", RC_interface->RX_payload.rcSwi1); // Switch noch nicht aktiv
        // LOGGER_NOTICE_FMT_CHK(RC_interface->RX_payload.rcSwi2, debugRX_payload.rcSwi2, "Received Swi2 = %i", RC_interface->RX_payload.rcSwi2);
        // LOGGER_NOTICE_FMT_CHK(RC_interface->RX_payload.rcSwi3, debugRX_payload.rcSwi3, "Received Swi3 = %i", RC_interface->RX_payload.rcSwi3);
        // LOGGER_NOTICE_FMT_CHK(RC_interface->RX_payload.rcAltitudeBaroAdj, debugRX_payload.rcAltitudeBaroAdj, "Received max Alt. = %i", RC_interface->RX_payload.rcAltitudeBaroAdj);
        // LOGGER_NOTICE_FMT_CHK(RC_interface->RX_payload.rcAltitudeSonicAdj, debugRX_payload.rcAltitudeSonicAdj, "Received max from ground = %i", RC_interface->RX_payload.rcAltitudeSonicAdj);
    } // ------------------- end of received_data_from_RC ---------------------------------------*/

    void transmit_data_to_RC()
    {
        LOGGER_NOTICE_FMT_CHK(RC_interface->TX_payload.yaw, debugTX_payload.yaw, "Transmit Yaw = %i", RC_interface->TX_payload.yaw);
        LOGGER_NOTICE_FMT_CHK(RC_interface->TX_payload.pitch, debugTX_payload.pitch, "Transmit Pitch = %i", RC_interface->TX_payload.pitch);
        LOGGER_NOTICE_FMT_CHK(RC_interface->TX_payload.roll, debugTX_payload.roll, "Transmit Roll = %i", RC_interface->TX_payload.roll);
        LOGGER_NOTICE_FMT_CHK(RC_interface->TX_payload.altitude, debugTX_payload.altitude, "Transmit Altitude = %i", RC_interface->TX_payload.altitude); // Switch noch nicht aktiv
        LOGGER_NOTICE_FMT_CHK(RC_interface->TX_payload.distance_down, debugTX_payload.distance_down, "Transmit Dist. down = %i", RC_interface->TX_payload.distance_down);
        LOGGER_NOTICE_FMT_CHK(RC_interface->TX_payload.distance_front, debugTX_payload.distance_front, "Transmit Dist. fronr = %i", RC_interface->TX_payload.distance_front);
        LOGGER_NOTICE_FMT_CHK(RC_interface->TX_payload.pressure, debugTX_payload.pressure, "Transmit Pressure = %i", RC_interface->TX_payload.pressure);
        LOGGER_NOTICE_FMT_CHK(RC_interface->TX_payload.temperature, debugTX_payload.temperature, "Transmit Temperatur = %i", RC_interface->TX_payload.temperature);
    } // ------------------- end of transmit_data_to_RC -----------------------------------------*/

}; /*----------------------------- end of radio class -------------------------------------------*/