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

typedef struct// __attribute__((__packed__))
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
    bool isInitialized;
} TX_payload_t;       // Transmit data to RC

typedef struct
{
    TX_payload_t TX_payload; // Do not change position !!!!! Must be the first entry
    RX_payload_t RX_payload;
    bool isconnect;
} RC_interface_t;

class Radio : public Task::Base
{
public:
typedef enum{
    init,                // only the first time to prevent init while flying after isConnected = false 
    commit,               // Motors will start turning
    ready,               // Preparing the on state
    on
}state_t;

typedef enum{
    throttle_max = 1,
    throttle_min = 2,
    yaw_max = 4,
    yaw_min = 8,    
    pitch_max = 16,
    pitch_min = 32,
    roll_max = 64,
    roll_min = 128
}init_flag_t;

private:
    const uint64_t _pipe_TX = 0xF0F0F0E1L;
    const uint64_t _pipe_RX = 0xF0F0F0D2L;
    unsigned long _lastReceivedPacket;
    uint16_t _lostAckPackageCount;
    state_t _state;
    uint8_t _initFlags;

protected:
    RF24 *_radio;
    RC_interface_t *_RC_interface;
    TX_payload_t _debugTX_payload;   //Displays only values that differ from the previous value
    RX_payload_t _debugRX_payload;

public:
    /// @brief Constructor
    /// @param name Name from Taskmanager
    Radio(const String &name)
        : Task::Base(name)
    {
        _lostAckPackageCount = 0;
        _state = init;
        _initFlags = 0;
    }
    /// @brief
    /// @param _model
    /// @return himself
    Radio *setModel(RC_interface_t *_model)
    {
        LOGGER_VERBOSE("Enter....");
        _RC_interface = _model;
        _RC_interface->isconnect = false;
        _RC_interface->TX_payload.isInitialized = false;
        LOGGER_VERBOSE("....leave");
        return this;
    } /*----------------------------- end of setModel ------------------------------------------*/

    virtual void begin() override
    {
        LOGGER_VERBOSE("Enter....");
        #ifndef _SERIAL1
            pinMode(LED_RADIO, OUTPUT);         // temp_debug Serial1
            digitalWrite(LED_RADIO, LOW);
        #endif

        SPI.begin();
        _radio = new RF24(PIN_RADIO_CE, PIN_RADIO_CSN);

        if (!_radio->begin(&SPI))
        {
            LOGGER_FATAL("Radio hardware is not responding!!");
            while (1)
            {}
        }
        _radio->setPALevel(RF24_PA_LOW);        // RF24_PA_MAX is default.
        _radio->enableDynamicPayloads();        // ACK payloads are dynamically sized
        _radio->enableAckPayload();
        _radio->openWritingPipe(_pipe_TX);       // always uses pipe 0
        _radio->openReadingPipe(1, _pipe_RX);    // using pipe 1
        // load the first response into the FIFO
        _radio->writeAckPayload(1, &_RC_interface->TX_payload, sizeof(TX_payload_t)); 
        _radio->startListening();
        LOGGER_VERBOSE("...leave");
    } /*----------------------------- end of begin ----------------------------------------------*/

    virtual void update() override
    {
        switch (_state)
        {
        case init:
            if(_RC_interface->RX_payload.rcThrottle>=78)
                _initFlags|=throttle_max;                   //  _initFlags = _initFlags | 1;
            if(_RC_interface->RX_payload.rcThrottle<=-80)
                _initFlags|=throttle_min;
            if(_RC_interface->RX_payload.rcYaw>=39)
                _initFlags|=yaw_min;
           if(_RC_interface->RX_payload.rcYaw<=-40)
                _initFlags|=yaw_max;                                  
            if(_RC_interface->RX_payload.rcPitch>=14)
                _initFlags|=pitch_max;                    
            if(_RC_interface->RX_payload.rcPitch<=-15)
                _initFlags|=pitch_min;
           if(_RC_interface->RX_payload.rcRoll>=14)
                _initFlags|=roll_max;
            if(_RC_interface->RX_payload.rcPitch<=-15)
                _initFlags|=roll_min;                
            if(_initFlags == 255){
                static int16_t debug_res;
                int16_t res = _RC_interface->RX_payload.rcThrottle +
                   _RC_interface->RX_payload.rcPitch +
                   _RC_interface->RX_payload.rcRoll +
                   _RC_interface->RX_payload.rcYaw;
                LOGGER_NOTICE_FMT_CHK(res,debug_res,"Must be 0 -> %d",res);
                if((res >= -6) && (res <= 6)){
                    _state = commit;
                    LOGGER_NOTICE("-> commit");
                }
            }
            break;
        case commit:
            if(_RC_interface->RX_payload.rcThrottle<=-80){
                _state = ready;
                LOGGER_NOTICE("-> ready");
            }
            break;
        case ready:
            LOGGER_NOTICE("Ready");
            _RC_interface->TX_payload.isInitialized = true;
            _state = on;
            LOGGER_NOTICE("-> on");
            break;
        case on:
            if(_RC_interface->TX_payload.isInitialized == false){           // Will be set to false from flycontroller
                _state = commit;
                LOGGER_NOTICE("-> commit");
            }
            break;
        default:
            break;
        }

        if (_radio->available())
        {
            //LOGGER_NOTICE("radio available");
            #ifndef _SERIAL1                    // temp_debug
                digitalWrite(LED_RADIO, HIGH);
            #endif
            _radio->read(&_RC_interface->RX_payload, sizeof(RX_payload_t));
            received_data_from_RC();

            _radio->writeAckPayload(1, &_RC_interface->TX_payload, sizeof(TX_payload_t));
            // transmit_data_to_RC();

            _lostAckPackageCount = 0;
            _RC_interface->isconnect = true;
            #ifndef _SERIAL1
                digitalWrite(LED_RADIO, LOW);   //temp_debug
            #endif
        }
        else
        {
            LOGGER_WARNING("Transmission fault or time out");
            _lostAckPackageCount++;
            if (_lostAckPackageCount > ACK_PACKAGE_MAX_COUNT)
            {
                _lostAckPackageCount = ACK_PACKAGE_MAX_COUNT; // protect against variable overflow (uint8_t)
                _RC_interface->isconnect = false;
            }
        }
        LOGGER_VERBOSE("....leave");
    } // ------------------- end of update ------------------------------------------------------*/

    void received_data_from_RC()
    {
        LOGGER_NOTICE_FMT_CHK(_RC_interface->RX_payload.rcThrottle, _debugRX_payload.rcThrottle, 
                                "Received Thottle = %i", _RC_interface->RX_payload.rcThrottle);
        // LOGGER_NOTICE_FMT_CHK(_RC_interface->RX_payload.rcYaw, _debugRX_payload.rcYaw, 
        //                         "Received Yaw = %i", _RC_interface->RX_payload.rcYaw);
        // LOGGER_NOTICE_FMT_CHK(_RC_interface->RX_payload.rcPitch, _debugRX_payload.rcPitch, 
        //                         "Received Pitch = %i", _RC_interface->RX_payload.rcPitch);
        // LOGGER_NOTICE_FMT_CHK(_RC_interface->RX_payload.rcRoll, _debugRX_payload.rcRoll, 
        //                         "Received Roll = %i", _RC_interface->RX_payload.rcRoll);
        // LOGGER_NOTICE_FMT_CHK(_RC_interface->RX_payload.rcSwi1, _debugRX_payload.rcSwi1, 
        //                         "Received Swi1 = %i", _RC_interface->RX_payload.rcSwi1); // Switch noch nicht aktiv
        // LOGGER_NOTICE_FMT_CHK(_RC_interface->RX_payload.rcSwi2, _debugRX_payload.rcSwi2, 
        //                         "Received Swi2 = %i", _RC_interface->RX_payload.rcSwi2);
        // LOGGER_NOTICE_FMT_CHK(_RC_interface->RX_payload.rcSwi3, _debugRX_payload.rcSwi3, 
        //                         "Received Swi3 = %i", _RC_interface->RX_payload.rcSwi3);
        // LOGGER_NOTICE_FMT_CHK(_RC_interface->RX_payload.rcAltitudeBaroAdj, _debugRX_payload.rcAltitudeBaroAdj, 
        //                         "Received max Alt. = %i", _RC_interface->RX_payload.rcAltitudeBaroAdj);
        // LOGGER_NOTICE_FMT_CHK(_RC_interface->RX_payload.rcAltitudeSonicAdj, _debugRX_payload.rcAltitudeSonicAdj, 
        //                         "Received max from ground = %i", _RC_interface->RX_payload.rcAltitudeSonicAdj);
    } // ------------------- end of received_data_from_RC ---------------------------------------*/

    void transmit_data_to_RC()
    {
        LOGGER_NOTICE_FMT_CHK(_RC_interface->TX_payload.yaw, _debugTX_payload.yaw, 
                                "Transmit Yaw = %i", _RC_interface->TX_payload.yaw);
        LOGGER_NOTICE_FMT_CHK(_RC_interface->TX_payload.pitch, _debugTX_payload.pitch, 
                                "Transmit Pitch = %i", _RC_interface->TX_payload.pitch);
        LOGGER_NOTICE_FMT_CHK(_RC_interface->TX_payload.roll, _debugTX_payload.roll, 
                                "Transmit Roll = %i", _RC_interface->TX_payload.roll);
        LOGGER_NOTICE_FMT_CHK(_RC_interface->TX_payload.altitude, _debugTX_payload.altitude, 
                                "Transmit Altitude = %i", _RC_interface->TX_payload.altitude); // Switch noch nicht aktiv
        LOGGER_NOTICE_FMT_CHK(_RC_interface->TX_payload.distance_down, _debugTX_payload.distance_down, 
                                "Transmit Dist. down = %i", _RC_interface->TX_payload.distance_down);
        LOGGER_NOTICE_FMT_CHK(_RC_interface->TX_payload.distance_front, _debugTX_payload.distance_front, 
                                "Transmit Dist. fronr = %i", _RC_interface->TX_payload.distance_front);
        LOGGER_NOTICE_FMT_CHK(_RC_interface->TX_payload.pressure, _debugTX_payload.pressure, 
                                "Transmit Pressure = %i", _RC_interface->TX_payload.pressure);
        LOGGER_NOTICE_FMT_CHK(_RC_interface->TX_payload.temperature, _debugTX_payload.temperature, 
                                "Transmit Temperatur = %i", _RC_interface->TX_payload.temperature);
    } // ------------------- end of transmit_data_to_RC -----------------------------------------*/

}; /*----------------------------- end of radio class -------------------------------------------*/