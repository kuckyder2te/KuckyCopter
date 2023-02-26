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
  TX_payload_t TX_payload;  //Do not change position !!!!! Must be the first entry
  RX_payload_t RX_payload; 
  bool isconnect;
} RC_interface_t;

// struct PayloadStruct {
//   char message[7];  // only using 6 characters for TX & ACK payloads
//   uint8_t counter;
// };
// PayloadStruct payload;


class Radio : public Task::Base
{
//    bool radioNumber; // 0 uses pipe[0] to transmit, 1 uses pipe[1] to received
//    bool role, _role,__role,___role;        // true(>0) = TX role, false(0) = RX role
    //const uint64_t pipe[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };
    const uint64_t pipe_TX = 0xF0F0F0F0E1LL;
    const uint64_t pipe_RX = 0xF0F0F0F0D2LL;
    RX_payload_t _RX_payload;
    unsigned long _lastReceivedPacket;

protected:
    RF24 *_radio; 
    RC_interface_t *RC_interface;   
  //  TX_payload_t TX_payload, _TX_payload;


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

          // // print example's introductory prompt
          // Serial2.println(F("RF24/examples/AcknowledgementPayloads"));

          // // To set the radioNumber via the Serial2 monitor on startup
          // Serial2.println(F("Which radio is this? Enter '0' or '1'. Defaults to '0'"));
          // while (!Serial2.available()) {
          //   // wait for user input
          // }
          // char input = Serial2.parseInt();
          // radioNumber = input == 1;
          // // Serial2.print(F("radioNumber = "));
          // // Serial2.println((int)radioNumber);

          // // role variable is hardcoded to RX behavior, inform the user of this
          // Serial2.println(F("*** PRESS 'T' to begin transmitting to the other node"));


 //       role = false;        // Parameter als Empfänger
        // _role = !role;
        // __role = _role;
        // ___role = _role;
 //       radioNumber = 1; 
        
        _radio->setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
        _radio->enableDynamicPayloads();  // ACK payloads are dynamically sized
        _radio->enableAckPayload();
        _radio->openWritingPipe(pipe_TX);     // always uses pipe 0
        _radio->openReadingPipe(1, pipe_RX); // using pipe 1
        _radio->startListening(); 

        // TX_payload.yaw = 129.9;
        // TX_payload.pitch = -12.8;
        // TX_payload.roll = 41.2;
        // TX_payload.altitude = 111;
        // //TX_payload.sonic = 222;
        // TX_payload.temperature = 24.5;
        // TX_payload.pressure = 12345.0;
  
    delay(100);
    
        LOGGER_VERBOSE("...leave");
    } /*----------------------------- end of begin ------------------------------------*/

    virtual void update() override
    {
//         if (role) {
//         // This device is a TX node

//         unsigned long start_timer = micros();                  // start the timer
//         bool report = _radio->write(&payload, sizeof(payload));  // transmit & save the report

//         //LOGGER_NOTICE_FMT("Role = %i  RadioNumber = %i", role, radioNumber);
//         Serial2.print("Role = ");Serial2.println(role);
//         Serial2.print("Radionumber = ");Serial2.println(radioNumber);

//         unsigned long end_timer = micros();                    // end the timer

//         if (report) {
//           //LOGGER_NOTICE_FMT("Time to transmit = %i", end_timer - start_timer);
//           Serial2.print(F("Transmission successful! "));  // payload was delivered
//           Serial2.print(F("Time to transmit = "));
//           Serial2.print(end_timer - start_timer);  // print the timer result
//           Serial2.print(F(" us. Sent: "));
//           //LOGGER_NOTICE_FMT("us. sent = %s  %s",payload.message,payload.counter);
//            Serial2.println(payload.message);  // print the outgoing message
//            Serial2.println(payload.counter);  // print the outgoing counter
//           uint8_t pipe;
//           if (_radio->available(&pipe)) {  // is there an ACK payload? grab the pipe number that received it
//             PayloadStruct received;
//             _radio->read(&received, sizeof(received));  // get incoming ACK payload
//  //           LOGGER_NOTICE_FMT("Received %i bytes on pipe %i",_radio->getDynamicPayloadSize(), pipe);
//             Serial2.print(F(" Recieved "));
//             Serial2.print(_radio->getDynamicPayloadSize());  // print incoming payload size
//             Serial2.print(F(" bytes on pipe "));
//             Serial2.print(pipe);  // print pipe number that received the ACK
//  //           LOGGER_NOTICE_FMT("Message = %s counter = %i",received.message,received.counter);
//             Serial2.print(F(": "));
//             Serial2.print(received.message);    // print incoming message
//             Serial2.println(received.counter);  // print incoming counter

//             // save incoming counter & increment for next outgoing
//             payload.counter = received.counter + 1;

//           } else {
//             LOGGER_FATAL("Recieved: an empty ACK packet");
//             //Serial2.println(F(" Recieved: an empty ACK packet"));  // empty ACK packet received
//           }
//         } else {
//           LOGGER_FATAL("Transmission failed or timed out");  // payload was not delivered
//         }
//         // to make this example readable in the serial monitor
//         delay(100);  // slow transmissions down by 1 second

//       } else {
        // This device is a RX node

      //  uint8_t pipe;
        if (_radio->available()) {                     // is there a payload? get the pipe number that recieved it
          uint8_t bytes = _radio->getDynamicPayloadSize();  // get the size of the payload
      //    PayloadStruct received;
          _radio->read(&RC_interface->RX_payload, sizeof(RX_payload_t));  // get incoming payload
//          LOGGER_NOTICE_FMT("Reseived %i bytes on pipe %i : %s %i", bytes, pipe,received.message,received.counter);

          Serial2.print(F("Received "));
          Serial2.print(bytes);  // print the size of the payload
          Serial2.println(F(" bytes"));
     //     Serial2.print(pipe);  // print the pipe number
     //     Serial2.print(F(": "));
          RC_interface->TX_payload.sonic = 8888;
          Serial2.println(RC_interface->RX_payload.rcThrottle);  // print incoming message
          Serial2.println(RC_interface->TX_payload.sonic);  // print incoming counter
          
//          LOGGER_NOTICE_FMT("Sent:  %s %i", payload.message,payload.counter);
          // Serial2.print(F(" Sent: "));
          // Serial2.print(payload.message);    // print outgoing message
          // Serial2.println(payload.counter);  // print outgoing counter

          // // save incoming counter & increment for next outgoing
          // payload.counter = received.counter + 1;
          // // load the payload for the first received transmission on pipe 0
          _radio->writeAckPayload(1, &RC_interface->TX_payload, sizeof(TX_payload_t));
          _radio->startListening();
        }
//      }  // role

      // if (Serial2.available()) {
      //   // change the role via the serial monitor

      //   char c = toupper(Serial2.read());
      //   if (c == 'T' && !role) {
      //     // Become the TX node

      //     role = true;
      //     Serial2.println("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK");

      //     memcpy(payload.message, "Hello ", 6);  // change payload message
      //     _radio->stopListening();                 // this also discards any unused ACK payloads

      //   } else if (c == 'R' && role) {
      //     // Become the RX node

      //     role = false;
      //     Serial2.println("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK");
      //     memcpy(payload.message, "World ", 6);  // change payload message

      //     // load the payload for the first received transmission on pipe 0
      //     _radio->writeAckPayload(1, &payload, sizeof(payload));
      //     _radio->startListening();
      //   }
//      }
        LOGGER_VERBOSE("....leave");
    } // ------------------- end of update --------------------------------------------*/
}; /*----------------------------- end of radio->.h class -------------------------------*/