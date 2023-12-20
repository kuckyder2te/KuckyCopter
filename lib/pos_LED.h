#pragma once
/*  File name : pos_LED.h
    Project name : KuckyCopter 2
    Author: Wilhelm Kuckelsberg
    Date : 2023-12-17

    Description : Blinks 4 position LEDs
*/

#include <Arduino.h>
#include <TaskManager.h>

#include "def.h"

class POS_LED : public Task::Base {
public:
    POS_LED(const String& name)
    : Task::Base(name) {
    }

    virtual ~POS_LED() {}

    virtual void update() override {

    static uint8_t count = 0;
     //   Serial2.println("LED_POS");
        digitalWrite(LED_POSITION, LOW);
        count++;
        if(count >= 4){
            digitalWrite(LED_POSITION, HIGH);
        count = 0;
        }
    }
}; /*----------------------------- end of pos_LED.h class ---------------------------------------*/
