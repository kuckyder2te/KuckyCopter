#pragma once
/*  File name : 
    Project name : KuCo_Phantom 1
    Author: Wilhelm Kuckelsberg
    Date : 2022-06-19

    Description : Drohne
 
*/

#include <Arduino.h>
#include <TaskManager.h>

class FlyController : public Task::Base {
public:
    FlyController(const String& name)
    : Task::Base(name) {
    }

    virtual ~FlyController() {}

    // optional (you can remove this method)
    // virtual void begin() override {
    // }

    // optional (you can remove this method)
    // virtual void enter() override {
    // }

    virtual void update() override {
    }

    // optional (you can remove this method)
    // virtual void exit() override {
    // }

    // optional (you can remove this method)
    // virtual void idle() override {
    // }

    // optional (you can remove this method)
    // virtual void reset() override {
    // }
};
