#pragma once

#include <Arduino.h>

typedef struct{
    uint16_t min_loop_time;
    uint16_t max_loop_time;
    uint16_t last_loop_time;
}performance_t;
