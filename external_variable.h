#ifndef EXTERNAL_VARIABLE_H
#define EXTERNAL_VARIABLE_H

#include "app_timer.h"

extern uint8_t times_up_value ;

APP_TIMER_DEF(dong_co_timer);
#define DONG_CO_TIMER_INTERVAL APP_TIMER_TICKS(20000) // dong mo dong co trong 20s 

#endif

