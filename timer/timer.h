#ifndef TIMER_H
#define TIMER_H
/**@brief Cac ham lien quan den timer 
 */
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_drv_timer.h"
#include "bsp.h"
#include "app_error.h"

/**@brief Ham khoi tao timer 
 * @param[in] timer_instance bien timer can khai bao ban dau 
 * @param[in] timer_handler  Ham xu ly ngat cua timer.
 */
void timer_init(nrfx_timer_t timer_instance , nrf_timer_event_handler_t timer_handler);
#endif

