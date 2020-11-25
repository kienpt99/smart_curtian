/**@brief Cac ham lien quan den timer 
 */
#include "timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/**@brief Ham khoi tao timer 
 * @param[in] TIMER_LED bien timer can khai bao ban dau 
 * @param[in] timer0_handler  Ham xu ly ngat cua timer.
 */
void timer_init(nrfx_timer_t timer_instance , nrf_timer_event_handler_t timer_handler)
{
		uint32_t err_code = NRF_SUCCESS;
		uint32_t time_ms = 2000 ;
		uint32_t time_ticks ; // thoi gain dang tich tac 
		nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG; // bien cau hinh timer 
		err_code = nrfx_timer_init(&timer_instance,&timer_config,timer_handler);// khoi tao 
		APP_ERROR_CHECK(err_code);
		time_ticks = nrfx_timer_ms_to_ticks(&timer_instance,time_ms); // chuyen ms sang tich tac 
		nrfx_timer_extended_compare(&timer_instance,NRF_TIMER_CC_CHANNEL1,time_ticks,NRF_TIMER_SHORT_COMPARE0_STOP_MASK,true);/* gan kenh cho timer va gan gia tri dem */ 
		NRF_LOG_INFO("Da khoi tao timer");
}
