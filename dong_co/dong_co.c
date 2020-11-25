/**@brief Cac ham lien quan den dong co 
 */
#include "dong_co.h"

uint8_t times_up_value = 0;

//const nrfx_timer_t TIMER_DONG_CO = NRFX_TIMER_INSTANCE(4); // khai bao bien timer 

/**@brief Ham xu ly ngat cua timer dong co 
	De bao hieu khi ma timer da dem xong thi gia tri times_up_value = 1 
 * @param[out] times_ip_value = 1 
 */
//void timer_dong_co_handler(nrf_timer_event_t event_type , void* p_context)
//	{
//		switch(event_type)
//			{
//			case NRF_TIMER_EVENT_COMPARE0:
//				times_up_value = 1 ;
//			break;
//			default:
//			// Khong lam j ca 
//			break;
//			}
//	}

/**@brief Ham mo dong co 
	Mo dong co trong vong 20s 
*/
void mo_dong_co( void )
	{
		nrf_gpio_pin_set(chieu_thuan); // bat dong co quay thuan
		app_timer_start(dong_co_timer,DONG_CO_TIMER_INTERVAL,NULL);
		// chua toi uu
		while(times_up_value == 0);
		nrf_gpio_pin_clear(chieu_thuan);
		times_up_value = 0 ;
	}

/**@brief Ham dong dong co 
	dong dong co trong vong 20s 
*/
void dong_dong_co(void)
	{
		nrf_gpio_pin_set(chieu_nghich); // bat dong co quay thuan
		app_timer_start(dong_co_timer,DONG_CO_TIMER_INTERVAL,NULL);
		// chua toi uu
		while(times_up_value == 0);
		nrf_gpio_pin_clear(chieu_nghich);
		times_up_value = 0 ;
	}	
	
/**@brief Khoi dong dong co , set chan dau ra cho cac chan dk , timer init
*/
void dong_co_init(void)
	{
		// Cau hinh 2 chan dieu khien dong co la dau ra 
		nrf_gpio_cfg_output(chieu_thuan);
		nrf_gpio_cfg_output(chieu_nghich);	
		// ban dau khong quay thuan cung nhu quay nghich
		nrf_gpio_pin_clear(chieu_thuan);
		nrf_gpio_pin_clear(chieu_nghich);
		// Khoi tao timer 
//		timer_init(TIMER_DONG_CO,timer_dong_co_handler);
		NRF_LOG_INFO("Da khoi tao dong co");
	}
