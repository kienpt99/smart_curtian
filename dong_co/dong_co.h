#ifndef DONG_CO_H
#define DONG_CO_H
/**@brief Cac ham lien quan den dong co 
 */
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_drv_timer.h"
#include "bsp.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define chieu_thuan 15
#define chieu_nghich 16 

/**@brief Ham xu ly ngat cua timer dong co 
	De bao hieu khi ma timer da dem xong thi gia tri times_up_value = 1 
 * @param[out] times_ip_value = 1 
 */
//void timer_dong_co_handler(nrf_timer_event_t event_type , void* p_context );

/**@brief Khoi dong dong co , set chan dau ra cho cac chan dk , timer init
*/
void dong_co_init(void);


/**@brief Ham mo dong co 
	Mo dong co trong vong 20s 
*/
void mo_dong_co(void);


/**@brief Ham dong dong co 
	dong dong co trong vong 20s 
*/
void dong_dong_co(void);
#endif

