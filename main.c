// Freertos
// Note
// mode 2 
// mode 1 
// not sure
// ble+
// do nothing 
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "fds.h"
#include "peer_manager.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_saadc.h"

// Freertos: include thu vien freertos
#include "FreeRTOS.h"
#include "task.h" 
#include "timers.h" 
#include "queue.h"
#include "semphr.h"
#include "app_timer.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_clock.h"

#include "service/our_service.h"


#define DEVICE_NAME                     "Smart_Curtian"                       /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                65535                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */


#define READ_ADC_INTERVAL         			10000                                 /**< Freertos: Read adc interval (ms). */
#define DONG_CO_TIMER_INTERVAL 					2000																/**< Freertos: Control motor interval (ms). */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

// Freertos
#define OSTIMER_WAIT_FOR_QUEUE              2                                       /**< Number of ticks to wait for the timer queue to be ready */

// Note: Configure pin to control curtain.
#define open_curtain 15
#define close_curtain 16 

// Note mode 2: Configure button to control curtain directly.
#define BTN_open_curtain 24 
#define BTN_close_curtain 25 

// Note: Definition of state variable of curtain. curtain_state_global = 0 (close) | curtain_state_global = 1 (open)
uint8_t curtain_state_global = 2 ;
uint8_t mode = 2;

NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */


// ble+: khai bao cac service duoc su dung.
ble_os_t m_our_service; // service on/off
ble_os_t timer_service; // service hen gio 
ble_os_t mode_selec_service; // service chuyen mode


// ble+: khai bai bien de adv uuid 
static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    {BLE_UUID_OUR_SERVICE_UUID, BLE_UUID_TYPE_BLE},
    {BLE_UUID_TM_SERVICE_UUID, BLE_UUID_TYPE_BLE},
    {BLE_UUID_MDSL_SERVICE_UUID, BLE_UUID_TYPE_BLE}
};

// mode 1 
static TimerHandle_t read_adc_timer;                               // Freertos: tham chieu den timer doc adc.
static TimerHandle_t dong_co_timer;                               // Freertos: tham chieu den timer dong co.

QueueHandle_t xQueue;																							//Freertos: Khai bao queue

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;                               // Freertos: Dinh nghia luong logger 
#endif

//// Freertos mode 1 : Ham callback cua adc_timer de doc gia tri tu adc, va gui du lieu vao message queue.(send)
static void read_adc_timer_callback(TimerHandle_t xTimer)
{
		UNUSED_PARAMETER(xTimer);
		if(mode == 1)
			{
				BaseType_t xStatus;
				nrf_saadc_value_t adc_value;
				nrfx_saadc_sample_convert(0,&adc_value);
				NRF_LOG_FLUSH();
				NRF_LOG_INFO("Gia tri doc duoc cua ADC: %d",adc_value);
				if(adc_value > 100)
					{
						if(curtain_state_global != 1)
							{
								curtain_state_global = 1;
								xStatus = xQueueOverwrite( xQueue, &curtain_state_global); // Freertos: block time = 0, not sure , khong wait khi queue is full.
								if( xStatus != pdPASS )
								{
								/* The send operation could not complete because the queue was full -
								this must be an error as the queue should never contain more than
								one item! */
								NRF_LOG_INFO("Khong gui duoc du lieu den queue ",adc_value);
								}
							}
					}else{
						if(curtain_state_global != false)
							{
								curtain_state_global = 0;
								xStatus = xQueueOverwrite( xQueue, &curtain_state_global);; // Freertos: block time = 0, not sure , khong wait khi queue is full.
								if( xStatus != pdPASS )
								{
								/* The send operation could not complete because the queue was full -
								this must be an error as the queue should never contain more than
								one item! */
								NRF_LOG_INFO("Khong gui duoc du lieu den queue ",adc_value);
								}
							}				
					}
			}
}

// Freertos: Ham callback cua timer dong co de dieu khien dong co trong 1 khoang tgian.
static void dong_co_timer_callback(TimerHandle_t xTimer)
{
		if(curtain_state_global == 1)
			{
				nrf_gpio_pin_clear(open_curtain);
			}
		else if(curtain_state_global == 0)
			{
				nrf_gpio_pin_clear(close_curtain);				
			}
		else
			{
				
			}
}

// Note mode 1: Ham bo tro cho saadc 
void saadc_callback_handler(nrfx_saadc_evt_t const *p_event )
	{
		
	}

void saadc_init(void)
{
	uint32_t err_code;
	nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
	
	err_code = nrf_drv_saadc_init(NULL, saadc_callback_handler);
	APP_ERROR_CHECK(err_code);
	
	err_code = nrfx_saadc_channel_init(0,&channel_config);
	APP_ERROR_CHECK(err_code);
	
}

// Note mode 2: Ham callback cua nut mo rem 
void open_button_callback(nrf_drv_gpiote_pin_t pin , nrf_gpiote_polarity_t action)
	{
		if(mode == 2)
		{
			BaseType_t xStatus;
			curtain_state_global = 1;
			xStatus = xQueueOverwrite( xQueue, &curtain_state_global); // Freertos: block time = 0, not sure , khong wait khi queue is full.
			if( xStatus != pdPASS )
			{
				/* The send operation could not complete because the queue was full -
				this must be an error as the queue should never contain more than
				one item! */
				NRF_LOG_INFO("Khong gui duoc du lieu den queue");
			}
		}
	}
	
// Note mode 2: Ham callback cua nut dong rem 
void close_button_callback(nrf_drv_gpiote_pin_t pin , nrf_gpiote_polarity_t action)
	{
		if(mode == 2)
		{
			BaseType_t xStatus;
			curtain_state_global = 0;
			xStatus = xQueueOverwrite( xQueue, &curtain_state_global); // Freertos: block time = 0, not sure , khong wait khi queue is full.
			if( xStatus != pdPASS )
			{
				/* The send operation could not complete because the queue was full -
				this must be an error as the queue should never contain more than
				one item! */
				NRF_LOG_INFO("Khong gui duoc du lieu den queue");
			}
		}
	}

// Note mode 2: Ham khoi tao GPIOTE
// Delete
void gpio_init()
	{
		uint32_t err_code ;
		if (!nrf_drv_gpiote_is_init())
		{
			err_code = nrf_drv_gpiote_init();
			APP_ERROR_CHECK(err_code);
		}
			nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
			in_config.pull = NRF_GPIO_PIN_PULLUP;
			err_code = nrf_drv_gpiote_in_init(BTN_open_curtain,&in_config,open_button_callback);
			APP_ERROR_CHECK(err_code);
			err_code = nrf_drv_gpiote_in_init(BTN_close_curtain,&in_config,close_button_callback);	
			APP_ERROR_CHECK(err_code);
			nrf_drv_gpiote_in_event_enable(BTN_open_curtain,true);
			nrf_drv_gpiote_in_event_enable(BTN_close_curtain,true);
			NRF_LOG_INFO("Da khoi tao xong GPIOTE");
	}

// Note: Ham khoi tao cac chan dieu khien dong co 
void dong_co_init(void)
	{
		// Cau hinh 2 chan dieu khien dong co la dau ra 
		nrf_gpio_cfg_output(open_curtain);
		nrf_gpio_cfg_output(close_curtain);	
		// ban dau khong dong cung nhu khong mo
		nrf_gpio_pin_clear(open_curtain);
		nrf_gpio_pin_clear(close_curtain);
		NRF_LOG_INFO("Da khoi tao dong co");
	}


static void advertising_start(void * p_erase_bonds);


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start(false);
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // This can happen when the local DB has changed.
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}



/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
// Delete
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);


    // OUR_JOB: Step 3.H, Initiate our timer gan day
    dong_co_timer = xTimerCreate("Dong co timer",
                                   DONG_CO_TIMER_INTERVAL,
                                   pdFALSE,
                                   NULL,
                                   dong_co_timer_callback);	
	
		// Freertos mode 1 : tao timer
    read_adc_timer = xTimerCreate("ADC timer",
                                   READ_ADC_INTERVAL,
                                   pdTRUE,
                                   NULL,
                                   read_adc_timer_callback);	
		// Check loi mode 1
    if (NULL == read_adc_timer || NULL == dong_co_timer)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
		// Check loi mode 2
    if (NULL == dong_co_timer)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{

	
		uint32_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};
		// ble+: khoi tao value ban dau cho characteristic
		uint8_t value_1 = 0; // 0: mo rem, 1: dong rem
		uint8_t value_3 = 0;	// 1: mode 1, 2: mode 2
		uint8_t value_2[6] = {0x12,0x23,0x34,0x45,0x56,0x67};	// ngay/thang/nam/nam/gio/phut

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // ble+: khoi tao cac service da duoc khai bao
    our_service_init(&m_our_service,BLE_UUID_OUR_BASE_UUID,BLE_UUID_OUR_SERVICE_UUID,BLE_UUID_OUR_CHARACTERISTC_UUID,1,&value_1);
		our_service_init(&timer_service,BLE_UUID_TMSV_BASE_UUID,BLE_UUID_TM_SERVICE_UUID, BLE_UUID_TM_CHARACTERISTC_UUID,6,value_2);
		our_service_init(&mode_selec_service,BLE_UUID_MDSL_BASE_UUID,BLE_UUID_MDSL_SERVICE_UUID, BLE_UUID_MDSL_CHARACTERISTC_UUID,1,&value_3);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
		// Freertos mode 1 : Bat dau chay timer 
    if (pdPASS != xTimerStart(read_adc_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
		// deep mode
		err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;
		

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            // LED indication will be changed when advertising starts.
            break;
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        default:
            // No implementation needed.
            break;
    }

		
}

// Freertos: Task dieu khien dong co(receive)
static void vControlMotorTask( void *pvParameters )
{
	while(true)
		{
			/* Declare the variable that will hold the values received from the queue. */
			NRF_LOG_INFO("Da vao ham dieu khien dong co");
			uint8_t curtain_state_received;
			BaseType_t xStatus;
			xStatus = xQueueReceive( xQueue, &curtain_state_received, portMAX_DELAY );
			NRF_LOG_INFO("Da qua cau lenh gui du lieu den queue");
			if( xStatus == pdPASS )
			{
				NRF_LOG_INFO( "Received = %d ", curtain_state_received );
			}else{
				NRF_LOG_INFO( "Could not receive from the queue.\r\n" );
			}
			if (curtain_state_received)
			{
					nrf_gpio_pin_set(open_curtain); // mo rem.
					curtain_state_global = curtain_state_received;
					xTimerStart(dong_co_timer, OSTIMER_WAIT_FOR_QUEUE);
			}
			else
			{
					nrf_gpio_pin_set(close_curtain); // dong rem.
					curtain_state_global = curtain_state_received;
					xTimerStart(dong_co_timer, OSTIMER_WAIT_FOR_QUEUE);
			}
		}
}
/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_led_service  Instance of LED Service to which the write applies.
 * @param[in] led_state      Written/desired state of the LED.
 */
static void curtain_write_handler(uint16_t conn_handle, ble_os_t * p_led_service, uint8_t curtain_state)
{
    if (curtain_state && mode == 2)
    {
			BaseType_t xStatus;
			curtain_state_global = 1;
			xStatus = xQueueOverwrite( xQueue, &curtain_state_global); // Freertos: block time = 0, not sure , khong wait khi queue is full.
			if( xStatus != pdPASS )
			{
				/* The send operation could not complete because the queue was full -
				this must be an error as the queue should never contain more than
				one item! */
				NRF_LOG_INFO("Khong gui duoc du lieu den queue ");
			}
    }
    else if(mode == 2)
    {
			BaseType_t xStatus;
			curtain_state_global = 0;
			xStatus = xQueueOverwrite( xQueue, &curtain_state_global); // Freertos: block time = 0, not sure , khong wait khi queue is full.
			if( xStatus != pdPASS )
			{
				/* The send operation could not complete because the queue was full -
				this must be an error as the queue should never contain more than
				one item! */
				NRF_LOG_INFO("Khong gui duoc du lieu den queue ");
			}
		}
}

static void timer_service_callback(uint16_t conn_handle, ble_os_t * p_led_service, uint8_t curtain_state)
{
	// do nothing 
}

static void mode_selec_service_callback(uint16_t conn_handle, ble_os_t * p_led_service, uint8_t curtain_state)
{
	if(curtain_state == 1)
		{
			NRF_LOG_INFO("Da vao mode 1: Tu dong");
			mode = 1;
		}
	else if(curtain_state == 2)
		{
			NRF_LOG_INFO("Da vao mode 2: Thu cong");
			mode = 2;
		}
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

    //OUR_JOB: Step 3.C Call ble_our_service_on_ble_evt() to do housekeeping of ble connections related to our service and characteristics
	
		m_our_service.curtain_write_handler = curtain_write_handler ;
		NRF_SDH_BLE_OBSERVER(m_our_service_observer, APP_BLE_OBSERVER_PRIO, ble_our_service_on_ble_evt_1,(void*)&m_our_service);

		timer_service.curtain_write_handler = timer_service_callback ;
		NRF_SDH_BLE_OBSERVER(timer_service_observer, APP_BLE_OBSERVER_PRIO, ble_our_service_on_ble_evt_2,(void*)&timer_service);
		
		mode_selec_service.curtain_write_handler = mode_selec_service_callback ;
		NRF_SDH_BLE_OBSERVER(mode_select_service_observer, APP_BLE_OBSERVER_PRIO, ble_our_service_on_ble_evt_3,(void*)&mode_selec_service);		
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

	
		init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
		init.srdata.uuids_complete.p_uuids = m_adv_uuids;
		
	
    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;
		
	// ble+: them apperance cho application
		sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_REMOTE_CONTROL);		

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

// Note : Ham khoi tao button va led mac dinh. 
/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising. */
static void advertising_start(void * p_erase_bonds)
{
    bool erase_bonds = *(bool*)p_erase_bonds;

    if (erase_bonds)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}

// Freertos
#if NRF_LOG_ENABLED
/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void * arg)
{
    UNUSED_PARAMETER(arg);

    while (1)
    {
        NRF_LOG_FLUSH();

        vTaskSuspend(NULL); // Suspend myself
    }
}
#endif //NRF_LOG_ENABLED


// Freertos
/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
#if NRF_LOG_ENABLED
     vTaskResume(m_logger_thread);
#endif
}

// Freertos
/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;
		// Freertos
		xQueue = xQueueCreate(1, sizeof( uint8_t ));

    // Initialize modules.
    log_init();
    clock_init();
	
		#if NRF_LOG_ENABLED
				// Start execution.
				if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 100, NULL, 1, &m_logger_thread))
				{
						APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
				}
		#endif
		if(pdPASS == xTaskCreate( vControlMotorTask, "Motor Task", 100, NULL, 3, NULL ))
			{
				NRF_LOG_INFO("Khoi tao task thanh cong");
			}else{
				NRF_LOG_INFO("Khoi tao task khong thanh cong");				
			}
				
				
		// Activate deep sleep mode.
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
				
    // Configure and initialize the BLE stack.
    ble_stack_init();
				
    // Initialize modules.		
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    gap_params_init();
    gatt_init();
		// delete
		dong_co_init();
		gpio_init(); // mode 2
		saadc_init();
	
	  services_init();
		advertising_init();
		
    conn_params_init();
    peer_manager_init();

    // Start execution.
    NRF_LOG_INFO("OurCharacteristics tutorial started.");
		NRF_LOG_FLUSH();
    application_timers_start();

    // Create a FreeRTOS task for the BLE stack.
    // The task will run advertising_start() before entering its loop.
    nrf_sdh_freertos_init(advertising_start, &erase_bonds);
		
    NRF_LOG_INFO("Smart Curtain started.");
    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    // Enter main loop.
    for (;;)
    {
			APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}


/**
 * @}
 */
