
#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "our_service.h"
#include "ble_srv_common.h"
#include "app_error.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//static const uint8_t RemCharName[] = "Open/Close";

static void da_ghi(ble_os_t * p_our_service, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    if (   (p_evt_write->handle == p_our_service->char_handles.value_handle)
        && (p_evt_write->len == 1)
        && (p_our_service->curtain_write_handler != NULL))
    {
        p_our_service->curtain_write_handler(p_ble_evt->evt.gap_evt.conn_handle, p_our_service, p_evt_write->data[0]);
    }
}

void ble_our_service_on_ble_evt_1(ble_evt_t const * p_ble_evt, void * p_context)
{
  	ble_os_t * p_our_service =(ble_os_t *) p_context;  
		// OUR_JOB: Step 3.D Implement switch case handling BLE events related to our service. 
		switch(p_ble_evt->header.evt_id)
		{
			case BLE_GAP_EVT_CONNECTED:
				p_our_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			break;
			case BLE_GAP_EVT_DISCONNECTED:
				p_our_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;			
			break;
			case BLE_GATTS_EVT_WRITE:
				da_ghi(p_our_service, p_ble_evt); // mode 2
			break;
			default:
				break;
		}
}

void ble_our_service_on_ble_evt_2(ble_evt_t const * p_ble_evt, void * p_context)
{
  	ble_os_t * p_our_service =(ble_os_t *) p_context;  
		// OUR_JOB: Step 3.D Implement switch case handling BLE events related to our service. 
		switch(p_ble_evt->header.evt_id)
		{
			case BLE_GAP_EVT_CONNECTED:
				p_our_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			break;
			case BLE_GAP_EVT_DISCONNECTED:
				p_our_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;			
			break;
			case BLE_GATTS_EVT_WRITE:
				// do nothing 
			break;
			default:
				break;
		}
}

void ble_our_service_on_ble_evt_3(ble_evt_t const * p_ble_evt, void * p_context)
{
  	ble_os_t * p_our_service =(ble_os_t *) p_context;  
		// OUR_JOB: Step 3.D Implement switch case handling BLE events related to our service. 
		switch(p_ble_evt->header.evt_id)
		{
			case BLE_GAP_EVT_CONNECTED:
				p_our_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			break;
			case BLE_GAP_EVT_DISCONNECTED:
				p_our_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;			
			break;
			case BLE_GATTS_EVT_WRITE:
				da_ghi(p_our_service, p_ble_evt); // su kien chuyen mode
			break;
			default:
				break;
		}
}

/**@brief Function for adding our new characterstic to "Our service" that we initiated in the previous tutorial. 
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
//static uint32_t our_char_add(ble_os_t * p_our_service_1, ble_os_t * p_our_service_2, ble_os_t * p_our_service_3)
static uint32_t our_char_add(ble_os_t * p_our_service, ble_uuid128_t service_uuid_128, uint16_t char_uuid_16, uint8_t length, uint8_t *value)
{
			uint32_t            err_code;
			ble_uuid_t          char_uuid;
			ble_uuid128_t       base_uuid = service_uuid_128;
			char_uuid.uuid      = char_uuid_16;
			err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
			APP_ERROR_CHECK(err_code); 
    
    // OUR_JOB: Step 2.F Add read/write properties to our characteristic
		
			ble_gatts_char_md_t char_md;
			memset(&char_md, 0, sizeof(char_md));
			char_md.char_props.read = 1;
			char_md.char_props.write = 1;

    // OUR_JOB: Step 2.B, Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md)); 
		attr_md.vloc = BLE_GATTS_VLOC_STACK;
		
    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);		
    // OUR_JOB: Step 2.C, Configure the characteristic value attribute
		ble_gatts_attr_t    attr_char_value;
		memset(&attr_char_value, 0, sizeof(attr_char_value));    
		attr_char_value.p_uuid      = &char_uuid;
		attr_char_value.p_attr_md   = &attr_md;

    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes

		attr_char_value.max_len = length;
		attr_char_value.init_len = length;
		attr_char_value.p_value = value;    

		err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
																			 &char_md,
																			 &attr_char_value,
																			 &p_our_service->char_handles);
		APP_ERROR_CHECK(err_code);																										

    return NRF_SUCCESS;
}


/**@brief Function for initiating our new service.
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
void our_service_init(ble_os_t * p_our_service, ble_uuid128_t service_uuid_128, uint16_t service_uuid16, uint16_t char_uuid16, uint8_t length, uint8_t *value )
{
    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions

    // ble+: Khai bao service 16 bit va 128 bit UUID them chung vao trong ble stack 
		// service on/off
    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = service_uuid_128;
    service_uuid.uuid = service_uuid16;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);    	
    
		p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;
	
		err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_our_service->service_handle);
    APP_ERROR_CHECK(err_code);
    
    // OUR_JOB: Call the function our_char_add() to add our new characteristic to the service. 
		our_char_add(p_our_service,service_uuid_128,char_uuid16,length,value);		
}

// ALREADY_DONE_FOR_YOU: Function to be called when updating characteristic value
void our_temperature_characteristic_update(ble_os_t *p_our_service, uint8_t* curtain_state)
{
	
}
