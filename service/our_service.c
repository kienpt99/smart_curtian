
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
		NRF_LOG_INFO("Da vao ham da ghi");
    if (   (p_evt_write->handle == p_our_service->char_handles.value_handle)
        && (p_evt_write->len == 1)
        && (p_our_service->curtain_write_handler != NULL))
    {
				NRF_LOG_INFO("Da vao trong cau lenh if");
        p_our_service->curtain_write_handler(p_ble_evt->evt.gap_evt.conn_handle, p_our_service, p_evt_write->data[0]);
    }
				NRF_LOG_INFO("Da ra khoi ham da ghi");
}

// ALREADY_DONE_FOR_YOU: Declaration of a function that will take care of some housekeeping of ble connections related to our service and characteristic

//void ble_our_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
//{
//  	ble_os_t * p_our_service =(ble_os_t *) p_context;  
//		// OUR_JOB: Step 3.D Implement switch case handling BLE events related to our service. 
//		switch(p_ble_evt->header.evt_id)
//		{
//			case BLE_GAP_EVT_CONNECTED:
//				p_our_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
//			break;
//			case BLE_GAP_EVT_DISCONNECTED:
//				p_our_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;			
//			break;
//			case BLE_GATTS_EVT_WRITE:
//				da_ghi(p_our_service, p_ble_evt); // mode 2
//			break;
//			default:
//				break;
//		}
//}

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
				// do nothing
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
			// value = 0 => Rem mo
			// value = 1 => Rem dong 
//			uint8_t value = 0;
			attr_char_value.p_value = value;
				
//		attr_char_value_1.max_len = 1;
//		attr_char_value_1.init_len = 1;
//		// value = 0 => Rem mo
//		// value = 1 => Rem dong 
//		uint8_t value_1 = 0;
//		attr_char_value_1.p_value = &value_1;
		
//		attr_char_value_2.max_len = 5;
//		attr_char_value_2.init_len = 5;
//		// value = ngay/thang/nam/gio/phut
//		uint8_t value_2[5] = {0x00,0x00,0x00,0x00,0x00};
//		attr_char_value_2.p_value = value_2;
		
//		attr_char_value_3.max_len = 1;
//		attr_char_value_3.init_len = 1;
//		// value = 1 => mode 1 
//		// value = 2 => mode 2  
//		uint8_t value_3 = 2;
//		attr_char_value_3.p_value = &value_3;


    // OUR_JOB: Step 2.E, Add our new characteristic to the service
//		err_code = sd_ble_gatts_characteristic_add(p_our_service_1->service_handle,
//																							 &char_md_1,
//																							 &attr_char_value_1,
//																							 &p_our_service_1->char_handles);
//		APP_ERROR_CHECK(err_code);

			err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
																				 &char_md,
																				 &attr_char_value,
																				 &p_our_service->char_handles);
			APP_ERROR_CHECK(err_code);			
																							 
//		err_code = sd_ble_gatts_characteristic_add(p_our_service_2->service_handle,
//																							 &char_md_2,
//																							 &attr_char_value_2,
//																							 &p_our_service_2->char_handles);
//		APP_ERROR_CHECK(err_code);

//		err_code = sd_ble_gatts_characteristic_add(p_our_service_3->service_handle,
//																							 &char_md_3,
//																							 &attr_char_value_3,
//																							 &p_our_service_3->char_handles);
//		APP_ERROR_CHECK(err_code);


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

		// service hen gio
//    ble_uuid_t        tmsv_uuid;
//    ble_uuid128_t     tmsv_base_uuid = BLE_UUID_TMSV_BASE_UUID;
//    tmsv_uuid.uuid = BLE_UUID_TM_SERVICE_UUID;
//    err_code = sd_ble_uuid_vs_add(&tmsv_base_uuid, &tmsv_uuid.type);
//    APP_ERROR_CHECK(err_code);    

		// service chuyen mode 
//    ble_uuid_t        mdsl_uuid;
//    ble_uuid128_t     mdsl_base_uuid = BLE_UUID_MDSL_BASE_UUID;
//    mdsl_uuid.uuid = BLE_UUID_MDSL_SERVICE_UUID;
//    err_code = sd_ble_uuid_vs_add(&mdsl_base_uuid, &mdsl_uuid.type);
//    APP_ERROR_CHECK(err_code);    	
    
		p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;
    // ble+: khoi tao trang thai ket noi ban dau la invalid 
//		p_our_service_1->conn_handle = BLE_CONN_HANDLE_INVALID;
//		p_our_service_2->conn_handle = BLE_CONN_HANDLE_INVALID;
//		p_our_service_3->conn_handle = BLE_CONN_HANDLE_INVALID;

		// ble+ : them service
		// service on/off
//		err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
//                                        &service_uuid,
//                                        &p_our_service_1->service_handle);
//    
//    APP_ERROR_CHECK(err_code);	
		err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_our_service->service_handle);
    APP_ERROR_CHECK(err_code);
		
		// service hen gio
//		err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
//                                        &tmsv_uuid,
//                                        &p_our_service_2->service_handle);
//    
//    APP_ERROR_CHECK(err_code);
		
		// service chuyen mode 
//		err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
//                                        &mdsl_uuid,
//                                        &p_our_service_3->service_handle);
//    
//    APP_ERROR_CHECK(err_code);
    
    // OUR_JOB: Call the function our_char_add() to add our new characteristic to the service. 
//    our_char_add(p_our_service_1,p_our_service_2,p_our_service_3);
			our_char_add(p_our_service,service_uuid_128,char_uuid16,length,value);		
}

// ALREADY_DONE_FOR_YOU: Function to be called when updating characteristic value
void our_temperature_characteristic_update(ble_os_t *p_our_service, uint8_t* curtain_state)
{
	
}
