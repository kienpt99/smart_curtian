
#ifndef OUR_SERVICE_H__
#define OUR_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"

// FROM_SERVICE_TUTORIAL: Defining 16-bit service and 128-bit base UUIDs
static const ble_uuid128_t BLE_UUID_OUR_BASE_UUID = {{0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}}; // 128-bit base UUID
static const uint16_t BLE_UUID_OUR_SERVICE_UUID = 0xF00D; // Just a random, but recognizable value

static const ble_uuid128_t BLE_UUID_TMSV_BASE_UUID = {{0x2A, 0x6C, 0x41, 0x78, 0x2B, 0x3E, 0x38, 0xAC, 0x2C, 0x4E, 0xD6, 0x94, 0x00, 0x00, 0xC2, 0x7E}}; // 128-bit base UUID
static const uint16_t BLE_UUID_TM_SERVICE_UUID = 0x893E; // Just a random, but recognizable value

static const ble_uuid128_t BLE_UUID_MDSL_BASE_UUID = {{0xC2, 0xB2, 0x44, 0x0C, 0xC9, 0x72, 0xD2, 0x8F, 0x2A, 0x4D, 0x04, 0xF0, 0x00, 0x00, 0x12, 0x69}}; // 128-bit base UUID
static const uint16_t BLE_UUID_MDSL_SERVICE_UUID = 0xBB35; // Just a random, but recognizable value

// ALREADY_DONE_FOR_YOU: Defining 16-bit characteristic UUID
static const uint16_t BLE_UUID_OUR_CHARACTERISTC_UUID = 0xBEEF; // Just a random, but recognizable value
static const uint16_t BLE_UUID_TM_CHARACTERISTC_UUID = 0xABCD; // Just a random, but recognizable value
static const uint16_t BLE_UUID_MDSL_CHARACTERISTC_UUID = 0x1234; // Just a random, but recognizable value

// Forward declaration of the custom_service_t type.
typedef struct ble_os_s ble_os_t;

typedef void (*ble_curtain_service_curtain_write_handler_t) (uint16_t conn_handle, ble_os_t * p_our_service, uint8_t new_state);

// This structure contains various status information for our service. 
// The name is based on the naming convention used in Nordics SDKs. 
// 'ble’ indicates that it is a Bluetooth Low Energy relevant structure and 
// ‘os’ is short for Our Service). 
typedef struct ble_os_s
{
    uint16_t                    conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
    uint16_t                    service_handle; /**< Handle of Our Service (as provided by the BLE stack). */
    // OUR_JOB: Step 2.D, Add handles for the characteristic attributes to our struct
		ble_gatts_char_handles_t 		char_handles;
		ble_curtain_service_curtain_write_handler_t curtain_write_handler;
}ble_os_t;

/**@brief Function for handling BLE Stack events related to our service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to Our Service.
 *
 * @param[in]   p_our_service       Our Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
//void ble_our_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

void ble_our_service_on_ble_evt_1(ble_evt_t const * p_ble_evt, void * p_context);

void ble_our_service_on_ble_evt_2(ble_evt_t const * p_ble_evt, void * p_context);

void ble_our_service_on_ble_evt_3(ble_evt_t const * p_ble_evt, void * p_context);
/**@brief Function for initializing our new service.
 *
 * @param[in]   p_our_service       Pointer to Our Service structure.
 */
//void our_service_init(ble_os_t * p_our_service_1, ble_os_t * p_our_service_2, ble_os_t * p_our_service_3);

void our_service_init(ble_os_t * p_our_service, ble_uuid128_t service_uuid_128, uint16_t service_uuid16, uint16_t char_uuid16, uint8_t length, uint8_t *value);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_our_service                     Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void our_temperature_characteristic_update(ble_os_t *p_our_service, uint8_t* curtain_state);

void da_ghi(ble_os_t * p_our_service, ble_evt_t const * p_ble_evt);

#endif  /* _ OUR_SERVICE_H__ */
