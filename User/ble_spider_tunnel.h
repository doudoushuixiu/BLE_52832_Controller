/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_srv_hrs Heart Rate Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Heart Rate Service module.
 *
 * @details This module implements the Heart Rate Service with the Heart Rate Measurement,
 *          Body Sensor Location and Heart Rate Control Point characteristics.
 *          During initialization it adds the Heart Rate Service and Heart Rate Measurement
 *          characteristic to the BLE stack database. Optionally it also adds the
 *          Body Sensor Location and Heart Rate Control Point characteristics.
 *
 *          If enabled, notification of the Heart Rate Measurement characteristic is performed
 *          when the application calls ble_hrs_heart_rate_measurement_send().
 *
 *          The Heart Rate Service also provides a set of functions for manipulating the
 *          various fields in the Heart Rate Measurement characteristic, as well as setting
 *          the Body Sensor Location characteristic value.
 *
 *          If an event handler is supplied by the application, the Heart Rate Service will
 *          generate Heart Rate Service events to the application.
 *
 * @note The application must propagate BLE stack events to the Heart Rate Service module by calling
 *       ble_hrs_on_ble_evt() from the @ref softdevice_handler callback.
 *
 * @note Attention! 
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile 
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef BLE_HRS_H__
#define BLE_HRS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define BLE_UUID_SPIDER_TUNNEL_SERVICE            0xFFE0                       /**< The UUID of the Nordic UART Service. */
#define BLE_UUID_SPIDER_TUNNEL_CHARACTERISTIC     0xFFE1                       /**< The UUID of the TX Characteristic. */
#define BLE_UUID_PASS_MODE_CHARACTERISTIC         0xFFE2                       /**< The UUID of the TX Characteristic. */
#define MY_SERVICE_PASSMODE_SPIDER_TUNNEL         0
#define MY_SERVICE_PASSMODE_STM32_ISP_UPDATE      1
#define MY_SERVICE_PASSMODE_PASS_THROUGH          2


/**@brief Heart Rate Service event type. */
typedef enum
{
    BLE_HRS_EVT_NOTIFICATION_ENABLED,                   /**< Heart Rate value notification enabled event. */
    BLE_HRS_EVT_NOTIFICATION_DISABLED                   /**< Heart Rate value notification disabled event. */
} ble_hrs_evt_type_t;

/**@brief Heart Rate Service event. */
typedef struct
{
    ble_hrs_evt_type_t evt_type;                        /**< Type of event. */
} ble_hrs_evt_t;

// Forward declaration of the ble_hrs_t type. 
typedef struct ble_hrs_s ble_hrs_t;

/**@brief Heart Rate Service event handler type. */
typedef void (*ble_hrs_evt_handler_t) (ble_hrs_t * p_hrs, ble_hrs_evt_t * p_evt);


//UART
typedef void (*ble_nus_data_handler_t) (ble_hrs_t * p_hrs, uint8_t * p_data, uint16_t length);





/**@brief Heart Rate Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    ble_hrs_evt_handler_t        evt_handler;
    ble_nus_data_handler_t       data_handler;	        /**< Event handler to be called for handling events in the Heart Rate Service. */

  	ble_srv_cccd_security_mode_t hrs_hrm_attr_md;                                      /**< Initial security level for heart rate service measurement attribute */
} ble_hrs_init_t;

/**@brief Heart Rate Service structure. This contains various status information for the service. */
struct ble_hrs_s
{
    ble_hrs_evt_handler_t        evt_handler;                                          /**< Event handler to be called for handling events in the Heart Rate Service. */
    ble_nus_data_handler_t       data_handler;
  	
    uint16_t                     service_handle;                                       /**< Handle of Heart Rate Service (as provided by the BLE stack). */
//    ble_gatts_char_handles_t     hrm_handles;                                          /**< Handles related to the Heart Rate Measurement characteristic. */
    ble_gatts_char_handles_t     tunnel_handles;
    ble_gatts_char_handles_t     pass_mode_handles;
	  bool                         is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
   
   	uint16_t                     conn_handle;                                          /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
};

/**@brief Function for initializing the Heart Rate Service.
 *
 * @param[out]  p_hrs       Heart Rate Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_hrs_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_spider_tunnel_init(ble_hrs_t * p_hrs, const ble_hrs_init_t * p_hrs_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Heart Rate Service.
 *
 * @param[in]   p_hrs      Heart Rate Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_hrs_on_ble_evt(ble_hrs_t * p_hrs, ble_evt_t * p_ble_evt);

/**@brief Function for sending heart rate measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed a heart rate measurement.
 *          If notification has been enabled, the heart rate measurement data is encoded and sent to
 *          the client.
 *
 * @param[in]   p_hrs                    Heart Rate Service structure.
 * @param[in]   heart_rate               New heart rate measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_hrs_heart_rate_measurement_send(ble_hrs_t * p_hrs, uint8_t * p_string, uint16_t length);

/**@brief Function for adding a RR Interval measurement to the RR Interval buffer.
 *
 * @details All buffered RR Interval measurements will be included in the next heart rate
 *          measurement message, up to the maximum number of measurements that will fit into the
 *          message. If the buffer is full, the oldest measurement in the buffer will be deleted.
 *
 * @param[in]   p_hrs        Heart Rate Service structure.
 * @param[in]   rr_interval  New RR Interval measurement (will be buffered until the next
 *                           transmission of Heart Rate Measurement).
 */
//void ble_hrs_rr_interval_add(ble_hrs_t * p_hrs, uint16_t rr_interval);

/**@brief Function for checking if RR Interval buffer is full.
 *
 * @param[in]   p_hrs        Heart Rate Service structure.
 *
 * @return      true if RR Interval buffer is full, false otherwise.
 */
//bool ble_hrs_rr_interval_buffer_is_full(ble_hrs_t * p_hrs);

uint32_t ble_nus_string_send(ble_hrs_t * p_nus, uint8_t * p_string, uint16_t length);


/**@brief Function for setting the state of the Sensor Contact Supported bit.
 *
 * @param[in]   p_hrs                        Heart Rate Service structure.
 * @param[in]   is_sensor_contact_supported  New state of the Sensor Contact Supported bit.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
//uint32_t ble_hrs_sensor_contact_supported_set(ble_hrs_t * p_hrs, bool is_sensor_contact_supported);

/**@brief Function for setting the state of the Sensor Contact Detected bit.
 *
 * @param[in]   p_hrs                        Heart Rate Service structure.
 * @param[in]   is_sensor_contact_detected   TRUE if sensor contact is detected, FALSE otherwise.
 */
//void ble_hrs_sensor_contact_detected_update(ble_hrs_t * p_hrs, bool is_sensor_contact_detected);

/**@brief Function for setting the Body Sensor Location.
 *
 * @details Sets a new value of the Body Sensor Location characteristic. The new value will be sent
 *          to the client the next time the client reads the Body Sensor Location characteristic.
 *
 * @param[in]   p_hrs                 Heart Rate Service structure.
 * @param[in]   body_sensor_location  New Body Sensor Location.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
//uint32_t ble_hrs_body_sensor_location_set(ble_hrs_t * p_hrs, uint8_t body_sensor_location);

#endif // BLE_HRS_H__

/** @} */

