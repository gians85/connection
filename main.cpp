#include "mbed.h"

/*******************************************************************************
*                                 INCLUDES
*******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include "bluenrg1_api.h"
#include "bluenrg1_events.h"
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "gp_timer.h"
#include "clock.h"
#include "BlueNRG1_sleep.h"
#include "SensorDemo_config.h"
#include "sensor.h"
#include "gatt_db.h"
#ifdef __cplusplus
}
#endif

/*******************************************************************************
*                                 OBJECTS INIT
*******************************************************************************/
Serial serialport(USBTX, USBRX);
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalIn but1(PUSH1);
DigitalIn but2(PUSH2);
SPI spi(SPI_MOSI, SPI_MISO, SPI_SCK, SPI_CS); // mosi, miso, sclk
DigitalOut cs(SPI_CS);

/*******************************************************************************
*                            FUNCTIONS PROTOTYPES
*******************************************************************************/
void boardInit(void);
uint8_t deviceInit(void);
void application(void);
void Set_DeviceConnectable(void);
uint8_t Sensor_DeviceInit(void);
void APP_Tick(void);
//tBleStatus Free_Fall_Notify(void);
//void Read_Request_CB(uint16_t);

/*******************************************************************************
*                                 DEFINES
*******************************************************************************/
#define BLE_SENSOR_VERSION_STRING "1.0.0"
#define UPDATE_CONN_PARAM 0 // Can be set to 1 only when no low power mode is used
#define ADV_INTERVAL_MIN_MS  1000
#define ADV_INTERVAL_MAX_MS  1200
#define SENSOR_EMULATION

/*******************************************************************************
*                                 VARIABLES
*******************************************************************************/

volatile uint8_t set_connectable = 1;
uint16_t connection_handle = 0;
uint8_t connInfo[20];

BOOL sensor_board = FALSE; // It is True if sensor boad has been detected

int connected = FALSE;
#if UPDATE_CONN_PARAM
int l2cap_request_sent = FALSE;
struct timer l2cap_req_timer;
#endif

#define SENSOR_TIMER 1
static uint16_t acceleration_update_rate = 200;
static uint8_t sensorTimer_expired = FALSE;

#ifndef SENSOR_EMULATION
PRESSURE_DrvTypeDef* xLPS25HBDrv = &LPS25HBDrv;
IMU_6AXES_DrvTypeDef *Imu6AxesDrv = NULL;
LSM6DS3_DrvExtTypeDef *Imu6AxesDrvExt = NULL;
static AxesRaw_t acceleration_data;
#endif

volatile uint8_t request_free_fall_notify = FALSE;

/*******************************************************************************
*                                     DEBUG
*******************************************************************************/
#ifndef DEBUG
#define DEBUG 1
#endif
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

uint8_t Services_Max_Attribute_Records[NUMBER_OF_APPLICATION_SERVICES] = {MAX_NUMBER_ATTRIBUTES_RECORDS_SERVICE_1, MAX_NUMBER_ATTRIBUTES_RECORDS_SERVICE_2};


/*******************************************************************************
* Function Name  : MAIN
* Description    : MAIN FUNCTION.
* Input          : None.
* Return         : None.
******************************************************************************/
int main() {
    uint8_t ret;

    /* board init */
    boardInit();

    /* BlueNRG-1 stack init */
    ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
    if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
        while(1);
    }

    PRINTF("BlueNRG-1 BLE Sensor Demo Application (version: %s)\r\n",
           BLE_SENSOR_VERSION_STRING);

    /* Device Init */
    //ret = deviceInit();
    ret = Sensor_DeviceInit();
    if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("Error in Device Initialization() 0x%02x\r\n", ret);
        while(1);
    }else{
        PRINTF("BLE_STATUS_SUCCESS\r\n");
    }


    while(1){
        /* BLE Stack Tick */
        BTLE_StackTick();

        /* Application Tick: user application where application state machine
        is handled */
        APP_Tick();

        /* Power Save management: enable sleep mode with wakeup on radio
        operating timings (adverting, connections intervals) */
        //BlueNRG_Sleep(SLEEPMODE_WAKETIMER, 0, 0);
        sleep();

    }/* end while (1) */

} /* end main() */





/*******************************************************************************
* Function Name  : boardInit
* Description    : Init the board.
* Input          : None.
* Return         : None.
******************************************************************************/
void boardInit(){
	// Buttons
	but1.mode(NoPull);
	but2.mode(NoPull);
	// SPI Interface
	cs=1;
	spi.format(8, 0);
	spi.frequency(100000);
	// Ready
	led2 = 1;
}


/******************************************************************************/
/*                 BlueNRG-1 Peripherals Interrupt Handlers                   */
/******************************************************************************/
extern "C" {
    void Blue_Handler(void){
        // Call RAL_Isr
        RAL_Isr();
    }

    void SysTick_Handler(void){
        SysCount_Handler();
    }
}


/*************************************************************************************************************************/
/*                 sensors.c                                                                                             */
/*************************************************************************************************************************/

/*******************************************************************************
 * Function Name  : Sensor_DeviceInit.
 * Description    : Init the device sensors.
 * Input          : None.
 * Return         : Status.
 *******************************************************************************/
uint8_t Sensor_DeviceInit()
{
    PRINTF("Sensor_DeviceInit()\r\n");
    uint8_t bdaddr[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x02};
    uint8_t ret;
    uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
    uint8_t device_name[] = {'B', 'l', 'u', 'e', 'N', 'R', 'G'};

    /* Set the device public address */
    ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN,
                                    bdaddr);
    if(ret != BLE_STATUS_SUCCESS) {
        PRINTF("aci_hal_write_config_data() failed: 0x%02x\r\n", ret);
        return ret;
    }

    /* Set the TX power -2 dBm */
    aci_hal_set_tx_power_level(1, 4);

    /* GATT Init */
    ret = aci_gatt_init();
    if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("aci_gatt_init() failed: 0x%02x\r\n", ret);
        return ret;
    }

    /* GAP Init */
    ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
    if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("aci_gap_init() failed: 0x%02x\r\n", ret);
        return ret;
    }

    /* Update device name */
    ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, sizeof(device_name), device_name);
    if(ret != BLE_STATUS_SUCCESS) {
        PRINTF("aci_gatt_update_char_value() failed: 0x%02x\r\n", ret);
        return ret;
    }

    /* BLE Security v4.2 is supported: BLE stack FW version >= 2.x (new API prototype) */
    ret = aci_gap_set_authentication_requirement(BONDING,
                                                 MITM_PROTECTION_REQUIRED,
                                                 SC_IS_SUPPORTED,
                                                 KEYPRESS_IS_NOT_SUPPORTED,
                                                 7,
                                                 16,
                                                 USE_FIXED_PIN_FOR_PAIRING,
                                                 123456,
                                                 0x00);
    if(ret != BLE_STATUS_SUCCESS) {
        PRINTF("aci_gap_set_authentication_requirement()failed: 0x%02x\r\n", ret);
        return ret;
    }

    PRINTF("BLE Stack Initialized with SUCCESS\r\n");

    /* Add services and Characteristics */

    return BLE_STATUS_SUCCESS;
}

/*******************************************************************************
 * Function Name  : Set_DeviceConnectable.
 * Description    : Puts the device in connectable mode.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Set_DeviceConnectable(void)
{
  uint8_t ret;
  uint8_t local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','N','R','G','m','b','e','d'};

#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
  hci_le_set_scan_response_data(18,BTLServiceUUID4Scan);
#else
  hci_le_set_scan_response_data(0,NULL);
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */
  PRINTF("Set General Discoverable Mode.\r\n");

  ret = aci_gap_set_discoverable(ADV_IND,
                                (ADV_INTERVAL_MIN_MS*1000)/625,(ADV_INTERVAL_MAX_MS*1000)/625,
                                 STATIC_RANDOM_ADDR, NO_WHITE_LIST_USE,
                                 sizeof(local_name), local_name, 0, NULL, 0, 0);
  if(ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("aci_gap_set_discoverable() failed: 0x%02x\r\n",ret);
    led3 = 1;//SdkEvalLedOn(LED3);
  }
  else
    PRINTF("aci_gap_set_discoverable() --> SUCCESS\r\n");
}

/*******************************************************************************
 * Function Name  : APP_Tick.
 * Description    : Sensor Demo state machine.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void APP_Tick(void)
{
    /* Make the device discoverable */
    if(set_connectable) {
        Set_DeviceConnectable();
        set_connectable = FALSE;
    }

#if UPDATE_CONN_PARAM
    /* Connection parameter update request */
    if(connected && !l2cap_request_sent && Timer_Expired(&l2cap_req_timer)){
        ret = aci_l2cap_connection_parameter_update_req(connection_handle, 9, 20, 0, 600); //24, 24
        l2cap_request_sent = TRUE;
    }
#endif

    /*Update sensor value */

    /* Free fall notification */
    if(request_free_fall_notify == TRUE) {
        request_free_fall_notify = FALSE;
        Free_Fall_Notify();
    }
}

/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/*******************************************************************************
* Function Name  : hci_le_connection_complete_event.
* Description    : This event indicates that a new connection has been created.
* Input          : See file bluenrg1_events.h
* Output         : See file bluenrg1_events.h
* Return         : See file bluenrg1_events.h
*******************************************************************************/
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)
{
    PRINTF("hci_le_connection_complete_event()\r\n");

    connected = TRUE;
    connection_handle = Connection_Handle;

#if UPDATE_CONN_PARAM
    l2cap_request_sent = FALSE;
    Timer_Set(&l2cap_req_timer, CLOCK_SECOND*2);
#endif

}/* end hci_le_connection_complete_event() */

/*******************************************************************************
* Function Name  : hci_disconnection_complete_event.
* Description    : This event occurs when a connection is terminated.
* Input          : See file bluenrg1_events.h
* Output         : See file bluenrg1_events.h
* Return         : See file bluenrg1_events.h
*******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
    PRINTF("hci_disconnection_complete_event()\n\r");
    connected = FALSE;
    /* Make the device connectable again. */
    set_connectable = TRUE;
    connection_handle =0;

    led1=1;//SdkEvalLedOn(LED1);//activity led
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
    OTA_terminate_connection();
#endif
}/* end hci_disconnection_complete_event() */


/*******************************************************************************
* Function Name  : aci_gatt_read_permit_req_event.
* Description    : This event is given when a read request is received
*                  by the server from the client.
* Input          : See file bluenrg1_events.h
* Output         : See file bluenrg1_events.h
* Return         : See file bluenrg1_events.h
*******************************************************************************/
void aci_gatt_read_permit_req_event(uint16_t Connection_Handle,
                                    uint16_t Attribute_Handle,
                                    uint16_t Offset)
{
    Read_Request_CB(Attribute_Handle);
}

/*******************************************************************************
* Function Name  : HAL_VTimerTimeoutCallback.
* Description    : This function will be called on the expiry of
*                  a one-shot virtual timer.
* Input          : See file bluenrg1_stack.h
* Output         : See file bluenrg1_stack.h
* Return         : See file bluenrg1_stack.h
*******************************************************************************/
void HAL_VTimerTimeoutCallback(uint8_t timerNum)
{
    if (timerNum == SENSOR_TIMER) {
        sensorTimer_expired = TRUE;
    }
}

/*******************************************************************************
* Function Name  : aci_gatt_attribute_modified_event.
* Description    : This event occurs when an attribute is modified.
* Input          : See file bluenrg1_events.h
* Output         : See file bluenrg1_events.h
* Return         : See file bluenrg1_events.h
*******************************************************************************/
void aci_gatt_attribute_modified_event(uint16_t Connection_Handle,
                                       uint16_t Attr_Handle,
                                       uint16_t Offset,
                                       uint16_t Attr_Data_Length,
                                       uint8_t Attr_Data[])
{
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
    OTA_Write_Request_CB(Connection_Handle, Attr_Handle, Attr_Data_Length, Attr_Data);
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */
}


void aci_hal_end_of_radio_activity_event(uint8_t Last_State,
                                         uint8_t Next_State,
                                         uint32_t Next_State_SysTime)
{
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
    if (Next_State == 0x02) /* 0x02: Connection event slave */
    {
        OTA_Radio_Activity(Next_State_SysTime);
    }
#endif
}




