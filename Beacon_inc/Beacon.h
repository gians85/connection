#ifndef _BEACON_H_
#define _BEACON_H_


/* Includes ------------------------------------------------------------------*/
#include "mbed.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "bluenrg1_api.h"
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "Beacon_config.h"
#include "clock.h"
#include "BlueNRG1_sleep.h"
#ifdef __cplusplus
}
#endif


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_BEACON_VERSION_STRING "1.1.0"


/* Set to 1 for enabling Flags AD Type position at the beginning
of the advertising packet */
#define ENABLE_FLAGS_AD_TYPE_AT_BEGINNING 1


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Device_Init(void);
static void Start_Beaconing(void);
void beacon(void);
Serial bleserial(USBTX, USBRX);




void beacon (){
    printf("\014");  //CLS
    uint8_t ret;
    /* BLE stack init */
	ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
	if (ret != BLE_STATUS_SUCCESS) {
		printf("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
		while(1);
	}
	/* Init the BlueNRG-1 device */
	Device_Init();

#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
	/* Initialize the button */
	SdkEvalPushButtonInit(BUTTON_1);
#endif /* ST_USE_OTA_SERVICE_MANAGER_APPLICATION */

	/* Start Beacon Non Connectable Mode*/
	Start_Beaconing();

	printf("BlueNRG-1 BLE Beacon Application (version: %s)\r\n", BLE_BEACON_VERSION_STRING);

	while(1)
	{
		/* BlueNRG-1 stack tick */
		BTLE_StackTick();

        //wait_ms(50);

		/* Enable Power Save according the Advertising Interval */
		BlueNRG_Sleep(SLEEPMODE_NOTIMER, 0, 0);

#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
		if (SdkEvalPushButtonGetState(BUTTON_1) == RESET)
		{
			OTA_Jump_To_Service_Manager_Application();
		}
#endif /* ST_USE_OTA_SERVICE_MANAGER_APPLICATION */
	}
}



void Device_Init()
{
	uint8_t ret;
	uint16_t service_handle;
	uint16_t dev_name_char_handle;
	uint16_t appearance_char_handle;

	/* Set the TX Power to -2 dBm */
	ret = aci_hal_set_tx_power_level(1,4);
	if(ret != 0) {
		printf ("Error in aci_hal_set_tx_power_level() 0x%04xr\n", ret);
		while(1);
	}

	/* Init the GATT */
	ret = aci_gatt_init();
	if (ret != 0)
		printf ("Error in aci_gatt_init() 0x%04xr\n", ret);
	else
		printf ("aci_gatt_init() --> SUCCESS\r\n");

	/* Init the GAP */
	ret = aci_gap_init(0x01, 0x00, 0x08, &service_handle, &dev_name_char_handle, &appearance_char_handle);
	if (ret != 0)
		printf ("Error in aci_gap_init() 0x%04x\r\n", ret);
	else
		printf ("aci_gap_init() --> SUCCESS\r\n");
}



static void Start_Beaconing()
{
	uint8_t ret = BLE_STATUS_SUCCESS;

#if ENABLE_FLAGS_AD_TYPE_AT_BEGINNING
	/* Set AD Type Flags at beginning on Advertising packet  */
	uint8_t adv_data[] = {
        /* Advertising data: Flags AD Type */
        0x02,
        0x01,
        0x06,
        /* Advertising data: manufacturer specific data */
        26, //len
        AD_TYPE_MANUFACTURER_SPECIFIC_DATA,  //manufacturer type
        0x30, 0x00, //Company identifier code (Default is 0x0030 - STMicroelectronics: To be customized for specific identifier)
        0x02,       // ID
        0x15,       //Length of the remaining payload
        0xE2, 0x0A, 0x39, 0xF4, 0x73, 0xF5, 0x4B, 0xC4, //Location UUID
        0xA1, 0x2F, 0x17, 0xD1, 0xAD, 0x07, 0xA9, 0x61,
        0x00, 0x00, // Major number
        0x00, 0x00, // Minor number
        0xC8        //2's complement of the Tx power (-56dB)};
	};
#else
	uint8_t manuf_data[] = {
        26, //len
        AD_TYPE_MANUFACTURER_SPECIFIC_DATA, //manufacturer type
        0x30, 0x00, //Company identifier code (Default is 0x0030 - STMicroelectronics: To be customized for specific identifier)
        0x02,       // ID
        0x15,       //Length of the remaining payload
        0xE2, 0x0A, 0x39, 0xF4, 0x73, 0xF5, 0x4B, 0xC4, //Location UUID
        0xA1, 0x2F, 0x17, 0xD1, 0xAD, 0x07, 0xA9, 0x61,
        0x00, 0x00, // Major number
        0x00, 0x00, // Minor number
        0xC8        //2's complement of the Tx power (-56dB)};
	};
#endif

	/* disable scan response */
	ret = hci_le_set_scan_response_data(0,NULL);
	if (ret != BLE_STATUS_SUCCESS)
	{
        printf ("Error in hci_le_set_scan_resp_data() 0x%04x\r\n", ret);
		return;
	}
	else
		printf ("hci_le_set_scan_resp_data() --> SUCCESS\r\n");

	/* put device in non connectable mode */
	ret = aci_gap_set_discoverable(ADV_NONCONN_IND, 160, 160, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                   0, NULL, 0, NULL, 0, 0);
	if (ret != BLE_STATUS_SUCCESS)
	{
		printf ("Error in aci_gap_set_discoverable() 0x%04x\r\n", ret);
		return;
	}
	else
		printf ("aci_gap_set_discoverable() --> SUCCESS\r\n");

#if ENABLE_FLAGS_AD_TYPE_AT_BEGINNING
	/* Set the  ADV data with the Flags AD Type at beginning of the
    advertsing packet,  followed by the beacon manufacturer specific data */
	ret = hci_le_set_advertising_data (sizeof(adv_data), adv_data);
	if (ret != BLE_STATUS_SUCCESS)
	{
		printf ("Error in hci_le_set_advertising_data() 0x%04x\r\n", ret);
		return;
	}
	else
		printf ("hci_le_set_advertising_data() --> SUCCESS\r\n");
#else
	/* Delete the TX power level information */
	ret = aci_gap_delete_ad_type(AD_TYPE_TX_POWER_LEVEL);
	if (ret != BLE_STATUS_SUCCESS)
	{
		printf ("Error in aci_gap_delete_ad_type() 0x%04x\r\n", ret);
		return;
	}
	else
		printf ("aci_gap_delete_ad_type() --> SUCCESS\r\n");

	/* Update the ADV data with the BEACON manufacturing data */
	ret = aci_gap_update_adv_data(27, manuf_data);
	if (ret != BLE_STATUS_SUCCESS)
	{
		printf ("Error in aci_gap_update_adv_data() 0x%04x\r\n", ret);
		return;
	}
	else
		printf ("aci_gap_update_adv_data() --> SUCCESS\r\n");
#endif
}


/******************************************************************************/
/*                 BlueNRG-1 Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (system_bluenrg1.c).                                               */
/******************************************************************************/
void Blue_Handler(void)
{
    // Call RAL_Isr
    RAL_Isr();
}

void SysTick_Handler(void)
{
    SysCount_Handler();
}




#endif