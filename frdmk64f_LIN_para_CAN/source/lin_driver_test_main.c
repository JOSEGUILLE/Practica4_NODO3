/*
 * lin_driver_test_main.c
 * Created on: Sep 15, 2018
 *     Author: Nico
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "fsl_uart_freertos.h"
#include "fsl_uart.h"
#include "fsl_adc16.h"

#include "pin_mux.h"
#include "clock_config.h"
#include <lin1d3_driver.h>
#include "FreeRTOSConfig.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define xJUST_MASTER

/* UART instance and clock */
#define MASTER_UART UART3
#define MASTER_UART_CLKSRC UART3_CLK_SRC
#define MASTER_UART_CLK_FREQ CLOCK_GetFreq(UART3_CLK_SRC)
#define MASTER_UART_RX_TX_IRQn UART3_RX_TX_IRQn

/* UART instance and clock */
#define SLAVE_UART UART4
#define SLAVE_UART_CLKSRC UART4_CLK_SRC
#define SLAVE_UART_CLK_FREQ CLOCK_GetFreq(UART4_CLK_SRC)
#define SLAVE_UART_RX_TX_IRQn UART4_RX_TX_IRQn

/* Task priorities. */
#define init_task_PRIORITY (configMAX_PRIORITIES - 2)
#define test_task_heap_size_d	(192)

#define app_message_id_1_d (0x01<<2|message_size_2_bytes_d)
#define app_message_id_2_d (0x02<<2|message_size_4_bytes_d)
#define app_message_id_3_d (0x03<<2|message_size_8_bytes_d)

//ADC
#define DEMO_ADC16_BASE (ADC0)
#define DEMO_ADC16_CHANNEL_GROUP (0U)
#define DEMO_ADC16_USER_CHANNEL (12U)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void test_task(void *pvParameters);

static void	message_1_callback_master(void* message);
static void	message_2_callback_master(void* message);
static void	message_3_callback_master(void* message);
static void	message_1_callback_slave(void* message);
static void	message_2_callback_slave(void* message);
static void	message_3_callback_slave(void* message);
/*******************************************************************************
 * Variables
 ******************************************************************************/

// ADC
volatile bool g_Adc16ConversionDoneFlag = false;
volatile uint32_t g_Adc16ConversionValue;
static adc16_channel_config_t adc16ChannelConfigStruct;


/*******************************************************************************
 * Code
 ******************************************************************************/
void ADC0_IRQHandler(void)
{
    g_Adc16ConversionDoneFlag = true;
    g_Adc16ConversionValue = ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP);
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void ADC_init(void)
{
    adc16_config_t adc16ConfigStruct;


    NVIC_EnableIRQ(ADC0_IRQn);
    ADC16_GetDefaultConfig(&adc16ConfigStruct);
    ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false); /* Make sure the software trigger is used. */
    (void)ADC16_DoAutoCalibration(DEMO_ADC16_BASE);
    adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true;
    adc16ChannelConfigStruct.enableDifferentialConversion = false;
    ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);

}



/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    NVIC_SetPriority(MASTER_UART_RX_TX_IRQn, 5);
    NVIC_SetPriority(SLAVE_UART_RX_TX_IRQn, 5);
	ADC_init();

    if (xTaskCreate(test_task, "test_task", test_task_heap_size_d, NULL, init_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("Init Task creation failed!.\r\n");
        while (1)
            ;
    }
    PRINTF(" *** LIN driver demo ***\r\n");
    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Task responsible for loopback.
 */
static void test_task(void *pvParameters)
{
	int error;
	lin1d3_nodeConfig_t node_config;
	lin1d3_handle_t* master_handle;
	lin1d3_handle_t* slave_handle;
	/* Set Master Config */
	node_config.type = lin1d3_master_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = MASTER_UART;
	node_config.srcclk = MASTER_UART_CLK_FREQ;
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
	node_config.messageTable[0].ID = app_message_id_1_d;
	node_config.messageTable[0].handler = message_1_callback_master;
	node_config.messageTable[1].ID = app_message_id_2_d;
	node_config.messageTable[1].handler = message_2_callback_master;
	node_config.messageTable[2].ID = app_message_id_3_d;
	node_config.messageTable[2].handler = message_3_callback_master;
	/* Init Master node */
	master_handle = lin1d3_InitNode(node_config);
#if !defined(JUST_MASTER)
	/* Set Slave Config */
	node_config.type = lin1d3_slave_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = SLAVE_UART;
	node_config.srcclk = SLAVE_UART_CLK_FREQ;
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
	node_config.messageTable[0].ID = app_message_id_1_d;
	node_config.messageTable[0].handler = message_1_callback_slave;
	node_config.messageTable[1].ID = app_message_id_2_d;
	node_config.messageTable[1].handler = message_2_callback_slave;
	node_config.messageTable[2].ID = app_message_id_3_d;
	node_config.messageTable[2].handler = message_3_callback_slave;
	/* Init Slave Node*/
	slave_handle = lin1d3_InitNode(node_config);
#endif

	if((NULL == master_handle)
#if !defined(JUST_MASTER)
		|| (NULL == slave_handle)
#endif
	   ){
		error = kStatus_Fail;
	}
	else {
		error = kStatus_Success;
	}

	while (kStatus_Success == error)
    {
    	vTaskDelay(200);
    	lin1d3_masterSendMessage(master_handle, app_message_id_1_d);
    	vTaskDelay(200);
    	lin1d3_masterSendMessage(master_handle, app_message_id_2_d);
    	vTaskDelay(200);
    	lin1d3_masterSendMessage(master_handle, app_message_id_3_d);
    }

    vTaskSuspend(NULL);
}


static void	message_1_callback_master(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Master got response to message 1 %d,%d\r\n", message_data[0], message_data[1]);
}

static void	message_2_callback_master(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Master got response to message 2 %d,%d,%d,%d\r\n", message_data[0], message_data[1], message_data[2], message_data[3]);
}

static void	message_3_callback_master(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Master got response to message 3 %d,%d,%d,%d,%d,%d,%d,%d\r\n",
			message_data[0], message_data[1], message_data[2], message_data[3],
			message_data[4], message_data[5], message_data[6], message_data[7]);
}

static void	message_1_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 1 request\r\n");
	message_data[0] = (g_Adc16ConversionValue)&0xFF;
	message_data[1] = (g_Adc16ConversionValue&0xFF00)>>8;
}

static void	message_2_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 2 request\r\n");
	message_data[0] = 79;
	message_data[1] = 80;
	message_data[2] = 81;
	message_data[3] = 82;
}

static void	message_3_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 3 request\r\n");
	message_data[0] = 79;
	message_data[1] = 80;
	message_data[2] = 81;
	message_data[3] = 82;
	message_data[4] = 83;
	message_data[5] = 84;
	message_data[6] = 85;
	message_data[7] = 86;
}

