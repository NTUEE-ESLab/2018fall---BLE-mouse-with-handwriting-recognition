/*********************************************************************
* INCLUDES
*/

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/display/Display.h>

#include "util.h"
#include <icall.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>
#include <devinfoservice.h>

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC


#include <ti/drivers/I2C.h>
#include <Board.h>
//#include "board.h"
#include "ble_mouse.h"



#define PZ_TASK_PRIORITY                     1

#ifndef PZ_TASK_STACK_SIZE
#define PZ_TASK_STACK_SIZE                   2048
#endif

// Internal Events used by OAD profile
#define PZ_OAD_QUEUE_EVT                     OAD_QUEUE_EVT       // Event_Id_01
#define PZ_OAD_COMPLETE_EVT                  OAD_DL_COMPLETE_EVT // Event_Id_02

// Internal Events for RTOS application
#define PZ_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define PZ_APP_MSG_EVT                       Event_Id_30

// Bitwise OR of all RTOS events to pend on
#define PZ_ALL_EVENTS                        (PZ_ICALL_EVT | \
                                              PZ_APP_MSG_EVT | \
                                              PZ_OAD_QUEUE_EVT | \
                                              PZ_OAD_COMPLETE_EVT)

#define i2c_init_adxl_event              0x0000
#define i2c_adxl_data0_tx_event          0x0001
#define i2c_adxl_data0_rx_event          0x0002
#define i2c_adxl_data1_tx_event          0x0003
#define i2c_adxl_data1_rx_event          0x0004
#define i2c_adxl_interrup_clear_event    0x0005

#define i2c_adxl_dataX_event       0x0100
#define i2c_adxl_dataY_event       0x0200
#define i2c_adxl_dataZ_event       0x0300

#define is_z_axis_init_acceleration_exceed(x)  (x - z_axis_init_acc > 5 \
                                                || z_axis_init_acc - x > 5)

#define adxl_addr            0x53
#define adxl_dataX0_addr     0x32
#define adxl_dataX1_addr     0x33
#define adxl_dataY0_addr     0x34
#define adxl_dataY1_addr     0x35
#define adxl_dataZ0_addr     0x36
#define adxl_dataZ1_addr     0x37


#define button0_event                   0x01
#define button1_event                   0x02
#define both_button_event               0x03
#define wait_release_other_button_event 0x80


#define button_queue_event              0x0000
#define i2c_queue_event                 0x0005
#define i2c_adv_queue_event             0x0006
#define adv_msg_queue_event             0x0007


#define z_axis_init_acc    -223
#define x_axis_init_acc    2



typedef struct
{
	uint16_t event;
	void *pBuf;
} appEventData_t;


/*********************************************************************
* GLOBAL VARIABLES
*/
// Task configuration
Task_Struct pzTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(appTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t appTaskStack[PZ_TASK_STACK_SIZE];

/*********************************************************************
* LOCAL VARIABLES
*/

static PIN_Handle buttonPinHandle;
static PIN_State buttonPinState;

static I2C_Handle i2c_handle;
static I2C_Params i2c_params;
static I2C_Transaction i2cTrans;

static uint8_t i2cTxbuf[2];
static uint8_t i2cRxbuf[2];



typedef struct
{
	uint16_t event;
	uint16_t counter;
}_i2c_event;


static _i2c_event i2c_event = {
	i2c_init_adxl_event,
	0x0000
};



static ICall_EntityID selfEntity;
static ICall_SyncHandle syncEvent;

static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

static uint8 b_advHandleLegacy;
static uint8 i_advHandleLegacy;

PIN_Config buttonPinTable[] = {
	Board_PIN_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
	Board_PIN_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
	Board_DIO22 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,   //ACT
																 //Board_DIO22 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,   //INACT
																 PIN_TERMINATE
};

static Clock_Struct button0DebounceClock;
static Clock_Struct button1DebounceClock;
static Clock_Struct longbuttonClock;
static Clock_Handle button0DebounceClockHandle;
static Clock_Handle button1DebounceClockHandle;
static Clock_Handle longbuttonClockHandle;


volatile static uint8_t button_clock_event = 0x00;
volatile static uint8_t act_intr = 0x00;


typedef struct
{
	uint8_t len;
	uint8_t ad_type;
	uint8_t time;
	uint8_t mouse_type;
	uint8_t data;
}_adv_mouse_data;

static _adv_mouse_data adv_mouse_data =
{
	0x03,
	0xFD,
	0x00,
	0x01,
	0x00
};


static _adv_mouse_data adv_cursor_data =
{
	0x04,
	0xFD,
	0x00,
	0x05,
	0x00
};



volatile static uint8_t left_push = 0x00;
volatile static uint8_t end_i2c = 0x00;
volatile static uint8_t right_push = 0x00;
volatile static uint8_t right_mode = 0x00;


/* Task functions */
static void ProjectZero_init(void);
static void ProjectZero_taskFxn(UArg a0,
	UArg a1);

static void processStackMsg(ICall_Hdr *pMsg);
static void processGapMessage(gapEventHdr_t* pMsg);
static void processAppMsg(appEventData_t* pMsg);
static void i_advCallback(uint32_t event, void *pBuf, uintptr_t arg);
static void b_advCallback(uint32_t event, void *pBuf, uintptr_t arg);
static void enqueue_AppMsg(uint16_t event, void* pBuf);

static void i2ctransferCallback(I2C_Handle handle, I2C_Transaction *transac, bool result);
static void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId);
static void buttonDebounceSwiFxn(UArg buttonId);
static void longbuttonDetectSwiFxn(UArg buttonId);
/*********************************************************************
* EXTERN FUNCTIONS
*/


/*********************************************************************


/*********************************************************************
* PUBLIC FUNCTIONS
*/



/*********************************************************************
* @fn      ProjectZero_createTask
*
* @brief   Task creation function for the Project Zero.
*/
void ProjectZero_createTask(void)
{
	Task_Params taskParams;

	// Configure task
	Task_Params_init(&taskParams);
	taskParams.stack = appTaskStack;
	taskParams.stackSize = PZ_TASK_STACK_SIZE;
	taskParams.priority = PZ_TASK_PRIORITY;

	Task_construct(&pzTask, ProjectZero_taskFxn, &taskParams, NULL);
}

/*********************************************************************
* @fn      ProjectZero_init
*
* @brief   Called during initialization and contains application
*          specific initialization (ie. hardware initialization/setup,
*          table initialization, power up notification, etc), and
*          profile initialization/setup.
*/
static void ProjectZero_init(void)
{

	ICall_registerApp(&selfEntity, &syncEvent);
	appMsgQueueHandle = Util_constructQueue(&appMsgQueue);

	GAP_RegisterForMsgs(selfEntity);
	GAP_DeviceInit(GAP_PROFILE_BROADCASTER, selfEntity, ADDRMODE_PUBLIC, NULL);


	buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
	PIN_registerIntCb(buttonPinHandle, &buttonCallbackFxn);
	button0DebounceClockHandle = Util_constructClock(&button0DebounceClock, buttonDebounceSwiFxn, 30,
		0, 0, Board_PIN_BUTTON0);
	button1DebounceClockHandle = Util_constructClock(&button1DebounceClock, buttonDebounceSwiFxn, 30,
		0, 0, Board_PIN_BUTTON1);
	longbuttonClockHandle = Util_constructClock(&longbuttonClock, longbuttonDetectSwiFxn, 195, 0, 0, 0);

	I2C_init();

	I2C_Params_init(&i2c_params);
	i2c_params.transferMode = I2C_MODE_CALLBACK;
	i2c_params.transferCallbackFxn = i2ctransferCallback;


	i2c_handle = I2C_open(Board_I2C0, &i2c_params);


	i2cTrans.writeCount = 2;
	i2cTrans.writeBuf = i2cTxbuf;
	i2cTrans.readCount = 0;
	i2cTrans.readBuf = i2cRxbuf;
	i2cTrans.slaveAddress = adxl_addr;
	i2cTrans.arg = (void*)&i2c_event;
	i2cTxbuf[0] = 0x31;
	i2cTxbuf[1] = 0x2B;
	I2C_transfer(i2c_handle, &i2cTrans);


	/*
	GapAdv_params_t advParamLegacy = GAPADV_PARAMS_LEGACY_SCANN_CONN;
	advParamLegacy.eventProps = GAP_ADV_PROP_LEGACY;

	GapAdv_create(&advCallback, &advParamLegacy, &advHandleLegacy);
	GapAdv_loadByHandle(advHandleLegacy,GAP_ADV_DATA_TYPE_ADV,sizeof(adv_mouse_data),&adv_mouse_data) ;

	GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX_EVENTS,1);*/


}

/*********************************************************************
* @fn      ProjectZero_taskFxn
*
* @brief   Application task entry point for the Project Zero.
*
* @param   a0, a1 - not used.
*/
static void ProjectZero_taskFxn(UArg a0, UArg a1)
{
	// Initialize application
	ProjectZero_init();

	for (;;)
	{
		uint32_t events;
		events = Event_pend(syncEvent, Event_Id_NONE, ICALL_MSG_EVENT_ID | UTIL_QUEUE_EVENT_ID, ICALL_TIMEOUT_FOREVER);

		if (events)
		{
			ICall_EntityID dest;
			ICall_ServiceEnum src;
			ICall_HciExtEvt *pMsg = NULL;

			if (ICall_fetchServiceMsg(&src, &dest, (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
			{
				if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity)) processStackMsg((ICall_Hdr *)pMsg);

				if (pMsg) ICall_freeMsg(pMsg);
			}
		}
		if (events & UTIL_QUEUE_EVENT_ID)
		{
			while (!Queue_empty(appMsgQueueHandle))
			{
				appEventData_t *pMsg = (appEventData_t*)Util_dequeueMsg(appMsgQueueHandle);
				if (pMsg)
				{
					processAppMsg(pMsg);
					ICall_free(pMsg);
				}
			}
		}
	}

}


static void processStackMsg(ICall_Hdr *pMsg)
{
	switch (pMsg->event)
	{
	case GAP_MSG_EVENT:
		processGapMessage((gapEventHdr_t*)pMsg);
		break;

	default:
		// do nothing
		break;
	}
}

static void processGapMessage(gapEventHdr_t* pMsg)
{
	switch (pMsg->opcode)
	{
	case GAP_DEVICE_INIT_DONE_EVENT:
	{

		gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;
		if (pPkt->hdr.status == SUCCESS)
		{
			GapAdv_params_t advParamLegacy = GAPADV_PARAMS_LEGACY_SCANN_CONN;
			advParamLegacy.eventProps = GAP_ADV_PROP_LEGACY;

			GapAdv_create(&i_advCallback, &advParamLegacy, &i_advHandleLegacy);
			GapAdv_create(&b_advCallback, &advParamLegacy, &b_advHandleLegacy);
			GapAdv_setEventMask(i_advHandleLegacy, GAP_ADV_EVT_MASK_START_AFTER_ENABLE);
			
		}
		break;
	}

	}
}

static void processAppMsg(appEventData_t* pMsg)
{
	switch (pMsg->event)
	{
	case button_queue_event:
		
		GapAdv_loadByHandle(b_advHandleLegacy, GAP_ADV_DATA_TYPE_ADV, 5, &adv_mouse_data);
		GapAdv_enable(b_advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX_EVENTS, 1);
		break;
	case i2c_queue_event:
		I2C_transfer(i2c_handle, &i2cTrans);
		break;

	case i2c_adv_queue_event:
		
		GapAdv_loadByHandle(i_advHandleLegacy, GAP_ADV_DATA_TYPE_ADV, 5, &adv_cursor_data);
		GapAdv_enable(i_advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX_EVENTS, 1);
		break;
	case adv_msg_queue_event:
		if (*(uint32_t*)(pMsg->pBuf) == GAP_EVT_ADV_START_AFTER_ENABLE)
		{
			
			i2cTrans.readCount = 0;
			i2cTrans.writeCount = 1;
			I2C_transfer(i2c_handle, &i2cTrans);

		}
		ICall_free(pMsg->pBuf);

		break;
	}
}

static void b_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{

}

static void i_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{
	uint8_t* m_event = ICall_malloc(sizeof(uint32_t));
	*(uint32_t*)m_event = event;
	enqueue_AppMsg(adv_msg_queue_event, m_event);
}

static void enqueue_AppMsg(uint16_t event, void* pBuf)
{
	appEventData_t* msg = ICall_malloc(sizeof(appEventData_t));
	msg->event = event;
	msg->pBuf = pBuf;
	Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t *)msg);
}



static void longbuttonDetectSwiFxn(UArg uarg)
{
	adv_mouse_data.data = 0x03;
	enqueue_AppMsg(button_queue_event, NULL);
	if (left_push  && !right_mode) end_i2c = 0x01;
	else if (right_push) right_mode ^= 0x01;

	
}





inline uint8_t acc_level(short acc)
{
	uint8_t level = 0;
	if (acc < 0)
	{
		level |= 0x40;
		acc = (-acc);
	}
	if (acc < 90) return 0;
	else if (acc < 140) return (level | 0x01);
	else if (acc < 160) return (level | 0x02);
	else if (acc < 180) return (level | 0x03);
	else if (acc < 200) return (level | 0x04);
	else if (acc < 220) return (level | 0x05);
	else return (level | 0x06);
}



static void i2ctransferCallback(I2C_Handle handle, I2C_Transaction *transac, bool result)
{

	_i2c_event* et = (_i2c_event*)(transac->arg);
	static short x_up_acc = 0, x_down_acc = 0, y_up_acc = 0, y_down_acc = 0;
	static uint32_t x_count = 0, y_count = 0;
	static uint8_t last_x_level = 0, last_y_level = 0;
	static uint8_t x_zero = 0, y_zero = 0;
	if (end_i2c) return;
	if (result)
	{

		switch ((et->event) & 0x00FF)
		{
		case i2c_init_adxl_event:
			if (et->counter != 7)
			{

				if (et->counter == 0)
				{
					i2cTxbuf[0] = 0x24;
					i2cTxbuf[1] = 0x06;
				}
				else if (et->counter == 1)
				{
					i2cTxbuf[0] = 0x25;
					i2cTxbuf[1] = 0x06;
				}
				else if (et->counter == 2)
				{
					i2cTxbuf[0] = 0x26;
					i2cTxbuf[1] = 0x01;
				}
				else if (et->counter == 3)
				{
					i2cTxbuf[0] = 0x27;
					i2cTxbuf[1] = 0x66;
				}
				else if (et->counter == 4)
				{
					i2cTxbuf[0] = 0x2F;
					i2cTxbuf[1] = 0x10;//activity to INT2
				}
				else if (et->counter == 5)
				{
					i2cTxbuf[0] = 0x2E;
					i2cTxbuf[1] = 0x10;//0x18;//set interrupt of activity
				}
				else
				{
					i2cTxbuf[0] = 0x2D;
					i2cTxbuf[1] = 0x08;
					et->event = 0xFFFF;
					et->counter = 0;
					enqueue_AppMsg(i2c_queue_event, NULL);
					return;
				}
				et->counter += 1;
				enqueue_AppMsg(i2c_queue_event, NULL);
			}
			break;
		case i2c_adxl_data0_tx_event:
		{
			et->event &= 0xFF00;
			short acc = (i2cRxbuf[1] << 8) + i2cRxbuf[0];

			if (et->event == i2c_adxl_dataX_event)
			{
				et->event = i2c_adxl_data0_rx_event | i2c_adxl_dataY_event;
				i2cTxbuf[0] = adxl_dataY0_addr;
				if (x_count)
				{
					if (x_count < 10)
					{
						if (acc > x_down_acc && acc < x_up_acc)
						{
							if (++x_count == 10)
							{
								short le = x_down_acc + 6;
								if (acc > 0) le -= 10;
								last_x_level = acc_level(le);
								adv_cursor_data.data = last_x_level;
								enqueue_AppMsg(i2c_adv_queue_event, NULL);
								return;

							}
						}
						else
						{
							x_zero = 1;
							x_count = 0;
						}
					}
					else
					{
						if (acc > 0) acc -= 10;
						uint8_t al = acc_level(acc);
						if (al != last_x_level)
						{
							if ((et->counter & 0x0001) == 0x0000) et->counter |= 0x0001;
							else
							{
								x_count = 0;
								if (al == 0)
								{
									x_zero = 1;
									adv_cursor_data.data = 0;
									enqueue_AppMsg(i2c_adv_queue_event, NULL);
									return;
								}

							}
						}
						else if ((++x_count) & 0x0000000F) et->counter &= 0xFFFE;

					}



				}
				else if (acc > 100 || acc < -90)
				{
					x_zero = 0;
					x_count = 1;
					et->counter &= 0xFFFD;
					x_up_acc = acc + 6;
					x_down_acc = acc - 6;
				}
				else if (x_zero)
				{
					adv_cursor_data.data = 0;
					if (++x_zero == 0x0A)
					{
						x_zero = 0;
						if (y_zero == 0 && y_count == 0)
						{
							et->event = i2c_adxl_interrup_clear_event;
							i2cTxbuf[0] = 0x30;
						}
					}
					enqueue_AppMsg(i2c_adv_queue_event, NULL);
					return;
				}

			}
			else
			{
				et->event = i2c_adxl_data0_rx_event | i2c_adxl_dataX_event;
				i2cTxbuf[0] = adxl_dataX0_addr;

				if (y_count)
				{
					if (y_count < 10)
					{
						if (acc > y_down_acc && acc < y_up_acc)
						{
							if (++y_count == 10)
							{
								last_y_level = acc_level(y_down_acc + 6);
								adv_cursor_data.data = (last_y_level | 0x80);
								enqueue_AppMsg(i2c_adv_queue_event, NULL);
								return;

							}
						}
						else
						{
							y_zero = 1;
							y_count = 0;
						}
					}
					else
					{
						uint8_t al = acc_level(acc);
						if (al != last_y_level)
						{
							if ((et->counter & 0x0100) == 0x0000) et->counter |= 0x0100;
							else
							{
								y_count = 0;
								if (al == 0)
								{
									y_zero = 1;
									adv_cursor_data.data = 0x80;
									enqueue_AppMsg(i2c_adv_queue_event, NULL);
									return;
								}

							}
						}
						else if ((++y_count) & 0x0000000F) et->counter &= 0xFEFF;

					}


				}
				else if (acc > 90 || acc < -90)
				{
					y_zero = 0;
					y_count = 1;
					et->counter &= 0xFDFF;
					y_up_acc = acc + 6;
					y_down_acc = acc - 6;
				}
				else if (y_zero)
				{
					adv_cursor_data.data = 0x80;
					if (++y_zero == 0x0A)
					{
						y_zero = 0;
						if (x_zero == 0 && x_count == 0)
						{
							et->event = i2c_adxl_interrup_clear_event;
							i2cTxbuf[0] = 0x30;
						}
					}
					enqueue_AppMsg(i2c_adv_queue_event, NULL);
					return;
				}


			}
			i2cTrans.readCount = 0;
			i2cTrans.writeCount = 1;
			enqueue_AppMsg(i2c_queue_event, NULL);
			break;
		}
		case i2c_adxl_data0_rx_event:
			et->event &= 0xFF00;
			et->event |= i2c_adxl_data1_tx_event;
			i2cTrans.readCount = 1;
			i2cTrans.writeCount = 0;
			i2cTrans.readBuf = i2cRxbuf;
			enqueue_AppMsg(i2c_queue_event, NULL);
			break;
		case i2c_adxl_data1_tx_event:
			et->event &= 0xFF00;
			if (et->event == i2c_adxl_dataX_event) i2cTxbuf[0] = adxl_dataX1_addr;
			else if (et->event == i2c_adxl_dataY_event) i2cTxbuf[0] = adxl_dataY1_addr;
			et->event |= i2c_adxl_data1_rx_event;
			i2cTrans.readCount = 0;
			i2cTrans.writeCount = 1;
			enqueue_AppMsg(i2c_queue_event, NULL);
			break;
		case i2c_adxl_data1_rx_event:
			et->event &= 0xFF00;
			et->event |= i2c_adxl_data0_tx_event;
			i2cTrans.readCount = 1;
			i2cTrans.writeCount = 0;
			i2cTrans.readBuf = i2cRxbuf + 1;
			enqueue_AppMsg(i2c_queue_event, NULL);
			break;
		case i2c_adxl_interrup_clear_event:
			if (i2cTrans.writeCount)
			{
				i2cTrans.writeCount = 0;
				i2cTrans.readCount = 1;
				i2cTrans.readBuf = i2cRxbuf;
				enqueue_AppMsg(i2c_queue_event, NULL);
			}
			else
			{
				et->event = 0xFFFF;
				et->counter = 0;
				act_intr = 0x00;
			}
			break;
		}
	}


}



static void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId)
{
	if (pinId == Board_DIO22)//ACT
	{
		if (act_intr) return;
		act_intr = 0x01;
		//PIN_setConfig(handle, PIN_BM_IRQ, pinId | PIN_IRQ_DIS);
		i2c_event.event = i2c_adxl_data0_rx_event | i2c_adxl_dataX_event;
		i2cTrans.writeCount = 1;
		i2cTrans.readCount = 0;
		i2cTxbuf[0] = adxl_dataX0_addr;
		enqueue_AppMsg(i2c_queue_event, NULL);
		return;
	}/*
	 if (pinId == Board_DIO22)
	 {
	 i2c_event.event = 0xFFFF;
	 i2c_event.counter = 0;
	 i2cTrans.readCount = 0;
	 PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, Board_DIO21 | PIN_IRQ_NEGEDGE);
	 return;
	 }*/
	PIN_setConfig(handle, PIN_BM_IRQ, pinId | PIN_IRQ_DIS);
	if (button_clock_event == 0x00)
	{
		if (pinId == Board_PIN_BUTTON1)
		{
			if (Util_isActive(&button0DebounceClock))
			{
				if (PIN_getInputValue(Board_PIN_BUTTON1)) button_clock_event |= wait_release_other_button_event;
				else
				{
					PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, Board_PIN_BUTTON1 | PIN_IRQ_POSEDGE);
					button_clock_event |= button1_event;
				}
				return;
			}
		}
		else
		{
			if (Util_isActive(&button1DebounceClock))
			{
				if (PIN_getInputValue(Board_PIN_BUTTON0)) button_clock_event |= wait_release_other_button_event;
				else
				{
					PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, Board_PIN_BUTTON0 | PIN_IRQ_POSEDGE);
					button_clock_event |= button0_event;
				}
				return;
			}
		}
	}
	else if ((pinId == Board_PIN_BUTTON1 && (button_clock_event == button0_event)) || (pinId == Board_PIN_BUTTON0 && (button_clock_event == button1_event)))
	{
		PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, pinId | PIN_IRQ_NEGEDGE);
		return;
	}

	switch (pinId)
	{
	case Board_PIN_BUTTON0:
		Util_startClock((Clock_Struct *)button0DebounceClockHandle);
		break;
	case Board_PIN_BUTTON1:
		Util_startClock((Clock_Struct *)button1DebounceClockHandle);
		break;
	}


}

static void buttonDebounceSwiFxn(UArg buttonId)
{


	PIN_Id the_other_pin = ((buttonId == Board_PIN_BUTTON1) ? Board_PIN_BUTTON0 : Board_PIN_BUTTON1);

	uint8_t buttonPinVal = PIN_getInputValue(buttonId);
	uint8_t b_event = ((buttonId == Board_PIN_BUTTON1) ? button1_event : button0_event);

	if (buttonPinVal)
	{

		if (button_clock_event & b_event)
		{
			if (button_clock_event == both_button_event) button_clock_event |= wait_release_other_button_event;
			else
			{
				if (Util_isActive(&longbuttonClock)) Util_stopClock(&longbuttonClock);
				PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, buttonId | PIN_IRQ_NEGEDGE);
				if (button_clock_event & wait_release_other_button_event)
				{
					button_clock_event &= (~wait_release_other_button_event);
					PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, the_other_pin | PIN_IRQ_NEGEDGE);
				}
				/*button end event*/
				adv_mouse_data.data = 0x04;
				enqueue_AppMsg(button_queue_event, NULL);
				right_push = (left_push = 0x00);
				if (end_i2c)
				{
					end_i2c = 0x00;
					i2c_event.event = i2c_adxl_interrup_clear_event;
					i2cTxbuf[0] = 0x30;
					i2cTrans.readCount = 0;
					i2cTrans.writeCount = 1;
					enqueue_AppMsg(i2c_queue_event, NULL);
				}
				
			}
			button_clock_event &= (~b_event);
		}
		else PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, buttonId | PIN_IRQ_NEGEDGE);
	}
	else
	{
		PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, buttonId | PIN_IRQ_POSEDGE);
		if (!(button_clock_event & b_event))
		{
			button_clock_event |= b_event;
			Util_startClock(&longbuttonClock);
			if (button_clock_event == button0_event)
			{
				adv_mouse_data.data = 0x00;
				enqueue_AppMsg(button_queue_event, NULL);
				left_push = 0x01;

			}
			else if (button_clock_event == button1_event)
			{
				adv_mouse_data.data = 0x01;
				enqueue_AppMsg(button_queue_event, NULL);
				right_push = 0x01;
				
			}
			else
			{
				adv_mouse_data.data = 0x02;
				enqueue_AppMsg(button_queue_event, NULL);
				
			}
		}
	}

}
