
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <openthread/instance.h>
#include <openthread/thread.h>

#include "app_scheduler.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "bsp_btn_ble.h"
#include "bsp_thread.h"
#include "FreeRTOS.h"
#include "task.h"
#include "boards.h"
#include "timers.h"
#include "thread_utils.h"
#include "semphr.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_drv_clock.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_uart.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

///Missing h-files
#include "nrf_delay.h"
#include "nrf_drv_ppi.h"
#include "nrf_twi_mngr.h"
#include <math.h>
#include <stdio.h>


//Own files

//Communication
#include "arq.h"
#include "network.h"
#include "server_communication.h"
#include "simple_protocol.h" 

//Drivers
#include "DisplayTask.h"
//#include "MPU6050.h"
#include "defines.h"
#include "software/globals.h"
//#include "encoder.h"
#include "encoder_with_counter.h"
#include "i2c.h"
#include "ir.h"
//#include "mag3110.h"
#include "microsd.h"
#include "motor.h"
#include "servo.h"
#include "oled20.h"
#include "led.h"
#include "ICM_20948.h"

//Software
#include "DebugFunctions.h"
#include "SensorTowerTask.h"
#include "ControllerTask.h"
#include "EstimatorTask.h"
#include "NewEstimatorTask.h"
#include "MainComTask.h"
#include "globals.h"
#include "EncoderTester.h"
#include "EncoderWithCounterTester.h"
#include "IMUTester.h"
#include "SensorTowerTester.h"
#include "positionEstimate.h"
#include "MotorSpeedControllerTask.h"

#include "robot_config.h"
#include "mqttsn_client.h"



#define NRF_LOG_ENABLED 1

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;                                /**< Definition of Logger thread. */
#define SCHED_QUEUE_SIZE       32                                           /**< Maximum number of events in the scheduler queue. */
#define SCHED_EVENT_DATA_SIZE  APP_TIMER_SCHED_EVENT_DATA_SIZE              /**< Maximum app_scheduler event size. */

#define THREAD_STACK_TASK_STACK_SIZE     (( 1024 * 8 ) / sizeof(StackType_t))   /**< FreeRTOS task stack size is determined in multiples of StackType_t. */
#define LOG_TASK_STACK_SIZE              ( 1024 / sizeof(StackType_t))          /**< FreeRTOS task stack size is determined in multiples of StackType_t. */
#define THREAD_STACK_TASK_PRIORITY       2
#define LOG_TASK_PRIORITY                1
#define LED1_TASK_PRIORITY               1
#define LED2_TASK_PRIORITY               1
#define LED1_BLINK_INTERVAL              427
#define LED2_BLINK_INTERVAL              472
#define SEARCH_GATEWAY_TIMEOUT 5   
#define TASK_DELAY        200           /**< Task delay. Delays a LED0 task for 200 ms */

typedef struct
{
    TaskHandle_t     thread_stack_task;     /**< Thread stack task handle */
    TaskHandle_t     logger_task;           /**< Definition of Logger task. */
    TaskHandle_t     led1_task;             /**< LED1 task handle*/
    TaskHandle_t     led2_task;             /**< LED2 task handle*/
} application_t;

application_t m_app =
{
    .thread_stack_task = NULL,
    .logger_task       = NULL,
    .led1_task         = NULL,
    .led2_task         = NULL,
};


#ifdef BSP_BUTTON_0
    #define PIN_IN BSP_BUTTON_0
#endif
#ifndef PIN_IN
    #error "Please indicate input pin"
#endif

#ifdef BSP_LED_0
    #define PIN_OUT BSP_LED_0
#endif
#ifndef PIN_OUT
    #error "Please indicate output pin"
#endif

#if LEDS_NUMBER <= 2
#error "Board is not equipped with enough amount of LEDs"
#endif

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

// MQTT Defines
#define MQTT_SUB "v1/sub"
#define MQTT_PUB "v1/pub"
#define MQTT_ID MQTT_SUB "-id"
#define MESSAGE_LENGTH 12                                      /**< Length of message. Max 255>*/

static mqttsn_client_t      m_client;                          /**< An MQTT-SN client instance. */
static mqttsn_remote_t      m_gateway_addr;                    /**< A gateway address. */
static uint8_t              m_gateway_id;                      /**< A gateway ID. */
static mqttsn_connect_opt_t m_connect_opt;                     /**< Connect options for the MQTT-SN client. */
static uint16_t             m_msg_id_sub           = 0;        /**< Message ID thrown with MQTTSN_EVENT_TIMEOUT. */
static uint16_t             m_msg_id_pub           = 0;        /**< Message ID thrown with MQTTSN_EVENT_TIMEOUT. */
static bool                 m_subscribed       = 0;            /**< Current subscription state. */
static char                 m_client_id[]    =  MQTT_ID;      /**< The MQTT-SN Client's ID. */

static char                 m_topic_pub_name[]   =  MQTT_PUB;  /**< Name of the topic to publish to. */
static mqttsn_topic_t       m_topic_pub            =           /**< Topic corresponding to publisher>*/
{
    .p_topic_name = (unsigned char *)m_topic_pub_name,
    .topic_id     = 0,
};

static char                 m_topic_sub_name[] = MQTT_SUB;     /**< Name of the topic to subscribe to. */
static mqttsn_topic_t       m_topic_sub        =               
{
    .p_topic_name = (unsigned char *)m_topic_sub_name,
    .topic_id     = 0,
};


TaskHandle_t led_toggle_task_handle;   /**< Reference to LED0 toggling FreeRTOS task. */

/* DEFINING GLOBAL AND SHARED FLAG VARIABLES */
TaskHandle_t handle_display_task,
    handle_user_task,
    handle_microsd_task,
    pose_estimator_task,
    pose_controller_task,
    motor_speed_controller_task,
    communication_task,
    sensor_tower_task,
    arq_task,
    imu_tester,
    encoder_tester,
    encoder_with_counter_tester,
    sensor_tower_tester;

/* Semaphore handles */
SemaphoreHandle_t xScanLock;
SemaphoreHandle_t xPoseMutex;
SemaphoreHandle_t xTickBSem;
SemaphoreHandle_t xControllerBSem;
SemaphoreHandle_t xCommandReadyBSem;
SemaphoreHandle_t mutex_spi;
SemaphoreHandle_t mutex_i2c;
//SemaphoreHandle_t xCollisionMutex;


/* Queues */
QueueHandle_t poseControllerQ = 0;
QueueHandle_t scanStatusQ = 0;
QueueHandle_t queue_microsd = 0;

// Flag to indicate connection status. Interrupt can change handshook status
uint8_t gHandshook = false;
uint8_t gPaused = false;

// all SPI driver interaction occurs within mutex, so can safely use global bool
bool shared_SPI_init = false;

/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook(void) {
#if NRF_LOG_ENABLED
     vTaskResume(m_logger_thread);
#endif
}

/**@brief Function for initializing the clock.
 */
static void clock_init(void) {
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}

//globals for encoder
int RightMotorDirection = 1;
int LeftMotorDirection = 1;


uint8_t tx_message[MESSAGE_LENGTH] = "publish msg"; 
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

    nrf_drv_gpiote_out_toggle(PIN_OUT);
    uint32_t ec = mqttsn_client_publish(&m_client, m_topic_pub.topic_id, tx_message, MESSAGE_LENGTH, &m_msg_id_pub);
    if (ec != NRF_SUCCESS)
    {
        NRF_LOG_INFO("PUBLISH message could not be sent. Error code: 0x%x\r\n", ec)
    }
}

/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_IN, true);
}


/***************************************************************************************************
 * @section CoAP
 **************************************************************************************************/

static inline void light_on(void)
{
    vTaskResume(m_app.led1_task);
    vTaskResume(m_app.led2_task);
}

static inline void light_off(void)
{
    vTaskSuspend(m_app.led1_task);
    LEDS_OFF(BSP_LED_2_MASK);

    vTaskSuspend(m_app.led2_task);
    LEDS_OFF(BSP_LED_3_MASK);
}

/***************************************************************************************************
 * @section Signal handling
 **************************************************************************************************/

void otTaskletsSignalPending(otInstance * p_instance)
{
    BaseType_t var = xTaskNotifyGive(m_app.thread_stack_task);
    UNUSED_VARIABLE(var);
}

void otSysEventSignalPending(void)
{
    static BaseType_t xHigherPriorityTaskWoken;

    vTaskNotifyGiveFromISR(m_app.thread_stack_task, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/***************************************************************************************************
 * @section State change handling
 **************************************************************************************************/

static void state_changed_callback(uint32_t flags, void * p_context)
{
    NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %d\r\n", flags, otThreadGetDeviceRole(p_context));
}


/**@brief Initializes MQTT-SN client's connection options. */
static void connect_opt_init(void)
{
    m_connect_opt.alive_duration = MQTTSN_DEFAULT_ALIVE_DURATION,
    m_connect_opt.clean_session  = MQTTSN_DEFAULT_CLEAN_SESSION_FLAG,
    m_connect_opt.will_flag      = MQTTSN_DEFAULT_WILL_FLAG,
    m_connect_opt.client_id_len  = strlen(m_client_id),

    memcpy(m_connect_opt.p_client_id,  (unsigned char *)m_client_id,  m_connect_opt.client_id_len);
}

/**@brief Processes GWINFO message from a gateway.
 *
 * @details This function updates MQTT-SN Gateway information.
 *
 * @param[in]    p_event  Pointer to MQTT-SN event.
 */
static void gateway_info_callback(mqttsn_event_t * p_event)
{
    m_gateway_addr  = *(p_event->event_data.connected.p_gateway_addr);
    NRF_LOG_INFO("NOT ERROR received address in callback.\r\n");
    m_gateway_id    = p_event->event_data.connected.gateway_id;
}

/***************************************************************************************************
 * @section MQTT-SN METHODS
 **************************************************************************************************/


/**@brief MQTT-SN subscription.
 *
 * @details This function Subscribes to topic
 */
static void subscribe(void)
{
    uint8_t  topic_sub_name_len = strlen(m_topic_sub_name);
    uint32_t err_code       = NRF_SUCCESS;

    if (m_subscribed)
    {
        err_code = mqttsn_client_unsubscribe(&m_client, m_topic_sub.p_topic_name, topic_sub_name_len, &m_msg_id_sub);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_INFO("UNSUBSCRIBE message could not be sent.\r\n");
        }
        else
        {
            m_subscribed = false;
        }
    }
    else
    {
        err_code = mqttsn_client_subscribe(&m_client, m_topic_sub.p_topic_name, topic_sub_name_len, &m_msg_id_sub);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_INFO("SUBSCRIBE message could not be sent.\r\n");
        }
        else
        {
            m_subscribed = true;
        }
    }
}

/**@brief Processes CONNACK message from a gateway.
 *
 * @details This function launches the topic registration procedure if necessary.
 */
static void connected_callback(void)
{
    light_on();

    uint32_t err_code = mqttsn_client_topic_register(&m_client,
                                                     m_topic_pub.p_topic_name,
                                                     strlen(m_topic_pub_name),
                                                     &m_msg_id_pub);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("REGISTER message could not be sent. Error code: 0x%x\r\n", err_code);
    }

    subscribe();
}

/**@brief Processes DISCONNECT message from a gateway. */
static void disconnected_callback(void)
{
    light_off();
}

/**@brief Processes REGACK message from a gateway.
 *
 * @param[in] p_event Pointer to MQTT-SN event.
 */
static void regack_callback(mqttsn_event_t * p_event)
{
    m_topic_pub.topic_id = p_event->event_data.registered.packet.topic.topic_id;
    NRF_LOG_INFO("MQTT-SN event: Topic has been registered with ID: %d.\r\n", p_event->event_data.registered.packet.topic.topic_id);
}


static void received_callback(mqttsn_event_t * p_event)
{
        char my_string[MESSAGE_LENGTH+1];
        NRF_LOG_INFO("MQTT-SN event: Content to subscribed topic received.\r\n");

        for(int i = 0; i < MESSAGE_LENGTH; i++)
        {
            my_string[i] = p_event->event_data.published.p_payload[i];
        }
        my_string[MESSAGE_LENGTH] = 0; 
        NRF_LOG_INFO("message: %s", my_string);
}

/**@brief Processes retransmission limit reached event. */
static void timeout_callback(mqttsn_event_t * p_event)
{
    NRF_LOG_INFO("MQTT-SN event: Timed-out message: %d. Message ID: %d.\r\n",                  p_event->event_data.error.msg_type,                  p_event->event_data.error.msg_id);
}

/**@brief Processes results of gateway discovery procedure. */
static void searchgw_timeout_callback(mqttsn_event_t * p_event)
{
    NRF_LOG_INFO("MQTT-SN event: Gateway discovery result: 0x%x.\r\n", p_event->event_data.discovery);
}


/**@brief Function for handling MQTT-SN events. */
void mqttsn_evt_handler(mqttsn_client_t * p_client, mqttsn_event_t * p_event)
{
    switch(p_event->event_id)
    {
        case MQTTSN_EVENT_GATEWAY_FOUND:
            NRF_LOG_INFO("MQTT-SN event: Client has found an active gateway.\r\n");
            gateway_info_callback(p_event);
            break;

        case MQTTSN_EVENT_CONNECTED:
            NRF_LOG_INFO("MQTT-SN event: Client connected.\r\n");
            connected_callback();
            break;

        case MQTTSN_EVENT_DISCONNECT_PERMIT:
            NRF_LOG_INFO("MQTT-SN event: Client disconnected.\r\n");
            disconnected_callback();
            break;

        case MQTTSN_EVENT_REGISTERED:
            NRF_LOG_INFO("MQTT-SN event: Client registered topic.\r\n");
            regack_callback(p_event);
            break;

        case MQTTSN_EVENT_PUBLISHED:
            NRF_LOG_INFO("MQTT-SN event: Client has successfully published content.\r\n");
            break;

        case MQTTSN_EVENT_SUBSCRIBED:
            NRF_LOG_INFO("MQTT-SN event: Client subscribed to topic.\r\n");
            LEDS_ON(BSP_LED_3_MASK);
            break;

        case MQTTSN_EVENT_UNSUBSCRIBED:
            NRF_LOG_INFO("MQTT-SN event: Client unsubscribed to topic.\r\n");
            break;

        case MQTTSN_EVENT_RECEIVED:
            NRF_LOG_INFO("MQTT-SN event: Client received content.\r\n");
            received_callback(p_event);
            break;

        case MQTTSN_EVENT_TIMEOUT:
            NRF_LOG_INFO("MQTT-SN event: Retransmission retries limit has been reached.\r\n");
            timeout_callback(p_event);
            break;

        case MQTTSN_EVENT_SEARCHGW_TIMEOUT:
            NRF_LOG_INFO("MQTT-SN event: Gateway discovery procedure has finished.\r\n");
            searchgw_timeout_callback(p_event);

            // topic registered

                uint32_t err_code;

            if (mqttsn_client_state_get(&m_client) == MQTTSN_CLIENT_CONNECTED)
            {
                err_code = mqttsn_client_disconnect(&m_client);
                if (err_code != NRF_SUCCESS)
                {
                    NRF_LOG_INFO("DISCONNECT message could not be sent. Error: 0x%x\r\n", err_code);
                }
            }
            else
            {
                err_code = mqttsn_client_connect(&m_client, &m_gateway_addr, m_gateway_id, &m_connect_opt);
                if (err_code != NRF_SUCCESS)
                {
                    NRF_LOG_INFO("CONNECT message could not be sent. Error: 0x%x\r\n", err_code);
                }
            }

            break;

        default:
            break;
    }
}


/***************************************************************************************************
 * @section State
 **************************************************************************************************/

/**@brief Function for initializing the Thread Stack
 */

static void timer_init(void)
{
    uint32_t error_code = app_timer_init();
    APP_ERROR_CHECK(error_code);
}

static void thread_instance_init(void)
{
    thread_configuration_t thread_configuration =
    {
        .radio_mode        = THREAD_RADIO_MODE_RX_ON_WHEN_IDLE, // Changed from .role, since thread_utils.h might have changed it with radio_mode
        .autocommissioning = true,
    };

    thread_init(&thread_configuration);
    thread_cli_init();
    thread_state_changed_callback_set(state_changed_callback);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void) // THIS METHOD MIGHT BE KILLED
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void mqttsn_init(void)
{
    uint32_t err_code = mqttsn_client_init(&m_client,
                                           MQTTSN_DEFAULT_CLIENT_PORT,
                                           mqttsn_evt_handler,
                                           thread_ot_instance_get());
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("MQTTS inited, error code: %d", err_code);

    connect_opt_init();
}

static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/***************************************************************************************************
 * @section Tasks
 **************************************************************************************************/

static void thread_stack_task(void * arg)
{
    UNUSED_PARAMETER(arg);
    
    mqttsn_init();

    uint32_t err_code = mqttsn_client_search_gateway(&m_client, SEARCH_GATEWAY_TIMEOUT);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("SEARCH GATEWAY message could not be sent. Error: 0x%x\r\n", err_code);
    }else{
        NRF_LOG_INFO("Search gateway message sendt");
    }

    while (1)
    {   

        thread_process();
        app_sched_execute();
        if (NRF_LOG_PROCESS() == false)
        {
            thread_sleep();
        }

        UNUSED_VARIABLE(ulTaskNotifyTake(pdTRUE, portMAX_DELAY));
    }
}

void pos_estimate_init()
{
    position_estimate_t pos_est = {0,0,0};
    set_position_estimate(&pos_est);
}

////////////////////// END OF CODE gotten from hunshamar //////////////////////////////


/**@brief User task
 *
 * @details Task for miscellaneous operations. Currently only adding display ops
 *			to the display_task's queue.
 */
 
static void user_task(void *arg) {

/*	Template for writing to microsd card. If logging faster than 400ms to sd card it can slow the robot down considerably.  
    microsd_write_operation_t write;
    write.filename = "Log.txt";			// Filename can be anything
    write.content = "Starting slam application \n";
    xQueueSendToBack(queue_microsd, &write, portMAX_DELAY);
*/
    NRF_LOG_INFO("user task: initializing");
   
    //UNUSED_PARAMETER(arg);
    //initialization of modules should be done after FreeRtos startup
    taskENTER_CRITICAL();
    motor_init();
    servo_init();
    encoder_init_int();
    //encoder_with_counter_init();
    taskEXIT_CRITICAL();
    i2c_init();
    vTaskDelay(30);
    IMU_init();

    vTaskDelay(1500);
    setMotorSpeedReference(60,60);

    vTaskPrioritySet(handle_user_task, 1);
    vTaskDelay(5000);

    NRF_LOG_INFO("IMU reading: %d\n\r", g_IMU_float_gyroX());

    //mag_init(MAG_OS_128);//oversampling rate used to set datarate 16->80hz 32->40hz 64->20hz 128->10hz
    //the rest of this is just used for testing and displaying values
    //vTaskSuspend(NULL);//no need to run more here except for debugging purposes

    //vTaskPrioritySet(handle_user_task, 1);
    
    //vTaskDelay(5000);

    //char str1[20];
    //char str2[20];
    //char str3[20];
    //char str4[20];
	//char str5[20];
	
	float targetX = 50;
	float targetY = 0;
	bool sent = false;
	bool testWaypoint = false;
    //cartesian target;
    
	
    NRF_LOG_INFO("User task: init complete");
    while(true){
        vTaskDelay(1000);
		
		// Test-function, sends targetX and targetY to controller some time after initialization, used to test waypoints without server running.
		if(testWaypoint){
			int time = (xTaskGetTickCount()/1000);
			NRF_LOG_INFO("Time: %i", (int)time);
		
			if ((time > 40) && (sent == false)){
				cartesian target = {targetX, targetY};
				xQueueSend(poseControllerQ, &target, 100); //Sends target to poseControllerQ, which is received and handled by ControllerTask
				sent = true;
				time = 0;
			}
		}
    }
}


/**@brief Function for application main entry.
*   
*   vMainCommunicationTask only runs if USEBLUETOOTH is true, same for vARQTask.
*
*/




void init_queues()
{
    //initialize queues
	queue_display = xQueueCreate(5, sizeof(display_operation_t));       //For sending things to display
	queue_microsd = xQueueCreate(5, sizeof(microsd_write_operation_t)); //For writing things to micro SD
	poseControllerQ = xQueueCreate(1, sizeof(struct sCartesian));       // For setpoints to controller
	scanStatusQ = xQueueCreate(1, sizeof(uint8_t));                     // For robot status
    encoderTicksToMotorSpeedControllerQ = xQueueCreate(100, sizeof(encoderTicks)); 
    encoderTicksToMotorPositionControllerQ = xQueueCreate(100, sizeof(encoderTicks)); 
    encoderTicksToEstimatorTaskQ = xQueueCreate(100, sizeof(encoderTicks)); 
} 

void init_mutex()
{
    //initialize mutexes
	mutex_spi = xSemaphoreCreateMutex();
    mutex_i2c = xSemaphoreCreateMutex();
	xPoseMutex = xSemaphoreCreateMutex();       // Global variables for robot pose. Only updated from estimator, accessed from many
	xTickBSem = xSemaphoreCreateBinary();       // Global variable to hold robot tick values
    xSemaphoreGive(xTickBSem);
	xControllerBSem = xSemaphoreCreateBinary(); // Estimator to Controller synchronization
	xCommandReadyBSem = xSemaphoreCreateBinary();
}

void initialize_system(){
    bsp_board_init(BSP_INIT_LEDS);
    log_init();
    scheduler_init();
    timer_init();
    gpio_init();
    thread_instance_init(); 
    // clock_init();
    ir_init();
    pos_estimate_init();
}

int main(void) 
{
    NRF_LOG_INFO("IM ALIVE!");
    bool erase_bonds = false;

    initialize_system();

    // Do not start any interrupt that uses system functions before system initialisation.
    // The best solution is to start the OS before any other initalisation.
    if(USEBLUETOOTH)
    {
        // BLE_init(); // Log_init ran from here, must be initialized after uart is initialized
        // arq_init(); // THIS is possibly also usefull when using thread, but FkIt for now
    }

    #if NRF_LOG_ENABLED
		if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 256, NULL, 1, &m_logger_thread))
			APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	#endif
    init_queues();
    init_mutex();

    // if (pdPASS != xTaskCreate(display_task, "DISP", 128, NULL, 1, &handle_display_task))
    //     APP_ERROR_HANDLER(NRF_ERROR_NO_MEM); 

    if (pdPASS != xTaskCreate(user_task, "USER", 128, NULL, 4, &handle_user_task)) //needs elevated priority because init functions
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);

    // if (pdPASS != xTaskCreate(Sensortower_tester, "SensortowerTest", 256, NULL, 1, &sensor_tower_tester)){
    //     APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    // }

    if(USE_SPEED_CONTROLLER)
    {
        if (pdPASS != xTaskCreate(vMotorSpeedControllerTask, "SPEED", 256, NULL, 2, &motor_speed_controller_task))
             APP_ERROR_HANDLER(NRF_ERROR_NO_MEM); 
    }

    // if (pdPASS != xTaskCreate(vMainPoseControllerTask, "POSC", 512, NULL, 1, &pose_controller_task))
    //     APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    
    // if (pdPASS != xTaskCreate(vMainSensorTowerTask, "SnsT", 256, NULL, 1, &sensor_tower_task))
	// 	APP_ERROR_HANDLER(NRF_ERROR_NO_MEM); 
    
	if (pdPASS != xTaskCreate(vMainCommunicationTask, "COM", 256, NULL, 1, &communication_task)) // Moved to this loop in order to use it for thread communications aswell
            APP_ERROR_HANDLER(NRF_ERROR_NO_MEM); 

    // Start thread stack execution.
    if (pdPASS != xTaskCreate(thread_stack_task, "THR", THREAD_STACK_TASK_STACK_SIZE, NULL, 2, &m_app.thread_stack_task))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    
    #if NRF_LOG_ENABLED
      

    //Start execution.
    if (pdPASS != xTaskCreate(logger_thread, "LOG", LOG_TASK_STACK_SIZE, NULL, 1, &m_app.logger_task))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    #endif // NRF_LOG_ENABLED
    

	// /* Not ran when using thread */
    // if (USEBLUETOOTH)
    // {
    //     if(pdPASS != xTaskCreate(vARQTask, "ARQ", 256, NULL, 2, &arq_task)) {
    //         NRF_LOG_INFO("vARQTask Creation Failed");
    //         APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    //     }
    //     vTaskSuspend(arq_task);//suspend arq task to avoid start before init and avoids init before freertos start at same time
    //     //    // Creates a FreeRTOS task for the BLE stack.
    //     //    // The task will run advertising_start() before entering its loop.
    //     nrf_sdh_freertos_init((nrf_sdh_freertos_task_hook_t) advertising_start,&erase_bonds);
    // } 

    size_t freeHeapSize5 = xPortGetMinimumEverFreeHeapSize();
    NRF_LOG_INFO("EverFreeHeapSize5 %d", freeHeapSize5); //If 
    NRF_LOG_INFO("\nInitialization done. SLAM application now starting.\n." + erase_bonds);
    if(PRINT_DEBUG)printf("Application starting");
    vTaskStartScheduler();
    for (;;) {
        /**
		* vTaskStartSchedule returns only if the system failed to allocate heap
		* memory. Either reduce heap memory usage, or increase the allocation
		* of heap memory in FreeRTOSConfig.h via configTOTAL_HEAP_SIZE
		*/
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN); //See comment above
    }
}



