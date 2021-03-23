/*
 * Based on Blom 2020
 * 2020 Eivind Jølsgard
 * 
 **/

#include "openTreadMQTT.h"

#include <openthread/thread.h>
#include "nrf.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "bsp_thread.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "mqttsn_client.h"
#include "thread_utils.h"

#include <openthread/instance.h>
#include <openthread/thread.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "globals.h"

#include "FreeRTOS.h"
#include "nrf_drv_clock.h"
#include "task.h"
#include "robot_config.h"

#define SEARCH_GATEWAY_TIMEOUT 5                                            /**< MQTT-SN Gateway discovery procedure timeout in [s]. */

static mqttsn_client_t      m_client;                          /**< An MQTT-SN client instance. */
static mqttsn_remote_t      m_gateway_addr;                    /**< A gateway address. */
static uint8_t              m_gateway_id;                      /**< A gateway ID. */
static mqttsn_connect_opt_t m_connect_opt;                     /**< Connect options for the MQTT-SN client. */

static char                 m_client_id[]    		=  ROBOT_NAME;      /**< The MQTT-SN Client's ID. */
#define MESSAGE_LENGTH 12                                      /**< Length of message. Max 255>*/




/***************************************************************************************************
 * @section Signal handling
 **************************************************************************************************/

void otTaskletsSignalPending(otInstance * p_instance)
{
    NRF_LOG_INFO("otTaskletsSignalPending is called");
    // BaseType_t var = xTaskNotifyGive(mqtt_task);
    // UNUSED_VARIABLE(var);
}

void otSysEventSignalPending(void)
{
    NRF_LOG_INFO("otTaskletsSignalPending is called");
    static BaseType_t xHigherPriorityTaskWoken;

    // vTaskNotifyGiveFromISR(mqtt_task, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

///
/***************************************************************************************************
 * @section State change handling
 **************************************************************************************************/

static void state_changed_callback(uint32_t flags, void * p_context)
{
    NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %d\r\n",
                 flags, otThreadGetDeviceRole(p_context));
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
    m_gateway_id    = p_event->event_data.connected.gateway_id;
}


///
/***************************************************************************************************
 * @section Subscribe / unsubcribe
 **************************************************************************************************/

void mqtt_subscribe(topic_t* topic)
{
    uint8_t  topic_sub_name_len = strlen((char*)topic->name);
    uint32_t err_code       = NRF_SUCCESS;
    topic->id_pub = 0;

    if (!topic->subscribed)
    {
        err_code = mqttsn_client_subscribe(&m_client, topic->name, topic_sub_name_len, &(topic->id_pub));
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("SUBSCRIBE message could not be sent.\r\n");
        }
        else
        {
            topic->subscribed = true;
        }
    }

}

void mqtt_unsubscribe(topic_t* topic)
{
    uint8_t  topic_sub_name_len = strlen((char*)topic->name);
    uint32_t err_code;

    if (topic->subscribed)
    {
        err_code = mqttsn_client_unsubscribe(&m_client, topic->name, topic_sub_name_len, &(topic->id_pub));
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("UNSUBSCRIBE message could not be sent.\r\n");
        }
        else
        {
            topic->subscribed = false;
        }
    }


}

void mqtt_register_publish_topic(topic_t* topic)
{
    uint32_t err_code = mqttsn_client_topic_register(&m_client,
                                                     topic->name,
                                                     strlen((char*)topic->name),
                                                     &(topic->id_pub));
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("REGISTER message could not be sent. Error code: 0x%x\r\n", err_code);
    }
}

/**@brief Processes CONNACK message from a gateway.
 *
 * @details This function launches the topic registration procedure if necessary.
 */
static void connected_callback(void)
{
    //Callback register publish topic

    //subscribe();
}


/**@brief Processes DISCONNECT message from a gateway. */
static void disconnected_callback(void)
{
    //
}


/**@brief Processes REGACK message from a gateway.
 *
 * @param[in] p_event Pointer to MQTT-SN event.
 */
static void regack_callback(mqttsn_event_t * p_event)
{
    //m_topic_pub.topic_id = p_event->event_data.registered.packet.topic.topic_id;
    NRF_LOG_INFO("MQTT-SN event: Topic has been registered with ID: %d.\r\n",
                 p_event->event_data.registered.packet.topic.topic_id);
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
    NRF_LOG_INFO("MQTT-SN event: Timed-out message: %d. Message ID: %d.\r\n",
                  p_event->event_data.error.msg_type,
                  p_event->event_data.error.msg_id);
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
                    NRF_LOG_ERROR("CONNECT message could not be sent. Error: 0x%x\r\n", err_code);
                }
            }

            break;

        default:
            break;
    }
}
//



/**@brief Function for initializing the Thread Stack
 */
static void thread_instance_init(void)
{
    thread_configuration_t thread_configuration =
    {
        .radio_mode        = THREAD_RADIO_MODE_RX_ON_WHEN_IDLE,
        .autocommissioning = true,
    };

    thread_init(&thread_configuration);
    thread_cli_init();
    thread_state_changed_callback_set(state_changed_callback);
}

//

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

//


void vMQTTTask(void *pvParameters)
{
    UNUSED_PARAMETER(pvParameters);

    thread_instance_init(); 
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
//        app_sched_execute();
        if (NRF_LOG_PROCESS() == false)
        {
            thread_sleep();
        }

        
        UNUSED_VARIABLE(ulTaskNotifyTake(pdTRUE, portMAX_DELAY));
    }
}

//



////////////////////////////

uint32_t publish_message(mqtt_message_t* mqtt_msg)
{

    mqttsn_client_publish(&m_client, mqtt_msg->topic.id, mqtt_msg->message, mqtt_msg->message_length, &(mqtt_msg->topic.id_pub));

    return 0;
}
