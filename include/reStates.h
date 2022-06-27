/* 
   EN: Module for centralized monitoring of the state of system components: WiFi, MQTT, Telegram, etc.
   RU: Модуль централизованного контроля состояния компонентов системы: WiFi, MQTT, Telegram и т.д.
   --------------------------
   (с) 2021 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_STATES_H__
#define __RE_STATES_H__

#include <stddef.h>
#include "esp_bit_defs.h"
#include "project_config.h"
#include "def_consts.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "rLog.h"
#include "rStrings.h"
#include "reLed.h"
#include "reEsp32.h"
#include "reEvents.h"
#include "reParams.h"
#if CONFIG_MQTT_OTA_ENABLE
#include "esp_ota_ops.h"
#endif // CONFIG_MQTT_OTA_ENABLE
#if CONFIG_HEAP_TRACING_STANDALONE
#include "esp_heap_trace.h"
#endif // CONFIG_HEAP_TRACING_STANDALONE
#if CONFIG_TELEGRAM_ENABLE
#include "reTgSend.h"
#endif // CONFIG_TELEGRAM_ENABLE
#if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_GLOBAL
  #include "reNotifier.h"
  #define CONFIG_ENABLE_STATES_NOTIFICATIONS 1
#else
  #define CONFIG_ENABLE_STATES_NOTIFICATIONS 0
#endif // CONFIG_TELEGRAM_ENABLE

static const uint32_t SYSTEM_STARTED       = BIT0;
// Time
static const uint32_t TIME_RTC_ENABLED     = BIT1;
static const uint32_t TIME_SNTP_SYNC_OK    = BIT2;
static const uint32_t TIME_SILENT_MODE     = BIT3;
static const uint32_t TIME_IS_OK           = TIME_RTC_ENABLED | TIME_SNTP_SYNC_OK;

// WiFi
static const uint32_t WIFI_STA_STARTED     = BIT5;
static const uint32_t WIFI_STA_CONNECTED   = BIT6;
static const uint32_t INET_AVAILABLED      = BIT7;
static const uint32_t INET_SLOWDOWN        = BIT8;

// System flags
static const uint32_t SYSTEM_OTA           = BIT12;

// MQTT 
static const uint32_t MQTT_1_ENABLED       = BIT16;
static const uint32_t MQTT_2_ENABLED       = BIT17;
static const uint32_t MQTT_CONNECTED       = BIT18;
static const uint32_t MQTT_PRIMARY         = BIT19;
static const uint32_t MQTT_LOCAL           = BIT20;

// Errors
static const uint32_t ERR_GENERAL          = BIT0;
static const uint32_t ERR_HEAP             = BIT1;
static const uint32_t ERR_MQTT             = BIT2;

static const uint32_t ERR_TELEGRAM         = BIT3;
static const uint32_t ERR_SMTP             = BIT4;
static const uint32_t ERR_NOTIFY           = ERR_TELEGRAM | ERR_SMTP;

static const uint32_t ERR_SITE             = BIT5;
static const uint32_t ERR_THINGSPEAK       = BIT6;
static const uint32_t ERR_OPENMON          = BIT7;
static const uint32_t ERR_NARODMON         = BIT8;
static const uint32_t ERR_PUBLISH          = ERR_SITE | ERR_THINGSPEAK | ERR_OPENMON | ERR_NARODMON;

static const uint32_t ERR_SENSOR_0         = BIT16;
static const uint32_t ERR_SENSOR_1         = BIT17;
static const uint32_t ERR_SENSOR_2         = BIT18;
static const uint32_t ERR_SENSOR_3         = BIT19;
static const uint32_t ERR_SENSOR_4         = BIT20;
static const uint32_t ERR_SENSOR_5         = BIT21;
static const uint32_t ERR_SENSOR_6         = BIT22;
static const uint32_t ERR_SENSOR_7         = BIT23;
static const uint32_t ERR_SENSORS          = ERR_SENSOR_0 | ERR_SENSOR_1 | ERR_SENSOR_2 | ERR_SENSOR_3 | ERR_SENSOR_4 | ERR_SENSOR_5 | ERR_SENSOR_6 | ERR_SENSOR_7;

#ifdef __cplusplus
extern "C" {
#endif

void statesInit(bool registerEventHandler);
void statesFree(bool unregisterEventHandler);
bool statesEventHandlerRegister();
void statesEventHandlerUnregister();

EventBits_t statesGet();
char* statesGetJson();
bool statesCheck(EventBits_t bits, const bool clearOnExit);
bool statesClear(EventBits_t bits);
bool statesSet(EventBits_t bits);
bool statesSetBit(EventBits_t bit, bool state);
EventBits_t statesWait(EventBits_t bits, BaseType_t clearOnExit, BaseType_t waitAllBits, TickType_t timeout);
EventBits_t statesWaitMs(EventBits_t bits, BaseType_t clearOnExit, BaseType_t waitAllBits, TickType_t timeout);

bool statesWiFiIsConnected();
bool statesWiFiWait(TickType_t timeout);
bool statesWiFiWaitMs(TickType_t timeout);
bool statesInetIsAvailabled();
bool statesInetIsDelayed();
bool statesInetIsGood(bool checkRssi);
bool statesInetWait(TickType_t timeout);
bool statesInetWaitMs(TickType_t timeout);

EventBits_t statesGetErrors();
char* statesGetErrorsJson();
bool statesCheckErrors(EventBits_t bits, const bool clearOnExit);
bool statesCheckErrorsAll(const bool clearOnExit);
bool statesSetErrors(EventBits_t bits);
bool statesSetError(EventBits_t bit, bool state);
bool statesClearErrors(EventBits_t bits);
bool statesClearErrorsAll();

bool statesTimeIsOk();
bool statesTimeWait(TickType_t timeout);
bool statesTimeWaitMs(TickType_t timeout);
#if CONFIG_SILENT_MODE_ENABLE
bool statesTimeIsSilent();
#endif // CONFIG_SILENT_MODE_ENABLE

bool statesMqttIsConnected();
bool statesMqttIsPrimary();
bool statesMqttIsLocal();
bool statesMqttIsEnabled();

void heapAllocFailedInit();
uint32_t heapAllocFailedCount();
void heapCapsDebug(const char *function_name);

void ledSysInit(int8_t ledGPIO, bool ledHigh, uint32_t taskStackSize, ledCustomControl_t customControl);
void ledSysFree();
void ledSysFree();
void ledSysOn(const bool fixed);
void ledSysOff(const bool fixed);
void ledSysSet(const bool newState);
void ledSysSetEnabled(const bool newEnabled);
void ledSysActivity();
void ledSysFlashOn(const uint16_t quantity, const uint16_t duration, const uint16_t interval);
void ledSysBlinkOn(const uint16_t quantity, const uint16_t duration, const uint16_t interval);
void ledSysBlinkOff();

#ifdef __cplusplus
}
#endif

#endif // __RE_STATES_H__
