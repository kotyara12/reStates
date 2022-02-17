#include "time.h"
#include "reStates.h"
#include "reEvents.h"
#include "rStrings.h"
#include "reEsp32.h"
#include "esp_timer.h"
#include "rLog.h"
#include "reSensor.h"
#include "reWiFi.h"
#include "reNvs.h"
#include "project_config.h"
#include "def_consts.h"
#if CONFIG_TELEGRAM_ENABLE
#include "reTgSend.h"
#endif // CONFIG_TELEGRAM_ENABLE
#if CONFIG_TELEGRAM_ENABLE
  #define CONFIG_ENABLE_STATE_NOTIFICATIONS 1
#else
  #define CONFIG_ENABLE_STATE_NOTIFICATIONS 0
#endif // CONFIG_TELEGRAM_ENABLE
#if CONFIG_ENABLE_STATE_NOTIFICATIONS
#include "reNotifier.h"
#endif // CONFIG_ENABLE_STATE_NOTIFICATIONS


static EventGroupHandle_t _evgStates = nullptr;
static EventGroupHandle_t _evgErrors = nullptr;

static const char* logTAG   = "STATES";
#define DEBUG_LOG_EVENT_MESSAGE "Received event: event_base=[%s], event_id=[%s]"
#define DEBUG_LOG_EVENT_MESSAGE_MODE "Received event: event_base=[%s], event_id=[%s], mode=[%d]"

#if CONFIG_STATES_STATIC_ALLOCATION
  StaticEventGroup_t _bufStates;
  StaticEventGroup_t _bufErrors;
#endif // CONFIG_STATES_STATIC_ALLOCATION

void ledSysBlinkAuto();

// -----------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------- System states ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void heapAllocFailedInit();

void statesInit(bool registerEventHandler)
{
  if (!_evgStates) {
    #if CONFIG_STATES_STATIC_ALLOCATION
      _evgStates = xEventGroupCreateStatic(&_bufStates);
    #else
      _evgStates = xEventGroupCreate();
    #endif // CONFIG_STATES_STATIC_ALLOCATION
    xEventGroupClearBits(_evgStates, 0x00FFFFFFU);
  };
  if (!_evgErrors) {
    #if CONFIG_STATES_STATIC_ALLOCATION
      _evgErrors = xEventGroupCreateStatic(&_bufErrors);
    #else
      _evgErrors = xEventGroupCreate();
    #endif // CONFIG_STATES_STATIC_ALLOCATION
    xEventGroupClearBits(_evgErrors, 0x00FFFFFFU);
  };

  if ((_evgStates) && (_evgErrors)) {
    heapAllocFailedInit();
  };

  if ((_evgStates) && (_evgErrors) && registerEventHandler) {
    statesEventHandlerRegister();
  };
}

void statesFree(bool unregisterEventHandler)
{
  if (_evgStates) {
    if (unregisterEventHandler) {
      statesEventHandlerUnregister();
    };
  };

  if (_evgErrors) {
    #if CONFIG_STATES_STATIC_ALLOCATION
    #endif // CONFIG_STATES_STATIC_ALLOCATION
    vEventGroupDelete(_evgErrors);
    _evgErrors = nullptr;
  };

  if (_evgStates) {
    #if CONFIG_STATES_STATIC_ALLOCATION
    #endif // CONFIG_STATES_STATIC_ALLOCATION
    vEventGroupDelete(_evgStates);
    _evgStates = nullptr;
  };
}

EventBits_t statesGet() 
{
  if (_evgStates) {
    return xEventGroupGetBits(_evgStates);
  };
  rlog_e(logTAG, "Failed to get status bits, event group is null!");
  return 0;
}

bool statesCheck(EventBits_t bits, const bool clearOnExit) 
{
  if (_evgStates) {
    if (clearOnExit) {
      return (xEventGroupClearBits(_evgStates, bits) & bits) == bits;
    } else {
      return (xEventGroupGetBits(_evgStates) & bits) == bits;
    };
  };
  rlog_e(logTAG, "Failed to check status bits: %X, event group is null!", bits);
  return false;
}

bool statesClear(EventBits_t bits)
{
  if (!_evgStates) {
    rlog_e(logTAG, "Failed to set status bits: %X, event group is null!", bits);
    return false;
  };
  EventBits_t prevClear = xEventGroupClearBits(_evgStates, bits);
  if ((prevClear & bits) != 0) {
    EventBits_t afterClear = xEventGroupGetBits(_evgStates);
    if ((afterClear & bits) != 0) {
      rlog_e(logTAG, "Failed to clear status bits: %X, current value: %X", bits, afterClear);
      return false;
    };
  };
  ledSysBlinkAuto();
  return true;
}

bool statesSet(EventBits_t bits)
{
  if (!_evgStates) {
    rlog_e(logTAG, "Failed to set status bits: %X, event group is null!", bits);
    return false;
  };
  EventBits_t afterSet = xEventGroupSetBits(_evgStates, bits);
  if ((afterSet & bits) != bits) {
    rlog_e(logTAG, "Failed to set status bits: %X, current value: %X", bits, afterSet);
    return false;
  };
  ledSysBlinkAuto();
  return true;
}

bool statesSetBit(EventBits_t bit, bool state)
{
  if (state) {
    return statesSet(bit);
  } else {
    return statesClear(bit);
  };
}

EventBits_t statesWait(EventBits_t bits, BaseType_t clearOnExit, BaseType_t waitAllBits, TickType_t timeout)
{
  if (_evgStates) {
    return xEventGroupWaitBits(_evgStates, bits, clearOnExit, waitAllBits, timeout) & bits; 
  };  
  return 0;
}

EventBits_t statesWaitMs(EventBits_t bits, BaseType_t clearOnExit, BaseType_t waitAllBits, TickType_t timeout)
{
  if (_evgStates) {
    if (timeout == 0) {
      return xEventGroupWaitBits(_evgStates, bits, clearOnExit, waitAllBits, portMAX_DELAY) & bits; 
    }
    else {
      return xEventGroupWaitBits(_evgStates, bits, clearOnExit, waitAllBits, pdMS_TO_TICKS(timeout)) & bits; 
    };
  };  
  return 0;
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- Custom routines ---------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool statesWiFiIsConnected()
{
  return statesCheck(WIFI_STA_CONNECTED, false);
}

bool statesWiFiWait(TickType_t timeout)
{
  return (statesWait(WIFI_STA_CONNECTED, pdFALSE, pdTRUE, timeout) & WIFI_STA_CONNECTED) != 0;
}

bool statesWiFiWaitMs(TickType_t timeout)
{
  return (statesWaitMs(WIFI_STA_CONNECTED, pdFALSE, pdTRUE, timeout) & WIFI_STA_CONNECTED) != 0;
}

bool statesInetIsAvailabled()
{
  return statesCheck(WIFI_STA_CONNECTED | INET_AVAILABLED, false);
}

bool statesInetIsDelayed()
{
  return statesCheck(WIFI_STA_CONNECTED | INET_AVAILABLED | INET_SLOWDOWN, false);
}

bool statesInetIsGood()
{
  return statesInetIsAvailabled() and !statesCheck(INET_SLOWDOWN, false);
}

bool statesInetWait(TickType_t timeout)
{
  return (statesWait(WIFI_STA_CONNECTED | INET_AVAILABLED, pdFALSE, pdTRUE, timeout) & (WIFI_STA_CONNECTED | INET_AVAILABLED)) != 0;
}

bool statesInetWaitMs(TickType_t timeout)
{
  return (statesWaitMs(WIFI_STA_CONNECTED | INET_AVAILABLED, pdFALSE, pdTRUE, timeout) & (WIFI_STA_CONNECTED | INET_AVAILABLED)) != 0;
}

// Time
bool statesTimeIsOk()
{
  return statesCheck(TIME_SNTP_SYNC_OK, false) || statesCheck(TIME_RTC_ENABLED, false);
}

bool statesTimeWait(TickType_t timeout)
{
  return (statesWait(TIME_SNTP_SYNC_OK | TIME_RTC_ENABLED, pdFALSE, pdFALSE, timeout) & (TIME_SNTP_SYNC_OK | TIME_RTC_ENABLED)) != 0;
}

bool statesTimeWaitMs(TickType_t timeout)
{
  return (statesWaitMs(TIME_SNTP_SYNC_OK | TIME_RTC_ENABLED, pdFALSE, pdFALSE, timeout) & (TIME_SNTP_SYNC_OK | TIME_RTC_ENABLED)) != 0;
}

#if CONFIG_SILENT_MODE_ENABLE
bool statesTimeIsSilent()
{
  return (statesCheck(TIME_SNTP_SYNC_OK, false) || statesCheck(TIME_RTC_ENABLED, false)) && (statesCheck(TIME_SILENT_MODE, false));
}
#endif // CONFIG_SILENT_MODE_ENABLE

// MQTT
bool statesMqttIsConnected()
{
  return statesCheck(MQTT_CONNECTED, false);
}

bool statesMqttIsPrimary()
{
  return statesCheck(MQTT_PRIMARY, false);
}

bool statesMqttIsLocal()
{
  return statesCheck(MQTT_LOCAL, false);
}

bool statesMqttIsServerEnabled()
{
  if (statesMqttIsLocal()) {
    return statesWiFiIsConnected();
  } else {
    return statesInetIsAvailabled();
  };
}

bool statesMqttIsEnabled()
{
  return statesMqttIsServerEnabled() && statesMqttIsConnected();
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- Errors routines ---------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

EventBits_t statesGetErrors() 
{
  if (_evgErrors) {
    return xEventGroupGetBits(_evgErrors);
  };
  rlog_e(logTAG, "Failed to get errors bits, event group is null!");
  return 0;
}

bool statesCheckErrors(EventBits_t bits, const bool clearOnExit) 
{
  if (_evgErrors) {
    if (clearOnExit) {
      return (xEventGroupClearBits(_evgErrors, bits) & bits) == bits;
    } else {
      return (xEventGroupClearBits(_evgErrors, 0) & bits) == bits;
    };
  };
  rlog_e(logTAG, "Failed to check error bits: %X, event group is null!", bits);
  return false;
}

bool statesCheckErrorsAll(const bool clearOnExit) 
{
  return statesCheckErrors(0x00FFFFFFU, clearOnExit);
}

bool statesClearErrors(EventBits_t bits)
{
  if (!_evgErrors) {
    rlog_e(logTAG, "Failed to set errors bits: %X, event group is null!", bits);
    return false;
  };
  EventBits_t prevClear = xEventGroupClearBits(_evgErrors, bits);
  if ((prevClear & bits) != 0) {
    EventBits_t afterClear = xEventGroupGetBits(_evgErrors);
    if ((afterClear & bits) != 0) {
      rlog_e(logTAG, "Failed to clear errors bits: %X, current value: %X", bits, afterClear);
      return false;
    };
  };
  ledSysBlinkAuto();
  return true;
}

bool statesClearErrorsAll()
{
  return statesClearErrors(0x00FFFFFFU);
}

bool statesSetErrors(EventBits_t bits)
{
  if (!_evgErrors) {
    rlog_e(logTAG, "Failed to set errors bits: %X, event group is null!", bits);
    return false;
  };
  EventBits_t afterSet = xEventGroupSetBits(_evgErrors, bits);
  if ((afterSet & bits) != bits) {
    rlog_e(logTAG, "Failed to set errors bits: %X, current value: %X", bits, afterSet);
    return false;
  };
  ledSysBlinkAuto();
  return true;
}

bool statesSetError(EventBits_t bit, bool state)
{
  if (state) {
    return statesSetErrors(bit);
  } else {
    return statesClearErrors(bit);
  };
}

// -----------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------- JSON routines ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

char* statesGetJson()
{
  EventBits_t states = statesGet(); 
  return malloc_stringf("{\"ota\":%d,\"rtc_enabled\":%d,\"sntp_sync\":%d,\"silent_mode\":%d,\"wifi_sta_started\":%d,\"wifi_sta_connected\":%d,\"inet_availabled\":%d,\"mqtt1_enabled\":%d,\"mqtt2_enabled\":%d,\"mqtt_connected\":%d,\"mqtt_primary\":%d,\"mqtt_local\":%d}",
    (states & SYSTEM_OTA) == SYSTEM_OTA,
    (states & TIME_RTC_ENABLED) == TIME_RTC_ENABLED,
    (states & TIME_SNTP_SYNC_OK) == TIME_SNTP_SYNC_OK,
    (states & TIME_SILENT_MODE) == TIME_SILENT_MODE,
    (states & WIFI_STA_STARTED) == WIFI_STA_STARTED,
    (states & WIFI_STA_CONNECTED) == WIFI_STA_CONNECTED,
    (states & INET_AVAILABLED) == INET_AVAILABLED,
    (states & MQTT_1_ENABLED) == MQTT_1_ENABLED,
    (states & MQTT_2_ENABLED) == MQTT_2_ENABLED,
    (states & MQTT_CONNECTED) == MQTT_CONNECTED,
    (states & MQTT_PRIMARY) == MQTT_PRIMARY,
    (states & MQTT_LOCAL) == MQTT_LOCAL);
};

char* statesGetErrorsJson()
{
  EventBits_t errors = statesGetErrors();
  return malloc_stringf("{\"general\":%d,\"heap\":%d,\"mqtt\":%d,\"telegram\":%d,\"smtp\":%d,\"site\":%d,\"thingspeak\":%d,\"openmon\":%d,\"narodmon\":%d,\"sensor1\":%d,\"sensor2\":%d,\"sensor3\":%d,\"sensor4\":%d,\"sensor5\":%d,\"sensor6\":%d,\"sensor7\":%d,\"sensor8\":%d}",
    (errors & ERR_GENERAL) == ERR_GENERAL,
    (errors & ERR_HEAP) == ERR_HEAP,
    (errors & ERR_MQTT) == ERR_MQTT,
    (errors & ERR_TELEGRAM) == ERR_TELEGRAM,
    (errors & ERR_SMTP) == ERR_SMTP,
    (errors & ERR_SITE) == ERR_SITE,
    (errors & ERR_THINGSPEAK) == ERR_THINGSPEAK,
    (errors & ERR_OPENMON) == ERR_OPENMON,
    (errors & ERR_NARODMON) == ERR_NARODMON,
    (errors & ERR_SENSOR_0) == ERR_SENSOR_0,
    (errors & ERR_SENSOR_1) == ERR_SENSOR_1,
    (errors & ERR_SENSOR_2) == ERR_SENSOR_2,
    (errors & ERR_SENSOR_3) == ERR_SENSOR_3,
    (errors & ERR_SENSOR_4) == ERR_SENSOR_4,
    (errors & ERR_SENSOR_5) == ERR_SENSOR_5,
    (errors & ERR_SENSOR_6) == ERR_SENSOR_6,
    (errors & ERR_SENSOR_7) == ERR_SENSOR_7);
};
  
// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------- Fixing memory allocation errors -------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static uint32_t heapFailsCount = 0;

void heapAllocFailedHook(size_t requested_size, uint32_t caps, const char *function_name)
{
  rlog_e("HEAP", "%s was called but failed to allocate %d bytes with 0x%X capabilities.", function_name, requested_size, caps);
  heapFailsCount++;
  #if CONFIG_HEAP_ALLOC_FAILED_RESTART
    #if defined(CONFIG_HEAP_ALLOC_FAILED_RESTART_DELAY) && (CONFIG_HEAP_ALLOC_FAILED_RESTART_DELAY > 0) 
      espRestart(RR_HEAP_ALLOCATION_FAILED, CONFIG_HEAP_ALLOC_FAILED_RESTART_DELAY); 
    #else
      espRestart(RR_HEAP_ALLOCATION_FAILED, 0); 
    #endif // CONFIG_HEAP_ALLOC_FAILED_RESTART_WAIT
  #endif // CONFIG_HEAP_ALLOC_FAILED_RESTART
}

uint32_t heapAllocFailedCount() 
{
  return heapFailsCount;
}

void heapAllocFailedInit()
{
  heap_caps_register_failed_alloc_callback(heapAllocFailedHook);
  heapFailsCount = 0;
}

void heapCapsDebug(const char *function_name)
{
  rlog_w("HEAP", "%s: heap free %.3f% %", function_name, 100.0 * (double)heap_caps_get_free_size(MALLOC_CAP_DEFAULT) / (double)heap_caps_get_total_size(MALLOC_CAP_DEFAULT));
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ System LED -----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static ledQueue_t _ledSysQueue = NULL;

void ledSysInit(int8_t ledGPIO, bool ledHigh, ledCustomControl_t customControl)
{
  if (_ledSysQueue == NULL) {
    _ledSysQueue = ledTaskCreate(ledGPIO, ledHigh, true, "led_system", customControl);
  };
}

void ledSysFree()
{
  if (_ledSysQueue) { 
    ledTaskDelete(_ledSysQueue);
    _ledSysQueue = nullptr;
  };
}

void ledSysOn(const bool fixed)
{
  if (_ledSysQueue) {
    ledTaskSend(_ledSysQueue, lmOn, (uint16_t)fixed, 0, 0);
  };
}

void ledSysOff(const bool fixed)
{
  if (_ledSysQueue) {
    ledTaskSend(_ledSysQueue, lmOff, (uint16_t)fixed, 0, 0);
  };
}

void ledSysSet(const bool newState)
{
  if (_ledSysQueue) {
    if (newState) {
      ledTaskSend(_ledSysQueue, lmOn, 0, 0, 0);
    }
    else {
      ledTaskSend(_ledSysQueue, lmOff, 0, 0, 0);
    };
  };
}

void ledSysSetEnabled(const bool newEnabled)
{
  if (_ledSysQueue) {
    ledTaskSend(_ledSysQueue, lmEnable, (uint16_t)newEnabled, 0, 0);
  };
}

void ledSysActivity()
{
  if (_ledSysQueue) {
    ledTaskSend(_ledSysQueue, lmFlash, CONFIG_LEDSYS_FLASH_QUANTITY, CONFIG_LEDSYS_FLASH_DURATION, CONFIG_LEDSYS_FLASH_INTERVAL);
  };
}

void ledSysFlashOn(const uint16_t quantity, const uint16_t duration, const uint16_t interval)
{
  if (_ledSysQueue) {
    ledTaskSend(_ledSysQueue, lmFlash, quantity, duration, interval);
  };
}

void ledSysBlinkOn(const uint16_t quantity, const uint16_t duration, const uint16_t interval)
{
  if (_ledSysQueue) {
    ledTaskSend(_ledSysQueue, lmBlinkOn, quantity, duration, interval);
  };
}

void ledSysBlinkOff()
{
  if (_ledSysQueue) {
    ledTaskSend(_ledSysQueue, lmBlinkOff, 0, 0, 0);
  };
}

void ledSysBlinkAuto()
{
  EventBits_t states = statesGet();
  EventBits_t errors = statesGetErrors();
  if (states & SYSTEM_OTA) {
    ledSysBlinkOn(CONFIG_LEDSYS_OTA_QUANTITY, CONFIG_LEDSYS_OTA_DURATION, CONFIG_LEDSYS_OTA_INTERVAL);
  }
  else if (errors & ERR_GENERAL) {
    ledSysBlinkOn(CONFIG_LEDSYS_ERROR_QUANTITY, CONFIG_LEDSYS_ERROR_DURATION, CONFIG_LEDSYS_ERROR_INTERVAL);
  }
  else if (errors & ERR_SENSORS) {
    ledSysBlinkOn(CONFIG_LEDSYS_SENSOR_ERROR_QUANTITY, CONFIG_LEDSYS_SENSOR_ERROR_DURATION, CONFIG_LEDSYS_SENSOR_ERROR_INTERVAL);
  }
  else if (!(states & WIFI_STA_CONNECTED)) {
    ledSysBlinkOn(CONFIG_LEDSYS_WIFI_INIT_QUANTITY, CONFIG_LEDSYS_WIFI_INIT_DURATION, CONFIG_LEDSYS_WIFI_INIT_INTERVAL);
  }
  else if (!(states & INET_AVAILABLED)) {
    ledSysBlinkOn(CONFIG_LEDSYS_PING_FAILED_QUANTITY, CONFIG_LEDSYS_PING_FAILED_DURATION, CONFIG_LEDSYS_PING_FAILED_INTERVAL);
  }
  else if (!(states & TIME_IS_OK)) {
    ledSysBlinkOn(CONFIG_LEDSYS_TIME_ERROR_QUANTITY, CONFIG_LEDSYS_TIME_ERROR_DURATION, CONFIG_LEDSYS_TIME_ERROR_INTERVAL);
  }
  else if (!(states & MQTT_CONNECTED) || (errors & ERR_MQTT)) {
    ledSysBlinkOn(CONFIG_LEDSYS_MQTT_ERROR_QUANTITY, CONFIG_LEDSYS_MQTT_ERROR_DURATION, CONFIG_LEDSYS_MQTT_ERROR_INTERVAL);
  }
  else if (errors & ERR_PUBLISH) {
    ledSysBlinkOn(CONFIG_LEDSYS_PUB_ERROR_QUANTITY, CONFIG_LEDSYS_PUB_ERROR_DURATION, CONFIG_LEDSYS_PUB_ERROR_INTERVAL);
  }
  else if (errors & ERR_TELEGRAM) {
    ledSysBlinkOn(CONFIG_LEDSYS_TG_ERROR_QUANTITY, CONFIG_LEDSYS_TG_ERROR_DURATION, CONFIG_LEDSYS_TG_ERROR_INTERVAL);
  }
  else if (errors & ERR_SMTP) {
    ledSysBlinkOn(CONFIG_LEDSYS_SMTP_ERROR_QUANTITY, CONFIG_LEDSYS_SMTP_ERROR_DURATION, CONFIG_LEDSYS_SMTP_ERROR_INTERVAL);
  }
  else {
    ledSysBlinkOn(CONFIG_LEDSYS_NORMAL_QUANTITY, CONFIG_LEDSYS_NORMAL_DURATION, CONFIG_LEDSYS_NORMAL_INTERVAL);
  };
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------- Event notifications -------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if CONFIG_ENABLE_STATE_NOTIFICATIONS

static time_t _statesNotifyDelayFailure = CONFIG_NOTIFY_TELEGRAM_MINIMUM_FAILURE_TIME;

static bool statesNotifySend(bool alert, const char* message, const char* object, notify_state_t state, time_t time_failure, time_t time_state)
{
  // Format failure start time
  struct tm tm_failure;
  char str_failure[CONFIG_FORMAT_STRFTIME_BUFFER_SIZE];
  memset(&str_failure, 0, sizeof(str_failure));
  localtime_r(&time_failure, &tm_failure);
  strftime(str_failure, sizeof(str_failure), CONFIG_FORMAT_DTS, &tm_failure);

  if (state == FNS_OK) {
    // Format failure end time
    struct tm tm_recovery;
    char str_recovery[CONFIG_FORMAT_STRFTIME_BUFFER_SIZE];
    memset(&str_recovery, 0, sizeof(str_recovery));
    localtime_r(&time_state, &tm_recovery);
    strftime(str_recovery, sizeof(str_recovery), CONFIG_FORMAT_DTS, &tm_recovery);
    // Format failure duration time
    time_t failure_duration = time_state - time_failure;
    uint16_t duration_h = failure_duration / 3600;
    uint16_t duration_m = failure_duration % 3600 / 60;
    uint16_t duration_s = failure_duration % 3600 % 60;
    // Send notify
    if (object) {
      return tgSend(TG_SERVICE, alert, CONFIG_TELEGRAM_DEVICE, message, object, str_failure, str_recovery, duration_h, duration_m, duration_s);
    } else {
      return tgSend(TG_SERVICE, alert, CONFIG_TELEGRAM_DEVICE, message, str_failure, str_recovery, duration_h, duration_m, duration_s);
    };
  } else {
    // Send notify
    if (object) {
      return tgSend(TG_SERVICE, alert, CONFIG_TELEGRAM_DEVICE, message, object, str_failure);
    } else {
      return tgSend(TG_SERVICE, alert, CONFIG_TELEGRAM_DEVICE, message, str_failure);
    };
  };
  return false;
}

static bool statesNotifySendHost(reFailureNotifier* notifier, bool notify_alert, const char* object, notify_state_t state, int32_t value,time_t time_failure, time_t time_state)
{
  if (state == FNS_OK) {
    return statesNotifySend(notify_alert, CONFIG_MESSAGE_TG_HOST_AVAILABLE, object, state, time_failure, time_state);
  } else {
    return statesNotifySend(notify_alert, CONFIG_MESSAGE_TG_HOST_UNAVAILABLE, object, state, time_failure, time_state);
  };
}

// --  WiFi --------------------------------------------------------------------------------------------------------------
#if CONFIG_NOTIFY_TELEGRAM_WIFI_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
  #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
    static uint8_t _notifyTgWifi = CONFIG_NOTIFY_TELEGRAM_WIFI_STATUS;
    static bool statesNotifySendWifi(reFailureNotifier* notifier, bool notify_alert, const char* object, notify_state_t state, int32_t value, time_t time_failure, time_t time_state)
    {
      if (_notifyTgWifi) {
        return statesNotifySend(notify_alert, CONFIG_MESSAGE_TG_WIFI_AVAILABLE, wifiGetSSID(), state, time_failure, time_state);
      };
      return false;
    };
  #else
    static bool statesNotifySendWifi(reFailureNotifier* notifier, bool notify_alert, const char* object, notify_state_t state, int32_t value, time_t time_failure, time_t time_state)
    {
      return statesNotifySend(notify_alert, CONFIG_MESSAGE_TG_WIFI_AVAILABLE, wifiGetSSID(), state, time_failure, time_state);
    };
  #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE

  reFailureNotifier notifierWifi("wifi", CONFIG_NOTIFY_TELEGRAM_ALERT_WIFI_STATUS, FNK_RECOVERY, 0, statesNotifySendWifi);
#endif // CONFIG_NOTIFY_TELEGRAM_WIFI_STATUS

// --  Ping --------------------------------------------------------------------------------------------------------------
#if CONFIG_PINGER_ENABLE && ((CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE > 0) || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
  #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
    static uint8_t _notifyTgInet = CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE;
    static bool statesNotifySendInet(reFailureNotifier* notifier, bool notify_alert, const char* object, notify_state_t state, int32_t value, time_t time_failure, time_t time_state)
    {
      if (state == FNS_OK) {
        if (_notifyTgInet > 0) {
          return statesNotifySend(notify_alert, CONFIG_MESSAGE_TG_INET_AVAILABLE, nullptr, state, time_failure, time_state);
        };
      } else if (state == FNS_SLOWDOWN) {
        if (_notifyTgInet > 1) {
          return statesNotifySend(notify_alert, CONFIG_MESSAGE_TG_INET_SLOWDOWN, nullptr, state, time_failure, time_state);
        };
      } else {
        if (_notifyTgInet > 0) {
          return statesNotifySend(notify_alert, CONFIG_MESSAGE_TG_INET_UNAVAILABLE, nullptr, state, time_failure, time_state);
        };
      };
      return false;
    }
  #else
    static bool statesNotifySendInet(reFailureNotifier* notifier, bool notify_alert, const char* object, notify_state_t state, int32_t value, time_t time_failure, time_t time_state)
    {
      if (state == FNS_OK) {
        return statesNotifySend(notify_alert, CONFIG_MESSAGE_TG_INET_AVAILABLE, nullptr, state, time_failure, time_state);
      } else if (state == FNS_SLOWDOWN) {
        return statesNotifySend(notify_alert, CONFIG_MESSAGE_TG_INET_SLOWDOWN, nullptr, state, time_failure, time_state);
      } else {
        return statesNotifySend(notify_alert, CONFIG_MESSAGE_TG_INET_UNAVAILABLE, nullptr, state, time_failure, time_state);
      };
    }
  #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE

  reFailureNotifier notifierInet("ping", CONFIG_NOTIFY_TELEGRAM_ALERT_INET_UNAVAILABLE, 
    FNK_AUTO, &_statesNotifyDelayFailure, statesNotifySendInet);
#endif // CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE

// --  MQTT --------------------------------------------------------------------------------------------------------------
#if CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
  #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
    static uint8_t _notifyTgMqtt = CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS;
    static bool statesNotifySendMqtt(reFailureNotifier* notifier, bool notify_alert, const char* object, notify_state_t state, int32_t value, time_t time_failure, time_t time_state)
    {
      if (value == 0) {
        if (state == FNS_OK) {
          if (_notifyTgMqtt) {
            return statesNotifySend(notify_alert, CONFIG_MESSAGE_TG_MQTT_CONN_OK, object, state, time_failure, time_state);
          };
        } else {
          if (_notifyTgMqtt) {
            return statesNotifySend(notify_alert, CONFIG_MESSAGE_TG_MQTT_CONN_LOST, object, state, time_failure, time_state);
          };
        };
      } else if (value == 1) {
        if (_notifyTgMqtt) {
          return statesNotifySend(notify_alert, CONFIG_MESSAGE_TG_MQTT_CONN_FAILED, object, state, time_failure, time_state);
        };
      } else if (value == 2) {
        if (_notifyTgMqtt) {
          return statesNotifySend(notify_alert, CONFIG_MESSAGE_TG_MQTT_SERVER_CHANGE_PRIMARY, object, state, time_failure, time_state);
        };
      } else if (value == 3) {
        if (_notifyTgMqtt) {
          return statesNotifySend(notify_alert, CONFIG_MESSAGE_TG_MQTT_SERVER_CHANGE_RESERVED, object, state, time_failure, time_state);
        };
      };
      return false;
    }

    #if (defined(CONFIG_MQTT1_TYPE) && CONFIG_MQTT1_PING_CHECK) || (defined(CONFIG_MQTT2_TYPE) && CONFIG_MQTT2_PING_CHECK)
    static bool statesNotifySendMqttPing(reFailureNotifier* notifier, bool notify_alert, const char* object, notify_state_t state, int32_t value, time_t time_failure, time_t time_state)
    {
      if (_notifyTgMqtt) {
        return statesNotifySendHost(notifier, notify_alert, object, state, value, time_failure, time_state);
      };
      return false;
    }
    #endif // CONFIG_MQTTx_PING_CHECK
  #else
    static bool statesNotifySendMqtt(reFailureNotifier* notifier, bool notify_alert, const char* object, notify_state_t state, int32_t value, time_t time_failure, time_t time_state)
    {
      if (value == 0) {
        if (state == FNS_OK) {
          return statesNotifySend(notify_alert, CONFIG_MESSAGE_TG_MQTT_CONN_OK, object, state, time_failure, time_state);
        } else {
          return statesNotifySend(notify_alert, CONFIG_MESSAGE_TG_MQTT_CONN_LOST, object, state, time_failure, time_state);
        };
      } else if (value == 1) {
        return statesNotifySend(notify_alert, CONFIG_MESSAGE_TG_MQTT_CONN_FAILED, object, state, time_failure, time_state);
      } else if (value == 2) {
        return statesNotifySend(notify_alert, CONFIG_MESSAGE_TG_MQTT_SERVER_CHANGE_PRIMARY, object, state, time_failure, time_state);
      } else if (value == 3) {
        return statesNotifySend(notify_alert, CONFIG_MESSAGE_TG_MQTT_SERVER_CHANGE_RESERVED, object, state, time_failure, time_state);
      };
      return false;
    }
  #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE

  reFailureNotifier notifierMqtt("mqtt", CONFIG_NOTIFY_TELEGRAM_ALERT_MQTT_STATUS, 
    FNK_AUTO, &_statesNotifyDelayFailure, statesNotifySendMqtt);
#endif // CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS

// --  MQTT Ping ---------------------------------------------------------------------------------------------------------
#if defined(CONFIG_MQTT1_TYPE) && CONFIG_MQTT1_PING_CHECK && (CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
  #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
    reFailureNotifier notifierMqttPing1(CONFIG_MQTT1_HOST, CONFIG_NOTIFY_TELEGRAM_ALERT_MQTT_STATUS, 
      FNK_AUTO, &_statesNotifyDelayFailure, statesNotifySendMqttPing);
  #else
    reFailureNotifier notifierMqttPing1(CONFIG_MQTT1_HOST, CONFIG_NOTIFY_TELEGRAM_ALERT_MQTT_STATUS, 
      FNK_AUTO, &_statesNotifyDelayFailure, statesNotifySendHost);
  #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
#endif // CONFIG_MQTT1_PING_CHECK

#if defined(CONFIG_MQTT2_TYPE) && CONFIG_MQTT2_PING_CHECK && (CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
  #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
    reFailureNotifier notifierMqttPing2(CONFIG_MQTT2_HOST, CONFIG_NOTIFY_TELEGRAM_ALERT_MQTT_STATUS, 
      FNK_AUTO, &_statesNotifyDelayFailure, statesNotifySendMqttPing);
  #else
    reFailureNotifier notifierMqttPing2(CONFIG_MQTT2_HOST, CONFIG_NOTIFY_TELEGRAM_ALERT_MQTT_STATUS, 
      FNK_AUTO, &_statesNotifyDelayFailure, statesNotifySendHost);
  #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
#endif // CONFIG_MQTT2_PING_CHECK

// -- OpenMon ------------------------------------------------------------------------------------------------------------
#if CONFIG_OPENMON_ENABLE && (CONFIG_NOTIFY_TELEGRAM_OPENMON_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
  #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
    static uint8_t _notifyTgOpenMon = CONFIG_NOTIFY_TELEGRAM_OPENMON_STATUS;
    static bool statesNotifySendOpenMon(reFailureNotifier* notifier, bool notify_alert, const char* object, notify_state_t state, int32_t value, time_t time_failure, time_t time_state)
    {
      if (_notifyTgOpenMon) {
        return statesNotifySendHost(notifier, notify_alert, object, state, value, time_failure, time_state);
      };
      return false;
    }
  #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
  
  reFailureNotifier notifierOpenMon("open-monitoring.online", CONFIG_NOTIFY_TELEGRAM_ALERT_OPENMON_STATUS, 
    FNK_AUTO, &_statesNotifyDelayFailure, 
    #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
      statesNotifySendOpenMon);
    #else
      statesNotifySendHost);
    #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
#endif // CONFIG_OPENMON_ENABLE && CONFIG_NOTIFY_TELEGRAM_OPENMON_STATUS

// --  ThingSpeak --------------------------------------------------------------------------------------------------------
#if CONFIG_THINGSPEAK_ENABLE && (CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
  #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
    static uint8_t _notifyTgThingSpeak = CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_STATUS;
    static bool statesNotifySendThingSpeak(reFailureNotifier* notifier, bool notify_alert, const char* object, notify_state_t state, int32_t value, time_t time_failure, time_t time_state)
    {
      if (_notifyTgThingSpeak) {
        return statesNotifySendHost(notifier, notify_alert, object, state, value, time_failure, time_state);
      };
      return false;
    }
  #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
  
  reFailureNotifier notifierThingSpeak("thingspeak.com", CONFIG_NOTIFY_TELEGRAM_ALERT_THINGSPEAK_STATUS, 
    FNK_AUTO, &_statesNotifyDelayFailure, 
    #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
      statesNotifySendThingSpeak);
    #else
      statesNotifySendHost);
    #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
#endif // CONFIG_THINGSPEAK_ENABLE && CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_STATUS

// --  Locks -------------------------------------------------------------------------------------------------------------
static void statesNotifyInetAvailable(bool setState)
{
  rlog_d(logTAG, "Sending notifications about the resumption of Internet access");

  #if ((CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE > 0) || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
    if (setState) {
      notifierInet.setState(FNS_OK, time(nullptr), nullptr);
    };
  #endif // CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE

  #if (CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
    notifierMqtt.unlock();
  #endif // CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS

  #if defined(CONFIG_MQTT1_TYPE) && CONFIG_MQTT1_PING_CHECK && (CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
    notifierMqttPing1.unlock();
  #endif // CONFIG_MQTT1_PING_CHECK

  #if defined(CONFIG_MQTT2_TYPE) && CONFIG_MQTT2_PING_CHECK && (CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
    notifierMqttPing2.unlock();
  #endif // CONFIG_MQTT2_PING_CHECK

  #if CONFIG_OPENMON_ENABLE && (CONFIG_NOTIFY_TELEGRAM_OPENMON_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
    notifierOpenMon.unlock();
  #endif // CONFIG_OPENMON_ENABLE && CONFIG_NOTIFY_TELEGRAM_OPENMON_STATUS

  #if CONFIG_THINGSPEAK_ENABLE && (CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
    notifierThingSpeak.unlock();
  #endif // CONFIG_THINGSPEAK_ENABLE && CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_STATUS
}

static void statesNotifyInetUnavailable(bool setState, time_t timeState)
{
  rlog_d(logTAG, "Sending notifications about the unavailability of the Internet");

  #if CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE > 0
    if (setState) {
      notifierInet.setState(FNS_FAILURE, timeState, nullptr);
    };
  #endif // CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE

  #if defined(CONFIG_MQTT1_TYPE) && CONFIG_MQTT1_PING_CHECK && (CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
    notifierMqttPing1.lock();
  #endif // CONFIG_MQTT1_PING_CHECK

  #if defined(CONFIG_MQTT2_TYPE) && CONFIG_MQTT2_PING_CHECK && (CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
    notifierMqttPing2.lock();
  #endif // CONFIG_MQTT2_PING_CHECK

  #if (CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
    notifierMqtt.lock();
  #endif // CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS

  #if CONFIG_OPENMON_ENABLE && (CONFIG_NOTIFY_TELEGRAM_OPENMON_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
    notifierOpenMon.lock();
  #endif // CONFIG_OPENMON_ENABLE && CONFIG_NOTIFY_TELEGRAM_OPENMON_STATUS

  #if CONFIG_THINGSPEAK_ENABLE && (CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
    notifierThingSpeak.lock();
  #endif // CONFIG_THINGSPEAK_ENABLE && CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_STATUS
}

static void statesNotifyWiFiAvailable(bool setState)
{
  rlog_d(logTAG, "Sending wifi connect notifications");

  #if CONFIG_NOTIFY_TELEGRAM_WIFI_STATUS
    if (setState) {
      notifierWifi.setState(FNS_OK, time(nullptr), nullptr);
    };
  #endif // CONFIG_NOTIFY_TELEGRAM_WIFI_STATUS

  #if CONFIG_PINGER_ENABLE && ((CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE > 0) || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
    notifierInet.unlock();
  #endif // CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE

  statesNotifyInetAvailable(false);
}

static void statesNotifyWiFiUnavailable(bool setState)
{
  rlog_d(logTAG, "Sending wifi disconnect notifications");

  #if CONFIG_NOTIFY_TELEGRAM_WIFI_STATUS
    if (setState) {
      notifierWifi.setState(FNS_FAILURE, time(nullptr), nullptr);
    };
  #endif // CONFIG_NOTIFY_TELEGRAM_WIFI_STATUS

  #if CONFIG_PINGER_ENABLE && ((CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE > 0) || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
    notifierInet.lock();
  #endif // CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE

  statesNotifyInetUnavailable(false, 0);
}

// -- Parameters ---------------------------------------------------------------------------------------------------------
#if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
  static uint8_t _notifyTgSensors = CONFIG_NOTIFY_TELEGRAM_SENSOR_STATE;
  static uint8_t _notifyTgMqttErrors = CONFIG_NOTIFY_TELEGRAM_MQTT_ERRORS;
  #if CONFIG_SILENT_MODE_ENABLE
    static uint8_t _notifyTgSilentMode = CONFIG_NOTIFY_TELEGRAM_SILENT_MODE;
  #endif // CONFIG_SILENT_MODE_ENABLE

  static void statesNotifyRegisterParameters()
  {
    paramsGroupHandle_t pgNotify = paramsRegisterGroup(nullptr, 
      CONFIG_STATES_NOTIFY_PGROUP_ROOT_KEY, CONFIG_STATES_NOTIFY_PGROUP_ROOT_TOPIC, CONFIG_STATES_NOTIFY_PGROUP_ROOT_FRIENDLY);

    paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U32, nullptr, pgNotify,
      CONFIG_NOTIFY_TELEGRAM_MINIMUM_FAILURE_TIME_KEY, CONFIG_NOTIFY_TELEGRAM_MINIMUM_FAILURE_TIME_FRIENDLY,
      CONFIG_MQTT_PARAMS_QOS, (void*)&_statesNotifyDelayFailure);

    paramsSetLimitsU8(
      paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgNotify,
        CONFIG_NOTIFY_TELEGRAM_WIFI_KEY, CONFIG_NOTIFY_TELEGRAM_WIFI_FRIENDLY,
        CONFIG_MQTT_PARAMS_QOS, (void*)&_notifyTgWifi),
      0, 1);
    paramsSetLimitsU8(
      paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgNotify,
        CONFIG_NOTIFY_TELEGRAM_INET_KEY, CONFIG_NOTIFY_TELEGRAM_INET_FRIENDLY,
        CONFIG_MQTT_PARAMS_QOS, (void*)&_notifyTgInet),
      0, 2);
    paramsSetLimitsU8(
      paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgNotify,
        CONFIG_NOTIFY_TELEGRAM_MQTT_KEY, CONFIG_NOTIFY_TELEGRAM_MQTT_FRIENDLY,
        CONFIG_MQTT_PARAMS_QOS, (void*)&_notifyTgMqtt),
      0, 1);
    paramsSetLimitsU8(
      paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgNotify,
        CONFIG_NOTIFY_TELEGRAM_MQTT_ERRORS_KEY, CONFIG_NOTIFY_TELEGRAM_MQTT_ERRORS_FRIENDLY,
        CONFIG_MQTT_PARAMS_QOS, (void*)&_notifyTgMqttErrors),
      0, 1);
    #if CONFIG_OPENMON_ENABLE
      paramsSetLimitsU8(
        paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgNotify,
          CONFIG_NOTIFY_TELEGRAM_OPENMON_KEY, CONFIG_NOTIFY_TELEGRAM_OPENMON_FRIENDLY,
          CONFIG_MQTT_PARAMS_QOS, (void*)&_notifyTgOpenMon),
        0, 1);
    #endif // CONFIG_OPENMON_ENABLE
    #if CONFIG_THINGSPEAK_ENABLE
      paramsSetLimitsU8(
        paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgNotify,
          CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_KEY, CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_FRIENDLY,
          CONFIG_MQTT_PARAMS_QOS, (void*)&_notifyTgThingSpeak),
        0, 1);
    #endif // CONFIG_THINGSPEAK_ENABLE
    paramsSetLimitsU8(
      paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgNotify,
        CONFIG_NOTIFY_TELEGRAM_SENSOR_KEY, CONFIG_NOTIFY_TELEGRAM_SENSOR_FRIENDLY,
        CONFIG_MQTT_PARAMS_QOS, (void*)&_notifyTgSensors),
      0, 1);
    #if CONFIG_SILENT_MODE_ENABLE
    paramsSetLimitsU8(
      paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgNotify,
        CONFIG_NOTIFY_TELEGRAM_SILENT_MODE_KEY, CONFIG_NOTIFY_TELEGRAM_SILENT_MODE_FRIENDLY,
        CONFIG_MQTT_PARAMS_QOS, (void*)&_notifyTgSilentMode),
      0, 1);
    #endif // CONFIG_SILENT_MODE_ENABLE
  }
#endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE

#endif // CONFIG_ENABLE_STATE_NOTIFICATIONS 

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- Event handlers ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static void statesEventCheckSystemStarted()
{
  if (!statesCheck(SYSTEM_STARTED, false)) {
    rlog_i(logTAG, "Check system started: wifi=%d, internet=%d, time=%d, mqtt=%d", 
      statesCheck(WIFI_STA_CONNECTED, false), statesCheck(INET_AVAILABLED, false), 
      (statesCheck(TIME_SNTP_SYNC_OK, false) || statesCheck(TIME_RTC_ENABLED, false)), 
      statesCheck(MQTT_CONNECTED, false));

    if ((statesCheck(TIME_SNTP_SYNC_OK, false) || statesCheck(TIME_RTC_ENABLED, false)) 
     && statesCheck(WIFI_STA_CONNECTED, false) 
     && statesCheck(INET_AVAILABLED, false) 
     && statesCheck(MQTT_CONNECTED, false)) {
       statesSet(SYSTEM_STARTED);
       eventLoopPostSystem(RE_SYS_STARTED, RE_SYS_SET, false, 0);
       #if CONFIG_ENABLE_STATE_NOTIFICATIONS
         tgSend(TG_MAIN, false, CONFIG_TELEGRAM_DEVICE, CONFIG_MESSAGE_TG_VERSION, 
           APP_VERSION, getResetReason(), getResetReasonRtc(0), getResetReasonRtc(1));
       #endif // CONFIG_ENABLE_STATE_NOTIFICATIONS
    };
  };
}

static void statesEventHandlerSystem(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  re_system_event_data_t* data = (re_system_event_data_t*)event_data;
  if (data) {
    // OTA
    if (event_id == RE_SYS_OTA) {
      statesSetBit(SYSTEM_OTA, data->type != RE_SYS_CLEAR);
    }
    // Error
    else if (event_id == RE_SYS_ERROR) {
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE_MODE, event_base, "RE_SYS_ERROR", data->type);
      statesSetError(ERR_GENERAL, data->type != RE_SYS_CLEAR);
    }
    // Telegram error
    else if (event_id == RE_SYS_TELEGRAM_ERROR) {
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE_MODE, event_base, "RE_SYS_TELEGRAM_ERROR", data->type);
      statesSetError(ERR_TELEGRAM, data->type != RE_SYS_CLEAR);
    }
    // OpenMon
    else if (event_id == RE_SYS_OPENMON_ERROR) {
      if (data->type == RE_SYS_CLEAR) {
        statesClearErrors(ERR_OPENMON);
        #if CONFIG_OPENMON_ENABLE && CONFIG_ENABLE_STATE_NOTIFICATIONS && (CONFIG_NOTIFY_TELEGRAM_OPENMON_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
          notifierOpenMon.setState(FNS_OK, time(nullptr), nullptr);
        #endif // CONFIG_OPENMON_ENABLE
      } else {
        statesSetErrors(ERR_OPENMON);
        #if CONFIG_OPENMON_ENABLE && CONFIG_ENABLE_STATE_NOTIFICATIONS && (CONFIG_NOTIFY_TELEGRAM_OPENMON_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
          notifierOpenMon.setState(FNS_FAILURE, (time_t)data->data, nullptr);
        #endif // CONFIG_OPENMON_ENABLE
      };
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE_MODE, event_base, "RE_SYS_OPENMON_ERROR", data->type);
    }
    // ThingSpeak
    else if (event_id == RE_SYS_THINGSPEAK_ERROR) {
      if (data->type == RE_SYS_CLEAR) {
        statesClearErrors(ERR_THINGSPEAK);
        #if CONFIG_THINGSPEAK_ENABLE && CONFIG_ENABLE_STATE_NOTIFICATIONS && (CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
          notifierThingSpeak.setState(FNS_OK, time(nullptr), nullptr);
        #endif // CONFIG_THINGSPEAK_ENABLE
      } else {
        statesSetErrors(ERR_THINGSPEAK);
        #if CONFIG_THINGSPEAK_ENABLE && CONFIG_ENABLE_STATE_NOTIFICATIONS && (CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
          notifierThingSpeak.setState(FNS_FAILURE, (time_t)data->data, nullptr);
        #endif // CONFIG_THINGSPEAK_ENABLE
      };
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE_MODE, event_base, "RE_SYS_THINGSPEAK_ERROR", data->type);
    };
  };
}

static void statesEventHandlerTime(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  switch (event_id) {
    // Received time from hardware real time clock
    case RE_TIME_RTC_ENABLED:
      statesSet(TIME_RTC_ENABLED);
      statesEventCheckSystemStarted();
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_TIME_RTC_ENABLED");
      break;

    // Received time from NTP server
    case RE_TIME_SNTP_SYNC_OK:
      statesSet(TIME_SNTP_SYNC_OK);
      statesEventCheckSystemStarted();
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "TIME_SNTP_SYNC_OK");
      break;

    #if CONFIG_SILENT_MODE_ENABLE
    // Silent mode on
    case RE_TIME_SILENT_MODE_ON:
      statesSet(TIME_SILENT_MODE);
      ledSysSetEnabled(false);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_TIME_SILENT_MODE_ON");
      #if CONFIG_TELEGRAM_ENABLE && (CONFIG_NOTIFY_TELEGRAM_SILENT_MODE || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
        #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
        if (_notifyTgSilentMode) {
        #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
          tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_SILENT_MODE, CONFIG_TELEGRAM_DEVICE, CONFIG_MESSAGE_TG_SILENT_MODE_ON);
        #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
        };
        #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
      #endif // CONFIG_NOTIFY_TELEGRAM_SILENT_MODE
      break;
    // Silent mode off
    case RE_TIME_SILENT_MODE_OFF:
      statesClear(TIME_SILENT_MODE);
      ledSysSetEnabled(true);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_TIME_SILENT_MODE_OFF");
      #if CONFIG_TELEGRAM_ENABLE && (CONFIG_NOTIFY_TELEGRAM_SILENT_MODE || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
        #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
        if (_notifyTgSilentMode) {
        #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
          tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_SILENT_MODE, CONFIG_TELEGRAM_DEVICE, CONFIG_MESSAGE_TG_SILENT_MODE_OFF);
        #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
        };
        #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
      #endif // CONFIG_NOTIFY_TELEGRAM_SILENT_MODE
      break;
    #endif // CONFIG_SILENT_MODE_ENABLE

    // Ignore...
    default:
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "IGNORED");
      break;
  };
}

static void statesEventHandlerWiFi(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  switch (event_id) {
    case RE_WIFI_STA_INIT:
      statesClear(WIFI_STA_STARTED | WIFI_STA_CONNECTED | INET_AVAILABLED | INET_SLOWDOWN);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_WIFI_STA_INIT");
      break;

    case RE_WIFI_STA_STARTED:
      statesSet(WIFI_STA_STARTED);
      statesClear(WIFI_STA_CONNECTED | INET_AVAILABLED | INET_SLOWDOWN);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_WIFI_STA_STARTED");
      break;

    case RE_WIFI_STA_GOT_IP:
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_WIFI_STA_GOT_IP");
      #if CONFIG_PINGER_ENABLE
        statesClear(INET_AVAILABLED | INET_SLOWDOWN);
      #else
        statesSet(WIFI_STA_CONNECTED | INET_AVAILABLED);
        statesClear(INET_SLOWDOWN);
        eventLoopPost(RE_WIFI_EVENTS, RE_WIFI_STA_PING_OK, nullptr, 0, portMAX_DELAY);
        #if CONFIG_ENABLE_STATE_NOTIFICATIONS
          statesNotifyWiFiAvailable(true);
        #endif // CONFIG_ENABLE_STATE_NOTIFICATIONS
      #endif // CONFIG_PINGER_ENABLE
      statesEventCheckSystemStarted();
      break;

    #if CONFIG_PINGER_ENABLE
      case RE_WIFI_STA_PING_OK:
        if (!statesCheck(WIFI_STA_CONNECTED, false)) {
          statesSet(WIFI_STA_CONNECTED);
          #if CONFIG_ENABLE_STATE_NOTIFICATIONS
            statesNotifyWiFiAvailable(true);
          #endif // CONFIG_ENABLE_STATE_NOTIFICATIONS
        };
        statesEventCheckSystemStarted();
        break;
    #endif // CONFIG_PINGER_ENABLE

    case RE_WIFI_STA_DISCONNECTED:
    case RE_WIFI_STA_STOPPED:
      #if CONFIG_ENABLE_STATE_NOTIFICATIONS 
        if (statesCheck(WIFI_STA_CONNECTED, false)) {
          statesNotifyWiFiUnavailable(true);
        };
      #endif // CONFIG_ENABLE_STATE_NOTIFICATIONS
      statesClear(WIFI_STA_CONNECTED | INET_AVAILABLED | INET_SLOWDOWN);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_WIFI_STA_DISCONNECTED / RE_WIFI_STA_STOPPED");
      break;

    default:
      // Ignore...
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "IGNORED");
      break;
  };
}

#if CONFIG_PINGER_ENABLE

static void statesEventHandlerPing(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  switch (event_id) {
    case RE_PING_INET_AVAILABLE: 
      statesSet(INET_AVAILABLED);
      statesClear(INET_SLOWDOWN);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_PING_INET_AVAILABLE");
      #if CONFIG_ENABLE_STATE_NOTIFICATIONS
        if (statesCheck(WIFI_STA_CONNECTED, false)) {
          statesNotifyInetAvailable(true);
        };
      #endif // CONFIG_ENABLE_STATE_NOTIFICATIONS
      eventLoopPost(RE_WIFI_EVENTS, RE_WIFI_STA_PING_OK, nullptr, 0, portMAX_DELAY);
      statesEventCheckSystemStarted();
      break;

    case RE_PING_INET_SLOWDOWN:
      statesSet(INET_AVAILABLED | INET_SLOWDOWN);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_PING_INET_SLOWDOWN");
      #if CONFIG_ENABLE_STATE_NOTIFICATIONS && ((CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE > 1) || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
        if (event_data) {
          ping_inet_data_t* data = (ping_inet_data_t*)event_data;
          notifierInet.setState(FNS_SLOWDOWN, data->time_unavailable, nullptr);
        } else {
          notifierInet.setState(FNS_SLOWDOWN, time(nullptr), nullptr);
        };
      #endif // CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE
      eventLoopPost(RE_WIFI_EVENTS, RE_WIFI_STA_PING_OK, nullptr, 0, portMAX_DELAY);
      statesEventCheckSystemStarted();
      break;

    case RE_PING_INET_UNAVAILABLE:
      statesClear(INET_AVAILABLED | INET_SLOWDOWN);
      eventLoopPost(RE_WIFI_EVENTS, RE_WIFI_STA_PING_FAILED, nullptr, 0, portMAX_DELAY);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_PING_INET_UNAVAILABLE");
      #if CONFIG_ENABLE_STATE_NOTIFICATIONS
        if (statesCheck(WIFI_STA_CONNECTED, false)) {
          if (event_data) {
            ping_inet_data_t* data = (ping_inet_data_t*)event_data;
            statesNotifyInetUnavailable(true, data->time_unavailable);
          } else {
            statesNotifyInetUnavailable(true, time(nullptr));
          };
        };
      #endif // CONFIG_ENABLE_STATE_NOTIFICATIONS
      break;

    case RE_PING_MQTT1_AVAILABLE:
      statesSet(MQTT_1_ENABLED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_PING_MQTT1_AVAILABLE");
      #if defined(CONFIG_MQTT1_TYPE) && CONFIG_ENABLE_STATE_NOTIFICATIONS && CONFIG_MQTT1_PING_CHECK && (CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
        notifierMqttPing1.setState(FNS_OK, time(nullptr), nullptr);
      #endif // CONFIG_MQTT1_PING_CHECK
      break;

    case RE_PING_MQTT2_AVAILABLE:
      statesSet(MQTT_2_ENABLED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_PING_MQTT2_AVAILABLE");
      #if defined(CONFIG_MQTT2_TYPE) && CONFIG_ENABLE_STATE_NOTIFICATIONS && CONFIG_MQTT2_PING_CHECK && (CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
        notifierMqttPing2.setState(FNS_OK, time(nullptr), nullptr);
      #endif // CONFIG_MQTT2_PING_CHECK
      break;

    case RE_PING_MQTT1_UNAVAILABLE:
      statesClear(MQTT_1_ENABLED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_PING_MQTT1_UNAVAILABLE");
      #if defined(CONFIG_MQTT1_TYPE) && CONFIG_ENABLE_STATE_NOTIFICATIONS && CONFIG_MQTT1_PING_CHECK && (CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
        if (event_data) {
          ping_host_data_t* data = (ping_host_data_t*)event_data;
          notifierMqttPing1.setState(FNS_FAILURE, data->time_unavailable, nullptr);
        } else {
          notifierMqttPing1.setState(FNS_FAILURE, time(nullptr), nullptr);
        };         
      #endif // CONFIG_MQTT1_PING_CHECK
      break;

    case RE_PING_MQTT2_UNAVAILABLE:
      statesClear(MQTT_2_ENABLED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_PING_MQTT2_UNAVAILABLE");
      #if defined(CONFIG_MQTT2_TYPE) && CONFIG_ENABLE_STATE_NOTIFICATIONS && CONFIG_MQTT2_PING_CHECK && (CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
        if (event_data) {
          ping_host_data_t* data = (ping_host_data_t*)event_data;
          notifierMqttPing2.setState(FNS_FAILURE, data->time_unavailable, nullptr);
        } else {
          notifierMqttPing2.setState(FNS_FAILURE, time(nullptr), nullptr);
        };         
      #endif // CONFIG_MQTT2_PING_CHECK
      break;

    default:
      // Ignore...
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "IGNORED");
      break;
  };
}
#endif // CONFIG_PINGER_ENABLE

static void statesEventHandlerMqtt(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  switch (event_id) {
    case RE_MQTT_CONNECTED:
      statesSet(MQTT_CONNECTED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_MQTT_CONNECTED");
      if (event_data) {
        re_mqtt_event_data_t* data = (re_mqtt_event_data_t*)event_data;
        statesSetBit(MQTT_PRIMARY, data->primary);
        statesSetBit(MQTT_LOCAL, data->local);
        #if CONFIG_ENABLE_STATE_NOTIFICATIONS && (CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
          notifierMqtt.setState(FNS_OK, time(nullptr), malloc_stringf("%s:%d", data->host, data->port));
        #endif // CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS
        statesEventCheckSystemStarted();
      };
      break;

    case RE_MQTT_CONN_LOST:
      statesClear(MQTT_CONNECTED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_MQTT_CONN_LOST");
      if (event_data) {
        re_mqtt_event_data_t* data = (re_mqtt_event_data_t*)event_data;
        #if CONFIG_ENABLE_STATE_NOTIFICATIONS && (CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
          notifierMqtt.setState(FNS_FAILURE, time(nullptr), malloc_stringf("%s:%d", data->host, data->port));
        #endif // CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS
      };
      break;

    case RE_MQTT_CONN_FAILED:
      statesClear(MQTT_CONNECTED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_MQTT_CONN_FAILED");
      if (event_data) {
        re_mqtt_event_data_t* data = (re_mqtt_event_data_t*)event_data;
        #if CONFIG_ENABLE_STATE_NOTIFICATIONS && (CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
          notifierMqtt.sendExNotify(FNS_FAILURE, time(nullptr), 1, nullptr);
        #endif // CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS
      };
      break;

    case RE_MQTT_SERVER_PRIMARY:
      // statesSet(MQTT_PRIMARY);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_MQTT_SERVER_PRIMARY");
      #if CONFIG_ENABLE_STATE_NOTIFICATIONS && (CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
        notifierMqtt.sendExNotify(FNS_FAILURE, time(nullptr), 2, nullptr);
      #endif // CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS
      break;

    case RE_MQTT_SERVER_RESERVED:
      // statesClear(MQTT_PRIMARY);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_MQTT_SERVER_RESERVED");
      #if CONFIG_ENABLE_STATE_NOTIFICATIONS && (CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
        notifierMqtt.sendExNotify(FNS_FAILURE, time(nullptr), 3, nullptr);
      #endif // CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS
      break;

    case RE_MQTT_ERROR:
      statesSetErrors(ERR_MQTT);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_MQTT_ERROR");
      #if CONFIG_ENABLE_STATE_NOTIFICATIONS && (CONFIG_NOTIFY_TELEGRAM_MQTT_ERRORS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
        #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
        if (_notifyTgMqttErrors) {
        #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
          if (event_data) {
            char* error = (char*)event_data;
            tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_MQTT_ERRORS, 
              CONFIG_TELEGRAM_DEVICE, 
              CONFIG_MESSAGE_TG_MQTT_ERROR, error);
          };
        #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
        };
        #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
      #endif // CONFIG_NOTIFY_TELEGRAM_MQTT_ERRORS
      break;

    case RE_MQTT_ERROR_CLEAR:
      statesClearErrors(ERR_MQTT);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_MQTT_ERROR_CLEAR");
      break;

    default:
      // Ignore...
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "IGNORED");
      break;
  };
}

static void statesEventHandlerSensor(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_SENSOR_STATUS_CHANGED");

  sensor_event_status_t* data = (sensor_event_status_t*)event_data;
  if (data) {
    // Set new status
    switch (data->sensor_id) {
      case 1:
        statesSetError(ERR_SENSOR_1, data->new_status != SENSOR_STATUS_OK);
        break;
      case 2:
        statesSetError(ERR_SENSOR_2, data->new_status != SENSOR_STATUS_OK);
        break;
      case 3:
        statesSetError(ERR_SENSOR_3, data->new_status != SENSOR_STATUS_OK);
        break;
      case 4:
        statesSetError(ERR_SENSOR_4, data->new_status != SENSOR_STATUS_OK);
        break;
      case 5:
        statesSetError(ERR_SENSOR_5, data->new_status != SENSOR_STATUS_OK);
        break;
      case 6:
        statesSetError(ERR_SENSOR_6, data->new_status != SENSOR_STATUS_OK);
        break;
      case 7:
        statesSetError(ERR_SENSOR_7, data->new_status != SENSOR_STATUS_OK);
        break;
      default:
        statesSetError(ERR_SENSOR_0, data->new_status != SENSOR_STATUS_OK);
        break;
    };

    #if CONFIG_ENABLE_STATE_NOTIFICATIONS && (CONFIG_NOTIFY_TELEGRAM_SENSOR_STATE || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
      // Sensor status change notification
      #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
      if (_notifyTgSensors) {
      #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
        rSensor* sensor = (rSensor*)data->sensor;
        switch ((sensor_status_t)data->new_status) {
          case SENSOR_STATUS_NAN:
            tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_SENSOR_STATE, 
              CONFIG_TELEGRAM_DEVICE, 
              CONFIG_MESSAGE_TG_SENSOR_STATE_NAN, sensor->getName());
            break;
          case SENSOR_STATUS_OK:
            tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_SENSOR_STATE, 
              CONFIG_TELEGRAM_DEVICE, 
              CONFIG_MESSAGE_TG_SENSOR_STATE_OK, sensor->getName());
            break;
          case SENSOR_STATUS_TIMEOUT:
            tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_SENSOR_STATE, 
              CONFIG_TELEGRAM_DEVICE, 
              CONFIG_MESSAGE_TG_SENSOR_STATE_TIMEOUT, sensor->getName());
            break;
          case SENSOR_STATUS_CAL_ERROR:
            tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_SENSOR_STATE, 
              CONFIG_TELEGRAM_DEVICE, 
              CONFIG_MESSAGE_TG_SENSOR_STATE_CALIBRATION, sensor->getName());
            break;
          case SENSOR_STATUS_CRC_ERROR:
            tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_SENSOR_STATE, 
              CONFIG_TELEGRAM_DEVICE, 
              CONFIG_MESSAGE_TG_SENSOR_STATE_CRC_ERROR, sensor->getName());
            break;
          case SENSOR_STATUS_ERROR:
            tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_SENSOR_STATE, 
              CONFIG_TELEGRAM_DEVICE, 
              CONFIG_MESSAGE_TG_SENSOR_STATE_ERROR, sensor->getName());
            break;
          default:
            tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_SENSOR_STATE, 
              CONFIG_TELEGRAM_DEVICE, 
              CONFIG_MESSAGE_TG_SENSOR_STATE_UNKNOWN_ERROR, sensor->getName());
            break;
        };
      #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
      };
      #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
    #endif // CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_SENSOR_STATE
  };
}

bool statesEventHandlerRegister()
{
  bool ret = eventHandlerRegister(RE_SYSTEM_EVENTS, ESP_EVENT_ANY_ID, &statesEventHandlerSystem, nullptr)
          && eventHandlerRegister(RE_TIME_EVENTS, ESP_EVENT_ANY_ID, &statesEventHandlerTime, nullptr)
          && eventHandlerRegister(RE_WIFI_EVENTS, ESP_EVENT_ANY_ID, &statesEventHandlerWiFi, nullptr)
          && eventHandlerRegister(RE_MQTT_EVENTS, ESP_EVENT_ANY_ID, &statesEventHandlerMqtt, nullptr)
          #if CONFIG_PINGER_ENABLE
            && eventHandlerRegister(RE_PING_EVENTS, ESP_EVENT_ANY_ID, &statesEventHandlerPing, nullptr)
          #endif // CONFIG_PINGER_ENABLE
          && eventHandlerRegister(RE_SENSOR_EVENTS, ESP_EVENT_ANY_ID, &statesEventHandlerSensor, nullptr);
  if (ret) {
    #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
      statesNotifyRegisterParameters();
    #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
    rlog_d(logTAG, "System states event handlers registered");
  };
  return ret;
}

void statesEventHandlerUnregister()
{
  eventHandlerUnregister(RE_SYSTEM_EVENTS, ESP_EVENT_ANY_ID, &statesEventHandlerSystem);
  eventHandlerUnregister(RE_TIME_EVENTS, ESP_EVENT_ANY_ID, &statesEventHandlerTime);
  eventHandlerUnregister(RE_WIFI_EVENTS, ESP_EVENT_ANY_ID, &statesEventHandlerWiFi);
  eventHandlerUnregister(RE_MQTT_EVENTS, ESP_EVENT_ANY_ID, &statesEventHandlerMqtt);
  #if CONFIG_PINGER_ENABLE
    eventHandlerUnregister(RE_PING_EVENTS, ESP_EVENT_ANY_ID, &statesEventHandlerPing);
  #endif // CONFIG_PINGER_ENABLE
  eventHandlerUnregister(RE_SENSOR_EVENTS, ESP_EVENT_ANY_ID, &statesEventHandlerSensor);
  rlog_d(logTAG, "System states event handlers unregistered");
}