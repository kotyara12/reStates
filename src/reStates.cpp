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


static EventGroupHandle_t _evgStates = nullptr;
static EventGroupHandle_t _evgErrors = nullptr;
static time_t _timeLostWiFi = 0;

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

#if CONFIG_TELEGRAM_ENABLE

// Sending a notification when access to something is restored
void tgNotifyOnRecoveryAccess(bool msg_notify, const char* msg_template, const char* msg_object, time_t time_unavailable, time_t minimal_failure)
{
  if (time_unavailable > 1000000000) {
    time_t time_restore = time(nullptr);
    time_t loss_period = time_restore - time_unavailable;
    if (loss_period > minimal_failure) {
      uint16_t loss_h = loss_period / 3600;
      uint16_t loss_m = loss_period % 3600 / 60;
      uint16_t loss_s = loss_period % 3600 % 60;
      char* s_time_restore = malloc_timestr(CONFIG_FORMAT_DTS, time_restore);
      char* s_time_unavailable = malloc_timestr(CONFIG_FORMAT_DTS, time_unavailable);
      if ((s_time_restore) && (s_time_unavailable)) {
        if (msg_object) {
          tgSend(TG_SERVICE, msg_notify, CONFIG_TELEGRAM_DEVICE, 
            msg_template, msg_object, s_time_unavailable, s_time_restore, loss_h, loss_m, loss_s);
        } else {
          tgSend(TG_SERVICE, msg_notify, CONFIG_TELEGRAM_DEVICE, 
            msg_template, s_time_unavailable, s_time_restore, loss_h, loss_m, loss_s);
        };
      };
      if (s_time_restore) free(s_time_restore);
      if (s_time_unavailable) free(s_time_unavailable);
    };
  };
}

#endif // CONFIG_TELEGRAM_ENABLE

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- Event handlers ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

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
      if (data->type != RE_SYS_CLEAR) {
        statesSetErrors(ERR_OPENMON);
        if (statesInetIsAvailabled()) {
          #if CONFIG_TELEGRAM_ENABLE & CONFIG_NOTIFY_TELEGRAM_OPENMON_STATUS
            tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_OPENMON_STATUS, 
              CONFIG_TELEGRAM_DEVICE,
              CONFIG_MESSAGE_TG_HOST_UNAVAILABLE, "open-monitoring.online");
          #endif // CONFIG_NOTIFY_TELEGRAM_OPENMON_STATUS
        };
      } else {
        statesClearErrors(ERR_OPENMON);
        if (data->data != 0) {
          #if CONFIG_TELEGRAM_ENABLE & CONFIG_NOTIFY_TELEGRAM_OPENMON_STATUS
            tgNotifyOnRecoveryAccess(
              CONFIG_NOTIFY_TELEGRAM_ALERT_OPENMON_STATUS, 
              CONFIG_MESSAGE_TG_HOST_AVAILABLE, "open-monitoring.online", 
              (time_t)data->data, 0);
          #endif // CONFIG_NOTIFY_TELEGRAM_OPENMON_STATUS
        };
      };
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE_MODE, event_base, "RE_SYS_OPENMON_ERROR", data->type);
    }
    // ThingSpeak
    else if (event_id == RE_SYS_THINGSPEAK_ERROR) {
      if (data->type != RE_SYS_CLEAR) {
        statesSetErrors(ERR_THINGSPEAK);
        if (statesInetIsAvailabled()) {
          #if CONFIG_TELEGRAM_ENABLE & CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_STATUS
            tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_THINGSPEAK_STATUS, 
              CONFIG_TELEGRAM_DEVICE,
              CONFIG_MESSAGE_TG_HOST_UNAVAILABLE, "thingspeak.com");
          #endif // CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_STATUS
        };
      } else {
        statesClearErrors(ERR_THINGSPEAK);
        if (data->data != 0) {
          #if CONFIG_TELEGRAM_ENABLE & CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_STATUS
            tgNotifyOnRecoveryAccess(
              CONFIG_NOTIFY_TELEGRAM_ALERT_THINGSPEAK_STATUS, 
              CONFIG_MESSAGE_TG_HOST_AVAILABLE, "thingspeak.com", 
              (time_t)data->data, 0);
          #endif // CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_STATUS
        };
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
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_TIME_RTC_ENABLED");
      break;

    // Received time from NTP server
    case RE_TIME_SNTP_SYNC_OK:
      statesSet(TIME_SNTP_SYNC_OK);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "TIME_SNTP_SYNC_OK");
      break;

    #if CONFIG_SILENT_MODE_ENABLE
    // Silent mode on
    case RE_TIME_SILENT_MODE_ON:
      statesSet(TIME_SILENT_MODE);
      ledSysSetEnabled(false);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_TIME_SILENT_MODE_ON");
      #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_SILENT_MODE
        tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_SILENT_MODE, CONFIG_TELEGRAM_DEVICE, CONFIG_MESSAGE_TG_SILENT_MODE_ON);
      #endif // CONFIG_NOTIFY_TELEGRAM_SILENT_MODE
      break;
    // Silent mode off
    case RE_TIME_SILENT_MODE_OFF:
      statesClear(TIME_SILENT_MODE);
      ledSysSetEnabled(true);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_TIME_SILENT_MODE_OFF");
      #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_SILENT_MODE
        tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_SILENT_MODE, CONFIG_TELEGRAM_DEVICE, CONFIG_MESSAGE_TG_SILENT_MODE_OFF);
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
      _timeLostWiFi = 0;
      statesClear(WIFI_STA_STARTED | WIFI_STA_CONNECTED | INET_AVAILABLED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_WIFI_STA_INIT");
      break;

    case RE_WIFI_STA_STARTED:
      statesSet(WIFI_STA_STARTED);
      statesClear(WIFI_STA_CONNECTED | INET_AVAILABLED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_WIFI_STA_STARTED");
      break;

    case RE_WIFI_STA_GOT_IP:
      statesSet(WIFI_STA_CONNECTED);
      statesClear(INET_AVAILABLED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_WIFI_STA_GOT_IP");
      break;
    
    case RE_WIFI_STA_PING_OK:
      statesSet(INET_AVAILABLED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_WIFI_STA_PING_OK");
      if (_timeLostWiFi == 0) {
        #if CONFIG_TELEGRAM_ENABLE & CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE
          if (event_data) {
            time_t* data = (time_t*)event_data;
            tgNotifyOnRecoveryAccess(
              CONFIG_NOTIFY_TELEGRAM_ALERT_INET_UNAVAILABLE, 
              CONFIG_MESSAGE_TG_INET_AVAILABLE, nullptr, 
              *data, CONFIG_NOTIFY_TELEGRAM_MINIMUM_FAILURE_TIME);
          };
        #endif // CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE
      } else {
        #if CONFIG_TELEGRAM_ENABLE & CONFIG_NOTIFY_TELEGRAM_WIFI_STATUS
          tgNotifyOnRecoveryAccess(
            CONFIG_NOTIFY_TELEGRAM_ALERT_WIFI_STATUS, 
            CONFIG_MESSAGE_TG_WIFI_AVAILABLE, wifiGetSSID(), 
            _timeLostWiFi, CONFIG_NOTIFY_TELEGRAM_MINIMUM_FAILURE_TIME);
        #endif // CONFIG_NOTIFY_TELEGRAM_WIFI_STATUS
        _timeLostWiFi = 0;
      };
      break;

    case RE_WIFI_STA_PING_FAILED:
      statesClear(INET_AVAILABLED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_WIFI_STA_PING_FAILED");
      break;

    case RE_WIFI_STA_DISCONNECTED:
    case RE_WIFI_STA_STOPPED:
      statesClear(WIFI_STA_CONNECTED | INET_AVAILABLED);
      if (_timeLostWiFi == 0) {
        _timeLostWiFi = time(nullptr);
      };
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_WIFI_STA_DISCONNECTED / RE_WIFI_STA_STOPPED");
      break;

    default:
      // Ignore...
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "IGNORED");
      break;
  };
}

static void statesEventHandlerMqtt(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS
  static bool _isFirstConnect = true;
  #endif // CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS

  switch (event_id) {
    case RE_MQTT_CONNECTED:
      statesSet(MQTT_CONNECTED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_MQTT_CONNECTED");
      if (event_data) {
        re_mqtt_event_data_t* data = (re_mqtt_event_data_t*)event_data;
        statesSetBit(MQTT_PRIMARY, data->primary);
        statesSetBit(MQTT_LOCAL, data->local);
        #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS
          if (_isFirstConnect) {
            _isFirstConnect = false;
          } else {
            if (statesInetIsAvailabled()) {
              tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_MQTT_STATUS, 
                CONFIG_TELEGRAM_DEVICE, 
                CONFIG_MESSAGE_TG_MQTT_CONN_OK, data->host, data->port);
            };
          };
        #endif // CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS
      };
      break;

    case RE_MQTT_CONN_LOST:
      statesClear(MQTT_CONNECTED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_MQTT_CONN_LOST");
      if (event_data) {
        re_mqtt_event_data_t* data = (re_mqtt_event_data_t*)event_data;
        #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS
          _isFirstConnect = false;
          if (statesInetIsAvailabled()) {
            tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_MQTT_STATUS, 
              CONFIG_TELEGRAM_DEVICE, 
              CONFIG_MESSAGE_TG_MQTT_CONN_LOST, data->host, data->port);
          };
        #endif // CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS
      };
      break;

    case RE_MQTT_CONN_FAILED:
      statesClear(MQTT_CONNECTED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_MQTT_CONN_FAILED");
      if (event_data) {
        re_mqtt_event_data_t* data = (re_mqtt_event_data_t*)event_data;
        #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS
          _isFirstConnect = false;
          if (statesInetIsAvailabled()) {
            tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_MQTT_STATUS, 
              CONFIG_TELEGRAM_DEVICE, 
              CONFIG_MESSAGE_TG_MQTT_CONN_FAILED, data->host, data->port);
          };
        #endif // CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS
      };
      break;

    case RE_MQTT_SERVER_PRIMARY:
      // statesSet(MQTT_PRIMARY);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_MQTT_SERVER_PRIMARY");
      #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS
        if (statesInetIsAvailabled()) {
          tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_MQTT_STATUS, 
            CONFIG_TELEGRAM_DEVICE, 
            CONFIG_MESSAGE_TG_MQTT_SERVER_CHANGE_PRIMARY);
        };
      #endif // CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS
      break;

    case RE_MQTT_SERVER_RESERVED:
      // statesClear(MQTT_PRIMARY);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_MQTT_SERVER_RESERVED");
      #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS
        if (statesInetIsAvailabled()) {
          tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_MQTT_STATUS, 
            CONFIG_TELEGRAM_DEVICE, 
            CONFIG_MESSAGE_TG_MQTT_SERVER_CHANGE_RESERVED);
        };
      #endif // CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS
      break;

    case RE_MQTT_ERROR:
      statesSetErrors(ERR_MQTT);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_MQTT_ERROR");
      #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_MQTT_ERRORS
        if (event_data) {
          char* error = (char*)event_data;
          tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_MQTT_ERRORS, 
            CONFIG_TELEGRAM_DEVICE, 
            CONFIG_MESSAGE_TG_MQTT_ERROR, error);
        };
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

#if CONFIG_PINGER_ENABLE
static void statesEventHandlerPing(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  switch (event_id) {
    case RE_PING_INET_AVAILABLE:
      // The status will be changed upon the RE_WIFI_STA_PING_OK event
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_PING_INET_AVAILABLE");
      break;

    case RE_PING_INET_UNAVAILABLE:
      // The status will be changed upon the RE_WIFI_STA_PING_FAILED event
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_PING_INET_UNAVAILABLE");
      break;

    case RE_PING_MQTT1_AVAILABLE:
      statesSet(MQTT_1_ENABLED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_PING_MQTT1_AVAILABLE");
      #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE
        if (statesInetIsAvailabled()) {
          ping_host_data_t* data = (ping_host_data_t*)event_data;
          if (data) {
            tgNotifyOnRecoveryAccess(
              CONFIG_NOTIFY_TELEGRAM_ALERT_INET_UNAVAILABLE, 
              CONFIG_MESSAGE_TG_HOST_AVAILABLE, data->host_name, 
              data->time_unavailable, 0);
          };
        };
      #endif // CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE
      break;

    case RE_PING_MQTT2_AVAILABLE:
      statesSet(MQTT_2_ENABLED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_PING_MQTT2_AVAILABLE");
      #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE
        if (statesInetIsAvailabled()) {
          ping_host_data_t* data = (ping_host_data_t*)event_data;
          if (data) {
            tgNotifyOnRecoveryAccess(
              CONFIG_NOTIFY_TELEGRAM_ALERT_INET_UNAVAILABLE, 
              CONFIG_MESSAGE_TG_HOST_AVAILABLE, data->host_name, 
              data->time_unavailable, 0);
          };
        };
      #endif // CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE
      break;

    case RE_PING_MQTT1_UNAVAILABLE:
      statesClear(MQTT_1_ENABLED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_PING_MQTT1_UNAVAILABLE");
      #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE
        if (statesInetIsAvailabled()) {
          ping_host_data_t* data = (ping_host_data_t*)event_data;
          if (data) {
            tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_INET_UNAVAILABLE,
              CONFIG_TELEGRAM_DEVICE, 
              CONFIG_MESSAGE_TG_HOST_UNAVAILABLE, data->host_name);
          };
        };
      #endif // CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE
      break;

    case RE_PING_MQTT2_UNAVAILABLE:
      statesClear(MQTT_2_ENABLED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_PING_MQTT2_UNAVAILABLE");
      #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE
        if (statesInetIsAvailabled()) {
          ping_host_data_t* data = (ping_host_data_t*)event_data;
          if (data) {
            tgSend(TG_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_INET_UNAVAILABLE, 
              CONFIG_TELEGRAM_DEVICE, 
              CONFIG_MESSAGE_TG_HOST_UNAVAILABLE, data->host_name);
          };
        };
      #endif // CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE
      break;

    default:
      // Ignore...
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "IGNORED");
      break;
  };
}
#endif // CONFIG_PINGER_ENABLE

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

    #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_SENSOR_STATE
      // Sensor status change notification
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