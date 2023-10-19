#include "reStates.h"
#include "time.h"
#include "esp_timer.h"
#include "reWiFi.h"
#include "reMqtt.h"
#if !defined(CONFIG_NO_SENSORS)
  #include "reSensor.h"
#endif // CONFIG_NO_SENSORS

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
// --------------------------------------------------- Watchdog timers ---------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if defined(CONFIG_MQTT_RESTART_DEVICE_MINUTES) && (CONFIG_MQTT_RESTART_DEVICE_MINUTES > 0)

re_restart_timer_t _wdtRestartMqtt;

void wdtRestartMqttInit()
{
  espRestartTimerInit(&_wdtRestartMqtt, RR_MQTT_TIMEOUT, "wdt_mqtt");
}

void wdtRestartMqttFree()
{
  espRestartTimerFree(&_wdtRestartMqtt);
}

void wdtRestartMqttStart()
{
  if (statesMqttIsEnabled()) {
    espRestartTimerStartM(&_wdtRestartMqtt, RR_MQTT_TIMEOUT, CONFIG_MQTT_RESTART_DEVICE_MINUTES, false);
  } else {
    espRestartTimerBreak(&_wdtRestartMqtt);
  };
}

void wdtRestartMqttBreak()
{
  espRestartTimerBreak(&_wdtRestartMqtt);
};

void wdtRestartMqttCheck()
{
  if (statesMqttIsConnected()) {
    wdtRestartMqttBreak();
  } else {
    if (statesMqttIsLocal() || statesInetIsAvailabled()) {
      wdtRestartMqttStart();
    } else {
      wdtRestartMqttBreak();
    };
  };
}

#else
  // Stubs
  #define wdtRestartMqttInit()
  #define wdtRestartMqttFree()
  #define wdtRestartMqttStart()
  #define wdtRestartMqttCheck()
  #define wdtRestartMqttBreak()
#endif 

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------- Verify OTA complete -------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if CONFIG_MQTT_OTA_ENABLE

#if defined(CONFIG_OTA_ROLLBACK_TIMEOUT) && (CONFIG_OTA_ROLLBACK_TIMEOUT > 0)

esp_timer_handle_t _otaVerifyTimer = nullptr;

static void statesFirmwareVerifyTimerEnd(void* arg)
{
  rlog_w(logTAG, "Firmware verify failed: rollback application!");
  espSetResetReason(RR_OTA_FAILED);
  esp_err_t err = esp_ota_mark_app_invalid_rollback_and_reboot();
  if (err != ESP_OK) {
    rlog_e(logTAG, "Failed to esp_ota_mark_app_invalid_rollback_and_reboot(): %d, %s", err, esp_err_to_name(err));
  };
}

static void statesFirmwareVerifyTimerStart()
{
  if (_otaVerifyTimer == nullptr) {
    esp_timer_create_args_t cfgTimer;
    memset(&cfgTimer, 0, sizeof(cfgTimer));
    cfgTimer.callback = statesFirmwareVerifyTimerEnd;
    cfgTimer.name = "app_verify";
    RE_OK_CHECK(esp_timer_create(&cfgTimer, &_otaVerifyTimer), return);
  };
  if ((_otaVerifyTimer != nullptr) && !esp_timer_is_active(_otaVerifyTimer)) {
    RE_OK_CHECK(esp_timer_start_once(_otaVerifyTimer, CONFIG_OTA_ROLLBACK_TIMEOUT * 1000000), return);
  };
  rlog_i(logTAG, "Firmware verify timer started for %d seconds", CONFIG_OTA_ROLLBACK_TIMEOUT);
}

static void statesFirmwareVerifyTimerStop()
{
  if (_otaVerifyTimer != nullptr) {
    if (esp_timer_is_active(_otaVerifyTimer)) {
      esp_timer_stop(_otaVerifyTimer);
    };
    RE_OK_CHECK(esp_timer_delete(_otaVerifyTimer), return);
    _otaVerifyTimer = nullptr;
  };
}

#endif // CONFIG_OTA_ROLLBACK_TIMEOUT

void statesFirmwareVerifyStart()
{
  if (espGetResetReason() == RR_OTA) {
    #if defined(CONFIG_OTA_ROLLBACK_TIMEOUT) && (CONFIG_OTA_ROLLBACK_TIMEOUT > 0)
      statesFirmwareVerifyTimerStart();
    #endif // CONFIG_OTA_ROLLBACK_TIMEOUT
  };
}

void statesFirmwareVerifyCompete()
{
  if (espGetResetReason() == RR_OTA) {
    rlog_i(logTAG, "Firmware verify completed");

    #if defined(CONFIG_OTA_ROLLBACK_TIMEOUT) && (CONFIG_OTA_ROLLBACK_TIMEOUT > 0)
      statesFirmwareVerifyTimerStop();
    #endif // CONFIG_OTA_ROLLBACK_TIMEOUT
  };

  esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
  if (err != ESP_OK) {
    rlog_e(logTAG, "Failed to esp_ota_mark_app_valid_cancel_rollback(): %d, %s", err, esp_err_to_name(err));
  };
}

#endif // CONFIG_MQTT_OTA_ENABLE

// -----------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------- System states ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void heapAllocFailedInit();

void statesInit(bool registerEventHandler)
{
  statesFirmwareVerifyStart();

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

  wdtRestartMqttInit();

  if ((_evgStates) && (_evgErrors)) {
    heapAllocFailedInit();
  };

  if ((_evgStates) && (_evgErrors) && registerEventHandler) {
    statesEventHandlerRegister();
  };
}

void statesFree(bool unregisterEventHandler)
{
  #if defined(CONFIG_OTA_ROLLBACK_TIMEOUT) && (CONFIG_OTA_ROLLBACK_TIMEOUT > 0)
  statesFirmwareVerifyTimerStop();
  #endif // CONFIG_OTA_ROLLBACK_TIMEOUT

  if (_evgStates) {
    if (unregisterEventHandler) {
      statesEventHandlerUnregister();
    };
  };

  if (_evgErrors) {
    vEventGroupDelete(_evgErrors);
    _evgErrors = nullptr;
  };

  if (_evgStates) {
    vEventGroupDelete(_evgStates);
    _evgStates = nullptr;
  };

  wdtRestartMqttFree();
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

bool statesInetIsGood(bool checkRssi)
{
  return statesInetIsAvailabled() 
      && !statesCheck(INET_SLOWDOWN, false) 
      && (!checkRssi || wifiRSSIIsOk());
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

bool statesMqttIsEnabled()
{
  if (statesMqttIsLocal()) {
    return statesWiFiIsConnected();
  } else {
    return statesWiFiIsConnected() && statesInetIsAvailabled();
  };
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
  return malloc_stringf("{\"general\":%d,\"heap\":%d,\"mqtt\":%d,\"telegram\":%d,\"smtp\":%d,\"site\":%d,\"thingspeak\":%d,\"openmon\":%d,\"narodmon\":%d,\"sensor0\":%d,\"sensor1\":%d,\"sensor2\":%d,\"sensor3\":%d,\"sensor4\":%d,\"sensor5\":%d,\"sensor6\":%d,\"sensor7\":%d}",
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
  #if CONFIG_HEAP_ABORT_WHEN_ALLOCATION_FAILS
    espSetResetReason(RR_HEAP_ALLOCATION_FAILED);
  #else
    heapFailsCount++;
    #if CONFIG_HEAP_ALLOC_FAILED_RESTART
      espRestart(RR_HEAP_ALLOCATION_FAILED); 
    #endif // CONFIG_HEAP_ALLOC_FAILED_RESTART
  #endif // CONFIG_HEAP_ABORT_WHEN_ALLOCATION_FAILS
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

#if CONFIG_HEAP_TRACING_STANDALONE

#define CONFIG_HEAP_TRACING_NUM_RECORDS 256
#define CONFIG_HEAP_LEAKS_NUM_RECORDS 256
#define CONFIG_HEAP_LEAKS_MIN_SIZE 1
#define CONFIG_HEAP_LEAKS_MIN_REPEATS 3

typedef struct {
  uint32_t ccount;
  void *address;
  size_t size;
  void *alloced_by[CONFIG_HEAP_TRACING_STACK_DEPTH];
  uint8_t confirm;
  uint32_t repeats;
  time_t timestamp;
} heap_leak_record_t;

static uint8_t leak_count = 0;
static heap_leak_record_t leaks_buffer[CONFIG_HEAP_LEAKS_NUM_RECORDS];
static heap_trace_record_t trace_buffer[CONFIG_HEAP_TRACING_NUM_RECORDS];

void heapLeaksStart()
{
  memset(&leaks_buffer, 0, sizeof(leaks_buffer));
  heap_trace_init_standalone(trace_buffer, CONFIG_HEAP_TRACING_NUM_RECORDS);
  heap_trace_start(HEAP_TRACE_LEAKS);
}

void heapLeaksStop()
{
  heap_trace_stop();
}

void heapLeaksScan()
{
  // Mark all current entries as lost
  for (uint16_t i = 0; i < CONFIG_HEAP_LEAKS_NUM_RECORDS; i++) {
    leaks_buffer[i].confirm = 0;
  };

  // Search for new leaks and comparison with current data
  heap_trace_record_t rec;
  for (uint16_t j = 0; j < CONFIG_HEAP_TRACING_NUM_RECORDS; j++) {
    if ((heap_trace_get(j, &rec) == ESP_OK) && (rec.address != NULL) && (rec.freed_by[0] == NULL) && (rec.size >= CONFIG_HEAP_LEAKS_MIN_SIZE) && ((rec.ccount & 1) > 0)) {
      // We are looking for this entry in the main list
      int16_t found = -1;
      for (uint16_t i = 0; i < CONFIG_HEAP_LEAKS_NUM_RECORDS; i++) {
        if ((leaks_buffer[i].address == rec.address) && (leaks_buffer[i].size == rec.size) && (leaks_buffer[i].ccount == rec.ccount)) {
          found = i;
          leaks_buffer[i].confirm = 1;
          leaks_buffer[i].repeats++;
          break;
        };
      };
      // Entry not found, fill first free entry in buffer
      if (found == -1) {
        for (uint16_t i = 0; i < CONFIG_HEAP_LEAKS_NUM_RECORDS; i++) {
          if ((leaks_buffer[i].address == NULL) || (leaks_buffer[i].size == 0)) {
            leaks_buffer[i].ccount = rec.ccount;
            leaks_buffer[i].address = rec.address;
            leaks_buffer[i].size = rec.size;
            #if CONFIG_HEAP_TRACING_STACK_DEPTH > 0
              memcpy(&leaks_buffer[i].alloced_by, &rec.alloced_by, sizeof(void*)*CONFIG_HEAP_TRACING_STACK_DEPTH);
            #endif // CONFIG_HEAP_TRACING_STACK_DEPTH
            leaks_buffer[i].confirm = 1;
            leaks_buffer[i].repeats = 1;
            leaks_buffer[i].timestamp = time(nullptr);
            break;
          };
        };
      };
    };
  };

  // Mark as free all records that have not been committed in this session
  leak_count = 0;
  for (uint16_t i = 0; i < CONFIG_HEAP_LEAKS_NUM_RECORDS; i++) {
    if ((leaks_buffer[i].confirm == 0) && (leaks_buffer[i].address != NULL)) {
      memset(&leaks_buffer[i], 0, sizeof(heap_leak_record_t));
    } else {
      leak_count++;
    };
  };
}

char* heapLeaksJson()
{
  uint16_t count = 0;
  char* json = nullptr;

  if (leak_count > 0) {
    char* item = nullptr;
    char* temp = nullptr;

    heap_leak_record_t rec;
    for (uint16_t i = 0; i < CONFIG_HEAP_LEAKS_NUM_RECORDS; i++) {
      if ((leaks_buffer[i].address != NULL) && (leaks_buffer[i].size > 0) && (leaks_buffer[i].confirm > 0) && (leaks_buffer[i].repeats > CONFIG_HEAP_LEAKS_MIN_REPEATS)) {
        count++;
        memcpy(&rec, &leaks_buffer[i], sizeof(heap_leak_record_t));

        // Create item JSON
        char* stack = nullptr;
        #if CONFIG_HEAP_TRACING_STACK_DEPTH == 0
          stack = malloc_string("");
        #elif CONFIG_HEAP_TRACING_STACK_DEPTH == 1
          stack = malloc_stringf("%p", rec.alloced_by[0]);
        #elif CONFIG_HEAP_TRACING_STACK_DEPTH == 2
          stack = malloc_stringf("%p %p", rec.alloced_by[0], rec.alloced_by[1]);
        #elif CONFIG_HEAP_TRACING_STACK_DEPTH == 3
          stack = malloc_stringf("%p %p %p", rec.alloced_by[0], rec.alloced_by[1], rec.alloced_by[2]);
        #elif CONFIG_HEAP_TRACING_STACK_DEPTH == 4
          stack = malloc_stringf("%p %p %p %p", rec.alloced_by[0], rec.alloced_by[1], rec.alloced_by[2], rec.alloced_by[3]);
        #elif CONFIG_HEAP_TRACING_STACK_DEPTH == 5
          stack = malloc_stringf("%p %p %p %p %p", rec.alloced_by[0], rec.alloced_by[1], rec.alloced_by[2], rec.alloced_by[3], rec.alloced_by[4]);
        #elif CONFIG_HEAP_TRACING_STACK_DEPTH == 6
          stack = malloc_stringf("%p %p %p %p %p %p", rec.alloced_by[0], rec.alloced_by[1], rec.alloced_by[2], rec.alloced_by[3], rec.alloced_by[4], rec.alloced_by[5]);
        #elif CONFIG_HEAP_TRACING_STACK_DEPTH == 7
          stack = malloc_stringf("%p %p %p %p %p %p %p", rec.alloced_by[0], rec.alloced_by[1], rec.alloced_by[2], rec.alloced_by[3], rec.alloced_by[4], rec.alloced_by[5], rec.alloced_by[6]);
        #elif CONFIG_HEAP_TRACING_STACK_DEPTH == 8
          stack = malloc_stringf("%p %p %p %p %p %p %p %p", rec.alloced_by[0], rec.alloced_by[1], rec.alloced_by[2], rec.alloced_by[3], rec.alloced_by[4], rec.alloced_by[5], rec.alloced_by[6], rec.alloced_by[7]);
        #elif CONFIG_HEAP_TRACING_STACK_DEPTH == 9
          stack = malloc_stringf("%p %p %p %p %p %p %p %p %p", rec.alloced_by[0], rec.alloced_by[1], rec.alloced_by[2], rec.alloced_by[3], rec.alloced_by[4], rec.alloced_by[5], rec.alloced_by[6], rec.alloced_by[7], rec.alloced_by[8]);
        #else 
          stack = malloc_stringf("%p %p %p %p %p %p %p %p %p %p", rec.alloced_by[0], rec.alloced_by[1], rec.alloced_by[2], rec.alloced_by[3], rec.alloced_by[4], rec.alloced_by[5], rec.alloced_by[6], rec.alloced_by[7], rec.alloced_by[8], rec.alloced_by[9]);
        #endif // #elif CONFIG_HEAP_TRACING_STACK_DEPTH

        // 2023-02-15: fixed possible sharing error from multiple tasks
        char ts_buffer[CONFIG_FORMAT_STRFTIME_BUFFER_SIZE];
        time2str_empty(CONFIG_FORMAT_DTS, &(rec.timestamp), &ts_buffer[0], sizeof(ts_buffer));

        if (stack) {
          item = malloc_stringf("{\"timestamp\":\"%s\",\"repeats\":%d,\"address\":\"%p\",\"size\":%d,\"cpu\":%d,\"ccount\":\"0x%08x\",\"stack\":\"%s\"}", 
            ts_buffer, rec.repeats, rec.address, rec.size, rec.ccount & 1, rec.ccount & ~3, stack);
          free(stack);
        };

        // Add item to JSON array
        if (item) {
          if (json) {
            temp = json;
            json = malloc_stringf("%s,%s", temp, item);
            free(temp);
            free(item);
            item = nullptr;
          } else {
            json = item;
            item = nullptr;
          };
        };
      };
    };
  };

  // Add brackets
  if (json) {
    char* buf = json;
    json = malloc_stringf("{\"total\":%d,\"details\":[%s]}", count, buf);
    free(buf);
  };

  return json;
}

void heapLeaksUpdate()
{
  heapLeaksScan();
  if (statesMqttIsEnabled()) {
    char* json = heapLeaksJson();
    mqttPublish(
      mqttGetTopicDevice1(statesMqttIsPrimary(), CONFIG_MQTT_HEAP_LEAKS_LOCAL, CONFIG_MQTT_HEAP_LEAKS_TOPIC), 
      json, CONFIG_MQTT_HEAP_LEAKS_QOS, CONFIG_MQTT_HEAP_LEAKS_RETAINED, true, true);
  };
};

#endif // CONFIG_HEAP_TRACING_STANDALONE

void heapCapsDebug(const char *function_name)
{
  size_t free = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
  size_t heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
  rlog_w("HEAP", "Heap free %.3f% %, free size: %d, total size: %d [ %s ]", 
    100.0 * (double)free / (double)heap, free, heap, function_name);
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ System LED -----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static ledQueue_t _ledSysQueue = NULL;

void ledSysInit(int8_t ledGPIO, bool ledHigh, uint32_t taskStackSize, ledCustomControl_t customControl)
{
  if (_ledSysQueue == NULL) {
    _ledSysQueue = ledTaskCreate(ledGPIO, ledHigh, true, "led_system", taskStackSize, customControl);
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
  #if !defined(CONFIG_OFFLINE_MODE) || (CONFIG_OFFLINE_MODE == 0)
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
  #else
    else if (!(states & TIME_IS_OK)) {
      ledSysBlinkOn(CONFIG_LEDSYS_TIME_ERROR_QUANTITY, CONFIG_LEDSYS_TIME_ERROR_DURATION, CONFIG_LEDSYS_TIME_ERROR_INTERVAL);
    }
  #endif // CONFIG_OFFLINE_MODE
  else {
    ledSysBlinkOn(CONFIG_LEDSYS_NORMAL_QUANTITY, CONFIG_LEDSYS_NORMAL_DURATION, CONFIG_LEDSYS_NORMAL_INTERVAL);
  };
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------- Event notifications -------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if CONFIG_ENABLE_STATES_NOTIFICATIONS

#if CONFIG_NOTIFY_TELEGRAM_WIFI_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
  #define ENABLE_NOTIFY_WIFI_STATUS 1
#else
  #define ENABLE_NOTIFY_WIFI_STATUS 0
#endif // CONFIG_NOTIFY_TELEGRAM_WIFI_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE

#if CONFIG_PINGER_ENABLE && (CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
  #define ENABLE_NOTIFY_INET_STATUS 1
#else
  #define ENABLE_NOTIFY_INET_STATUS 0
#endif // CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE

#if CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
  #define ENABLE_NOTIFY_MQTT_STATUS 1
#else
  #define ENABLE_NOTIFY_MQTT_STATUS 0
#endif // CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS 

#if CONFIG_ENABLE_STATES_NOTIFICATIONS && (CONFIG_NOTIFY_TELEGRAM_MQTT_ERRORS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
  #define ENABLE_NOTIFY_MQTT_ERRORS 1
#else
  #define ENABLE_NOTIFY_MQTT_ERRORS 0
#endif // CONFIG_NOTIFY_TELEGRAM_MQTT_ERRORS

#if defined(CONFIG_MQTT1_TYPE) && CONFIG_MQTT1_PING_CHECK && (CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
  #define ENABLE_NOTIFY_MQTT1_PING 1
#else
  #define ENABLE_NOTIFY_MQTT1_PING 0
#endif // CONFIG_MQTT1_PING_CHECK

#if defined(CONFIG_MQTT2_TYPE) && CONFIG_MQTT2_PING_CHECK && (CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
  #define ENABLE_NOTIFY_MQTT2_PING 1
#else
  #define ENABLE_NOTIFY_MQTT2_PING 0
#endif // CONFIG_MQTT1_PING_CHECK

#if CONFIG_OPENMON_ENABLE && (CONFIG_NOTIFY_TELEGRAM_OPENMON_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
  #define ENABLE_NOTIFY_OPENMON_STATUS 1
#else
  #define ENABLE_NOTIFY_OPENMON_STATUS 0
#endif // CONFIG_NOTIFY_TELEGRAM_OPENMON_STATUS

#if CONFIG_NARODMON_ENABLE && (CONFIG_NOTIFY_TELEGRAM_NARODMON_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
  #define ENABLE_NOTIFY_NARODMON_STATUS 1
#else
  #define ENABLE_NOTIFY_NARODMON_STATUS 0
#endif // CONFIG_NOTIFY_TELEGRAM_NARODMON_STATUS

#if CONFIG_THINGSPEAK_ENABLE && (CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_STATUS || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
  #define ENABLE_NOTIFY_THINGSPEAK_STATUS 1
#else
  #define ENABLE_NOTIFY_THINGSPEAK_STATUS 0
#endif // CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_STATUS

#if !defined(CONFIG_NO_SENSORS) && (CONFIG_NOTIFY_TELEGRAM_SENSOR_STATE || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
  #define ENABLE_NOTIFY_SENSOR_STATE 1
#else
  #define ENABLE_NOTIFY_SENSOR_STATE 0
#endif // CONFIG_NOTIFY_TELEGRAM_SENSOR_STATE

#if CONFIG_SILENT_MODE_ENABLE && (CONFIG_NOTIFY_TELEGRAM_SILENT_MODE || CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE)
  #define ENABLE_NOTIFY_SILENT_MODE 1
#else
  #define ENABLE_NOTIFY_SILENT_MODE 0
#endif // CONFIG_NOTIFY_TELEGRAM_SILENT_MODE

static bool healthMonitorNotify(hm_notify_data_t *notify_data)
{
  if (notify_data != nullptr) {
    // Format failure start time
    struct tm tm_failure;
    char str_failure[CONFIG_FORMAT_STRFTIME_BUFFER_SIZE];
    memset(&str_failure, 0, sizeof(str_failure));
    localtime_r(&notify_data->time_failure, &tm_failure);
    strftime(str_failure, sizeof(str_failure), CONFIG_FORMAT_DTS, &tm_failure);
    
    if (notify_data->state == ESP_OK) {
      // Format failure end time
      struct tm tm_recovery;
      char str_recovery[CONFIG_FORMAT_STRFTIME_BUFFER_SIZE];
      memset(&str_recovery, 0, sizeof(str_recovery));
      localtime_r(&notify_data->time_state, &tm_recovery);
      strftime(str_recovery, sizeof(str_recovery), CONFIG_FORMAT_DTS, &tm_recovery);

      // Format failure duration time
      time_t failure_duration = notify_data->time_state - notify_data->time_failure;
      uint16_t duration_h = failure_duration / 3600;
      uint16_t duration_m = failure_duration % 3600 / 60;
      uint16_t duration_s = failure_duration % 3600 % 60;

      // Send notify
      if (notify_data->msg_template) {
        if (notify_data->object == nullptr) {
          return tgSendMsg(notify_data->msg_options, CONFIG_TELEGRAM_DEVICE, notify_data->msg_template, 
            str_failure, str_recovery, duration_h, duration_m, duration_s);
        } else {
          return tgSendMsg(notify_data->msg_options, CONFIG_TELEGRAM_DEVICE, notify_data->msg_template, notify_data->object, 
            str_failure, str_recovery, duration_h, duration_m, duration_s);
        };
      };
    } else {
      // Send notify
      if (notify_data->msg_template) {
        uint32_t err_code = notify_data->state;
        const char* err_text = esp_err_to_name(notify_data->state);
        if ((notify_data->state > 0x7064) && (notify_data->state < 0x8000)) {
          // HTTP errors
          err_code = notify_data->state - 0x7000;
          if (err_code == 300) err_text = "Multiple Choices";
          else if (err_code == 301) err_text = "Moved Permanently";
          else if (err_code == 302) err_text = "Moved Temporarily";
          else if (err_code == 307) err_text = "Temporary Redirect";
          else if (err_code == 308) err_text = "Permanent Redirect";
          else if (err_code == 400) err_text = "Bad Request";
          else if (err_code == 401) err_text = "Unauthorized";
          else if (err_code == 403) err_text = "Forbidden";
          else if (err_code == 404) err_text = "Not Found";
          else if (err_code == 429) err_text = "Too Many Requests";
          else if (err_code == 431) err_text = "Request Header Fields Too Large";
          else if (err_code == 500) err_text = "Internal Server Error";
          else if (err_code == 501) err_text = "Not Implemented";
          else if (err_code == 502) err_text = "Bad Gateway";
          else if (err_code == 503) err_text = "Service Unavailable";
          else if (err_code == 504) err_text = "Gateway Timeout";
        };
        if (notify_data->object == nullptr) {
          return tgSendMsg(notify_data->msg_options, CONFIG_TELEGRAM_DEVICE, notify_data->msg_template, 
            notify_data->state, err_code, err_text, str_failure);
        } else {
          return tgSendMsg(notify_data->msg_options, CONFIG_TELEGRAM_DEVICE, notify_data->msg_template, notify_data->object, 
            notify_data->state, err_code, err_text, str_failure);
        };
      };
    };
  };
  return false;
}

// --- WiFi --------------------------------------------------------------------------------------------------------------
#if ENABLE_NOTIFY_WIFI_STATUS
  
  reHealthMonitor hmWifi(nullptr, HM_RECOVERY, 
    encMsgOptions(MK_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_WIFI_STATUS, CONFIG_NOTIFY_TELEGRAM_WIFI_PRIORITY), 
    CONFIG_MESSAGE_TG_WIFI_AVAILABLE, nullptr, CONFIG_NOTIFY_TELEGRAM_WIFI_THRESOLD, healthMonitorNotify);

#endif // ENABLE_NOTIFY_WIFI_STATUS

// --- Ping --------------------------------------------------------------------------------------------------------------
#if ENABLE_NOTIFY_INET_STATUS

  reHealthMonitor hmInet(nullptr, HM_AUTO, 
    encMsgOptions(MK_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_INET_UNAVAILABLE, CONFIG_NOTIFY_TELEGRAM_INET_PRIORITY), 
    CONFIG_MESSAGE_TG_INET_AVAILABLE, CONFIG_MESSAGE_TG_INET_UNAVAILABLE, CONFIG_NOTIFY_TELEGRAM_INET_THRESOLD, healthMonitorNotify);

#endif // ENABLE_NOTIFY_INET_STATUS

// --  MQTT --------------------------------------------------------------------------------------------------------------
#if ENABLE_NOTIFY_MQTT_STATUS

  reHealthMonitor hmMqtt(nullptr, HM_AUTO, 
    encMsgOptions(MK_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_MQTT_STATUS, CONFIG_NOTIFY_TELEGRAM_MQTT_PRIORITY),
    CONFIG_MESSAGE_TG_MQTT_CONN_OK, CONFIG_MESSAGE_TG_MQTT_CONN_LOST, CONFIG_NOTIFY_TELEGRAM_MQTT_THRESOLD, healthMonitorNotify);

  #if ENABLE_NOTIFY_MQTT1_PING
    reHealthMonitor hmMqttPing1(CONFIG_MQTT1_HOST, HM_AUTO, 
      encMsgOptions(MK_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_MQTT_STATUS, CONFIG_NOTIFY_TELEGRAM_MQTT_PRIORITY),
      CONFIG_MESSAGE_TG_HOST_AVAILABLE, CONFIG_MESSAGE_TG_HOST_UNAVAILABLE, CONFIG_NOTIFY_TELEGRAM_MQTT_PING_THRESOLD, healthMonitorNotify);
  #endif // ENABLE_NOTIFY_MQTT1_PING

  #if ENABLE_NOTIFY_MQTT2_PING
    reHealthMonitor hmMqttPing2(CONFIG_MQTT2_HOST, HM_AUTO, 
      encMsgOptions(MK_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_MQTT_STATUS, CONFIG_NOTIFY_TELEGRAM_MQTT_PRIORITY),
      CONFIG_MESSAGE_TG_HOST_AVAILABLE, CONFIG_MESSAGE_TG_HOST_UNAVAILABLE, CONFIG_NOTIFY_TELEGRAM_MQTT_PING_THRESOLD, healthMonitorNotify);
  #endif // ENABLE_NOTIFY_MQTT2_PING
#endif // ENABLE_NOTIFY_MQTT_STATUS

// -- OpenMon ------------------------------------------------------------------------------------------------------------
#if ENABLE_NOTIFY_OPENMON_STATUS

  reHealthMonitor hmOpenMon("open-monitoring.online", HM_AUTO,
    encMsgOptions(MK_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_OPENMON_STATUS, CONFIG_NOTIFY_TELEGRAM_OPENMON_PRIORITY),
    CONFIG_MESSAGE_TG_HOST_AVAILABLE, CONFIG_MESSAGE_TG_HOST_UNAVAILABLE, CONFIG_NOTIFY_TELEGRAM_OPENMON_THRESOLD, healthMonitorNotify);

#endif // ENABLE_NOTIFY_OPENMON_STATUS

// -- NarodMon -----------------------------------------------------------------------------------------------------------
#if ENABLE_NOTIFY_NARODMON_STATUS

  reHealthMonitor hmNarodMon("narodmon.ru", HM_AUTO, 
    encMsgOptions(MK_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_NARODMON_STATUS, CONFIG_NOTIFY_TELEGRAM_NARODMON_PRIORITY),
    CONFIG_MESSAGE_TG_HOST_AVAILABLE, CONFIG_MESSAGE_TG_HOST_UNAVAILABLE, CONFIG_NOTIFY_TELEGRAM_NARODMON_THRESOLD, healthMonitorNotify);

#endif // ENABLE_NOTIFY_NARODMON_STATUS

// --  ThingSpeak --------------------------------------------------------------------------------------------------------
#if ENABLE_NOTIFY_THINGSPEAK_STATUS

  reHealthMonitor hmThingSpeak("thingspeak.com", HM_AUTO,
    encMsgOptions(MK_SERVICE, CONFIG_NOTIFY_TELEGRAM_ALERT_THINGSPEAK_STATUS, CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_PRIORITY),
    CONFIG_MESSAGE_TG_HOST_AVAILABLE, CONFIG_MESSAGE_TG_HOST_UNAVAILABLE, CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_THRESOLD, healthMonitorNotify);

#endif // ENABLE_NOTIFY_THINGSPEAK_STATUS

// -- Locks --------------------------------------------------------------------------------------------------------------
static void healthMonitorsInetAvailable(bool setInetState)
{
  rlog_d(logTAG, "Sending notifications about the resumption of Internet access");

  #if ENABLE_NOTIFY_INET_STATUS
    if (setInetState) hmInet.setState(ESP_OK, time(nullptr));
  #endif // ENABLE_NOTIFY_INET_STATUS

  #if ENABLE_NOTIFY_MQTT_STATUS
    hmMqtt.unlock();
  #endif // ENABLE_NOTIFY_MQTT_STATUS

  #if ENABLE_NOTIFY_MQTT1_PING
    hmMqttPing1.unlock();
  #endif // ENABLE_NOTIFY_MQTT1_PING

  #if ENABLE_NOTIFY_MQTT2_PING
    hmMqttPing2.unlock();
  #endif // ENABLE_NOTIFY_MQTT2_PING

  #if ENABLE_NOTIFY_OPENMON_STATUS
    hmOpenMon.unlock();
  #endif // ENABLE_NOTIFY_OPENMON_STATUS

  #if ENABLE_NOTIFY_NARODMON_STATUS
    hmNarodMon.unlock();
  #endif // ENABLE_NOTIFY_NARODMON_STATUS

  #if ENABLE_NOTIFY_THINGSPEAK_STATUS
    hmThingSpeak.unlock();
  #endif // ENABLE_NOTIFY_THINGSPEAK_STATUS
}

static void healthMonitorsInetUnavailable(esp_err_t inetState, time_t timeState)
{
  rlog_d(logTAG, "Sending notifications about the unavailability of the Internet");

  #if ENABLE_NOTIFY_INET_STATUS
    if (inetState != ESP_OK) hmInet.setState(inetState, timeState);
  #endif // ENABLE_NOTIFY_INET_STATUS

  #if ENABLE_NOTIFY_MQTT1_PING
    hmMqttPing1.lock();
  #endif // ENABLE_NOTIFY_MQTT1_PING

  #if ENABLE_NOTIFY_MQTT2_PING
    hmMqttPing2.lock();
  #endif // ENABLE_NOTIFY_MQTT2_PING

  #if ENABLE_NOTIFY_MQTT_STATUS
    hmMqtt.lock();
  #endif // ENABLE_NOTIFY_MQTT_STATUS

  #if ENABLE_NOTIFY_OPENMON_STATUS
    hmOpenMon.lock();
  #endif // ENABLE_NOTIFY_OPENMON_STATUS

  #if ENABLE_NOTIFY_NARODMON_STATUS
    hmNarodMon.lock();
  #endif // ENABLE_NOTIFY_NARODMON_STATUS

  #if ENABLE_NOTIFY_THINGSPEAK_STATUS
    hmThingSpeak.lock();
  #endif // ENABLE_NOTIFY_THINGSPEAK_STATUS
}

static void healthMonitorsWiFiAvailable(bool setWifiState)
{
  rlog_d(logTAG, "Sending wifi connect notifications");

  #if ENABLE_NOTIFY_WIFI_STATUS
    if (setWifiState) {
      hmWifi.setStateCustom(ESP_OK, time(nullptr), true, malloc_string(wifiGetSSID()));
    };
  #endif // ENABLE_NOTIFY_WIFI_STATUS

  #if ENABLE_NOTIFY_INET_STATUS
    hmInet.unlock();
  #endif // ENABLE_NOTIFY_INET_STATUS

  healthMonitorsInetAvailable(false);
}

static void healthMonitorsWiFiUnavailable(esp_err_t wifiState)
{
  rlog_d(logTAG, "Sending wifi disconnect notifications");

  #if ENABLE_NOTIFY_WIFI_STATUS
    if (wifiState != ESP_OK) hmWifi.setState(wifiState, time(nullptr));
  #endif // ENABLE_NOTIFY_WIFI_STATUS

  #if ENABLE_NOTIFY_INET_STATUS
    hmInet.lock();
  #endif // ENABLE_NOTIFY_INET_STATUS

  healthMonitorsInetUnavailable(ESP_ERR_INVALID_STATE, 0);
}

// -- Parameters ---------------------------------------------------------------------------------------------------------
#if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
  static uint32_t  _hmNotifyDelayFailure = CONFIG_NOTIFY_TELEGRAM_MINIMUM_FAILURE_TIME;
  static uint8_t   _hmNotifyWifi = CONFIG_NOTIFY_TELEGRAM_WIFI_STATUS;
  static uint8_t   _hmNotifyMqtt = CONFIG_NOTIFY_TELEGRAM_MQTT_STATUS;
  static uint8_t   _hmNotifyMqttErrors = CONFIG_NOTIFY_TELEGRAM_MQTT_ERRORS;
  #if CONFIG_PINGER_ENABLE
    static uint8_t  _hmNotifyInet = CONFIG_NOTIFY_TELEGRAM_INET_UNAVAILABLE;
  #endif // CONFIG_PINGER_ENABLE
  #if CONFIG_OPENMON_ENABLE
    static uint8_t  _hmNotifyOpenMon = CONFIG_NOTIFY_TELEGRAM_OPENMON_STATUS;
  #endif // CONFIG_OPENMON_ENABLE
  #if CONFIG_NARODMON_ENABLE
    static uint8_t  _hmNotifyNarodMon = CONFIG_NOTIFY_TELEGRAM_NARODMON_STATUS;
  #endif // CONFIG_NARODMON_ENABLE
  #if CONFIG_THINGSPEAK_ENABLE
    static uint8_t  _hmNotifyThingSpeak = CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_STATUS;
  #endif // CONFIG_THINGSPEAK_ENABLE
  static uint8_t   _hmNotifySensors = CONFIG_NOTIFY_TELEGRAM_SENSOR_STATE;
  #if CONFIG_SILENT_MODE_ENABLE
    static uint8_t _hmNotifySilentMode = CONFIG_NOTIFY_TELEGRAM_SILENT_MODE;
  #endif // CONFIG_SILENT_MODE_ENABLE

  static void healthMonitorsRegisterParameters()
  {
    paramsGroupHandle_t pgNotify = paramsRegisterGroup(nullptr, 
      CONFIG_STATES_NOTIFY_PGROUP_ROOT_KEY, CONFIG_STATES_NOTIFY_PGROUP_ROOT_TOPIC, CONFIG_STATES_NOTIFY_PGROUP_ROOT_FRIENDLY);
    paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U32, nullptr, pgNotify,
      CONFIG_NOTIFY_TELEGRAM_MINIMUM_FAILURE_TIME_KEY, CONFIG_NOTIFY_TELEGRAM_MINIMUM_FAILURE_TIME_FRIENDLY,
      CONFIG_MQTT_PARAMS_QOS, (void*)&_hmNotifyDelayFailure);

    // -- WiFi --------------------------------------------------------------------
    hmWifi.assignParams(nullptr, &_hmNotifyWifi);
    paramsSetLimitsU8(
      paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgNotify,
        CONFIG_NOTIFY_TELEGRAM_WIFI_KEY, CONFIG_NOTIFY_TELEGRAM_WIFI_FRIENDLY,
        CONFIG_MQTT_PARAMS_QOS, (void*)&_hmNotifyWifi),
      0, 1);
    
    // -- Ping --------------------------------------------------------------------
    #if CONFIG_PINGER_ENABLE
      hmInet.assignParams(&_hmNotifyDelayFailure, &_hmNotifyInet);
      paramsSetLimitsU8(
        paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgNotify,
          CONFIG_NOTIFY_TELEGRAM_INET_KEY, CONFIG_NOTIFY_TELEGRAM_INET_FRIENDLY,
          CONFIG_MQTT_PARAMS_QOS, (void*)&_hmNotifyInet),
        0, 1);
    #endif // CONFIG_PINGER_ENABLE

    // -- MQTT --------------------------------------------------------------------
    hmMqtt.assignParams(&_hmNotifyDelayFailure, &_hmNotifyMqtt);
    #if CONFIG_PINGER_ENABLE && defined(CONFIG_MQTT1_TYPE) && CONFIG_MQTT1_PING_CHECK
      hmMqttPing1.assignParams(&_hmNotifyDelayFailure, &_hmNotifyMqtt);
    #endif // CONFIG_MQTT1_PING_CHECK
    #if CONFIG_PINGER_ENABLE && defined(CONFIG_MQTT2_TYPE) && CONFIG_MQTT2_PING_CHECK
      hmMqttPing2.assignParams(&_hmNotifyDelayFailure, &_hmNotifyMqtt);
    #endif // CONFIG_MQTT2_PING_CHECK
    paramsSetLimitsU8(
      paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgNotify,
        CONFIG_NOTIFY_TELEGRAM_MQTT_KEY, CONFIG_NOTIFY_TELEGRAM_MQTT_FRIENDLY,
        CONFIG_MQTT_PARAMS_QOS, (void*)&_hmNotifyMqtt),
      0, 1);
    paramsSetLimitsU8(
      paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgNotify,
        CONFIG_NOTIFY_TELEGRAM_MQTT_ERRORS_KEY, CONFIG_NOTIFY_TELEGRAM_MQTT_ERRORS_FRIENDLY,
        CONFIG_MQTT_PARAMS_QOS, (void*)&_hmNotifyMqttErrors),
      0, 1);

    // -- OpenMon -----------------------------------------------------------------
    #if CONFIG_OPENMON_ENABLE
      hmOpenMon.assignParams(&_hmNotifyDelayFailure, &_hmNotifyOpenMon);
      paramsSetLimitsU8(
        paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgNotify,
          CONFIG_NOTIFY_TELEGRAM_OPENMON_KEY, CONFIG_NOTIFY_TELEGRAM_OPENMON_FRIENDLY,
          CONFIG_MQTT_PARAMS_QOS, (void*)&_hmNotifyOpenMon),
        0, 1);
    #endif // CONFIG_OPENMON_ENABLE

    // -- NarodMon ----------------------------------------------------------------
    #if CONFIG_NARODMON_ENABLE
      hmNarodMon.assignParams(&_hmNotifyDelayFailure, &_hmNotifyNarodMon);
      paramsSetLimitsU8(
        paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgNotify,
          CONFIG_NOTIFY_TELEGRAM_NARODMON_KEY, CONFIG_NOTIFY_TELEGRAM_NARODMON_FRIENDLY,
          CONFIG_MQTT_PARAMS_QOS, (void*)&_hmNotifyNarodMon),
        0, 1);
    #endif // CONFIG_NARODMON_ENABLE

    // -- ThingSpeak --------------------------------------------------------------
    #if CONFIG_THINGSPEAK_ENABLE
      hmThingSpeak.assignParams(&_hmNotifyDelayFailure, &_hmNotifyThingSpeak);
      paramsSetLimitsU8(
        paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgNotify,
          CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_KEY, CONFIG_NOTIFY_TELEGRAM_THINGSPEAK_FRIENDLY,
          CONFIG_MQTT_PARAMS_QOS, (void*)&_hmNotifyThingSpeak),
        0, 1);
    #endif // CONFIG_THINGSPEAK_ENABLE

    // -- Sensors -----------------------------------------------------------------
    paramsSetLimitsU8(
      paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgNotify,
        CONFIG_NOTIFY_TELEGRAM_SENSOR_KEY, CONFIG_NOTIFY_TELEGRAM_SENSOR_FRIENDLY,
        CONFIG_MQTT_PARAMS_QOS, (void*)&_hmNotifySensors),

      0, 1);

    // -- Silent mode -------------------------------------------------------------
    #if CONFIG_SILENT_MODE_ENABLE
    paramsSetLimitsU8(
      paramsRegisterValue(OPT_KIND_PARAMETER, OPT_TYPE_U8, nullptr, pgNotify,
        CONFIG_NOTIFY_TELEGRAM_SILENT_MODE_KEY, CONFIG_NOTIFY_TELEGRAM_SILENT_MODE_FRIENDLY,
        CONFIG_MQTT_PARAMS_QOS, (void*)&_hmNotifySilentMode),
      0, 1);
    #endif // CONFIG_SILENT_MODE_ENABLE
  }
#endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE

#else

  #define ENABLE_NOTIFY_WIFI_STATUS 0
  #define ENABLE_NOTIFY_INET_STATUS 0
  #define ENABLE_NOTIFY_MQTT_STATUS 0
  #define ENABLE_NOTIFY_MQTT_ERRORS 0
  #define ENABLE_NOTIFY_MQTT1_PING 0
  #define ENABLE_NOTIFY_MQTT2_PING 0
  #define ENABLE_NOTIFY_OPENMON_STATUS 0
  #define ENABLE_NOTIFY_NARODMON_STATUS 0
  #define ENABLE_NOTIFY_THINGSPEAK_STATUS 0
  #define ENABLE_NOTIFY_SENSOR_STATE 0
  #define ENABLE_NOTIFY_SILENT_MODE 0

#endif // CONFIG_ENABLE_STATES_NOTIFICATIONS 

// -----------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------- Debug info -------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if CONFIG_RESTART_DEBUG_INFO

static char* statesGetDebugHeap(re_restart_debug_t *debug)
{
  if ((debug->heap_total > 0) && (debug->heap_total > debug->heap_free)) {
    struct tm timeinfo;
    localtime_r(&debug->heap_min_time, &timeinfo);
    char time_buffer[CONFIG_FORMAT_STRFTIME_DTS_BUFFER_SIZE];
    memset(&time_buffer, 0, CONFIG_FORMAT_STRFTIME_DTS_BUFFER_SIZE);
    strftime(time_buffer, CONFIG_FORMAT_STRFTIME_DTS_BUFFER_SIZE, CONFIG_FORMAT_DTS, &timeinfo);

    double heapTotal = (double)debug->heap_total / 1024;
    double heapFree = (double)debug->heap_free / 1024;
    double heapFreeMin = (double)debug->heap_free_min / 1024;

    return malloc_stringf("%.1fkB : %.1fkB (%.1f%%) : %.1fkB (%.1f%%) %s", 
      heapTotal,
      heapFree, 100.0 * (heapFree / heapTotal),
      heapFreeMin, 100.0 * (heapFreeMin / heapTotal), time_buffer);
  };
  return nullptr;
}

#if CONFIG_RESTART_DEBUG_STACK_DEPTH > 0

static char* statesGetDebugTrace(re_restart_debug_t *debug)
{
  char* backtrace = nullptr;
  char* item = nullptr;
  char* temp = nullptr;
  for (uint8_t i = 0; i < CONFIG_RESTART_DEBUG_STACK_DEPTH; i++) {
    if (debug->backtrace[i] != 0) {
      item = malloc_stringf("0x%08x", debug->backtrace[i]);
      if (item) {
        if (backtrace) {
          temp = backtrace;
          backtrace = malloc_stringf("%s %s", temp, item);
          free(item);
          free(temp);
        } else {
          backtrace = item;
        };
        item = nullptr;
      };
    } else {
      break;
    }
  };
  return backtrace;
}

#endif // CONFIG_RESTART_DEBUG_STACK_DEPTH
#endif // CONFIG_RESTART_DEBUG_INFO


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

    #if CONFIG_MQTT_OTA_ENABLE
      if (statesCheck(WIFI_STA_CONNECTED, false) && statesCheck(MQTT_CONNECTED, false)) {
        statesFirmwareVerifyCompete();
      };
    #endif // CONFIG_MQTT_OTA_ENABLE

    if ((statesCheck(TIME_SNTP_SYNC_OK, false) || statesCheck(TIME_RTC_ENABLED, false)) 
     && statesCheck(WIFI_STA_CONNECTED, false) 
     && statesCheck(INET_AVAILABLED, false) 
     && statesCheck(MQTT_CONNECTED, false)) {
      statesSet(SYSTEM_STARTED);
      eventLoopPostSystem(RE_SYS_STARTED, RE_SYS_SET, false, 0);
      #if CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_START
        #if CONFIG_RESTART_DEBUG_INFO
          re_restart_debug_t debug = debugGet();
          char* debug_heap = statesGetDebugHeap(&debug);
          char* debug_trace = nullptr;
          if (debug_heap) {
            #if CONFIG_RESTART_DEBUG_STACK_DEPTH > 0
              debug_trace = statesGetDebugTrace(&debug);
            #endif // CONFIG_RESTART_DEBUG_STACK_DEPTH
            if (debug_trace) {
              tgSend(MK_MAIN, CONFIG_NOTIFY_TELEGRAM_START_PRIORITY, CONFIG_NOTIFY_TELEGRAM_ALERT_START, CONFIG_TELEGRAM_DEVICE, 
                CONFIG_MESSAGE_TG_VERSION_TRACE, APP_VERSION, getResetReason(), getResetReasonRtc(0), getResetReasonRtc(1), 
                debug_heap, debug_trace);
              free(debug_trace);
            } else {
              tgSend(MK_MAIN, CONFIG_NOTIFY_TELEGRAM_START_PRIORITY, CONFIG_NOTIFY_TELEGRAM_ALERT_START, CONFIG_TELEGRAM_DEVICE, 
                CONFIG_MESSAGE_TG_VERSION_HEAP, APP_VERSION, getResetReason(), getResetReasonRtc(0), getResetReasonRtc(1), 
                debug_heap);
            };
            free(debug_heap);
          } else {
            tgSend(MK_MAIN, CONFIG_NOTIFY_TELEGRAM_START_PRIORITY, CONFIG_NOTIFY_TELEGRAM_ALERT_START, CONFIG_TELEGRAM_DEVICE, 
              CONFIG_MESSAGE_TG_VERSION_DEF, APP_VERSION, getResetReason(), getResetReasonRtc(0), getResetReasonRtc(1));
          };
        #else
          tgSend(MK_MAIN, CONFIG_NOTIFY_TELEGRAM_START_PRIORITY, CONFIG_NOTIFY_TELEGRAM_ALERT_START, CONFIG_TELEGRAM_DEVICE, 
            CONFIG_MESSAGE_TG_VERSION_DEF, APP_VERSION, getResetReason(), getResetReasonRtc(0), getResetReasonRtc(1));
        #endif // CONFIG_RESTART_DEBUG_INFO
      #endif // CONFIG_TELEGRAM_ENABLE && CONFIG_NOTIFY_TELEGRAM_START
    };
  };
}

static void statesEventHandlerSystem(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  // System started
  if (event_id == RE_SYS_STARTED) {
    #if CONFIG_HEAP_TRACING_STANDALONE
      heapLeaksStart();
    #endif // CONFIG_HEAP_TRACING_STANDALONE  
  }
  // OTA
  else if (event_id == RE_SYS_OTA) {
    re_system_event_data_t* data = (re_system_event_data_t*)event_data;
    if (data) {
      statesSetBit(SYSTEM_OTA, data->type != RE_SYS_CLEAR);
    };
  }
  // Error
  else if (event_id == RE_SYS_ERROR) {
    // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE_MODE, event_base, "RE_SYS_ERROR", data->type);
    if (event_data) {
      re_error_event_data_t* data = (re_error_event_data_t*)event_data;
      statesSetError(ERR_GENERAL, data->err_code != ESP_OK);
    };
  }
  // Telegram error
  else if (event_id == RE_SYS_TELEGRAM_ERROR) {
    // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE_MODE, event_base, "RE_SYS_TELEGRAM_ERROR", data->type);
    if (event_data) {
      re_error_event_data_t* data = (re_error_event_data_t*)event_data;
      statesSetError(ERR_TELEGRAM, data->err_code != ESP_OK);
    };
  }
  // OpenMon
  else if (event_id == RE_SYS_OPENMON_ERROR) {
    // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE_MODE, event_base, "RE_SYS_OPENMON_ERROR", data->type);
    if (event_data) {
      re_error_event_data_t* data = (re_error_event_data_t*)event_data;
      statesSetError(ERR_OPENMON, data->err_code != ESP_OK);
      #if ENABLE_NOTIFY_OPENMON_STATUS
        hmOpenMon.setState(data->err_code, time(nullptr));
      #endif // ENABLE_NOTIFY_OPENMON_STATUS
    };
  }
  // NarodMon
  else if (event_id == RE_SYS_NARODMON_ERROR) {
    // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE_MODE, event_base, "RE_SYS_NARODMON_ERROR", data->type);
    if (event_data) {
      re_error_event_data_t* data = (re_error_event_data_t*)event_data;
      statesSetError(ERR_NARODMON, data->err_code != ESP_OK);
      #if ENABLE_NOTIFY_NARODMON_STATUS
        hmNarodMon.setState(data->err_code, time(nullptr));
      #endif // ENABLE_NOTIFY_NARODMON_STATUS
    };
  }
  // ThingSpeak
  else if (event_id == RE_SYS_THINGSPEAK_ERROR) {
    // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE_MODE, event_base, "RE_SYS_THINGSPEAK_ERROR", data->type);
    if (event_data) {
      re_error_event_data_t* data = (re_error_event_data_t*)event_data;
      statesSetError(ERR_THINGSPEAK, data->err_code != ESP_OK);
      #if ENABLE_NOTIFY_THINGSPEAK_STATUS
        hmThingSpeak.setState(data->err_code, time(nullptr));
      #endif // ENABLE_NOTIFY_THINGSPEAK_STATUS
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

    case RE_TIME_EVERY_MINUTE:
      #if CONFIG_RESTART_DEBUG_INFO && CONFIG_RESTART_DEBUG_HEAP_SIZE_SCHEDULE
        debugHeapUpdate();
      #endif // CONFIG_RESTART_DEBUG_HEAP_SIZE_SCHEDULE
      #if CONFIG_HEAP_TRACING_STANDALONE
        heapLeaksUpdate();
      #endif // CONFIG_HEAP_TRACING_STANDALONE  
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
      #if ENABLE_NOTIFY_SILENT_MODE
        #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
        if (_hmNotifySilentMode) {
        #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
          tgSend(MK_SERVICE, CONFIG_NOTIFY_TELEGRAM_SILENT_MODE_PRIORITY, CONFIG_NOTIFY_TELEGRAM_ALERT_SILENT_MODE, CONFIG_TELEGRAM_DEVICE, 
            CONFIG_MESSAGE_TG_SILENT_MODE_ON);
        #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
        };
        #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
      #endif // ENABLE_NOTIFY_SILENT_MODE
      break;
    // Silent mode off
    case RE_TIME_SILENT_MODE_OFF:
      statesClear(TIME_SILENT_MODE);
      ledSysSetEnabled(true);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_TIME_SILENT_MODE_OFF");
      #if ENABLE_NOTIFY_SILENT_MODE
        #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
        if (_hmNotifySilentMode) {
        #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
          tgSend(MK_SERVICE, CONFIG_NOTIFY_TELEGRAM_SILENT_MODE_PRIORITY, CONFIG_NOTIFY_TELEGRAM_ALERT_SILENT_MODE, CONFIG_TELEGRAM_DEVICE, 
            CONFIG_MESSAGE_TG_SILENT_MODE_OFF);
        #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
        };
        #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
      #endif // ENABLE_NOTIFY_SILENT_MODE
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
      statesClear(WIFI_STA_STARTED | WIFI_STA_CONNECTED | INET_AVAILABLED | INET_SLOWDOWN | MQTT_CONNECTED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_WIFI_STA_INIT");
      wdtRestartMqttBreak();
      break;

    case RE_WIFI_STA_STARTED:
      statesSet(WIFI_STA_STARTED);
      statesClear(WIFI_STA_CONNECTED | INET_AVAILABLED | INET_SLOWDOWN | MQTT_CONNECTED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_WIFI_STA_STARTED");
      wdtRestartMqttBreak();
      break;

    case RE_WIFI_STA_GOT_IP:
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_WIFI_STA_GOT_IP");
      statesSet(WIFI_STA_CONNECTED | INET_AVAILABLED);
      statesClear(INET_SLOWDOWN | MQTT_CONNECTED);
      eventLoopPost(RE_WIFI_EVENTS, RE_WIFI_STA_PING_OK, nullptr, 0, portMAX_DELAY);
      #if CONFIG_ENABLE_STATES_NOTIFICATIONS
        healthMonitorsWiFiAvailable(true);
      #endif // CONFIG_ENABLE_STATES_NOTIFICATIONS
      statesEventCheckSystemStarted();
      wdtRestartMqttStart();
      break;

    case RE_WIFI_STA_DISCONNECTED:
    case RE_WIFI_STA_STOPPED:
      #if CONFIG_ENABLE_STATES_NOTIFICATIONS 
        if (statesCheck(WIFI_STA_CONNECTED, false)) {
          healthMonitorsWiFiUnavailable(ESP_ERR_INVALID_STATE);
        };
      #endif // CONFIG_ENABLE_STATES_NOTIFICATIONS
      statesClear(WIFI_STA_CONNECTED | INET_AVAILABLED | INET_SLOWDOWN | MQTT_CONNECTED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_WIFI_STA_DISCONNECTED / RE_WIFI_STA_STOPPED");
      wdtRestartMqttBreak();
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
      #if CONFIG_ENABLE_STATES_NOTIFICATIONS
        if (statesCheck(WIFI_STA_CONNECTED, false)) {
          healthMonitorsInetAvailable(true);
        };
      #endif // CONFIG_ENABLE_STATES_NOTIFICATIONS
      eventLoopPost(RE_WIFI_EVENTS, RE_WIFI_STA_PING_OK, nullptr, 0, portMAX_DELAY);
      statesEventCheckSystemStarted();
      wdtRestartMqttCheck();
      break;

    case RE_PING_INET_SLOWDOWN: {
        statesSet(INET_AVAILABLED | INET_SLOWDOWN);
        // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_PING_INET_SLOWDOWN");
      };
      break;

    case RE_PING_INET_UNAVAILABLE:
      statesClear(INET_AVAILABLED | INET_SLOWDOWN);
      eventLoopPost(RE_WIFI_EVENTS, RE_WIFI_STA_PING_FAILED, nullptr, 0, portMAX_DELAY);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_PING_INET_UNAVAILABLE");
      #if CONFIG_ENABLE_STATES_NOTIFICATIONS
        if (statesCheck(WIFI_STA_CONNECTED, false)) {
          if (event_data) {
            ping_inet_data_t* data = (ping_inet_data_t*)event_data;
            healthMonitorsInetUnavailable(ESP_ERR_TIMEOUT, data->time_unavailable);
          } else {
            healthMonitorsInetUnavailable(ESP_ERR_TIMEOUT, time(nullptr));
          };
        };
      #endif // CONFIG_ENABLE_STATES_NOTIFICATIONS
      wdtRestartMqttCheck();
      break;

    case RE_PING_MQTT1_AVAILABLE:
      statesSet(MQTT_1_ENABLED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_PING_MQTT1_AVAILABLE");
      #if ENABLE_NOTIFY_MQTT1_PING
        hmMqttPing1.setState(ESP_OK, time(nullptr));
      #endif // ENABLE_NOTIFY_MQTT1_PING
      break;

    case RE_PING_MQTT2_AVAILABLE:
      statesSet(MQTT_2_ENABLED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_PING_MQTT2_AVAILABLE");
      #if ENABLE_NOTIFY_MQTT2_PING
        hmMqttPing2.setState(ESP_OK, time(nullptr));
      #endif // ENABLE_NOTIFY_MQTT2_PING
      break;

    case RE_PING_MQTT1_UNAVAILABLE:
      statesClear(MQTT_1_ENABLED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_PING_MQTT1_UNAVAILABLE");
      #if ENABLE_NOTIFY_MQTT1_PING
        if (event_data) {
          ping_host_data_t* data = (ping_host_data_t*)event_data;
          hmMqttPing1.setState(ESP_ERR_TIMEOUT, data->time_unavailable);
        } else {
          hmMqttPing1.setState(ESP_ERR_TIMEOUT, time(nullptr));
        };         
      #endif // ENABLE_NOTIFY_MQTT1_PING
      break;

    case RE_PING_MQTT2_UNAVAILABLE:
      statesClear(MQTT_2_ENABLED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_PING_MQTT2_UNAVAILABLE");
      #if ENABLE_NOTIFY_MQTT2_PING
        if (event_data) {
          ping_host_data_t* data = (ping_host_data_t*)event_data;
          hmMqttPing2.setState(ESP_ERR_TIMEOUT, data->time_unavailable);
        } else {
          hmMqttPing2.setState(ESP_ERR_TIMEOUT, time(nullptr));
        };         
      #endif // ENABLE_NOTIFY_MQTT2_PING
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
      wdtRestartMqttBreak();
      if (event_data) {
        re_mqtt_event_data_t* data = (re_mqtt_event_data_t*)event_data;
        statesSetBit(MQTT_PRIMARY, data->primary);
        statesSetBit(MQTT_LOCAL, data->local);
        #if ENABLE_NOTIFY_MQTT_STATUS
          hmMqtt.setStateCustom(ESP_OK, time(nullptr), false, malloc_stringf("%s:%d", data->host, data->port));
        #endif // ENABLE_NOTIFY_MQTT_STATUS
        statesEventCheckSystemStarted();
      };
      break;

    case RE_MQTT_CONN_LOST:
      statesClear(MQTT_CONNECTED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_MQTT_CONN_LOST");
      wdtRestartMqttStart();
      if (event_data) {
        re_mqtt_event_data_t* data = (re_mqtt_event_data_t*)event_data;
        #if ENABLE_NOTIFY_MQTT_STATUS
          hmMqtt.setStateCustom(ESP_ERR_INVALID_STATE, time(nullptr), false, malloc_stringf("%s:%d", data->host, data->port));
        #endif // ENABLE_NOTIFY_MQTT_STATUS
      };
      break;

    case RE_MQTT_CONN_FAILED:
      statesClear(MQTT_CONNECTED);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_MQTT_CONN_FAILED");
      wdtRestartMqttStart();
      if (event_data) {
        re_mqtt_event_data_t* data = (re_mqtt_event_data_t*)event_data;
        #if ENABLE_NOTIFY_MQTT_STATUS
          hmMqtt.forcedTimeout();
          #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
          if (_hmNotifyMqtt) {
          #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
            tgSend(MK_SERVICE, CONFIG_NOTIFY_TELEGRAM_MQTT_ERRORS_PRIORITY, CONFIG_NOTIFY_TELEGRAM_ALERT_MQTT_ERRORS, CONFIG_TELEGRAM_DEVICE, 
              CONFIG_MESSAGE_TG_MQTT_CONN_FAILED, data->host, data->port);
          #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
          };
          #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
        #endif // ENABLE_NOTIFY_MQTT_STATUS
      };
      break;

    case RE_MQTT_SERVER_PRIMARY:
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_MQTT_SERVER_PRIMARY");
      #if defined(CONFIG_MQTT1_TYPE) && ENABLE_NOTIFY_MQTT_STATUS
        hmMqtt.forcedTimeout();
        #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
        if (_hmNotifyMqtt) {
        #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
          tgSend(MK_SERVICE, CONFIG_NOTIFY_TELEGRAM_MQTT_ERRORS_PRIORITY, CONFIG_NOTIFY_TELEGRAM_ALERT_MQTT_ERRORS, CONFIG_TELEGRAM_DEVICE, 
            CONFIG_MESSAGE_TG_MQTT_SERVER_CHANGE_PRIMARY, 
            #if CONFIG_MQTT1_TLS_ENABLED
              CONFIG_MQTT1_HOST, CONFIG_MQTT1_PORT_TLS
            #else
              CONFIG_MQTT1_HOST, CONFIG_MQTT1_PORT_TCP
            #endif // CONFIG_MQTT1_TLS_ENABLED
            );
        #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
        };
        #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
      #endif // ENABLE_NOTIFY_MQTT_STATUS
      break;

    case RE_MQTT_SERVER_RESERVED:
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_MQTT_SERVER_RESERVED");
      #if defined(CONFIG_MQTT2_TYPE) && ENABLE_NOTIFY_MQTT_STATUS
        hmMqtt.forcedTimeout();
        #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
        if (_hmNotifyMqtt) {
        #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
          tgSend(MK_SERVICE, CONFIG_NOTIFY_TELEGRAM_MQTT_ERRORS_PRIORITY, CONFIG_NOTIFY_TELEGRAM_ALERT_MQTT_ERRORS, CONFIG_TELEGRAM_DEVICE, 
            CONFIG_MESSAGE_TG_MQTT_SERVER_CHANGE_RESERVED, 
            #if CONFIG_MQTT2_TLS_ENABLED
              CONFIG_MQTT2_HOST, CONFIG_MQTT2_PORT_TLS
            #else
              CONFIG_MQTT2_HOST, CONFIG_MQTT2_PORT_TCP
            #endif // CONFIG_MQTT2_TLS_ENABLED
            );
        #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
        };
        #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
      #endif // ENABLE_NOTIFY_MQTT_STATUS
      break;

    case RE_MQTT_ERROR:
      statesSetErrors(ERR_MQTT);
      // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_MQTT_ERROR");
      #if ENABLE_NOTIFY_MQTT_ERRORS
        #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
        if (_hmNotifyMqttErrors) {
        #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
          if (event_data) {
            char* error = (char*)event_data;
            tgSend(MK_SERVICE, CONFIG_NOTIFY_TELEGRAM_MQTT_ERRORS_PRIORITY, CONFIG_NOTIFY_TELEGRAM_ALERT_MQTT_ERRORS, CONFIG_TELEGRAM_DEVICE, 
              CONFIG_MESSAGE_TG_MQTT_ERROR, error);
          };
        #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
        };
        #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
      #endif // ENABLE_NOTIFY_MQTT_ERRORS
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

#ifndef CONFIG_NO_SENSORS

static void statesEventHandlerSensor(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  // rlog_w(logTAG, DEBUG_LOG_EVENT_MESSAGE, event_base, "RE_SENSOR_STATUS_CHANGED");

  if (event_data != nullptr) {
    sensor_event_status_t* data = (sensor_event_status_t*)event_data;

    // Set new status
    switch (data->sensor_id) {
      case 1:
        statesSetError(ERR_SENSOR_1, ((sensor_status_t)data->new_status != SENSOR_STATUS_OK));
        break;
      case 2:
        statesSetError(ERR_SENSOR_2, ((sensor_status_t)data->new_status != SENSOR_STATUS_OK));
        break;
      case 3:
        statesSetError(ERR_SENSOR_3, ((sensor_status_t)data->new_status != SENSOR_STATUS_OK));
        break;
      case 4:
        statesSetError(ERR_SENSOR_4, ((sensor_status_t)data->new_status != SENSOR_STATUS_OK));
        break;
      case 5:
        statesSetError(ERR_SENSOR_5, ((sensor_status_t)data->new_status != SENSOR_STATUS_OK));
        break;
      case 6:
        statesSetError(ERR_SENSOR_6, ((sensor_status_t)data->new_status != SENSOR_STATUS_OK));
        break;
      case 7:
        statesSetError(ERR_SENSOR_7, ((sensor_status_t)data->new_status != SENSOR_STATUS_OK));
        break;
      default:
        statesSetError(ERR_SENSOR_0, ((sensor_status_t)data->new_status != SENSOR_STATUS_OK));
        break;
    };

    #if ENABLE_NOTIFY_SENSOR_STATE
      // Sensor status change notification
      #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
      if (_hmNotifySensors) {
      #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
        rSensor* sensor = (rSensor*)data->sensor;
        if ((sensor_status_t)data->new_status == SENSOR_STATUS_OK) {
          tgSend(MK_SERVICE, CONFIG_NOTIFY_TELEGRAM_SENSOR_PRIORITY, CONFIG_NOTIFY_TELEGRAM_ALERT_SENSOR_STATE, CONFIG_TELEGRAM_DEVICE, 
            CONFIG_MESSAGE_TG_SENSOR_OK, sensor->getName());
        } else {
          tgSend(MK_SERVICE, CONFIG_NOTIFY_TELEGRAM_SENSOR_PRIORITY, CONFIG_NOTIFY_TELEGRAM_ALERT_SENSOR_STATE, CONFIG_TELEGRAM_DEVICE, 
            CONFIG_MESSAGE_TG_SENSOR_ERROR, sensor->getName(), sensor->statusString((sensor_status_t)data->new_status));
        };
      #if CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
      };
      #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
    #endif // ENABLE_NOTIFY_SENSOR_STATE
  };
}

#endif // CONFIG_NO_SENSORS

bool statesEventHandlerRegister()
{
  rlog_d(logTAG, "Register system states event handlers...");
  bool ret = eventHandlerRegister(RE_TIME_EVENTS, ESP_EVENT_ANY_ID, &statesEventHandlerTime, nullptr)
          && eventHandlerRegister(RE_WIFI_EVENTS, ESP_EVENT_ANY_ID, &statesEventHandlerWiFi, nullptr)
          && eventHandlerRegister(RE_MQTT_EVENTS, ESP_EVENT_ANY_ID, &statesEventHandlerMqtt, nullptr)
          #if CONFIG_PINGER_ENABLE
            && eventHandlerRegister(RE_PING_EVENTS, ESP_EVENT_ANY_ID, &statesEventHandlerPing, nullptr)
          #endif // CONFIG_PINGER_ENABLE
          #ifndef CONFIG_NO_SENSORS
          && eventHandlerRegister(RE_SENSOR_EVENTS, ESP_EVENT_ANY_ID, &statesEventHandlerSensor, nullptr)
          #endif // CONFIG_NO_SENSORS
          && eventHandlerRegister(RE_SYSTEM_EVENTS, ESP_EVENT_ANY_ID, &statesEventHandlerSystem, nullptr);
  if (ret) {
    #if CONFIG_ENABLE_STATES_NOTIFICATIONS && CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
      healthMonitorsRegisterParameters();
    #endif // CONFIG_NOTIFY_TELEGRAM_CUSTOMIZABLE
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
  #ifndef CONFIG_NO_SENSORS
    eventHandlerUnregister(RE_SENSOR_EVENTS, ESP_EVENT_ANY_ID, &statesEventHandlerSensor);
  #endif // CONFIG_NO_SENSORS
  rlog_d(logTAG, "System states event handlers unregistered");
}