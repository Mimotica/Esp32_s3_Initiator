#include <stdio.h>               // Standard I/O library for printf and related functions
#include <string.h>              // String manipulation functions (e.g., memcpy, strncpy)
#include <inttypes.h>            // Format macros for fixed-width integer types (e.g., PRIu32, PRIu64)
#include "nvs_flash.h"           // Non-volatile storage (NVS) flash API for persistent storage
#include "esp_wifi.h"            // ESP‑Wi‑Fi API for configuring and controlling Wi‑Fi
#include "esp_netif.h"           // Network interface API for TCP/IP stack
#include "esp_event.h"           // Event loop library for registering and handling events
#include "esp_log.h"             // Logging library for ESP‑IDF
#include "freertos/FreeRTOS.h"   // FreeRTOS core API
#include "freertos/task.h"       // FreeRTOS task creation and management API
#include "freertos/event_groups.h"// FreeRTOS event groups API
#include "driver/gptimer.h"      // General-purpose timer (GPTimer) driver API
#include "esp_wifi_types.h"      // Definitions for Wi‑Fi types (e.g., wifi_second_chan_t)
#include "lwip/ip_addr.h"        // LwIP IP address definitions and printing macros
#include "esp_timer.h"           // For esp_timer_get_time()

// Wi‑Fi FTM settings (update these as needed)
#define HARD_CODED_SSID "ESP_FTM_AP"   // SSID of the target Wi‑Fi AP
#define HARD_CODED_PASS "12345678"     // Password of the target Wi‑Fi AP

static const char *TAG = "FTM_INITIATOR"; // Tag used for ESP_LOGx messages

// Event group for signaling reception of an FTM report
static EventGroupHandle_t s_ftm_event_group;
#define FTM_REPORT_BIT BIT0             // Bit mask for FTM report event

// Event group for signaling IP acquisition
static EventGroupHandle_t s_ip_event_group;
#define GOT_IP_BIT BIT0                 // Bit mask for IP acquired event

// Variables for FTM report measurements (round‑trip time and distance estimates)
static uint32_t s_rtt_est, s_dist_est;

// Hardware timer (GPTimer) variables (1 tick = 1 µs)
static gptimer_handle_t s_timer = NULL;
static const uint32_t TIMER_RESOLUTION_HZ = 1000000; // Timer runs at 1 MHz (1 µs resolution)
static uint64_t s_start_time = 0;  // TX timestamp captured before sending FTM frame (in µs)
static uint64_t s_end_time = 0;    // RX timestamp fallback if high‑precision driver not enabled

// Global variables to store AP information after connection
static uint8_t s_primary_chan = 0;     // Primary channel number of the AP
static uint8_t s_ap_bssid[6] = {0};    // BSSID (MAC address) of the AP

// -----------------------------------------------------------------------------
// Wi‑Fi event handler for STA and FTM report events
// -----------------------------------------------------------------------------
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_STA_CONNECTED) {
        // Called when station successfully associates with AP
        ESP_LOGI(TAG, "Connected to AP: %s", HARD_CODED_SSID);
        // Extract BSSID from event data and store it for FTM initiator
        wifi_event_sta_connected_t *conn_evt = (wifi_event_sta_connected_t *) event_data;
        memcpy(s_ap_bssid, conn_evt->bssid, sizeof(s_ap_bssid));
    }
    else if (event_id == WIFI_EVENT_FTM_REPORT) {
        // **Log the raw system clock at the moment of report arrival**
        uint64_t sys_us = esp_timer_get_time();
        ESP_LOGI(TAG, "SYSCLK: %" PRIu64 " µs", sys_us);

        // Called when an FTM report is received after initiating an FTM session
        wifi_event_ftm_report_t *ftm_event = (wifi_event_ftm_report_t *) event_data;

#ifdef CONFIG_FTM_HIGH_PRECISION
        // If high‑precision driver enabled, get RX timestamp from packet metadata (in ns)
        uint64_t rx_ts_ns = WIFI_PKT_RX_TIMESTAMP_NSEC(&(ftm_event->rx_ctrl));
        ESP_LOGI(TAG, "  RX timestamp (ns): %" PRIu64, rx_ts_ns);

        // Convert nanoseconds to microseconds for delta calc
        uint64_t rx_ts_us = rx_ts_ns / 1000ULL;
        ESP_LOGI(TAG, "  RX timestamp (µs): %" PRIu64, rx_ts_us);

        // Compute difference between RX and stored TX timestamps
        uint64_t delta_us = (rx_ts_us > s_start_time) ? (rx_ts_us - s_start_time) : 0;
        ESP_LOGI(TAG, "  TX: %" PRIu64 " µs, Δ = %" PRIu64 " µs",
                 s_start_time, delta_us);
#else
        // Fallback: read the hardware timer count at reception to compute delta
        ESP_ERROR_CHECK(gptimer_get_raw_count(s_timer, &s_end_time));
        ESP_LOGI(TAG, "  RX timestamp (µs): %" PRIu64, s_end_time);

        uint64_t delta_us = (s_end_time > s_start_time) ? (s_end_time - s_start_time) : 0;
        ESP_LOGI(TAG, "  TX: %" PRIu64 " µs, Δ = %" PRIu64 " µs",
                 s_start_time, delta_us);
#endif

        // Signal to the FTM initiator task that the report has arrived
        xEventGroupSetBits(s_ftm_event_group, FTM_REPORT_BIT);
    }
}

// -----------------------------------------------------------------------------
// IP event handler: Wait for the station to get an IP address.
// -----------------------------------------------------------------------------
static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    if (event_id == IP_EVENT_STA_GOT_IP) {
        // Called when DHCP assigns an IP to the station
        ip_event_got_ip_t *ip_evt = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&ip_evt->ip_info.ip)); // Print IP
        // Signal that IP has been acquired
        xEventGroupSetBits(s_ip_event_group, GOT_IP_BIT);
    }
}

// -----------------------------------------------------------------------------
// Initialize Wi‑Fi in station mode.
// -----------------------------------------------------------------------------
void initialise_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());                   // Initialize TCP/IP network interface
    ESP_ERROR_CHECK(esp_event_loop_create_default());    // Create default event loop
    esp_netif_create_default_wifi_sta();                 // Create default Wi‑Fi station netif

    // Configure Wi‑Fi driver with default settings
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));                // Initialize Wi‑Fi driver

    // Register event handler for Wi‑Fi events (connect, FTM reports, etc.)
    esp_event_handler_instance_t wifi_handler;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &wifi_handler));

    // Register event handler for IP events (DHCP)
    esp_event_handler_instance_t ip_handler;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL, &ip_handler));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));    // Set Wi‑Fi mode to station
    ESP_ERROR_CHECK(esp_wifi_start());                    // Start Wi‑Fi driver
    ESP_LOGI(TAG, "Wi‑Fi initialized in STA mode");
}

// -----------------------------------------------------------------------------
// Connect to AP and wait until association (IP acquired and channel valid).
// -----------------------------------------------------------------------------
bool wifi_cmd_sta_join(const char* ssid, const char* pass)
{
    wifi_config_t wifi_config = {0};                      // Zero‑initialize config struct
    // Copy SSID and password into the config (ensuring no overflow)
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config)); // Apply config
    ESP_ERROR_CHECK(esp_wifi_connect());                       // Initiate connection
    ESP_LOGI(TAG, "Connecting to AP: %s", ssid);

    // Create event group and wait for GOT_IP_BIT for up to 10 seconds
    s_ip_event_group = xEventGroupCreate();
    EventBits_t ip_bits = xEventGroupWaitBits(
        s_ip_event_group, GOT_IP_BIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(10000));
    if (!(ip_bits & GOT_IP_BIT)) {
        ESP_LOGE(TAG, "Failed to obtain IP address within timeout");
        return false;
    }
    ESP_LOGI(TAG, "IP address acquired");

    // Poll for a valid Wi‑Fi channel (primary_chan != 0)
    uint8_t primary_chan = 0;
    wifi_second_chan_t secondary_chan = WIFI_SECOND_CHAN_NONE;
    for (int i = 0; i < 10; i++) {
        if (esp_wifi_get_channel(&primary_chan, &secondary_chan) == ESP_OK && primary_chan != 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // Wait 500 ms before retrying
    }
    if (primary_chan == 0) {
        ESP_LOGE(TAG, "Failed to obtain a valid channel");
        return false;
    } else {
        s_primary_chan = primary_chan;   // Save channel for FTM sessions
        ESP_LOGI(TAG, "Associated on channel: %d", s_primary_chan);
    }
    return true;
}

// -----------------------------------------------------------------------------
// Initialize the hardware timer (GPTimer) without starting it.
// -----------------------------------------------------------------------------
void init_hardware_timer(void)
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, // Default clock source (~80 MHz APB clock)
        .direction = GPTIMER_COUNT_UP,      // Count upwards
        .resolution_hz = TIMER_RESOLUTION_HZ,// 1 MHz resolution (1 µs per tick)
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &s_timer)); // Create new GPTimer
    ESP_LOGI(TAG, "Hardware timer created");
}

// -----------------------------------------------------------------------------
// Timer callback: signals the FTM initiator task to start an FTM session.
// -----------------------------------------------------------------------------
static bool IRAM_ATTR gptimer_ftm_cb(gptimer_handle_t timer,
                                     const gptimer_alarm_event_data_t *edata,
                                     void *user_data)
{
    // user_data is the EventGroupHandle_t passed when registering callback
    EventGroupHandle_t timer_event_group = (EventGroupHandle_t)user_data;
    xEventGroupSetBits(timer_event_group, BIT0); // Signal the initiator task
    return false; // Return false so the alarm is not disabled
}

// -----------------------------------------------------------------------------
// FTM initiator task: waits for timer signal, then initiates an FTM session.
// -----------------------------------------------------------------------------
void ftm_initiator_task(void *pvParameters)
{
    // pvParameters is the EventGroupHandle_t for timer events
    EventGroupHandle_t timer_event_group = (EventGroupHandle_t)pvParameters;
    while (1) {
        // Wait indefinitely for BIT0 from timer callback
        EventBits_t bits = xEventGroupWaitBits(
            timer_event_group, BIT0, pdTRUE, pdFALSE, portMAX_DELAY);
        if (bits & BIT0) {
            // Stamp TX time with both timers
            ESP_ERROR_CHECK(gptimer_get_raw_count(s_timer, &s_start_time));
            uint64_t sys_us = esp_timer_get_time();
            ESP_LOGI(TAG, "SYSCLK: %" PRIu64 " µs, TX timer: %" PRIu64 " µs",
                     sys_us, s_start_time);

            // Configure the FTM initiator session parameters
            wifi_ftm_initiator_cfg_t ftm_cfg = {
                .frm_count = 8,           // Number of FTM request-response frames
                .burst_period = 2,        // Interval between bursts (unused here)
                .use_get_report_api = true,// Use callback-based report retrieval
            };

            // Fill in the responder MAC and channel
            memcpy(ftm_cfg.resp_mac, s_ap_bssid, sizeof(ftm_cfg.resp_mac));
            ftm_cfg.channel = s_primary_chan;

            // Start the FTM session
            esp_err_t res = esp_wifi_ftm_initiate_session(&ftm_cfg);
            if (res == ESP_OK) {
                ESP_LOGI(TAG, "FTM session started successfully");
                // Wait up to 10 seconds for the FTM report bit
                EventBits_t ftm_bits = xEventGroupWaitBits(
                    s_ftm_event_group, FTM_REPORT_BIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(10000));
                if (!(ftm_bits & FTM_REPORT_BIT)) {
                    ESP_LOGE(TAG, "FTM session timed out");
                    esp_wifi_ftm_end_session(); // End session on timeout
                }
            } else {
                ESP_LOGE(TAG, "Failed to start FTM session (err 0x%x)", res);
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Main Application Entry Point
// -----------------------------------------------------------------------------
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());             // Initialize NVS flash for storage
    s_ftm_event_group = xEventGroupCreate();       // Create event group for FTM reports
    EventGroupHandle_t timer_event_group = xEventGroupCreate(); // For timer signals

    initialise_wifi();                             // Set up Wi‑Fi in station mode
    if (!wifi_cmd_sta_join(HARD_CODED_SSID, HARD_CODED_PASS)) {
        ESP_LOGE(TAG, "Failed to join AP: %s", HARD_CODED_SSID);
        return;                                    // Abort if connection fails
    }
    ESP_LOGI(TAG, "Connected to AP: %s", HARD_CODED_SSID);

    init_hardware_timer();                         // Initialize but don't start timer

    // Configure timer alarm to fire every 1 second (1,000,000 µs)
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000000,                    // Alarm after 1e6 ticks (1 second)
        .flags.auto_reload_on_alarm = true,        // Automatically reload for periodic alarms
    };
    // Register the timer callback to set the event bit
    gptimer_event_callbacks_t cbs = { .on_alarm = gptimer_ftm_cb };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_timer, &cbs, timer_event_group));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_timer, &alarm_config));

    // Enable and start the timer
    ESP_ERROR_CHECK(gptimer_enable(s_timer));
    ESP_ERROR_CHECK(gptimer_start(s_timer));
    ESP_LOGI(TAG, "FTM Initiator: Timer started; FTM sessions will be triggered every second.");

    // Create the FreeRTOS task that handles FTM initiation
    xTaskCreate(ftm_initiator_task, "ftm_initiator_task", 4096, timer_event_group, 10, NULL);

    // Main loop can perform other application tasks (here we just sleep)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));            // Delay 1 second
    }
}
