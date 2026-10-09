// HOJA ESP32 HCI bridge
// The ESP32 only runs its Bluetooth controller, the host runs on the RP2040.
// HCI packets are relayed between the HOJA link (I2C) and VHCI.

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_bt.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "link_i2c.h"

static const char *TAG = "bridge";

static RingbufHandle_t _rx_ring; // [channel][data]
static SemaphoreHandle_t _vhci_ready;
static volatile bool _radio_up = false;
static volatile uint32_t _vhci_dropped = 0;
static volatile uint32_t _rx_ring_dropped = 0;
static volatile bool _hci_packet_lost = false;

// With the radio up and the RP2040 quiet this long, restart so hosts see the links drop
#define PEER_QUIET_RESTART_MS 2000

static uint32_t _now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

// Runs in the interrupt
static bool IRAM_ATTR _link_rx(void *ctx, uint8_t channel, const uint8_t *data, uint16_t len)
{
    (void)ctx;

    // Refuse when full and the RP2040 sends it again. Ring items have an 8 byte header.
    if(xRingbufferGetCurFreeSize(_rx_ring) < (size_t)len + 1 + 8)
    {
        _rx_ring_dropped++;
        return channel == HLINK_CH_IDLE;
    }

    static uint8_t item[HLINK_PACKET_MAX + 1];
    item[0] = channel;
    if(len) memcpy(&item[1], data, len);

    BaseType_t woken = pdFALSE;
    if(xRingbufferSendFromISR(_rx_ring, item, len + 1, &woken) != pdTRUE)
        _rx_ring_dropped++;
    if(woken) portYIELD_FROM_ISR();
    return true;
}

static void _vhci_send_available(void)
{
    xSemaphoreGive(_vhci_ready);
}

// Controller to host. The H4 packet type is also the link channel.
static int _vhci_recv(uint8_t *data, uint16_t len)
{
    if(len < 1) return 0;
    if(!link_i2c_send2(data[0], NULL, 0, &data[1], len - 1, 100))
    {
        _vhci_dropped++;
        _hci_packet_lost = true;
    }
    return 0;
}

static const esp_vhci_host_callback_t _vhci_cb = {
    .notify_host_send_available = _vhci_send_available,
    .notify_host_recv = _vhci_recv,
};

static void _ctrl_send(const uint8_t *msg, uint16_t len)
{
    link_i2c_send(HLINK_CH_CTRL, msg, len, 50);
}

static uint8_t _radio_start(const uint8_t bd_addr[6])
{
    // Only once per boot, the RP2040 resets us to change the address
    if(_radio_up) return 0;

    esp_err_t err = esp_iface_mac_addr_set(bd_addr, ESP_MAC_BT);
    if(err != ESP_OK)
        ESP_LOGW(TAG, "set BT address: %s", esp_err_to_name(err));

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&cfg);
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "controller init: %s", esp_err_to_name(err));
        return 1;
    }

    err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "controller enable: %s", esp_err_to_name(err));
        return 2;
    }

    esp_vhci_host_register_callback(&_vhci_cb);
    _radio_up = true;

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    ESP_LOGI(TAG, "radio up, address %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return 0;
}

// Host to controller
static void _vhci_send(uint8_t *h4, uint16_t len)
{
    if(!_radio_up) return;

    uint32_t start = _now_ms();
    while(!esp_vhci_host_check_send_available())
    {
        xSemaphoreTake(_vhci_ready, pdMS_TO_TICKS(5));
        if((_now_ms() - start) > 500)
        {
            _vhci_dropped++;
            _hci_packet_lost = true;
            return;
        }
    }
    esp_vhci_host_send_packet(h4, len);
}

// Battery on an ESP32 ADC pin (GCU 1: GPIO36, 75k/75k divider)
#define BATMON_PERIOD_MS 1000
#define BATMON_DIVIDER   2

static adc_oneshot_unit_handle_t _adc;
static adc_cali_handle_t _adc_cali;
static adc_channel_t _adc_channel;
static bool _batmon_on = false;
static uint32_t _batmon_last_ms = 0;

static void _batmon_start(uint8_t gpio)
{
    if(_batmon_on) return;

    adc_unit_t unit;
    if(adc_oneshot_io_to_channel(gpio, &unit, &_adc_channel) != ESP_OK || unit != ADC_UNIT_1)
    {
        ESP_LOGW(TAG, "battery monitor: GPIO %u is not an ADC1 pin", gpio);
        return;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &_adc));
    adc_oneshot_chan_cfg_t chan_cfg = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(_adc, _adc_channel, &chan_cfg));

    adc_cali_line_fitting_config_t cali_cfg = {.unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    if(adc_cali_create_scheme_line_fitting(&cali_cfg, &_adc_cali) != ESP_OK)
        _adc_cali = NULL;

    _batmon_on = true;
    _batmon_last_ms = 0;
    ESP_LOGI(TAG, "battery monitor on GPIO %u%s", gpio, _adc_cali ? "" : " (uncalibrated)");
}

static void _batmon_task(uint32_t now_ms)
{
    if(!_batmon_on || (now_ms - _batmon_last_ms) < BATMON_PERIOD_MS) return;
    _batmon_last_ms = now_ms;

    int raw = 0, mv = 0;
    if(adc_oneshot_read(_adc, _adc_channel, &raw) != ESP_OK) return;
    if(!_adc_cali || adc_cali_raw_to_voltage(_adc_cali, raw, &mv) != ESP_OK)
        mv = (raw * 3300) / 4095;
    mv *= BATMON_DIVIDER;

    uint8_t msg[] = {HLINK_CTRL_BATTERY, mv >> 8, mv & 0xFF};
    link_i2c_send(HLINK_CH_CTRL, msg, sizeof(msg), 10);
}

static void _print_stats(void)
{
    hlink_stats_s l;
    link_i2c_stats_s b;
    link_i2c_get_stats(&l, &b);
    printf("[link] tx %lu rx %lu frames | crc %lu ooo %lu retx %lu | pkts tx %lu rx %lu drop %lu | restarts %lu\n",
           l.frames_tx, l.frames_rx, l.crc_errors, l.out_of_order, l.retransmits,
           l.packets_tx, l.packets_rx, l.packets_dropped, l.peer_restarts);
    printf("[bus]  xfers %lu w %lu r %lu short %lu legacy %lu reinit %lu | vhci drop %lu ring full %lu\n",
           b.transactions, b.writes, b.reads, b.short_writes, b.legacy_probes, b.reinits,
           _vhci_dropped, _rx_ring_dropped);
}

static void _handle_ctrl(const uint8_t *msg, uint16_t len)
{
    if(!len) return;

    switch(msg[0])
    {
        case HLINK_CTRL_HELLO:
        {
            uint8_t rsp[] = {HLINK_CTRL_HELLO_RSP, HLINK_PROTOCOL_VERSION, BRIDGE_FW_VERSION >> 8, BRIDGE_FW_VERSION & 0xFF};
            _ctrl_send(rsp, sizeof(rsp));
            ESP_LOGI(TAG, "RP2040 hello, protocol %u", len > 1 ? msg[1] : 0);
        }
        break;

        case HLINK_CTRL_START_RADIO:
        if(len >= 7)
        {
            uint8_t rsp[] = {HLINK_CTRL_RADIO_READY, _radio_start(&msg[1])};
            _ctrl_send(rsp, sizeof(rsp));
        }
        break;

        case HLINK_CTRL_BATMON:
        if(len >= 3 && msg[1])
            _batmon_start(msg[2]);
        break;

        default:
        break;
    }
}

// A lost HCI packet leaves the RP2040 waiting on a reply that never comes, and a quiet RP2040
// leaves hosts connected to a controller that does nothing. The RP2040 recovers from a restart.
static void _restart_if_stuck(uint32_t now_ms)
{
    const char *why = NULL;
    if(_hci_packet_lost)
        why = "HCI packet lost";
    else if(_radio_up && link_i2c_peer_quiet_ms(now_ms) >= PEER_QUIET_RESTART_MS)
        why = "RP2040 went quiet";

    if(!why) return;
    ESP_LOGE(TAG, "restarting, %s", why);
    esp_restart();
}

static void _bridge_task(void *arg)
{
    (void)arg;
    esp_task_wdt_add(NULL);
    for(;;)
    {
        esp_task_wdt_reset();

        size_t size = 0;
        uint8_t *item = xRingbufferReceive(_rx_ring, &size, pdMS_TO_TICKS(20));
        if(item)
        {
            uint8_t ch = item[0];
            uint8_t *data = &item[1];
            uint16_t len = size - 1;

            switch(ch)
            {
                case HLINK_CH_IDLE:
                ESP_LOGW(TAG, "RP2040 restarted");
                break;

                // The item is already [H4 type][packet]
                case HLINK_CH_HCI_CMD:
                case HLINK_CH_HCI_ACL:
                case HLINK_CH_HCI_SCO:
                _vhci_send(item, size);
                break;

                case HLINK_CH_CTRL:
                _handle_ctrl(data, len);
                break;

                case HLINK_CH_LOG:
                printf("[rp] %.*s", len, (const char *)data);
                if(len && data[len - 1] != '\n') printf("\n");
                break;

                default:
                break;
            }
            vRingbufferReturnItem(_rx_ring, item);
        }

        link_i2c_watchdog(_now_ms());
        _restart_if_stuck(_now_ms());
        _batmon_task(_now_ms());
    }
}

// Console lines go to the RP2040, apart from "stats"
static void _console_task(void *arg)
{
    (void)arg;
    char line[64];
    size_t n = 0;
    for(;;)
    {
        uint8_t c;
        if(uart_read_bytes(UART_NUM_0, &c, 1, portMAX_DELAY) != 1) continue;

        if(c == '\r' || c == '\n')
        {
            if(!n) continue;
            line[n] = 0;

            if(!strcmp(line, "stats"))
            {
                _print_stats();
            }
            else
            {
                uint8_t msg[sizeof(line) + 1];
                msg[0] = HLINK_CTRL_CONSOLE;
                memcpy(&msg[1], line, n);
                _ctrl_send(msg, n + 1);
                printf("> %s\n", line);
            }
            n = 0;
        }
        else if(n < sizeof(line) - 1)
        {
            line[n++] = c;
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "HOJA HCI bridge 0x%04X, link protocol %u", BRIDGE_FW_VERSION, HLINK_PROTOCOL_VERSION);

    // The PHY keeps its RF calibration in NVS
    esp_err_t nvs = nvs_flash_init();
    if(nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_flash_init();
    }

    _rx_ring = xRingbufferCreate(16 * 1024, RINGBUF_TYPE_NOSPLIT);
    _vhci_ready = xSemaphoreCreateBinary();

    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

    link_i2c_init(_link_rx, NULL, esp_random());

    xTaskCreatePinnedToCore(_bridge_task, "bridge", 6144, NULL, 20, NULL, 0);
    xTaskCreatePinnedToCore(_console_task, "console", 3072, NULL, 5, NULL, 0);
}
