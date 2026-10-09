// I2C slave side of the HOJA link.
// The slave can't stretch the clock, so the next frame is loaded as soon as a read finishes.

#include "link_i2c.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_intr_alloc.h"
#include "esp_private/periph_ctrl.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "hal/i2c_hal.h"
#include "hal/i2c_ll.h"
#include "soc/i2c_periph.h"
#include "soc/i2c_struct.h"
#include "soc/i2c_reg.h"

#define LINK_I2C_SDA_IO   19
#define LINK_I2C_SCL_IO   20
#define LINK_I2C_ADDRESS  0x76
#define LINK_I2C_DEV      (&I2C0)

// Old RP2040 firmware asks for the version with this command byte
#define LEGACY_CMD_FW_VERSION 0xFD

// Reset the peripheral if the bus goes quiet this long
#define LINK_I2C_SILENCE_MS 250

static const char *TAG = "link_i2c";

static portMUX_TYPE _lock = portMUX_INITIALIZER_UNLOCKED;
static hlink_s _link;
static uint8_t _txq[8192];
static link_i2c_stats_s _bus;
static intr_handle_t _intr;
static volatile uint32_t _last_activity_ms;
static volatile uint32_t _last_peer_ms;
static volatile bool _legacy_reply_pending;

static void IRAM_ATTR _load_tx(const uint8_t *frame)
{
    i2c_ll_txfifo_rst(LINK_I2C_DEV);
    i2c_ll_write_txfifo(LINK_I2C_DEV, frame, HLINK_FRAME_SIZE);
}

static void IRAM_ATTR _load_next_frame(void)
{
    uint8_t frame[HLINK_FRAME_SIZE];

    if(_legacy_reply_pending)
    {
        // Old firmware reads the version from bytes 1 and 2
        _legacy_reply_pending = false;
        memset(frame, 0, sizeof(frame));
        frame[1] = BRIDGE_FW_VERSION >> 8;
        frame[2] = BRIDGE_FW_VERSION & 0xFF;
        _load_tx(frame);
        return;
    }

    hlink_build(&_link, frame);
    _load_tx(frame);
}

static void IRAM_ATTR _isr(void *arg)
{
    (void)arg;
    i2c_dev_t *dev = LINK_I2C_DEV;
    uint32_t status = dev->int_status.val;

    if(status & I2C_TRANS_COMPLETE_INT_ST_M)
    {
        portENTER_CRITICAL_ISR(&_lock);
        _bus.transactions++;

        // The next transfer may already have started, so take whole frames only
        uint32_t rx_count = dev->status_reg.rx_fifo_cnt;
        while(rx_count >= HLINK_FRAME_SIZE)
        {
            uint8_t frame[HLINK_FRAME_SIZE];
            i2c_ll_read_rxfifo(dev, frame, HLINK_FRAME_SIZE);
            rx_count -= HLINK_FRAME_SIZE;
            _bus.writes++;

            if(!hlink_receive(&_link, frame) && frame[0] == LEGACY_CMD_FW_VERSION)
            {
                _bus.legacy_probes++;
                _legacy_reply_pending = true;
                _load_next_frame();
            }
        }

        bool bus_busy = dev->status_reg.bus_busy;

        // A write that ended short of a frame
        if(rx_count && !bus_busy)
        {
            i2c_ll_rxfifo_rst(dev);
            _bus.short_writes++;
        }

        // The TX FIFO was read and the bus is idle, load the next frame.
        // Never touch it while a read is in progress.
        if(!bus_busy && dev->status_reg.tx_fifo_cnt < HLINK_FRAME_SIZE)
        {
            _bus.reads++;
            _load_next_frame();
        }

        portEXIT_CRITICAL_ISR(&_lock);
        _last_activity_ms = esp_timer_get_time() / 1000;
        _last_peer_ms = _last_activity_ms;
    }

    i2c_ll_clear_intr_mask(dev, status);
}

// Same bring-up as the old baseband's mitch_i2c.c
static void _peripheral_setup(void)
{
    static bool pins_done = false;
    static i2c_hal_context_t hal = {0};
    i2c_dev_t *dev = LINK_I2C_DEV;
    hal.dev = dev;

    i2c_config_t c = {
        .sda_io_num = LINK_I2C_SDA_IO,
        .scl_io_num = LINK_I2C_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .mode = I2C_MODE_SLAVE,
        .slave.maximum_speed = 1000 * 1000,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = LINK_I2C_ADDRESS,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &c));

    if(!pins_done)
    {
        i2c_set_pin(I2C_NUM_0, c.sda_io_num, c.scl_io_num, c.sda_pullup_en, c.scl_pullup_en, c.mode);
    }
    else
    {
        portENTER_CRITICAL(&_lock);
        i2c_hal_deinit(&hal);
        periph_module_disable(PERIPH_I2C0_MODULE);
        portEXIT_CRITICAL(&_lock);
    }
    pins_done = true;

    periph_module_enable(PERIPH_I2C0_MODULE);
    i2c_hal_init(&hal, I2C_NUM_0);

    i2c_ll_disable_intr_mask(dev, I2C_LL_INTR_MASK);
    i2c_ll_clear_intr_mask(dev, I2C_LL_INTR_MASK);

    i2c_ll_slave_init(dev);
    i2c_ll_set_data_mode(dev, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
    i2c_ll_txfifo_rst(dev);
    i2c_ll_rxfifo_rst(dev);
    i2c_ll_set_slave_addr(dev, LINK_I2C_ADDRESS, false);
    i2c_ll_set_rxfifo_full_thr(dev, HLINK_FRAME_SIZE);
    i2c_ll_set_txfifo_empty_thr(dev, 0);
    i2c_ll_master_set_filter(dev, 2);
    i2c_ll_set_sda_timing(dev, 10, 10);
    i2c_ll_set_tout(dev, 32000);

    portENTER_CRITICAL(&_lock);
    _load_next_frame();
    portEXIT_CRITICAL(&_lock);

    dev->int_ena.val |= I2C_TRANS_COMPLETE_INT_ENA_M;
}

void link_i2c_init(hlink_rx_cb_t rx_cb, void *ctx, uint8_t session)
{
    hlink_init(&_link, _txq, sizeof(_txq), session, rx_cb, ctx);
    _peripheral_setup();
    ESP_ERROR_CHECK(esp_intr_alloc(i2c_periph_signal[0].irq, ESP_INTR_FLAG_IRAM, _isr, NULL, &_intr));
    _last_activity_ms = esp_timer_get_time() / 1000;
    _last_peer_ms = _last_activity_ms;
    ESP_LOGI(TAG, "slave 0x%02X on SDA %d SCL %d", LINK_I2C_ADDRESS, LINK_I2C_SDA_IO, LINK_I2C_SCL_IO);
}

bool link_i2c_send2(uint8_t channel, const uint8_t *hdr, uint16_t hdr_len,
                    const uint8_t *body, uint16_t body_len, uint32_t wait_ms)
{
    TickType_t start = xTaskGetTickCount();
    for(;;)
    {
        portENTER_CRITICAL(&_lock);
        bool ok = hlink_send2(&_link, channel, hdr, hdr_len, body, body_len);
        portEXIT_CRITICAL(&_lock);

        if(ok) return true;
        if((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(wait_ms)) return false;
        vTaskDelay(1);
    }
}

bool link_i2c_send(uint8_t channel, const uint8_t *data, uint16_t len, uint32_t wait_ms)
{
    return link_i2c_send2(channel, data, len, NULL, 0, wait_ms);
}

void link_i2c_watchdog(uint32_t now_ms)
{
    if((now_ms - _last_activity_ms) < LINK_I2C_SILENCE_MS) return;

    // The RP2040 talks constantly once it has said hello, so silence means the slave got stuck.
    // Before that, only when the bus is held.
    if(!_link.peer_seen && !LINK_I2C_DEV->status_reg.bus_busy) return;

    esp_intr_disable(_intr);
    _peripheral_setup();
    _bus.reinits++;
    _last_activity_ms = now_ms;
    esp_intr_enable(_intr);
}

uint32_t link_i2c_peer_quiet_ms(uint32_t now_ms)
{
    return now_ms - _last_peer_ms;
}

void link_i2c_get_stats(hlink_stats_s *link, link_i2c_stats_s *bus)
{
    portENTER_CRITICAL(&_lock);
    if(link) *link = _link.stats;
    if(bus) *bus = _bus;
    portEXIT_CRITICAL(&_lock);
}
