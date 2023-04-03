#include "TWAI.h"

#include "Arduino.h"

namespace {
    ESP32TWAI *object = nullptr;
}

ESP32TWAI *ESP32TWAI::instance() {
    return object;
}

void ESP32TWAI::initialize(int crxPin, int ctxPin, long baudrate, twai_mode_t mode, bool autorecover, uint8_t rx_queue_length, uint8_t tx_queue_length, uint32_t acceptance_filter, uint32_t filter_mask, bool extended) {
    if (object) {
        return;
    }

    object = new ESP32TWAI(crxPin, ctxPin, baudrate, mode, autorecover, rx_queue_length, tx_queue_length,acceptance_filter, filter_mask, extended);
}

// constructor
ESP32TWAI::ESP32TWAI(int crxPin, int ctxPin, long baudrate, twai_mode_t mode, bool autorecover, uint8_t rx_queue_length, uint8_t tx_queue_length, uint32_t acceptance_filter, uint32_t filter_mask, bool extended) {
    _g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)ctxPin, (gpio_num_t)crxPin, mode);
    _g_config.rx_queue_len = rx_queue_length;
    _g_config.tx_queue_len = tx_queue_length;
    // Set filter
    if (acceptance_filter == 0)
    {
        _f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    }
    else
    {
        // Bitshift input filter based on extended or standard frame
        if (extended)
        {
            _f_config.acceptance_code = (acceptance_filter << (32 - 29));
            _f_config.acceptance_mask = ((~(CAN_EXTD_ID_MASK << (32 - 29))) | (filter_mask << (32 - 29)));
        }
        // Otherwise it is a normal CAN frame (mostly used)
        else
        {
            _f_config.acceptance_code = (acceptance_filter << (32 - 11));
            _f_config.acceptance_mask = ((~(CAN_STD_ID_MASK << (32 - 11))) | (filter_mask << (32 - 11)));
        }
        _f_config.single_filter = true;
    }
    setBaudrate(baudrate);
    this->autorecover = autorecover;
}

// destructor
ESP32TWAI::~ESP32TWAI() {
    end();
}

esp_err_t ESP32TWAI::install() {
    esp_err_t err = twai_driver_install(&_g_config, &_t_config, &_f_config);
    _driver_installed = err == ESP_OK;

    return err;
}

esp_err_t ESP32TWAI::start(bool async) {
    if (!_driver_installed) {
        return ESP_ERR_INVALID_STATE;
        _driver_started = false;
    }
    esp_err_t err = twai_start();
    _driver_started = err == ESP_OK;

    if (_driver_started) 
    {
        twai_reconfigure_alerts(TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_BUS_OFF | TWAI_ALERT_RX_FIFO_OVERRUN  | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_ERR_PASS | TWAI_ALERT_TX_SUCCESS, NULL);
    }

    if (async) {
        this->async = true;
        xTaskCreatePinnedToCore([](void* param){
            while (true) {
                object->poll();
                vTaskDelay(5 / portTICK_PERIOD_MS);
            }
        }, "twaiTask", 16384, NULL, 4, &_task_handle, 1);
    }

    return err;
}

esp_err_t ESP32TWAI::end() {
    if (async && _task_handle) {
        vTaskDelete(_task_handle);
    }
    esp_err_t err = twai_driver_uninstall();
    if (err != ESP_OK){
        return err;
    }
    return twai_stop();
}

void ESP32TWAI::setMode(twai_mode_t mode) {
    if (isRunning()) {
        Serial.println("[TWAI] Can not change mode while driver is running");
        return;
    }
    _g_config.mode = mode;
}

void ESP32TWAI::setBaudrate(long baudrate) {
    switch (baudrate)
    {

    #if (SOC_TWAI_BRP_MAX > 256)
    case 1000:
        _t_config = TWAI_TIMING_CONFIG_1KBITS();
        break;
    case 5000:
        _t_config = TWAI_TIMING_CONFIG_5KBITS();
        break;
    case 10000:
        _t_config = TWAI_TIMING_CONFIG_10KBITS();
        break;
    #endif
    #if (SOC_TWAI_BRP_MAX > 128) || (CONFIG_ESP32_REV_MIN >= 2)
    case 12500:
        _t_config = TWAI_TIMING_CONFIG_12_5KBITS();
        break;
    case 16000:
        _t_config = TWAI_TIMING_CONFIG_16KBITS();
        break;
    case 20000:
        _t_config = TWAI_TIMING_CONFIG_20KBITS();
        break;
    #endif
    case 25000:
        _t_config = TWAI_TIMING_CONFIG_25KBITS();
        break;
    case 50000:
        _t_config = TWAI_TIMING_CONFIG_50KBITS();
        break;
    case 100000:
        _t_config = TWAI_TIMING_CONFIG_100KBITS();
        break;
    case 125000:
        _t_config = TWAI_TIMING_CONFIG_125KBITS();
        break;
    case 250000:
        _t_config = TWAI_TIMING_CONFIG_250KBITS();
        break;
    case 500000:
        _t_config = TWAI_TIMING_CONFIG_500KBITS();
        break;
    case 800000:
        _t_config = TWAI_TIMING_CONFIG_800KBITS();
        break;
    case 1000000:
        _t_config = TWAI_TIMING_CONFIG_1MBITS();
        break;
    default:
        _t_config = TWAI_TIMING_CONFIG_125KBITS();
        Serial.printf("[TWAI] Unsupported baudrate %dbits. Defaulting to 125kBits.\n", baudrate);
    }
}

int ESP32TWAI::availableMessages() {
    if (twai_get_status_info(&_status_info) == ESP_OK)
    {
        return _status_info.msgs_to_rx;
    }
    return -1;
}

esp_err_t ESP32TWAI::receiveMessage(uint16_t wait) {
    return twai_receive(&rxMessage, pdMS_TO_TICKS(wait));
}


esp_err_t ESP32TWAI::sendMessage(uint16_t wait) {
    return twai_transmit(&txMessage, pdMS_TO_TICKS(wait));
}

void ESP32TWAI::readAlerts(uint16_t wait) {
    uint32_t alerts;
    twai_status_info_t status_info;
    twai_read_alerts(&alerts, pdMS_TO_TICKS(wait));
    twai_get_status_info(&status_info);

    if (alerts != _alerts && alerts != 0x00) 
    {
        _alerts = alerts;

        if (alerts & TWAI_ALERT_BUS_RECOVERED) 
        {
            Serial.println("[TWAI] TWAI controller has successfully completed bus recovery");
            if (_bus_recovered_cb) {
                _bus_recovered_cb();
            }
        }
        if (alerts & TWAI_ALERT_ERR_PASS)
        {
            Serial.println("[TWAI] TWAI controller has become error passive.");
            Serial.printf("State: %d\n", status_info.state);
            Serial.printf("TX queue size: %d\n", status_info.msgs_to_tx);
            Serial.printf("TX error counter: %d\n", status_info.tx_error_counter);
            Serial.printf("TX failed counter: %d\n", status_info.tx_failed_count);
            Serial.printf("RX queue size: %d\n", status_info.msgs_to_rx);
            Serial.printf("RX error counter: %d\n", status_info.rx_error_counter);
            Serial.printf("RX missed counter: %d\n", status_info.rx_missed_count);
            Serial.printf("BUS error count: %d\n", status_info.bus_error_count);
        }
        if (alerts & TWAI_ALERT_RX_QUEUE_FULL) 
        {
            Serial.println("[TWAI] The RX queue is full causing a received frame to be lost. Clearing queue");
            Serial.printf("RX buffered: %d\t", status_info.msgs_to_rx);
            Serial.printf("RX missed: %d\n", status_info.rx_missed_count);
            Serial.printf("RX overrun %d\n", status_info.rx_overrun_count);
            if (_rx_queue_full_cb) {
                _rx_queue_full_cb();
            }
            // TODO: Make this configurable
            twai_clear_receive_queue();
        }
        if (alerts & TWAI_ALERT_BUS_ERROR)
        {
            Serial.println("[TWAI] A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
            Serial.printf("Bus error count: %d\n", status_info.bus_error_count);
        }
        if (alerts & TWAI_ALERT_BUS_OFF) 
        {
            if (_bus_off_cb) {
                _bus_off_cb();
            }
            if (autorecover) {
                Serial.println("[TWAI] Bus-off condition occurred. TWAI controller can no longer influence bus. Initiating recovery");
                twai_initiate_recovery();
            } else {
                Serial.println("[TWAI] Bus-off condition occurred. TWAI controller can no longer influence bus");
            }
        }
    }
}

// Add check for TX alert? such that the code will wait until it is transmitted?
bool ESP32TWAI::txSuccess(uint16_t wait)
{
    // TODO: This should probably be a global variable that will only be updated during
    uint32_t alerts;
    twai_read_alerts(&alerts, pdMS_TO_TICKS(wait));
    if (alerts & TWAI_ALERT_TX_SUCCESS)
    {
        return true;
        Serial.println("[TWAI] Tx success!");
    }
    else
    {
        return false;
    }
    
}

void ESP32TWAI::poll() {
    twai->readAlerts();

    while (twai->availableMessages() > 0) {
        twai->receiveMessage();
        if (_message_received_cb) {
            _message_received_cb(rxMessage);
        }
    }
}