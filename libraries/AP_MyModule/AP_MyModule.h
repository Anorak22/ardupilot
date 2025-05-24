#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Module/AP_Module.h>
#include <AP_BattMonitor/AP_BattMonitor.h>

#define MAXTEXTLENGTH 38
#define BUFSIZE 43
//uint8_t outputBuf [BUFSIZE];


uint8_t checksum(uint8_t *buf, uint8_t bufsize);

uint16_t sendTextline(AP_HAL::UARTDriver *uart, uint8_t *text, uint8_t line);

uint16_t sendTelemetry(AP_HAL::UARTDriver *uart,
                      uint16_t pitch, uint16_t roll, uint16_t alt, 
                      uint16_t X, uint16_t Y, uint8_t status, 
                      uint16_t I, uint16_t V, uint8_t db, uint16_t thrust);

class MyModule {
public:
    static MyModule *instance() {
        return _instance;
    }

    // Конструктор и инициализация
    MyModule();
    void init();
    void update();

    // Функции тестирования UART
    static void setup_uart(AP_HAL::UARTDriver *uart, const char *name);

    static const struct AP_Param::GroupInfo var_info[];
    
    void set_enabled(bool enabled) {
        _enabled = enabled;
    }

    bool _enabled = true;

private:
    static MyModule *_instance;
    uint32_t _last_run_ms;
    bool _initialized;
};