#pragma once

#include "can_bus.hpp"
#include "stm32f4xx_hal.h"

class CanBus : public ICanBus {
public:
    CanBus();
    void init();
    void start() override;
    bool transmit(uint32_t id, const uint8_t* data, uint8_t length) override;
    bool receive(CanFrame& frame) override;
    void processRx() override;

private:
    CAN_HandleTypeDef hcan_{ };
    UART_HandleTypeDef huart2_{ };
    bool initialized_ = false;

    void initMonitorUart();
    void printReceivedFrame(const CanFrame& frame);
};
