#include "canbus.hpp"
#include "can_frame.hpp"

#include <cstdio>

CanBus::CanBus() = default;

void CanBus::initMonitorUart()
{
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpioInit{};
    gpioInit.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    huart2_.Instance = USART2;
    huart2_.Init.BaudRate = 115200;
    huart2_.Init.WordLength = UART_WORDLENGTH_8B;
    huart2_.Init.StopBits = UART_STOPBITS_1;
    huart2_.Init.Parity = UART_PARITY_NONE;
    huart2_.Init.Mode = UART_MODE_TX_RX;
    huart2_.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2_.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2_);
}

void CanBus::printReceivedFrame(const CanFrame& frame)
{
    char line[96];
    const char* idType = frame.extended ? "EXT" : "STD";
    int length = std::snprintf(line, sizeof(line), "CAN %s 0x%lX DLC %lu",
                               idType, static_cast<unsigned long>(frame.id),
                               static_cast<unsigned long>(frame.length));

    if (frame.remote) {
        length += std::snprintf(line + length, sizeof(line) - static_cast<size_t>(length), " RTR");
    } else {
        length += std::snprintf(line + length, sizeof(line) - static_cast<size_t>(length), " DATA");
        for (uint32_t index = 0; index < frame.length && index < 8u; ++index) {
            length += std::snprintf(line + length, sizeof(line) - static_cast<size_t>(length),
                                    " %02X", frame.data[index]);
        }
    }
    length += std::snprintf(line + length, sizeof(line) - static_cast<size_t>(length), "\r\n");
    HAL_UART_Transmit(&huart2_, reinterpret_cast<uint8_t*>(line), static_cast<uint16_t>(length), HAL_MAX_DELAY);
}

void CanBus::init()
{
    initMonitorUart();
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpioInit{};
    gpioInit.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    hcan_.Instance = CAN1;
    hcan_.Init.Prescaler = 3;
    hcan_.Init.Mode = CAN_MODE_NORMAL;
    hcan_.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan_.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan_.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan_.Init.TimeTriggeredMode = DISABLE;
    hcan_.Init.AutoBusOff = DISABLE;
    hcan_.Init.AutoWakeUp = DISABLE;
    hcan_.Init.AutoRetransmission = ENABLE;
    hcan_.Init.ReceiveFifoLocked = DISABLE;
    hcan_.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan_) != HAL_OK) {
        return;
    }

    CAN_FilterTypeDef filter{};
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan_, &filter) != HAL_OK) {
        return;
    }

    if (HAL_CAN_Start(&hcan_) != HAL_OK) {
        return;
    }

    initialized_ = true;
}

void CanBus::start()
{
    if (!initialized_) {
        init();
    }
}

bool CanBus::transmit(uint32_t id, const uint8_t* data, uint8_t length)
{
    if (!initialized_ || !isValidStandardCanFrame(id, data, length)) {
        return false;
    }

    CAN_TxHeaderTypeDef txHeader{};
    txHeader.StdId = id;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = length;
    txHeader.TransmitGlobalTime = DISABLE;

    uint32_t mailbox = 0;
    if (HAL_CAN_AddTxMessage(&hcan_, &txHeader, const_cast<uint8_t*>(data), &mailbox) != HAL_OK) {
        return false;
    }

    return true;
}

void CanBus::processRx()
{
    if (!initialized_) {
        return;
    }

    CanFrame frame{};
    while (receive(frame)) {
        printReceivedFrame(frame);
    }
}

bool CanBus::receive(CanFrame& frame)
{
    if (!initialized_ || HAL_CAN_GetRxFifoFillLevel(&hcan_, CAN_RX_FIFO0) == 0u) {
        return false;
    }

    CAN_RxHeaderTypeDef rxHeader{};
    uint8_t rxData[8] = {0};
    if (HAL_CAN_GetRxMessage(&hcan_, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
        return false;
    }

    frame.id = rxHeader.IDE == CAN_ID_STD ? rxHeader.StdId : rxHeader.ExtId;
    frame.length = static_cast<uint8_t>(rxHeader.DLC);
    frame.remote = rxHeader.RTR == CAN_RTR_REMOTE;
    frame.extended = rxHeader.IDE == CAN_ID_EXT;
    for (uint8_t index = 0; index < frame.length && index < 8u; ++index) {
        frame.data[index] = rxData[index];
    }
    return true;
}
