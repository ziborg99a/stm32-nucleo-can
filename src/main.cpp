#include "stm32f4xx_hal.h"
#include "canbus.hpp"

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef oscInit = {};
    RCC_ClkInitTypeDef clkInit = {};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    oscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscInit.HSEState = RCC_HSE_BYPASS;
    oscInit.PLL.PLLState = RCC_PLL_ON;
    oscInit.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscInit.PLL.PLLM = 4;
    oscInit.PLL.PLLN = 180;
    oscInit.PLL.PLLP = RCC_PLLP_DIV2;
    oscInit.PLL.PLLQ = 7;
    oscInit.PLL.PLLR = 2;
    HAL_RCC_OscConfig(&oscInit);

    clkInit.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clkInit.APB1CLKDivider = RCC_HCLK_DIV4;
    clkInit.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&clkInit, FLASH_LATENCY_5);
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpioInit = {};
    gpioInit.Pin = GPIO_PIN_5;
    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpioInit);
}

static void writeLittleEndian16(uint8_t* data, uint16_t value)
{
    data[0] = static_cast<uint8_t>(value & 0xFFu);
    data[1] = static_cast<uint8_t>(value >> 8);
}

static void transmitBatteryTelemetry(CanBus& canBus, uint8_t counter)
{
    uint8_t payload[8] = {};
    writeLittleEndian16(&payload[0], 1260u);
    writeLittleEndian16(&payload[2], 45u);
    writeLittleEndian16(&payload[4], 250u);
    payload[6] = 87u;
    payload[7] = counter;
    canBus.transmit(0x180, payload, sizeof(payload));
}

static void transmitVehicleStatus(CanBus& canBus, uint8_t counter)
{
    uint8_t payload[8] = {};
    writeLittleEndian16(&payload[0], 1850u);
    writeLittleEndian16(&payload[2], 523u);
    payload[4] = 0x01u;
    payload[5] = 0x03u;
    payload[6] = counter;
    canBus.transmit(0x100, payload, sizeof(payload));
}

static void transmitControlCommand(CanBus& canBus, uint8_t counter)
{
    uint8_t payload[8] = {};
    payload[0] = 0x01u;
    writeLittleEndian16(&payload[1], 500u);
    payload[3] = counter;
    canBus.transmit(0x200, payload, sizeof(payload));
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    CanBus canBus;
    canBus.start();
    canBus.processConsole();

    uint8_t counter = 0;

    while (true) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        canBus.processRx();
        canBus.processConsole();
        transmitVehicleStatus(canBus, counter);
        transmitBatteryTelemetry(canBus, counter);
        transmitControlCommand(canBus, counter);
        ++counter;
        HAL_Delay(100);
    }

    return 0;
}

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}
