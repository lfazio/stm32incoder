#include "board/board.h"

/* 180 MHz from either source:
 *   HSI 16 MHz : PLLM=16 -> 1 MHz VCO in, PLLN=360 -> 360 MHz VCO, PLLP=2
 *   HSE  8 MHz : PLLM=8  -> 1 MHz VCO in, PLLN=360 -> 360 MHz VCO, PLLP=2
 * Over-drive is mandatory above 168 MHz, and flash needs 5 wait states.
 *
 * AHB /1 = 180 MHz, APB1 /4 = 45 MHz, APB2 /2 = 90 MHz.
 * ST's own UART_Printf example for this board uses APB2 /1, which would clock
 * APB2 at 180 MHz -- twice the 90 MHz maximum stated in DS10693 3.6. We use /2.
 */
static void clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

#if defined(SIMENC_CLOCK_SOURCE_HSE)
    /* 8 MHz square wave from the ST-LINK MCO, hence BYPASS, not a crystal.
     * Requires SB54/SB16/SB50 ON and SB55 OFF [UM1724 7.9.1]. */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_BYPASS;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = 8;
#else
    osc.OscillatorType       = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState             = RCC_HSI_ON;
    osc.HSICalibrationValue  = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLSource        = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM             = 16;
#endif
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLN     = 360;
    osc.PLL.PLLP     = RCC_PLLP_DIV2;
    osc.PLL.PLLQ     = 8;
    osc.PLL.PLLR     = 2;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        while (1) { }
    }

    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        while (1) { }
    }

    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) {
        while (1) { }
    }
}

static void gpio_init(void)
{
    GPIO_InitTypeDef io = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* LD2 status LED. */
    io.Pin   = LED_PIN;
    io.Mode  = GPIO_MODE_OUTPUT_PP;
    io.Pull  = GPIO_NOPULL;
    io.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO, &io);

    /* B1 user button: external pull-up on the board, pressed = low. */
    io.Pin  = BUTTON_PIN;
    io.Mode = GPIO_MODE_INPUT;
    io.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BUTTON_GPIO, &io);

    /* Analog angle input. */
    io.Pin  = ANGLE_ADC_PIN;
    io.Mode = GPIO_MODE_ANALOG;
    io.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ANGLE_ADC_GPIO, &io);
}

void board_init(void)
{
    HAL_Init();
    clock_init();
    gpio_init();
}

void board_led_set(bool on)
{
    HAL_GPIO_WritePin(LED_GPIO, LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void board_led_toggle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO, LED_PIN);
}

bool board_button_pressed(void)
{
    return HAL_GPIO_ReadPin(BUTTON_GPIO, BUTTON_PIN) == GPIO_PIN_RESET;
}
