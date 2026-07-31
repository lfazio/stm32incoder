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
static clock_source_t s_clock_source = CLK_SRC_HSI;

/* Both sources reach 360 MHz VCO / 180 MHz SYSCLK; only the input divider
 * differs, because HSE is 8 MHz and HSI is 16 MHz. */
static HAL_StatusTypeDef pll_try(bool use_hse)
{
    RCC_OscInitTypeDef osc = {0};

    if (use_hse) {
        /* 8 MHz square wave from the ST-LINK MCO, hence BYPASS, not a crystal.
         * Requires SB54/SB16/SB50 ON and SB55 OFF [UM1724 7.9.1]. */
        osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
        osc.HSEState       = RCC_HSE_BYPASS;
        osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
        osc.PLL.PLLM       = 8;
    } else {
        osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
        osc.HSIState            = RCC_HSI_ON;
        osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
        osc.PLL.PLLM            = 16;
    }
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLN     = 360;
    osc.PLL.PLLP     = RCC_PLLP_DIV2;
    osc.PLL.PLLQ     = 8;
    osc.PLL.PLLR     = 2;
    return HAL_RCC_OscConfig(&osc);
}

/* HSE is the default because the Time Stamp field is specified accurate to
 * "better than 1% (based on the system oscillator)" (Product Guide 5.4.2), and
 * measurement against a logic analyser puts HSI at +1.40% and HSE at +0.02%.
 * HSI therefore cannot meet the timestamp specification.
 *
 * A board whose solder bridges do not route the ST-LINK MCO has no HSE, so
 * rather than hang we fall back to HSI and report it — a running emulator with
 * a known-inaccurate timestamp beats a dead board. Force one or the other with
 * -DSIMENC_CLOCK_SOURCE=HSE or =HSI. */
static void clock_init(void)
{
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

#if defined(SIMENC_CLOCK_SOURCE_HSI)
    if (pll_try(false) != HAL_OK) {
        while (1) { }
    }
    s_clock_source = CLK_SRC_HSI;
#else
    if (pll_try(true) == HAL_OK) {
        s_clock_source = CLK_SRC_HSE;
    } else {
# if defined(SIMENC_CLOCK_SOURCE_HSE)
        while (1) { }          /* HSE explicitly required but not present */
# else
        if (pll_try(false) != HAL_OK) {
            while (1) { }
        }
        s_clock_source = CLK_SRC_HSI;
# endif
    }
#endif

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

clock_source_t board_clock_source(void)
{
    return s_clock_source;
}

const char *board_clock_source_name(void)
{
    return (s_clock_source == CLK_SRC_HSE) ? "HSE(8MHz MCO)" : "HSI(16MHz RC)";
}

bool board_timestamp_in_spec(void)
{
    return s_clock_source == CLK_SRC_HSE;
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
