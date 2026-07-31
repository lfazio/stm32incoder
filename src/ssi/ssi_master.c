#include "ssi/ssi_master.h"
#include "board/board.h"

/* SPI2 lives on APB1, so its clock is PCLK1 = 45 MHz. The SSI clock window is
 * 100 kHz .. 2 MHz (section 5.4.1); with the SPI's power-of-two prescalers the
 * usable divisors are therefore 32 (1.406 MHz) through 256 (176 kHz). */
#define SSI_MASTER_MIN_HZ  100000u
#define SSI_MASTER_MAX_HZ  2000000u

static uint32_t s_clock_hz;
static uint8_t  s_br;   /* CR1.BR field value */

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void ssi_master_delay_us(uint32_t us)
{
    uint32_t start  = DWT->CYCCNT;
    uint32_t cycles = us * (SYSCLK_HZ / 1000000u);
    while ((DWT->CYCCNT - start) < cycles) { }
}

static void spi_apply(void)
{
    SSI_MASTER_SPI->CR1 &= ~SPI_CR1_SPE;
    /* Master, CPOL=1/CPHA=1 to match the slave, 8-bit, MSB first.
     * SSM=1 with SSI=1 holds the internal NSS high, which a master needs to
     * avoid a mode fault. */
    SSI_MASTER_SPI->CR1 = SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA
                        | SPI_CR1_SSM | SPI_CR1_SSI
                        | ((uint32_t)s_br << SPI_CR1_BR_Pos);
    SSI_MASTER_SPI->CR2 = 0;
    SSI_MASTER_SPI->CR1 |= SPI_CR1_SPE;
}

void ssi_master_init(void)
{
    GPIO_InitTypeDef io = {0};

    dwt_init();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();

    io.Pin       = SSI_MASTER_SCK_PIN;
    io.Mode      = GPIO_MODE_AF_PP;
    io.Pull      = GPIO_PULLUP;         /* SSI clock idles HIGH */
    io.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    io.Alternate = SSI_MASTER_AF;
    HAL_GPIO_Init(SSI_MASTER_SCK_GPIO, &io);

    io.Pin  = SSI_MASTER_MISO_PIN;
    io.Pull = GPIO_PULLUP;              /* DATA idles HIGH */
    HAL_GPIO_Init(SSI_MASTER_MISO_GPIO, &io);

    s_br       = 4;                      /* /32 */
    s_clock_hz = PCLK1_HZ >> (s_br + 1u);
    spi_apply();
}

uint32_t ssi_master_set_clock(uint32_t requested_hz)
{
    for (uint8_t br = 0; br < 8u; br++) {
        uint32_t hz = PCLK1_HZ >> (br + 1u);
        if (hz > SSI_MASTER_MAX_HZ || hz < SSI_MASTER_MIN_HZ) {
            continue;
        }
        if (hz <= requested_hz || s_clock_hz == 0u) {
            s_br       = br;
            s_clock_hz = hz;
            spi_apply();
            return s_clock_hz;
        }
    }
    /* Nothing at or below the request is legal: fall back to the slowest. */
    s_br       = 7;
    s_clock_hz = PCLK1_HZ >> 8u;
    spi_apply();
    return s_clock_hz;
}

uint32_t ssi_master_get_clock(void)
{
    return s_clock_hz;
}

bool ssi_master_data_idle_high(void)
{
    return (SSI_MASTER_MISO_GPIO->IDR & SSI_MASTER_MISO_PIN) != 0u;
}

bool ssi_loopback_check(bool *clock_ok, bool *data_ok)
{
    const uint32_t drive = SSI_MASTER_SCK_PIN | SSI_SLAVE_DATA_PIN;

    /* Silence the slave's first-edge handover first. Driving the clock pin low
     * here looks exactly like the start of a Read Cycle, and EXTI3_IRQHandler
     * would rewrite GPIOB->MODER underneath this test -- putting PB10 back into
     * alternate-function mode, so the pin would stop following ODR and every
     * wire would be misreported as OPEN. ssi_slave_start() re-arms it. */
    EXTI->IMR &= ~SSI_SLAVE_SCK_PIN;
    EXTI->PR   = SSI_SLAVE_SCK_PIN;

    uint32_t moder_save = GPIOB->MODER;
    uint32_t odr_save   = GPIOB->ODR;

    /* PB10 and PB4 as push-pull outputs; PB3/PB14 stay as they are and are
     * simply sampled through IDR, which reads the pad in any mode. */
    uint32_t m = moder_save;
    m &= ~((3u << (10u * 2u)) | (3u << (4u * 2u)));
    m |=  ((1u << (10u * 2u)) | (1u << (4u * 2u)));
    GPIOB->MODER = m;

    bool ck = true;
    bool dt = true;

    for (uint32_t lvl = 0; lvl <= 1u; lvl++) {
        GPIOB->BSRR = lvl ? drive : (drive << 16u);
        ssi_master_delay_us(50);

        uint32_t idr  = GPIOB->IDR;
        bool     want = (lvl != 0u);

        if (((idr & SSI_SLAVE_SCK_PIN) != 0u) != want) {
            ck = false;
        }
        if (((idr & SSI_MASTER_MISO_PIN) != 0u) != want) {
            dt = false;
        }
    }

    GPIOB->ODR   = odr_save;
    GPIOB->MODER = moder_save;

    *clock_ok = ck;
    *data_ok  = dt;
    return ck && dt;
}

bool ssi_master_read(uint8_t n_bits, uint32_t *raw)
{
    if ((n_bits % 8u) != 0u || n_bits > 32u) {
        return false;
    }
    if (!ssi_master_data_idle_high()) {
        return false;
    }

    uint8_t  nbytes = (uint8_t)(n_bits / 8u);
    uint32_t value  = 0;

    /* Writing a byte is what produces 8 clock pulses; the byte value itself is
     * irrelevant because the slave ignores MOSI. */
    for (uint8_t i = 0; i < nbytes; i++) {
        while (!(SSI_MASTER_SPI->SR & SPI_SR_TXE)) { }
        *(volatile uint8_t *)&SSI_MASTER_SPI->DR = 0xFFu;
        while (!(SSI_MASTER_SPI->SR & SPI_SR_RXNE)) { }
        value = (value << 8u) | *(volatile uint8_t *)&SSI_MASTER_SPI->DR;
    }
    while (SSI_MASTER_SPI->SR & SPI_SR_BSY) { }

    *raw = value;
    return true;
}
