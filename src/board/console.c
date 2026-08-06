#include "board/console.h"
#include "board/board.h"
#include "board/trace.h"
#include "encoder/incoder.h"
#include "encoder/position_source.h"
#include "ssi/ssi_master.h"
#include "ssi/ssi_slave.h"
#include "ssi/ssi_variant.h"

#include <string.h>

#define LINE_MAX 64

static char     s_line[LINE_MAX];
static uint32_t s_len;

/* Deliberately not strtok()/strtoul(): picolibc keeps strtok's saved pointer
 * in .tbss and sets errno, both of which live in thread-local storage. This
 * firmware links with --crt0=none, so no TLS block is ever initialised and
 * those accesses would read (and write) a garbage address. Parsing by hand
 * keeps the console free of hidden per-thread state. */

/* Splits off the next space-delimited token, advancing *cursor past it. */
static char *next_token(char **cursor)
{
    char *p = *cursor;

    while (*p == ' ' || *p == '\t') {
        p++;
    }
    if (*p == '\0') {
        *cursor = p;
        return NULL;
    }

    char *start = p;
    while (*p != '\0' && *p != ' ' && *p != '\t') {
        p++;
    }
    if (*p != '\0') {
        *p++ = '\0';
    }
    *cursor = p;
    return start;
}

/* Accepts decimal, or hex with a 0x prefix. Returns false on a bad digit. */
static bool parse_u32(const char *s, uint32_t *out)
{
    uint32_t base = 10u;
    uint32_t v    = 0;

    if (s == NULL || *s == '\0') {
        return false;
    }
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        base = 16u;
        s += 2;
        if (*s == '\0') {
            return false;
        }
    }

    for (; *s != '\0'; s++) {
        uint32_t d;
        if (*s >= '0' && *s <= '9') {
            d = (uint32_t)(*s - '0');
        } else if (base == 16u && *s >= 'a' && *s <= 'f') {
            d = (uint32_t)(*s - 'a') + 10u;
        } else if (base == 16u && *s >= 'A' && *s <= 'F') {
            d = (uint32_t)(*s - 'A') + 10u;
        } else {
            return false;
        }
        if (d >= base) {
            return false;
        }
        v = v * base + d;
    }
    *out = v;
    return true;
}

static bool parse_i32(const char *s, int32_t *out)
{
    bool neg = false;

    if (s == NULL || *s == '\0') {
        return false;
    }
    if (*s == '-') {
        neg = true;
        s++;
    } else if (*s == '+') {
        s++;
    }

    uint32_t mag;
    if (!parse_u32(s, &mag)) {
        return false;
    }
    *out = neg ? -(int32_t)mag : (int32_t)mag;
    return true;
}

void console_init(void)
{
    s_len = 0;
}

void console_banner(void)
{
    trace_printf("\r\n=== simenc: Zettlex IncOder emulator (SSI4) ===\r\n");
    trace_printf("board   : NUCLEO-F446RE @ %lu MHz, clock %s%s\r\n",
                 (unsigned long)(SYSCLK_HZ / 1000000u),
                 board_clock_source_name(),
                 board_timestamp_in_spec() ? "" : "  [timestamp OUT OF SPEC]");
    trace_printf("SSI slave: CLK=PB3(D3) DATA=PB4(D5)   %s, n=%u bits, Tmu=20us\r\n",
                 ssi_variant_name(incoder_variant()),
                 (unsigned)ssi_variant_frame_bits(incoder_variant()));
    trace_printf("SSI master(test): CLK=PB10(D6) DATA=PB14(CN10-28) @ %lu Hz\r\n",
                 (unsigned long)ssi_master_exact_hz());
    trace_printf("angle in : PA0 (A0), 0..VDDA -> 0..%lu counts\r\n",
                 (unsigned long)((1u << ssi_variant_position_bits(incoder_variant())) - 1u));
    trace_printf("type 'help' for commands\r\n");
}

/* One trace_printf per line: the whole listing is far longer than
 * trace_printf's 192-byte stack buffer, which silently truncated it. */
static void cmd_help(void)
{
    trace_printf("commands:\r\n");
    trace_printf("  help                 this list\r\n");
    trace_printf("  stat                 encoder + link state\r\n");
    trace_printf("  src adc|fixed|ramp   position source; adc is the default\r\n");
    trace_printf("  fixed <counts>       fixed position, width follows the variant\r\n");
    trace_printf("  ramp <step>          counts per 100us update, selects 'ramp'\r\n");
    trace_printf("  zero set|reset       zero point; reset restores factory (ZPD=1)\r\n");
    trace_printf("  err on|off           force PV=0; makes D31 a meaningful test bit\r\n");
    trace_printf("  ssi 1|2|4|6|9        payload variant; 4 is the default\r\n");
    trace_printf("test master (loopback only):\r\n");
    trace_printf("  wire                 check the loopback jumpers for continuity\r\n");
    trace_printf("  clk <hz>             clock rate, any value 100k..2M (timer)\r\n");
    trace_printf("  read [n]             n Read Cycles, decode each\r\n");
    trace_printf("  burst [n] [gapus]    n cycles, no tracing between; for capture\r\n");
    trace_printf("  readspi [n]          read via the SPI baud generator\r\n");
    trace_printf("  burstspi [n] [gapus] burst via the SPI baud generator\r\n");
}

static void cmd_stat(void)
{
    incoder_state_t st;
    ssi_slave_stats_t ss;

    incoder_get_state(&st);
    ssi_slave_get_stats(&ss);

    trace_printf("variant=%s n=%u bits pos=%u bits\r\n",
                 ssi_variant_name(incoder_variant()),
                 (unsigned)ssi_variant_frame_bits(incoder_variant()),
                 (unsigned)ssi_variant_position_bits(incoder_variant()));
    trace_printf("src=%s pos=%lu ts=%u pv=%u zpd=%u updates=%lu\r\n",
                 position_source_name(position_source_get()),
                 (unsigned long)st.position, (unsigned)st.timestamp,
                 (unsigned)st.valid, (unsigned)st.zero_default,
                 (unsigned long)st.updates);
    trace_printf("adc_raw=%u zero_off=%lu frames=%lu resyncs=%lu dropped=%lu\r\n",
                 (unsigned)position_source_raw_adc(),
                 (unsigned long)incoder_zero_offset(),
                 (unsigned long)ss.frames, (unsigned long)ss.resyncs,
                 (unsigned long)trace_dropped());
    trace_printf("data_line_idle_high=%u  measured_T=%luns\r\n",
                 (unsigned)ssi_master_data_idle_high(),
                 (unsigned long)ss.period_ns);
#if defined(SIMENC_CLOCK_COUNTER)
    /* Names the missing wire rather than leaving a dead link unexplained. */
    trace_printf("eom=clock-counter etr=%s\r\n",
                 ssi_slave_clock_counter_ok() ? "counting"
                                              : "NO EDGES - check PD2/CN7-4");
#endif
}

static void cmd_read(uint32_t count, bool fast)
{
    if (count == 0u) {
        count = 1u;
    }

    for (uint32_t i = 0; i < count; i++) {
        ssi_variant_t v = incoder_variant();
        uint8_t  n   = ssi_variant_frame_bits(v);
        uint32_t raw = 0;
        bool ok = fast ? ssi_master_read_timer(n, &raw)
                       : ssi_master_read(n, &raw);

        if (!ok) {
            trace_printf("read %lu: DATA not idle high, aborted\r\n", (unsigned long)i);
        } else {
            ssi_sample_t f;
            ssi_variant_unpack(v, raw, &f);
            trace_printf("read %lu: raw=%0*lX pv=%u zpd=%u pd=%lu ts=%u %s\r\n",
                         (unsigned long)i, n / 4, (unsigned long)raw,
                         (unsigned)f.pv, (unsigned)f.zpd,
                         (unsigned long)f.pd, (unsigned)f.ts,
                         ssi_variant_check(v, raw) ? "" : "CHECK-FAIL");
        }
        /* Timg must exceed Tmu (20 us); leave clear margin. */
        ssi_master_delay_us(200);
    }
}

/* Back-to-back Read Cycles with a fixed gap and no tracing in between, so a
 * logic analyser sees a clean regular pattern to trigger on. Tracing only
 * happens after the burst. The gap is well above Tmu (20 us) as Timg requires. */
static void cmd_burst(uint32_t count, bool fast, uint32_t gap_us)
{
    uint32_t ok = 0;
    uint32_t bad = 0;
    uint32_t first = 0;
    bool     have_first = false;

    if (count == 0u) {
        count = 1u;
    }
    if (gap_us < 25u) {
        gap_us = 25u;      /* Timg must exceed Tmu = 20 us */
    }

    /* Two warm-up cycles, not one. A frame is staged at the end of Tmu, so
     * after a value change the *first two* cycles can still carry the old
     * payload: cycle 0 was staged before the change, and cycle 1 may have been
     * staged before the console command finished. Taking the reference from
     * cycle 1 made every subsequent correct frame count as bad -- an ok=1
     * bad=38 that looked like a link failure and was purely this. */
    const uint32_t warmup = (count > 2u) ? 2u : 0u;

    for (uint32_t i = 0; i < count; i++) {
        ssi_variant_t v = incoder_variant();
        uint32_t raw = 0;
        bool got = fast ? ssi_master_read_timer(ssi_variant_frame_bits(v), &raw)
                        : ssi_master_read(ssi_variant_frame_bits(v), &raw);
        if (i < warmup) {
            ssi_master_delay_us(gap_us);
            continue;
        }
        if (!got) {
            bad++;
        } else {
            ssi_sample_t f;
            ssi_variant_unpack(v, raw, &f);
            if (!ssi_variant_check(v, raw)) {
                bad++;
                ssi_master_delay_us(gap_us);
                continue;
            }
            if (!have_first) {
                first = f.pd;
                have_first = true;
                ok++;
            } else if (f.pd == first) {
                ok++;
            } else {
                bad++;
            }
        }
        ssi_master_delay_us(gap_us);
    }

    trace_printf("burst: %lu cycles @ %lu Hz, gap %luus -> ok=%lu bad=%lu pd=%lu\r\n",
                 (unsigned long)count,
                 (unsigned long)(fast ? ssi_master_exact_hz() : ssi_master_get_clock()),
                 (unsigned long)gap_us, (unsigned long)ok,
                 (unsigned long)bad, (unsigned long)first);
}

static void handle_line(char *line)
{
    char *cursor = line;
    char *cmd    = next_token(&cursor);
    char *arg    = next_token(&cursor);

    if (cmd == NULL) {
        return;
    }

    if (strcmp(cmd, "help") == 0) {
        cmd_help();
    } else if (strcmp(cmd, "stat") == 0) {
        cmd_stat();
    } else if (strcmp(cmd, "src") == 0 && arg != NULL) {
        if (strcmp(arg, "adc") == 0) {
            position_source_select(POS_SRC_ADC);
        } else if (strcmp(arg, "fixed") == 0) {
            position_source_select(POS_SRC_FIXED);
        } else if (strcmp(arg, "ramp") == 0) {
            position_source_select(POS_SRC_RAMP);
        } else {
            trace_printf("unknown source '%s'\r\n", arg);
            return;
        }
        trace_printf("source = %s\r\n",
                     position_source_name(position_source_get()));
    } else if (strcmp(cmd, "fixed") == 0 && arg != NULL) {
        uint32_t v;
        if (!parse_u32(arg, &v)) {
            trace_printf("bad number '%s'\r\n", arg);
            return;
        }
        position_source_set_fixed(v);
        position_source_select(POS_SRC_FIXED);
        uint32_t mask = (1u << ssi_variant_position_bits(incoder_variant())) - 1u;
        trace_printf("fixed = %lu\r\n", (unsigned long)(v & mask));
    } else if (strcmp(cmd, "ramp") == 0 && arg != NULL) {
        int32_t step;
        if (!parse_i32(arg, &step)) {
            trace_printf("bad number '%s'\r\n", arg);
            return;
        }
        position_source_set_ramp_step(step);
        position_source_select(POS_SRC_RAMP);
        trace_printf("ramp step = %ld\r\n", (long)step);
    } else if (strcmp(cmd, "zero") == 0 && arg != NULL) {
        if (strcmp(arg, "set") == 0) {
            incoder_zero_set();
        } else if (strcmp(arg, "reset") == 0) {
            incoder_zero_reset();
        }
        trace_printf("zero offset = %lu\r\n", (unsigned long)incoder_zero_offset());
    } else if (strcmp(cmd, "clk") == 0 && arg != NULL) {
        uint32_t req;
        if (!parse_u32(arg, &req)) {
            trace_printf("bad number '%s'\r\n", arg);
            return;
        }
        /* The timer engine reaches 180 MHz / N, so anything in the SSI window
         * is available -- not just the SPI baud generator's four rates. */
        uint32_t got = ssi_master_set_exact_clock(req);
        uint32_t spi = ssi_master_set_clock(req);
        long err_ppt = (req != 0u)
            ? (long)(((int64_t)got - (int64_t)req) * 1000 / (int64_t)req) : 0;
        trace_printf("master clock = %lu Hz (timer, %+ld.%ld%%)  spi fallback %lu Hz\r\n",
                     (unsigned long)got, err_ppt / 10, (err_ppt < 0 ? -err_ppt : err_ppt) % 10,
                     (unsigned long)spi);
    } else if (strcmp(cmd, "wire") == 0) {
        bool ck = false;
        bool dt = false;
        bool ok = ssi_loopback_check(&ck, &dt);

        trace_printf("CLOCK PB10(CN10-25,D6) -> PB3(CN10-31,D3) : %s\r\n",
                     ck ? "OK" : "OPEN");
        trace_printf("DATA  PB4 (CN10-27,D5) -> PB14(CN10-28)   : %s\r\n",
                     dt ? "OK" : "OPEN");
        trace_printf("loopback %s\r\n", ok ? "ready" : "NOT wired");
        ssi_slave_start();      /* the test borrowed the DATA pin */
    } else if (strcmp(cmd, "read") == 0 || strcmp(cmd, "readspi") == 0) {
        uint32_t n    = 1u;
        bool     fast = (strcmp(cmd, "readspi") != 0);
        if (arg != NULL && !parse_u32(arg, &n)) {
            trace_printf("bad number '%s'\r\n", arg);
            return;
        }
        trace_printf("clocking at %lu Hz (%s)\r\n",
                     (unsigned long)(fast ? ssi_master_exact_hz()
                                          : ssi_master_get_clock()),
                     fast ? "timer" : "spi");
        cmd_read(n, fast);
        if (fast) {
            uint32_t d[7];
            ssi_master_timer_debug(d);
            trace_printf("  ndtr lo=%lu hi=%lu smp=%lu  tim cnt=%lu sr=%08lX\r\n",
                         (unsigned long)d[0], (unsigned long)d[1],
                         (unsigned long)d[2], (unsigned long)d[3],
                         (unsigned long)d[4]);
            trace_printf("  dma2 lisr=%08lX hisr=%08lX\r\n",
                         (unsigned long)d[5], (unsigned long)d[6]);
        }
    } else if (strcmp(cmd, "burst") == 0 || strcmp(cmd, "burstspi") == 0) {
        uint32_t n    = 50u;
        bool     fast = (strcmp(cmd, "burstspi") != 0);
        char    *gap  = next_token(&cursor);
        uint32_t g    = 100u;

        if (arg != NULL && !parse_u32(arg, &n)) {
            trace_printf("bad number '%s'\r\n", arg);
            return;
        }
        if (gap != NULL && !parse_u32(gap, &g)) {
            trace_printf("bad number '%s'\r\n", gap);
            return;
        }
        cmd_burst(n, fast, g);
    } else if (strcmp(cmd, "ssi") == 0 && arg != NULL) {
        ssi_variant_t v;
        if (!ssi_variant_from_name(arg, &v)) {
            trace_printf("unknown variant '%s' (try 1, 2, 4, 6, 9)\r\n", arg);
            return;
        }
        if (!incoder_set_variant(v)) {
            trace_printf("variant %s rejected\r\n", ssi_variant_name(v));
            return;
        }
        trace_printf("variant = %s, n = %u bits, position %u bits\r\n",
                     ssi_variant_name(v),
                     (unsigned)ssi_variant_frame_bits(v),
                     (unsigned)ssi_variant_position_bits(v));
    } else if (strcmp(cmd, "err") == 0 && arg != NULL) {
        incoder_force_error(strcmp(arg, "on") == 0);
        trace_printf("force error = %u (PV will read %u)\r\n",
                     (unsigned)incoder_error_forced(),
                     (unsigned)!incoder_error_forced());
    } else {
        trace_printf("unknown command '%s', try 'help'\r\n", cmd);
    }
}

void console_poll(void)
{
    if (!(TRACE_UART->SR & USART_SR_RXNE)) {
        return;
    }

    char c = (char)(TRACE_UART->DR & 0xFFu);

    if (c == '\r' || c == '\n') {
        if (s_len > 0u) {
            s_line[s_len] = '\0';
            trace_printf("\r\n");
            handle_line(s_line);
            s_len = 0;
        }
        return;
    }

    if (c == '\b' || c == 0x7F) {
        if (s_len > 0u) {
            s_len--;
            trace_write("\b \b", 3);
        }
        return;
    }

    if (s_len < (LINE_MAX - 1u)) {
        s_line[s_len++] = c;
        trace_write(&c, 1);
    }
}
