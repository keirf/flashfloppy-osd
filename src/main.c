/*
 * main.c
 * 
 * Bootstrap the STM32F103C8T6 and get things moving.
 * 
 * Written & released by Keir Fraser <keir.xen@gmail.com>
 * 
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

/*
 * PIN ASSIGNMENTS:
 * 
 * Rotary Encoder:
 *  A0: CLK
 *  A1: DAT
 *  A2: SEL
 * 
 * Serial Console:
 *  A9: TX
 *  A10: RX
 * 
 * I2C Interface (to Gotek):
 *  B6: CLK
 *  B7: DAT
 * 
 * Buttons (Gotek):
 *  C13: PREV/LEFT/DOWN
 *  C14: NEXT/RIGHT/UP
 *  C15: SELECT/EJECT
 * 
 * Display:
 *  A8: CSYNC or HSYNC
 *  B14: VSYNC (only needed with HSYNC)
 *  B15: Display output
 * 
 * Amiga keyboard:
 *  B3: KBDAT
 *  B4: KBCLK
 */

#define gpio_csync gpioa
#define pin_csync  8
#define irq_csync  23
void IRQ_23(void) __attribute__((alias("IRQ_csync"))); /* EXTI9_5 */

#define gpio_vsync gpiob
#define pin_vsync  14
#define irq_vsync  40
void IRQ_40(void) __attribute__((alias("IRQ_vsync"))); /* EXTI15_10 */

#define tim2_irq 28
//void IRQ_28(void) __attribute__((alias("IRQ_TIM2")));
#define tim2_up_dma (dma1->ch2)
#define tim2_up_dma_ch 2
#define tim2_up_dma_tc_irq 12
//void IRQ_12(void) __attribute__((alias("IRQ_TIM2_DMA_TC")));

#define tim3_irq 29
//void IRQ_29(void) __attribute__((alias("IRQ_TIM3")));
#define tim3_up_dma (dma1->ch3)
#define tim3_up_dma_ch 3
#define tim3_up_dma_tc_irq 13
//void IRQ_13(void) __attribute__((alias("IRQ_TIM3_DMA_TC")));

#define gpio_display gpiob
#define pin_display  15
#define spi_display  (spi2)
#define dma_display  (dma1->ch5)
#define dma_display_ch 5
#define dma_display_irq 15
void IRQ_15(void) __attribute__((alias("IRQ_display_dma_complete")));

int EXC_reset(void) __attribute__((alias("main")));

#include "font.h"

/* Guard the stacks with known values. */
static void canary_init(void)
{
    _irq_stackbottom[0] = _thread_stackbottom[0] = 0xdeadbeef;
}

/* Has either stack been clobbered? */
static void canary_check(void)
{
    ASSERT(_irq_stackbottom[0] == 0xdeadbeef);
    ASSERT(_thread_stackbottom[0] == 0xdeadbeef);
}

static struct timer button_timer;
static uint8_t rotary;
static volatile uint8_t buttons;
static void button_timer_fn(void *unused)
{
    /* Rotary encoder outputs a Gray code, counting clockwise: 00-01-11-10. */
    enum { ROT_none, ROT_full, ROT_half, ROT_quarter } rotary_type = ROT_full;
    const uint32_t rotary_transitions[] = {
        [ROT_none]    = 0x00000000, /* No encoder */
        [ROT_full]    = 0x20000100, /* 4 transitions (full cycle) per detent */
        [ROT_half]    = 0x24000018, /* 2 transitions (half cycle) per detent */
        [ROT_quarter] = 0x24428118  /* 1 transition (quarter cyc) per detent */
    };

    static uint16_t _b;
    uint8_t b = B_PROCESSED;

    /* We debounce the switch by waiting for it to be pressed continuously 
     * for 16 consecutive sample periods (16 * 5ms == 80ms) */
    _b <<= 1;
    _b |= gpio_read_pin(gpioa, 2);
    if (_b == 0)
        b |= B_SELECT;

    rotary = ((rotary << 2) | (gpioa->idr & 3)) & 15;
    b |= (rotary_transitions[rotary_type] >> (rotary << 1)) & 3;

    /* Latch final button state and reset the timer. */
    buttons |= b;
    timer_set(&button_timer, button_timer.deadline + time_ms(5));
    
}

static int hline, frame;
#define HLINE_EOF -1
#define HLINE_VBL 0
#define HLINE_SOF 1

static uint16_t display_dat[42][40/2+1];
static struct display *cur_display = &lcd_display;

static void IRQ_vsync(void)
{
    exti->pr = m(pin_vsync);
    tim1->smcr = 0;
    hline = HLINE_VBL;
}

static void IRQ_csync(void)
{
    exti->pr = m(pin_csync);

    if (hline <= 0) { /* EOF or VBL */

        static time_t p;
        time_t t = time_now();

        /* Trigger on both sync edges so we can measure sync pulse width: 
         * Normal Sync ~= 5us, Porch+Data ~= 59us */
        exti->ftsr |= m(pin_csync) | m(pin_vsync);
        exti->rtsr |= m(pin_csync) | m(pin_vsync);

        if (gpio_read_pin(gpio_csync, pin_csync) == config.polarity) {

            /* Sync pulse start: remember the current time. */
            p = t;

        } else if (time_diff(p, t) > time_us(10)) {

            /* Long sync: We are in vblank. */
            hline = HLINE_VBL;

        } else if (hline == HLINE_VBL) {

            /* Short sync: We are outside the vblank period. Start frame (we
             * were previously in vblank). */
            if (!cur_display->on)
                goto eof;
            hline = HLINE_SOF;
            set_polarity();

        }

    } else if (++hline == config.v_off) {

        /* Point SPI DMA at first row of data and enable the TIM1 trigger. */
        dma_display.cmar = (uint32_t)(unsigned long)display_dat;
        tim1->smcr = (TIM_SMCR_MSM
                      | TIM_SMCR_TS(5) /* Filtered TI1 */
                      | TIM_SMCR_SMS(4)); /* Reset Mode */

    } else if (hline >= (config.v_off + (cur_display->rows*10+2))) {

    eof:
        /* End of frame: Disable TIM1 trigger and signal main loop. */
        tim1->smcr = 0;
        hline = HLINE_EOF;
        frame++;

    }
}

static uint32_t gpio_display_crh;
static uint16_t dma_display_ccr = (DMA_CCR_PL_V_HIGH |
                                   DMA_CCR_MSIZE_16BIT |
                                   DMA_CCR_PSIZE_16BIT |
                                   DMA_CCR_MINC |
                                   DMA_CCR_DIR_M2P |
                                   DMA_CCR_TCIE |
                                   DMA_CCR_EN);

static void IRQ_display_dma_complete(void)
{
    /* Precise timing to disable the display output pin. */
    IRQ_global_disable();
    while (!(spi_display->sr & SPI_SR_TXE))
        cpu_relax();
    if (cur_display->cols & 1)
        delay_ticks(5);
    gpio_configure_pin(gpio_display, pin_display, GPI_floating);
    IRQ_global_enable();

    /* Reset display output SPI DMA. Point at next row of data. */
    dma_display.ccr = 0;
    dma_display.cndtr = cur_display->cols/2 + 1;
    dma_display.cmar += sizeof(display_dat[0]);
    dma1->ifcr = DMA_IFCR_CGIF(dma_display_ch);
}

/* Set up a slave timer to be triggered by TIM1. */
static void setup_slave_timer(TIM tim)
{
    tim->psc = 0;
    tim->egr = TIM_EGR_UG; /* update CNT, PSC, ARR */
    tim->cr2 = 0;
    tim->dier = TIM_DIER_UDE;
    tim->cr1 = TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_OPM;
    tim->smcr = (TIM_SMCR_TS(0) /* Timer 1 */
                 | TIM_SMCR_SMS(6)); /* Trigger Mode (starts counter) */
}

void slave_arr_update(void)
{
    unsigned int hstart = config.h_off * 20;
    tim2->arr = hstart-1;
    tim3->arr = hstart-65;
}

void set_polarity(void)
{
    if (config.polarity) {
        /* Active High: Rising edge = sync start */
        exti->ftsr &= ~(m(pin_csync) | m(pin_vsync));
        exti->rtsr |= m(pin_csync) | m(pin_vsync); /* Rising edge */
        tim1->ccer = TIM_CCER_CC1E | TIM_CCER_CC1P; /* Falling edge */
    } else {
        /* Active Low: Falling edge = sync start */
        exti->rtsr &= ~(m(pin_csync) | m(pin_vsync));
        exti->ftsr |= m(pin_csync) | m(pin_vsync); /* Falling edge */
        tim1->ccer = TIM_CCER_CC1E; /* Rising edge */
    }
}

static void wakeup(void *_c)
{
    struct cancellation *c = _c;
    cancel_call(c);
}

static int wait(void *_c)
{
    struct cancellation *c = _c;
    struct timer t;

    timer_init(&t, wakeup, c);
    timer_set(&t, time_now() + time_ms(5));

    for (;;) ;

    return 0;
}

static void render_line(unsigned int y, const struct display *display)
{
    unsigned int x, row;
    const uint8_t *t;
    uint16_t *d = display_dat[y];

    memset(d, 0, sizeof(display_dat[0]));

    y -= 2;

    row = y / 10;
    if (row >= display->rows)
        return;

    y %= 10;
    if (y >= 8)
        return;

    t = display->text[row];

    for (x = 0; x < display->cols; x++) {
        uint8_t c = *t++;
        if ((c < 0x20) || (c > 0x7f))
            c = 0x20;
        c -= 0x20;
        d[x/2] |= (uint16_t)font[(c<<3)+y] << ((x&1)?0:8);
    }
}

/* We snapshot the relevant Amiga keys so that we can scan the keymap (and 
 * clear the sticky bits) in one place in the main loop. */
static uint8_t keys;
#define K_LEFT   B_LEFT
#define K_RIGHT  B_RIGHT
#define K_SELECT B_SELECT
#define K_MENU   8

static void update_amiga_keys(void)
{
    keys = 0;
    if (amiga_key_pressed(AMI_LEFT)) keys |= K_LEFT;
    if (amiga_key_pressed(AMI_RIGHT)) keys |= K_RIGHT;
    if (amiga_key_pressed(AMI_UP)) keys |= K_SELECT;
    if (amiga_key_pressed(AMI_F1)) keys |= K_MENU;
}

struct gotek_button {
    bool_t pressed;
    time_t t;
} gl, gr, gs;

static bool_t gotek_active;
static void emulate_gotek_button(
    uint8_t keycode, struct gotek_button *button, int pin)
{
    bool_t pressed = (keys & keycode) && gotek_active;
    if (!(pressed ^ button->pressed))
        return; /* no change */
    if (pressed) {
        button->t = time_now();
        button->pressed = TRUE;
        gpio_write_pin(gpioc, pin, LOW);
    } else if (time_diff(button->t, time_now()) > time_ms(200)) {
        button->pressed = FALSE;
        gpio_write_pin(gpioc, pin, HIGH);
    }
}

static void emulate_gotek_buttons(void)
{
    if (config_active)
        gotek_active = FALSE;
    else if (!gotek_active && !keys)
        gotek_active = TRUE; /* only after keys are released */
    emulate_gotek_button(K_LEFT, &gl, 13);
    emulate_gotek_button(K_RIGHT, &gr, 14);
    emulate_gotek_button(K_SELECT, &gs, 15);
}

int main(void)
{
    int i;
    time_t frame_time;
    bool_t lost_sync;

    /* Relocate DATA. Initialise BSS. */
    if (_sdat != _ldat)
        memcpy(_sdat, _ldat, _edat-_sdat);
    memset(_sbss, 0, _ebss-_sbss);

    canary_init();

    stm32_init();
    time_init();
    console_init();
    lcd_init();

    /* PA0, PA1, PA2: Rotary encoder */
    for (i = 0; i < 3; i++)
        gpio_configure_pin(gpioa, i, GPI_pull_up);

    /* PA8 = CSYNC/HSYNC input */
    gpio_configure_pin(gpio_csync, pin_csync, GPI_pull_up);

    /* PB14 = VSYNC input */
    gpio_configure_pin(gpio_vsync, pin_vsync, GPI_pull_up);

    /* PB15 = Colour output */
    gpio_configure_pin(gpio_display, pin_display, GPI_floating);

    /* PC13,14,15: Gotek buttons */
    gpio_configure_pin(gpioc, 13, GPO_opendrain(_2MHz, HIGH));
    gpio_configure_pin(gpioc, 14, GPO_opendrain(_2MHz, HIGH));
    gpio_configure_pin(gpioc, 15, GPO_opendrain(_2MHz, HIGH));

    /* Turn on the clocks. */
    rcc->apb1enr |= (RCC_APB1ENR_SPI2EN
                     | RCC_APB1ENR_TIM2EN
                     | RCC_APB1ENR_TIM3EN);
    rcc->apb2enr |= RCC_APB2ENR_TIM1EN;

    config_init();

    /* Configure SPI: 8-bit mode, MSB first, CPOL Low, CPHA Leading Edge. */
    spi_display->cr2 = SPI_CR2_TXDMAEN;
    spi_display->cr1 = (SPI_CR1_MSTR | /* master */
                        SPI_CR1_SSM | SPI_CR1_SSI | /* software NSS */
                        SPI_CR1_SPE | /* enable */
                        SPI_CR1_DFF | /* 16-bit */
                        SPI_CR1_CPHA |
                        SPI_CR1_BR_DIV4); /* 9MHz */

    /* Display DMA setup: From memory into the Display Timer's CCRx. */
    dma_display.cpar = (uint32_t)(unsigned long)&spi_display->dr;
    IRQx_set_prio(dma_display_irq, DISPLAY_IRQ_PRI);
    IRQx_enable(dma_display_irq);
    IRQx_set_pending(dma_display_irq);

    /* PA8 -> EXTI8 ; PB14 -> EXTI14 */
    afio->exticr4 |= 0x0100;
    exti->imr |= m(pin_csync) | m(pin_vsync);
    IRQx_set_prio(irq_csync, SYNC_IRQ_PRI);
    IRQx_set_prio(irq_vsync, SYNC_IRQ_PRI);
    IRQx_enable(irq_csync);
    IRQx_enable(irq_vsync);

    /* Timer 2 is triggered by Timer 1. On underflow it triggers DMA 
     * to start SPI transfer for the current hline. */
    tim2_up_dma.cpar = (uint32_t)(unsigned long)&dma_display.ccr;
    tim2_up_dma.cmar = (uint32_t)(unsigned long)&dma_display_ccr;
    tim2_up_dma.cndtr = 1;
    tim2_up_dma.ccr = (DMA_CCR_PL_V_HIGH |
                       DMA_CCR_MSIZE_16BIT |
                       DMA_CCR_PSIZE_32BIT |
                       DMA_CCR_CIRC |
                       DMA_CCR_DIR_M2P |
                       DMA_CCR_EN);
    setup_slave_timer(tim2);

    /* Timer 3 is triggered by Timer 1. On underflow it triggers DMA 
     * to switch on the SPI output pin. */
    gpio_configure_pin(gpio_display, pin_display, AFO_pushpull(_50MHz));
    gpio_display_crh = gpio_display->crh;
    gpio_configure_pin(gpio_display, pin_display, GPI_floating);
    tim3_up_dma.cpar = (uint32_t)(unsigned long)&gpio_display->crh;
    tim3_up_dma.cmar = (uint32_t)(unsigned long)&gpio_display_crh;
    tim3_up_dma.cndtr = 1;
    tim3_up_dma.ccr = (DMA_CCR_PL_V_HIGH |
                       DMA_CCR_MSIZE_32BIT |
                       DMA_CCR_PSIZE_32BIT |
                       DMA_CCR_CIRC |
                       DMA_CCR_DIR_M2P |
                       DMA_CCR_EN);
    setup_slave_timer(tim3);

    /* CSYNC is on Timer 1 Channel 1. Use it to trigger Timer 2 and 3. */
    tim1->psc = 0;
    tim1->arr = 0;
    tim1->ccmr1 = TIM_CCMR1_CC1S(TIM_CCS_INPUT_TI1);
    tim1->dier = 0;
    tim1->cr2 = TIM_CR2_MMS(2); /* UEV -> TRGO */
    tim1->cr1 = 0;

    slave_arr_update();
    set_polarity();

    amiga_init();

    rotary = (gpioc->idr >> 10) & 3;
    timer_init(&button_timer, button_timer_fn, NULL);
    timer_set(&button_timer, time_now());

    frame_time = time_now();
    lost_sync = FALSE;

    for (;;) {

        canary_check();

        /* Quiesce the CPU as much as possible while displaying OSD box. */
        if (hline >= (config.v_off - 3)) {
            struct cancellation wait_c;
            timer_cancel(&button_timer); /* avoids IRQ glitches */
            call_cancellable_fn(&wait_c, wait, &wait_c);
            timer_set(&button_timer, time_now());
        }

        /* Check for losing sync: no valid frame in over 100ms. */
        if (!lost_sync && (time_diff(frame_time, time_now()) > time_ms(100))) {
            lost_sync = TRUE;
            IRQ_global_disable();
            tim1->smcr = 0;
            hline = HLINE_EOF;
            IRQ_global_enable();
            printk("Sync lost\n");
        }

        /* Have we just finished generating a frame? */
        if (frame) {
            if (lost_sync) {
                printk("Sync found\n");
                lost_sync = FALSE;
            }
            frame_time = time_now();
            frame = 0;
            cur_display = config_active ? &config_display : &lcd_display;
            for (i = 0; i < cur_display->rows*10+2; i++)
                render_line(i, cur_display);
        }

        update_amiga_keys();
        emulate_gotek_buttons();

        if (buttons) {
            /* Atomically snapshot and clear the button state. */
            uint8_t b;
            uint32_t oldpri;
            oldpri = IRQ_save(TIMER_IRQ_PRI);
            b = buttons;
            buttons = 0;
            IRQ_restore(oldpri);
            /* Fold in keyboard presses. */
            if (config_active) {
                b |= keys & (B_LEFT | B_RIGHT | B_SELECT);
            } else {
                if (keys & K_MENU) b |= B_SELECT;
            }
            /* Pass button presses to config subsystem for processing. */
            config_process(b & ~B_PROCESSED);
        }

        lcd_process();
    }

    return 0;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "Linux"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
