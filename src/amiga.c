/*
 * amiga.c
 * 
 * Inspect the Amiga keyboard serial protocol and remember key presses.
 * 
 * Written & released by Keir Fraser <keir.xen@gmail.com>
 * 
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

/* Amiga keyboard:
 *  B3: KBDAT
 *  B4: KBCLK
 */

#define gpio_amikbd gpiob
#define pin_amikbd_dat 3
#define pin_amikbd_clk 4

#define irq_tim3 29
void IRQ_29(void) __attribute__((alias("IRQ_amikbd_clk")));

static uint8_t keymap[0x68];

static void handshake(void)
{
    /* Allow time for CIA to clock the bit. */
    delay_us(3);

    /* Force handshake. 100us is plenty long enough. */
    gpio_configure_pin(gpio_amikbd, pin_amikbd_dat, GPO_opendrain(_2MHz, LOW));
    delay_us(100);
    gpio_configure_pin(gpio_amikbd, pin_amikbd_dat, GPI_pull_up);
}

static void IRQ_amikbd_clk(void)
{
    time_t t = time_now();
    int bit = gpio_read_pin(gpio_amikbd, pin_amikbd_dat);
    static uint8_t keycode, bitpos;
    static time_t timestamp;
    static int resync;

    (void)tim3->ccr1; /* clear irq */

    /* Sync to keycode start by observing delay in comms. */
    if ((time_diff(timestamp, t) > time_ms(1)) && (bitpos != 0)) {
        bitpos = 0;
        if (resync++) {
            /* Two OOS in a row! The keyboard has lost sync: Handshake. */
            handshake();
            return;
        }
    } else {
        resync = 0;
    }

    timestamp = t;
    bitpos = (bitpos + 1) & 7;

    if (bitpos == 0) {
        /* Received full byte. Decode and update the keymap. */
        keycode = ~keycode & 0x7f;
        if (keycode < sizeof(keymap)) {
            if (bit)
                keymap[keycode] = 3;
            else
                keymap[keycode] &= 2;
        }
        handshake();
    }

    keycode = (keycode << 1) | bit;
}

bool_t amiga_key_pressed(uint8_t keycode)
{
    uint8_t state;
    /* Grab keypress state and clear the sticky bit. */
    do {
        state = keymap[keycode];
    } while ((state & 2)
             && (cmpxchg(&keymap[keycode], state, state & 1) != state));
    /* Only return key presses while the modifier keys are held. */
    if ((keymap[AMI_L_CTRL] & 1) && (keymap[AMI_L_ALT] & 1))
        return !!state;
    return FALSE;
}

void amiga_init(void)
{
    /* PB3, PB4: Amiga Keyboard */
    gpio_configure_pin(gpio_amikbd, pin_amikbd_dat, GPI_pull_up);
    gpio_configure_pin(gpio_amikbd, 4, GPI_pull_up);

    /* We map PB4 (KBCLK) to TIM3,CH1 and use its input filter and edge 
     * detector to generate interrupts on clock transitions. */
    afio->mapr |= AFIO_MAPR_TIM3_REMAP_PARTIAL;
    /* f_sampling = 72MHz/8 = 9MHz, N=8 -> must be stable for 889us. */
    tim3->ccmr1 = TIM_CCMR1_CC1S(TIM_CCS_INPUT_TI1) | TIM_CCMR1_IC1F(9);
    tim3->ccer = TIM_CCER_CC1E; /* Rising edge */
    tim3->dier |= TIM_DIER_CC1IE;

    IRQx_set_prio(irq_tim3, AMIKBD_IRQ_PRI);
    IRQx_enable(irq_tim3);
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
