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

static uint8_t keymap[0x68];

/* Reconfiguring KB_DAT in IRQ context is safe as this modifies GPIOB_CRL which 
 * is otherwise static. The only other dynamically-modified configuration 
 * register is GPIOB_CRH (for toggling display output pin). */
#define dat_pull_low() \
    gpio_configure_pin(gpio_amikbd, pin_amikbd_dat, GPO_opendrain(_2MHz, LOW))
#define dat_input() \
    gpio_configure_pin(gpio_amikbd, pin_amikbd_dat, GPI_pull_up)

static void handshake(void)
{
    time_t t = time_now();

    /* Wait for rising edge. */
    while ((time_diff(t, time_now()) < time_us(40))
           && !gpio_read_pin(gpio_amikbd, pin_amikbd_clk))
        continue;

    /* Allow time for CIA to clock the bit. */
    delay_us(5);

    /* Force handshake. 100us is plenty long enough. */
    dat_pull_low();
    delay_us(100);
    dat_input();
}

bool_t keyboard_held;
static bool_t modifiers_held(void)
{
    return (keymap[AMI_L_CTRL] & 1) && (keymap[AMI_L_ALT] & 1);
}

void IRQ_amikbd_clk(void)
{
    time_t t = time_now();
    int bit = gpio_read_pin(gpio_amikbd, pin_amikbd_dat);
    static uint8_t keycode, bitpos;
    static time_t timestamp;
    static int resync;

    /* Clear the irq line. */
    (void)tim3->ccr1;

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
    keycode = (keycode << 1) | bit;

    /* Bail if we have not yet received a full byte. */
    if (bitpos != 0)
        return;

    /* Forcibly filter out key presses modified by L.Ctrl + L.Alt: We force
     * them to be viewed as key-release events by blatting KBDAT. */
    if (bit && (modifiers_held() || keyboard_held))
        dat_pull_low();

    /* Decode the keycode and update the keymap. */
    keycode = ~(keycode >> 1) & 0x7f;
    if (keycode < sizeof(keymap)) {
        if (bit) {
            if ((keycode == AMI_RETURN) && modifiers_held())
                keyboard_held ^= 1;
            keymap[keycode] = 3;
        } else {
            keymap[keycode] &= 2;
        }
    }

    keycode |= !bit << 7;
    if (keycode == 0xfd) {
        /* 0xFD: "Initiate power-up key stream".
         * Clear the keymap as all pressed keys are being re-sent. */
        memset(keymap, 0, sizeof(keymap));
    }

    /* Acknowledge the byte (some games and demos have no keyboard handler). */
    handshake();
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
    return (modifiers_held() || keyboard_held) ? !!state : FALSE;
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
    tim3->ccer = TIM_CCER_CC1E | TIM_CCER_CC1P; /* Falling edge */
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
