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
#define irq_amikbd_clk 10
void IRQ_10(void) __attribute__((alias("IRQ_amikbd_clk"))); /* EXTI4 */

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

    exti->pr = m(pin_amikbd_clk);

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
//        printk("[%02x,%u]", keycode, bit);
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

    /* Interrupts on falling edge of PB4 (via EXTI4). */
    afio->exticr2 |= 0x0001; /* PB4 -> EXTI4 */
    exti->rtsr |= m(pin_amikbd_clk); /* Data clocked in on the rising edge */
    exti->imr |= m(pin_amikbd_clk);

    IRQx_set_prio(irq_amikbd_clk, AMIKBD_IRQ_PRI);
    IRQx_enable(irq_amikbd_clk);
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
