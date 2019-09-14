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

static time_t timestamp;

/* Keymap */
uint8_t amiga_keymap[0x68];

static void IRQ_amikbd_clk(void)
{
    time_t t = time_now();
    int bit = gpio_read_pin(gpio_amikbd, pin_amikbd_dat);
    static uint8_t keycode, bitpos;

    exti->pr = m(pin_amikbd_clk);

    /* Sync to keycode start by observing delay in comms. */
    if (time_diff(timestamp, t) > time_us(500))
        bitpos = 0;
    timestamp = t;

    bitpos = (bitpos + 1) & 7;
    if (bitpos == 0) {
        /* Received full byte. Decode and update the keymap. */
        keycode = ~keycode & 0x7f;
        if (keycode < sizeof(amiga_keymap))
            amiga_keymap[keycode] = bit;
    }

    keycode = (keycode << 1) | bit;
}

void amiga_init(void)
{
    /* PB3, PB4: Amiga Keyboard */
    gpio_configure_pin(gpiob, 3, GPI_pull_up);
    gpio_configure_pin(gpiob, 4, GPI_pull_up);

    /* Interrupts on falling edge of PB4 (via EXTI4). */
    afio->exticr2 |= 0x0001; /* PB4 -> EXTI4 */
    exti->ftsr |= m(pin_amikbd_clk);
    exti->imr |= m(pin_amikbd_clk);

    timestamp = time_now();
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
