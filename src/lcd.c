/*
 * lcd.c
 * 
 * 1. HD44780 LCD controller via a PCF8574 I2C backpack.
 * 
 * Written & released by Keir Fraser <keir.xen@gmail.com>
 * 
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

/* PCF8574 pin assignment: D7-D6-D5-D4-BL-EN-RW-RS */
#define _D7 (1u<<7)
#define _D6 (1u<<6)
#define _D5 (1u<<5)
#define _D4 (1u<<4)
#define _BL (1u<<3)
#define _EN (1u<<2)
#define _RW (1u<<1)
#define _RS (1u<<0)

/* STM32 I2C peripheral. */
#define i2c i2c1
#define SCL 6
#define SDA 7

/* I2C error ISR. */
#define I2C_ERROR_IRQ 32
void IRQ_32(void) __attribute__((alias("IRQ_i2c_error")));

/* I2C event ISR. */
#define I2C_EVENT_IRQ 31
void IRQ_31(void) __attribute__((alias("IRQ_i2c_event")));

/* I2C data ring. */
static uint8_t ring[1024];
static uint16_t ring_cons, ring_prod;

static bool_t lcd_inc;
static uint8_t lcd_ddraddr;
struct display lcd_display;

/* I2C Error ISR: As slave with clock stretc we can only receive:
 *  Bus error (BERR): Peripheral automatically recovers
 *  Arbitration lost (ARLO): Peripheral automatically recovers */
static void IRQ_i2c_error(void)
{
    /* Clear I2C errors. Nothing else needs to be done. */
    i2c->sr1 &= ~I2C_SR1_ERRORS;
}

static void IRQ_i2c_event(void)
{
    uint16_t sr1 = i2c->sr1;

    if (sr1 & I2C_SR1_ADDR) {
        /* Read SR2 clears SR1_ADDR. */
        (void)i2c->sr2;
    }

    if (sr1 & I2C_SR1_STOPF) {
        /* Write CR1 clears SR1_STOPF. */
        i2c->cr1 = I2C_CR1_ACK | I2C_CR1_PE;
    }

    if (sr1 & I2C_SR1_RXNE) {
        /* Read DR clear SR1_RXNE. */
        ring[ring_prod] = i2c->dr;
        ring_prod = (ring_prod + 1) & (ARRAY_SIZE(ring) - 1);
    }
}

static void process_cmd(uint8_t cmd)
{
    uint8_t x = 0x80;
    int c = 0;

    if (!cmd)
        return;

    while (!(cmd & x)) {
        x >>= 1;
        c++;
    }

    switch (c) {
    case 0: /* Set DDR Address */
        lcd_ddraddr = cmd & 127;
        break;
    case 1: /* Set CGR Address */
        break;
    case 2: /* Function Set */
        break;
    case 3: /* Cursor or Display Shift */
        break;
    case 4: /* Display On/Off Control */
        break;
    case 5: /* Entry Mode Set */
        lcd_inc = !!(cmd & 2);
        break;
    case 6: /* Return Home */
        lcd_ddraddr = 0;
        break;
    case 7: /* Clear Display */
        memset(lcd_display.text, ' ', sizeof(lcd_display.text));
        lcd_ddraddr = 0;
        break;
    }
}

static void process_dat(uint8_t dat)
{
    int x, y;
    if (lcd_ddraddr >= 0x68)
        lcd_ddraddr = 0x00; /* jump to line 2 */
    if ((lcd_ddraddr >= 0x28) && (lcd_ddraddr < 0x40))
        lcd_ddraddr = 0x40; /* jump to line 1 */
    x = lcd_ddraddr & 0x3f;
    y = lcd_ddraddr >> 6;
    if ((lcd_display.rows == 4) && (x >= 20)) {
        x -= 20;
        y += 2;
    }
    lcd_display.text[y][x] = dat;
    lcd_ddraddr++;
    if (x >= lcd_display.cols)
        lcd_display.cols = min_t(unsigned int, x+1, config.max_cols);
}

void lcd_process(void)
{
    const uint16_t buf_mask = ARRAY_SIZE(ring) - 1;
    uint16_t c, p = ring_prod;
    static uint16_t dat = 1;
    static bool_t rs;

    /* Process the command sequence. */
    for (c = ring_cons; c != p; c = (c+1) & buf_mask) {
        uint8_t x = ring[c];
        if ((x & (_EN|_RW)) != _EN)
            continue;
        lcd_display.on = !!(x & _BL);
        if (rs != !!(x & _RS)) {
            rs ^= 1;
            dat = 1;
        }
        dat <<= 4;
        dat |= x >> 4;
        if (dat & 0x100) {
            if (rs)
                process_dat(dat);
            else
                process_cmd(dat);
            dat = 1;
        }
    }

    ring_cons = c;
}

void lcd_init(void)
{
    rcc->apb1enr |= RCC_APB1ENR_I2C1EN;

    gpio_configure_pin(gpiob, SCL, AFO_opendrain(_2MHz));
    gpio_configure_pin(gpiob, SDA, AFO_opendrain(_2MHz));

    /* Enable the Event IRQ. */
    IRQx_set_prio(I2C_EVENT_IRQ, I2C_IRQ_PRI);
    IRQx_clear_pending(I2C_EVENT_IRQ);
    IRQx_enable(I2C_EVENT_IRQ);

    /* Enable the Error IRQ. */
    IRQx_set_prio(I2C_ERROR_IRQ, I2C_IRQ_PRI);
    IRQx_clear_pending(I2C_ERROR_IRQ);
    IRQx_enable(I2C_ERROR_IRQ);

    /* Initialise I2C. */
    i2c->cr1 = 0;
    i2c->oar1 = 0x3a << 1;
    i2c->cr2 = (I2C_CR2_FREQ(36) |
                I2C_CR2_ITERREN |
                I2C_CR2_ITEVTEN |
                I2C_CR2_ITBUFEN);
    i2c->cr1 = I2C_CR1_ACK | I2C_CR1_PE;
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
