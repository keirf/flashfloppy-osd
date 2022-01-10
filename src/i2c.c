/*
 * i2c.c
 * 
 * I2C communications to the host:
 *  1. Emulate HD44780 LCD controller via a PCF8574 I2C backpack.
 *  2. Support extended custom protocol with bidirectional comms.
 * 
 * I2C communications to a "slave" OSD that duplicates the output.
 *
 * Written & released by Keir Fraser <keir.xen@gmail.com>
 * I2C "slave" OSD driving contributed by Torsten Kurbad <amiga@tk-webart.de>
 * 
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

#define _BL (1u<<3)
#define _EN (1u<<2)
#define _RW (1u<<1)
#define _RS (1u<<0)

/* FF OSD I2C address. */
#define OSD_I2C_ADDR     0x10

/* Current position in FF OSD I2C Protocol character data. */
static uint8_t ff_osd_x, ff_osd_y;

/* STM32 I2C peripherals. */
/* Slave: i2c1 */
#define i2c i2c1
#define SCL 6
#define SDA 7
/* Master: i2c2 */
#define i2cm i2c2
#define SCL2 10
#define SDA2 11

/* I2C error ISR. */
/* Slave. */
#define I2C_ERROR_IRQ 32
void IRQ_32(void) __attribute__((alias("IRQ_i2c_error")));
/* Master. */
#define I2CM_ERROR_IRQ 34
void IRQ_34(void) __attribute__((alias("IRQ_i2cm_error")));

/* I2C event ISR. */
/* Slave. */
#define I2C_EVENT_IRQ 31
void IRQ_31(void) __attribute__((alias("IRQ_i2c_event")));
/* Master. */
#define I2CM_EVENT_IRQ 33
void IRQ_33(void) __attribute__((alias("IRQ_i2cm_event")));

/* I2C data ring. */
static uint8_t d_ring[1024];
static uint16_t d_cons, d_prod;
#define MASK(r,x) ((x) & (ARRAY_SIZE(r)-1))

/* Transaction ring: Data-ring offset of each transaction start. */
static uint16_t t_ring[8];
static uint16_t t_cons, t_prod;

/* Display state, exported to display routines. */
struct display i2c_display;

/* LCD state. */
static bool_t lcd_inc;
static uint8_t lcd_ddraddr;

/* I2C custom protocol state. */
bool_t i2c_osd_protocol; /* using the custom protocol? */
uint8_t i2c_buttons_rx; /* button state: Gotek -> OSD */
struct i2c_osd_info i2c_osd_info; /* state: OSD -> Gotek */

/* I2C Slave Error ISR: As slave with clock stretch we can only receive:
 *  Bus error (BERR): Peripheral automatically recovers
 *  Acknowledge Failure (AF): Peripheral automatically recovers */
static void IRQ_i2c_error(void)
{
    /* Clear I2C errors. Nothing else needs to be done. */
    i2c->sr1 &= ~I2C_SR1_ERRORS;
}

/* I2C Slave Event ISR. */
static void IRQ_i2c_event(void)
{
    static uint8_t rp;
    uint16_t sr1 = i2c->sr1;

    if (sr1 & I2C_SR1_ADDR) {
        /* Read SR2 clears SR1_ADDR. */
        uint16_t sr2 = i2c->sr2;
        if (!(sr2 & I2C_SR2_TRA))
            t_ring[MASK(t_ring, t_prod++)] = d_prod;
        rp = 0;
    }

    if (sr1 & I2C_SR1_STOPF) {
        /* Write CR1 clears SR1_STOPF. */
        i2c->cr1 = I2C_CR1_ACK | I2C_CR1_PE;
    }

    if (sr1 & I2C_SR1_RXNE) {
        /* Read DR clears SR1_RXNE. */
        d_ring[MASK(d_ring, d_prod++)] = i2c->dr;
    }

    if (sr1 & I2C_SR1_TXE) {
        /* Write DR clears SR1_TXE. */
        uint8_t *info = (uint8_t *)&i2c_osd_info;
        i2c->dr = (rp < sizeof(i2c_osd_info)) ? info[rp++] : 0;
    }
}

/* I2C Master data buffer. Data is sent via interrupt to the Slave OSD. */
static uint8_t buffer[256];
volatile static uint8_t buffer_len, b;

/* I2C Master Error ISR: Reset the peripheral and reinit everything. */
static void IRQ_i2cm_error(void)
{
    /* Dump and clear I2C errors. */
    printk("I2C2: Error (%04x)\n", (uint16_t)(i2cm->sr1 & I2C_SR1_ERRORS));
    i2cm->sr1 &= ~I2C_SR1_ERRORS;

    /* Clear the I2C peripheral. */
    i2cm->cr1 = 0;
    i2cm->cr1 = I2C_CR1_SWRST;

    slave_init();
}

/* I2C Master Event ISR. */
static void IRQ_i2cm_event(void)
{
    uint16_t sr1 = i2cm->sr1;

    if (sr1 & I2C_SR1_SB) {
        /* Send address. Clears SR1_SB. */
        i2cm->dr = OSD_I2C_ADDR << 1;
        b = 0;
    }

    if (sr1 & I2C_SR1_ADDR) {
        /* Read SR2 clears SR1_ADDR. */
        (void)i2cm->sr2;
    }

    if (sr1 & I2C_SR1_TXE) {
        /* Send buffer byte by byte. */
        if (b < buffer_len) {
            uint8_t *buf = buffer;
            i2cm->dr = buf[b++];
        }
    }

    if (sr1 & I2C_SR1_RXNE) {
        /* Read DR clears SR1_RXNE. */
        (void)i2cm->dr;
    }

    if (sr1 & I2C_SR1_BTF) {
        if (b == buffer_len) {
            /* Transfer ended. Send stop sequence and disable IRQ*/
            i2cm->cr1 |= I2C_CR1_STOP;
            while (i2cm->cr1 & I2C_CR1_STOP)
                continue;
            i2cm->cr2 &= ~I2C_CR2_ITEVTEN;
        }
    }
}

/* FF OSD command set */
#define OSD_BACKLIGHT    0x00 /* [0] = backlight on */
#define OSD_DATA         0x02 /* next columns*rows bytes are text data */
#define OSD_ROWS         0x10 /* [3:0] = #rows */
#define OSD_HEIGHTS      0x20 /* [3:0] = 1 iff row is 2x height */
#define OSD_BUTTONS      0x30 /* [3:0] = button mask */
#define OSD_COLUMNS      0x40 /* [6:0] = #columns */

/* I2C Master. */
static bool_t has_slave;
static uint8_t slave_ver;
static bool_t i2cm_dead;

/* Display data sent to slave display. */
struct display *slave_display;

/* Wait for given status condition @s while also checking for errors. */
static bool_t i2cm_wait(uint8_t s)
{
    stk_time_t t = stk_now();
    while ((i2cm->sr1 & s) != s) {
        if (i2cm->sr1 & I2C_SR1_ERRORS) {
            i2cm->sr1 &= ~I2C_SR1_ERRORS;
            return FALSE;
        }
        if (stk_diff(t, stk_now()) > stk_ms(10)) {
            /* I2C bus seems to be locked up. */
            i2cm_dead = TRUE;
            return FALSE;
        }
    }
    return TRUE;
}

/* Synchronously transmit the I2C START sequence.
 * Caller must already have asserted I2C_CR1_START. */
#define I2C_RD TRUE
#define I2C_WR FALSE
static bool_t i2cm_start(bool_t rd)
{
    if (!i2cm_wait(I2C_SR1_SB))
        return FALSE;
    i2cm->dr = (OSD_I2C_ADDR << 1) | rd;
    if (!i2cm_wait(I2C_SR1_ADDR))
        return FALSE;
    (void)i2cm->sr2;
    return TRUE;
}

/* Synchronously transmit the I2C STOP sequence. */
static void i2cm_stop(void)
{
    i2cm->cr1 |= I2C_CR1_STOP;
    while (i2cm->cr1 & I2C_CR1_STOP)
        continue;
}

/* Synchronously transmit an I2C byte. */
static bool_t i2cm_sync_write(uint8_t b)
{
    i2cm->dr = b;
    return i2cm_wait(I2C_SR1_BTF);
}

/* Check whether an I2C device is responding at OSD_I2C_ADDR. */
static bool_t i2cm_probe(void)
{
    i2cm->cr1 |= I2C_CR1_START;
    if (!i2cm_start(I2C_WR) || !i2cm_sync_write(0))
        return FALSE;
    i2cm_stop();
    return TRUE;
}

/* Snapshot display buffer into the command and start sending
 * asynchronously. */
void slave_send_display(void)
{
    if (has_slave) {
        uint8_t *q = buffer;
        unsigned int row;

        *q++ = OSD_BACKLIGHT | slave_display->on;
        *q++ = OSD_COLUMNS | slave_display->cols;
        *q++ = OSD_ROWS | slave_display->rows;
        *q++ = OSD_HEIGHTS | slave_display->heights;
        *q++ = OSD_BUTTONS | 0;
        *q++ = OSD_DATA;
        for (row = 0; row < slave_display->rows; row++) {
            memcpy(q, slave_display->text[row], slave_display->cols);
            q += slave_display->cols;
        }

        buffer_len = q - buffer;

        i2cm->cr2 |= I2C_CR2_ITEVTEN;
        i2cm->cr1 |= I2C_CR1_START;
    }
}

bool_t slave_init(void)
{
    i2cm_dead = FALSE;

    /* Check we have a clear I2C bus. Both clock and data must be high. If SDA
     * is stuck low then slave may be stuck in an ACK cycle. We can try to
     * unwedge the slave in that case and drive it into the STOP condition. */
    gpio_configure_pin(gpiob, SCL2, GPO_opendrain(_2MHz, HIGH));
    gpio_configure_pin(gpiob, SDA2, GPO_opendrain(_2MHz, HIGH));
    delay_us(10);
    if (gpio_read_pin(gpiob, SCL2) && !gpio_read_pin(gpiob, SDA2)) {
        printk("I2C Master: SDA2 held by slave? Fixing... ");
        /* We will hold SDA2 low (as slave is) and also drive SCL2 low to end
         * the current ACK cycle. */
        gpio_write_pin(gpiob, SDA2, FALSE);
        gpio_write_pin(gpiob, SCL2, FALSE);
        delay_us(10);
        /* Slave should no longer be driving SDA2 low (but we still are).
         * Now prepare for the STOP condition by setting SCL2 high. */
        gpio_write_pin(gpiob, SCL2, TRUE);
        delay_us(10);
        /* Enter the STOP condition by setting SDA high while SCL is high. */
        gpio_write_pin(gpiob, SDA2, TRUE);
        delay_us(10);
        printk("%s\n",
               !gpio_read_pin(gpiob, SCL2) || !gpio_read_pin(gpiob, SDA2)
               ? "Still held" : "Done");
    }

    /* Check the bus is not floating (or still stuck!). We shouldn't be able to
     * pull the lines low with our internal weak pull-downs (min. 30kohm). */
    if (!has_slave) {
        bool_t scl2, sda2;
        gpio_configure_pin(gpiob, SCL2, GPI_pull_down);
        gpio_configure_pin(gpiob, SDA2, GPI_pull_down);
        delay_us(10);
        scl2 = gpio_read_pin(gpiob, SCL2);
        sda2 = gpio_read_pin(gpiob, SDA2);
        if (!scl2 || !sda2) {
            gpio_configure_pin(gpiob, SCL2, AFO_opendrain(_2MHz));
            gpio_configure_pin(gpiob, SDA2, AFO_opendrain(_2MHz));
            printk("I2C Master: Invalid bus SCL2=%u SDA2=%u\n", scl2, sda2);
            return FALSE;
        }
    }

    /* Reset I2C2 peripheral. */
    rcc->apb1enr &= ~RCC_APB1ENR_I2C2EN;
    rcc->apb1enr |= RCC_APB1ENR_I2C2EN;

    /* Standard Mode (100kHz) */
    i2cm->cr1 = 0;
    i2cm->cr2 = I2C_CR2_FREQ(2);
    i2cm->ccr = I2C_CCR_CCR(180);
    i2cm->trise = 37;
    i2cm->cr1 = I2C_CR1_PE;

    gpio_configure_pin(gpiob, SCL2, AFO_opendrain(_2MHz));
    gpio_configure_pin(gpiob, SDA2, AFO_opendrain(_2MHz));

    if (!has_slave) {
        /* Probe the bus for I2C devices: We support a single FF OSD device. */
        has_slave = i2cm_probe();
        if (!has_slave) {
            printk("I2C Master: %s\n",
                   i2cm_dead ? "Bus locked up?" : "No device found");
            return FALSE;
        }

        /* Probe the FF OSD device if we found one. */
        if (has_slave) {
            /* Read: Retrieve the version number. */
            i2cm->cr1 |= I2C_CR1_START;
            if (i2cm_start(I2C_RD) && i2cm_wait(I2C_SR1_RXNE))
                slave_ver = i2cm->dr;
            printk("I2C Master: Slave OSD found (ver %x)\n", slave_ver);
            i2cm_stop();
        }
    }

    /* Enable the Event IRQ. */
    IRQx_set_prio(I2CM_EVENT_IRQ, I2CM_IRQ_PRI);
    IRQx_clear_pending(I2CM_EVENT_IRQ);
    IRQx_enable(I2CM_EVENT_IRQ);

    /* Enable the Error IRQ. */
    IRQx_set_prio(I2CM_ERROR_IRQ, I2CM_IRQ_PRI);
    IRQx_clear_pending(I2CM_ERROR_IRQ);
    IRQx_enable(I2CM_ERROR_IRQ);

    return TRUE;
}

/* I2C Slave. */
static void ff_osd_process(void)
{
    uint16_t d_c, d_p, t_c, t_p;

    d_c = d_cons;
    d_p = d_prod;
    barrier(); /* Get data ring producer /then/ transaction ring producer */
    t_c = t_cons;
    t_p = t_prod;

    /* We only care about the last full transaction, and newer. */
    if ((uint16_t)(t_p - t_c) >= 2) {
        /* Discard older transactions, and in-progress old transaction. */
        t_c = t_p - 2;
        d_c = t_ring[MASK(t_ring, t_c)];
        ff_osd_y = 0;
    }

    /* Data ring should not be more than half full. We don't want it to 
     * overrun during the processing loop below: That should be impossible
     * with half a ring free. */
    ASSERT((uint16_t)(d_p - d_c) < (ARRAY_SIZE(d_ring)/2));

    /* Process the command sequence. */
    for (; d_c != d_p; d_c++) {
        uint8_t x = d_ring[MASK(d_ring, d_c)];
        if ((t_c != t_p) && (d_c == t_ring[MASK(t_ring, t_c)])) {
            t_c++;
            ff_osd_y = 0;
        }
        if (ff_osd_y != 0) {
            /* Character Data. */
            i2c_display.text[ff_osd_y-1][ff_osd_x] = x;
            if (++ff_osd_x >= i2c_display.cols) {
                ff_osd_x = 0;
                if (++ff_osd_y > i2c_display.rows)
                    ff_osd_y = 0;
            }
        } else {
            /* Command. */
            if ((x & 0xc0) == OSD_COLUMNS) {
                /* 0-40 */
                i2c_display.cols = min_t(uint16_t, 40, x & 0x3f);
            } else {
                switch (x & 0xf0) {
                case OSD_BUTTONS:
                    i2c_buttons_rx = x & 0x0f;
                    break;
                case OSD_ROWS:
                    /* 0-3 */
                    i2c_display.rows = x & 0x03;
                    break;
                case OSD_HEIGHTS:
                    i2c_display.heights = x & 0x0f;
                    break;
                case OSD_BACKLIGHT:
                    switch (x & 0x0f) {
                    case 0:
                        i2c_display.on = FALSE;
                        break;
                    case 1:
                        i2c_display.on = TRUE;
                        break;
                    case 2:
                        ff_osd_x = 0;
                        ff_osd_y = 1;
                        break;
                    }
                }
            }
        }
    }

    d_cons = d_c;
    t_cons = t_c;
}

static void lcd_process_cmd(uint8_t cmd)
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
        memset(i2c_display.text, ' ', sizeof(i2c_display.text));
        lcd_ddraddr = 0;
        break;
    }
}

static void lcd_process_dat(uint8_t dat)
{
    int x, y;
    if (lcd_ddraddr >= 0x68)
        lcd_ddraddr = 0x00; /* jump to line 2 */
    if ((lcd_ddraddr >= 0x28) && (lcd_ddraddr < 0x40))
        lcd_ddraddr = 0x40; /* jump to line 1 */
    x = lcd_ddraddr & 0x3f;
    y = lcd_ddraddr >> 6;
    if ((i2c_display.rows == 4) && (x >= 20)) {
        x -= 20;
        y += 2;
    }
    i2c_display.text[y][x] = dat;
    lcd_ddraddr++;
    if (x >= i2c_display.cols)
        i2c_display.cols = min_t(unsigned int, x+1, config.max_cols);
}

static void lcd_process(void)
{
    uint16_t d_c, d_p = d_prod;
    static uint16_t dat = 1;
    static bool_t rs;

    /* Process the command sequence. */
    for (d_c = d_cons; d_c != d_p; d_c++) {
        uint8_t x = d_ring[MASK(d_ring, d_c)];
        if ((x & (_EN|_RW)) != _EN)
            continue;
        i2c_display.on = !!(x & _BL);
        if (rs != !!(x & _RS)) {
            rs ^= 1;
            dat = 1;
        }
        dat <<= 4;
        dat |= x >> 4;
        if (dat & 0x100) {
            if (rs)
                lcd_process_dat(dat);
            else
                lcd_process_cmd(dat);
            dat = 1;
        }
    }

    d_cons = d_c;
}

void i2c_process(void)
{
    return i2c_osd_protocol ? ff_osd_process() : lcd_process();
}

void i2c_init(void)
{
    char *p;

    i2c_osd_protocol = gpio_pins_connected(gpioa, 0, gpioa, 1);

    i2c_osd_info.protocol_ver = 0;
    i2c_osd_info.fw_major = strtol(fw_ver, &p, 10);
    i2c_osd_info.fw_minor = strtol(p+1, NULL, 10);

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
    i2c->oar1 = (i2c_osd_protocol ? OSD_I2C_ADDR : 0x27) << 1;
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
