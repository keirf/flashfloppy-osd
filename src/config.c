/*
 * config.c
 * 
 * Read/write/modify configuration parameters.
 * 
 * Written & released by Keir Fraser <keir.xen@gmail.com>
 * 
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

const static struct config *flash_config = (struct config *)0x08007c00;
const static struct config dfl_config = {
    .polarity = FALSE,
    .h_off = 42,
    .v_off = 50,
    .min_cols = 16,
    .max_cols = 40,
    .rows = 2,
};
struct config config;

static void config_printk(const struct config *conf)
{
    printk("\nCurrent config:\n");
    printk(" Sync: Active %s\n", conf->polarity ? "HIGH" : "LOW");
    printk(" H.Off: %u\n", conf->h_off);
    printk(" V.Off: %u\n", conf->v_off);
    printk(" Rows: %u\n", conf->rows);
    printk(" Columns: %u-%u\n", conf->min_cols, conf->max_cols);
}

static void config_write_flash(struct config *conf)
{
    conf->crc16_ccitt = htobe16(
        crc16_ccitt(conf, sizeof(*conf)-2, 0xffff));
    fpec_init();
    fpec_page_erase((uint32_t)flash_config);
    fpec_write(conf, sizeof(*conf), (uint32_t)flash_config);
}

static void lcd_display_update(void)
{
    lcd_display.rows = config.rows;
    lcd_display.cols = config.min_cols;
}

void config_init(void)
{
    uint16_t crc;

    printk("\n** FF OSD v%s **\n", fw_ver);
    printk("** Keir Fraser <keir.xen@gmail.com>\n");
    printk("** https://github.com/keirf/FF_OSD\n");

    config = *flash_config;
    crc = crc16_ccitt(&config, sizeof(config), 0xffff);
    if (crc)
        config = dfl_config;

    config_printk(&config);

    printk("\nKeys:\n Space: Select\n P: Up\n L: Down\n");

    lcd_display_update();
    (void)usart1->dr;
}

bool_t config_active;
struct display config_display = {
    .cols = 16, .rows = 2, .on = TRUE,
};

static enum {
    C_idle = 0,
    C_banner,
    C_polarity,
    C_h_off,
    C_v_off,
    C_rows,
    C_min_cols,
    C_max_cols,
    C_save,
    C_max
} config_state;

static void cnf_prt(int row, const char *format, ...)
{
    va_list ap;
    char *r = (char *)config_display.text[row];

    memset(r, 0, 20);

    va_start(ap, format);
    (void)vsnprintf(r, 20, format, ap);
    va_end(ap);

    printk((row == 0) ? "\n%s%14s" : "\b\b\b\b\b\b\b\b\b\b\b\b\b%13s", r, "");
}

static struct repeat {
    int repeat;
    time_t prev;
} left, right;

uint8_t button_repeat(uint8_t pb, uint8_t b, uint8_t m, struct repeat *r)
{
    if (pb & m) {
        /* Is this button held down? */
        if (b & m) {
            time_t delta = time_ms(r->repeat ? 100 : 500);
            if (time_diff(r->prev, time_now()) > delta) {
                /* Repeat this button now. */
                r->repeat++;
            } else {
                /* Not ready to repeat this button. */
                b &= ~m;
            }
        } else {
            /* Button not pressed. Reset repeat count. */
            r->repeat = 0;
        }
    }
    if (b & m) {
        /* Remember when we actioned this button press/repeat. */
        r->prev = time_now();
    }
    return b;
}

void config_process(uint8_t b)
{
    uint8_t _b;
    static uint8_t pb;
    bool_t changed = FALSE;
    static enum { C_SAVE = 0, C_USE, C_DISCARD, C_RESET } new_config;
    static struct config old_config;

    _b = b;
    b &= b ^ (pb & B_SELECT);
    b = button_repeat(pb, b, B_LEFT, &left);
    b = button_repeat(pb, b, B_RIGHT, &right);
    pb = _b;

    if (usart1->sr & USART_SR_RXNE) {
        char c = usart1->dr;
        switch (tolower(c)) {
        case ' ': b |= B_SELECT; break;
        case 'p': b |= B_RIGHT; break;
        case 'l': b |= B_LEFT; break;
        }
    }

    if (b & B_SELECT) {
        if (++config_state >= C_max) {
            config_state = C_idle;
            switch (new_config) {
            case C_SAVE:
                config_write_flash(&config);
                break;
            case C_USE:
                break;
            case C_DISCARD:
                config = old_config;
                slave_arr_update();
                break;
            case C_RESET:
                config = dfl_config;
                slave_arr_update();
                config_write_flash(&config);
                break;
            }
            printk("\n");
            config_printk(&config);
            lcd_display_update();
        }
        config_active = (config_state != C_idle);
        changed = TRUE;
    }

    switch (config_state) {
    default:
        break;
    case C_banner:
        if (changed) {
            cnf_prt(0, "FF OSD v%s", fw_ver);
            cnf_prt(1, "Flash Config");
            old_config = config;
        }
        break;
    case C_polarity:
        if (changed)
            cnf_prt(0, "Sync:");
        if (b & (B_LEFT|B_RIGHT))
            config.polarity ^= 1;
        if (b)
            cnf_prt(1, "Active %s", config.polarity ? "HIGH" : "LOW");
        break;
    case C_h_off:
        if (changed)
            cnf_prt(0, "H.Off (1-199):");
        if (b & B_LEFT)
            config.h_off = max_t(uint16_t, config.h_off-1, 1);
        if (b & B_RIGHT)
            config.h_off = min_t(uint16_t, config.h_off+1, 199);
        if (b) {
            cnf_prt(1, "%u", config.h_off);
            slave_arr_update();
        }
        break;
    case C_v_off:
        if (changed)
            cnf_prt(0, "V.Off (2-299):");
        if (b & B_LEFT)
            config.v_off = max_t(uint16_t, config.v_off-1, 2);
        if (b & B_RIGHT)
            config.v_off = min_t(uint16_t, config.v_off+1, 299);
        if (b) {
            cnf_prt(1, "%u", config.v_off);
            slave_arr_update();
        }
        break;
    case C_rows:
        if (changed)
            cnf_prt(0, "Rows (2 or 4):");
        if (b & (B_LEFT|B_RIGHT))
            config.rows = (config.rows == 2) ? 4 : 2;
        if (b)
            cnf_prt(1, "%u", config.rows);
        break;
    case C_min_cols:
        if (changed)
            cnf_prt(0, "Min.Col (1-%u):", 80 / config.rows);
        if (b & B_LEFT)
            config.min_cols--;
        if (b & B_RIGHT)
            config.min_cols++;
        config.min_cols = min_t(uint16_t, max_t(uint16_t, config.min_cols, 1),
                                80 / config.rows);
        if (b)
            cnf_prt(1, "%u", config.min_cols);
        break;
    case C_max_cols:
        if (changed)
            cnf_prt(0, "Max.Col (%u-%u):", config.min_cols, 80 / config.rows);
        if (b & B_LEFT)
            config.max_cols--;
        if (b & B_RIGHT)
            config.max_cols++;
        config.max_cols = min_t(uint16_t, max_t(uint16_t, config.max_cols,
                                                config.min_cols),
                                80 / config.rows);
        if (b)
            cnf_prt(1, "%u", config.max_cols);
        break;
    case C_save: {
        const static char *str[] = { "Save", "Use",
                                     "Discard", "Factory Reset" };
        if (changed) {
            cnf_prt(0, "Save New Config?");
            new_config = C_SAVE;
        }
        if (b & B_LEFT)
            new_config = (new_config - 1) & 3;
        if (b & B_RIGHT)
            new_config = (new_config + 1) & 3;
        if (b)
            cnf_prt(1, "%s", str[new_config]);
        break;
    }
    }
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

