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

#define F(x) (x-1)
#define U(x) (1u<<x)

const static struct config *flash_config = (struct config *)0x0800fc00;

#include "default_config.c"

struct config config;

extern void setup_spi(uint16_t video_mode);
extern uint16_t running_polarity;
extern uint16_t running_display_timing;

const static char *dispen_pretty[] = {
    "None", "PA15 Act.HIGH", "PA15 Act.LOW" };

const static char *timing_pretty[] = { "15kHz", "VGA", "Auto" };

const static char *polarity_pretty[] = { "Low", "High", "Auto" };

static void config_printk(const struct config *conf)
{
    printk("\nCurrent config:\n");
    printk(" Sync Polarity: %s\n", polarity_pretty[conf->polarity]);
    printk(" Pixel Timing: %s\n", timing_pretty[config.display_timing]);
    printk(" Display Height: %s\n", conf->display_2Y ? "Double" : "Normal");
    printk(" Display Output: %s\n",
           config.display_spi ? "PA7/SPI1" : "PB15/SPI2");
    printk(" Display Enable: %s\n", dispen_pretty[config.dispctl_mode] );
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
    if (i2c_osd_protocol)
        return;
    i2c_display.rows = config.rows;
    i2c_display.cols = config.min_cols;
}

void config_init(void)
{
    uint16_t crc;

    printk("\n** FF OSD v%s **\n", fw_ver);
    printk("** Keir Fraser <keir.xen@gmail.com>\n");
    printk("** https://github.com/keirf/FF_OSD\n");

    config = *flash_config;
    crc = crc16_ccitt(&config, sizeof(config), 0xffff);
    if (crc) {
        printk("\nConfig corrupt: Resetting to Factory Defaults\n");
        config = dfl_config;
    } else if (gpio_pins_connected(gpioa, 1, gpioa, 2)) {
        printk("\nA1-A2 Jumpered: Resetting to Factory Defaults\n");
        config = dfl_config;
        config_write_flash(&config);
    }

    /* Hotkey configuration is stored in flash-config space but not actually
     * runtime modifiable or viewable. So, to avoid confusion, always use the 
     * compile-time hotkey configuration. */
    memcpy(config.hotkey, dfl_config.hotkey, sizeof(config.hotkey));

    config_printk(&config);

    printk("\nKeys:\n Space: Select\n O: Down\n P: Up\n");

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
    /* Output */
    C_polarity,
    C_disptiming,
    C_disp2Y,
    C_spibus,
    C_dispen,
    C_h_off,
    C_v_off,
    /* LCD */
    C_rows,
    C_min_cols,
    C_max_cols,
    /* Exit */
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

void config_process(uint8_t b, bool_t autosync_changed)
{
    uint8_t _b;
    static uint8_t pb;
    bool_t changed = FALSE;
    static enum { C_SAVE = 0, C_SAVEREBOOT, C_USE, C_DISCARD,
                  C_RESET, C_NC_MAX } new_config;
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
        case 'o': b |= B_LEFT; break;
        }
    }

    if (b & B_SELECT) {
        if (++config_state >= C_max) {
            config_state = C_idle;
            display_off();
            switch (new_config) {
            case C_SAVE:
                config_write_flash(&config);
                break;
            case C_SAVEREBOOT:
                config_write_flash(&config);
                while(1) {} /* hang and let WDT reboot */
                break;
            case C_USE:
                break;
            case C_DISCARD:
                config = old_config;
                break;
            case C_RESET:
                config = dfl_config;
                config_write_flash(&config);
                while(1) {} /* hang and let WDT reboot */
                break;
            case C_NC_MAX:
                break;
            }
            printk("\n");
            config_printk(&config);
            lcd_display_update();
        }
        if ((config_state == C_rows) && i2c_osd_protocol) {
            /* Skip LCD config options if using the extended OSD protocol. */
            config_state = C_save;
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
            cnf_prt(0, "Sync Polarity:");
        if (b & B_LEFT) {
            if (config.polarity > 0)
                --config.polarity;
            else
                config.polarity = SYNC_MAX-1;
        }
        if (b & B_RIGHT) {
            if (++config.polarity >= SYNC_MAX)
                config.polarity = 0;
        }
        if (b & (B_LEFT|B_RIGHT)) {
            if (config.polarity != SYNC_AUTO)
                running_polarity = config.polarity;
        }
        if (b || autosync_changed) {
            if (config.polarity == SYNC_AUTO)
                cnf_prt(1, "%s (%s)", polarity_pretty[config.polarity],
                        polarity_pretty[running_polarity]);
            else
                cnf_prt(1, "%s", polarity_pretty[config.polarity]);
        }
        break;
    case C_disptiming:
        if (changed)
            cnf_prt(0, "Pixel Timing:");
        if (b & B_LEFT) {
            if (config.display_timing > 0)
                --config.display_timing;
            else
                config.display_timing = DISP_MAX-1;
        }
        if (b & B_RIGHT) {
            if (++config.display_timing >= DISP_MAX)
                config.display_timing = 0;
        }
        if (b & (B_LEFT|B_RIGHT)) {
            if (config.display_timing != DISP_AUTO)
                setup_spi(config.display_timing);
        }
        if (b || autosync_changed) {
            if (config.display_timing == DISP_AUTO)
                cnf_prt(1, "%s (%s)", timing_pretty[config.display_timing],
                        timing_pretty[running_display_timing]);
            else
                cnf_prt(1, "%s", timing_pretty[config.display_timing]);
        }
        break;
    case C_disp2Y:
        if (changed)
            cnf_prt(0, "Display Height:");
        if (b & (B_LEFT|B_RIGHT)) {
            config.display_2Y ^= 1;
        }
        if (b)
            cnf_prt(1, "%s", config.display_2Y ? "Double" : "Normal");
        break;
    case C_spibus:
        if (changed)
            cnf_prt(0, "Display Output:");
        if (b & (B_LEFT|B_RIGHT)) {
            config.display_spi ^= 1;
        }
        if (b)
            cnf_prt(1, "%s", config.display_spi ? "PA7/SPI1" : "PB15/SPI2");
        break;
    case C_dispen:
        if (changed)
            cnf_prt(0, "Display Enable:");
        if (b & B_LEFT) {
            if (config.dispctl_mode > 0)
                --config.dispctl_mode;
            else
                config.dispctl_mode = DISPCTL_MAX-1;
        }
        if (b & B_RIGHT)
            if (++config.dispctl_mode >= DISPCTL_MAX)
                config.dispctl_mode = 0;
        if (b)
            cnf_prt(1, "%s", dispen_pretty[config.dispctl_mode] );
        break;
    case C_h_off:
        if (changed)
            cnf_prt(0, "H.Off (1-199):");
        if (b & B_LEFT)
            config.h_off = max_t(uint16_t, config.h_off-1, 1);
        if (b & B_RIGHT)
            config.h_off = min_t(uint16_t, config.h_off+1, 199);
        if (b)
            cnf_prt(1, "%u", config.h_off);
        break;
    case C_v_off:
        if (changed)
            cnf_prt(0, "V.Off (2-299):");
        if (b & B_LEFT)
            config.v_off = max_t(uint16_t, config.v_off-1, 2);
        if (b & B_RIGHT)
            config.v_off = min_t(uint16_t, config.v_off+1, 299);
        if (b)
            cnf_prt(1, "%u", config.v_off);
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
        const static char *str[] = { "Save", "Save+Reset", "Use",
                                     "Discard", "Factory Reset" };
        if (changed) {
            cnf_prt(0, "Save New Config?");
            if ((old_config.display_spi == config.display_spi)
             && (old_config.dispctl_mode == config.dispctl_mode) )
                new_config = C_SAVE;
            else
                new_config = C_SAVEREBOOT;
        }
        if (b & B_LEFT) {
            if (new_config > 0)
                --new_config;
            else
                new_config = C_NC_MAX-1;
        }
        if (b & B_RIGHT)
            if (++new_config >= C_NC_MAX)
                new_config = 0;
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

