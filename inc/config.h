/*
 * config.h
 * 
 * User configuration.
 * 
 * Written & released by Keir Fraser <keir.xen@gmail.com>
 * 
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

extern struct __packed config {

    uint16_t polarity;

    uint16_t h_off, v_off;

    uint16_t min_cols, max_cols;

    uint16_t rows;

#define DISPCTL_tristate    0 /* PB15 is tristate outside OSD; PA15 unused */
#define DISPCTL_enable_high 1 /* PA15 is Display Enable: Active HIGH */
#define DISPCTL_enable_low  2 /* PA15 is Display Enable: Active LOW */
    uint16_t dispctl_mode;

    /* Mask of user-assigned pins configured in open-drain mode. */
    uint8_t user_pin_opendrain;
    /* Mask of user-assigned pins configured in push-pull mode. */
    uint8_t user_pin_pushpull;
    /* Mask of user-assigned pins which are HIGH at power on. */
    uint8_t user_pin_high;
    uint8_t _pad;
    struct __packed config_hotkey {
        /* Mask of user pins modified by this hotkey. */
        uint8_t pin_mod;
        /* Mask of user pins driven HIGH by this hotkey. 
         * Pins in @pin_mod but not in @pin_high are driven LOW. */
        uint8_t pin_high;
        char str[30];
    } hotkey[10];

    uint16_t crc16_ccitt;

} config;

extern bool_t config_active;
extern struct display config_display;

void config_init(void);
void config_process(uint8_t b);

/*
 * Local variables:
 * mode: C
 * c-file-style: "Linux"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
