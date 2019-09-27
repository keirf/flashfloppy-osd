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

extern struct config {

    uint16_t polarity;

    uint16_t h_off, v_off;

    uint16_t min_cols, max_cols;

    uint16_t rows;

#define DISPCTL_tristate    0 /* PB15 is tristate outside OSD; PA15 unused */
#define DISPCTL_enable_high 1 /* PA15 is Display Enable: Active HIGH */
#define DISPCTL_enable_low  2 /* PA15 is Display Enable: Active LOW */
    uint16_t dispctl_mode;

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
