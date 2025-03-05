/*
 * userpin.c
 * 
 * User pin functions for FlashFloppy OSD.
 * 
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

/* Set initial videoswitch state. */
static bool_t has_videoswitch = FALSE;
uint8_t videoswitch_state = VS_AUTO;
uint32_t videoswitch_pin_mask = 0;

/* Initialize user output pins. */
void user_pin_init(void)
{
    int i;
    uint8_t pin_ux;
    bool_t level;

    /* Video input switching enabled for any of the hotkeys? */
    for (i = 0; i < ARRAY_SIZE(config.hotkey); i++) {
        struct config_hotkey *hk = &config.hotkey[i];
        if (hk->flags & HKF_videoswitch)
            has_videoswitch = TRUE;
    }

    /* Set up user output pins. */
    for (i = 0; i < 3; i++) {
        pin_ux = (i<2 ? pin_u0+i : pin_u2);

        if (has_videoswitch && (i == 2)) {
            gpio_configure_pin(gpio_amicts, pin_amicts, GPI_pull_up);
            gpio_configure_pin(gpio_user, pin_ux,
                               GPO_opendrain(_2MHz,
                                   gpio_read_pin(gpio_amicts, pin_amicts)));
            continue;
        }

        level = (config.user_pin_high >> i) & 1;
        if (config.user_pin_opendrain & (1u<<i))
            gpio_configure_pin(gpio_user, pin_ux,
                               GPO_opendrain(_2MHz, level));
        if (config.user_pin_pushpull & (1u<<i))
            gpio_configure_pin(gpio_user, pin_ux,
                               GPO_pushpull(_2MHz, level));
    }
}

/* State machine for video switching. Switch output is active low. */
void videoswitch_next(void)
{
    if (has_videoswitch) {
        switch (videoswitch_state) {
            case VS_AUTO:
                /* Switch in "Auto" position. Advance to state 1. */
                gpio_write_pin(gpio_user, pin_u2, 1);
                videoswitch_state = VS_AMIGA;
                break;
            case VS_AMIGA:
                /* Switch in "Amiga" position. Advance to state 2. */
                gpio_write_pin(gpio_user, pin_u2, 0);
                videoswitch_state = VS_RTG;
                break;
            default:
                /* Switch in "RTG" position. Reset to state 0. */
                videoswitch_state = VS_AUTO;
                break;
        }
    }
}

/* Update videoswitch output for AUTO state. */
void videoswitch_update(void)
{
    /* No video switch configured or video switch not in "Auto" state? */
    if (!has_videoswitch || videoswitch_state)
        return;

    gpio_write_pin(gpio_user, pin_u2, gpio_read_pin(gpio_amicts, pin_amicts));
}

/* Pretty print video input switch states. */
const char *vs_state_pretty[] = {
    "Video Source Auto",
    "Video Source Amiga",
    "Video Source RTG",
};

/*
 * Local variables:
 * mode: C
 * c-file-style: "Linux"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
