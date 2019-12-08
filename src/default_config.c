/*
 * default_config.c
 * 
 * Default configuration values.
 * 
 * Written & released by Keir Fraser <keir.xen@gmail.com>
 * 
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

const static struct config dfl_config = {

    .polarity = FALSE,
    .h_off = 42,
    .v_off = 50,
    .min_cols = 16,
    .max_cols = 40,
    .dispctl_mode = DISPCTL_tristate,
    .rows = 2,
    .display_timing = DISP_15KHZ,
    .display_spi = DISP_SPI2,
    .display_2Y = FALSE,

#define F(x) (x-1)   /* Hotkey (F1-F10) array index */
#define U(x) (1u<<x) /* User pin (U0-U2) bitmask */

#if 0
    /* An example configuration for switching ROMs and PAL/NTSC or DF0/DF1:
     *  F1-F4: Switch between ROMs #1-#4 via binary value at pins U1,U0. 
     *  F9:    Switch PAL (or Gotek DF0) via U2 LOW. 
     *  F10:   Switch NTSC (or Gotek DF1) via U2 HIGH. */

    /* U0-U2 are configured open drain and need external pullups. 
     * Or move U0-U2 out of @user_pin_opendrain and into @user_pin_pushpull. */
    .user_pin_opendrain = U(2) | U(1) | U(0),
    .user_pin_pushpull  = 0,

    /* ROM #1, Output2 LOW */
    .user_pin_high      = 0,

    .hotkey = {
        /* F1-F4: ROM switching. */
        [F(1)]  = { .str = "ROM #1",
                    .pin_mod  = U(1) | U(0), },
        [F(2)]  = { .str = "ROM #2",
                    .pin_mod  = U(1) | U(0),
                    .pin_high =        U(0), },
        [F(3)]  = { .str = "ROM #3",
                    .pin_mod  = U(1) | U(0),
                    .pin_high = U(1)       , },
        [F(4)]  = { .str = "ROM #4",
                    .pin_mod  = U(1) | U(0),
                    .pin_high = U(1) | U(0), },
        /* F9-F10: PAL/NTSC, DF0/DF1. */
        [F(9)]  = { .str = "Output2 LOW",
                    .pin_mod  = U(2), },
        [F(10)] = { .str = "Output2 HIGH",
                    .pin_mod  = U(2),
                    .pin_high = U(2), },
    }
#endif

#if 0
    /* An example configuration for DF0/DF1 switching, for Piotr. 
     * Also an example of multi-row hotkey description text. */
    .user_pin_opendrain = 0,
    .user_pin_pushpull  = U(0),
    .user_pin_high      = U(0), /* Default = DF0: AmiGotek */
    .hotkey = {
        [F(9)]  = { .str = "DF0: AmiGotek\0DF1: Floppy",
                    .pin_mod  = U(0),
                    .pin_high = U(0), },
        [F(10)] = { .str = "DF0: Floppy\0DF1: AmiGotek",
                    .pin_mod  = U(0), },
    }
#endif

#undef F
#undef U

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

