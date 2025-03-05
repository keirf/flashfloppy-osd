/*
 * userpin.h
 *
 * User pin definitions for FlashFloppy OSD.
 *
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

/* Amiga CTS Input (A12): For use with automatic video output switch. */
#define gpio_amicts gpioa
#define pin_amicts  12

/* User outputs are PB8, PB9, PB12. */
#define gpio_user gpiob
#define pin_u0 8
#define pin_u2 12

/* Video input switch. */
enum vs_state { VS_AUTO=0, VS_AMIGA, VS_RTG};
extern uint8_t videoswitch_state;
const extern char *vs_state_pretty[];

/* User pin function prototypes. */
void user_pin_init(void);
void videoswitch_next(void);
void videoswitch_update(void);

/*
 * Local variables:
 * mode: C
 * c-file-style: "Linux"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
