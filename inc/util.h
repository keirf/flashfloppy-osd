/*
 * util.h
 * 
 * Utility definitions.
 * 
 * Written & released by Keir Fraser <keir.xen@gmail.com>
 * 
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

#define ASSERT(p) if (!(p)) illegal();

typedef char bool_t;
#define TRUE 1
#define FALSE 0

#define m(bitnr) (1u<<(bitnr))

#ifndef offsetof
#define offsetof(a,b) __builtin_offsetof(a,b)
#endif
#define container_of(ptr, type, member) ({                      \
        typeof( ((type *)0)->member ) *__mptr = (ptr);          \
        (type *)( (char *)__mptr - offsetof(type,member) );})
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define min(x,y) ({                             \
    const typeof(x) _x = (x);                   \
    const typeof(y) _y = (y);                   \
    (void) (&_x == &_y);                        \
    _x < _y ? _x : _y; })

#define max(x,y) ({                             \
    const typeof(x) _x = (x);                   \
    const typeof(y) _y = (y);                   \
    (void) (&_x == &_y);                        \
    _x > _y ? _x : _y; })

#define min_t(type,x,y) \
    ({ type __x = (x); type __y = (y); __x < __y ? __x: __y; })
#define max_t(type,x,y) \
    ({ type __x = (x); type __y = (y); __x > __y ? __x: __y; })

void *memset(void *s, int c, size_t n);
void *memcpy(void *dest, const void *src, size_t n);
void *memmove(void *dest, const void *src, size_t n);

size_t strlen(const char *s);
size_t strnlen(const char *s, size_t maxlen);
int strcmp(const char *s1, const char *s2);
int strncmp(const char *s1, const char *s2, size_t n);
char *strcpy(char *dest, const char *src);
char *strrchr(const char *s, int c);
int tolower(int c);
int isspace(int c);

long int strtol(const char *nptr, char **endptr, int base);

uint16_t crc16_ccitt(const void *buf, size_t len, uint16_t crc);

int vsnprintf(char *str, size_t size, const char *format, va_list ap)
    __attribute__ ((format (printf, 3, 0)));

int snprintf(char *str, size_t size, const char *format, ...)
    __attribute__ ((format (printf, 3, 4)));

int vprintk(const char *format, va_list ap)
    __attribute__ ((format (printf, 1, 0)));

int printk(const char *format, ...)
    __attribute__ ((format (printf, 1, 2)));

#if !defined(NDEBUG)
#define dprintk(f, a...) printk(f, ##a)
#else
static inline int dprintk(const char *format, ...) { return 0; }
#endif

#define le16toh(x) (x)
#define le32toh(x) (x)
#define htole16(x) (x)
#define htole32(x) (x)
#define be16toh(x) _rev16(x)
#define be32toh(x) _rev32(x)
#define htobe16(x) _rev16(x)
#define htobe32(x) _rev32(x)

/* Display control */
void display_off(void);

/* Amiga keyboard */
#define AMI_RETURN 0x44
#define AMI_F(x)   (0x4f+(x))
#define AMI_DEL    0x46
#define AMI_HELP   0x5f
#define AMI_L_CTRL 0x63
#define AMI_L_ALT  0x64
#define AMI_LEFT   0x4f
#define AMI_RIGHT  0x4e
#define AMI_UP     0x4c
#define AMI_KPLEFTPAREN  0x5a
#define AMI_KPRIGHTPAREN 0x5b
#define AMI_KPSLASH      0x5c
#define AMI_KPPLUS       0x5e
#define AMI_KPMINUS      0x4a

#define AMI_W            0x11
#define AMI_A            0x20
#define AMI_S            0x21
#define AMI_D            0x22

extern bool_t keyboard_held;
bool_t amiga_key_pressed(uint8_t keycode);
#define amiga_key_pressed_now(k) (amiga_key_pressed(k) & 1)
void amiga_init(void);

/* Button codes */
#define B_LEFT 1
#define B_RIGHT 2
#define B_SELECT 4
#define B_PROCESSED 8

/* Serial I/O */
void console_init(void);
void console_sync(void);
void console_barrier(void);

struct display {
    int rows, cols, on;
    uint8_t heights;
    uint8_t text[4][40];
};

/* LCD / FF-OSD I2C Protocol. */
void i2c_init(void);
void i2c_process(void);
extern struct display i2c_display;
extern bool_t i2c_osd_protocol;
extern uint8_t i2c_buttons_rx; /* Gotek -> FF_OSD */
extern struct packed i2c_osd_info {
    uint8_t protocol_ver;
    uint8_t fw_major, fw_minor;
    uint8_t buttons;
} i2c_osd_info;

/* Slave OSD on I2C2. */
bool_t slave_init(void);

/* Build info. */
extern const char fw_ver[];

/* Text/data/BSS address ranges. */
extern char _stext[], _etext[];
extern char _sdat[], _edat[], _ldat[];
extern char _sbss[], _ebss[];

/* Stacks. */
extern uint32_t _thread_stacktop[], _thread_stackbottom[];
extern uint32_t _irq_stacktop[], _irq_stackbottom[];

/* IRQ priorities, 0 (highest) to 15 (lowest). */
#define SYNC_IRQ_PRI          2
#define I2C_IRQ_PRI           4
#define AMIKBD_IRQ_PRI        5
#define TIMER_IRQ_PRI         8
#define CONSOLE_IRQ_PRI      14

/*
 * Local variables:
 * mode: C
 * c-file-style: "Linux"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
