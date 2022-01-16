OBJS += amiga.o
OBJS += build_info.o
OBJS += cancellation.o
OBJS += config.o
OBJS += console.o
OBJS += i2c.o
OBJS += main.o
OBJS += string.o
OBJS += stm32f10x.o
OBJS += time.o
OBJS += timer.o
OBJS += userpin.o
OBJS += util.o
OBJS += vectors.o

.PHONY: build_info.c
build_info.o: CFLAGS += -DFW_VER="\"$(FW_VER)\""
