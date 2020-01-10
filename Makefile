CC=avr-gcc
OBJCOPY=avr-objcopy
AR=avr-gcc-ar

NAME = sketch

# main sketch
OBJ += sketch.o

# rest of the objects
OBJ += arduboy.o main.o USBCore.o CDC.o

CFLAGS= -O2 -w -std=c99
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -flto -fno-fat-lto-objects
CFLAGS += -mmcu=atmega32u4
CFLAGS += -DF_CPU=16000000L
CFLAGS += -DARDUBOY_10 -DUSB_VID=0x2341 -DUSB_PID=0x8036
CFLAGS += "-DUSB_MANUFACTURER=\"Unknown\""
CFLAGS += "-DUSB_PRODUCT=\"Arduboy\""
CFLAGS += -Wall

UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S),Linux)
PORT=/dev/ttyACM0
STTYFLAGS=-F
endif

ifeq ($(UNAME_S),Darwin)
PORT=/dev/cu.usbmodem1411
STTYFLAGS=-f
endif

default: $(NAME).hex

%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

$(NAME).elf: $(OBJ)
	$(CC) -w -O2 -flto -fuse-linker-plugin -Wl,--gc-sections -mmcu=atmega32u4 -L. -o $@ $(OBJ)

OBJFLAGS=-O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load \
	--no-change-warnings --change-section-lma .eeprom=0

$(NAME).hex: $(NAME).elf
	$(OBJCOPY) $(OBJFLAGS) $^ $(NAME).eep
	$(OBJCOPY) -O ihex -R .eeprom $^ $@

flash: $(NAME).hex
	echo $(UNAME_S)
	stty $(STTYFLAGS) $(PORT) 1200
	sleep 1
	avrdude -patmega32u4 -cavr109 -P$(PORT) -b57600 -v -U flash:w:$<

clean:
	$(RM) $(OBJ) $(NAME).hex $(NAME).elf $(NAME).eep
