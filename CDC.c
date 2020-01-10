#include <avr/wdt.h>
#include <util/atomic.h>
#include <stdbool.h>

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#include "USBAPI.h"

typedef struct
{
	u32	dwDTERate;
	u8	bCharFormat;
	u8 	bParityType;
	u8 	bDataBits;
	u8	lineState;
} LineInfo;

static volatile LineInfo _usbLineInfo = { 57600, 0x00, 0x00, 0x00, 0x00 };
static volatile int32_t breakValue = -1;

static u8 wdtcsr_save;

#define WEAK __attribute__ ((weak))

extern const CDCDescriptor _cdcInterface PROGMEM;
const CDCDescriptor _cdcInterface =
{
	D_IAD(0, 2,
          CDC_COMMUNICATION_INTERFACE_CLASS,
          CDC_ABSTRACT_CONTROL_MODEL, 1),

	/* CDC communication interface */
	D_INTERFACE(CDC_ACM_INTERFACE,1,CDC_COMMUNICATION_INTERFACE_CLASS,CDC_ABSTRACT_CONTROL_MODEL,0),
	D_CDCCS(CDC_HEADER,0x10,0x01), // Header (1.10 bcd)
	D_CDCCS(CDC_CALL_MANAGEMENT,1,1), // Device handles call management (not)
	D_CDCCS4(CDC_ABSTRACT_CONTROL_MANAGEMENT,6), // SET_LINE_CODING, GET_LINE_CODING, SET_CONTROL_LINE_STATE supported
	D_CDCCS(CDC_UNION,CDC_ACM_INTERFACE,CDC_DATA_INTERFACE), // Communication interface is master, data interface is slave 0
	D_ENDPOINT(USB_ENDPOINT_IN (CDC_ENDPOINT_ACM),
               USB_ENDPOINT_TYPE_INTERRUPT,
               0x10,
               0x40),

	/* CDC data interface */
	D_INTERFACE(CDC_DATA_INTERFACE,
                2,
                CDC_DATA_INTERFACE_CLASS,
                0, 0),
	D_ENDPOINT(USB_ENDPOINT_OUT(CDC_ENDPOINT_OUT),
               USB_ENDPOINT_TYPE_BULK,
               USB_EP_SIZE,
               0),
	D_ENDPOINT(USB_ENDPOINT_IN (CDC_ENDPOINT_IN),
               USB_ENDPOINT_TYPE_BULK,
               USB_EP_SIZE,
               0)
};

int CDC_GetInterface(u8* interfaceNum)
{
	interfaceNum[0] += 2;	// uses 2
	return USB_SendControl(TRANSFER_PGM,&_cdcInterface,sizeof(_cdcInterface));
}

bool CDC_Setup(USBSetup *setup)
{
    u8 r = setup->bRequest;
    u8 requestType = setup->bmRequestType;

    if (REQUEST_DEVICETOHOST_CLASS_INTERFACE == requestType) {
        if (CDC_GET_LINE_CODING == r) {
            USB_SendControl(0,(void*)&_usbLineInfo,7);
            return true;
        }
    }

    if (REQUEST_HOSTTODEVICE_CLASS_INTERFACE == requestType) {
        if (CDC_SEND_BREAK == r) {
            breakValue =
                ((uint16_t)setup->wValueH << 8) | setup->wValueL;
        }

        if (CDC_SET_LINE_CODING == r) {
            USB_RecvControl((void*)&_usbLineInfo,7);
        }

        if (CDC_SET_CONTROL_LINE_STATE == r) {
            _usbLineInfo.lineState = setup->wValueL;

            // auto-reset into the bootloader is triggered when the port, already 
            // open at 1200 bps, is closed.  this is the signal to start the watchdog
            // with a relatively long period so it can finish housekeeping tasks
            // like servicing endpoints before the sketch ends

            uint16_t magic_key_pos = MAGIC_KEY_POS;

            // We check DTR state to determine if host port is open (bit 0 of lineState).
            if (_usbLineInfo.dwDTERate == 1200 &&
                (_usbLineInfo.lineState & 0x01) == 0) {
                // Store boot key
                *(uint16_t *)magic_key_pos = MAGIC_KEY;
                // Save the watchdog state in case the reset is aborted.
                wdtcsr_save = WDTCSR;
                wdt_enable(WDTO_120MS);
            } else if (*(uint16_t *)magic_key_pos == MAGIC_KEY) {
                // Most OSs do some intermediate steps when configuring ports and DTR can
                // twiggle more than once before stabilizing.
                // To avoid spurious resets we set the watchdog to 120ms and eventually
                // cancel if DTR goes back high.
                // Cancellation is only done if an auto-reset was started, which is
                // indicated by the magic key having been set.

                wdt_reset();
                // Restore the watchdog state in case the sketch was using it.
                WDTCSR |= (1<<WDCE) | (1<<WDE);
                WDTCSR = wdtcsr_save;

                {
                    /* Clean up RAMEND key */
                    *(uint16_t *)magic_key_pos = 0x0000;
                }
            }
        }
        return true;
    }
    return false;
}
