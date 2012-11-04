/*
             LUFA Library
     Copyright (C) Dean Camera, 2010.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com
*/

/*
  Copyright 2010  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this 
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in 
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting 
  documentation, and that the name of the author not be used in 
  advertising or publicity pertaining to distribution of the 
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the Arduino-usbserial project. This file contains the main tasks of
 *  the project and is responsible for the initial application hardware configuration.
 */

#include "Arduino-usbserial.h"

#include <arduino/pins.h>
#include <arduino/spi.h>

#include "ev_fat.h"

/*
  These pins are the SPI pins on the SPI header next to the reset button and
  USB connector on the Arduino Uno rev.3. Additionally, we put slave select
  and card detect on two of the four extra Atmega16U2 GPIO that have pads
  (but no header) marked "JP2". (Card detect is currently unused).

  SPI Pinout:

    MISO 1 * * 2 5V
     SCK 3 * * 4 MOSI
   RESET 5 * * 6 GND     (the reset pin is the one closest to the reset button)

  JP2 pad pinout:

        19 * * 18        (18 is the one closest to the TX LED)
	21 * * 20        (21 is the one closest to the reset button)
*/
#define pin_MISO MISO
#define pin_MOSI MOSI
#define pin_SCK  SCLK
#define pin_SS   18
#define pin_CD   20

#define FRAMERATE 2   /* period 2 centiseconds -> 50 Hz framerate */
#define FRAME_SIZE 672


/** Circular buffer to hold data from the host before it is sent to the device via the serial port. */
RingBuff_t USBtoUSART_Buffer;

/** Circular buffer to hold data from the serial port before it is sent to the host. */
RingBuff_t USARTtoUSB_Buffer;

/** Pulse generation counters to keep track of the number of milliseconds remaining for each pulse type */
volatile struct
{
	uint8_t TxLEDPulse; /**< Milliseconds remaining for data Tx LED pulse */
	uint8_t RxLEDPulse; /**< Milliseconds remaining for data Rx LED pulse */
	uint8_t PingPongLEDPulse; /**< Milliseconds remaining for enumeration Tx/Rx ping-pong LED pulse */
} PulseMSRemaining;

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config = 
			{
				.ControlInterfaceNumber         = 0,

				.DataINEndpointNumber           = CDC_TX_EPNUM,
				.DataINEndpointSize             = CDC_TXRX_EPSIZE,
				.DataINEndpointDoubleBank       = false,

				.DataOUTEndpointNumber          = CDC_RX_EPNUM,
				.DataOUTEndpointSize            = CDC_TXRX_EPSIZE,
				.DataOUTEndpointDoubleBank      = false,

				.NotificationEndpointNumber     = CDC_NOTIFICATION_EPNUM,
				.NotificationEndpointSize       = CDC_NOTIFICATION_EPSIZE,
				.NotificationEndpointDoubleBank = false,
			},
	};


static const char *filename = "LED-CUBE.000";

/* This is the current state in the SD card reader state machine. */
static uint8_t spi_current;
/*
  General counter used in SPI state machine to count how many bytes to receive
  etc.
*/
static uint16_t spi_counter;
/*
  Counter to handle timouts.
  Set to the desired timeout, in units of centiseconds (1/100 seconds).
  Then it counts down in a timer interrupt.
  When it reaches zero, it is set to (and remains at) 0xff as a timeout flag.
  Set to 0 to disable timeout.
*/
static volatile uint8_t timeout_counter;
/*
  When set to 1, then timeout sends 0xff to the SPI output register to wakeup
  the SPI state machine, instead of setting the 0xff timeout flag.
*/
static volatile uint8_t timeout_wake_spi_flag;
/* Pending data byte when throttling to maintain correct frame rate. */
static volatile uint8_t delayed_byte;
/* Set to 1 if waiting for time slot of start of next frame. */
static volatile uint8_t delayed_frame_flag;
/* Counts down from FRAMERATE to 0 to maintain correct frame rate. */
static volatile uint8_t frame_timer;
/* Set to 1 when spi temporarily paused waiting for room in serial FIFO. */
static volatile uint8_t spi_delay_flag;
/*
  Set to 0 when not doing SPI / SD-card.
  Set to 1 when trying to init SD-card.
  Set to 2 when SD-card detected and actively streaming data to serial.
*/
static volatile uint8_t spi_running;

/* Different types of SD cards, detected during card init. */
enum enum_sd_type { SD_TYPE_SD1, SD_TYPE_SD2, SD_TYPE_SDHC };
static int8_t sd_type;
/* State while running an SD card command. */
static uint8_t cmd_buf[6];
static uint8_t cmd_ptr;
static uint8_t cmd_result;
static uint8_t sd_acmd_cmd;
static uint32_t sd_acmd_arg;

static void sdcard_init(void);
static void spi_start(void);

/* For debugging, inject one byte on the serial->usb data stream. */
static void
outc(char c)
{
  if (USB_DeviceState == DEVICE_STATE_Configured && !RingBuffer_IsFull(&USARTtoUSB_Buffer))
    RingBuffer_Insert(&USARTtoUSB_Buffer, c);
}

/* Deselect the SD card, disabling it. */
static void
sd_card_deselect(void)
{
  pin_high(pin_SS);
}


/* Select the SD card, enabling it. */
static void
sd_card_select(void)
{
  pin_low(pin_SS);
}


static void
sd_error(void)
{
  spi_current = 0xfe;
  sd_card_deselect();
  /* Set a new timeout so we try again a bit later. */
  timeout_wake_spi_flag = 0;
  timeout_counter = 200;
  spi_running = 0;
}


/* Don't do SD card until >2 sek of USB inactivity. */
static void
mark_usb_activity(void)
{
  uint8_t running = spi_running;
  if (running)
  {
    sd_error();
    /* Reset the serial speed back to what USB thinks it is. */
    if (running == 2)
      EVENT_CDC_Device_LineEncodingChanged(&VirtualSerial_CDC_Interface);
  }
  timeout_counter = 200;
}


/* Change the serial speed to the 500kbit/sec that cube needs. */
static void
set_serial_to_500k(void)
{
  /* Wait for serial idle first. */
  while (!(UCSR1A & (1 << UDRE1)))
    ;
  /* Turn off serial before reconfiguring. */
  UCSR1B = 0;
  UCSR1A = 0;
  UCSR1C = 0;
  /* Set 500kbit, no parity, 8 bit, one stop bit. */
  UBRR1 = 3;
  UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
  UCSR1A = (1 << U2X1);
  UCSR1B = (1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1);
}


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	static const int8_t disable_usb = 1;

	SetupHardware();
        sdcard_init();

	
	RingBuffer_InitBuffer(&USBtoUSART_Buffer);
	RingBuffer_InitBuffer(&USARTtoUSB_Buffer);

	sei();

        timeout_counter = 200;

	for (;;)
	{
		/* Only try to read in bytes from the CDC interface if the transmit buffer is not full */
		if (!disable_usb && !(RingBuffer_IsFull(&USBtoUSART_Buffer)))
		{
			int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

			/* Read bytes from the USB OUT endpoint into the USART transmit buffer */
			if (!(ReceivedByte < 0))
			{
			  RingBuffer_Insert(&USBtoUSART_Buffer, ReceivedByte);
			  mark_usb_activity();
			}
		}
		
		/* Check if the UART receive buffer flush timer has expired or the buffer is nearly full */
		RingBuff_Count_t BufferCount = RingBuffer_GetCount(&USARTtoUSB_Buffer);
		if ((TIFR0 & (1 << TOV0)) || (BufferCount > BUFFER_NEARLY_FULL))
		{
			TIFR0 |= (1 << TOV0);

			if (USARTtoUSB_Buffer.Count) {
				LEDs_TurnOnLEDs(LEDMASK_TX);
				PulseMSRemaining.TxLEDPulse = TX_RX_LED_PULSE_MS;
			}

			/* Read bytes from the USART receive buffer into the USB IN endpoint */
			while (!disable_usb && BufferCount--)
			  CDC_Device_SendByte(&VirtualSerial_CDC_Interface, RingBuffer_Remove(&USARTtoUSB_Buffer));
			  
			/* Turn off TX LED(s) once the TX pulse period has elapsed */
			if (PulseMSRemaining.TxLEDPulse && !(--PulseMSRemaining.TxLEDPulse))
			  LEDs_TurnOffLEDs(LEDMASK_TX);

			/* Turn off RX LED(s) once the RX pulse period has elapsed */
			if (PulseMSRemaining.RxLEDPulse && !(--PulseMSRemaining.RxLEDPulse))
			  LEDs_TurnOffLEDs(LEDMASK_RX);
		}
		
		/* Load the next byte from the USART transmit buffer into the USART */
		if (!(RingBuffer_IsEmpty(&USBtoUSART_Buffer))) {
		  uint8_t b = RingBuffer_Remove(&USBtoUSART_Buffer);
		  Serial_TxByte(b);
		  	
		  	LEDs_TurnOnLEDs(LEDMASK_RX);
			PulseMSRemaining.RxLEDPulse = TX_RX_LED_PULSE_MS;

                        /*
                          If SPI was throttled due to full FIFO, restart it
                          now that there is room again.

			  Note! We have to re-check buffer full, in case we
			  race and fill it again between removing one byte and
			  disabling interrupt.
                        */
                        cli();
                        if (spi_delay_flag && !(RingBuffer_IsFull(&USBtoUSART_Buffer)))
                        {
                          spi_delay_flag = 0;
                          SPDR = 0xff;
                        }
                        sei();
		}

                if (!disable_usb)
                {
                  CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
                  USB_USBTask();
                }

		/* If no activity for 2 sec, try detect and use an SD card. */
                if (!spi_running && timeout_counter == 0xff)
                {
                  timeout_counter = 0;
                  spi_start();
                }
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Hardware Initialization */
	Serial_Init(9600, false);
	LEDs_Init();
	USB_Init();

	/* Start the flush timer so that overflows occur rapidly to push received bytes to the USB interface */
	TCCR0B = (1 << CS02);
	
	/* Pull target /RESET line high */
	AVR_RESET_LINE_PORT |= AVR_RESET_LINE_MASK;
	AVR_RESET_LINE_DDR  |= AVR_RESET_LINE_MASK;
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
}

/** Event handler for the library USB Unhandled Control Request event. */
void EVENT_USB_Device_UnhandledControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

/** Event handler for the CDC Class driver Line Encoding Changed event.
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	uint8_t ConfigMask = 0;

	switch (CDCInterfaceInfo->State.LineEncoding.ParityType)
	{
		case CDC_PARITY_Odd:
			ConfigMask = ((1 << UPM11) | (1 << UPM10));		
			break;
		case CDC_PARITY_Even:
			ConfigMask = (1 << UPM11);		
			break;
	}

	if (CDCInterfaceInfo->State.LineEncoding.CharFormat == CDC_LINEENCODING_TwoStopBits)
	  ConfigMask |= (1 << USBS1);

	switch (CDCInterfaceInfo->State.LineEncoding.DataBits)
	{
		case 6:
			ConfigMask |= (1 << UCSZ10);
			break;
		case 7:
			ConfigMask |= (1 << UCSZ11);
			break;
		case 8:
			ConfigMask |= ((1 << UCSZ11) | (1 << UCSZ10));
			break;
	}

	/* Must turn off USART before reconfiguring it, otherwise incorrect operation may occur */
	UCSR1B = 0;
	UCSR1A = 0;
	UCSR1C = 0;

	/* Special case 57600 baud for compatibility with the ATmega328 bootloader. */	
	UBRR1  = (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 57600)
			 ? SERIAL_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS)
			 : SERIAL_2X_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS);	

	UCSR1C = ConfigMask;
	UCSR1A = (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 57600) ? 0 : (1 << U2X1);
	UCSR1B = ((1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1));

	mark_usb_activity();
}

/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
 *  for later transmission to the host.
 */
ISR(USART1_RX_vect, ISR_BLOCK)
{
	uint8_t ReceivedByte = UDR1;

	if (USB_DeviceState == DEVICE_STATE_Configured)
	  RingBuffer_Insert(&USARTtoUSB_Buffer, ReceivedByte);
}

/** Event handler for the CDC Class driver Host-to-Device Line Encoding Changed event.
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	bool CurrentDTRState = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);

	if (CurrentDTRState)
	  AVR_RESET_LINE_PORT &= ~AVR_RESET_LINE_MASK;
	else
	  AVR_RESET_LINE_PORT |= AVR_RESET_LINE_MASK;
	mark_usb_activity();
}


/*
  Deliver a byte to the serial FIFO.

  Return 1 if there is still room in the FIFO, 0 if it is full after
  delivering the byte.

  If 0 is returned, SPI should be throttled (no further bytes delivered).
  When the serial interrupt removes a byte from the buffer, it will write
  0xff to the SPI output register to restart SPI.
*/
static uint8_t
deliver_or_throttle_serial(uint8_t byte, uint8_t interrupt_enabled)
{
  /* Deliver the byte. */
  RingBuffer_Insert(&USBtoUSART_Buffer, byte);
  // ToDo serial_interrupt_dre_enable();

  /* Atomically check if FIFO is full, and set the throttle flag if so. */
  cli();
  if (RingBuffer_IsFull(&USBtoUSART_Buffer))
  {
    /* Full. Throttle SPI until there is room again. */
    spi_delay_flag = 1;
    if (interrupt_enabled)
      sei();
    return 0;
  }
  else
  {
    if (interrupt_enabled)
      sei();
    return 1;
  }
}


/*
  Deliver a byte to the serial FIFO, but throttle to not overflow the FIFO
  and to not exceed 50Hz frame rate.
*/
static uint8_t
throttle_deliver_byte(uint8_t b, uint8_t throttle_timer,
                      uint8_t interrupts_enabled)
{
  if (throttle_timer)
  {
    uint8_t delay = 0;
    /* Delay delivery until next 50 Hz frame point in time. */
    cli();
    if (frame_timer > 0)
    {
      delay = 1;
      delayed_frame_flag = 1;
    }
    else
      frame_timer = 1 + FRAMERATE;
    if (interrupts_enabled)
      sei();
    if (delay)
    {
      delayed_byte = b;
      return 0;
    }
  }

  /*
    Deliver the byte to the serial FIFO.
    If this makes the FIFO full, we return 0 to throttle SPI until the next
    serial interrupt frees some space, and re-starts SPI.
  */
  return deliver_or_throttle_serial(b, interrupts_enabled);
}


static void
delayed_deliver_byte(void)
{
  frame_timer = FRAMERATE;
  if (throttle_deliver_byte(delayed_byte, 0, 0))
    SPDR = 0xff;

  /* Even more throttling due to serial FIFO still full. */
}


static void
spi_config_lowspeed(void)
{
  /* - Enable
     - Enable interrupt
     - Master
     - For SD card init use SPI clock d64 (256kHz) - later we can increase it.
  */
  SPCR = (1<<SPE) | (1<<SPIE) | (1<<MSTR) | (1<<SPR1) | (1<<SPR0);
  SPSR = (1<<SPI2X);
}


static void
spi_config_highspeed(void)
{
  /* - Enable
     - Enable interrupt
     - Master
     - 4 MHz clock (d4) (8MHz doesn't seem to work).
  */
  SPCR = (1<<SPE) | (1<<SPIE) | (1<<MSTR) | 0*(1<<SPR1) | 0*(1<<SPR0);
  SPSR = 0*(1<<SPI2X);
}


/*
  Timer 1 interrupt.

  Count down every 10 msec. If it reaches 0, a timeout occured. This is marked
  by setting the counter to 0xff.

  Setting to 0 disables timeout.
*/
ISR(TIMER1_COMPA_vect) {
  uint8_t val = timeout_counter;
  if (val != 0 && val != 0xff)
  {
    --val;
    if (val == 0)
    {
      if (timeout_wake_spi_flag)
      {
        /* Wake up the SPI machine by writing 0xff to SPI output register. */
        timeout_wake_spi_flag = 0;
        timeout_counter = 0;
        SPDR = 0xff;
      }
      else
      {
        /*
          SPI is running. As SPI interrupt is running with nested interrupts
          enabled, just set a flag to avoid nasty races; SPI interrupt
          handler will check the flag on next byte received.
        */
        timeout_counter = 0xff;
      }
    }
    else
      timeout_counter = val;
  }

  /* Handle the 50 Hz frame limit timer. */
  val = frame_timer;
  if (val > 0)
  {
    --val;
    frame_timer = val;
    if (val == 0 && delayed_frame_flag)
    {
      delayed_frame_flag = 0;
      delayed_deliver_byte();
    }
  }
}


/*
  Send one byte on the SPI.
  Disable interrupts first, so we avoid getting a nested interrupt in the
  SPI interrupt handler if the sending completes before returning from the
  previous interrupt.
*/
static inline void
spi_send_from_ISR(uint8_t byte)
{
  cli();
  SPDR = byte;
}


static void
sd_cmd_start(uint8_t cmd, uint32_t arg)
{
  sd_card_select();
  spi_send_from_ISR(0xff);
  cmd_buf[0] = cmd | 0x40;
  cmd_buf[1] = (arg >> 24) & 0xff;
  cmd_buf[2] = (arg >> 16) & 0xff;
  cmd_buf[3] = (arg >> 8) & 0xff;
  cmd_buf[4] = arg & 0xff;
  cmd_buf[5] = cmd ? 0x87 : 0x95;
  cmd_ptr = 0;
}

static uint8_t
sd_cmd_cont(void)
{
  uint8_t received_byte = SPDR;
  /* First wait until card is no longer busy. */
  if (cmd_ptr == 0 && received_byte != 0xff)
  {
    spi_send_from_ISR(0xff);
    return 1;
  }

  if (cmd_ptr < 6)
  {
    spi_send_from_ISR(cmd_buf[cmd_ptr]);
    ++cmd_ptr;
    return 1;
  }

  if (received_byte & 0x80)
  {
    spi_send_from_ISR(0xff);
    return 1;
  }

  cmd_result = received_byte;
  return 0;
}


static void
sd_acmd_start(uint8_t cmd, uint32_t arg)
{
  sd_acmd_cmd = cmd;
  sd_acmd_arg = arg;
  sd_cmd_start(55, 0);
}


static uint8_t
sd_acmd_cont(void)
{
  if (sd_acmd_cmd)
  {
    if (sd_cmd_cont())
      return 1;
    sd_card_deselect();

    /* Done with command 55, now send real acmd. */
    sd_cmd_start(sd_acmd_cmd, sd_acmd_arg);
    sd_acmd_cmd = 0;
    return 1;
  }
  else
    return sd_cmd_cont();
}


/* The SPI interrupt routine. */
ISR(SPI_STC_vect)
{
  uint8_t b;
  int res;
  uint32_t sector;
  static struct ev_file_status fat_st;
  static uint16_t read_skip;
  static uint16_t read_len;
  static uint8_t meta_data;
  static uint8_t file_open;
  static uint16_t frame_sofar;
  static uint32_t remain_bytes;

  /*
    We need to take care that the SPI interrupt handler does not lock out
    the serial interrupt for too long. At 500kbit, we need to service the
    serial interrupt with less than 320 instructions latency, or we can lose
    bytes.

    There are two ways that we can lock out other interrupt handlers:
     - By spending too many cycles in one handler invocation (ie. the
       ev_file_xxx() calls).
     - By running SPI at such high speed that the next SPI interrupt
       is triggered before the first one has finished. The SPI interrupt
       on AVR runs at a rather high priority, higher than the UART
       interrupts. At 4MHz SPI, there are just 32 cycles from writing a
       byte to the interrupt triggers, and this can easily be eaten up
       by the popping of all registers in the interrupt handler epilogue.

    We solve this by enabling interrupts inside the SPI handler at the start,
    but disabling it again before sending the next byte. Then we need to handle
    any timeout synchroneously by checking at the start of the handler, so
    we don't get nasty races with the timer interrupt.
  */
  sei();
  if (timeout_counter == 0xff)
  {
    sd_error();
    return;
  }

  switch(spi_current)
  {
  case 0:
    if (spi_counter++ < 10)
    {
      spi_send_from_ISR(0xff);
      break;
    }

    /* Now send a command 0, GO_IDLE_STATE. */
    sd_cmd_start(0,0);
    ++spi_current;
    break;
  case 1:
    if (sd_cmd_cont())
      break;
    sd_card_deselect();
    if (cmd_result != 0x01)
    {
      /* Repeat until we get "ok" response. */
      sd_cmd_start(0,0);
      break;
    }

    /* Send a command 8, SEND_IF_COND. */
    sd_cmd_start(8, 0x1aa);
    ++spi_current;
    break;
  case 2:
    if (sd_cmd_cont())
      break;
    if (cmd_result & 0x04)
    {
      /* Error means SD 1 type. */
      sd_card_deselect();
      sd_type = SD_TYPE_SD1;
      ++spi_current;
      goto go_do_acmd41;
    }
    sd_type = SD_TYPE_SD2;
    /* Receive 4 more bytes. */
    spi_counter = 4;
    spi_send_from_ISR(0xff);
    ++spi_current;
    break;
  case 3:
    --spi_counter;
    b = SPDR;
    if (spi_counter != 0)
    {
      spi_send_from_ISR(0xff);
      break;
    }
    sd_card_deselect();
    if (b != 0xaa)
    {
      sd_error();
      break;
    }
  go_do_acmd41:
    /* Now send acmd 41, SD_SEND_OP_COMD */
    sd_acmd_start(41, (sd_type == SD_TYPE_SD2 ? 0x40000000 : 0));
    ++spi_current;
  case 4:
    if (sd_acmd_cont())
      break;
    sd_card_deselect();
    if (cmd_result != 0)
    {
      sd_acmd_start(41, (sd_type == SD_TYPE_SD2 ? 0x40000000 : 0));
      break;
    }
    if (sd_type == SD_TYPE_SD1)
      goto after_sdhc_check;
    /* Send command 58 (READ_OCR) to check if SDHC card. */
    sd_cmd_start(58, 0);
    ++spi_current;
    break;
  case 5:
    if (sd_cmd_cont())
      break;
    if (cmd_result != 0)
    {
      sd_error();
      break;
    }
    /* Receive 4 more bytes. */
    spi_counter = 4;
    spi_send_from_ISR(0xff);
    ++spi_current;
    break;
  case 6:
    b = SPDR;
    if (spi_counter == 4)
    {
      if ((b & 0xc0) == 0xc0)
      {
        sd_type = SD_TYPE_SDHC;
      }
      else
      {
      }
    }
    --spi_counter;
    if (spi_counter != 0)
    {
      spi_send_from_ISR(0xff);
      break;
    }
    sd_card_deselect();

    /* Now card init is done. Switch to full speed and start reading! */
  after_sdhc_check:
    /*
      Wait 2 seconds, so cube gets time to settle and we get in sync.
      Then start reading the file!

      We do the wait by setting a flag that makes the timeout handler
      write an 0xff to the SPI output register to get us rolling again.
    */

    timeout_wake_spi_flag = 1;
    timeout_counter = 200;
    spi_current = 10;
    break;

  /* Now we start reading out data from the file. */
  case 10:
    spi_config_highspeed();
    spi_running = 2;
    set_serial_to_500k();

  start_over_from_beginning:
    fat_st.state = 0;
    file_open = 0;

  do_next_ev_fat_action:

    if (!file_open)
      res = ev_file_get_first_block(filename, &fat_st);
    else
      res = ev_file_get_next_block(&fat_st);

    if (res == EV_FILE_ST_STREAM_BYTES)
      goto stream_data_to_ev_fat;
    else if (res == EV_FILE_ST_DONE)
      goto read_data_from_sector;
    else
    {
      sd_error();
      break;
    }

  stream_data_to_ev_fat:
    sector = fat_st.st_stream_bytes.sec;
    read_skip = fat_st.st_stream_bytes.offset;
    read_len = fat_st.st_stream_bytes.len;
    meta_data = 1;
    spi_current = 20;
    goto do_read_block;

  read_data_from_sector:
    if (!file_open)
    {
      remain_bytes = fat_st.st_get_block_done.length;
      frame_sofar = 0;
      file_open = 1;
    }
    sector = fat_st.st_get_block_done.sector;
    meta_data = 0;
    spi_current = 20;
    goto do_read_block;


  case 20:
  do_read_block:
    /* Set a 500 msec read timeout. */
    timeout_counter = 50;
    sd_cmd_start(17, (sd_type == SD_TYPE_SDHC ? sector : sector << 9));
    ++spi_current;
    break;

  case 21:
    if (sd_cmd_cont())
      break;
    if (cmd_result)
    {
      sd_error();
      break;
    }
    spi_send_from_ISR(0xff);
    ++spi_current;
    break;
  case 22:
    b = SPDR;
    if (b == 0xff)
    {
      /* Still waiting ... */
      spi_send_from_ISR(0xff);
      break;
    }
    timeout_counter = 0;
    if (b != 0xfe)
    {
      sd_error();
      break;
    }
    spi_send_from_ISR(0xff);
    spi_counter = 512;
    ++spi_current;
    break;
  case 23:
    if (spi_counter)
    {
      b = SPDR;
      --spi_counter;
      if (meta_data)
      {
        /* Stream the byte to the ev_fat library. */
        if (read_skip > 0)
          --read_skip;
        else if (read_len > 0)
        {
          ev_file_stream_bytes(b, &fat_st);
          read_len--;
        }
        spi_send_from_ISR(0xff);
      }
      else
      {
        uint8_t throttle;
        /* Skip any extra data at the end of the last block of the file. */
        if (remain_bytes == 0)
        {
          spi_send_from_ISR(0xff);
          break;
        }
        /*
          Try to deliver the byte to the serial port.
          This may fail due to needing to throttle (serial FIFO is full or
          needing to delay until next 50 Hz frame slot). If so, non-zero is
          returned, we do nothing here, and later some other event (serial
          send or timer) will send a new 0xff on the SPI and get things
          running again.
        */
	--remain_bytes;
        ++frame_sofar;
        if (frame_sofar == FRAME_SIZE)
        {
          /* Throttle sending of last byte of frame to a 50Hz rate. */
          frame_sofar = 0;
          throttle = 1;
        }
        else
          throttle = 0;
        if (throttle_deliver_byte(b, throttle, 1))
          spi_send_from_ISR(0xff);
      }
      break;
    }
    /* Discard the first CRC byte. */
    spi_send_from_ISR(0xff);
    ++spi_current;
    break;
  case 24:
    /* Discard the second CRC byte. */
    sd_card_deselect();
    spi_send_from_ISR(0xff);
    ++spi_current;
    break;
  case 25:
    if (file_open && remain_bytes == 0)
      goto start_over_from_beginning;
    else
      goto do_next_ev_fat_action;
  }
}


static void init_timer(void)
{
  timeout_counter = 0;
  /* Interrupt every 20000 count, with d8 clock that means 100 interrupts/s. */
  OCR1AH = (uint16_t)19999 >> 8;
  OCR1AL = (uint16_t)19999 & 0xff;
  /* Set WGM1 3:0 to 4 (CTC mode), clock source to d8. */
  TCCR1A = 0;
  TCCR1B = 0x0a;
  /* Enable interrupt on OCF1A, compare match A. */
  TIMSK1 = 2;
}


static void
sdcard_init(void)
{
  pin_mode_output(pin_MOSI);
  pin_mode_input(pin_MISO);
  pin_mode_output(pin_SCK);
  pin_mode_output(pin_SS);
  /*
    The "real" SPI slave select pin must be set to output for SPI to work. But
    it is not exposed on the Arduino Uno, so we use a different pin to connect
    to slave select on the SD-card.
  */
  pin_mode_output(SS);

  pin_mode_input(pin_CD);
  /*
    Setting input-mode pin high turns up internal pull-up.
    This is needed so we can reliably detect card no present as high pin_CD,
    and card present as low pin_CD.
  */
  pin_high(pin_CD);

  sd_card_deselect();

  spi_config_lowspeed();

  init_timer();
}


static void
spi_state_reset(void)
{
  sd_card_deselect();
  spi_current = 0;
  spi_counter = 0;
  delayed_frame_flag = 0;
  frame_timer = 0;
  timeout_counter = 0;
}


/* Start SD card init by sending 10 x 0xff while card deselected. */
static void
spi_start(void)
{
  spi_state_reset();

  /* Set a 2 second timeout for initialisation. */
  timeout_counter=200;
  spi_running = 1;
  /* Transmit first byte, and let SPI interrupt handle things from here. */
  SPDR = 0xff;
}
