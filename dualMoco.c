/*
     dualMocoLUFA Project
     Copyright (C) 2013 by morecat_lab

     2013/09/22
              
     http://morecatlab.akiba.coocan.jp/

     based on LUFA-100807
*/
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
 *  Main source file for the dualMocoLUFA project. 
 * This file contains the main tasks of the project and is responsible for the initial application hardware configuration.
 */

#include "dualMoco.h"
#include "Arduino-usbserial.h"

uchar mocoMode = 1;	/* 0: Serial , 1: MIDI */
uchar highSpeed = 0;		/* 0: normal speed(31250bps),
				   1: high speed (1250000bps) */

/** Circular buffer to hold data from the host before it is sent to the device via the serial port. */
RingBuff_t USBtoUSART_Buffer;

/** Circular buffer to hold data from the serial port before it is sent to the host. */
RingBuff_t USARTtoUSB_Buffer;

/** Pulse generation counters to keep track of the number of milliseconds remaining for each pulse type */
volatile struct {
  uint8_t TxLEDPulse; /**< Milliseconds remaining for data Tx LED pulse */
  uint8_t RxLEDPulse; /**< Milliseconds remaining for data Rx LED pulse */
  uint8_t PingPongLEDPulse; /**< Milliseconds remaining for enumeration Tx/Rx ping-pong LED pulse */
} PulseMSRemaining;

#define    RX_SIZE        (HW_CDC_BULK_IN_SIZE)
static uchar utxrdy = FALSE;	/* USB Packet ready in utx_buf */
static uchar rx_buf[RX_SIZE];	/* tempory buffer */
static uchar utx_buf[RX_SIZE];	/* BULK_IN buffer */

#define    TX_SIZE        (HW_CDC_BULK_OUT_SIZE<<2)
#define    TX_MASK        (TX_SIZE-1)
static uchar uwptr = 0, irptr = 0;
static uchar tx_buf[TX_SIZE];

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
/* for serial */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface = {
  .Config = {
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
/* for MIDI */
USB_ClassInfo_MIDI_Device_t Keyboard_MIDI_Interface =  {
  .Config = {
    .StreamingInterfaceNumber = 1,
    
    .DataINEndpointNumber      = MIDI_STREAM_IN_EPNUM,
    .DataINEndpointSize        = MIDI_STREAM_EPSIZE,
    .DataINEndpointDoubleBank  = false,
    
    .DataOUTEndpointNumber     = MIDI_STREAM_OUT_EPNUM,
    .DataOUTEndpointSize       = MIDI_STREAM_EPSIZE,
    .DataOUTEndpointDoubleBank = false,
  },
};

void parseUSBMidiMessage(uchar *data, uchar len) {
  uchar cin = (*data) & 0x0f;	/* ignore cable no */
  uchar i;

  if (cin > 1) {		/* ignore cin == 0 and cin == 1 */
    for (i = 1 ; i < 4 ; i++) {
      tx_buf[uwptr++] = *(data + i); /* copy to buffer */
      uwptr &= TX_MASK;
      if (i == 1) {
	if ((cin == 5) || /* single byte system common */
	    (cin == 15))  /* single byte */
	  break;
      }
      if (i == 2) {
	if ((cin == 2) ||  /* two-byte system common */
	    (cin == 6) ||  /* system ex end with 2 bytes */
	    (cin == 12) || /* program change */
	    (cin == 13))   /* channel pressure */
	  break;
      }
    }
  }

  if (len > 4) {
    parseUSBMidiMessage(data+4, len-4);
  }
}

uchar parseSerialMidiMessage(uchar RxByte) {
  static uchar PC = 0;
  static uchar SysEx = FALSE;
  static uchar sysExCnt = 0;
  static uchar stateTransTable[] = {
    0, 				/* 0 dummy */
    0,				/* 1 dummy */
    3,				/* 2->3 NOTE OFF (3) */
    2 | 0x80,			/* 3->2 */
    5,				/* 4->5 NOTE ON (3) */
    4 | 0x80,			/* 5->4 */
    7,				/* 6->7 Polyphonic key pressure (3) */
    6 | 0x80,			/* 7->6 */
    9,				/* 8->9 Control Change (3) */
    8 | 0x80,			/* 8->9 */
    10 | 0x80,			/* 10->10 program change (2) */
    0,				/* dummy */
    12 | 0x80,			/* 12->12 Channel Pressure (2) */
    0,				/* 13 dummy */
    15,				/* 14->15 Pitch Bend (3) */
    14 | 0x80			/* 15->14 */
  };

  if(SysEx){  /* MIDI System Message */
    if(RxByte == 0xf7){		/* MIDI_EndSysEx */
      utx_buf[sysExCnt++] = RxByte;
      utx_buf[0] = sysExCnt + 3; /* 0x05(single byte), 0x06(two bytes), or 0x07(three bytes) */
      SysEx = FALSE;
      PC = 0;
      return TRUE;		/* send sysEx */
    } else {
      utx_buf[sysExCnt++] = RxByte;
      if (sysExCnt == 4) {	/* buffer full */
	utx_buf[0] = 0x04;	/* sysEx start */
	sysExCnt = 1;
	return TRUE;		/* send sysEx */
      }
    }
    return FALSE;
  }
  if (RxByte >= 0xf8){		/* Single Byte Message */
    utx_buf[0] = 0x0f;
    utx_buf[1] = RxByte;
    utx_buf[2] = 0;
    utx_buf[3] = 0;
    return TRUE;
  }

  if(RxByte > 0x7f){		/* Channel message */
    if(RxByte == 0xf0){		/* MIDI_StartSysEx */
      SysEx = TRUE;
      utx_buf[1] = 0xf0;
      sysExCnt = 2;		/* start utx_buf[2]  */
      return FALSE;
    }
    PC = 0;
  }

  if (PC == 0) {
    PC = (((RxByte >> 4) & 0x07) + 1) * 2;
    // conversion
    // 0x80 -> 2, 0x90 -> 4, 0xa0 -> 6, 0xb0 -> 8, 0xc0 -> 10, 0xd0 -> 12, 0xe0 -> 14
    rx_buf[0] = RxByte >> 4;
    rx_buf[1] = RxByte;
    rx_buf[3] = 0;
  } else {
    uchar tt = stateTransTable[PC];
    rx_buf[(PC & 1) + 2] = RxByte;
    PC = tt & 0x0f;
    if ((tt & 0x80) != 0) {
      memcpy(utx_buf, rx_buf, 4);
      return TRUE;
    }
  }
  return FALSE;
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void) {
  SetupHardware();

  if (mocoMode == 1) {
    processMIDI();
  } else {
    processSerial();
  }
}


void processMIDI() {
#define CNTMAX 40
  static uint8_t cnt = CNTMAX;

  sei();

  for (;;){
    /* receive from Serial MIDI line */
    if( UCSR1A & (1<<RXC1)) {
      utxrdy |= parseSerialMidiMessage(UDR1);
      LEDs_TurnOnLEDs(LEDMASK_TX);
      PulseMSRemaining.TxLEDPulse = TX_RX_LED_PULSE_MS;
    }

    /* send packets to USB MIDI */
    if( utxrdy ) {
      MIDI_Device_SendEventPacket(&Keyboard_MIDI_Interface, (MIDI_EventPacket_t *)&utx_buf);
      MIDI_Device_Flush(&Keyboard_MIDI_Interface);
      utxrdy = FALSE;
    }

    /* receive from USB MIDI */
    MIDI_EventPacket_t ReceivedMIDIEvent;
    while (MIDI_Device_ReceiveEventPacket(&Keyboard_MIDI_Interface, &ReceivedMIDIEvent)) {
      /* for each MIDI packet w/ 4 bytes */
      parseUSBMidiMessage((uchar *)&ReceivedMIDIEvent, 4);
      LEDs_TurnOnLEDs(LEDMASK_RX);
      PulseMSRemaining.RxLEDPulse = TX_RX_LED_PULSE_MS;
    }
      
    /* send to Serial MIDI line  */
    if( (UCSR1A & (1<<UDRE1)) && uwptr!=irptr ) {
      UDR1 = tx_buf[irptr++];
      irptr &= TX_MASK;
    }
    
    if (TIFR0 & (1 << TOV0)) {
      TIFR0 |= (1 << TOV0);
      /* Turn off TX LED(s) once the TX pulse period has elapsed */
      if (PulseMSRemaining.TxLEDPulse && !(--PulseMSRemaining.TxLEDPulse))
	LEDs_TurnOffLEDs(LEDMASK_TX);
      
      /* Turn off RX LED(s) once the RX pulse period has elapsed */
      if (PulseMSRemaining.RxLEDPulse && !(--PulseMSRemaining.RxLEDPulse))
	LEDs_TurnOffLEDs(LEDMASK_RX);
    }

    MIDI_Device_USBTask(&Keyboard_MIDI_Interface);
    USB_USBTask();

    if (highSpeed == 1) {
      cnt--;
      if (cnt == 0) {
	cnt = CNTMAX;
	PORTB ^= 0x02;
      }
    }
  } /* for */
}


void processSerial(void) {
	
  RingBuffer_InitBuffer(&USBtoUSART_Buffer);
  RingBuffer_InitBuffer(&USARTtoUSB_Buffer);

  sei();

  for (;;)
    {
      /* Only try to read in bytes from the CDC interface if the transmit buffer is not full */
      if (!(RingBuffer_IsFull(&USBtoUSART_Buffer)))
	{
	  int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
	  
	  /* Read bytes from the USB OUT endpoint into the USART transmit buffer */
	  if (!(ReceivedByte < 0))
	    RingBuffer_Insert(&USBtoUSART_Buffer, ReceivedByte);
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
	  while (BufferCount--)
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
	Serial_TxByte(RingBuffer_Remove(&USBtoUSART_Buffer));
	
	LEDs_TurnOnLEDs(LEDMASK_RX);
	PulseMSRemaining.RxLEDPulse = TX_RX_LED_PULSE_MS;
      }
		
      CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
      USB_USBTask();
    }
}


/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void) {
  /* Disable watchdog if enabled by bootloader/fuses */
  MCUSR &= ~(1 << WDRF);
  wdt_disable();

  /* PB1 = LED, PB2 = (MIDI/SERIAL), PB3 (NORMAL/HIGH) */
  DDRB = 0x02;		/* PB1 = OUTPUT, PB2 = INPUT, PB3 = INPUT */
  PORTB = 0x0C;		/* PULL-UP PB2, PB3 */

  if ((PINB & 0x04) == 0) { /* Arduino-serial (JUMPER BTW PB2-GND) */
    mocoMode = 0;
    Serial_Init(9600, false);
  } else {		/* Moco mode */
    mocoMode = 1;
    if ((PINB & 0x08) == 0) { /* high speed mode */
      highSpeed = 1;
      // UBRR1L = 1;		/* 500K at 16MHz clock */
      UBRR1L = 0;		/* 1M at 16MHz clock */
    } else {
      UBRR1L = 31;	/* 31250Hz at 16MHz clock */
    }
    UCSR1B = (1<<RXEN1) | (1<<TXEN1);
    PORTB = 0x0E;	       /* PORTB1 = HIGH */
  }

  /* Start the flush timer so that overflows occur rapidly to push received bytes to the USB interface */
  TCCR0B = (1 << CS02);
  
  /* Hardware Initialization */
  USB_Init();
  LEDs_Init();

  if (mocoMode == 0) {
    /* Pull target /RESET line high */
    AVR_RESET_LINE_PORT |= AVR_RESET_LINE_MASK;
    AVR_RESET_LINE_DDR  |= AVR_RESET_LINE_MASK;
  }
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void) {
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void) {
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void) {
  if (mocoMode == 1) {
    bool ConfigSuccess = true;
    ConfigSuccess &= MIDI_Device_ConfigureEndpoints(&Keyboard_MIDI_Interface);
  } else {
    CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
  }
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void) {
  if (mocoMode == 1) {
    MIDI_Device_ProcessControlRequest(&Keyboard_MIDI_Interface);
  }
}

/** Event handler for the library USB Unhandled Control Request event. */
void EVENT_USB_Device_UnhandledControlRequest(void) {
  if (mocoMode == 0) {
    CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
  } else {
    MIDI_Device_ProcessControlRequest(&Keyboard_MIDI_Interface);
  }
}

/** Event handler for the CDC Class driver Line Encoding Changed event.
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo) {
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
}

/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
 *  for later transmission to the host.
 */
ISR(USART1_RX_vect, ISR_BLOCK) {
  if (mocoMode == 0) {
    uint8_t ReceivedByte = UDR1;

    if (USB_DeviceState == DEVICE_STATE_Configured)
      RingBuffer_Insert(&USARTtoUSB_Buffer, ReceivedByte);
  }
}

/** Event handler for the CDC Class driver Host-to-Device Line Encoding Changed event.
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo) {
  bool CurrentDTRState = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);
	
  if (CurrentDTRState)
    AVR_RESET_LINE_PORT &= ~AVR_RESET_LINE_MASK;
  else
    AVR_RESET_LINE_PORT |= AVR_RESET_LINE_MASK;
}
