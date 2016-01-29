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
 *  USB Device Descriptors, for library use when in USB device mode. Descriptors are special 
 *  computer-readable structures which the host requests upon device enumeration, to determine
 *  the device's capabilities and functions.  
 */

#include "Descriptors.h"
#include "dualMoco.h"

/* On some devices, there is a factory set internal serial number which can be automatically sent to the host as
 * the device's serial number when the Device Descriptor's .SerialNumStrIndex entry is set to USE_INTERNAL_SERIAL.
 * This allows the host to track a device across insertions on different ports, allowing them to retain allocated
 * resources like COM port numbers and drivers. On demos using this feature, give a warning on unsupported devices
 * so that the user can supply their own serial number descriptor instead or remove the USE_INTERNAL_SERIAL value
 * from the Device Descriptor (forcing the host to generate a serial number for each device from the VID, PID and
 * port location).
 */
#if (USE_INTERNAL_SERIAL == NO_DESCRIPTOR)
	#warning USE_INTERNAL_SERIAL is not available on this AVR - please manually construct a device serial descriptor.
#endif

/** Device descriptor structure. This descriptor, located in FLASH memory, describes the overall
 *  device characteristics, including the supported USB version, control endpoint size and the
 *  number of device configurations. The descriptor is read out by the USB host when the enumeration
 *  process begins.
 */
/* for SERIAL */
const USB_Descriptor_Device_t PROGMEM DeviceDescriptorSerial =
{
	.Header                 = {.Size = sizeof(USB_Descriptor_Device_t), .Type = DTYPE_Device},
		
	.USBSpecification       = VERSION_BCD(01.10),
	.Class                  = 0x02,
	.SubClass               = 0x00,
	.Protocol               = 0x00,
				
	.Endpoint0Size          = FIXED_CONTROL_ENDPOINT_SIZE,
		
	.VendorID               = ARDUINO_VID, // VID

	.ProductID          	= ARDUINO_MODEL_PID, // PID
	.ReleaseNumber          = 0x0001,
		
	.ManufacturerStrIndex   = 0x01,
	.ProductStrIndex        = 0x02,
	.SerialNumStrIndex      = USE_INTERNAL_SERIAL,
		
	.NumberOfConfigurations = FIXED_NUM_CONFIGURATIONS
};

/* for MIDI */
const USB_Descriptor_Device_t PROGMEM DeviceDescriptorMIDI =
{
	.Header                 = {.Size = sizeof(USB_Descriptor_Device_t), .Type = DTYPE_Device},
		
	.USBSpecification       = VERSION_BCD(01.10),
	.Class                  = 0x00,
	.SubClass               = 0x00,
	.Protocol               = 0x00,
				
	.Endpoint0Size          = FIXED_CONTROL_ENDPOINT_SIZE,
		
	.VendorID               = 0x03EB, // Atmel
	.ProductID              = 0x2048, // LUFA USB-MIDI Demo application
	.ReleaseNumber          = 0x0000,
		
	.ManufacturerStrIndex   = 0x01,
	.ProductStrIndex        = 0x02,
	.SerialNumStrIndex      = NO_DESCRIPTOR,
		
	.NumberOfConfigurations = FIXED_NUM_CONFIGURATIONS
};

/** Configuration descriptor structure. This descriptor, located in FLASH memory, describes the usage
 *  of the device in one of its supported configurations, including information about any device interfaces
 *  and endpoints. The descriptor is read out by the USB host during the enumeration process when selecting
 *  a configuration so that the host may correctly communicate with the USB device.
 */
/* for Serial */
const USB_Descriptor_ConfigurationCDC_t PROGMEM ConfigurationDescriptorSerial =
{
	.Config = 
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Configuration_Header_t), .Type = DTYPE_Configuration},

			.TotalConfigurationSize = sizeof(USB_Descriptor_ConfigurationCDC_t),
			.TotalInterfaces        = 2,
				
			.ConfigurationNumber    = 1,
			.ConfigurationStrIndex  = NO_DESCRIPTOR,
				
			.ConfigAttributes       = (USB_CONFIG_ATTR_BUSPOWERED | USB_CONFIG_ATTR_SELFPOWERED),
			
			.MaxPowerConsumption    = USB_CONFIG_POWER_MA(100)
		},
		
	.CDC_CCI_Interface = 
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber        = 0,
			.AlternateSetting       = 0,
			
			.TotalEndpoints         = 1,
				
			.Class                  = 0x02,
			.SubClass               = 0x02,
			.Protocol               = 0x01,
				
			.InterfaceStrIndex      = NO_DESCRIPTOR
		},

	.CDC_Functional_IntHeader = 
		{
			.Header                 = {.Size = sizeof(CDC_FUNCTIONAL_DESCRIPTOR(2)), .Type = 0x24},
			.SubType                = 0x00,
			
			.Data                   = {0x01, 0x10}
		},

	.CDC_Functional_AbstractControlManagement = 
		{
			.Header                 = {.Size = sizeof(CDC_FUNCTIONAL_DESCRIPTOR(1)), .Type = 0x24},
			.SubType                = 0x02,
			
			.Data                   = {0x06}
		},
		
	.CDC_Functional_Union = 
		{
			.Header                 = {.Size = sizeof(CDC_FUNCTIONAL_DESCRIPTOR(2)), .Type = 0x24},
			.SubType                = 0x06,
			
			.Data                   = {0x00, 0x01}
		},

	.CDC_NotificationEndpoint = 
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
			
			.EndpointAddress        = (ENDPOINT_DESCRIPTOR_DIR_IN | CDC_NOTIFICATION_EPNUM),
			.Attributes             = (EP_TYPE_INTERRUPT | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = CDC_NOTIFICATION_EPSIZE,
			.PollingIntervalMS      = 0xFF
		},

	.CDC_DCI_Interface = 
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber        = 1,
			.AlternateSetting       = 0,
			
			.TotalEndpoints         = 2,
				
			.Class                  = 0x0A,
			.SubClass               = 0x00,
			.Protocol               = 0x00,
				
			.InterfaceStrIndex      = NO_DESCRIPTOR
		},

	.CDC_DataOutEndpoint = 
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
			
			.EndpointAddress        = (ENDPOINT_DESCRIPTOR_DIR_OUT | CDC_RX_EPNUM),
			.Attributes             = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = CDC_TXRX_EPSIZE,
			.PollingIntervalMS      = 0x01
		},
		
	.CDC_DataInEndpoint = 
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
			
			.EndpointAddress        = (ENDPOINT_DESCRIPTOR_DIR_IN | CDC_TX_EPNUM),
			.Attributes             = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = CDC_TXRX_EPSIZE,
			.PollingIntervalMS      = 0x01
		}
};

/* for MIDI */
const USB_Descriptor_ConfigurationMIDI_t PROGMEM ConfigurationDescriptorMIDI =
{
	.Config = 
		{
			.Header                   = {.Size = sizeof(USB_Descriptor_Configuration_Header_t), .Type = DTYPE_Configuration},

			.TotalConfigurationSize   = sizeof(USB_Descriptor_ConfigurationMIDI_t),
			.TotalInterfaces          = 2,

			.ConfigurationNumber      = 1,
			.ConfigurationStrIndex    = NO_DESCRIPTOR,
				
			.ConfigAttributes         = (USB_CONFIG_ATTR_BUSPOWERED | USB_CONFIG_ATTR_SELFPOWERED),
			
			.MaxPowerConsumption      = USB_CONFIG_POWER_MA(100)
		},
		
	.Audio_ControlInterface = 
		{
			.Header                   = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber          = 0,
			.AlternateSetting         = 0,
			
			.TotalEndpoints           = 0,
				
			.Class                    = 0x01,
			.SubClass                 = 0x01,
			.Protocol                 = 0x00,
				
			.InterfaceStrIndex        = NO_DESCRIPTOR
		},
	
	.Audio_ControlInterface_SPC = 
		{
			.Header                   = {.Size = sizeof(USB_Audio_Interface_AC_t), .Type = DTYPE_AudioInterface},
			.Subtype                  = DSUBTYPE_Header,
			
			.ACSpecification          = VERSION_BCD(01.00),
			.TotalLength              = sizeof(USB_Audio_Interface_AC_t),
			
			.InCollection             = 1,
			.InterfaceNumbers         = {1},
		},

	.Audio_StreamInterface = 
		{
			.Header                   = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber          = 1,
			.AlternateSetting         = 0,
			
			.TotalEndpoints           = 2,
				
			.Class                    = 0x01,
			.SubClass                 = 0x03,
			.Protocol                 = 0x00,
				
			.InterfaceStrIndex        = NO_DESCRIPTOR
		},
		
	.Audio_StreamInterface_SPC = 
		{
			.Header                   = {.Size = sizeof(USB_MIDI_AudioInterface_AS_t), .Type = DTYPE_AudioInterface},
			.Subtype                  = DSUBTYPE_General,

			.AudioSpecification       = VERSION_BCD(01.00),
			
			.TotalLength              = (sizeof(USB_Descriptor_ConfigurationMIDI_t) -
			                             offsetof(USB_Descriptor_ConfigurationMIDI_t, Audio_StreamInterface_SPC))
		},

	.MIDI_In_Jack_Emb = 
		{
			.Header                   = {.Size = sizeof(USB_MIDI_In_Jack_t), .Type = DTYPE_AudioInterface},
			.Subtype                  = DSUBTYPE_InputJack,
			
			.JackType                 = MIDI_JACKTYPE_EMBEDDED,
			.JackID                   = 0x01,
			
			.JackStrIndex             = NO_DESCRIPTOR
		},

	.MIDI_In_Jack_Ext = 
		{
			.Header                   = {.Size = sizeof(USB_MIDI_In_Jack_t), .Type = DTYPE_AudioInterface},
			.Subtype                  = DSUBTYPE_InputJack,
			
			.JackType                 = MIDI_JACKTYPE_EXTERNAL,
			.JackID                   = 0x02,
			
			.JackStrIndex             = NO_DESCRIPTOR
		},
		
	.MIDI_Out_Jack_Emb = 
		{
			.Header                   = {.Size = sizeof(USB_MIDI_Out_Jack_t), .Type = DTYPE_AudioInterface},
			.Subtype                  = DSUBTYPE_OutputJack,
			
			.JackType                 = MIDI_JACKTYPE_EMBEDDED,
			.JackID                   = 0x03,

			.NumberOfPins             = 1,
			.SourceJackID             = {0x02},
			.SourcePinID              = {0x01},
			
			.JackStrIndex             = NO_DESCRIPTOR
		},

	.MIDI_Out_Jack_Ext = 
		{
			.Header                   = {.Size = sizeof(USB_MIDI_Out_Jack_t), .Type = DTYPE_AudioInterface},
			.Subtype                  = DSUBTYPE_OutputJack,
			
			.JackType                 = MIDI_JACKTYPE_EXTERNAL,
			.JackID                   = 0x04,

			.NumberOfPins             = 1,
			.SourceJackID             = {0x01},
			.SourcePinID              = {0x01},
			
			.JackStrIndex             = NO_DESCRIPTOR
		},

	.MIDI_In_Jack_Endpoint = 
		{
			.Endpoint = 
				{
					.Header              = {.Size = sizeof(USB_Audio_StreamEndpoint_Std_t), .Type = DTYPE_Endpoint},

					.EndpointAddress     = (ENDPOINT_DESCRIPTOR_DIR_OUT | MIDI_STREAM_OUT_EPNUM),
					.Attributes          = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
					.EndpointSize        = MIDI_STREAM_EPSIZE,
					.PollingIntervalMS   = 0
				},
			
			.Refresh                  = 0,
			.SyncEndpointNumber       = 0
		},
		
	.MIDI_In_Jack_Endpoint_SPC = 
		{
			.Header                   = {.Size = sizeof(USB_MIDI_Jack_Endpoint_t), .Type = DTYPE_AudioEndpoint},
			.Subtype                  = DSUBTYPE_General,

			.TotalEmbeddedJacks       = 0x01,
			.AssociatedJackID         = {0x01}
		},

	.MIDI_Out_Jack_Endpoint = 
		{
			.Endpoint = 
				{
					.Header              = {.Size = sizeof(USB_Audio_StreamEndpoint_Std_t), .Type = DTYPE_Endpoint},

					.EndpointAddress     = (ENDPOINT_DESCRIPTOR_DIR_IN | MIDI_STREAM_IN_EPNUM),
					.Attributes          = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
					.EndpointSize        = MIDI_STREAM_EPSIZE,
					.PollingIntervalMS   = 0
				},
			
			.Refresh                  = 0,
			.SyncEndpointNumber       = 0
		},
		
	.MIDI_Out_Jack_Endpoint_SPC = 
		{
			.Header                   = {.Size = sizeof(USB_MIDI_Jack_Endpoint_t), .Type = DTYPE_AudioEndpoint},
			.Subtype                  = DSUBTYPE_General,

			.TotalEmbeddedJacks       = 0x01,
			.AssociatedJackID         = {0x03}
		}
};

/** Language descriptor structure. This descriptor, located in FLASH memory, is returned when the host requests
 *  the string descriptor with index 0 (the first index). It is actually an array of 16-bit integers, which indicate
 *  via the language ID table available at USB.org what languages the device supports for its string descriptors.
 */
const USB_Descriptor_String_t PROGMEM LanguageString =
{
	.Header                 = {.Size = USB_STRING_LEN(1), .Type = DTYPE_String},
		
	.UnicodeString          = {LANGUAGE_ID_ENG}
};

/** Manufacturer descriptor string. This is a Unicode string containing the manufacturer's details in human readable
 *  form, and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.
 */
/* for Serial */
const USB_Descriptor_String_t PROGMEM ManufacturerStringSerial =
{
	.Header                 = {.Size = USB_STRING_LEN(24), .Type = DTYPE_String},
		
	.UnicodeString          = L"Arduino (www.arduino.cc)"
};

/* for MIDI */
const USB_Descriptor_String_t PROGMEM ManufacturerStringMIDI =
{
	.Header                 = {.Size = USB_STRING_LEN(17), .Type = DTYPE_String},

	.UnicodeString          = L"kuwatay@nifty.com"
};
/** Product descriptor string. This is a Unicode string containing the product's details in human readable form,
 *  and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.
 */
/* for Serial */
const USB_Descriptor_String_t PROGMEM ProductStringSerial =
{
	#if (ARDUINO_MODEL_PID == ARDUINO_UNO_PID)
		.Header                 = {.Size = USB_STRING_LEN(11), .Type = DTYPE_String},
			
		.UnicodeString          = L"Arduino Uno"
	#elif (ARDUINO_MODEL_PID == ARDUINO_MEGA2560_PID)
		.Header                 = {.Size = USB_STRING_LEN(17), .Type = DTYPE_String},
			
		.UnicodeString          = L"Arduino Mega 2560"
	#elif (ARDUINO_MODEL_PID == ATMEL_LUFA_DEMO_PID)
		.Header                 = {.Size = USB_STRING_LEN(14), .Type = DTYPE_String},
			
		.UnicodeString          = L"Lufa USBSerial"
	#endif
	
};
/* for MIDI */
const USB_Descriptor_String_t PROGMEM ProductStringMIDI =
{
	.Header                 = {.Size = USB_STRING_LEN(8), .Type = DTYPE_String},

	.UnicodeString          = L"MocoLUFA"
};

/** This function is called by the library when in device mode, and must be overridden (see library "USB Descriptors"
 *  documentation) by the application code so that the address and size of a requested descriptor can be given
 *  to the USB library. When the device receives a Get Descriptor request on the control endpoint, this function
 *  is called so that the descriptor details can be passed back and the appropriate descriptor sent back to the
 *  USB host.
 */
uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint8_t wIndex,
                                    void** const DescriptorAddress)
{
	const uint8_t  DescriptorType   = (wValue >> 8);
	const uint8_t  DescriptorNumber = (wValue & 0xFF);

	void*    Address = NULL;
	uint16_t Size    = NO_DESCRIPTOR;

	switch (DescriptorType)
	{
		case DTYPE_Device: 
		  if (mocoMode == 1) {
			Address = (void*)&DeviceDescriptorMIDI;
			Size    = sizeof(USB_Descriptor_Device_t);
		  } else {
			Address = (void*)&DeviceDescriptorSerial;
			Size    = sizeof(USB_Descriptor_Device_t);
		  }
			break;
		case DTYPE_Configuration: 
		  if (mocoMode == 1) {
			Address = (void*)&ConfigurationDescriptorMIDI;
			Size    = sizeof(USB_Descriptor_ConfigurationMIDI_t);
		  } else {
			Address = (void*)&ConfigurationDescriptorSerial;
			Size    = sizeof(USB_Descriptor_ConfigurationCDC_t);
		  }
			break;
		case DTYPE_String: 
			switch (DescriptorNumber)
			{
				case 0x00:
					Address = (void*)&LanguageString;
					Size    = pgm_read_byte(&LanguageString.Header.Size);
					break;
				case 0x01:
				  if (mocoMode == 1) {
					Address = (void*)&ManufacturerStringMIDI;
					Size    = pgm_read_byte(&ManufacturerStringMIDI.Header.Size);
				  } else {
					Address = (void*)&ManufacturerStringSerial;
					Size    = pgm_read_byte(&ManufacturerStringSerial.Header.Size);
				  }
					break;
				case 0x02:
				  if (mocoMode == 1) {
					Address = (void*)&ProductStringMIDI;
					Size    = pgm_read_byte(&ProductStringMIDI.Header.Size);
				  } else {
					Address = (void*)&ProductStringSerial;
					Size    = pgm_read_byte(&ProductStringSerial.Header.Size);
				  }
					break;
			}
			
			break;
	}
	
	*DescriptorAddress = Address;
	return Size;
}
