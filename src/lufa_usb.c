/*
 * lufa_usb.c
 *
 *  Created on: Jan 28, 2015
 *      Author: martin
 */


#include "VirtualSerial.h"

//USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_InterfaceR =
//{
//
//	{
//		INTERFACE_ID_CDC_CCI,
//
//		{
//			 CDC_TX_EPADDR,
//			 CDC_TXRX_EPSIZE,
//			 1
//		},
//
//		{
//			CDC_RX_EPADDR,
//			CDC_TXRX_EPSIZE,
//			1
//		},
//
//		{
//			CDC_NOTIFICATION_EPADDR,
//			CDC_NOTIFICATION_EPSIZE,
//			1
//		}
//	}
//};

//
///** Standard file stream for the CDC interface when set up, so that the virtual CDC COM port can be
// *  used like any regular character stream in the C APIs.
// */
////static FILE USBSerialStream;
//
///** Event handler for the library USB Connection event. */
//void EVENT_USB_Device_Connect(void)
//{
////	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
//}
//
///** Event handler for the library USB Disconnection event. */
//void EVENT_USB_Device_Disconnect(void)
//{
////	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
//}
//
///** Event handler for the library USB Configuration Changed event. */
//void EVENT_USB_Device_ConfigurationChanged(void)
//{
//	bool ConfigSuccess = true;
//
//	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_InterfaceR);
//
////	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
//}
//
///** Event handler for the library USB Control Request reception event. */
//void EVENT_USB_Device_ControlRequest(void)
//{
//	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_InterfaceR);
//}
