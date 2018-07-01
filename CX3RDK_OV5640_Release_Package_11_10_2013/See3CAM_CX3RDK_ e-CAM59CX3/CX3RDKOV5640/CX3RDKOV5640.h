/*
 ## e-con Systems USB UVC Stack – See3CAMCX3RDK Platform

 ## source file : CX3RDKOV5640.h
 ## ===========================
 ##
 ##  Copyright E-Con Systems, 2012-2013,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  PROPERTY OF ECON SYSTEMS

 ## www.e-consystems.com
 ##
 ##
 ## ===========================
*/

#ifndef _INCLUDED_CX3RDKOV5640_H_
#define _INCLUDED_CX3RDKOV5640_H_

#include <cyu3externcstart.h>
#include <cyu3types.h>
#include <cyu3usbconst.h>
#include "cyu3os.h"
#include <cyu3externcend.h>

/* This header file comprises of the UVC application contants and
 * the video frame configurations */

#define UVC_APP_THREAD_STACK           	(0x1000)        	/* Thread stack size */
#define UVC_APP_THREAD_PRIORITY        	(8)             	/* Thread priority */

/* I2C Data rate */
#define ES_USBI2C_I2C_BITRATE        	(100000)

/* Endpoint definition for UVC application */
#define ES_EP_UVC_VIDEO              	0x83           			/* EP 1 IN */
#define ES_EP_VIDEO_CONS_SOCKET      	CY_U3P_UIB_SOCKET_CONS_3 	/* Consumer socket 1 */
#define ES_EP_CONTROL_STATUS         	0x82           			/* EP 2 IN */

#define ES_PRODUCER_PPORT_SOCKET    	CY_U3P_PIB_SOCKET_0    		/* P-port Socket 0 is producer */
#define ES_PRODUCER_PPORT_SOCKET_1    	CY_U3P_PIB_SOCKET_1    	/* P-port Socket 0 is producer */

/* UVC descriptor types */
#define ES_INTF_ASSN_DSCR_TYPE       	(11)           /* Interface association descriptor type. */

/* UVC video streaming endpoint packet Size */
#define ES_EP_ISO_VIDEO_PKT_SIZE    	(0x400)
#define ES_EP_BULK_VIDEO_PKT_SIZE    	(0x400)

/* UVC video streaming endpoint packet Count */
#define ES_EP_ISO_VIDEO_PKTS_COUNT  	(0x03)

#define ES_EP_BULK_VIDEO_BURST_COUNT  	(0x0C)

/* UVC Buffer size - Will map to ISO Transaction size */
#define ES_UVC_ISO_STREAM_BUF_SIZE      (2928)

#define ES_UVC_BULK_STREAM_BUF_SIZE		(ES_EP_BULK_VIDEO_PKT_SIZE * ES_EP_BULK_VIDEO_BURST_COUNT);

/* UVC Buffer count */
#define ES_UVC_STREAM_BUF_COUNT     	(10)

/* Low byte - UVC video streaming endpoint packet size */
#define ES_EP_ISO_VIDEO_PKT_SIZE_L  	(uint8_t)(ES_EP_ISO_VIDEO_PKT_SIZE & 0x00FF)

/* High byte - UVC video streaming endpoint packet size and No. of ISO packets */
#define ES_EP_ISO_VIDEO_PKT_SIZE_H  	(uint8_t)(((ES_EP_ISO_VIDEO_PKT_SIZE & 0xFF00) >> 8)  \
                                                 | ((ES_EP_ISO_VIDEO_PKTS_COUNT-1) << 3))

//foR SUPER SPEED
#define ES_EP_BULK_VIDEO_PKT_SIZE_L		(uint8_t)(ES_EP_BULK_VIDEO_PKT_SIZE & 0x00FF)
#define ES_EP_BULK_VIDEO_PKT_SIZE_H  	(uint8_t)((ES_EP_BULK_VIDEO_PKT_SIZE & 0xFF00) >> 8)


#define ES_UVC_MAX_HEADER           	(12)         /* Maximum number of header bytes in UVC */
#define ES_UVC_HEADER_DEFAULT_BFH   	(0x8C)       /* Default BFH(Bit Field Header) for the UVC Header */

#define ES_UVC_MAX_PROBE_SETTING    		(26)     /* Maximum number of bytes in Probe Control */
#define ES_UVC_MAX_PROBE_SETTING_ALIGNED    (32) 	 /* Maximum number of bytes in Probe Control aligned to 32 byte */
#define ES_UVC_MAX_STILL_PROBE_SETTING 		(11)	 /* Maximum number of bytes in Still Probe Control */


#define ES_UVC_HEADER_FRAME         (0)                    /* Normal frame indication */
#define ES_UVC_HEADER_EOF           (uint8_t)(1 << 1)      /* End of frame indication */
#define ES_UVC_HEADER_FRAME_ID      (uint8_t)(1 << 0)      /* Frame ID toggle bit */

#define ES_USB_UVC_SET_REQ_TYPE     (uint8_t)(0x21)        /* UVC interface SET request type */
#define ES_USB_UVC_GET_REQ_TYPE     (uint8_t)(0xA1)        /* UVC Interface GET request type */
#define ES_USB_UVC_GET_CUR_REQ      (uint8_t)(0x81)        /* UVC GET_CUR request */
#define ES_USB_UVC_SET_CUR_REQ      (uint8_t)(0x01)        /* UVC SET_CUR request */
#define ES_USB_UVC_GET_MIN_REQ      (uint8_t)(0x82)        /* UVC GET_MIN Request */
#define ES_USB_UVC_GET_MAX_REQ      (uint8_t)(0x83)        /* UVC GET_MAX Request */
#define ES_USB_UVC_GET_RES_REQ      (uint8_t)(0x84)        /* UVC GET_RES Request */
#define ES_USB_UVC_GET_INFO_REQ     (uint8_t)(0x86)        /* UVC GET_INFO Request */
#define ES_USB_UVC_GET_DEF_REQ      (uint8_t)(0x87)        /* UVC GET_DEF Request */

#define ES_FRAME_DMADONE_EVENT		(1<<2)
#define ES_EXTERNAL_TRIGGER_EVENT	(1<<3)
#define ES_STILL_IMAGE_EVENT 	   	(1 << 7)

#define ES_UVC_GET_DEF_EVENT        (1 << 0)     /* UVC GET_DEF request event flag */
#define ES_UVC_GET_RES_EVENT        (1 << 1)     /* UVC GET_RES request event flag */
#define ES_UVC_GET_INFO_EVENT       (1 << 2)     /* UVC GET_INFO request event flag */
#define ES_UVC_SET_CUR_EVENT        (1 << 3)     /* UVC SET_CUR request event flag */
#define ES_UVC_GET_CUR_EVENT        (1 << 4)     /* UVC GET_CUR request event flag */
#define ES_UVC_GET_MIN_EVENT        (1 << 5)     /* UVC GET_MIN request event flag */
#define ES_UVC_GET_MAX_EVENT        (1 << 6)     /* UVC GET_MAX request event flag */

#define ES_UVC_STREAM_INTERFACE     (uint8_t)(1)           /* Streaming Interface : Alternate setting 1 */

#define ES_UVC_HEADER_STI			(uint8_t)(1<<5)

/* Extern definitions of the USB Enumeration constant arrays used for the Application */
extern const uint8_t esUSB20DeviceDscr[];
extern const uint8_t esUSB30DeviceDscr[];
extern const uint8_t esUSBDeviceQualDscr[];
extern const uint8_t esUSBFSConfigDscr[];
extern const uint8_t esUSBHSConfigDscr[];
extern const uint8_t esUSBBOSDscr[];
extern const uint8_t esUSBSSConfigDscr[];
extern const uint8_t esUSBStringLangIDDscr[];
extern const uint8_t esUSBManufactureDscr[];
extern const uint8_t esUSBProductDscr[];

CyU3PThread uvcAppThread,uvcCtrlThread;           				/* Thread structure */
CyU3PEvent glDMADoneEvent,glStillImageEvent,glUVC_CtrlEvent;    /* GPIO input event group. */

void esUVCAppThread_Entry (uint32_t input);
void esUVC_Ctrl_Thread_Entry (uint32_t input);
void esUVCApplnDebugInit (void);


#endif /* _INCLUDED_CX3RDKOV5640_H_ */

/*[]*/

