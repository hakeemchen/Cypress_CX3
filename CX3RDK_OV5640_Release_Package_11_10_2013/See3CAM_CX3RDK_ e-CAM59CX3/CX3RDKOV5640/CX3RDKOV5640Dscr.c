/*
 ## e-con Systems USB UVC Stack – See3CAMCX3RDK Platform

 ## source file : CX3RDKOV5640Dscr.c
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

/* This file contains the USB enumeration descriptors for the UVC (in memory) application example.
 * The descriptor arrays must be 32 byte aligned if the D-cache is turned on. If the linker
 * used is not capable of supporting the aligned feature for this, then dynamically allocated
 * buffer must be used, and the descriptor must be loaded into it. */

#include <CX3RDKOV5640.h>

/* Standard device descriptor for USB 3.0 */
const uint8_t esUSB30DeviceDscr[] =
{
    0x12,                           /* Descriptor size */
    CY_U3P_USB_DEVICE_DESCR,        /* Device descriptor type */
    0x00,0x03,                      /* USB 3.0 */
    0xEF,                           /* Device class */
    0x02,                           /* Device Sub-class */
    0x01,                           /* Device protocol */
    0x09,                           /* Maxpacket size for EP0 : 2^9 */
    0x60,0x25,                      /* Vendor ID */
    0x51,0xD0,                      /* Product ID */
    0x00,0x00,                      /* Device release number */
    0x01,                           /* Manufacture string index */
    0x02,                           /* Product string index */
    0x00,                           /* Serial number string index */
    0x01                            /* Number of configurations */
};

/* Standard device descriptor */
const uint8_t esUSB20DeviceDscr[] =
{
    0x12,                           /* Descriptor size */
    CY_U3P_USB_DEVICE_DESCR,        /* Device descriptor type */
    0x00,0x02,                      /* USB 2.0 */
    0xEF,                           /* Device class */
    0x02,                           /* Device sub-class */
    0x01,                           /* Device protocol */
    0x40,                           /* Maxpacket size for EP0 : 64 bytes */
    0x60,0x25,                      /* Vendor ID */
	0x51,0xD0,                      /* Product ID */
    0x00,0x00,                      /* Device release number */
    0x01,                           /* Manufacture string index */
    0x02,                           /* Product string index */
    0x00,                           /* Serial number string index */
    0x01                            /* Number of configurations */
};

/* Binary device object store descriptor */
const uint8_t esUSBBOSDscr[] =
{
    0x05,                           /* Descriptor size */
    CY_U3P_BOS_DESCR,               /* Device descriptor type */
    0x16,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x02,                           /* Number of device capability descriptors */

    /* USB 2.0 Extension */
    0x07,                           /* Descriptor size */
    CY_U3P_DEVICE_CAPB_DESCR,       /* Device capability type descriptor */
    CY_U3P_USB2_EXTN_CAPB_TYPE,     /* USB 2.0 extension capability type */
    0x00,0x00,0x00,0x00,            /* Supported device level features  */

    /* SuperSpeed Device Capability */
    0x0A,                           /* Descriptor size */
    CY_U3P_DEVICE_CAPB_DESCR,       /* Device capability type descriptor */
    CY_U3P_SS_USB_CAPB_TYPE,        /* SuperSpeed device capability type */
    0x00,                           /* Supported device level features  */
    0x0E,0x00,                      /* Speeds supported by the device : SS, HS and FS */
    0x03,                           /* Functionality support */
    0x00,                           /* U1 device exit latency */
    0x00,0x00                       /* U2 device exit latency */
};

/* Standard device qualifier descriptor */
const uint8_t esUSBDeviceQualDscr[] =
{
    0x0A,                           /* descriptor size */
    CY_U3P_USB_DEVQUAL_DESCR,       /* Device qualifier descriptor type */
    0x00,0x02,                      /* USB 2.0 */
    0xEF,                           /* Device class */
    0x02,                           /* Device sub-class */
    0x01,                           /* Device protocol */
    0x40,                           /* Maxpacket size for EP0 : 64 bytes */
    0x01,                           /* Number of configurations */
    0x00                            /* Reserved */
};

/* Standard super speed configuration descriptor */
/* Super Speed Configuration Descriptor */
const uint8_t esUSBSSConfigDscr[] =
{
	/* Configuration Descriptor Type */
	0x09,                           /* Descriptor Size */
	CY_U3P_USB_CONFIG_DESCR,        /* Configuration Descriptor Type */
	0x43,0x01,                      /* Length of this descriptor and all sub descriptors */
	0x02,                           /* Number of interfaces */
	0x01,                           /* Configuration number */
	0x00,                           /* Configuration string index */
	0x80,                           /* Config characteristics - Bus powered */
	0x64,                           /* Max power consumption of device (in 8mA unit) : 400mA */

	/* Interface Association Descriptor */
	0x08,                           /* Descriptor Size */
	ES_INTF_ASSN_DSCR_TYPE,      /* Interface Association Descr Type: 11 */
	0x00,                           /* I/f number of first VideoControl i/f */
	0x02,                           /* Number of Video i/f */
	0x0E,                           /* CC_VIDEO : Video i/f class code */
	0x03,                           /* SC_VIDEO_INTERFACE_COLLECTION : Subclass code */
	0x00,                           /* Protocol : Not used */
	0x00,                           /* String desc index for interface */

	/* Standard Video Control Interface Descriptor */
	0x09,                           /* Descriptor size */
	CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
	0x00,                           /* Interface number */
	0x00,                           /* Alternate setting number */
	0x01,                           /* Number of end points */
	0x0E,                           /* CC_VIDEO : Interface class */
	0x01,                           /* CC_VIDEOCONTROL : Interface sub class */
	0x00,                           /* Interface protocol code */
	0x00,                           /* Interface descriptor string index */

	/* Class specific VC Interface Header Descriptor */
	0x0D,                           /* Descriptor size */
	0x24,                           /* Class Specific I/f Header Descriptor type */
	0x01,                           /* Descriptor Sub type : VC_HEADER */
	0x00,0x01,                      /* Revision of class spec : 1.0 */
	0x50,0x00,                      /* Total Size of class specific descriptors (till Output terminal) */
	0x00,0x6C,0xDC,0x02,            /* Clock frequency : 48MHz(Deprecated) */
	0x01,                           /* Number of streaming interfaces */
	0x01,                           /* Video streaming I/f 1 belongs to VC i/f */

	/* Input (Camera) Terminal Descriptor */
	0x12,                           /* Descriptor size */
	0x24,                           /* Class specific interface desc type */
	0x02,                           /* Input Terminal Descriptor type */
	0x01,                           /* ID of this terminal */
	0x01,0x02,                      /* Camera terminal type */
	0x00,                           /* No association terminal */
	0x00,                           /* String desc index : Not used */
	0x00,0x00,                      /* No optical zoom supported */
	0x00,0x00,                      /* No optical zoom supported */
	0x00,0x00,                      /* No optical zoom supported */
	0x03,                           /* Size of controls field for this terminal : 3 bytes */
	0x2A,0x00,0x02,                 /* No controls supported */

	/* Processing Unit Descriptor */
	0x0C,                           /* Descriptor size */
	0x24,                           /* Class specific interface desc type */
	0x05,                           /* Processing Unit Descriptor type */
	0x02,                           /* ID of this terminal */
	0x01,                           /* Source ID : 1 : Conencted to input terminal */
	0x00,0x00,                      /* Digital multiplier */
	0x03,                           /* Size of controls field for this terminal : 3 bytes */
	0x5F,0x10,0x00,                 /* No controls supported */
	0x00,                           /* String desc index : Not used */

	/* Extension Unit Descriptor */
	0x1C,                           /* Descriptor size */
	0x24,                           /* Class specific interface desc type */
	0x06,                           /* Extension Unit Descriptor type */
	0x03,                           /* ID of this terminal */
	0xFF,0xFF,0xFF,0xFF,            /* 16 byte GUID */
	0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,
	0x00,                           /* Number of controls in this terminal */
	0x01,                           /* Number of input pins in this terminal */
	0x02,                           /* Source ID : 2 : Connected to Proc Unit */
	0x03,                           /* Size of controls field for this terminal : 3 bytes */
	0x00,0x00,0x00,                 /* No controls supported */
	0x00,                           /* String desc index : Not used */

	/* Output Terminal Descriptor */
	0x09,                           /* Descriptor size */
	0x24,                           /* Class specific interface desc type */
	0x03,                           /* Output Terminal Descriptor type */
	0x04,                           /* ID of this terminal */
	0x01,0x01,                      /* USB Streaming terminal type */
	0x00,                           /* No association terminal */
	0x03,                           /* Source ID : 3 : Connected to Extn Unit */
	0x00,                           /* String desc index : Not used */

	/* Video Control Status Interrupt Endpoint Descriptor */
	0x07,                           /* Descriptor size */
	CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
	ES_EP_CONTROL_STATUS,        /* Endpoint address and description */
	CY_U3P_USB_EP_INTR,             /* Interrupt End point Type */
	0x00,0x04,                      /* Max packet size = 1024 bytes */
	0x01,                           /* Servicing interval */

	/* Super Speed Endpoint Companion Descriptor */
	0x06,                           /* Descriptor size */
	CY_U3P_SS_EP_COMPN_DESCR,       /* SS Endpoint Companion Descriptor Type */
	0x00,                           /* Max no. of packets in a Burst : 1 */
	0x00,                           /* Attribute: N.A. */
	0x00,                           /* Bytes per interval:1024 */
	0x04,

	/* Class Specific Interrupt Endpoint Descriptor */
	0x05,                           /* Descriptor size */
	0x25,                           /* Class Specific Endpoint Descriptor Type */
	CY_U3P_USB_EP_INTR,             /* End point Sub Type */
	0x40,0x00,                      /* Max packet size = 64 bytes */

	/* Standard Video Streaming Interface Descriptor (Alternate Setting 0) */
	0x09,                           /* Descriptor size */
	CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
	0x01,                           /* Interface number */
	0x00,                           /* Alternate setting number */
	0x01,                           /* Number of end points */
	0x0E,                           /* Interface class : CC_VIDEO */
	0x02,                           /* Interface sub class : CC_VIDEOSTREAMING */
	0x00,                           /* Interface protocol code : Undefined */
	0x00,                           /* Interface descriptor string index */

   /* Class-specific Video Streaming Input Header Descriptor */
	0x0E,                           /* Descriptor size */
	0x24,                           /* Class-specific VS I/f Type */
	0x01,                           /* Descriptotor Subtype : Input Header */
	0x01,                           /* 1 format desciptor follows */
	0xB1,0x00,                      /* Total size of Class specific VS descr */
	ES_EP_UVC_VIDEO,             /* EP address for BULK video data */
	0x00,                           /* No dynamic format change supported */
	0x04,                           /* Output terminal ID : 4 */
	0x02,                           /* Still image capture method 1 supported */
	0x01,                           /* Hardware trigger NOT supported */
	0x01,                           /* Hardware to initiate still image capture NOT supported */
	0x01,                           /* Size of controls field : 1 byte */
	0x00,                           /* D2 : Compression quality supported */

   /* Class specific Uncompressed VS format descriptor */
	0x1B,  							/* Descriptor size */
	0x24,  							/* Class-specific VS I/f Type */
	0x04,  							/* Subtype : uncompressed format I/F */
	0x01,  							/* Format desciptor index */
	0x04,  							/* number of frame descriptor followed */
	/* GUID, globally unique identifier used to identify streaming-encoding format: YUY2  */
	0x59,0x55,0x59,0x32,
	0x00,0x00,0x10,0x00,
	0x80,0x00,0x00,0xAA,
	0x00,0x38,0x9B,0x71,
	0x10,  							/* Number of bits per pixel */
	0x01,  							/* Optimum Frame Index for this stream: 1 */
	0x00, 							/* X dimension of the picture aspect ratio; Non-interlaced */
	0x00, 							/* Y dimension of the pictuer aspect ratio: Non-interlaced */
	0x00,  							/* Interlace Flags: Progressive scanning, no interlace */
	0x00,  							/* duplication of the video stream restriction: 0 - no restriction */

	/* Class specific Uncompressed VS frame descriptor */
	0x1E,  							/* Descriptor size */
	0x24,  							/* Descriptor type*/
	0x05,  							/* Subtype: uncompressed frame I/F */
	0x01,  							/* Frame Descriptor Index */
	0x00,  							/* Still image capture method 1 supported, fixed frame rate */
	0x80,0x07, 						/* Width in pixel */	//1920
	0x38,0x04, 						/* Height in pixel */	//1080
	0x00,0x00,0x08,0xCA,            /* Min bit rate bits/s */
	0x00,0x00,0x08,0xCA,            /* Min bit rate bits/s */
	0x00,0x48,0x3F,0x00,  			/* Maximum video or still frame size in bytes(Deprecated)*/
	0x15,0x16,0x05,0x00,            /* Default frame interval */
    0x01,                           /* Frame interval type : No of discrete intervals */
    0x15,0x16,0x05,0x00,            /* Frame interval 3 */

	/* Class specific Uncompressed VS frame descriptor */
	0x1E,  							/* Descriptor size */
	0x24,  							/* Descriptor type*/
	0x05,  							/* Subtype: uncompressed frame I/F */
	0x02,  							/* Frame Descriptor Index */
	0x00,  							/* Still image capture method 1 supported, fixed frame rate */
	0x80,0x02, 						/* Width in pixel */	//VGA
	0xE0,0x01, 						/* Height in pixel */
	0x00,0x00,0x08,0xCA,            /* Min bit rate bits/s */
	0x00,0x00,0x08,0xCA,            /* Min bit rate bits/s */
	0x00,0x60,0x09,0x00,  			/* Maximum video or still frame size in bytes(Deprecated)*/
    0x0A,0x8B,0x02,0x00,            /* Default frame interval */
    0x01,                           /* Frame interval type : No of discrete intervals */
    0x0A,0x8B,0x02,0x00,            /* Frame interval 3 */

	/* Class specific Uncompressed VS frame descriptor */
	0x1E,  							/* Descriptor size */
	0x24,  							/* Descriptor type*/
	0x05,  							/* Subtype: uncompressed frame I/F */
	0x03,  							/* Frame Descriptor Index */
	0x00,  							/* Still image capture method 1 supported, fixed frame rate */
	0x00,0x05, 						/* Width in pixel */	//1280
	0xD0,0x02, 						/* Height in pixel */	//720
	0x00,0x00,0x08,0xCA,            /* Min bit rate bits/s */
	0x00,0x00,0x08,0xCA,            /* Min bit rate bits/s */
	0x00,0x20,0x1C,0x00,  			/* Maximum video or still frame size in bytes(Deprecated)*/
	0x0A,0x8B,0x02,0x00,            /* Default frame interval */
    0x01,                           /* Frame interval type : No of discrete intervals */
    0x0A,0x8B,0x02,0x00,            /* Frame interval 3 */

	/* Class specific Uncompressed VS frame descriptor */
	0x1E,  							/* Descriptor size */
	0x24,  							/* Descriptor type*/
	0x05,  							/* Subtype: uncompressed frame I/F */
	0x04,  							/* Frame Descriptor Index */
	0x00,  							/* Still image capture method 1 supported, fixed frame rate */
	0x20,0x0A, 						/* Width in pixel */	//2592
	0x98,0x07, 						/* Height in pixel */	//1944
	0x00,0x00,0x08,0xCA,            /* Min bit rate bits/s */
	0x00,0x00,0x08,0xCA,            /* Min bit rate bits/s */
	0x00,0xC6,0x99,0x00,  			/* Maximum video or still frame size in bytes(Deprecated)*/
	0x2A,0x2C,0x0A,0x00,            /* Default frame interval */
    0x01,                           /* Frame interval type : No of discrete intervals */
    0x2A,0x2C,0x0A,0x00,            /* Frame interval 3 */


    /* Still image descriptor -YUV with QVGA resolution */
	0x0A,
	0x24,
	0x03,
	0x00,
	0x01,					//No of frame Resolutions Follows
	0x20,0x0A,				//2592x1944
	0x98,0x07,
	0x00,

	/* Color matching descriptor */
	0x06,
	0x24,
	0x0D,
	0x00,
	0x00,
	0x00,

	/* Endpoint Descriptor for BULK Streaming Video Data */
	0x07,                           /* Descriptor size */
	CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
	ES_EP_UVC_VIDEO,             	/* Endpoint address and description */
	CY_U3P_USB_EP_BULK,             /* BULK End point */
	ES_EP_BULK_VIDEO_PKT_SIZE_L,  	/* BULK endpoint size lower byte */
	ES_EP_BULK_VIDEO_PKT_SIZE_H,  	/* BULK endpoint size higher byte*/
	0x01,                           /* Servicing interval for data transfers */

	/* Super Speed Endpoint Companion Descriptor */
	0x06,                           /* Descriptor size */
	CY_U3P_SS_EP_COMPN_DESCR,       /* SS Endpoint Companion Descriptor Type */
	0x0B,                           /* Max number of packets per burst: 16 */
	0x00,                           /* Attribute: Streams not defined */
	0x00,                           /* No meaning for bulk */
	0x00,

};

/* Standard High Speed Configuration Descriptor */
const uint8_t esUSBHSConfigDscr[] =
{
    /* Configuration descriptor */
    0x09,                           /* Descriptor size */
    CY_U3P_USB_CONFIG_DESCR,        /* Configuration descriptor type */
    0xDC,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x02,                           /* Number of interfaces */
    0x01,                           /* Configuration number */
    0x00,                           /* COnfiguration string index */
    0x80,                           /* Config characteristics - bus powered */
    0xFA,                           /* Max power consumption of device (in 2mA unit) : 500mA */

    /* Interface association descriptor */
    0x08,                           /* Descriptor size */
    ES_INTF_ASSN_DSCR_TYPE,      	/* Interface association descr type */
    0x00,                           /* I/f number of first video control i/f */
    0x02,                           /* Number of video streaming i/f */
    0x0E,                           /* CC_VIDEO : Video i/f class code */
    0x03,                           /* SC_VIDEO_INTERFACE_COLLECTION : subclass code */
    0x00,                           /* Protocol : not used */
    0x00,                           /* String desc index for interface */

    /* Standard video control interface descriptor */
    0x09,                           /* Descriptor size */
    CY_U3P_USB_INTRFC_DESCR,        /* Interface descriptor type */
    0x00,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x01,                           /* Number of end points */
    0x0E,                           /* CC_VIDEO : Interface class */
    0x01,                           /* CC_VIDEOCONTROL : Interface sub class */
    0x00,                           /* Interface protocol code */
    0x00,                           /* Interface descriptor string index */

    /* Class specific VC interface header descriptor */
    0x0D,                           /* Descriptor size */
    0x24,                           /* Class Specific I/f header descriptor type */
    0x01,                           /* Descriptor sub type : VC_HEADER */
    0x00,0x01,                      /* Revision of class spec : 1.0 */
    0x50,0x00,                      /* Total size of class specific descriptors (till output terminal) */
    0x00,0x6C,0xDC,0x02,            /* Clock frequency : 48MHz */
    0x01,                           /* Number of streaming interfaces */
    0x01,                           /* Video streaming I/f 1 belongs to VC i/f */

    /* Input (camera) terminal descriptor */
    0x12,                           /* Descriptor size */
    0x24,                           /* Class specific interface desc type */
    0x02,                           /* Input Terminal Descriptor type */
    0x01,                           /* ID of this terminal */
    0x01,0x02,                      /* Camera terminal type */
    0x00,                           /* No association terminal */
    0x00,                           /* String desc index : not used */
    0x00,0x00,                      /* No optical zoom supported */
    0x00,0x00,                      /* No optical zoom supported */
    0x00,0x00,                      /* No optical zoom supported */
    0x03,                           /* Size of controls field for this terminal : 3 bytes */
    0x2A,0x00,0x02,                 /* No controls supported */

    /* Processing unit descriptor */
    0x0C,                           /* Descriptor size */
    0x24,                           /* Class specific interface desc type */
    0x05,                           /* Processing unit descriptor type */
    0x02,                           /* ID of this terminal */
    0x01,                           /* Source ID : 1 : conencted to input terminal */
    0x00,0x00,                      /* Digital multiplier */
    0x03,                           /* Size of controls field for this terminal : 3 bytes */
    0x5F,0x10,0x00,                 /* No controls supported */
    0x00,                           /* String desc index : not used */

    /* Extension unit descriptor */
    0x1C,                           /* Descriptor size */
    0x24,                           /* Class specific interface desc type */
    0x06,                           /* Extension unit descriptor type */
    0x03,                           /* ID of this terminal */
    0xFF,0xFF,0xFF,0xFF,            /* 16 byte GUID */
    0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,
    0x00,                           /* Number of controls in this terminal */
    0x01,                           /* Number of input pins in this terminal */
    0x02,                           /* Source ID : 2 : connected to proc unit */
    0x03,                           /* Size of controls field for this terminal : 3 bytes */
    0x00,0x00,0x00,                 /* No controls supported */
    0x00,                           /* String desc index : not used */

    /* Output terminal descriptor */
    0x09,                           /* Descriptor size */
    0x24,                           /* Class specific interface desc type */
    0x03,                           /* Output terminal descriptor type */
    0x04,                           /* ID of this terminal */
    0x01,0x01,                      /* USB Streaming terminal type */
    0x00,                           /* No association terminal */
    0x03,                           /* Source ID : 3 : connected to extn unit */
    0x00,                           /* String desc index : not used */

    /* Video control status interrupt endpoint descriptor */
    0x07,                           /* Descriptor size */
    CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint descriptor type */
    ES_EP_CONTROL_STATUS,        /* Endpoint address and description */
    CY_U3P_USB_EP_INTR,             /* Interrupt end point type */
    0x40,0x00,                      /* Max packet size = 64 bytes */
    0x08,                           /* Servicing interval : 8ms */

    /* Class specific interrupt endpoint descriptor */
    0x05,                           /* Descriptor size */
    0x25,                           /* Class specific endpoint descriptor type */
    CY_U3P_USB_EP_INTR,             /* End point sub type */
    0x40,0x00,                      /* Max packet size = 64 bytes */

    /* Standard video streaming interface descriptor (alternate setting 0) */
    0x09,                           /* Descriptor size */
    CY_U3P_USB_INTRFC_DESCR,        /* Interface descriptor type */
    0x01,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x00,                           /* Number of end points : zero bandwidth */
    0x0E,                           /* Interface class : CC_VIDEO */
    0x02,                           /* Interface sub class : CC_VIDEOSTREAMING */
    0x00,                           /* Interface protocol code : undefined */
    0x00,                           /* Interface descriptor string index */

    /* Class-specific video streaming input header descriptor */
    0x0E,                           /* Descriptor size */
    0x24,                           /* Class-specific VS i/f type */
    0x01,                           /* Descriptotor subtype : input header */
    0x01,                           /* 1 format desciptor follows */
    0x4D,0x00,                      /* Total size of class specific VS descr */
    ES_EP_UVC_VIDEO,             /* EP address for ISO video data */
    0x01,                           /* No dynamic format change supported */
    0x04,                           /* Output terminal ID : 4 */
    0x02,                           /* Still image capture method 1 supported */
    0x01,                           /* Hardware trigger supported for still image */
    0x01,                           /* Hardware to initiate still image capture */
    0x01,                           /* Size of controls field : 1 byte */
    0x00,                           /* D2 : Compression quality supported */

    /* Class specific VS format descriptor */
    0x1B,                           /* Descriptor size */
    0x24,                           /* Class-specific VS i/f type */
    0x04,                           /* Descriptotor subtype : VS_FORMAT_MJPEG */
    0x01,                           /* Format desciptor index */
    0x01,                           /* 1 Frame desciptor follows */
	0x59, 0x55, 0x59, 0x32,			/* guid */
	0x00, 0x00,
	0x10, 0x00,
	0x80, 0x00,
	0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71,
	0x10,                           /* bBitsPerPixel*/
    0x01,                           /* Default frame index is 1 */
    0x00,                           /* bAspectRatioX */
    0x00,                           /* bAspectRatioY */
    0x00,                           /* bmInterlaceFlags */
    0x00,                           /* bCopyProtect */

	/* Class specific Uncompressed VS frame descriptor */
	0x1E,  							/* Descriptor size */
	0x24,  							/* Descriptor type*/
	0x05,  							/* Subtype: uncompressed frame I/F */
	0x01,  							/* Frame Descriptor Index */
	0x00,  							/* Still image capture method 1 supported, fixed frame rate */
	0x80,0x02, 						/* Width in pixel */
	0xE0,0x01, 						/* Height in pixel */
	0x00,0x00,0x08,0xCA,            /* Min bit rate bits/s */
	0x00,0x00,0x08,0xCA,            /* Min bit rate bits/s */
	0x00,0x60,0x09,0x00,  			/* Maximum video or still frame size in bytes(Deprecated)*/
	0x15,0x16,0x05,0x00,            /* Default frame interval */
    0x01,                           /* Frame interval type : No of discrete intervals */
    0x15,0x16,0x05,0x00,            /* Frame interval 3 */

	/* Color matching descriptor */
	0x06,
	0x24,
	0x0D,
	0x00,
	0x00,
	0x00,

    /* Standard video streaming interface descriptor (Alternate Setting 1) */
    0x09,                           /* Descriptor size */
    CY_U3P_USB_INTRFC_DESCR,        /* Interface descriptor type */
    0x01,                           /* Interface number */
    0x01,                           /* Alternate setting number */
    0x01,                           /* Number of end points : 1 ISO EP */
    0x0E,                           /* Interface class : CC_VIDEO */
    0x02,                           /* Interface sub class : CC_VIDEOSTREAMING */
    0x00,                           /* Interface protocol code : Undefined */
    0x00,                           /* Interface descriptor string index */

    /* Endpoint descriptor for ISO streaming video data */
    0x07,                           /* Descriptor size */
    CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint descriptor type */
    ES_EP_UVC_VIDEO,             /* Endpoint address and description */
    0x05,                           /* ISO End point : Async */
    ES_EP_ISO_VIDEO_PKT_SIZE_L,  /* 1 transaction per microframe */
    ES_EP_ISO_VIDEO_PKT_SIZE_H,  /* ES_EP_ISO_VIDEO_PKT_SIZE max bytes */
    0x01,                            /* Servicing interval for data transfers */
};

/* Standard full speed configuration descriptor : full speed is not supported. */
const uint8_t esUSBFSConfigDscr[] =
{
    /* Configuration descriptor */
    0x09,                           /* Descriptor size */
    CY_U3P_USB_CONFIG_DESCR,        /* Configuration descriptor type */
    0x09,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x00,                           /* Number of interfaces */
    0x01,                           /* Configuration number */
    0x00,                           /* COnfiguration string index */
    0x80,                           /* Config characteristics - bus powered */
    0x32,                           /* Max power consumption of device (in 2mA unit) : 100mA */
};

/* Standard language ID string descriptor */
const uint8_t esUSBStringLangIDDscr[] =
{
    0x04,                           /* Descriptor size */
    CY_U3P_USB_STRING_DESCR,        /* Device descriptor type */
    0x09,0x04                       /* Language ID supported */
};

/* Standard manufacturer string descriptor */
const uint8_t esUSBManufactureDscr[] =
{
	0x1A,                           /* Descriptor size */
	CY_U3P_USB_STRING_DESCR,        /* Device descriptor type */
	'e',0x00,
	'-',0x00,
	'c',0x00,
	'o',0x00,
	'n',0x00,
	's',0x00,
	'y',0x00,
	's',0x00,
	't',0x00,
	'e',0x00,
	'm',0x00,
	's',0x00
};

/* Standard product string descriptor */
const uint8_t esUSBProductDscr[] =
{
	0x38,                           /* Descriptor Size */
	CY_U3P_USB_STRING_DESCR,        /* Device descriptor type */
	'e',0x00,
	'-',0x00,
	'c',0x00,
	'o',0x00,
	'n',0x00,
	'\'',0x00,
	's',0x00,
	' ',0x00,
	'C',0x00,
	'X',0x00,
	'3',0x00,
	' ',0x00,
	'R',0x00,
	'D',0x00,
	'K',0x00,
	' ',0x00,
	'w',0x00,
	'i',0x00,
	't',0x00,
	'h',0x00,
	' ',0x00,
	'O',0x00,
	'V',0x00,
	'5',0x00,
	'6',0x00,
	'4',0x00,
	'0',0x00,
};

/* [ ] */

