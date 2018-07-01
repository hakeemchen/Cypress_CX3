/*
 ## e-con Systems USB UVC Stack – See3CAMCX3RDK Platform

 ## source file : CX3RDKOV5640.c
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

#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3usb.h"
#include "cyu3spi.h"
#include "cyu3i2c.h"
#include "cyu3uart.h"
#include "cyu3gpio.h"
#include "cyu3utils.h"
#include "cyu3pib.h"
#include "cyu3socket.h"
#include "sock_regs.h"

#include "cyu3mipicsi.h"
#include "CX3OV5640Lib.h"
#include "CX3RDKOV5640.h"

CyU3PUSBSpeed_t glUSBSpeed = 0x00;
uint32_t glDMATxCount = 0,glDmaDone = 0;
CyU3PDmaMultiChannel glChHandleUVCStream;    /* DMA Channel Handle  */
uint8_t glprevprobe=0,gStillReq = 0,frame_index=0,Still_frame_index=0,g_IsAutoFocus=1;
CyBool_t gHitFV = CyFalse,glstillflag=CyFalse;
CyBool_t glIsApplnActive = CyFalse;     /* Whether the loopback application is active or not. */

/* UVC Header */
uint8_t glUVCHeader[ES_UVC_MAX_HEADER] =
{
    0x0C,                           /* Header Length */
    0x8C,                           /* Bit field header field */
    0x00,0x00,0x00,0x00,            /* Presentation time stamp field */
    0x00,0x00,0x00,0x00,0x00,0x00   /* Source clock reference field */
};
/* UVC Probe Control Setting */
uint8_t glProbeCtrl[ES_UVC_MAX_PROBE_SETTING] = {
		0x00,0x00,                       /* bmHint : No fixed parameters */
		0x01,                            /* Use 1st Video format index */
		0x01,                            /* Use 1st Video frame index */
		0x0A,0x8B,0x02,0x00,             /* Desired frame interval in 100ns */
		0x00,0x00,                       /* Key frame rate in key frame/video frame units */
		0x00,0x00,                       /* PFrame rate in PFrame / key frame units */
		0x00,0x00,                       /* Compression quality control */
		0x00,0x00,                       /* Window size for average bit rate */
		0x00,0x00,                       /* Internal video streaming i/f latency in ms */
		0x00,0x60,0x09,0x00,             /* Max video frame size in bytes (100KB) */
		0x00,0x30,0x00,0x00              /* No. of bytes device can rx in single payload */
};
/* Video Probe Commit Control */
uint8_t glCommitCtrl[ES_UVC_MAX_PROBE_SETTING_ALIGNED];

/* Still Probe Control Setting */
uint8_t glStillProbeCtrl[ES_UVC_MAX_STILL_PROBE_SETTING] =
{
    0x01,                            /* Use 1st Video format index */
    0x01,                            /* Use 1st Video frame index */
    0x00,							 /* Compression quality */
    0x00,0xC6,0x99,0x00,             /* Max video frame size in bytes (100KB) */
    0x00,0x0c,0x00,0x00              /* No. of bytes device can rx in single payload */
};

uint8_t gStillCommitCtrl[ES_UVC_MAX_PROBE_SETTING_ALIGNED];

struct UVC_CTL
{
	uint8_t		bRequest;
	uint16_t	wValue;
	uint16_t	wIndex;
	uint16_t	wLength;
}uvc_ctrl;

/* Configuration parameters for VGA @60FPS for the OV5640 sensor */
CyU3PMipicsiCfg_t esUvcVGA60 =  { CY_U3P_CSI_DF_YUV422_8_2, 1, 0, 0x4, 0x06F,
	CY_U3P_CSI_PLL_FRS_250_500M, CY_U3P_CSI_PLL_CLK_DIV_2,
    CY_U3P_CSI_PLL_CLK_DIV_8, CY_U3P_CSI_PLL_CLK_DIV_8, 640};

/* Configuration parameters for 720p @30FPS for the OV5640 sensor */
CyU3PMipicsiCfg_t esUvc720p60 =  { CY_U3P_CSI_DF_YUV422_8_2, 2, 0, 0x1, 0x03D,
	CY_U3P_CSI_PLL_FRS_250_500M, CY_U3P_CSI_PLL_CLK_DIV_4,
    CY_U3P_CSI_PLL_CLK_DIV_8, CY_U3P_CSI_PLL_CLK_DIV_4, 1280};

/* Configuration parameters for 1080p @30FPS for the OV5640 sensor */
CyU3PMipicsiCfg_t esUvc1080p30 =  { CY_U3P_CSI_DF_YUV422_8_2, 2, 0, 0x1, 0x03D,
	CY_U3P_CSI_PLL_FRS_250_500M, CY_U3P_CSI_PLL_CLK_DIV_4,
    CY_U3P_CSI_PLL_CLK_DIV_8, CY_U3P_CSI_PLL_CLK_DIV_4, 1920};

/* Configuration parameters for 5mp @15FPS for the OV5640 sensor */
CyU3PMipicsiCfg_t esUvc5MP15 =  { CY_U3P_CSI_DF_YUV422_8_2, 2, 0, 0x1, 0x040,
	CY_U3P_CSI_PLL_FRS_250_500M, CY_U3P_CSI_PLL_CLK_DIV_4,
    CY_U3P_CSI_PLL_CLK_DIV_8, CY_U3P_CSI_PLL_CLK_DIV_4, 2592};

void esGpifCB (CyU3PGpifEventType event,uint8_t currentState);

/* Application error handler */
void
esAppErrorHandler (
        CyU3PReturnStatus_t apiRetStatus    /* API return status */
        )
{
    /* Application failed with the error code apiRetStatus */

    /* Add custom debug or recovery actions here */

    /* Loop indefinitely */
    for (;;)
    {
        /* Thread sleep : 100 ms */
        CyU3PThreadSleep (100);
    }
}
/* UVC header addition function */
static void
esUVCAddHeader (
        uint8_t *buffer_p, /* Buffer pointer */
        uint8_t frameInd   /* EOF or normal frame indication */
    )
{
    /* Copy header to buffer */
    CyU3PMemCopy (buffer_p, (uint8_t *)glUVCHeader, ES_UVC_MAX_HEADER);

    /* Check if last packet of the frame. */
    if (frameInd == ES_UVC_HEADER_EOF)
    {
    	/* Modify UVC header to toggle Frame ID */
        glUVCHeader[1] ^= ES_UVC_HEADER_FRAME_ID;
        /* Indicate End of Frame in the buffer */
        buffer_p[1] |=  ES_UVC_HEADER_EOF;
    }
}
/* This function initializes the debug module for the UVC application */
void
esUVCApplnDebugInit (void)
{
    CyU3PUartConfig_t uartConfig;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Initialize the UART for printing debug messages */
    apiRetStatus = CyU3PUartInit();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        esAppErrorHandler(apiRetStatus);
    }

    /* Set UART Configuration */
    uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
    uartConfig.stopBit = CY_U3P_UART_ONE_STOP_BIT;
    uartConfig.parity = CY_U3P_UART_NO_PARITY;
    uartConfig.txEnable = CyTrue;
    uartConfig.rxEnable = CyFalse;
    uartConfig.flowCtrl = CyFalse;
    uartConfig.isDma = CyTrue;

    /* Set the UART configuration */
    apiRetStatus = CyU3PUartSetConfig (&uartConfig, NULL);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        esAppErrorHandler(apiRetStatus);
    }

    /* Set the UART transfer */
    apiRetStatus = CyU3PUartTxSetBlockXfer (0xFFFFFFFF);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        esAppErrorHandler(apiRetStatus);
    }

    /* Initialize the debug application */
    apiRetStatus = CyU3PDebugInit (CY_U3P_LPP_SOCKET_UART_CONS, 8);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        esAppErrorHandler(apiRetStatus);
    }
}

/* DMA callback function to handle the produce events for P to U transfers. */
void
esSlFifoPtoUDmaCallback (
		CyU3PDmaMultiChannel   *chHandle,
        CyU3PDmaCbType_t  type,
        CyU3PDmaCBInput_t *input
        )
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PDmaBuffer_t Dmabuffer;
    if (type == CY_U3P_DMA_CB_PROD_EVENT)
    {
    	status = CyU3PDmaMultiChannelGetBuffer(chHandle,&Dmabuffer,CYU3P_NO_WAIT);
    	if(status != CY_U3P_SUCCESS)
    		CyU3PDebugPrint (4, "\rGetBuffer failed= 0x%x\r\n", status);
    	else
    	{
			if(gHitFV == CyTrue)
				esUVCAddHeader ((input->buffer_p.buffer - 12), ES_UVC_HEADER_EOF);
			else
				esUVCAddHeader ((input->buffer_p.buffer - 12), ES_UVC_HEADER_FRAME);

			status = CyU3PDmaMultiChannelCommitBuffer (chHandle, (input->buffer_p.count + 12), 0);
			if (status != CY_U3P_SUCCESS)
			{
				CyU3PDebugPrint (4, "\rCommitBuffer failed= 0x%x DmaDone=%d\r\n", status,glDmaDone);
				CyU3PDmaMultiChannelDiscardBuffer(chHandle);
			}
			else
			{
				glDMATxCount++;
				glDmaDone++;
			}
    	}
    }
    else if((type == CY_U3P_DMA_CB_CONS_EVENT))
    {
    	glDmaDone--;
    	if((gHitFV == CyTrue) && (glDmaDone == 0))
			CyU3PEventSet(&glDMADoneEvent, ES_FRAME_DMADONE_EVENT,CYU3P_EVENT_OR);
    }
}

/* This function starts the video streaming application. It is called
 * when there is a SET_INTERFACE event for alternate interface 1. */
CyU3PReturnStatus_t
esUVCApplnStart (void)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    uint8_t SMState = 0;

	apiRetStatus = CyU3PDmaMultiChannelReset (&glChHandleUVCStream);
	if (apiRetStatus != CY_U3P_SUCCESS)
		CyU3PDebugPrint (4,"\rDMA Channel Reset Failed or No streaming in progress, Error Code = %d\r\n",apiRetStatus);

	apiRetStatus = CyU3PDmaMultiChannelSetXfer (&glChHandleUVCStream, 0, 0);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint (4, "\rCyU3PDmaChannelSetXfer failed, error code = %d\r\n", apiRetStatus);
		return apiRetStatus;
	}

	if(frame_index == 1)//1080p
	{
		if(glUSBSpeed == 0x03)
		{
			apiRetStatus = CyU3PMipicsiSetIntfParams (&esUvc1080p30, CyFalse);
			if (apiRetStatus != CY_U3P_SUCCESS)
				CyU3PDebugPrint (4, "\rCyCx3UvcApplnUSBSetupCB:SetIntfParams failed, Error = %d\r\n", apiRetStatus);
			esOV5640_1080P_config();
		}
		else if((glUSBSpeed == 0x02))
		{
				apiRetStatus = CyU3PMipicsiSetIntfParams (&esUvcVGA60, CyFalse);
				if (apiRetStatus != CY_U3P_SUCCESS)
					CyU3PDebugPrint (4, "\rCyCx3UvcApplnUSBSetupCB:SetIntfParams failed, Error = %d\r\n", apiRetStatus);

				esOV5640_VGA_config();
				esOV5640_VGA_HS_config();
		}
	}
	else if(frame_index == 2 )//Vga
	{
		apiRetStatus = CyU3PMipicsiSetIntfParams (&esUvcVGA60, CyFalse);
		if (apiRetStatus != CY_U3P_SUCCESS)
			CyU3PDebugPrint (4, "\rCyCx3UvcApplnUSBSetupCB:SetIntfParams failed, Error = %d\r\n", apiRetStatus);

		esOV5640_VGA_config();
	}
	else if(frame_index == 3 )//720P
	{
		apiRetStatus = CyU3PMipicsiSetIntfParams (&esUvc720p60, CyFalse);
		if (apiRetStatus != CY_U3P_SUCCESS)
			CyU3PDebugPrint (4, "\rCyCx3UvcApplnUSBSetupCB:SetIntfParams failed, Error = %d\r\n", apiRetStatus);
		esOV5640_720P_config();
	}
	else if(frame_index == 4)//5MP
	{
		apiRetStatus = CyU3PMipicsiSetIntfParams (&esUvc5MP15, CyFalse);
		if (apiRetStatus != CY_U3P_SUCCESS)
			CyU3PDebugPrint (4, "\rCyCx3UvcApplnUSBSetupCB:SetIntfParams failed, Error = %d\r\n", apiRetStatus);
		esOV5640_5MP_config();
	}

    CyU3PGpifSMControl(CyFalse);
    CyU3PThreadSleep(10);
    CyU3PGpifSMSwitch(257,0,257,0,2);
    CyU3PGpifGetSMState(&SMState);

   	CyU3PMipicsiWakeup();
    esCamera_Power_Up();

	/* Update the flag so that the application thread is notified of this. */
	glIsApplnActive = CyTrue;

	if(glstillflag!= CyTrue)
	{
		if(g_IsAutoFocus)
			esOV5640_SetAutofocus(g_IsAutoFocus);
	}
	return CY_U3P_SUCCESS;
}

/* This function stops the video streaming. It is called from the USB event
 * handler, when there is a reset / disconnect or SET_INTERFACE for alternate
 * interface 0. */
void
esUVCApplnStop (void)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Update the flag so that the application thread is notified of this. */
    glIsApplnActive = CyFalse;

    CyU3PMipicsiSleep();

    esCamera_Power_Down();
    CyU3PGpifSMControl(CyTrue);

    CyU3PThreadSleep(50);

    // Reset the channel: Set to DSCR chain starting point in PORD/CONS SCKT; set DSCR_SIZE field in DSCR memory
	apiRetStatus = CyU3PDmaMultiChannelReset(&glChHandleUVCStream);
	if (apiRetStatus != CY_U3P_SUCCESS)
		CyU3PDebugPrint (4,"\rDMA Channel Reset Failed, Error Code = %d\r\n",apiRetStatus);

	CyU3PThreadSleep(50);

    /* Flush the endpoint memory */
    CyU3PUsbFlushEp(ES_EP_UVC_VIDEO);
    glDMATxCount = 0;
	glDmaDone = 0;
	gHitFV = CyFalse;
}

/* This is the Callback function to handle the USB Events */
static void
esUVCApplnUSBEventCB (
    CyU3PUsbEventType_t evtype, /* Event type */
    uint16_t            evdata  /* Event data */
    )
{
    uint8_t interface = 0, altSetting = 0;

    switch (evtype)
    {
        case CY_U3P_USB_EVENT_SETINTF:
             /* Start the video streamer application if theinterface requested was 1. If not, stop the streamer. */
            interface = CY_U3P_GET_MSB(evdata);
            altSetting = CY_U3P_GET_LSB(evdata);

            if ((altSetting == ES_UVC_STREAM_INTERFACE) && (interface == 1))
            {
                /* Stop the application before re-starting. */
                if (glIsApplnActive)
                {
                    esUVCApplnStop ();
                }
                esUVCApplnStart ();
            }
            else
            {
            	if (glIsApplnActive)
				{
					esUVCApplnStop ();
				}
            }
            /* Fall-through. */
            break;

        case CY_U3P_USB_EVENT_SETCONF:
        case CY_U3P_USB_EVENT_RESET:
        	/* Stop the video streamer application. */
			if (glIsApplnActive)
			{
				esUVCApplnStop ();
				CyU3PThreadSleep(2000);
				esUVCApplnStart();
			}
			break;
        case CY_U3P_USB_EVENT_DISCONNECT:
            /* Stop the video streamer application. */
            if (glIsApplnActive)
            {
                esUVCApplnStop ();
            }
            break;

        default:
            break;
    }
}

/* Callback to handle the USB Setup Requests and UVC Class events */
static CyBool_t
esUVCApplnUSBSetupCB (
        uint32_t setupdat0, /* SETUP Data 0 */
        uint32_t setupdat1  /* SETUP Data 1 */
    )
{
	uint8_t  bRequest, bType,bRType;
	uint16_t wValue, wIndex, wLength;
    uint16_t readCount = 0;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t eventFlag;
	uint8_t glimageskip=0;

    /* Decode the fields from the setup request. */
	bType    = (setupdat0 & CY_U3P_USB_TYPE_MASK);
	bRType = (setupdat0 & (0x000000FF));
	bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
	wValue = (setupdat0 & (0xFFFF0000)) >> 16;
	wIndex = setupdat1 & (0x0000FFFF);
	wLength = (setupdat1 & (0xFFFF0000)) >>16;

	//CyU3PDebugPrint(4, "\rbRType = 0x%x, bRequest = 0x%x, wValue = 0x%x, wIndex = 0x%x, wLength= 0x%x\r\n",bRType, bRequest, wValue, wIndex, wLength);

	uvc_ctrl.bRequest 	= bRequest;
	uvc_ctrl.wIndex		= wIndex;
	uvc_ctrl.wLength	= wLength;
	uvc_ctrl.wValue		= wValue;

	if(bRType == 0x02 && wIndex == 0x83)
	{
		if (glIsApplnActive)
		{
			esUVCApplnStop ();
		}
		return CyFalse;
	}

    /* Check for UVC Class Requests */
    if (bType != CY_U3P_USB_CLASS_RQT)
    {
        return CyFalse;
    }
    if(wIndex == 0x0001)
	{
		if((wValue == 0x0100) || (wValue ==0x0200))	//Probe /Commit Ctrl
		{
			/* UVC Specific Requests */
			if ((bRequest == ES_USB_UVC_GET_CUR_REQ) || (bRequest == ES_USB_UVC_GET_MIN_REQ) || (bRequest == ES_USB_UVC_GET_MAX_REQ))
			{
				glProbeCtrl[3] = frame_index ;
				if(glUSBSpeed == 0x03)
				{
					if(frame_index == 1)//1080P
					{
						glProbeCtrl[4] = 0x15;
						glProbeCtrl[5] = 0x16;
						glProbeCtrl[6] = 0x05;
						glProbeCtrl[18] = 0x00;
						glProbeCtrl[19] = 0x48;
						glProbeCtrl[20] = 0x3F;
					}
					else if(frame_index == 2)//Vga
					{
						glProbeCtrl[4] = 0x15;
						glProbeCtrl[5] = 0x16;
						glProbeCtrl[6] = 0x05;
						glProbeCtrl[18] = 0x00;
						glProbeCtrl[19] = 0x60;
						glProbeCtrl[20] = 0x09;
					}
					else if(frame_index == 3)//720P
					{
						glProbeCtrl[4] = 0x0A;
						glProbeCtrl[5] = 0x8B;
						glProbeCtrl[6] = 0x02;
						glProbeCtrl[18] = 0x00;
						glProbeCtrl[19] = 0x20;
						glProbeCtrl[20] = 0x1C;
					}
					else if(frame_index == 4)//5MP
					{
						glProbeCtrl[4] = 0x0A;
						glProbeCtrl[5] = 0x8B;
						glProbeCtrl[6] = 0x02;
						glProbeCtrl[18] = 0x00;
						glProbeCtrl[19] = 0xC6;
						glProbeCtrl[20] = 0x99;
					}
				}
				if(glUSBSpeed == 0x02)
				{
					glProbeCtrl[4] = 0x15;
					glProbeCtrl[5] = 0x16;
					glProbeCtrl[6] = 0x05;
					glProbeCtrl[18] = 0x00;
					glProbeCtrl[19] = 0x60;
					glProbeCtrl[20] = 0x09;
					glProbeCtrl[22] = 0x00;
					glProbeCtrl[23] = 0x0C;
					glProbeCtrl[24] = 0x00;
					glProbeCtrl[25] = 0x00;
				}
				/* Host requests for probe data of 26 bytes. Send it over EP0. */
				status = CyU3PUsbSendEP0Data(ES_UVC_MAX_PROBE_SETTING,(uint8_t *)glProbeCtrl);
				if (status != CY_U3P_SUCCESS)
				{
					CyU3PDebugPrint (4, "\rCyU3PUsbSendEP0Data, error code = %d\r\n", status);
					return CyFalse;
				}
			}
			else if (bRequest == ES_USB_UVC_SET_CUR_REQ)
			{
				/* Get the UVC probe/commit control data from EP0 */
				status = CyU3PUsbGetEP0Data(ES_UVC_MAX_PROBE_SETTING_ALIGNED, glCommitCtrl, &readCount);
				if (status != CY_U3P_SUCCESS)
					CyU3PDebugPrint (4, "\rCyU3PUsbGetEP0Data failed, error code = %d\r\n", status);

				frame_index = glCommitCtrl[3];
				/* Check the read count. Expecting a count of ES_UVC_MAX_PROBE_SETTING bytes. */
				if (readCount != (uint16_t)ES_UVC_MAX_PROBE_SETTING)
				{
					CyU3PDebugPrint (4, "\rInvalid number of bytes received in SET_CUR Request\r\n");
					return CyFalse;
				}
				else
				{
					if(wValue == 0x0200)
					{
						if(glUSBSpeed == 0x03)
						{
							if (glIsApplnActive)
							{
								esUVCApplnStop ();
							}
							esUVCApplnStart ();
						}
					}
				}
			}
			else
			{
				/* Mark with some error. */
				status = CY_U3P_ERROR_FAILURE;
				return CyFalse;
			}
			return CyTrue;
		}
		else if((wValue == 0x0300) || (wValue ==0x0400))	//Still Probe /Commit Ctrl
		{
			if((bRequest == ES_USB_UVC_GET_CUR_REQ) || (bRequest == ES_USB_UVC_GET_MIN_REQ) || (bRequest == ES_USB_UVC_GET_MAX_REQ))
			{
				glStillProbeCtrl[1]=Still_frame_index;

				status = CyU3PUsbSendEP0Data(ES_UVC_MAX_STILL_PROBE_SETTING,(uint8_t*)glStillProbeCtrl);
				if(status != CY_U3P_SUCCESS)
					CyU3PDebugPrint(4,"\rStill CyU3PUsbSendEP0Data Failed 0x%x\r\n",status);
			}
			else if(bRequest == ES_USB_UVC_SET_CUR_REQ)
			{
				/* Get the UVC probe/commit control data from EP0 */
				status = CyU3PUsbGetEP0Data(16,gStillCommitCtrl, &readCount);
				if(status != CY_U3P_SUCCESS)
					CyU3PDebugPrint(4,"\rStill CyU3PUsbGetEP0Data Failed 0x%x\r\n",status);

				Still_frame_index = gStillCommitCtrl[1];
			}
			return CyTrue;
		}
		else if (wValue == 0x0500)	//Still Trigger
		{
			status = CyU3PUsbGetEP0Data(16,&gStillReq, &readCount);
			if(status != CY_U3P_SUCCESS)
				CyU3PDebugPrint(4,"\rStill CyU3PUsbGetEP0Data Failed 0x%x\r\n",status);

			glstillflag=CyTrue;

			CyU3PEventGet (&glStillImageEvent,ES_STILL_IMAGE_EVENT, CYU3P_EVENT_OR_CLEAR, &eventFlag, CYU3P_WAIT_FOREVER);
			if(glProbeCtrl[3]!= 4)//if Preview Resolution is not 5MP
			{
				esUVCApplnStop();
				glprevprobe = glProbeCtrl[3];

				glimageskip = 3;

				frame_index = 4;//5MP
				esUVCApplnStart();
			}
			while(glimageskip!=0)
			{
				CyU3PEventGet (&glStillImageEvent,ES_STILL_IMAGE_EVENT, CYU3P_EVENT_OR_CLEAR, &eventFlag, CYU3P_WAIT_FOREVER);
				glimageskip--;
			}

			glUVCHeader[1]^=ES_UVC_HEADER_STI;
			CyU3PEventGet (&glStillImageEvent,ES_STILL_IMAGE_EVENT, CYU3P_EVENT_OR_CLEAR, &eventFlag, CYU3P_WAIT_FOREVER);
			glUVCHeader[1]^=ES_UVC_HEADER_STI;

			if(glprevprobe)
			{
				esUVCApplnStop();
				frame_index=glprevprobe;
				glprevprobe=0;
				glstillflag=CyFalse;
				esUVCApplnStart();
			}
			glstillflag=CyFalse;

			return CyTrue;
		}
		return CyFalse;
	}
    else if ((wIndex == 0x0200 && ((wValue == 0x0200)||(wValue == 0x0300)||(wValue == 0x0400) ||(wValue == 0x0600)||(wValue == 0x0700)||(wValue == 0x0800)||(wValue == 0x0A00)||(wValue == 0x0B00)) )	||  ((wIndex == 0x0100)&&((wValue == 0x0400)||(wValue == 0x0200)||(wValue == 0x0800)||(wValue == 0x0600))) )
	{
		/* Check for SET_CUR or GET_CUR request */
		if (bRequest == ES_USB_UVC_GET_CUR_REQ)
		{
			CyU3PEventSet(&glUVC_CtrlEvent,ES_UVC_GET_CUR_EVENT,CYU3P_EVENT_OR);
		}
		else if (bRequest == ES_USB_UVC_GET_MIN_REQ)
		{
			CyU3PEventSet(&glUVC_CtrlEvent,ES_UVC_GET_MIN_EVENT,CYU3P_EVENT_OR);
		}
		else if (bRequest == ES_USB_UVC_GET_MAX_REQ)
		{
			CyU3PEventSet(&glUVC_CtrlEvent,ES_UVC_GET_MAX_EVENT,CYU3P_EVENT_OR);
		}
		else if (bRequest == ES_USB_UVC_GET_RES_REQ)
		{
			CyU3PEventSet(&glUVC_CtrlEvent,ES_UVC_GET_RES_EVENT,CYU3P_EVENT_OR);
		}
		else if (bRequest == ES_USB_UVC_GET_INFO_REQ)
		{
			CyU3PEventSet(&glUVC_CtrlEvent,ES_UVC_GET_INFO_EVENT,CYU3P_EVENT_OR);
		}
		else if (bRequest == ES_USB_UVC_GET_DEF_REQ)
		{
			CyU3PEventSet(&glUVC_CtrlEvent,ES_UVC_GET_DEF_EVENT,CYU3P_EVENT_OR);
		}
		else if (bRequest == ES_USB_UVC_SET_CUR_REQ)
		{
			CyU3PEventSet(&glUVC_CtrlEvent,ES_UVC_SET_CUR_EVENT,CYU3P_EVENT_OR);
		}
		else
		{
			/* ignore all other requests */
			return CyFalse;
		}
		return CyTrue;
	}
    if((bRequest == 0x0A) && (wIndex == 0x2))
    {
    	status = CY_U3P_ERROR_FAILURE;
    	return CyFalse;
    }
    if (status != CY_U3P_SUCCESS)
    {
    	return CyFalse;
    }
    return CyFalse;
}

/* GpifCB callback function is invoked when FV triggers GPIF interrupt */
void
esGpifCB (
       CyU3PGpifEventType event,
       uint8_t currentState
       )
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS ;

	if (event == CYU3P_GPIF_EVT_SM_INTERRUPT)
	{
		gHitFV = CyTrue;
		if(currentState == CX3_PARTIAL_BUFFER_IN_SCK0)
		{
			apiRetStatus = CyU3PDmaMultiChannelSetWrapUp(&glChHandleUVCStream,0);
			if (apiRetStatus != CY_U3P_SUCCESS)
				CyU3PDebugPrint (4, "\rChannel 0 Set WrapUp failed, Error Code = %d\r\n",apiRetStatus);
		}
		else if(currentState == CX3_PARTIAL_BUFFER_IN_SCK1)
		{
			apiRetStatus = CyU3PDmaMultiChannelSetWrapUp(&glChHandleUVCStream,1);
			if (apiRetStatus != CY_U3P_SUCCESS)
				CyU3PDebugPrint (4, "\rChannel 1 Set WrapUp failed, Error Code = %d\r\n",apiRetStatus);
		}
	}
}

/* This function initializes the USB Module, creates event group,
   sets the enumeration descriptors, configures the Endpoints and
   configures the DMA module for the UVC Application */
void
esUVCApplnInit (void)
{
	CyU3PEpConfig_t epCfg;
	CyU3PDmaMultiChannelConfig_t dmaCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Initialize the I2C interface for Mipi Block Usage and Camera. */
    apiRetStatus = CyU3PMipicsiInitializeI2c (CY_U3P_MIPICSI_I2C_100KHZ);
    if( apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\rCyCx3UvcApplnInit: Initiliaize I2C Failed , Error = %d\r\n",apiRetStatus);
        esAppErrorHandler(apiRetStatus);
    }

    /* Initialize GPIO module. */
    apiRetStatus = CyU3PMipicsiInitializeGPIO ();
    if( apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\rCyCx3UvcApplnInit: Initiliaize GPIO Failed , Error = %d\r\n",apiRetStatus);
        esAppErrorHandler(apiRetStatus);
    }

    /* Initialize the PIB block */
    apiRetStatus = CyU3PMipicsiInitializePIB ();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\rCyCx3UvcApplnInit: Initiliaize PIB Failed , Error = %d\r\n",apiRetStatus);
        esAppErrorHandler(apiRetStatus);
    }

	CyU3PGpifRegisterCallback(esGpifCB);

    /* Start the USB functionality */
    apiRetStatus = CyU3PUsbStart();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\rUSB Function Failed to Start, Error Code = %d\r\n",apiRetStatus);
        esAppErrorHandler(apiRetStatus);
    }

    /* The fast enumeration is the easiest way to setup a USB connection,
     * where all enumeration phase is handled by the library. Only the
     * class / vendor requests need to be handled by the application. */
    CyU3PUsbRegisterSetupCallback(esUVCApplnUSBSetupCB, CyFalse);

    /* Setup the callback to handle the USB events */
    CyU3PUsbRegisterEventCallback(esUVCApplnUSBEventCB);

    /* Set the USB Enumeration descriptors */

    /* Super speed device descriptor. */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, NULL, (uint8_t *)esUSB30DeviceDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\rUSB set device descriptor failed, Error code = %d\r\n", apiRetStatus);
        esAppErrorHandler(apiRetStatus);
    }

    /* High speed device descriptor. */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, NULL, (uint8_t *)esUSB20DeviceDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\rUSB set device descriptor failed, Error code = %d\r\n", apiRetStatus);
        esAppErrorHandler(apiRetStatus);
    }

    /* BOS descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, NULL, (uint8_t *)esUSBBOSDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\rUSB set configuration descriptor failed, Error code = %d\r\n", apiRetStatus);
        esAppErrorHandler(apiRetStatus);
    }

    /* Device qualifier descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, NULL, (uint8_t *)esUSBDeviceQualDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\rUSB set device qualifier descriptor failed, Error code = %d\r\n", apiRetStatus);
        esAppErrorHandler(apiRetStatus);
    }

    /* Super speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, NULL, (uint8_t *)esUSBSSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\rUSB set configuration descriptor failed, Error code = %d\r\n", apiRetStatus);
        esAppErrorHandler(apiRetStatus);
    }

    /* High speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, NULL, (uint8_t *)esUSBHSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\rUSB Set Other Speed Descriptor failed, Error Code = %d\r\n", apiRetStatus);
        esAppErrorHandler(apiRetStatus);
    }

    /* Full speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, NULL, (uint8_t *)esUSBFSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\rUSB Set Configuration Descriptor failed, Error Code = %d\r\n", apiRetStatus);
        esAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 0 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)esUSBStringLangIDDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\rUSB set string descriptor failed, Error code = %d\r\n", apiRetStatus);
        esAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 1 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)esUSBManufactureDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\rUSB set string descriptor failed, Error code = %d\r\n", apiRetStatus);
        esAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 2 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)esUSBProductDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\rUSB set string descriptor failed, Error code = %d\r\n", apiRetStatus);
        esAppErrorHandler(apiRetStatus);
    }

    /* Since the status interrupt endpoint is not used in this application,
     * just enable the EP in the beginning. */
    /* Control status interrupt endpoint configuration */
    epCfg.enable = 1;
    epCfg.epType = CY_U3P_USB_EP_INTR;
    epCfg.pcktSize = 64;
    epCfg.isoPkts  = 1;
    epCfg.burstLen = 1;

    apiRetStatus = CyU3PSetEpConfig(ES_EP_CONTROL_STATUS, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\rCyU3PSetEpConfig failed, error code = %d\r\n",apiRetStatus);
        esAppErrorHandler(apiRetStatus);
    }

    /* Connect the USB pins and enable super speed operation */
    apiRetStatus = CyU3PConnectState(CyTrue, CyTrue);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\rUSB connect failed, Error Code = %d\r\n",apiRetStatus);
        esAppErrorHandler(apiRetStatus);
    }

    CyU3PThreadSleep(1000);

    glUSBSpeed = 0x00;
    glUSBSpeed = CyU3PUsbGetSpeed();

	/* Video streaming endpoint configuration */
    if(glUSBSpeed == 0x03)
    {
		epCfg.enable 	= CyTrue;
		epCfg.epType 	= CY_U3P_USB_EP_BULK;
		epCfg.pcktSize 	= ES_EP_BULK_VIDEO_PKT_SIZE;
		epCfg.isoPkts 	= 0;
		epCfg.burstLen 	= 12;
		epCfg.streams 	= 0;
    }
    else if(glUSBSpeed == 0x02)
    {
    	epCfg.enable 	= CyTrue;
		epCfg.epType 	= CY_U3P_USB_EP_ISO;
		epCfg.pcktSize 	= ES_EP_ISO_VIDEO_PKT_SIZE;
		epCfg.isoPkts 	= ES_EP_ISO_VIDEO_PKTS_COUNT;
		epCfg.burstLen 	= 1;
		epCfg.streams 	= 0;
    }

	apiRetStatus = CyU3PSetEpConfig(ES_EP_UVC_VIDEO, &epCfg);
	if (apiRetStatus != CY_U3P_SUCCESS)
		CyU3PDebugPrint (4, "\rCyU3PSetEpConfig failed, Error Code = %d\r\n", apiRetStatus);

	/* Flush the endpoint memory */
	CyU3PUsbFlushEp(ES_EP_UVC_VIDEO);

	/* Create a DMA Manual OUT channel for streaming data */
	/* Video streaming Channel is not active till a stream request is received */
	dmaCfg.size 			= ES_UVC_BULK_STREAM_BUF_SIZE;
	dmaCfg.count 			= 8;
	dmaCfg.validSckCount 	= 2;
	dmaCfg.prodSckId[0] 	= ES_PRODUCER_PPORT_SOCKET;
	dmaCfg.prodSckId[1] 	= ES_PRODUCER_PPORT_SOCKET_1;
	dmaCfg.consSckId[0] 	= ES_EP_VIDEO_CONS_SOCKET;
	dmaCfg.dmaMode 			= CY_U3P_DMA_MODE_BYTE;
	dmaCfg.notification 	= CY_U3P_DMA_CB_PROD_EVENT | CY_U3P_DMA_CB_CONS_EVENT;
	dmaCfg.cb 				= esSlFifoPtoUDmaCallback;
	dmaCfg.prodHeader 		= 12;
	dmaCfg.prodFooter 		= 4;
	dmaCfg.consHeader 		= 0;
	dmaCfg.prodAvailCount 	= 0;

	if(glUSBSpeed == 0x02)
	{
		dmaCfg.size			= ES_UVC_ISO_STREAM_BUF_SIZE;
	}

	apiRetStatus = CyU3PDmaMultiChannelCreate (&glChHandleUVCStream, CY_U3P_DMA_TYPE_MANUAL_MANY_TO_ONE , &dmaCfg);
	if (apiRetStatus != CY_U3P_SUCCESS)
		CyU3PDebugPrint (4, "\rCyU3PDmaChannelCreate failed, error code = %d\r\n",apiRetStatus);

	CyU3PThreadSleep(100);

	// Reset the channel: Set to DSCR chain starting point in PORD/CONS SCKT; set DSCR_SIZE field in DSCR memory
	apiRetStatus = CyU3PDmaMultiChannelReset(&glChHandleUVCStream);
	if (apiRetStatus != CY_U3P_SUCCESS)
		CyU3PDebugPrint (4,"\rDMA Channel Reset Failed, Error Code = %d\r\n",apiRetStatus);

	if(glUSBSpeed == 0x03)
	{
		apiRetStatus = CyU3PMipicsiGpifLoad(CY_U3P_MIPICSI_BUS_16,12272);
	}
	else if(glUSBSpeed == 0x02)
	{
		apiRetStatus = CyU3PMipicsiGpifLoad(CY_U3P_MIPICSI_BUS_16,2912);
	}

	if (apiRetStatus != CY_U3P_SUCCESS)
	{
	   CyU3PDebugPrint (4, "\rCyCx3UvcApplnInit:CyU3PMipicsiGpifLoad failed, Error = %d\r\n",apiRetStatus);
	   esAppErrorHandler(apiRetStatus);
	}
	CyU3PThreadSleep(10);

    /* Start the state machine. */
    apiRetStatus = CyU3PGpifSMStart (CX3_START, CX3_ALPHA_START);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\rCyCx3UvcApplnInit:CyU3PGpifSMStart failed, Error = %d\r\n",apiRetStatus);
        esAppErrorHandler(apiRetStatus);
    }

	CyU3PGpifSMControl(CyTrue);

    /* Initialize the MIPI block */
    apiRetStatus =  CyU3PMipicsiInit();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "\rCyCx3UvcApplnInit:CyU3PMipicsiInit failed, Error = %d\r\n",apiRetStatus);
        esAppErrorHandler(apiRetStatus);
    }

    apiRetStatus = CyU3PMipicsiSetIntfParams(&esUvcVGA60, CyFalse);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		CyU3PDebugPrint (4, "\rCyCx3UvcApplnInit:CyU3PMipicsiSetIntfParams failed, Error = %d\r\n",apiRetStatus);
		esAppErrorHandler(apiRetStatus);
	}

	/***************CAMERA********************/
	esOV5640_Base_Config();
	esOV5640_Auto_Focus_Config();
	esCamera_Power_Down();

}

/* Entry function for the UVC application thread. */
void
esUVCAppThread_Entry (
        uint32_t input)
{
	uint32_t eventFlag;
	CyU3PReturnStatus_t apiRetStatus;

    /* Initialize the UVC Application */
    esUVCApplnInit();

    for (;;)
    {
		CyU3PEventGet (&glDMADoneEvent,ES_FRAME_DMADONE_EVENT, CYU3P_EVENT_OR_CLEAR, &eventFlag, CYU3P_WAIT_FOREVER);

		if((gHitFV == CyTrue) && (glDmaDone == 0))
		{
			CyU3PGpifSMSwitch(257,0,257,0,2);

			if(glUSBSpeed == 0x02)
				CyU3PBusyWait(80);

			glDMATxCount = 0;
			glDmaDone = 0;
			gHitFV = CyFalse;

			// Reset the channel: Set to DSCR chain starting point in PORD/CONS SCKT; set DSCR_SIZE field in DSCR memory
			apiRetStatus = CyU3PDmaMultiChannelReset(&glChHandleUVCStream);
			if (apiRetStatus != CY_U3P_SUCCESS)
				CyU3PDebugPrint (4,"\rDMA Channel Reset Failed, Error Code = %d\r\n",apiRetStatus);

			// Start Channel Immediately
			apiRetStatus = CyU3PDmaMultiChannelSetXfer (&glChHandleUVCStream, 0, 0);
			if (apiRetStatus != CY_U3P_SUCCESS)
				CyU3PDebugPrint (4,"\rDMA Channel Set Transfer Failed, Error Code = %d\r\n",apiRetStatus);

			if(glstillflag==CyTrue)
				CyU3PEventSet(&glStillImageEvent, ES_STILL_IMAGE_EVENT,CYU3P_EVENT_OR);
		}
    } /* End of for(;;) */
}

void esUVC_Ctrl_Thread_Entry (uint32_t input)
{
	uint32_t eventFlag;
	CyU3PReturnStatus_t status;
	int32_t glexposureCtrl,Requ_Cam_Exposure;
	uint16_t readCount = 0,Requ_Cam_ManualFocus;
	uint8_t glautofocusCtrl,bGet_Info=0x03,glAutoExpCtrl;
    int16_t glbrightnessCtrl,glcontrastCtrl,glhueCtrl,glsaturationCtrl,glsharpnessCtrl,glwhite_balCtrl,glmanualfocusCtrl;
    uint8_t Requ_Cam_Hue=0,Requ_Cam_Saturation=0,Requ_Cam_Sharpness,Requ_Cam_White_Bal,Requ_Cam_AutoFocus,Requ_Cam_bright,Requ_Cam_AutoExp;

	for(;;)
	{
		if(CyU3PEventGet (&glUVC_CtrlEvent,(ES_UVC_GET_CUR_EVENT | ES_UVC_SET_CUR_EVENT | ES_UVC_GET_MIN_EVENT | ES_UVC_GET_MAX_EVENT | ES_UVC_GET_RES_EVENT | ES_UVC_GET_DEF_EVENT | ES_UVC_GET_INFO_EVENT), CYU3P_EVENT_OR_CLEAR, &eventFlag, CYU3P_WAIT_FOREVER) == CY_U3P_SUCCESS)
		{
			if (eventFlag & ES_UVC_GET_CUR_EVENT)
			{
				switch(uvc_ctrl.wValue)
				{
				case 0x0200:
					if(uvc_ctrl.wIndex == 0x0100)//Auto Exposure
					{
						glAutoExpCtrl = esOV5640_GetAutoExposure(0x04);
						CyU3PUsbSendEP0Data(1,(uint8_t*)&glAutoExpCtrl);
					}
					else if(uvc_ctrl.wIndex == 0x0200)//Brighntess
					{
						glbrightnessCtrl = (int16_t)esOV5640_GetBrightness(0x04);
						status=CyU3PUsbSendEP0Data(2,(uint8_t*)&glbrightnessCtrl);
						if(status != CY_U3P_SUCCESS)
							CyU3PDebugPrint(4,"\rGet_Cur Brightness Req failed\r\n");
					}
					break;
				case 0x0300://Contrast
					glcontrastCtrl = esOV5640_GetContrast(0x04);
					CyU3PUsbSendEP0Data(2,(uint8_t*)&glcontrastCtrl);
					break;
				case 0x0400://Manual Exposure
					glexposureCtrl = esOV5640_GetExposure(0x04);
					CyU3PUsbSendEP0Data(4,(uint8_t*)&glexposureCtrl);
					break;
				case 0x0600:
					if(uvc_ctrl.wIndex == 0x0200)//hue
					{
						glhueCtrl= esOV5640_GetHue(0x04);
						CyU3PUsbSendEP0Data(2,(uint8_t*)&glhueCtrl);
					}
					else if(uvc_ctrl.wIndex == 0x0100)//Manual Focus
					{
						glmanualfocusCtrl = esOV5640_GetManualfocus(0x04);
						CyU3PUsbSendEP0Data(2,(uint8_t*)&glmanualfocusCtrl);
					}
					break;
				case 0x0700://Saturation
					glsaturationCtrl = esOV5640_GetSaturation(0x04);
					CyU3PUsbSendEP0Data(2,(uint8_t*)&glsaturationCtrl);
					break;
				case 0x0800:
					if(uvc_ctrl.wIndex == 0x0200)//Sharpness
					{
						glsharpnessCtrl = esOV5640_GetSharpness(0x04);
						CyU3PUsbSendEP0Data(2,(uint8_t*)&glsharpnessCtrl);
					}
					else if(uvc_ctrl.wIndex == 0x0100)//Auto Focus
					{
						glautofocusCtrl = esOV5640_GetAutofocus(0x04);
						CyU3PUsbSendEP0Data(1,(uint8_t*)&glautofocusCtrl);
					}
					break;
				case 0x0A00://White Balance manual
					glwhite_balCtrl = esOV5640_GetWhiteBalance(0x04);
					CyU3PUsbSendEP0Data(2,(uint8_t*)&glwhite_balCtrl);
					break;
				case 0x0B00://White Balance Auto
					glwhite_balCtrl = esOV5640_GetAutoWhiteBalance(0x04);
					CyU3PUsbSendEP0Data(1,(uint8_t*)&glwhite_balCtrl);
					break;
				}
			}
			else if (eventFlag & ES_UVC_GET_MIN_EVENT)
			{
				switch(uvc_ctrl.wValue)
				{
				case 0x0200:
					if(uvc_ctrl.wIndex == 0x0100)//Auto Exposure
					{
						glAutoExpCtrl = esOV5640_GetAutoExposure(0x02);
						CyU3PUsbSendEP0Data(1,(uint8_t*)&glAutoExpCtrl);
					}
					else if(uvc_ctrl.wIndex == 0x0200)//Brighntess
					{
						glbrightnessCtrl = esOV5640_GetBrightness(0x02);
						CyU3PUsbSendEP0Data(2,(uint8_t*)&glbrightnessCtrl);
					}
					break;
				case 0x0300:
					glcontrastCtrl = esOV5640_GetContrast(0x02);
					CyU3PUsbSendEP0Data(2,(uint8_t*)&glcontrastCtrl);
					break;
				case 0x0400://Manual Exposure
					glexposureCtrl = esOV5640_GetExposure(0x02);
					CyU3PUsbSendEP0Data(4,(uint8_t*)&glexposureCtrl);
					break;
				case 0x0600:
					if(uvc_ctrl.wIndex == 0x0200)//Hue
					{
						glhueCtrl = esOV5640_GetHue(0x02);
						CyU3PUsbSendEP0Data(2,(uint8_t*)&glhueCtrl);
					}
					else if(uvc_ctrl.wIndex == 0x0100)//Manual Focus
					{
						glmanualfocusCtrl = esOV5640_GetManualfocus(0x02);
						CyU3PUsbSendEP0Data(2,(uint8_t*)&glmanualfocusCtrl);
					}
					break;
				case 0x0700:
					glsaturationCtrl = esOV5640_GetSaturation(0x02);
					CyU3PUsbSendEP0Data(2,(uint8_t*)&glsaturationCtrl);
					break;
				case 0x0800:
					if(uvc_ctrl.wIndex == 0x0200)//sharpness
					{
						glsharpnessCtrl = esOV5640_GetSharpness(0x02);
						CyU3PUsbSendEP0Data(2,(uint8_t*)&glsharpnessCtrl);
					}
					else if(uvc_ctrl.wIndex == 0x0100)//Auto focus
					{
					}
					break;
				case 0x0A00:
					glwhite_balCtrl = esOV5640_GetWhiteBalance(0x02);
					CyU3PUsbSendEP0Data(2,(uint8_t*)&glwhite_balCtrl);
					break;
				case 0x0B00:
					break;
				}
			}
			else if (eventFlag & ES_UVC_GET_MAX_EVENT)
			{
				switch(uvc_ctrl.wValue)
				{
				case 0x0200:
					if(uvc_ctrl.wIndex == 0x0100)//Auto Exposure
					{
						glAutoExpCtrl = esOV5640_GetAutoExposure(0x03);
						CyU3PUsbSendEP0Data(1,(uint8_t*)&glAutoExpCtrl);
					}
					else if(uvc_ctrl.wIndex == 0x0200)//Brighntess
					{
						glbrightnessCtrl = esOV5640_GetBrightness(0x03);
						CyU3PUsbSendEP0Data(2,(uint8_t*)&glbrightnessCtrl);
					}
					break;
				case 0x0300:
					glcontrastCtrl = esOV5640_GetContrast(0x03);
					CyU3PUsbSendEP0Data(2,(uint8_t*)&glcontrastCtrl);
					break;
				case 0x0400://Manual Exposure
					glexposureCtrl = esOV5640_GetExposure(0x03);
					CyU3PUsbSendEP0Data(4,(uint8_t*)&glexposureCtrl);
					break;
				case 0x0600:
					if(uvc_ctrl.wIndex == 0x0200)//Hue
					{
						glhueCtrl = esOV5640_GetHue(0x03);
						CyU3PUsbSendEP0Data(2,(uint8_t*)&glhueCtrl);
					}
					if(uvc_ctrl.wIndex == 0x0100)//Manual Focus
					{
						glmanualfocusCtrl = esOV5640_GetManualfocus(0x03);
						CyU3PUsbSendEP0Data(2,(uint8_t*)&glmanualfocusCtrl);
					}
					break;
				case 0x0700:
					glsaturationCtrl = esOV5640_GetSaturation(0x03);
					CyU3PUsbSendEP0Data(2,(uint8_t*)&glsaturationCtrl);
					break;
				case 0x0800:
					if(uvc_ctrl.wIndex == 0x0200)//Sharpness
					{
						glsharpnessCtrl = esOV5640_GetSharpness(0x03);
						CyU3PUsbSendEP0Data(2,(uint8_t*)&glsharpnessCtrl);
					}
					else if(uvc_ctrl.wIndex == 0x0100)//Auto Focus
					{
					}
					break;
				case 0x0A00:
					glwhite_balCtrl = esOV5640_GetWhiteBalance(0x03);
					CyU3PUsbSendEP0Data(2,(uint8_t*)&glwhite_balCtrl);
					break;
				case 0x0B00:
					break;
				}
			}
			else if (eventFlag & ES_UVC_GET_RES_EVENT)
			{
				switch(uvc_ctrl.wValue)
				{
				case 0x0200:
					if(uvc_ctrl.wIndex == 0x0100)//Auto Exposure
					{
						glAutoExpCtrl = esOV5640_GetAutoExposure(0x05);
						CyU3PUsbSendEP0Data(1,(uint8_t*)&glAutoExpCtrl);
					}
					else if(uvc_ctrl.wIndex == 0x0200)//Brighntess
					{
						glbrightnessCtrl = esOV5640_GetBrightness(0x05);
						CyU3PUsbSendEP0Data(2,(uint8_t*)&glbrightnessCtrl);
					}
					break;
				case 0x0300:
					glcontrastCtrl = esOV5640_GetContrast(0x05);
					CyU3PUsbSendEP0Data(2,(uint8_t*)&glcontrastCtrl);
					break;
				case 0x0400://Manual Exposure
					glexposureCtrl = esOV5640_GetExposure(0x05);
					CyU3PUsbSendEP0Data(4,(uint8_t*)&glexposureCtrl);
					break;
				case 0x0600:
					if(uvc_ctrl.wIndex == 0x0200)//Hue
					{
						glhueCtrl = esOV5640_GetHue(0x05);
						CyU3PUsbSendEP0Data(2,(uint8_t*)&glhueCtrl);
					}
					else if(uvc_ctrl.wIndex == 0x0100)//Manual Focus
					{
						glmanualfocusCtrl = esOV5640_GetManualfocus(0x05);
						CyU3PUsbSendEP0Data(2,(uint8_t*)&glmanualfocusCtrl);
					}
					break;
				case 0x0700:
					glsaturationCtrl = esOV5640_GetSaturation(0x05);
					CyU3PUsbSendEP0Data(2,(uint8_t*)&glsaturationCtrl);
					break;
				case 0x0800:
					if(uvc_ctrl.wIndex == 0x0200)//Sharpness
					{
						glsharpnessCtrl = esOV5640_GetSharpness(0x05);
						CyU3PUsbSendEP0Data(2,(uint8_t*)&glsharpnessCtrl);
					}
					else if(uvc_ctrl.wIndex == 0x0100)//Auto Focus
					{
					}
					break;
				case 0x0A00:
					glwhite_balCtrl = esOV5640_GetWhiteBalance(0x05);
					CyU3PUsbSendEP0Data(2,(uint8_t*)&glwhite_balCtrl);
					break;
				case 0x0B00:
					break;
				}
			}
			else if (eventFlag & ES_UVC_GET_DEF_EVENT)
			{
				switch(uvc_ctrl.wValue)
				{
				case 0x0200:
					if(uvc_ctrl.wIndex == 0x0100)//Auto Exposure
					{
						glAutoExpCtrl = esOV5640_GetAutoExposure(0x01);
						CyU3PUsbSendEP0Data(1,(uint8_t*)&glAutoExpCtrl);
					}
					else if(uvc_ctrl.wIndex == 0x0200)//Brighntess
					{
						glbrightnessCtrl = esOV5640_GetBrightness(0x01);
						CyU3PUsbSendEP0Data(2,(uint8_t*)&glbrightnessCtrl);
					}
					break;
				case 0x0300:
					glcontrastCtrl = esOV5640_GetContrast(0x01);
					CyU3PUsbSendEP0Data(2,(uint8_t*)&glcontrastCtrl);
					break;
				case 0x0400://Manual Exposure
					glexposureCtrl = esOV5640_GetExposure(0x01);
					CyU3PUsbSendEP0Data(4,(uint8_t*)&glexposureCtrl);
					break;
				case 0x0600:
					if(uvc_ctrl.wIndex == 0x0200)//Hue
					{
						glhueCtrl = esOV5640_GetHue(0x01);
						CyU3PUsbSendEP0Data(2,(uint8_t*)&glhueCtrl);
					}
					else if(uvc_ctrl.wIndex == 0x0100)//Manual Focus
					{
						glmanualfocusCtrl = esOV5640_GetManualfocus(0x01);
						CyU3PUsbSendEP0Data(2,(uint8_t*)&glmanualfocusCtrl);
					}
					break;
				case 0x0700:
					glsaturationCtrl = esOV5640_GetSaturation(0x01);
					CyU3PUsbSendEP0Data(2,(uint8_t*)&glsaturationCtrl);
					break;
				case 0x0800:
					if(uvc_ctrl.wIndex == 0x0200)//Sharpness
					{
						glsharpnessCtrl = esOV5640_GetSharpness(0x01);
						CyU3PUsbSendEP0Data(2,(uint8_t*)&glsharpnessCtrl);
					}
					else if(uvc_ctrl.wIndex == 0x0100)//Auto Focus
					{
						glautofocusCtrl = esOV5640_GetAutofocus(0x01);
						CyU3PUsbSendEP0Data(1,(uint8_t*)&glautofocusCtrl);
					}
					break;
				case 0x0A00:
					glwhite_balCtrl = esOV5640_GetWhiteBalance(0x01);
					CyU3PUsbSendEP0Data(2,(uint8_t*)&glwhite_balCtrl);
					break;
				case 0x0B00:
					glwhite_balCtrl = esOV5640_GetAutoWhiteBalance(0x01);
					CyU3PUsbSendEP0Data(1,(uint8_t*)&glwhite_balCtrl);
					break;
				}
			}
			else if (eventFlag & ES_UVC_GET_INFO_EVENT)
			{
					bGet_Info=0x03;
					CyU3PUsbSendEP0Data(1,(uint8_t*)&bGet_Info);
			}
			else if (eventFlag & ES_UVC_SET_CUR_EVENT)
			{
				switch(uvc_ctrl.wValue)
				{
				case 0x0200:
					if(uvc_ctrl.wIndex == 0x0100)//Auto Exposure
					{
						CyU3PUsbGetEP0Data(16,(uint8_t*)&Requ_Cam_AutoExp,&readCount);
						esOV5640_SetAutoExposure(Requ_Cam_AutoExp);
					}
					else if(uvc_ctrl.wIndex == 0x0200)//Brighntess
					{
						status = CyU3PUsbGetEP0Data(16,(uint8_t*)&Requ_Cam_bright,&readCount);
						if (status != CY_U3P_SUCCESS)
							CyU3PDebugPrint (4,"\rCyU3PUsbGetEP0Data failed, error code = %d\r\n",status);
						if (readCount != 2)
							CyU3PDebugPrint (4,"\rInvalid number of bytes received in SET_CUR Request\r\n");
						esOV5640_SetBrightness(Requ_Cam_bright);
					}
					break;
				case 0x0300:
					status = CyU3PUsbGetEP0Data(16,(uint8_t*)&Requ_Cam_bright,&readCount);
					if (status != CY_U3P_SUCCESS)
						CyU3PDebugPrint (4,"\rCyU3PUsbGetEP0Data failed, error code = %d\r\n",status);
					if (readCount != 2)
						CyU3PDebugPrint (4,"\rInvalid number of bytes received in SET_CUR Request\r\n");
					esOV5640_SetContrast(Requ_Cam_bright);
					break;
				case 0x0400://Manual Exposure
					CyU3PUsbGetEP0Data(16,(uint8_t*)&Requ_Cam_Exposure,&readCount);
					CyU3PEventGet (&glDMADoneEvent,ES_FRAME_DMADONE_EVENT, CYU3P_EVENT_OR, &eventFlag, CYU3P_WAIT_FOREVER);
					esOV5640_SetExposure(Requ_Cam_Exposure);
					break;
				case 0x0600:
					if(uvc_ctrl.wIndex == 0x0200)//Hue
					{
						CyU3PUsbGetEP0Data(16,(uint8_t*)&Requ_Cam_Hue,&readCount);
						esOV5640_SetHue((int8_t)Requ_Cam_Hue);
					}
					else if(uvc_ctrl.wIndex == 0x0100)//Manual Focus
					{
						status = CyU3PUsbGetEP0Data(16,(uint8_t*)&Requ_Cam_ManualFocus,&readCount);
						if (status != CY_U3P_SUCCESS)
							CyU3PDebugPrint (4,"\r CyU3PUsbGetEP0Data failed, error code = %d\r\n", status);
						esOV5640_SetManualfocus(Requ_Cam_ManualFocus);
						g_IsAutoFocus = 0;
					}
					break;
				case 0x0700:
					CyU3PUsbGetEP0Data(16,(uint8_t*)&Requ_Cam_Saturation,&readCount);
					esOV5640_SetSaturation(Requ_Cam_Saturation);
					break;
				case 0x0800:
					if(uvc_ctrl.wIndex == 0x0200)//Sharpness
					{
						CyU3PUsbGetEP0Data(16,(uint8_t*)&Requ_Cam_Sharpness,&readCount);
						esOV5640_SetSharpness(Requ_Cam_Sharpness);
					}
					else if(uvc_ctrl.wIndex == 0x0100)//Auto focus
					{
						status = CyU3PUsbGetEP0Data(16,(uint8_t*)&Requ_Cam_AutoFocus,&readCount);
						if (status != CY_U3P_SUCCESS)
							CyU3PDebugPrint (4,"\r CyU3PUsbGetEP0Data failed, error code = %d\r\n", status);
						esOV5640_SetAutofocus(Requ_Cam_AutoFocus);
						g_IsAutoFocus = 1;
					}
					break;
				case 0x0A00:
					CyU3PUsbGetEP0Data(16,(uint8_t*)&Requ_Cam_White_Bal,&readCount);
					esOV5640_SetWhiteBalance(Requ_Cam_White_Bal);
					break;
				case 0x0B00:
					CyU3PUsbGetEP0Data(16,(uint8_t*)&Requ_Cam_White_Bal,&readCount);
					esOV5640_SetAutoWhiteBalance(Requ_Cam_White_Bal);
					break;
				}
			}
		}
	}
}

/* Application define function which creates the threads. */
void
CyFxApplicationDefine (
        void)
{
    void *ptr = NULL,*ptr1 = NULL;
    uint32_t retThrdCreate = CY_U3P_SUCCESS;

    /* Initialize the Debug Module */
	esUVCApplnDebugInit();

	/* Allocate the memory for the thread and create the thread */
	ptr = CyU3PMemAlloc (UVC_APP_THREAD_STACK);
	retThrdCreate = CyU3PThreadCreate (&uvcAppThread,   /* UVC Thread structure */
						   "30:UVC_app_thread",         /* Thread Id and name */
						   esUVCAppThread_Entry,          /* UVC Application Thread Entry function */
						   0,                           /* No input parameter to thread */
						   ptr,                         /* Pointer to the allocated thread stack */
						   UVC_APP_THREAD_STACK,        /* UVC Application Thread stack size */
						   UVC_APP_THREAD_PRIORITY,     /* UVC Application Thread priority */
						   UVC_APP_THREAD_PRIORITY,     /* Pre-emption threshold */
						   CYU3P_NO_TIME_SLICE,         /* No time slice for the application thread */
						   CYU3P_AUTO_START             /* Start the Thread immediately */
						   );

	/* Check the return code */
	if (retThrdCreate != 0)
	{
		/* Thread Creation failed with the error code retThrdCreate */

		/* Add custom recovery or debug actions here */

		/* Application cannot continue */
		/* Loop indefinitely */
		while(1);
	}

	/* Allocate the memory for the thread and create the thread */
	ptr1 = CyU3PMemAlloc (UVC_APP_THREAD_STACK);
	retThrdCreate = CyU3PThreadCreate (&uvcCtrlThread,   /* UVC Thread structure */
						   "32:UVC_Ctrl_thread",         /* Thread Id and name */
						   esUVC_Ctrl_Thread_Entry,          /* UVC Application Thread Entry function */
						   0,                           /* No input parameter to thread */
						   ptr1,                         /* Pointer to the allocated thread stack */
						   UVC_APP_THREAD_STACK,        /* UVC Application Thread stack size */
						   UVC_APP_THREAD_PRIORITY,     /* UVC Application Thread priority */
						   UVC_APP_THREAD_PRIORITY,     /* Pre-emption threshold */
						   CYU3P_NO_TIME_SLICE,         /* No time slice for the application thread */
						   CYU3P_AUTO_START             /* Start the Thread immediately */
						   );
	/* Check the return code */
	if (retThrdCreate != 0)
	{
		/* Thread Creation failed with the error code retThrdCreate */
		/* Add custom recovery or debug actions here */
		/* Application cannot continue */
		/* Loop indefinitely */
		while(1);
	}


	/* Create GPIO application event group */
	retThrdCreate = CyU3PEventCreate(&glDMADoneEvent);
	if (retThrdCreate != 0)
	{
		/* Event group creation failed with the error code retThrdCreate */

		/* Add custom recovery or debug actions here */

		/* Application cannot continue */
		/* Loop indefinitely */
		while(1);
	}

	/* Create GPIO application event group */
	retThrdCreate = CyU3PEventCreate(&glUVC_CtrlEvent);
	if (retThrdCreate != 0)
	{
		/* Event group creation failed with the error code retThrdCreate */
		/* Add custom recovery or debug actions here */
		/* Application cannot continue */
		/* Loop indefinitely */
		while(1);
	}

	/* Create GPIO application event group */
	retThrdCreate = CyU3PEventCreate(&glStillImageEvent);
	if (retThrdCreate != 0)
	{
		/* Event group creation failed with the error code retThrdCreate */
		/* Add custom recovery or debug actions here */
		/* Application cannot continue */
		/* Loop indefinitely */
		while(1);
	}
}

/*
 * Main function
 */
int
main (void)
{
    CyU3PIoMatrixConfig_t io_cfg;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Initialize the device */
    status = CyU3PDeviceInit (NULL);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Initialize the caches. Enable instruction cache and keep data cache disabled.
     * The data cache is useful only when there is a large amount of CPU based memory
     * accesses. When used in simple cases, it can decrease performance due to large
     * number of cache flushes and cleans and also it adds to the complexity of the
     * code. */
    status = CyU3PDeviceCacheControl (CyTrue, CyFalse, CyFalse);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Configure the IO matrix for the device. On the FX3 DVK board, the COM port
     * is connected to the IO(53:56). This means that either DQ32 mode should be
     * selected or lppMode should be set to UART_ONLY. Here we are choosing
     * UART_ONLY configuration. */
    io_cfg.isDQ32Bit = CyFalse;
    io_cfg.s0Mode = CY_U3P_SPORT_INACTIVE;
	io_cfg.s1Mode = CY_U3P_SPORT_INACTIVE;

    io_cfg.useUart   = CyTrue;
    io_cfg.useI2C    = CyTrue;
    io_cfg.useI2S    = CyFalse;
    io_cfg.useSpi    = CyFalse;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
    /* No GPIOs are enabled. */
    io_cfg.gpioSimpleEn[0]  = 0;
    io_cfg.gpioSimpleEn[1]  = 0;
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    status = CyU3PDeviceConfigureIOMatrix (&io_cfg);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* This is a non returnable call for initializing the RTOS kernel */
    CyU3PKernelEntry ();

    /* Dummy return to make the compiler happy */
    return 0;

handle_fatal_error:
    /* Cannot recover from this error. */
    while (1);
}
