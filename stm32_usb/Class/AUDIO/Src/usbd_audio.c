/*******************************************************************************
  * @file    usbd_audio.c
  * @author  MCD Application Team
  * @brief   This file provides the Audio core functions.
  *
  *                                AUDIO Class  Description
  *          ===================================================================
  *           This driver manages the Audio Class 1.0 following the "USB Device Class Definition for
  *           Audio Devices V1.0 Mar 18, 98".
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Standard AC Interface Descriptor management
  *             - 1 Audio Streaming Interface (with single channel, PCM, Stereo mode)
  *             - 1 Audio Streaming Endpoint
  *             - 1 Audio Terminal Input (1 channel)
  *             - Audio Class-Specific AC Interfaces
  *             - Audio Class-Specific AS Interfaces
  *             - AudioControl Requests: only SET_CUR and GET_CUR requests are supported (for Mute)
  *             - Audio Feature Unit (Mute and Volume control)
  *             - Audio Synchronization type: Asynchronous
  *          The current audio class version supports the following audio features:
  *             - Pulse Coded Modulation (PCM) format
  *             - sampling rate: 44.1kHz, 48kHz, 96kHz
  *             - Bit resolution: 24
  *             - Number of channels: 2
  *             - Volume control max=0dB, min=-96dB, 3dB attenuation steps
  *             - Mute/Unmute
  *             - Asynchronous Endpoints
  *             - Endpoint for Sampling frequency DbgFeedbackHistory 10.14 3bytes
  ******************************************************************************
  */

#include "math.h"
#include "usbd_audio.h"
#include "usbd_ctlreq.h"
#include "bsp_audio.h"

#define AUDIO_SAMPLE_FREQ(frq) (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

 // Max packet size: (freq / 1000 + extra_samples) * channels * bytes_per_sample
 // e.g. (48000 / 1000 + 1) * 2(stereo) * 3(24bit) = 388

#define AUDIO_PACKET_SZE_24B(frq) (uint8_t)(((frq / 1000U + 1) * 4U * 3U) & 0xFFU), \
                                  (uint8_t)((((frq / 1000U + 1) * 4U * 3U) >> 8) & 0xFFU)


#define AUDIO_FB_DEFAULT 0x1800ED70 // I2S_Clk_Config24[2].nominal_fdbk (96kHz, 24bit, USE_MCLK_OUT false)

// DbgFeedbackHistory is limited to +/- 1kHz
#define  AUDIO_FB_DELTA_MAX (uint32_t)(1 << 22)

#define LOG_VOLUME(vol) expf(6.908 * vol) / 1000.0

static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef* pdev, uint8_t cfgidx);
static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef* pdev, uint8_t cfgidx);
static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);
static uint8_t* USBD_AUDIO_GetCfgDesc(uint16_t* length);
static uint8_t* USBD_AUDIO_GetDeviceQualifierDesc(uint16_t* length);
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef* pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef* pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef* pdev);
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef* pdev);
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef* pdev);
static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef* pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef* pdev, uint8_t epnum);
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);
static void AUDIO_REQ_GetMax(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);
static void AUDIO_REQ_GetMin(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);
static void AUDIO_REQ_GetRes(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);
static void AUDIO_OUT_StopAndReset(USBD_HandleTypeDef* pdev);
static void AUDIO_OUT_Restart(USBD_HandleTypeDef* pdev);
static int32_t USBD_AUDIO_Get_Vol3dB_Shift(int16_t volume);


USBD_ClassTypeDef USBD_AUDIO = {
    USBD_AUDIO_Init,
    USBD_AUDIO_DeInit,
    USBD_AUDIO_Setup,
    USBD_AUDIO_EP0_TxReady,
    USBD_AUDIO_EP0_RxReady,
    USBD_AUDIO_DataIn,
    USBD_AUDIO_DataOut,
    USBD_AUDIO_SOF,
    USBD_AUDIO_IsoINIncomplete,
    USBD_AUDIO_IsoOutIncomplete,
    USBD_AUDIO_GetCfgDesc,
    USBD_AUDIO_GetCfgDesc,
    USBD_AUDIO_GetCfgDesc,
    USBD_AUDIO_GetDeviceQualifierDesc,
};

// USB AUDIO device Configuration Descriptor
__ALIGN_BEGIN static uint8_t USBD_AUDIO_CfgDesc[USB_AUDIO_CONFIG_DESC_SIZ] __ALIGN_END = {
    // Configuration 1
    0x09,                              /* bLength */
    USB_DESC_TYPE_CONFIGURATION,       /* bDescriptorType */
    LOBYTE(USB_AUDIO_CONFIG_DESC_SIZ), /* wTotalLength bytes*/
    HIBYTE(USB_AUDIO_CONFIG_DESC_SIZ),
    0x02, /* bNumInterfaces: (2) 1 AudioControl, 1 AudioStreaming */
    0x01, /* bConfigurationValue */
    0x00, /* iConfiguration */
    0x80, /* bmAttributes  BUS Powered (0xC0 = self-powered) */
    0x32, /* bMaxPower = 50*2mA = 100 mA*/
    // 09 byte

    // Standard AC (AudioControl) Interface Descriptor (4.3.1)
    AUDIO_INTERFACE_DESC_SIZE,   /* bLength */
    USB_DESC_TYPE_INTERFACE,     /* bDescriptorType */
    0x00,                        /* bInterfaceNumber */
    0x00,                        /* bAlternateSetting: value used to select an alternate setting for the interface identified in field above */
    0x00,                        /* bNumEndpoints: 0 by default, 1 if the optional status interrupt endpoint is present */
    USB_DEVICE_CLASS_AUDIO,      /* bInterfaceClass */
    AUDIO_SUBCLASS_AUDIOCONTROL, /* bInterfaceSubClass */
    AUDIO_PROTOCOL_UNDEFINED,    /* bInterfaceProtocol: unused, must be set to 0 (UNDEFINED) */
    0x00,                        /* iInterface: Index of a string descriptor that describes this interface */
    // 09 byte

    // Class-specific AC (AudioControl) Interface Descriptor (4.3.2)
    AUDIO_INTERFACE_DESC_SIZE,       /* bLength: Size of this descriptor, in bytes: 8+n */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType: CS_INTERFACE descriptor type */
    AUDIO_CONTROL_HEADER,            /* bDescriptorSubtype: HEADER descriptor subtype */
    0x00, 0x01,                      /* bcdADC: 2 byte Audio Device Class Spec Release Number (here, Class 1.0) */
    0x27, 0x00,                      /* wTotalLength = 39: total number of bytes returned for class-specific AC interface desc */
    0x01,                            /* bInCollection: Number of {Audio,MIDI}Streaming interfaces in the collection this controller belongs to (n) */
    0x01,                            /* baInterfaceNr: Interface numer of the {Audio,MIDI}Streaming interface in the Collection */
    // 09 byte

    // Input Terminal Descriptor (4.3.2.1)
    AUDIO_INPUT_TERMINAL_DESC_SIZE,  /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
    AUDIO_CONTROL_INPUT_TERMINAL,    /* bDescriptorSubtype */
    0x01,                            /* bTerminalID: Unique constant identifying the terminal. Used as an address. */
    0x01, 0x01,                      /* wTerminalType: AUDIO_TERMINAL_USB_STREAMING (0x0101) */
    0x00,                            /* bAssocTerminal: used to associate an output terminal to an input terminal. Type must be bidirectional if used, zero otherwise. */
    0x04,                            /* bNrChannels: Number of logical output channels. */
    0x03, 0x00,                      /* wChannelConfig: Describes the spatial location of the logical channels (0x0003: FL FR) */
    0x00,                            /* iChannelNames: Index of a string descriptor describing the name of the first logical channel. */
    0x00,                            /* iTerminal: Index of a string descriptor describing the Input Terminal */
    // 12 byte

    // Feature Unit Descriptor (4.3.2.5)
    0x09,                            /* bLength: Size of descriptor in bytes (7+(ch+1)*n) */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
    AUDIO_CONTROL_FEATURE_UNIT,      /* bDescriptorSubtype */
    AUDIO_OUT_STREAMING_CTRL,        /* bUnitID */
    0x01,                            /* bSourceID: ID of the Unit or Terminal to which this Feature Unit is connected */
    0x01,                            /* bControlSize: Size in bytes of an element of the bmaControls() array */
	  (AUDIO_CONTROL_MUTE | AUDIO_CONTROL_VOL),                            /* bmaControls(0): A bitset to indicate Control is supported for master channel 0 */
    0x00,                            /* bmaControls(1): A bitset to indicate control is supported for logical chanenl 1 */
    0x00,                            /* iTerminal: Index of a string descriptor describing the Feature Unit */
    // 09 byte

    // Output Terminal Descriptor (4.3.2.2)
    0x09,                            /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
    AUDIO_CONTROL_OUTPUT_TERMINAL,   /* bDescriptorSubtype */
    0x03,                            /* bTerminalID */
    0x01, 0x03,                      /* wTerminalType (0x0301) */
    0x00,                            /* bAssocTerminal */
    0x02,                            /* bSourceID: ID of Unit or Terminal to which this Terminal is connected */
    0x00,                            /* iTerminal: Index of a string descriptor describing the Input Terminal */
    // 09 byte

    // Standard AS (AudioStreaming) Interface Descriptor (4.5.1)
    // Interface 1
    // Alternate Setting 0: zero bandwidth mode with zero endpoints, used to relinquish bandwidth when audio not used.
    AUDIO_INTERFACE_DESC_SIZE,     /* bLength */
    USB_DESC_TYPE_INTERFACE,       /* bDescriptorType */
    0x01,                          /* bInterfaceNumber */
    0x00,                          /* bAlternateSetting */
    0x00,                          /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,        /* bInterfaceClass */
    AUDIO_SUBCLASS_AUDIOSTREAMING, /* bInterfaceSubClass */
    AUDIO_PROTOCOL_UNDEFINED,      /* bInterfaceProtocol */
    0x00,                          /* iInterface */
    // 09 byte

    // Standard AS (AudioStreaming) Interface Descriptor (4.5.1)
    // Interface 1
    // Alternate Setting 1: used when Audio Streaming is in operation
    AUDIO_INTERFACE_DESC_SIZE,     /* bLength */
    USB_DESC_TYPE_INTERFACE,       /* bDescriptorType */
    0x01,                          /* bInterfaceNumber */
    0x01,                          /* bAlternateSetting */
    0x02,                          /* bNumEndpoints - 1 output & 1 feedback */
    USB_DEVICE_CLASS_AUDIO,        /* bInterfaceClass */
    AUDIO_SUBCLASS_AUDIOSTREAMING, /* bInterfaceSubClass */
    AUDIO_PROTOCOL_UNDEFINED,      /* bInterfaceProtocol: Not used, must be 0 (UNDEFINED) */
    0x00,                          /* iInterface */
    // 09 byte

    // Class-Specific AS (AudioStreaming) Interface Descriptor (4.5.2)
    AUDIO_STREAMING_INTERFACE_DESC_SIZE, /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,     /* bDescriptorType */
    AUDIO_STREAMING_GENERAL,             /* bDescriptorSubtype */
    0x01,                                /* bTerminalLink: the terminal ID of the associated terminal this interface is connected to */
    0x01,                                /* bDelay: delay (in frames) introduced by internal processing */
    0x01, 0x00,                          /* wFormatTag (AUDIO_FORMAT_PCM: 0x0001) */
    // 07 byte

    // USB Speaker Audio Type I Format Interface Descriptor
    14,                              /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
    AUDIO_STREAMING_FORMAT_TYPE,     /* bDescriptorSubtype */
    AUDIO_FORMAT_TYPE_I,             /* bFormatType */
    4,                               /* bNrChannels */
    3,                               /* bSubFrameSize :  3 Bytes per frame (24bits) */
    24,                              /* bBitResolution (24-bits per sample) */
    2,                               /* bSamFreqType 3 frequencies supported */
    AUDIO_SAMPLE_FREQ(44100),        /* Audio sampling frequency coded on 3 bytes */
    AUDIO_SAMPLE_FREQ(48000),        /* Audio sampling frequency coded on 3 bytes */
    // AUDIO_SAMPLE_FREQ(96000),        /* Audio sampling frequency coded on 3 bytes */
    // 17 byte

    // Endpoint 1 - Standard Descriptor
	  // Standard AS Isochronous Aduio Data Endpoint Descriptor (4.6.1.1)
    AUDIO_STANDARD_ENDPOINT_DESC_SIZE,         /* bLength */
    USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType */
    AUDIO_OUT_EP,                              /* bEndpointAddress 1 out endpoint */
    USBD_EP_TYPE_ISOC_ASYNC,                   /* bmAttributes */
    AUDIO_PACKET_SZE_24B(USBD_AUDIO_FREQ_MAX), /* wMaxPacketSize in Bytes (freq / 1000 + extra_samples) * channels * bytes_per_sample */
    0x01,                                      /* bInterval: Interval (in ms) for polling endpoint for data */
    0x00,                                      /* bRefresh: Reset to 0 */
    AUDIO_IN_EP,                               /* bSynchAddress */
    // 09 byte

    // Class-Specific AS Isochronous Audio Data Endpoint Descriptor (4.6.1.2)
    // Used to specify sample rate as controllable.
    AUDIO_STREAMING_ENDPOINT_DESC_SIZE, /* bLength */
    AUDIO_ENDPOINT_DESCRIPTOR_TYPE,     /* bDescriptorType */
    AUDIO_ENDPOINT_GENERAL,             /* bDescriptor */
    0x01,                               /* bmAttributes - Sampling Frequency control is supported. See UAC Spec 1.0 p.62 */
    0x00,                               /* bLockDelayUnits */
    0x00, 0x00,                              /* wLockDelay */
    // 07 byte

    // Endpoint 2 - Standard Descriptor - See UAC Spec 1.0 p.63 4.6.2.1 Standard AS Isochronous Synch Endpoint Descriptor
  	// 3byte 10.14 sampling frequency feedback to host
    AUDIO_STANDARD_ENDPOINT_DESC_SIZE, /* bLength */
    USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType */
    AUDIO_IN_EP,                       /* bEndpointAddress */
    0x11,                              /* bmAttributes */
    0x03, 0x00,                        /* wMaxPacketSize in Bytes */
    0x01,                              /* bInterval 1ms */
    SOF_RATE,                          /* bRefresh 4ms = 2^2 */
    0x00,                              /* bSynchAddress */
    // 09 byte
};

/** 
 * USB Standard Device Descriptor
 * @see https://www.keil.com/pack/doc/mw/USB/html/_u_s_b__device__qualifier__descriptor.html
 */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END = {
    USB_LEN_DEV_QUALIFIER_DESC,
    USB_DESC_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x01,
    0x00,
};

volatile uint32_t tx_flag = 1;
volatile uint32_t is_playing = 0;
volatile uint32_t all_ready = 0;

volatile uint32_t fb_nom = AUDIO_FB_DEFAULT;
volatile uint32_t fb_value = AUDIO_FB_DEFAULT;
volatile uint32_t audio_buf_writable_samples_last = AUDIO_TOTAL_BUF_SIZE /(2*6);

__IO float channelLevels[2] = { 0, 0 };
__IO float logChannelLevels[2] = { 0, 0 };
volatile uint8_t fb_data[3] = {
    (uint8_t)((AUDIO_FB_DEFAULT >> 8) & 0x000000FF),
    (uint8_t)((AUDIO_FB_DEFAULT >> 16) & 0x000000FF),
    (uint8_t)((AUDIO_FB_DEFAULT >> 24) & 0x000000FF)
};

// FNSOF is critical for frequency changing to work
volatile uint32_t fnsof = 0;

// volume attenuation is from 0dB (max volume, 0x0000) to -96dB (min volume, 0xA000) in 3dB steps
static int32_t USBD_AUDIO_Get_Vol3dB_Shift(int16_t volume ){
	if (volume < (int16_t)USBD_AUDIO_VOL_MIN) volume = (int16_t)USBD_AUDIO_VOL_MIN;
	if (volume > (int16_t)USBD_AUDIO_VOL_MAX) volume = (int16_t)USBD_AUDIO_VOL_MAX;
	return (int32_t)((((int16_t)USBD_AUDIO_VOL_MAX - volume) + (int16_t)USBD_AUDIO_VOL_STEP/2)/(int16_t)USBD_AUDIO_VOL_STEP);
	}

/**
  * @brief  USBD_AUDIO_Init
  *         Initialize the AUDIO interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef* pdev, uint8_t cfgidx)
{
  USBD_AUDIO_HandleTypeDef* haudio;

  /* Open EP OUT */
  USBD_LL_OpenEP(pdev, AUDIO_OUT_EP, USBD_EP_TYPE_ISOC, AUDIO_OUT_PACKET_24B);
  pdev->ep_out[AUDIO_OUT_EP & 0xFU].is_used = 1U;

  /* Open EP IN */
  USBD_LL_OpenEP(pdev, AUDIO_IN_EP, USBD_EP_TYPE_ISOC, AUDIO_IN_PACKET);
  pdev->ep_in[AUDIO_IN_EP & 0xFU].is_used = 1U;

  /* Flush feedback endpoint */
  USBD_LL_FlushEP(pdev, AUDIO_IN_EP);

  /** 
   * Set tx_flag 1 to block feedback transmission in SOF handler since 
   * device is not ready.
   */
  tx_flag = 1U;

  /* Allocate Audio structure */
  pdev->pClassData = USBD_malloc(sizeof(USBD_AUDIO_HandleTypeDef));

  if (pdev->pClassData == NULL) {
    return USBD_FAIL;
  } else {
    haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;
    haudio->alt_setting = 0U;
    haudio->offset = AUDIO_OFFSET_UNKNOWN;
    haudio->wr_ptr = 0U;
    haudio->rd_ptr = 0U;
    haudio->rd_enable = 0U;
    haudio->freq = USBD_AUDIO_FREQ_DEFAULT;
    haudio->bit_depth = USBD_AUDIO_BIT_DEPTH_DEFAULT;
    haudio->volume = USBD_AUDIO_VOL_DEFAULT;
    haudio->vol_3dB_shift = USBD_AUDIO_Get_Vol3dB_Shift(USBD_AUDIO_VOL_DEFAULT);
    haudio->mute = USBD_AUDIO_MUTE_DEFAULT;

    // Initialize the Audio output Hardware layer
    if (((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->Init(haudio->freq, haudio->volume, haudio->mute) != 0) {
      return USBD_FAIL;
    }
  }
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_DeInit
  *         DeInitialize the AUDIO layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef* pdev,
                                 uint8_t cfgidx)
{
  /* Flush all endpoints */
  USBD_LL_FlushEP(pdev, AUDIO_OUT_EP);
  USBD_LL_FlushEP(pdev, AUDIO_IN_EP);

  /* Close EP OUT */
  USBD_LL_CloseEP(pdev, AUDIO_OUT_EP);
  pdev->ep_out[AUDIO_OUT_EP & 0xFU].is_used = 0U;

  /* Close EP IN */
  USBD_LL_CloseEP(pdev, AUDIO_IN_EP);
  pdev->ep_in[AUDIO_IN_EP & 0xFU].is_used = 0U;

  /* Clear feedback transmission flag */
  tx_flag = 0U;

  /* DeInit physical Interface components */
  if (pdev->pClassData != NULL) {
    ((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->DeInit(0U);
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }

  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Setup
  *         Handle the AUDIO specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef* pdev,
                                USBD_SetupReqTypedef* req)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  uint16_t len;
  uint8_t* pbuf;
  uint16_t status_info = 0U;
  uint8_t ret = USBD_OK;

  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  switch (req->bmRequest & USB_REQ_TYPE_MASK) {
    /* AUDIO Class Requests */
    case USB_REQ_TYPE_CLASS:
      switch (req->bRequest) {
        case AUDIO_REQ_GET_CUR:
          AUDIO_REQ_GetCurrent(pdev, req);
          break;

        case AUDIO_REQ_GET_MAX:
          AUDIO_REQ_GetMax(pdev, req);
          break;

        case AUDIO_REQ_GET_MIN:
          AUDIO_REQ_GetMin(pdev, req);
          break;

        case AUDIO_REQ_GET_RES:
          AUDIO_REQ_GetRes(pdev, req);
          break;

        case AUDIO_REQ_SET_CUR:
          AUDIO_REQ_SetCurrent(pdev, req);
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    /* Standard Requests */
    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest) {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            USBD_CtlSendData(pdev, (uint8_t*)(void*)&status_info, 2U);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_DESCRIPTOR:
          if ((req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE) {
            pbuf = USBD_AUDIO_CfgDesc + 18;
            len = MIN(USB_AUDIO_DESC_SIZ, req->wLength);

            USBD_CtlSendData(pdev, pbuf, len);
          }
          break;

        case USB_REQ_GET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            USBD_CtlSendData(pdev, (uint8_t*)(void*)&haudio->alt_setting, 1U);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            if ((uint8_t)(req->wValue) <= USBD_MAX_NUM_INTERFACES) {
              /* Do things only when alt_setting changes */
              if (haudio->alt_setting != (uint8_t)(req->wValue)) {
                haudio->alt_setting = (uint8_t)(req->wValue);
                if (haudio->alt_setting == 0U) {
                	AUDIO_OUT_StopAndReset(pdev);
                	}
                else {
                	haudio->bit_depth = USBD_AUDIO_BIT_DEPTH_DEFAULT;
                  	AUDIO_OUT_Restart(pdev);
                	}
              	}
              USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
            } else {
              /* Call the error management function (command will be nacked */
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
            }
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;
    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return ret;
}


/**
  * @brief  USBD_AUDIO_GetCfgDesc
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t* USBD_AUDIO_GetCfgDesc(uint16_t* length)
{
  *length = sizeof(USBD_AUDIO_CfgDesc);
  return USBD_AUDIO_CfgDesc;
}

/**
  * @brief  USBD_AUDIO_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef* pdev,
                                 uint8_t epnum)
{
  /* epnum is the lowest 4 bits of bEndpointAddress. See UAC 1.0 spec, p.61 */
  if (epnum == (AUDIO_IN_EP & 0xf)) {
    tx_flag = 0U;
  }
  return USBD_OK;
}


#ifdef DEBUG_FEEDBACK_ENDPOINT
volatile uint32_t  DbgMinWritableSamples = 99999;
volatile uint32_t  DbgMaxWritableSamples = 0;
volatile uint32_t  DbgSofHistory[256] = {0};
volatile uint32_t  DbgWritableSampleHistory[256] = {0};
volatile float     DbgFeedbackHistory[256] = {0};
volatile uint8_t   DbgIndex = 0; // roll over every 256 entries
static volatile uint32_t  DbgSofCounter = 0;
#endif

/**
  * @brief  USBD_AUDIO_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef* pdev)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;
  static volatile uint32_t sof_count = 0;

  /* Do stuff only when playing */
  if (haudio->rd_enable == 1U && all_ready == 1U) {
#ifdef DEBUG_FEEDBACK_ENDPOINT
	DbgSofCounter++;
#endif
	// Update audio read pointer
    haudio->rd_ptr = AUDIO_TOTAL_BUF_SIZE - BSP_AUDIO_OUT_GetRemainingDataSize();

    // Calculate remaining writable buffer samples
    uint32_t audio_buf_writable_samples = haudio->rd_ptr < haudio->wr_ptr ?
    		  (haudio->rd_ptr + AUDIO_TOTAL_BUF_SIZE - haudio->wr_ptr)/6 : (haudio->rd_ptr - haudio->wr_ptr)/6;

    // Monitor remaining writable buffer samples with LED
    if (audio_buf_writable_samples < AUDIO_BUF_SAFEZONE_SAMPLES) {
    	BSP_OnboardLED_On();
    } else {
    	BSP_OnboardLED_Off();
    }

    sof_count += 1;

    if (sof_count == 1U) {
      sof_count = 0;
      // we start transmitting to I2S DAC when the audio buffer is half full, so the optimal
      // remaining writable size is (AUDIO_TOTAL_BUF_SIZE/2)/6 samples
      // Calculate feedback value based on the deviation from optimal
      int32_t audio_buf_writable_dev_from_nom_samples = audio_buf_writable_samples - AUDIO_TOTAL_BUF_SIZE/(2*6);
      // The feedback is ideally the true Fs generated by the I2S PLL clock and dividers. Unfortunately we have no means
      // to measure it internally. So we can only start with a nominal value calculated by assuming the HSE clock crystal
      // has 0ppm accuracy, and calculate the Fs frequency generated by the PLLI2S N, R, I2SDIV and ODD register values.
      // We then modify this nominal feedback frequency by the deviation from the ideal write pointer position wrt the read
      // pointer over time.
      // Need to multiply by at least a "PID k factor" of (1<<22) + 256 for a deviation of 1 sample to produce a change in feedback
      // as the internal fb value = (10.14) shifted 8bits in uint32_t.
      // We also should use the minimum "PID k factor" that keeps the write-pointer to read-pointer distance out of the
      // danger zone. This is to minimize the distortion caused by changes in host sampling frequency Fs.
      uint64_t tmp = (uint64_t)((int32_t)(1<<22) + (audio_buf_writable_dev_from_nom_samples * 256));
      uint64_t pid_k = ((uint64_t)fb_nom) * tmp;
      fb_value = (uint32_t)(pid_k >> 22);
      // Clamp feedback value to nominal value +/- 1kHz
      if (fb_value > fb_nom +  AUDIO_FB_DELTA_MAX) {
        fb_value = fb_nom +  AUDIO_FB_DELTA_MAX;
      } else if (fb_value < fb_nom -  AUDIO_FB_DELTA_MAX) {
        fb_value = fb_nom -  AUDIO_FB_DELTA_MAX;
      }

      #ifdef DEBUG_FEEDBACK_ENDPOINT
      if (audio_buf_writable_samples != audio_buf_writable_samples_last) {
        if (audio_buf_writable_samples > DbgMaxWritableSamples) DbgMaxWritableSamples = audio_buf_writable_samples;
        if (audio_buf_writable_samples < DbgMinWritableSamples) DbgMinWritableSamples = audio_buf_writable_samples;
        DbgWritableSampleHistory[DbgIndex] = audio_buf_writable_samples;
        DbgFeedbackHistory[DbgIndex] = (float)(fb_value >> 8)/(float)(1<<14);
        DbgSofHistory[DbgIndex] = DbgSofCounter;
        DbgIndex++; // uint8_t, so only record last 256 entries
        }
      #endif

      // Update last writable buffer size
      audio_buf_writable_samples_last = audio_buf_writable_samples;
      // Set 10.14 format feedback data
      // Order of 3 bytes in feedback packet: { LO byte, MID byte, HI byte }
      fb_data[0] = (uint8_t)((fb_value >> 8) & 0x000000FF);
      fb_data[1] = (uint8_t)((fb_value >> 16) & 0x000000FF);
      fb_data[2] = (uint8_t)((fb_value >> 24) & 0x000000FF);
		}

    /* Transmit feedback only when the last one is transmitted */
    if (tx_flag == 0U) {
      /* Get FNSOF. Use volatile for fnsof_new since its address is mapped to a hardware register. */
      USB_OTG_GlobalTypeDef* USBx = USB_OTG_FS;
      uint32_t USBx_BASE = (uint32_t)USBx;
      uint32_t volatile fnsof_new = (USBx_DEVICE->DSTS & USB_OTG_DSTS_FNSOF) >> 8;

      if ((fnsof & 0x1) == (fnsof_new & 0x1)) {
        USBD_LL_Transmit(pdev, AUDIO_IN_EP, (uint8_t*)fb_data, 3U);
        /* Block transmission until it's finished. */
        tx_flag = 1U;
      }
    }
  }

  return USBD_OK;
}


/**
  * @brief  USBD_AUDIO_Sync
  *         handle Sync event called from usbd_audio_if.c
  * @param  pdev: device instance
  * @retval status
  */
void USBD_AUDIO_Sync(USBD_HandleTypeDef* pdev, AUDIO_OffsetTypeDef offset)
{
}

/**
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * USBD_AUDIO_IsoINIncomplete & USBD_AUDIO_IsoOutIncomplete are not 
 * enabled by default.
 * 
 * Go to Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
 * Fill in USBD_LL_IsoINIncomplete and USBD_LL_IsoOUTIncomplete with 
 * actual handler functions.
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 */

/**
  * @brief  USBD_AUDIO_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef* pdev, uint8_t epnum)
{
  USB_OTG_GlobalTypeDef* USBx = USB_OTG_FS;
  uint32_t USBx_BASE = (uint32_t)USBx;
  fnsof = (USBx_DEVICE->DSTS & USB_OTG_DSTS_FNSOF) >> 8;

  if (tx_flag == 1U) {
    tx_flag = 0U;
    USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
  }

  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef* pdev, uint8_t epnum)
{
  return USBD_OK;
}


typedef  union UN32_ {
	uint8_t b[4];
	int32_t s;
} UN32;

// ref : https://www.microchip.com/forums/m932509.aspx

inline int32_t USBD_AUDIO_Volume_Ctrl(int32_t sample, int32_t shift_3dB){
	int32_t sample_atten = sample;
	int32_t shift_6dB = shift_3dB>>1;

	if (shift_3dB & 1) {
	    // shift_3dB is odd, implement 6dB shift and compensate
	    shift_6dB++;
        sample_atten >>= shift_6dB;
        sample_atten += (sample_atten>>1);
	    }
	else{
	    // shift_3dB is even, implement with 6dB shift
	    sample_atten >>= shift_6dB;
		}
	return sample_atten;
	}


/**
  * @brief  USBD_AUDIO_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
// incoming USB audio data buffer : uint8_t array
// Each 24bit stereo sample is encoded as : L channel 3bytes + R channel 3bytes, LSbyte first
// b0:lo_L, b1:mid_L, b2:hi_L, b3:lo_R, b4:mid_R, b5:hi_R

// volume control is implemented by scaling the data, attenuation resolution is 3dB.
// 6dB is equivalent to a shift right by 1 bit.

// outgoing I2S Philips data format is : left-aligned 24bits in 32bit frame, MSbyte first
// STM32 I2S peripheral uses a 16bit data register
// => outgoing I2S transmit data buffer : uint16_t array
// Each I2S stereo sample is encoded as {hi_L:mid_L}, {lo_L:0x00}, {hi_R:mid_R}, {lo_R:0x00}

static float s24_to_float(uint8_t *buf) {
  int32_t s32 = ((buf[2] << 24) | (buf[1] << 16) | (buf[0] << 8)) >> 8;
  return ((float)s32) / (float)0x7fffff;
}

static int32_t float_to_s32(float sample) {
  return (int32_t)(sample * (float)0x7fffff);
}

static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef* pdev,  uint8_t epnum){
	USBD_AUDIO_HandleTypeDef* haudio;
	haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

	static uint8_t tmpbuf[1024];

	if (all_ready == 1U && epnum == AUDIO_OUT_EP) {
		uint32_t curr_length = USBD_GetRxCount(pdev, epnum);
		// Ignore strangely large packets
		if (curr_length > AUDIO_OUT_PACKET_24B) {
			curr_length = 0U;
    }

		uint32_t tmpbuf_ptr = 0U;
    uint32_t sample_size = USB_AUDIO_CHANNELS * USB_AUDIO_BYTES_PER_SAMPLE;
		uint32_t num_samples = curr_length / sample_size;

		for (int i = 0; i < num_samples; i++) {
			float in_samples[USB_AUDIO_CHANNELS];
      float dsp_out[2];
			UN32 out_samples[2];

      for (int j = 0; j < USB_AUDIO_CHANNELS; j++) {
        in_samples[j] = s24_to_float(&tmpbuf[tmpbuf_ptr + (j * USB_AUDIO_BYTES_PER_SAMPLE)]);
      }

      dsp_out[0] = (in_samples[0] * logChannelLevels[0]) + (in_samples[2] * logChannelLevels[1]);
      dsp_out[1] = (in_samples[1] * logChannelLevels[0]) + (in_samples[3] * logChannelLevels[1]);

      for (int j = 0; j < 2; j++) {
        out_samples[j].s = float_to_s32(dsp_out[j]);
        out_samples[j].s = USBD_AUDIO_Volume_Ctrl(out_samples[j].s,haudio->vol_3dB_shift);

        haudio->buffer[haudio->wr_ptr++] = (((uint16_t)out_samples[j].b[2]) << 8) | (uint16_t)out_samples[j].b[1];
        haudio->buffer[haudio->wr_ptr++] = ((uint16_t)out_samples[j].b[0]) << 8;
      }

			tmpbuf_ptr += sample_size;

			// Rollover at end of buffer
			if (haudio->wr_ptr >= AUDIO_TOTAL_BUF_SIZE) {
				haudio->wr_ptr = 0U;
				}
			}

		// Start playing when half of the audio buffer is filled
		// so if you increase the buffer length too much, the audio latency will be obvious when watching video+audio
		if (haudio->offset == AUDIO_OFFSET_UNKNOWN && is_playing == 0U) {
			if (haudio->wr_ptr >= AUDIO_TOTAL_BUF_SIZE / 2U) {
				haudio->offset = AUDIO_OFFSET_NONE;
				is_playing = 1U;

				if (haudio->rd_enable == 0U) {
					haudio->rd_enable = 1U;
					// Set last writable buffer size to actual value. Note that rd_ptr is 0 now.
					audio_buf_writable_samples_last = (AUDIO_TOTAL_BUF_SIZE - haudio->wr_ptr)/sample_size;
        }

				((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->AudioCmd(&haudio->buffer[0], AUDIO_TOTAL_BUF_SIZE * 2, AUDIO_CMD_START);
      }
    }

		USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, tmpbuf, AUDIO_OUT_PACKET_24B);
		}

	return USBD_OK;
}

/**
 * @brief  AUDIO_Req_GetCurrent
 *         Handles the GET_CUR Audio control request.
 * @param  pdev: instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  if ((req->bmRequest & 0x1f) == AUDIO_CONTROL_REQ) {
    switch (HIBYTE(req->wValue)) {
      case AUDIO_CONTROL_REQ_FU_MUTE: {
        // Current mute state
        USBD_CtlSendData(pdev, &(haudio->mute), 1);
      };
          break;
      case AUDIO_CONTROL_REQ_FU_VOL: {
        // Current volume. See USB Device Class Defintion for Audio Devices v1.0 p.77
        USBD_CtlSendData(pdev, (uint8_t*)&haudio->volume, 2);
      };
          break;
    }
  } else if ((req->bmRequest & 0x1f) == AUDIO_STREAMING_REQ) {
    if (HIBYTE(req->wValue) == AUDIO_STREAMING_REQ_FREQ_CTRL) {
      // Current frequency
      uint32_t freq __attribute__((aligned(4))) = haudio->freq;
      USBD_CtlSendData(pdev, (uint8_t*)&freq, 3);
    }
  }
}


/**
 * @brief  AUDIO_Req_GetMax
 *         Handles the GET_MAX Audio control request.
 * @param  pdev: instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_GetMax(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req)
{
  if ((req->bmRequest & 0x1f) == AUDIO_CONTROL_REQ) {
    switch (HIBYTE(req->wValue)) {
      case AUDIO_CONTROL_REQ_FU_VOL: {
        int16_t vol_max = (int16_t)USBD_AUDIO_VOL_MAX;
        USBD_CtlSendData(pdev, (uint8_t*)&vol_max, 2);
      };
          break;
    }
  }
}


/**
 * @brief  AUDIO_Req_GetMin
 *         Handles the GET_MIN Audio control request.
 * @param  pdev: instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_GetMin(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req)
{
  if ((req->bmRequest & 0x1f) == AUDIO_CONTROL_REQ) {
    switch (HIBYTE(req->wValue)) {
      case AUDIO_CONTROL_REQ_FU_VOL: {
        int16_t vol_min = (int16_t)USBD_AUDIO_VOL_MIN;
        USBD_CtlSendData(pdev, (uint8_t*)&vol_min, 2);
      };
          break;
    }
  }
}


/**
 * @brief  AUDIO_Req_GetRes
 *         Handles the GET_RES Audio control request.
 * @param  pdev: instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_GetRes(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req)
{
  if ((req->bmRequest & 0x1f) == AUDIO_CONTROL_REQ) {
    switch (HIBYTE(req->wValue)) {
      case AUDIO_CONTROL_REQ_FU_VOL: {
        int16_t vol_res = (int16_t)USBD_AUDIO_VOL_STEP;
        USBD_CtlSendData(pdev, (uint8_t*)&vol_res, 2);
      };
          break;
    }
  }
}


/**
  * @brief  AUDIO_Req_SetCurrent
  *         Handles the SET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  if (req->wLength) {
    /* Prepare the reception of the buffer over EP0 */
    USBD_CtlPrepareRx(pdev,
                      haudio->control.data,
                      req->wLength);

    haudio->control.cmd = AUDIO_REQ_SET_CUR;          /* Set the request value */
    haudio->control.req_type = req->bmRequest & 0x1f; /* Set the request type. See UAC Spec 1.0 - 5.2.1 Request Layout */
    haudio->control.len = (uint8_t)req->wLength;      /* Set the request data length */
    haudio->control.unit = HIBYTE(req->wIndex);       /* Set the request target unit */
    haudio->control.cs = HIBYTE(req->wValue);         /* Set the request control selector (high byte) */
    haudio->control.cn = LOBYTE(req->wValue);         /* Set the request control number (low byte) */
  }
}


/**
  * @brief  USBD_AUDIO_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef* pdev)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  if (haudio->control.cmd == AUDIO_REQ_SET_CUR) { /* In this driver, to simplify code, only SET_CUR request is managed */

    if (haudio->control.req_type == AUDIO_CONTROL_REQ) {
      switch (haudio->control.cs) {
        // Mute Control
        case AUDIO_CONTROL_REQ_FU_MUTE: {
        	haudio->mute = haudio->control.data[0];
          ((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->MuteCtl(haudio->control.data[0]);
        };
            break;
        // Volume Control
        case AUDIO_CONTROL_REQ_FU_VOL: {
          int16_t volume = *(int16_t*)&haudio->control.data[0];
          haudio->volume = volume;
          haudio->vol_3dB_shift = USBD_AUDIO_Get_Vol3dB_Shift(volume);
          ((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->VolumeCtl(volume);
        };
            break;
      }

    } else if (haudio->control.req_type == AUDIO_STREAMING_REQ) {
      // Frequency Control
      if (haudio->control.cs == AUDIO_STREAMING_REQ_FREQ_CTRL) {
        uint32_t new_freq = *(uint32_t*)&haudio->control.data & 0x00ffffff;

        if (haudio->freq != new_freq) {
          haudio->freq = new_freq;
          AUDIO_OUT_Restart(pdev);
        }
      }
    }

    haudio->control.req_type = 0U;
    haudio->control.cs = 0U;
    haudio->control.cn = 0U;
    haudio->control.cmd = 0U;
    haudio->control.len = 0U;
  }

  return USBD_OK;
}


/**
  * @brief  USBD_AUDIO_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef* pdev)
{
  /* Only OUT control data are processed */
  return USBD_OK;
}


/**
 * @brief  Stop playing and reset buffer pointers
 * @param  pdev: instance
 */
static void AUDIO_OUT_StopAndReset(USBD_HandleTypeDef* pdev)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  all_ready = 0U;
  tx_flag = 1U;
  is_playing = 0U;
  audio_buf_writable_samples_last = AUDIO_TOTAL_BUF_SIZE /(2*6);
#ifdef DEBUG_FEEDBACK_ENDPOINT
  DbgMinWritableSamples = 99999;
  DbgMaxWritableSamples = 0;
  DbgIndex = 0;
  DbgSofCounter = 0;
#endif
  haudio->offset = AUDIO_OFFSET_UNKNOWN;
  haudio->rd_enable = 0U;
  haudio->rd_ptr = 0U;
  haudio->wr_ptr = 0U;

  USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
  USBD_LL_FlushEP(pdev, AUDIO_OUT_EP);

  ((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->DeInit(0);
}


/**
 * @brief  Restart playing with new parameters
 * @param  pdev: instance
 */
static void AUDIO_OUT_Restart(USBD_HandleTypeDef* pdev)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  AUDIO_OUT_StopAndReset(pdev);

  switch (haudio->freq) {
    case 44100:
      fb_nom = fb_value = I2S_Clk_Config24[0].nominal_fdbk;
      break;
    case 48000:
      fb_nom = fb_value = I2S_Clk_Config24[1].nominal_fdbk;
      break;
    case 96000:
    default :
      fb_nom = fb_value = I2S_Clk_Config24[2].nominal_fdbk;
      break;
  }

  ((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->Init(haudio->freq, haudio->volume, haudio->mute);

  tx_flag = 0U;
  all_ready = 1U;
}


/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t* USBD_AUDIO_GetDeviceQualifierDesc(uint16_t* length)
{
  *length = sizeof(USBD_AUDIO_DeviceQualifierDesc);
  return USBD_AUDIO_DeviceQualifierDesc;
}


/**
* @brief  USBD_AUDIO_RegisterInterface
* @param  fops: Audio interface callback
* @retval status
*/
uint8_t USBD_AUDIO_RegisterInterface(USBD_HandleTypeDef* pdev,
                                     USBD_AUDIO_ItfTypeDef* fops)
{
  if (fops != NULL) {
    pdev->pUserData = fops;
  }
  return USBD_OK;
}





