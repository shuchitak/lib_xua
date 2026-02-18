// Copyright 2011-2026 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
/**
 * @file xua_audiohub.xc
 * @brief XMOS USB 2.0 Audio Reference Design.  Audio Functions.
 * @author Ross Owen, XMOS Semiconductor Ltd
 *
 * This thread handles I2S and forwards samples to the SPDIF Tx core.
 * Additionally this thread handles clocking and CODEC/DAC/ADC config.
 **/

#include <syscall.h>
#include <platform.h>
#include <xs1.h>
#include <xclib.h>
#include <xs1_su.h>
#include <string.h>
#include <xassert.h>

#include "xua.h"

#include "audiohw.h"
#include "audioports.h"
#if (XUA_SPDIF_TX_EN)
#include "spdif.h"
#endif
#if (XUA_ADAT_TX_EN)
#include "adat_tx.h"
#ifndef ADAT_TX_USE_SHARED_BUFF
#error Designed for ADAT tx shared buffer mode ONLY
#endif
#endif

#if (XUA_NUM_PDM_MICS > 0)
#include "xua_pdm_mic.h"
#if (MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME != 1)
#error Only sample based interface supported between mic array and XUA
#endif
#endif

#if (AUD_TO_USB_RATIO > 1)
#include "src.h"
#endif

#include "xua_commands.h"
#include "xc_ptr.h"

#define DEBUG_UNIT XUA_AUDIOHUB
#include "debug_print.h"

#ifndef _XUA_ENABLE_I2S_TIMING_CHECK
    #define _XUA_ENABLE_I2S_TIMING_CHECK (0)
#endif

#define OUT_CHAN_COUNT (I2S_CHANS_DAC + (8*XUA_ADAT_TX_EN) + (2*XUA_SPDIF_TX_EN))
unsigned samplesOut[XUA_MAX(NUM_USB_CHAN_OUT, OUT_CHAN_COUNT)];

/* Two buffers for ADC data to allow for DAC and ADC I2S ports being offset */
#define IN_CHAN_COUNT (I2S_CHANS_ADC + XUA_NUM_PDM_MICS + (8*XUA_ADAT_RX_EN) + (2*XUA_SPDIF_RX_EN))

unsigned samplesIn[2][XUA_MAX(NUM_USB_CHAN_IN, IN_CHAN_COUNT)];

#if (XUA_ADAT_TX_EN)
extern buffered out port:32 p_adat_tx;
#endif

#if (XUA_ADAT_TX_EN)
extern clock    clk_mst_spd;
#endif

#if CODEC_MASTER
void InitPorts_slave
#else
void InitPorts_master
#endif
(buffered _XUA_CLK_DIR port:32 p_lrclk, buffered _XUA_CLK_DIR port:32 p_bclk, buffered out port:32 (&?p_i2s_dac)[I2S_WIRES_DAC],
    buffered in port:32  (&?p_i2s_adc)[I2S_WIRES_ADC]);


/***********************************/
#include "xua.h"
#if (XUA_DFU_EN== 1)
#include <xs1.h>
#include <platform.h>

#if XUA_USB_EN
#include "xud_device.h"
#include "dfu_types.h"
#include "flash_interface.h"
#include "dfu_interface.h"

#if defined(__XS2A__)
/* Note range 0x7FFC8 - 0x7FFFF guarenteed to be untouched by tools */
#define FLAG_ADDRESS 0x7ffcc
#else
/* Note range 0xFFFC8 - 0xFFFFF guarenteed to be untouched by tools */
#define FLAG_ADDRESS 0xfffcc
#endif

#define DEBUG_MEMORY_LOG_ENABLED 0
#ifdef DEBUG_MEMORY_LOG_ENABLED
    #include <stdlib.h>
    unsigned int debug_memory_log_buffer_index = 0;
    #define DEBUG_MEMORY_LOG_BUFFER_SIZE 2048
    unsigned char debug_memory_log_buffer[DEBUG_MEMORY_LOG_BUFFER_SIZE];
    // Override the weak symbol used for print messages
    int _write(int fd, const unsigned char data[], size_t len) {
        // Check for wrap of the circular buffer
        if ((debug_memory_log_buffer_index + len + 1) > DEBUG_MEMORY_LOG_BUFFER_SIZE)
        debug_memory_log_buffer_index = 0;
        // Copy write message into log buffer
        for(unsigned int i = 0; i < len; i++) {
        debug_memory_log_buffer[debug_memory_log_buffer_index] = data[i];
        debug_memory_log_buffer_index++;
        }
        // Terminate the whole buffer after the current message
        debug_memory_log_buffer[debug_memory_log_buffer_index] = '\0';
        return len;
    }
#endif

#define _BOOT_DFU_MODE_FLAG (0x11042011)

/* Store Flag to fixed address */
void SetDFUFlag(unsigned x)
{
    asm volatile("stw %0, %1[0]" :: "r"(x), "r"(FLAG_ADDRESS));
}

/* Load flag from fixed address */
static unsigned GetDFUFlag()
{
    unsigned x;
    asm volatile("ldw %0, %1[0]" : "=r"(x) : "r"(FLAG_ADDRESS));
    return x;
}

static int g_DFU_state = STATE_APP_IDLE;
static int DFU_status = DFU_OK;
static timer DFUTimer;
static unsigned int DFUTimerStart = 0;
static unsigned int DFUResetTimeout = 100000000; // 1 second default
static int DFU_flash_connected = 0;

static unsigned int subPagesLeft = 0;
static int flash_cmd_start_write_image_in_progress = 1;

extern void DFUCustomFlashEnable();
extern void DFUCustomFlashDisable();

static unsigned int save_blk0_request_data[_DFU_TRANSFER_SIZE_WORDS];

void DFUDelay(unsigned d)
{
    timer tmr;
    unsigned s;
    tmr :> s;
    tmr when timerafter(s + d) :> void;
}

/* Return non-zero on error */
static int DFU_OpenFlash()
{
	if (!DFU_flash_connected)
	{
        unsigned int cmd_data[_DFU_TRANSFER_SIZE_WORDS];
        DFUCustomFlashEnable();
        int error = flash_cmd_init();
        if(error)
        {
            return error;
        }

    	DFU_flash_connected = 1;
  	}

  	return 0;
}

static int DFU_CloseFlash(chanend ?c_user_cmd)
{
    if (DFU_flash_connected)
    {
        unsigned int cmd_data[_DFU_TRANSFER_SIZE_WORDS];
        DFUCustomFlashDisable();
        flash_cmd_deinit();
        DFU_flash_connected = 0;
    }
    return 0;
}

static int DFU_Dnload(unsigned int request_len, unsigned int block_num, const unsigned request_data[_DFU_TRANSFER_SIZE_WORDS], chanend ?c_user_cmd, int &return_data_len, unsigned &DFU_state)
{
    unsigned int fromDfuIdle = 0;
    return_data_len = 0;
    int error;
    // Get DFU packets here, sequence is
    // DFU_DOWNLOAD -> DFU_DOWNLOAD_SYNC
    // GET_STATUS -> DFU_DOWNLOAD_SYNC (flash busy) || DFU_DOWNLOAD_IDLE
    // REPEAT UNTIL DFU_DOWNLOAD with 0 length -> DFU_MANIFEST_SYNC

    if((error = DFU_OpenFlash()))
    {
        return error;
    }

    switch (DFU_state)
    {
        case STATE_DFU_IDLE:
        case STATE_DFU_DOWNLOAD_IDLE:
            break;
        default:
            DFU_state = STATE_DFU_ERROR;
            return 1;
    }

    if ((DFU_state == STATE_DFU_IDLE) && (request_len == 0))
    {
        DFU_state = STATE_DFU_ERROR;
        return 1;
    }
    else if (DFU_state == STATE_DFU_IDLE)
    {
        fromDfuIdle = 1;
    }
    else
    {
        fromDfuIdle = 0;
    }

    if (request_len == 0)
    {
        // Host signalling complete download
        if (subPagesLeft)
        {
            unsigned int subPagePad[_DFU_TRANSFER_SIZE_WORDS] = {0};
            for (unsigned i = 0; i < subPagesLeft; i++)
            {
                flash_cmd_write_page_data((subPagePad, unsigned char[_DFU_TRANSFER_SIZE_BYTES]));
            }
        }
        flash_cmd_end_write_image();
        DFU_state = STATE_DFU_MANIFEST_SYNC;
    }
    else
    {
        DFU_state = STATE_DFU_DOWNLOAD_SYNC; //from the spec. dfuDNLOAD-SYNC = Device has received a block and is waiting for the host to
        // solicit the status via DFU_GETSTATUS. So if the host were to do a GetState right after this, it should see the device state as STATE_DFU_DOWNLOAD_SYNC.
        // That is why, even when flash_cmd_start_write_image() returns not complete, we don't transition to STATE_DFU_DOWNLOAD_BUSY at this point but do it only
        // from DFU_GetStatus()
        if (!(block_num % _NUM_DFU_PAGES_PER_FLASH_PAGE)) // Every 4th block
        {
            flash_cmd_reset_subpage_index();
            subPagesLeft = _NUM_DFU_PAGES_PER_FLASH_PAGE;
            if (fromDfuIdle) // Only relevant for block 0 which is when fromDfuIdle is also true
            {
                // Erase flash on block 0
                flash_cmd_erase_all();

                flash_cmd_start_write_image_in_progress = flash_cmd_start_write_image();

                if(flash_cmd_start_write_image_in_progress) // flash_cmd_start_write_image() still in progress
                {
                    for (unsigned i = 0; i < _DFU_TRANSFER_SIZE_WORDS; i++)
                    {
                        save_blk0_request_data[i] = request_data[i]; // save block 0 request data to be written to flash once flash_cmd_start_write_image() is complete
                    }
                    return 0; // return from here. We only write block 0 to flash once flash_cmd_start_write_image() completes.
                    //Further checks for flash_cmd_start_write_image() completion and subsequent writing of block 0 to flash happen in DFU_GetStatus()
                }
            }
        }

        unsigned int cmd_data[_DFU_TRANSFER_SIZE_WORDS];
        for (unsigned i = 0; i < _DFU_TRANSFER_SIZE_WORDS; i++)
        {
            cmd_data[i] = request_data[i];
        }
        flash_cmd_write_page_data((cmd_data, unsigned char[_DFU_TRANSFER_SIZE_BYTES]));
        subPagesLeft--;
    }

    return 0;
}


static int DFU_Upload(unsigned int request_len, unsigned int block_num, unsigned data_out[_DFU_TRANSFER_SIZE_WORDS], unsigned &DFU_state)
{
    unsigned int cmd_data[1];
    unsigned int firstRead = 0;

    // Start at flash address 0
    // Keep reading flash pages until read_page returns 1 (address out of range)
    // Return terminating upload packet at this point
    DFU_OpenFlash();

    switch (DFU_state)
    {
        case STATE_DFU_IDLE:
        case STATE_DFU_UPLOAD_IDLE:
            break;
        default:
            DFU_state = STATE_DFU_ERROR;
            return 0;
    }

    if ((DFU_state == STATE_DFU_IDLE) && (request_len == 0))
    {
        DFU_state = STATE_DFU_ERROR;
        return 0;
    }
    else if (DFU_state == STATE_DFU_IDLE)
    {
        firstRead = 1;
        subPagesLeft = 0;
    }

    if (!subPagesLeft)
    {
        cmd_data[0] = !firstRead;

        // Read whole (256bytes) page from the image on the flash into a memory buffer
        flash_cmd_read_page((cmd_data, unsigned char[1]));
        subPagesLeft = _NUM_DFU_PAGES_PER_FLASH_PAGE;

        // If address out of range, terminate!
        if (cmd_data[0] == 1)
        {
            subPagesLeft = 0;
            // Back to idle state, upload complete
            DFU_state = STATE_DFU_IDLE;
            return 0;
        }
    }

    // Get _DFU_TRANSFER_SIZE_BYTES bytes of page data from memory
    flash_cmd_read_page_data((data_out, unsigned char[_DFU_TRANSFER_SIZE_BYTES]));

    subPagesLeft--;

    DFU_state = STATE_DFU_UPLOAD_IDLE;

    return _DFU_TRANSFER_SIZE_BYTES;
}

#define GET_STATUS_POLL_TIMEOUT_MS     (400)    // Erasing 512*1024 bytes of flash requires about 26 instances of the device returning STATE_DFU_DOWNLOAD_BUSY
static unsigned transition_dfu_download_state()
{
    if(!flash_cmd_start_write_image_in_progress) // If flash_cmd_start_write_image() is done, transition to IDLE since the actual flash writes (flash_cmd_write_page_data) are synchronous
    {
        return STATE_DFU_DOWNLOAD_IDLE;
    }
    else
    {
        timer tmr;
        unsigned time;
        tmr :> time;
        unsigned end_time = time + (XS1_TIMER_KHZ * GET_STATUS_POLL_TIMEOUT_MS);

        while(timeafter(end_time, time)) // Erase as many sectors as we can in GET_STATUS_POLL_TIMEOUT_MS time duration
        {
            if(!flash_cmd_start_write_image_in_progress)
            {
                break;
            }
            flash_cmd_start_write_image_in_progress = flash_cmd_start_write_image();
            tmr :> time;
        }

        if(!flash_cmd_start_write_image_in_progress)
        {
            // Write block 0 to flash
            flash_cmd_write_page_data((save_blk0_request_data, unsigned char[_DFU_TRANSFER_SIZE_BYTES]));
            subPagesLeft--;
            return STATE_DFU_DOWNLOAD_IDLE;
        }
        else // Continue to wait for flash_cmd_start_write_image() to complete
        {
            return STATE_DFU_DOWNLOAD_BUSY;
        }

    }

}

static int DFU_GetStatus(unsigned int request_len, unsigned data_buffer[_DFU_TRANSFER_SIZE_WORDS], chanend ?c_user_cmd, unsigned &DFU_state)
{
    unsigned int timeout = 0;

    data_buffer[0] = (timeout << 8) | (unsigned char)DFU_status;

    switch (DFU_state)
    {
        case STATE_DFU_MANIFEST:
        case STATE_DFU_MANIFEST_WAIT_RESET:
            DFU_state = STATE_DFU_ERROR;
            break;
        case STATE_DFU_DOWNLOAD_BUSY:
        case STATE_DFU_DOWNLOAD_SYNC:
            DFU_state = transition_dfu_download_state();
            break;
        case STATE_DFU_MANIFEST_SYNC:
            // Check if complete here
            DFU_state = STATE_DFU_IDLE;
            break;
        default:
            break;
    }

    data_buffer[1] = DFU_state;

    return 6;

}

static int DFU_ClrStatus(unsigned &DFU_state)
{
    if (DFU_state == STATE_DFU_ERROR)
    {
        DFU_state = STATE_DFU_IDLE;
    }
    else
    {
        DFU_state = STATE_DFU_ERROR;
    }
    return 0;
}

static int DFU_GetState(unsigned int request_len, unsigned int request_data[_DFU_TRANSFER_SIZE_WORDS], chanend ?c_user_cmd, unsigned &DFU_state)
{
    request_data[0] = DFU_state;

    switch (DFU_state)
    {
        case STATE_DFU_DOWNLOAD_BUSY:
        case STATE_DFU_MANIFEST:
        case STATE_DFU_MANIFEST_WAIT_RESET:
            DFU_state = STATE_DFU_ERROR;
            break;
        default:
        break;
    }

    return 1;
}

static int DFU_Abort(unsigned &DFU_state)
{
    DFU_state = STATE_DFU_IDLE;
    return 0;
}

// Tell the DFU state machine that a USB reset has occured
int DFUReportResetState(chanend ?c_user_cmd)
{
    unsigned int inDFU = 0;
    unsigned int currentTime = 0;

    unsigned flag;
    flag = GetDFUFlag();

//#define START_IN_DFU 1
#ifdef START_IN_DFU
    flag = _BOOT_DFU_MODE_FLAG;
#endif

    if (flag == _BOOT_DFU_MODE_FLAG)
    {
        unsigned int cmd_data[_DFU_TRANSFER_SIZE_WORDS];
        inDFU = 1;
        g_DFU_state = STATE_DFU_IDLE;
        return inDFU;
    }

    switch(g_DFU_state)
    {
        case STATE_APP_DETACH:
        case STATE_DFU_IDLE:
            g_DFU_state = STATE_DFU_IDLE;

            DFUTimer :> currentTime;
            if (currentTime - DFUTimerStart > DFUResetTimeout)
            {
                g_DFU_state = STATE_APP_IDLE;
                inDFU = 0;
            }
            else
            {
                inDFU = 1;
            }
            break;
        case STATE_APP_IDLE:
        case STATE_DFU_DOWNLOAD_SYNC:
        case STATE_DFU_DOWNLOAD_BUSY:
        case STATE_DFU_DOWNLOAD_IDLE:
        case STATE_DFU_MANIFEST_SYNC:
        case STATE_DFU_MANIFEST:
        case STATE_DFU_MANIFEST_WAIT_RESET:
        case STATE_DFU_UPLOAD_IDLE:
        case STATE_DFU_ERROR:
            inDFU = 0;
            g_DFU_state = STATE_APP_IDLE;
            break;
        default:
            g_DFU_state = STATE_DFU_ERROR;
            inDFU = 1;
        break;
    }

    if (!inDFU)
    {
        DFU_CloseFlash(c_user_cmd);
    }

    return inDFU;
}

static int XMOS_DFU_RevertFactory(chanend ?c_user_cmd)
{
    unsigned s = 0;

    DFU_OpenFlash();

    flash_cmd_erase_all();

    DFUTimer :> s;
    DFUTimer when timerafter(s + 25000000) :> s; // Wait for flash erase

    return 0;
}

static int XMOS_DFU_SelectImage(unsigned int index, chanend ?c_user_cmd)
{
    // Select the image index for firmware update
    // Currently not used or implemented
    return 0;
}

[[distributable]]
void DFUHandler(server interface i_dfu i, chanend ?c_user_cmd)
{
    //printstrln("DFUHandler");
    while(1)
    {
        select
        {
            case i.HandleDfuRequest(USB_SetupPacket_t &sp, unsigned data_buffer[], unsigned data_buffer_length, unsigned dfuState)
                -> {unsigned reset_device_after_ack, int return_data_len, int dfu_reset_override, int returnVal, unsigned newDfuState}:

                reset_device_after_ack = 0;
                return_data_len = 0;
                dfu_reset_override = 0;
                unsigned tmpDfuState = dfuState;
                returnVal = 0;
                printstrln("DFUHandler HandleDfuRequest");
                // Map Standard DFU commands onto device level firmware upgrade mechanism
                switch (sp.bRequest)
                {
                    case DFU_DETACH:
                        if(dfuState == STATE_APP_IDLE)
                        {
                            dfu_reset_override = _BOOT_DFU_MODE_FLAG; // Reboot in DFU mode
                        }
                        else
                        {
                            // We expect to come here only in the STATE_DFU_IDLE state but to be safe,
                            // in every state other than APP_IDLE, reboot in APP mode.
                            dfu_reset_override = 0;
                        }
                        reset_device_after_ack = 1;
                        return_data_len = 0;
                        break;

                    case DFU_DNLOAD:
                        unsigned data[_DFU_TRANSFER_SIZE_WORDS];
                        for(int i = 0; i < _DFU_TRANSFER_SIZE_WORDS; i++)
                            data[i] = data_buffer[i];
                        returnVal = DFU_Dnload(sp.wLength, sp.wValue, data, c_user_cmd, return_data_len, tmpDfuState);
                        break;

                    case DFU_UPLOAD:
                        unsigned data_out[_DFU_TRANSFER_SIZE_WORDS];
                        return_data_len = DFU_Upload(sp.wLength, sp.wValue, data_out, tmpDfuState);
                        for(int i = 0; i < _DFU_TRANSFER_SIZE_WORDS; i++)
                            data_buffer[i] = data_out[i];
                        break;

                    case DFU_GETSTATUS:
                        //printstrln("GETSTATUS");
                        unsigned data_out[_DFU_TRANSFER_SIZE_WORDS];
                        return_data_len = DFU_GetStatus(sp.wLength, data_out, c_user_cmd, tmpDfuState);
                        for(int i = 0; i < _DFU_TRANSFER_SIZE_WORDS; i++)
                            data_buffer[i] = data_out[i];
                        break;

                    case DFU_CLRSTATUS:
                        return_data_len = DFU_ClrStatus(tmpDfuState);
                        break;

                    case DFU_GETSTATE:
                        unsigned data_out[_DFU_TRANSFER_SIZE_WORDS];
                        return_data_len = DFU_GetState(sp.wLength, data_out, c_user_cmd, tmpDfuState);
                        for(int i = 0; i < _DFU_TRANSFER_SIZE_WORDS; i++)
                            data_buffer[i] = data_out[i];
                        break;

                    case DFU_ABORT:
                        return_data_len = DFU_Abort(tmpDfuState);
                        break;

                    /* XMOS Custom DFU requests */
                    case XMOS_DFU_RESETDEVICE:
                        reset_device_after_ack = 1;
                        return_data_len = 0;
                        break;

                    case XMOS_DFU_REVERTFACTORY:
                        return_data_len = XMOS_DFU_RevertFactory(c_user_cmd);
                        break;

                    case XMOS_DFU_RESETINTODFU:
                        reset_device_after_ack = 1;
                        dfu_reset_override = _BOOT_DFU_MODE_FLAG;
                        return_data_len = 0;
                        break;

                    case XMOS_DFU_RESETFROMDFU:
                        reset_device_after_ack = 1;
                        dfu_reset_override = 0;
                        return_data_len = 0;
                        break;

                    case XMOS_DFU_SELECTIMAGE:
                        return_data_len = XMOS_DFU_SelectImage(sp.wValue, c_user_cmd);
                        break;

                    default:
                        returnVal = XUD_RES_ERR; // Unrecognised request
                        break;
                }
				newDfuState = tmpDfuState;
                break;

           case i.finish():
                return;
        }
    }
}

int DFUDeviceRequests(XUD_ep ep0_out, XUD_ep &?ep0_in, USB_SetupPacket_t &sp, chanend ?c_user_cmd, unsigned int altInterface, client interface i_dfu i,int &reset)
{
    unsigned int return_data_len = 0;
    unsigned int data_buffer_len = 0;
    unsigned int data_buffer[17];
    unsigned int reset_device_after_ack = 0;
    int returnVal = 0;
    unsigned int dfuState = g_DFU_state;
    int dfuResetOverride;

    if(sp.bmRequestType.Direction == USB_BM_REQTYPE_DIRECTION_H2D)
    {
        // Host to device
        if (sp.wLength)
            XUD_GetBuffer(ep0_out, (data_buffer, unsigned char[]), data_buffer_len);
    }
    /* Interface used here such that the handler can be on another tile */
    printstrln("Call HandleDfuRequest");
    printintln(sp.bRequest);
    {reset_device_after_ack, return_data_len, dfuResetOverride, returnVal, dfuState} = i.HandleDfuRequest(sp, data_buffer, data_buffer_len, g_DFU_state);
    printstrln("After HandleDfuRequest");
    SetDFUFlag(dfuResetOverride);

    /* Update our version of dfuState */
    g_DFU_state = dfuState;

    /* Check if the request was handled */
    if(returnVal == 0)
    {
        if (sp.bmRequestType.Direction == USB_BM_REQTYPE_DIRECTION_D2H && sp.wLength != 0)
        {
            //printstrln("XUD_DoGetRequest");
            returnVal = XUD_DoGetRequest(ep0_out, ep0_in, (data_buffer, unsigned char[]), return_data_len, return_data_len);
        }
        else
        {
            //printstrln("XUD_DoSetRequestStatus");
            returnVal = XUD_DoSetRequestStatus(ep0_in);
        }

  	    // If device reset requested, handle after command acknowledgement
  	    if (reset_device_after_ack)
  	    {
  	        reset = 1;
        }
    }
  	return returnVal;
}
#endif
#endif
/***********************************/





unsigned dsdMode = DSD_MODE_OFF;

#if (DSD_CHANS_DAC != 0) && (NUM_USB_CHAN_OUT > 0)
#include "audiohub_dsd.h"
#endif

#if (XUA_ADAT_TX_EN)
#include "audiohub_adat.h"
#endif
#include "xua_audiohub_st.h"

static inline int HandleSampleClock(int frameCount, buffered _XUA_CLK_DIR port:32 p_lrclk, int first_frame)
{
#if CODEC_MASTER
    unsigned syncError = 0;
    unsigned lrval = 0;
    const unsigned lrval_mask = (0xffffffff << (32 - XUA_I2S_N_BITS));

    if(XUA_I2S_N_BITS != 32)
    {
        asm volatile("in %0, res[%1]":"=r"(lrval):"r"(p_lrclk):"memory");
        set_port_shift_count(p_lrclk, XUA_I2S_N_BITS);
    }
    else
    {
        p_lrclk :> lrval;
    }

    if(XUA_PCM_FORMAT == XUA_PCM_FORMAT_TDM)
    {
        /* Only check for the rising edge of frame sync being in the right place because falling edge timing not specified */
        if (frameCount == 1)
        {
            lrval &= 0xc0000000;                 // Mask off last two (MSB) frame clock bits which are the most recently sampled
            syncError += (lrval != 0x80000000);  // We need MSB = 1 and MSB-1 = 0 to signify rising edge
        }
        else
        {
            /* We do not check this part of the frame because TDM frame sync falling egde timing
             * is not defined. We only care about rising edge which is checked in first half of frame */
        }
    }
    else
    {
        if(XUA_I2S_N_BITS == 32)
        {
            if(frameCount == 0)
                syncError = (lrval != 0x80000000);
            else
                syncError = (lrval != 0x7FFFFFFF);
        }
        else
        {
            if(frameCount == 0)
                syncError = ((lrval & lrval_mask) != 0x80000000);
            else
                syncError = ((lrval | (~lrval_mask)) != 0x7FFFFFFF);
        }
    }

    return syncError;

#else
    static unsigned short port_ts, prev_port_ts;
    unsigned clkVal;
    if(XUA_PCM_FORMAT == XUA_PCM_FORMAT_TDM)
    {
        if(frameCount == (I2S_CHANS_PER_FRAME-1))
            clkVal = 0x80000000;
        else
            clkVal = 0x00000000;
    }
    else
    {
        if(frameCount == 0)
            clkVal = 0x80000000;
        else
            clkVal = 0x7fffffff;
    }

    if(XUA_I2S_N_BITS == 32)
    {
        p_lrclk <: clkVal;
        asm volatile(" getts %0, res[%1]" : "=r" (port_ts) : "r" (p_lrclk));
    }
    else
    {
        partout(p_lrclk, XUA_I2S_N_BITS, clkVal >> (32 - XUA_I2S_N_BITS));
    }

    if(!first_frame)
    {
        unsigned short diff = port_ts - prev_port_ts;
#if _XUA_ENABLE_I2S_TIMING_CHECK
        asm volatile("ecallf %0":: "r" (diff == 32));
#endif
        //xassert((diff == 32)); // compiling with asserts enabled has a separate set of problems, even without this check (sw_usb_audio issue 340)
    }
    prev_port_ts = port_ts;

    return 0;
#endif

}

#pragma unsafe arrays
unsigned static AudioHub_MainLoop(chanend ?c_aud, chanend ?c_spd_out
#if (XUA_ADAT_TX_EN)
    , chanend c_adat_out
    , unsigned adatSmuxMode
#endif
    , unsigned divide, unsigned curSamFreq
#if (XUA_SPDIF_RX_EN || XUA_ADAT_RX_EN)
    , chanend c_dig_rx
#endif
#if (XUA_NUM_PDM_MICS > 0)
    , chanend c_pdm_pcm
#endif
    , buffered _XUA_CLK_DIR port:32 ?p_lrclk,
    buffered _XUA_CLK_DIR port:32 ?p_bclk,
    buffered out port:32 (&?p_i2s_dac)[I2S_WIRES_DAC],
    buffered in port:32  (&?p_i2s_adc)[I2S_WIRES_ADC]
)
{
    /* Since DAC and ADC buffered ports off by one sample we buffer previous ADC frame */
    unsigned readBuffNo = 0;
    unsigned index;

#if (DSD_CHANS_DAC != 0)
    unsigned dsdMarker = DSD_MARKER_2;    /* This alternates between DSD_MARKER_1 and DSD_MARKER_2 */
    int dsdCount = 0;
    int everyOther = 1;
    unsigned dsdSample_l = 0x96960000;
    unsigned dsdSample_r = 0x96960000;
#endif
    unsigned underflowWord = 0;

#if (XUA_ADAT_TX_EN)
    adatCounter = 0;
#endif

#if(DSD_CHANS_DAC != 0)
    if(dsdMode == DSD_MODE_DOP)
    {
        underflowWord = 0xFA969600;
    }
    else if(dsdMode == DSD_MODE_NATIVE)
    {
        underflowWord = 0x96969696;
    }
#endif

    unsigned audioToUsbRatioCounter = 0;
#if (XUA_NUM_PDM_MICS > 0)
    unsigned audioToMicsRatioCounter = 0;
#endif

#if (AUD_TO_USB_RATIO > 1)
    union i2sInDs3
    {
        long long doubleWordAlignmentEnsured;
        int32_t delayLine[I2S_DOWNSAMPLE_CHANS_IN][SRC_FF3V_FIR_NUM_PHASES][SRC_FF3V_FIR_TAPS_PER_PHASE];
    } i2sInDs3;
    memset(&i2sInDs3.delayLine, 0, sizeof i2sInDs3.delayLine);
    int64_t i2sInDs3Sum[I2S_DOWNSAMPLE_CHANS_IN];

    union i2sOutUs3
    {
        long long doubleWordAlignmentEnsured;
        int32_t delayLine[I2S_CHANS_DAC][SRC_FF3V_FIR_TAPS_PER_PHASE];
    } i2sOutUs3;
    memset(&i2sOutUs3.delayLine, 0, sizeof i2sOutUs3.delayLine);
#endif /* (AUD_TO_USB_RATIO > 1) */

    UserBufferManagementInit(curSamFreq);

    /* Get initial samples for first I2S output */
    unsigned command = DoSampleTransfer(c_aud, readBuffNo, underflowWord);

    // Reinitialise user state before entering the main loop
    UserBufferManagementInit(curSamFreq);

#if (XUA_ADAT_TX_EN)
    unsafe{
    //TransferAdatTxSamples(c_adat_out, samplesOut, adatSmuxMode, 0);
    volatile unsigned * unsafe samplePtr = &samplesOut[ADAT_TX_INDEX];
    outuint(c_adat_out, (unsigned) samplePtr);
    }
#endif
    if(command != XUA_AUDCTL_NO_COMMAND)
    {
        return command;
    }

    /* Main Audio I/O loop */
    while (1)
    {
        unsigned syncError = 0;
        unsigned frameCount = 0;
        unsigned skip_ts_check = 1; // Skip I2S port timestamp check for the very first frame

        if ((I2S_CHANS_DAC > 0 || I2S_CHANS_ADC > 0))
        {
#if CODEC_MASTER
            InitPorts_slave(p_lrclk, p_bclk, p_i2s_dac, p_i2s_adc);
#else
            InitPorts_master(p_lrclk, p_bclk, p_i2s_dac, p_i2s_adc);
#endif
        }

        /* Note we always expect syncError to be 0 when we are master */
        while(!syncError)
        {
#if (DSD_CHANS_DAC != 0) && (NUM_USB_CHAN_OUT > 0)
            if(dsdMode == DSD_MODE_NATIVE)
                DoDsdNative(samplesOut, dsdSample_l, dsdSample_r, divide);
            else if(dsdMode == DSD_MODE_DOP)
                DoDsdDop(everyOther, samplesOut, dsdSample_l, dsdSample_r, divide);
            else
#endif
            {
#if (I2S_CHANS_ADC != 0)
#if (AUD_TO_USB_RATIO > 1)
                if (0 == audioToUsbRatioCounter)
                {
                    memset(&i2sInDs3Sum, 0, sizeof i2sInDs3Sum);
                }
#endif /* (AUD_TO_USB_RATIO > 1) */
                /* Input previous L sample into L in buffer */
                index = 0;
                /* First input (i.e. frameCount == 0) we read last ADC channel of previous frame.. */
                unsigned buffIndex = (frameCount > 1) ? !readBuffNo : readBuffNo;

#pragma loop unroll
                /* First time around we get channel 7 of TDM8 */
                for(int i = 0; i < I2S_CHANS_ADC; i+=I2S_CHANS_PER_FRAME)
                {
                    // p_i2s_adc[index++] :> sample;
                    // Manual IN instruction since compiler generates an extra setc per IN (bug #15256)
                    unsigned sample;
                    asm volatile("in %0, res[%1]" : "=r"(sample)  : "r"(p_i2s_adc[index]));

                    sample = bitrev(sample);
                    if(XUA_I2S_N_BITS != 32)
                    {
                        set_port_shift_count(p_i2s_adc[index], XUA_I2S_N_BITS);
                        sample <<= (32 - XUA_I2S_N_BITS);
                    }
                    index++;

                    int chanIndex = ((frameCount-2) & (I2S_CHANS_PER_FRAME-1)) + i; // channels 0, 2, 4.. on each line.

#if (AUD_TO_USB_RATIO > 1)
                    if ((AUD_TO_USB_RATIO - 1) == audioToUsbRatioCounter)
                    {
                        samplesIn[buffIndex][chanIndex] =
                            src_ds3_voice_add_final_sample(
                                i2sInDs3Sum[chanIndex],
                                i2sInDs3.delayLine[chanIndex][audioToUsbRatioCounter],
                                src_ff3v_fir_coefs[audioToUsbRatioCounter],
                                sample);
                    }
                    else
                    {
                        i2sInDs3Sum[chanIndex] =
                            src_ds3_voice_add_sample(
                                i2sInDs3Sum[chanIndex],
                                i2sInDs3.delayLine[chanIndex][audioToUsbRatioCounter],
                                src_ff3v_fir_coefs[audioToUsbRatioCounter],
                                sample);
                    }
#else
                    samplesIn[buffIndex][chanIndex] = sample;
#endif /* (AUD_TO_USB_RATIO > 1) */
                }
#endif

#if (I2S_CHANS_ADC != 0 || I2S_CHANS_DAC != 0)
                syncError += HandleSampleClock(frameCount, p_lrclk, skip_ts_check);
#endif

#pragma xta endpoint "i2s_output_l"

#if (I2S_CHANS_DAC != 0)
                index = 0;
#pragma loop unroll
                /* Output "even" channel to DAC (i.e. left) */
                for(int i = 0; i < I2S_CHANS_DAC; i+=I2S_CHANS_PER_FRAME)
                {
#if (AUD_TO_USB_RATIO > 1)
                    if (0 == audioToUsbRatioCounter)
                    {
                        samplesOut[frameCount+i] = src_us3_voice_input_sample(i2sOutUs3.delayLine[i],
                                                                              src_ff3v_fir_coefs[2],
                                                                              samplesOut[frameCount+i]);
                    }
                    else /* audioToUsbRatioCounter == 1 or 2 */
                    {
                        samplesOut[frameCount+i] = src_us3_voice_get_next_sample(i2sOutUs3.delayLine[i],
                                                                                 src_ff3v_fir_coefs[2-audioToUsbRatioCounter]);
                    }
#endif /* (AUD_TO_USB_RATIO > 1) */
                    if(XUA_I2S_N_BITS == 32)
                        p_i2s_dac[index++] <: bitrev(samplesOut[frameCount +i]);
                    else
                        partout(p_i2s_dac[index++], XUA_I2S_N_BITS, bitrev(samplesOut[frameCount +i]));
                }
#endif // (I2S_CHANS_DAC != 0)

            if(frameCount == 0)
            {
#if (XUA_ADAT_TX_EN)
                TransferAdatTxSamples(c_adat_out, samplesOut, adatSmuxMode, 1);
#endif
#if (XUA_SPDIF_TX_EN) && (NUM_USB_CHAN_OUT > 0)
                outuint(c_spd_out, samplesOut[SPDIF_TX_INDEX]);  /* Forward samples to S/PDIF Tx thread */
                outuint(c_spd_out, samplesOut[SPDIF_TX_INDEX + 1]);
#endif

#if (XUA_SPDIF_RX_EN || XUA_ADAT_RX_EN)
                /* Sync with clockgen */
                inuint(c_dig_rx);

                /* Note, digi-data we just store in samplesIn[readBuffNo] - we only double buffer the I2S input data */
#endif
#if (XUA_SPDIF_RX_EN)
                asm("ldw %0, dp[g_digData]"  :"=r"(samplesIn[readBuffNo][SPDIF_RX_INDEX + 0]));
                asm("ldw %0, dp[g_digData+4]":"=r"(samplesIn[readBuffNo][SPDIF_RX_INDEX + 1]));
#endif
#if (XUA_ADAT_RX_EN)
                asm("ldw %0, dp[g_digData+8]" :"=r"(samplesIn[readBuffNo][ADAT_RX_INDEX]));
                asm("ldw %0, dp[g_digData+12]":"=r"(samplesIn[readBuffNo][ADAT_RX_INDEX + 1]));
                asm("ldw %0, dp[g_digData+16]":"=r"(samplesIn[readBuffNo][ADAT_RX_INDEX + 2]));
                asm("ldw %0, dp[g_digData+20]":"=r"(samplesIn[readBuffNo][ADAT_RX_INDEX + 3]));
                asm("ldw %0, dp[g_digData+24]":"=r"(samplesIn[readBuffNo][ADAT_RX_INDEX + 4]));
                asm("ldw %0, dp[g_digData+28]":"=r"(samplesIn[readBuffNo][ADAT_RX_INDEX + 5]));
                asm("ldw %0, dp[g_digData+32]":"=r"(samplesIn[readBuffNo][ADAT_RX_INDEX + 6]));
                asm("ldw %0, dp[g_digData+36]":"=r"(samplesIn[readBuffNo][ADAT_RX_INDEX + 7]));
#endif

#if (XUA_SPDIF_RX_EN || XUA_ADAT_RX_EN)
                /* Request digital data (with prefill) */
                outuint(c_dig_rx, 0);
#endif

#if (XUA_NUM_PDM_MICS > 0)
                if ((AUD_TO_MICS_RATIO - 1) == audioToMicsRatioCounter)
                unsafe {
                    chanend_t c_m2a = (chanend_t)c_pdm_pcm;
                    int32_t *mic_samps_base_addr = (int32_t*)&samplesIn[readBuffNo][XUA_PDM_MIC_INDEX];
                    ma_frame_rx(mic_samps_base_addr, c_m2a, MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME, MIC_ARRAY_CONFIG_MIC_COUNT);
                    xua_user_pdm_process(mic_samps_base_addr);
                    audioToMicsRatioCounter = 0;
                }
                else
                {
                    ++audioToMicsRatioCounter;
                }
#endif
            }

           frameCount++;

#if (I2S_CHANS_ADC != 0)
                index = 0;
                /* Channels 0, 2, 4.. on each line */
#pragma loop unroll
                for(int i = 0; i < I2S_CHANS_ADC; i += I2S_CHANS_PER_FRAME)
                {
                    /* Manual IN instruction since compiler generates an extra setc per IN (bug #15256) */
                    unsigned sample;
                    asm volatile("in %0, res[%1]" : "=r"(sample)  : "r"(p_i2s_adc[index]));
                    sample = bitrev(sample);
                    if(XUA_I2S_N_BITS != 32)
                    {
                        set_port_shift_count(p_i2s_adc[index], XUA_I2S_N_BITS);
                        sample <<= (32 - XUA_I2S_N_BITS);
                    }
                    index++;

                    int chanIndex = ((frameCount-2)&(I2S_CHANS_PER_FRAME-1))+i; // channels 1, 3, 5.. on each line.
#if (AUD_TO_USB_RATIO > 1 && !I2S_DOWNSAMPLE_MONO_IN)
                    if ((AUD_TO_USB_RATIO - 1) == audioToUsbRatioCounter)
                    {
                        samplesIn[buffIndex][chanIndex] =
                            src_ds3_voice_add_final_sample(
                                i2sInDs3Sum[chanIndex],
                                i2sInDs3.delayLine[chanIndex][audioToUsbRatioCounter],
                                src_ff3v_fir_coefs[audioToUsbRatioCounter],
                                sample);
                    }
                    else
                    {
                        i2sInDs3Sum[chanIndex] =
                            src_ds3_voice_add_sample(
                                i2sInDs3Sum[chanIndex],
                                i2sInDs3.delayLine[chanIndex][audioToUsbRatioCounter],
                                src_ff3v_fir_coefs[audioToUsbRatioCounter],
                                sample);
                    }
#else
                    samplesIn[buffIndex][chanIndex] = sample;
#endif /* (AUD_TO_USB_RATIO > 1) && !I2S_DOWNSAMPLE_MONO_IN */
                }
#endif /* I2S_CHANS_ADC != 0) */

#if (I2S_CHANS_ADC != 0 || I2S_CHANS_DAC != 0)
                syncError += HandleSampleClock(frameCount, p_lrclk, skip_ts_check);
#endif

                index = 0;
#if (I2S_CHANS_DAC != 0)
                /* Output "odd" channel to DAC (i.e. right) */
#pragma loop unroll
                for(int i = 0; i < I2S_CHANS_DAC; i+=I2S_CHANS_PER_FRAME)
                {
#if (AUD_TO_USB_RATIO > 1)
                    if (audioToUsbRatioCounter == 0)
                    {
                        samplesOut[frameCount+i] = src_us3_voice_input_sample(i2sOutUs3.delayLine[i],
                                                                              src_ff3v_fir_coefs[2],
                                                                              samplesOut[frameCount+i]);
                    }
                    else
                    { /* audioToUsbRatioCounter is 1 or 2 */
                        samplesOut[frameCount+i] = src_us3_voice_get_next_sample(i2sOutUs3.delayLine[i],
                                                                                 src_ff3v_fir_coefs[2-audioToUsbRatioCounter]);
                    }
#endif /* (AUD_TO_USB_RATIO > 1) */
                    if(XUA_I2S_N_BITS == 32)
                        p_i2s_dac[index++] <: bitrev(samplesOut[frameCount + i]);
                    else
                        partout(p_i2s_dac[index++], XUA_I2S_N_BITS, bitrev(samplesOut[frameCount + i]));
                }
#endif // (I2S_CHANS_DAC != 0)

            }  // !dsdMode


#if (DSD_CHANS_DAC != 0) && (NUM_USB_CHAN_OUT > 0)
            if(DoDsdDopCheck(dsdMode, dsdCount, curSamFreq, samplesOut, dsdMarker) == 0)
            {
#if (I2S_CHANS_ADC != 0) || (I2S_CHANS_DAC != 0)
                // Set clocks low
                p_lrclk <: 0;
                p_bclk <: 0;
#endif
                p_dsd_clk <: 0;
                return 0;
            }
#endif

#if (XUA_PCM_FORMAT == XUA_PCM_FORMAT_TDM)
            /* Increase frameCount by 2 since we have output two channels (per data line) */
            frameCount+=1;
            if(frameCount == I2S_CHANS_PER_FRAME)
#endif
            {
                if ((AUD_TO_USB_RATIO - 1) == audioToUsbRatioCounter)
                {
                    /* Do samples transfer */
                    /* The below looks a bit odd but forces the compiler to inline twice */
                    unsigned command;
                    if(readBuffNo)
                        command = DoSampleTransfer(c_aud, 1, underflowWord);
                    else
                        command = DoSampleTransfer(c_aud, 0, underflowWord);

                    if(command != XUA_AUDCTL_NO_COMMAND)
                    {
                        return command;
                    }

                    /* Reset audio to usb counter because we have now completed one USB transfer and flip the ADC buffer */
                    audioToUsbRatioCounter = 0;
                    readBuffNo = !readBuffNo;
                }
                else
                {
                    ++audioToUsbRatioCounter;
                }
                /* Reset the framecount because we have outputted all channels in the frame now */
                frameCount = 0;
            }
            skip_ts_check = 0;
        }
    }
    return 0;
}

/* This helper function receives the command using the right transaction from Decouple when audiohub breaks */
static void receive_command(unsigned command,
                            chanend c_aud,
                            unsigned &curSamFreq,
                            unsigned &dsdMode,
                            unsigned &curSamRes_DAC,
                            unsigned &audioActive)
{
    debug_printf("receive_command: %d\n", command);
    if(command == XUA_AUDCTL_SET_SAMPLE_FREQ)
    {
        curSamFreq = inuint(c_aud) * AUD_TO_USB_RATIO;
        debug_printf("receive_command set sr: %d\n", curSamFreq);
    }
    else if(command == XUA_AUD_SET_AUDIO_START)
    {
        /* Off = 0
         * DOP = 1
         * Native = 2
         */
        dsdMode = inuint(c_aud);
        curSamRes_DAC = inuint(c_aud);
        audioActive = 1;
        debug_printf("aud stream start\n");
    }
    else if (command == XUA_AUD_SET_AUDIO_STOP)
    {
        debug_printf("aud stream stop\n");
        if(XUA_LOW_POWER_NON_STREAMING)
        {
            audioActive = 0;
        }
    }
    else
    {
        debug_printf("aud unhandled cmd  %u\n", command);
    }
    /* Not we do not ACK back here - it is done when we re-start audio */
}

/* This function is a dummy version of the deliver thread that does not
   connect to the codec ports. It is used during DFU reset and during idle non-streaming mode, if enabled.
   Note there are two paths through depending on dfuMode.*/
[[combinable]]
static void dummy_deliver(chanend ?c_aud, unsigned sampFreq, unsigned dfuMode, unsigned &command, server interface i_dfu i, chanend ?c_user_cmd)
{
    const int wait_ticks = XS1_TIMER_HZ / sampFreq;
    timer tmr;
    int tmr_trigger;
    tmr :> tmr_trigger;
    tmr_trigger += wait_ticks;

    if(!dfuMode)
    {
        StartSampleTransfer(c_aud, 0);
    }

/* This is a bit annoyting but we have to end in a while(1) select to be combinable */
#if ((NUM_USB_CHAN_OUT > 0) || (NUM_USB_CHAN_IN > 0))
#define COMPLETE_SAMPLE_TRANSFER(c, buf, cmd)   CompleteSampleTransferUsbChans(c, buf, cmd)
#else
#define COMPLETE_SAMPLE_TRANSFER(c, buf, cmd)   CheckForCmdNoUsbChans(c, buf, cmd)
#endif
    while (1)
    {
        /* Note, a select is used such that this task is combinable */
        select
        {
            case COMPLETE_SAMPLE_TRANSFER(c_aud, 0, command):   /* Check for command & transfer the samples & UBM */
                if(command)
                {
                    if(dfuMode)
                    {
                        /* Just consume the command, ignore it + keep on looping forever */
                        unsigned dummy1, dummy2, dummy3, dummy4;
                        receive_command(command, c_aud, dummy1, dummy2, dummy3, dummy4);
                        outct(c_aud, XS1_CT_END);
                    }
                    else
                    {
                        /* Process the command in the callee */
                        return;
                    }
                }
                /* Wait one sample period */
                tmr when timerafter(tmr_trigger + wait_ticks) :> void;
                tmr_trigger += wait_ticks;


                /* Request more data/commands */
                if((NUM_USB_CHAN_OUT > 0) || (NUM_USB_CHAN_IN > 0))
                {
                    //printstrln("start sample transfer");
                    StartSampleTransfer(c_aud, 0);
                }
            break;
#if (XUA_XUD_TILE_NUM != 0) && (XUA_AUDIO_IO_TILE_NUM == 0)
            case i.HandleDfuRequest(USB_SetupPacket_t &sp, unsigned data_buffer[], unsigned data_buffer_length, unsigned dfuState)
                -> {unsigned reset_device_after_ack, int return_data_len, int dfu_reset_override, int returnVal, unsigned newDfuState}:

                reset_device_after_ack = 0;
                return_data_len = 0;
                dfu_reset_override = 0;
                unsigned tmpDfuState = dfuState;
                returnVal = 0;
                printstrln("DFUHandler HandleDfuRequest");
                // Map Standard DFU commands onto device level firmware upgrade mechanism
                switch (sp.bRequest)
                {
                    case DFU_DETACH:
                        if(dfuState == STATE_APP_IDLE)
                        {
                            dfu_reset_override = _BOOT_DFU_MODE_FLAG; // Reboot in DFU mode
                        }
                        else
                        {
                            // We expect to come here only in the STATE_DFU_IDLE state but to be safe,
                            // in every state other than APP_IDLE, reboot in APP mode.
                            dfu_reset_override = 0;
                        }
                        reset_device_after_ack = 1;
                        return_data_len = 0;
                        break;

                    case DFU_DNLOAD:
                        unsigned data[_DFU_TRANSFER_SIZE_WORDS];
                        for(int i = 0; i < _DFU_TRANSFER_SIZE_WORDS; i++)
                            data[i] = data_buffer[i];
                        returnVal = DFU_Dnload(sp.wLength, sp.wValue, data, c_user_cmd, return_data_len, tmpDfuState);
                        break;

                    case DFU_UPLOAD:
                        unsigned data_out[_DFU_TRANSFER_SIZE_WORDS];
                        return_data_len = DFU_Upload(sp.wLength, sp.wValue, data_out, tmpDfuState);
                        for(int i = 0; i < _DFU_TRANSFER_SIZE_WORDS; i++)
                            data_buffer[i] = data_out[i];
                        break;

                    case DFU_GETSTATUS:
                        //printstrln("GETSTATUS");
                        unsigned data_out[_DFU_TRANSFER_SIZE_WORDS];
                        return_data_len = DFU_GetStatus(sp.wLength, data_out, c_user_cmd, tmpDfuState);
                        for(int i = 0; i < _DFU_TRANSFER_SIZE_WORDS; i++)
                            data_buffer[i] = data_out[i];
                        break;

                    case DFU_CLRSTATUS:
                        return_data_len = DFU_ClrStatus(tmpDfuState);
                        break;

                    case DFU_GETSTATE:
                        unsigned data_out[_DFU_TRANSFER_SIZE_WORDS];
                        return_data_len = DFU_GetState(sp.wLength, data_out, c_user_cmd, tmpDfuState);
                        for(int i = 0; i < _DFU_TRANSFER_SIZE_WORDS; i++)
                            data_buffer[i] = data_out[i];
                        break;

                    case DFU_ABORT:
                        return_data_len = DFU_Abort(tmpDfuState);
                        break;

                    /* XMOS Custom DFU requests */
                    case XMOS_DFU_RESETDEVICE:
                        reset_device_after_ack = 1;
                        return_data_len = 0;
                        break;

                    case XMOS_DFU_REVERTFACTORY:
                        return_data_len = XMOS_DFU_RevertFactory(c_user_cmd);
                        break;

                    case XMOS_DFU_RESETINTODFU:
                        reset_device_after_ack = 1;
                        dfu_reset_override = _BOOT_DFU_MODE_FLAG;
                        return_data_len = 0;
                        break;

                    case XMOS_DFU_RESETFROMDFU:
                        reset_device_after_ack = 1;
                        dfu_reset_override = 0;
                        return_data_len = 0;
                        break;

                    case XMOS_DFU_SELECTIMAGE:
                        return_data_len = XMOS_DFU_SelectImage(sp.wValue, c_user_cmd);
                        break;

                    default:
                        returnVal = XUD_RES_ERR; // Unrecognised request
                        break;
                }
				newDfuState = tmpDfuState;
                break;

           case i.finish():
                return;
#endif

        }
    }
}


#if XUA_DFU_EN
/* External DFU handler task */
[[distributable]]
void DFUHandler(server interface i_dfu i, chanend ?c_user_cmd);


/* Helper function to see if a request for entry to DFU has been issued via a sample rate change.
   If so, enter DFU. Note this code will never return. The device will
   need to be reset which is part of the DFU sequence */
void check_and_enter_dfu(unsigned curSamFreq, chanend c_aud, server interface i_dfu ?dfuInterface)
{
    /* Currently no more audio will happen after this point */
    if ((curSamFreq / AUD_TO_USB_RATIO) == AUDIO_STOP_FOR_DFU)
    {
        /* Handshake back the SR change command that put us in DFU*/
        outct(c_aud, XS1_CT_END);

        /* Request more data/commands */
        if((NUM_USB_CHAN_OUT > 0) || (NUM_USB_CHAN_IN > 0))
        {
            StartSampleTransfer(c_aud, 0);          /* Send first token to fire ISR in decouple */
            //printstrln("AUDIOHUB stop for DFU");
        }

        unsigned command = XUA_AUDCTL_NO_COMMAND;
        while (1)
        {
            [[combine]]
            par
            {
/*#if (XUA_XUD_TILE_NUM != 0) && (XUA_AUDIO_IO_TILE_NUM == 0)
                DFUHandler(dfuInterface, null);
#endif*/
                /* This never exits because we set DFU mode*/
                dummy_deliver(c_aud, 48000, 1, command, dfuInterface, null);
            }
            /* Note, we shouldn't reach here. Audio, once stopped for DFU, cannot be resumed */
        }
    }
}
#endif /* XUA_DFU_EN */


void XUA_AudioHub(chanend ?c_aud, clock ?clk_audio_mclk, clock ?clk_audio_bclk,
    in port ?p_mclk_in,
    buffered _XUA_CLK_DIR port:32 ?p_lrclk,
    buffered _XUA_CLK_DIR port:32 ?p_bclk,
    buffered out port:32 (&?p_i2s_dac)[I2S_WIRES_DAC],
    buffered in port:32  (&?p_i2s_adc)[I2S_WIRES_ADC]
#if (XUA_SPDIF_TX_EN)
    , chanend c_spdif_out
#endif
#if (XUA_ADAT_RX_EN || XUA_SPDIF_RX_EN)
    , chanend c_dig_rx
#endif
#if ADJUSTABLE_MCLK_REQUIRED
    , chanend c_audio_rate_change
#endif
#if (XUA_XUD_TILE_NUM != 0) && (XUA_AUDIO_IO_TILE_NUM == 0) && (XUA_DFU_EN == 1)
    , server interface i_dfu ?dfuInterface
#endif
#if (XUA_NUM_PDM_MICS > 0)
    , chanend c_pdm_in
#endif
)
{
/* This is a bit annoying but we have a mixture of nullable interfaces and variadic function signatures based on defines */
#if !((XUA_XUD_TILE_NUM != 0) && (XUA_AUDIO_IO_TILE_NUM == 0) && (XUA_DFU_EN == 1))
#define dfuInterface null
#endif
#if (XUA_ADAT_TX_EN)
    chan c_adat_out;
    unsigned adatSmuxMode = 0;
    unsigned adatMultiple = 0;
#endif
    unsigned curSamFreq = DEFAULT_FREQ * AUD_TO_USB_RATIO;
    unsigned curSamRes_DAC = STREAM_FORMAT_OUTPUT_1_RESOLUTION_BITS; /* Default to something reasonable */
    unsigned curSamRes_ADC = STREAM_FORMAT_INPUT_1_RESOLUTION_BITS; /* Default to something reasonable - note, currently this never changes*/
    unsigned command = XUA_AUDCTL_NO_COMMAND;
    unsigned mClk;
    unsigned divide;
    /* Flag used to indicate whether both interfaces are set to Alt 0 or not for power saving option
       This is only used when XUA_LOW_POWER_NON_STREAMING is defined to non-zero */
    unsigned audioActive = XUA_LOW_POWER_NON_STREAMING ? 0 : 1;
    /* This flag is to ensure that the decouple<->audio channel protocol is observed at startup.
       We need this because we hold off the ACK back to decouple as late as possible so that the control path knows audio is fully ready */
    unsigned firstRun = 1;

    while(1)
    {
        /* This is the "low power non-streaming" loop */
        while(audioActive == 0)
        {
            /* We only want to ACK after we have done our first transaction with decouple */
            if(firstRun == 0)
            {
                /* Handshake back previous command */
                if(XUA_USB_EN)
                {
                    outct(c_aud, XS1_CT_END);
                }
            }
            else
            {
                firstRun = 0;
            }
            /* Now run dummy loop with no IO. This is sufficient to poll for commands from decouple */
            dummy_deliver(c_aud, 1000, 0, command, dfuInterface, null);  /* Run loop at 1kHz for min power and exit if command */
            receive_command(command, c_aud, curSamFreq, dsdMode, curSamRes_DAC, audioActive);
#if (XUA_DFU_EN == 1)
            check_and_enter_dfu(curSamFreq, c_aud, dfuInterface);
#endif /* (XUA_DFU_EN == 1) */
        } /* audioActive == 0 */

#if (DSD_CHANS_DAC > 0)
        /* Make sure the DSD ports are on and buffered - just in case they are not shared with I2S */
        EnableBufferedPort(p_dsd_clk, 32);
        for(int i = 0; i< DSD_CHANS_DAC; i++)
        {
            EnableBufferedPort(p_dsd_dac[i], 32);
        }
#endif

#if (MCLK_REQUIRED)
        xassert((!isnull(clk_audio_mclk) && !isnull(p_mclk_in)) && "Error: must provide non-null MCLK port and MCLK clock-block if digital Rx is enabled or XUA_AUDIO_IO_TILE_NUM==XUA_XUD_TILE_NUM");
        /* Clock master clock-block from master-clock port */
        configure_clock_src(clk_audio_mclk, p_mclk_in);
#if (XUA_ADAT_TX_EN)
        /* Set ADAT Tx port to be clock from master clock-block*/
        configure_out_port_no_ready(p_adat_tx, clk_audio_mclk, 0);
        set_clock_fall_delay(clk_audio_mclk, 7);
#endif
        /* If the XUD tile is different from AUDIO tile, then we start a clkblk for counting clocks on the XUD tile and start it in main.
        If XUD is on the same tile as AUDIO then we just connect p_for_mclk_count to the  clk_audio_mclk in main, but
        we need to start it here after all of the connections have been made.
        Note. we do not need a clk_audio_mclk if no dig Tx and XUD and AUDIO are on different tiles because I2S is driven by BCLK clkblk
        directly from the MCLK port. */
        /* Start the master clock-block */
        start_clock(clk_audio_mclk);
#endif /* (MCLK_REQUIRED) */

        /* Perform required CODEC/ADC/DAC initialisation */
        AudioHwInit();

        /* Only break this loop if LP non streaming enabled and streams are both Alt 0 */
        while((XUA_LOW_POWER_NON_STREAMING == 0) || audioActive)
        {
            /* Calculate what master clock we should be using */
            if (((MCLK_441) % curSamFreq) == 0)
            {
                mClk = MCLK_441;
#if (XUA_ADAT_TX_EN)
                /* Calculate ADAT SMUX mode (1, 2, 4) */
                adatSmuxMode = curSamFreq / 44100;
                adatMultiple = mClk / 44100;
#endif
            }
            else if (((MCLK_48) % curSamFreq) == 0)
            {
                mClk = MCLK_48;
#if (XUA_ADAT_TX_EN)
                /* Calculate ADAT SMUX mode (1, 2, 4) */
                adatSmuxMode = curSamFreq / 48000;
                adatMultiple = mClk / 48000;
#endif
            }

            /* Calculate master clock to bit clock (or DSD clock) divide for current sample freq
             * e.g. 11.289600 / (176400 * 64)  = 1 */
            {
                unsigned numBits = XUA_I2S_N_BITS * I2S_CHANS_PER_FRAME;

#if (DSD_CHANS_DAC > 0)
                if(dsdMode == DSD_MODE_DOP)
                {
                    /* DoP we receive in 16bit chunks */
                    numBits = 16;
                }
                else if(dsdMode == DSD_MODE_NATIVE)
                {
                    /* DSD native we receive in 32bit chunks */
                    numBits = 32;
                }
#endif
                divide = mClk / (curSamFreq * numBits);

                // Do some checks
                xassert((divide > 0) && "Error: divider is 0, BCLK rate unachievable");

                unsigned remainder = mClk % ( curSamFreq * numBits);
                xassert((!remainder) && "Error: MCLK not divisible into BCLK by an integer number");

                /* Ignore special divide = 1 case */
                unsigned divider_is_odd = (divide & 0x1) && (divide != 1);
                xassert((!divider_is_odd) && "Error: divider is odd, clockblock cannot produce desired BCLK");

           }

#if (I2S_CHANS_DAC != 0) || (I2S_CHANS_ADC != 0)
#if (DSD_CHANS_DAC > 0)
            if(dsdMode)
            {
                /* Configure audio ports */
                ConfigAudioPortsWrapper(
#if (I2S_CHANS_DAC != 0) || (DSD_CHANS_DAC != 0)
                    p_dsd_dac,
                    DSD_CHANS_DAC,
#endif // (I2S_CHANS_DAC != 0) || (DSD_CHANS_DAC != 0)
#if (I2S_CHANS_ADC != 0)
                    p_i2s_adc,
                    I2S_WIRES_ADC,
#endif // (I2S_CHANS_ADC != 0)
                    null,
                    p_dsd_clk,
                    p_mclk_in, clk_audio_bclk, divide, curSamFreq);
            }
            else
#endif // (DSD_CHANS_DAC > 0)
            {
                ConfigAudioPortsWrapper(
#if (I2S_CHANS_DAC != 0)
                    p_i2s_dac,
                    I2S_WIRES_DAC,
#endif // (I2S_CHANS_DAC != 0)
#if (I2S_CHANS_ADC != 0)
                    p_i2s_adc,
                    I2S_WIRES_ADC,
#endif // (I2S_CHANS_ADC != 0)
                    p_lrclk,
                    p_bclk,
                    p_mclk_in, clk_audio_bclk, divide, curSamFreq);
            }
#endif // (I2S_CHANS_DAC != 0) || (I2S_CHANS_ADC != 0)

            {
                unsigned curFreq = curSamFreq;
#if (DSD_CHANS_DAC > 0)
                /* Make AudioHwConfig() implementation a little more user friendly in DSD mode...*/
                if(dsdMode == DSD_MODE_NATIVE)
                {
                    curFreq *= 32;
                }
                else if(dsdMode == DSD_MODE_DOP)
                {
                    curFreq *= 16;
                }
#endif
                /* Configure Clocking/CODEC/DAC/ADC for SampleFreq/MClk */

                /* User should mute audio hardware */
                AudioHwConfig_Mute();

                /* User code should configure audio harware for SampleFreq/MClk etc */

#if defined(__XS3A__) && (!ADJUSTABLE_MCLK_REQUIRED) && (XUA_USE_SW_PLL)
                sw_pll_fixed_clock(mClk); // output a fixed clock using the application PLL
#endif
                AudioHwConfig(curFreq, mClk, dsdMode, curSamRes_DAC, curSamRes_ADC);
#if (ADJUSTABLE_MCLK_REQUIRED)
                /* Notify clockgen of new mCLk */
                c_audio_rate_change <: mClk;
                c_audio_rate_change <: curFreq;

                /* Wait for ACK back from clockgen or ep_buffer to signal clocks all good */
                c_audio_rate_change :> int _;
#if XUA_USE_SW_PLL
                timer t;
                unsigned time;
                /* Allow some time for mclk to lock and MCLK to stabilise - this is important to avoid glitches at start of stream */
                t :> time;
                t when timerafter(time+40000000) :> void;
#endif

#endif
                /* User should unmute audio hardware */
                AudioHwConfig_UnMute();
            }

#if (XUA_NUM_PDM_MICS > 0)
            /* Send decimation factor to PDM task(s) */
            c_pdm_in <: curSamFreq / AUD_TO_MICS_RATIO;
#endif

            if(firstRun == 0)
            {
                /* TODO wait for good mclk instead of delay */
                /* No delay for DFU modes */
                if (command)
                {
#if 0
                    /* User should ensure MCLK is stable in AudioHwConfig */
                    if(retVal1 == XUA_AUDCTL_SET_SAMPLE_FREQ)
                    {
                        timer t;
                        unsigned time;
                        t :> time;
                        t when timerafter(time+AUDIO_PLL_LOCK_DELAY) :> void;
                    }
#endif
                    /* Handshake back from previous loop's command*/
                    if(XUA_USB_EN)
                    {
                        outct(c_aud, XS1_CT_END);
                    }
                }
            }
            firstRun = 0;

            par
            {

#if (XUA_ADAT_TX_EN)
                {
                    set_thread_fast_mode_on();
                    adat_tx_port(c_adat_out, p_adat_tx);
                }
#endif
                {
#if (XUA_SPDIF_TX_EN)
                    /* Communicate master clock and sample freq to S/PDIF thread */
                    outct(c_spdif_out, XS1_CT_END);
                    outuint(c_spdif_out, curSamFreq);
                    outuint(c_spdif_out, mClk);
                    if(curSamRes_DAC >= 24) {
                        outuint(c_spdif_out, 24);
                    }
                    else {
                        outuint(c_spdif_out, 16);
                    }
#endif

#if (XUA_ADAT_TX_EN)
                    // Configure ADAT parameters ...
                    //
                    // adat_oversampling =  256 for MCLK = 12M288 or 11M2896
                    //                   =  512 for MCLK = 24M576 or 22M5792
                    //                   = 1024 for MCLK = 49M152 or 45M1584
                    //
                    // adatSmuxMode   = 1 for FS =  44K1 or  48K0
                    //                = 2 for FS =  88K2 or  96K0
                    //                = 4 for FS = 176K4 or 192K0
                    outuint(c_adat_out, adatMultiple);
                    outuint(c_adat_out, adatSmuxMode);
#endif
                    command = AudioHub_MainLoop(c_aud
#if (XUA_SPDIF_TX_EN)
                       , c_spdif_out
#else
                       , null
#endif
#if (XUA_ADAT_TX_EN)
                       , c_adat_out
                       , adatSmuxMode
#endif
                       , divide, curSamFreq
#if (XUA_ADAT_RX_EN || XUA_SPDIF_RX_EN)
                       , c_dig_rx
#endif
#if (XUA_NUM_PDM_MICS > 0)
                       , c_pdm_in
#endif
                      , p_lrclk, p_bclk, p_i2s_dac, p_i2s_adc);

#if (XUA_USB_EN)
                    /* Now perform any additional inputs and update state accordingly */
                    receive_command(command, c_aud, curSamFreq, dsdMode, curSamRes_DAC, audioActive);
#if (XUA_DFU_EN == 1)
                    check_and_enter_dfu(curSamFreq, c_aud, dfuInterface);

#endif /* (XUA_DFU_EN == 1) */
#endif /* XUA_USB_EN */


#if XUA_NUM_PDM_MICS > 0
                    unsafe {
                        ma_shutdown((chanend_t)c_pdm_in); // shutdown mics
                    }
#endif

#if (XUA_ADAT_TX_EN)
#ifdef ADAT_TX_USE_SHARED_BUFF
                    /* Take out-standing handshake from ADAT core */
                    inuint(c_adat_out);
#endif
                    /* Notify ADAT Tx thread of impending new freq... */
                    outct(c_adat_out, XS1_CT_END);
#endif /* XUA_ADAT_TX_EN) */
                } /* AudioHub_MainLoop and command handler */
            } /* par */
        } /* while((!XUA_LOW_POWER_NON_STREAMING) || audioActive) */


        /* The following code can only be reached if XUA_LOW_POWER_NON_STREAMING is enabled and all streams stopped */

        /* First shutdown and reset ports before we may shutdown any clocks */
#if (I2S_CHANS_DAC != 0) || (I2S_CHANS_ADC != 0)
#if (DSD_CHANS_DAC > 0)
        if(dsdMode)
        {
            /* Configure audio ports */
            DeConfigAudioPorts(
#if (I2S_CHANS_DAC != 0) || (DSD_CHANS_DAC != 0)
                p_dsd_dac,
                DSD_CHANS_DAC,
#endif // (I2S_CHANS_DAC != 0) || (DSD_CHANS_DAC != 0)
#if (I2S_CHANS_ADC != 0)
                p_i2s_adc,
                I2S_WIRES_ADC,
#endif // (I2S_CHANS_ADC != 0)
                null,
                p_dsd_clk,
                p_mclk_in,
                clk_audio_bclk);
        }
        else
#endif // (DSD_CHANS_DAC > 0)
        {
            DeConfigAudioPorts(
#if (I2S_CHANS_DAC != 0)
                p_i2s_dac,
                I2S_WIRES_DAC,
#endif // (I2S_CHANS_DAC != 0)
#if (I2S_CHANS_ADC != 0)
                p_i2s_adc,
                I2S_WIRES_ADC,
#endif // (I2S_CHANS_ADC != 0)
                p_lrclk,
                p_bclk,
                p_mclk_in,
                clk_audio_bclk);
        }
#endif // (I2S_CHANS_DAC != 0) || (I2S_CHANS_ADC != 0)

#if (MCLK_REQUIRED)
        /* Start the master clock-block */
        stop_clock(clk_audio_mclk);
#endif

        /* Call user functions for core power down (eg. MCLK disable) and system component power down */
        AudioHwShutdown();

    } /* while(1)*/
}
