/*
 * USB mass storage (Bulk-Only Transport, SCSI transparent command set) for the tinySA ULTRA SD card
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * The software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * The SD card is exposed as a removable disk while "USB DISK" mode is active (usb_disk_mode() in ui.c).
 * Outside that mode the LUN reports "medium not present", like an empty card reader, so the
 * composite CDC + MSC device never has to re-enumerate.
 * While the medium is ready the MSC thread owns SPI1 and spi_buffer: the sweep thread only polls
 * touch/buttons and the shell refuses commands that could use SPI1.
 *
 * The endpoint register handling (data toggle reset on CLEAR_FEATURE, manual transfer abort) is
 * specific to the USBv1 driver of this ChibiOS fork: re-check it after a ChibiOS upgrade.
 */
#include "ch.h"
#include "hal.h"
#include "nanovna.h"
#include <string.h>

#ifdef __USE_USB_MSC__
#include "usb_msc.h"

#define MSC_PACKET_SIZE        64
#define MSC_SECTOR_SIZE        512
#define MSC_CHUNK_SECTORS      (sizeof(spi_buffer) / MSC_SECTOR_SIZE)
#define MSC_IO_STOP_TIMEOUT    MS2ST(2000)   // Max wait for a transfer in progress when leaving USB DISK mode

// Bulk-Only Transport
#define CBW_SIGNATURE          0x43425355    // "USBC"
#define CSW_SIGNATURE          0x53425355    // "USBS"
#define CBW_LENGTH             31
#define CSW_LENGTH             13
#define CBW_FLAGS_IN           0x80
#define CSW_PASSED             0
#define CSW_FAILED             1
#define CSW_PHASE_ERROR        2
#define CSW_NONE               0xFF          // Transfer aborted by a reset, no CSW
#define MSC_REQ_GET_MAX_LUN    0xFE
#define MSC_REQ_RESET          0xFF

// SCSI commands
#define SCSI_TEST_UNIT_READY   0x00
#define SCSI_REQUEST_SENSE     0x03
#define SCSI_INQUIRY           0x12
#define SCSI_MODE_SENSE6       0x1A
#define SCSI_START_STOP_UNIT   0x1B
#define SCSI_PREVENT_ALLOW     0x1E
#define SCSI_READ_FORMAT_CAP   0x23
#define SCSI_READ_CAPACITY10   0x25
#define SCSI_READ10            0x28
#define SCSI_WRITE10           0x2A
#define SCSI_VERIFY10          0x2F
#define SCSI_SYNC_CACHE10      0x35
#define SCSI_MODE_SENSE10      0x5A

// Sense key, additional sense code and qualifier
#define SENSE(key, asc, ascq)  (((uint32_t)(key) << 16) | ((asc) << 8) | (ascq))
#define SENSE_NONE             SENSE(0x00, 0x00, 0x00)
#define SENSE_NOT_PRESENT      SENSE(0x02, 0x3A, 0x00)
#define SENSE_READ_ERROR       SENSE(0x03, 0x11, 0x00)
#define SENSE_WRITE_ERROR      SENSE(0x03, 0x0C, 0x00)
#define SENSE_INVALID_OPCODE   SENSE(0x05, 0x20, 0x00)
#define SENSE_LBA_RANGE        SENSE(0x05, 0x21, 0x00)
#define SENSE_INVALID_FIELD    SENSE(0x05, 0x24, 0x00)
#define SENSE_MEDIUM_CHANGED   SENSE(0x06, 0x28, 0x00)

// Data phase direction announced by the host
#define DIR_NONE               0
#define DIR_IN                 1
#define DIR_OUT                2

// Wake up reasons of the MSC thread
#define EV_IN_DONE             0x01
#define EV_OUT_DONE            0x02
#define EV_HALT_CLEARED        0x04

typedef struct __attribute__((packed)) {
  uint32_t signature;
  uint32_t tag;
  uint32_t data_length;
  uint8_t  flags;
  uint8_t  lun;
  uint8_t  cb_length;
  uint8_t  cb[16];
} msc_cbw_t;

typedef struct __attribute__((packed)) {
  uint32_t signature;
  uint32_t tag;
  uint32_t residue;
  uint8_t  status;
} msc_csw_t;

// The USB driver copies whole packets, so the CBW buffer must hold a full packet
static union {
  uint8_t   raw[MSC_PACKET_SIZE];
  msc_cbw_t cbw;
} cbw_buf __attribute__((aligned(4)));

static union {
  uint8_t   raw[16];
  msc_csw_t csw;
} csw_buf __attribute__((aligned(4)));

// Response data of all commands except READ/WRITE (spi_buffer is only used while the medium is ready)
static uint8_t resp[36] __attribute__((aligned(4)));

static const uint8_t inquiry_data[36] = {
  0x00,                                       // Direct access block device
  0x80,                                       // Removable medium
  0x02,                                       // Version
  0x02,                                       // Response data format
  36 - 5,                                     // Additional length
  0x00, 0x00, 0x00,
  't','i','n','y','S','A',' ',' ',            // Vendor
  'U','L','T','R','A',' ','S','D',' ','c','a','r','d',' ',' ',' ', // Product
  '1','.','0','0'                             // Revision
};

static USBInEndpointState  ep3in;
static USBOutEndpointState ep3out;

static thread_reference_t msc_tr = NULL;
static volatile uint8_t   msc_ev;             // EV_xxx bits
static volatile uint16_t  msc_epoch;          // Changes on USB reset, configuration, BOT reset and forced abort
static volatile bool      msc_configured;     // EP3 initialized
static volatile bool      msc_need_reset;     // Invalid CBW: keep both pipes stalled until a BOT reset
static volatile bool      msc_ready;          // Medium present (USB DISK mode)
static volatile bool      msc_ua;             // Unit attention pending (medium changed)
static volatile bool      msc_eject;          // Host ejected the medium
static volatile bool      msc_io;             // READ/WRITE in progress, uses SPI1 and spi_buffer
static volatile uint32_t  msc_blocks;         // Medium size in sectors
static uint32_t           msc_sense;          // Sense data of the last failed command

volatile bool msc_disk_mode     = false;
volatile bool msc_enter_request = false;
volatile bool msc_exit_request  = false;

static void msc_in_cb(USBDriver *usbp, usbep_t ep);
static void msc_out_cb(USBDriver *usbp, usbep_t ep);

const USBEndpointConfig msc_ep_config = {
  USB_EP_MODE_TYPE_BULK,
  NULL,
  msc_in_cb,
  msc_out_cb,
  MSC_PACKET_SIZE,
  MSC_PACKET_SIZE,
  &ep3in,
  &ep3out
};

static inline uint32_t umin(uint32_t a, uint32_t b) { return a < b ? a : b; }
static inline uint16_t get_be16(const uint8_t *p) { return ((uint16_t)p[0] << 8) | p[1]; }
static inline uint32_t get_be32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }
static inline void put_be32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }

//*******************************************************
//  ISR side: endpoint callbacks, requests hook, usb_event
//*******************************************************
static void msc_wakeup_I(uint8_t ev) {
  msc_ev |= ev;
  osalThreadResumeI(&msc_tr, MSG_OK);
}

// Invalidate transfers of the previous epoch
static void msc_new_epoch_I(void) {
  msc_epoch++;
  msc_ev = 0;
  osalThreadResumeI(&msc_tr, MSG_RESET);
}

static void msc_in_cb(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  (void)ep;
  osalSysLockFromISR();
  msc_wakeup_I(EV_IN_DONE);
  osalSysUnlockFromISR();
}

static void msc_out_cb(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  (void)ep;
  osalSysLockFromISR();
  msc_wakeup_I(EV_OUT_DONE);
  osalSysUnlockFromISR();
}

// Cancel the transfers in progress on the bulk endpoints (the driver has no abort function)
static void msc_abort_ep_I(void) {
  if (!msc_configured)
    return;
  if ((STM32_USB->EPR[MSC_EP] & EPR_STAT_TX_MASK) == EPR_STAT_TX_VALID)
    EPR_SET_STAT_TX(MSC_EP, EPR_STAT_TX_NAK);
  if ((STM32_USB->EPR[MSC_EP] & EPR_STAT_RX_MASK) == EPR_STAT_RX_VALID)
    EPR_SET_STAT_RX(MSC_EP, EPR_STAT_RX_NAK);
  ep3in.txsize  = ep3in.txcnt;                // A pending IN completion ends the transfer
  ep3out.rxbuf  = cbw_buf.raw;                // A pending OUT packet lands in the CBW buffer, not in spi_buffer
  ep3out.rxcnt  = 0;
  ep3out.rxsize = MSC_PACKET_SIZE;
  USBD1.transmitting &= ~(1U << MSC_EP);
  USBD1.receiving    &= ~(1U << MSC_EP);
}

// CLEAR_FEATURE(ENDPOINT_HALT): reset the data toggle to DATA0 (USB 2.0 9.4.5, not done by the driver)
// and change STALL to NAK, unless the pipes must stay stalled until a BOT reset. VALID is left alone.
static void msc_clear_halt_I(bool in) {
  uint32_t epr = STM32_USB->EPR[MSC_EP];
  uint32_t tog;                               // Toggle bits, writing 1 flips them
  if (in) {
    tog = epr & EPR_DTOG_TX;
    if (!msc_need_reset && (epr & EPR_STAT_TX_MASK) == EPR_STAT_TX_STALL)
      tog |= EPR_STAT_TX_STALL ^ EPR_STAT_TX_NAK;
  } else {
    tog = epr & EPR_DTOG_RX;
    if (!msc_need_reset && (epr & EPR_STAT_RX_MASK) == EPR_STAT_RX_STALL)
      tog |= EPR_STAT_RX_STALL ^ EPR_STAT_RX_NAK;
  }
  // Keep type and address, write 0 to the other toggle bits and 1 to the CTR flags (no change)
  STM32_USB->EPR[MSC_EP] = (epr & ~EPR_TOGGLE_MASK) | EPR_CTR_MASK | tog;
}

// Called from the setup packet ISR without the system lock
bool msc_requests_hook(USBDriver *usbp) {
  const setup_pack_t *s = &usbp->setup;
  uint8_t type = s->bmRequestType & (USB_RTYPE_TYPE_MASK | USB_RTYPE_RECIPIENT_MASK);
  bool to_host = (s->bmRequestType & USB_RTYPE_DIR_MASK) == USB_RTYPE_DIR_DEV2HOST;
  if (type == (USB_RTYPE_TYPE_CLASS | USB_RTYPE_RECIPIENT_INTERFACE) && (s->wIndex & 0xFF) == MSC_IF) {
    if (s->bRequest == MSC_REQ_GET_MAX_LUN && to_host) {
      static const uint8_t max_lun = 0;
      usbSetupTransfer(usbp, (uint8_t *)&max_lun, 1, NULL);
      return true;
    }
    if (s->bRequest == MSC_REQ_RESET && !to_host) {
      // Bulk-Only Mass Storage Reset: cancel the transfers, the host clears the halts next
      osalSysLockFromISR();
      msc_abort_ep_I();
      msc_need_reset = false;
      msc_new_epoch_I();
      osalSysUnlockFromISR();
      usbSetupTransfer(usbp, NULL, 0, NULL);
      return true;
    }
    return false;
  }
  if (type == (USB_RTYPE_TYPE_STD | USB_RTYPE_RECIPIENT_ENDPOINT) && s->bRequest == USB_REQ_CLEAR_FEATURE &&
      s->wValue == USB_FEATURE_ENDPOINT_HALT && (s->wIndex & 0x0F) == MSC_EP && msc_configured) {
    osalSysLockFromISR();
    msc_clear_halt_I((s->wIndex & 0x80) != 0);
    msc_wakeup_I(EV_HALT_CLEARED);
    osalSysUnlockFromISR();
    usbSetupTransfer(usbp, NULL, 0, NULL);
    return true;
  }
  return false;
}

void msc_usb_reset_I(void) {
  msc_configured = false;
  msc_need_reset = false;
  msc_new_epoch_I();
}

void msc_configured_I(bool configured) {
  msc_configured = configured;
  msc_need_reset = false;
  msc_new_epoch_I();
}

//*******************************************************
//  MSC thread side
//*******************************************************
// Wait (system locked) for an event of this epoch, false if the transfer was aborted
static bool msc_wait_S(uint8_t ev, uint16_t epoch) {
  while (!(msc_ev & ev)) {
    if (msc_epoch != epoch || !msc_configured)
      return false;
    osalThreadSuspendS(&msc_tr);
  }
  return msc_epoch == epoch;
}

// Wait (system locked) until the host has cleared a halt of the endpoint
static bool msc_wait_not_stalled_S(uint32_t mask, uint32_t stall, uint16_t epoch) {
  while (msc_epoch == epoch && msc_configured && (STM32_USB->EPR[MSC_EP] & mask) == stall) {
    msc_ev &= ~EV_HALT_CLEARED;
    if (!msc_wait_S(EV_HALT_CLEARED, epoch))
      return false;
  }
  return msc_epoch == epoch && msc_configured;
}

// Send data on the bulk IN endpoint and wait until the host has read it
static bool msc_tx(const uint8_t *buf, uint32_t n, uint16_t epoch) {
  chSysLock();
  bool ok = msc_wait_not_stalled_S(EPR_STAT_TX_MASK, EPR_STAT_TX_STALL, epoch);
  if (ok) {
    msc_ev &= ~EV_IN_DONE;
    usbStartTransmitI(&USBD1, MSC_EP, buf, n);
    ok = msc_wait_S(EV_IN_DONE, epoch);
  }
  chSysUnlock();
  return ok;
}

// Receive up to n bytes (multiple of the packet size) on the bulk OUT endpoint
// Returns the received size, -1 if the transfer was aborted
static int32_t msc_rx(uint8_t *buf, uint32_t n, uint16_t epoch) {
  int32_t cnt = -1;
  chSysLock();
  if (msc_wait_not_stalled_S(EPR_STAT_RX_MASK, EPR_STAT_RX_STALL, epoch)) {
    msc_ev &= ~EV_OUT_DONE;
    usbStartReceiveI(&USBD1, MSC_EP, buf, n);
    if (msc_wait_S(EV_OUT_DONE, epoch))
      cnt = ep3out.rxcnt;
  }
  chSysUnlock();
  return cnt;
}

// Stall the pipe in which the host expects more data
static void msc_stall(uint8_t dir, uint16_t epoch) {
  chSysLock();
  if (msc_epoch == epoch && msc_configured) {
    if (dir == DIR_IN)
      usbStallTransmitI(&USBD1, MSC_EP);
    else
      usbStallReceiveI(&USBD1, MSC_EP);
  }
  chSysUnlock();
}

static bool msc_medium_ready(void) {
  if (msc_ready && SD_Inserted())
    return true;
  msc_sense = SENSE_NOT_PRESENT;
  return false;
}

// Report the medium change once to the first command after entering USB DISK mode
static bool msc_unit_attention(void) {
  chSysLock();
  bool ua = msc_ua;
  msc_ua = false;
  chSysUnlock();
  if (ua)
    msc_sense = SENSE_MEDIUM_CHANGED;
  return ua;
}

// Start a READ/WRITE data phase, only allowed while the medium is ready
static bool msc_io_begin(void) {
  chSysLock();
  msc_io = msc_ready;
  chSysUnlock();
  if (msc_io && SD_Inserted())
    return true;
  msc_io = false;
  msc_sense = SENSE_NOT_PRESENT;
  return false;
}

// READ(10) / WRITE(10): the data goes through spi_buffer in chunks of up to MSC_CHUNK_SECTORS
static uint8_t msc_read_write(const uint8_t *cb, uint8_t dir, uint32_t length, uint32_t *done, uint16_t epoch) {
  bool write = cb[0] == SCSI_WRITE10;
  uint32_t lba = get_be32(&cb[2]);
  uint32_t count = get_be16(&cb[7]);
  if (msc_unit_attention())
    return CSW_FAILED;
  if (dir == DIR_NONE)                                          // Host expects no data
    return count ? CSW_PHASE_ERROR : CSW_PASSED;
  if ((dir == DIR_OUT) != write || length < count * MSC_SECTOR_SIZE) // Wrong direction or host expects less data
    return CSW_PHASE_ERROR;
  if (!msc_io_begin())
    return CSW_FAILED;
  uint8_t status = CSW_PASSED;
  if (lba + count > msc_blocks || lba + count < lba) {
    msc_sense = SENSE_LBA_RANGE;
    status = CSW_FAILED;
    count = 0;
  }
  uint8_t *buf = (uint8_t *)spi_buffer;
  while (count) {
    uint32_t n = umin(count, MSC_CHUNK_SECTORS);
    uint32_t size = n * MSC_SECTOR_SIZE;
    if (!msc_medium_ready()) {                                  // Leaving USB DISK mode or card removed
      status = CSW_FAILED;
      break;
    }
    if (write) {
      int32_t r = msc_rx(buf, size, epoch);
      if (r < 0) {status = CSW_NONE; break;}
      if ((uint32_t)r != size) {status = CSW_PHASE_ERROR; break;}   // Short packet, host sent less than announced
      if (disk_write(0, buf, lba, n) != RES_OK) {msc_sense = SENSE_WRITE_ERROR; status = CSW_FAILED; break;}
    } else {
      if (disk_read(0, buf, lba, n) != RES_OK) {msc_sense = SENSE_READ_ERROR; status = CSW_FAILED; break;}
      if (!msc_tx(buf, size, epoch)) {status = CSW_NONE; break;}
    }
    *done += size;
    lba += n;
    count -= n;
  }
  msc_io = false;
  return status;
}

// All commands except READ/WRITE, the response goes to resp[] and *n is its size
static uint8_t msc_scsi(const uint8_t *cb, uint32_t *n) {
  uint8_t op = cb[0];
  if (op != SCSI_INQUIRY && op != SCSI_REQUEST_SENSE && msc_unit_attention())
    return CSW_FAILED;
  switch (op) {
  case SCSI_TEST_UNIT_READY:
    return msc_medium_ready() ? CSW_PASSED : CSW_FAILED;
  case SCSI_REQUEST_SENSE: {
    if (msc_sense == SENSE_NONE)
      msc_unit_attention();
    uint32_t sense = msc_sense;
    msc_sense = SENSE_NONE;
    memset(resp, 0, 18);
    resp[0]  = 0x70;                          // Current error, fixed format
    resp[2]  = sense >> 16;                   // Sense key
    resp[7]  = 18 - 8;                        // Additional sense length
    resp[12] = sense >> 8;                    // Additional sense code
    resp[13] = sense;                         // Additional sense code qualifier
    *n = umin(18, cb[4]);
    return CSW_PASSED;
  }
  case SCSI_INQUIRY: {
    uint32_t len;
    if (cb[1] & 0x01) {                       // Vital product data
      memset(resp, 0, 8);
      if (cb[2] == 0x00) {                    // Supported pages
        resp[3] = 2;
        resp[5] = 0x80;
        len = 6;
      } else if (cb[2] == 0x80) {             // Unit serial number: MCU unique ID
        const uint32_t *uid = (const uint32_t *)UID_BASE;
        resp[1] = 0x80;
        resp[3] = 24;
        for (int i = 0; i < 24; i++)
          resp[4 + i] = "0123456789ABCDEF"[(uid[i / 8] >> (28 - 4 * (i % 8))) & 0x0F];
        len = 4 + 24;
      } else {
        msc_sense = SENSE_INVALID_FIELD;
        return CSW_FAILED;
      }
    } else {
      memcpy(resp, inquiry_data, sizeof(inquiry_data));
      len = sizeof(inquiry_data);
    }
    *n = umin(len, get_be16(&cb[3]));
    return CSW_PASSED;
  }
  case SCSI_MODE_SENSE6:                      // Header only: no block descriptor, not write protected
    memset(resp, 0, 4);
    resp[0] = 4 - 1;                          // Mode data length
    *n = umin(4, cb[4]);
    return CSW_PASSED;
  case SCSI_MODE_SENSE10:
    memset(resp, 0, 8);
    resp[1] = 8 - 2;                          // Mode data length
    *n = umin(8, get_be16(&cb[7]));
    return CSW_PASSED;
  case SCSI_START_STOP_UNIT:
    if ((cb[4] & 0x03) == 0x02) {             // LoEj without Start: the host ejects the medium
      chSysLock();
      msc_ready = false;
      msc_eject = true;
      chSysUnlock();
    }
    return CSW_PASSED;
  case SCSI_PREVENT_ALLOW:
    return CSW_PASSED;
  case SCSI_READ_FORMAT_CAP:
    if (!msc_medium_ready())
      return CSW_FAILED;
    memset(resp, 0, 12);
    resp[3] = 8;                              // Capacity list length
    put_be32(&resp[4], msc_blocks);           // Number of blocks
    resp[8] = 0x02;                           // Formatted media
    resp[10] = MSC_SECTOR_SIZE >> 8;          // Block length (24 bit)
    *n = umin(12, get_be16(&cb[7]));
    return CSW_PASSED;
  case SCSI_READ_CAPACITY10:
    if (!msc_medium_ready())
      return CSW_FAILED;
    put_be32(&resp[0], msc_blocks - 1);       // Last LBA
    put_be32(&resp[4], MSC_SECTOR_SIZE);
    *n = 8;
    return CSW_PASSED;
  case SCSI_VERIFY10:
    if (!msc_medium_ready())
      return CSW_FAILED;
    if (cb[1] & 0x02) {                       // BYTCHK: compare with host data not supported
      msc_sense = SENSE_INVALID_FIELD;
      return CSW_FAILED;
    }
    if (get_be32(&cb[2]) + get_be16(&cb[7]) > msc_blocks) {
      msc_sense = SENSE_LBA_RANGE;
      return CSW_FAILED;
    }
    return CSW_PASSED;
  case SCSI_SYNC_CACHE10:                     // Writes are not cached
    return msc_medium_ready() ? CSW_PASSED : CSW_FAILED;
  default:
    msc_sense = SENSE_INVALID_OPCODE;
    return CSW_FAILED;
  }
}

// Execute a valid CBW: data phase and CSW
static void msc_command(uint16_t epoch) {
  uint8_t  cb[16];
  uint32_t tag    = cbw_buf.cbw.tag;
  uint32_t length = cbw_buf.cbw.data_length;
  uint8_t  dir    = length == 0 ? DIR_NONE : (cbw_buf.cbw.flags & CBW_FLAGS_IN) ? DIR_IN : DIR_OUT;
  uint32_t done   = 0;                        // Bytes of the data phase transferred
  uint8_t  status;
  memcpy(cb, cbw_buf.cbw.cb, sizeof(cb));
  if (cb[0] != SCSI_REQUEST_SENSE)
    msc_sense = SENSE_NONE;
  if (cbw_buf.cbw.lun != 0 || cbw_buf.cbw.cb_length == 0 || cbw_buf.cbw.cb_length > 16) {
    msc_sense = SENSE_INVALID_FIELD;
    status = CSW_FAILED;
  } else if (cb[0] == SCSI_READ10 || cb[0] == SCSI_WRITE10) {
    status = msc_read_write(cb, dir, length, &done, epoch);
    if (status == CSW_NONE)
      return;
  } else {
    uint32_t n = 0;
    status = msc_scsi(cb, &n);
    if (n) {                                  // The command returns data
      if (dir != DIR_IN)
        status = CSW_PHASE_ERROR;
      else {
        n = umin(n, length);                  // Host expects less: send what it asked for
        if (!msc_tx(resp, n, epoch))
          return;
        done = n;
      }
    }
  }
  // Host expects more data: stall its pipe, for IN the CSW follows after the host cleared the halt
  if (done < length)
    msc_stall(dir, epoch);
  csw_buf.csw.signature = CSW_SIGNATURE;
  csw_buf.csw.tag       = tag;
  csw_buf.csw.residue   = length - done;
  csw_buf.csw.status    = status;
  msc_tx(csw_buf.raw, CSW_LENGTH, epoch);
}

static THD_WORKING_AREA(waMSC, 384);
static THD_FUNCTION(msc_thread, arg)
{
  (void)arg;
  chRegSetThreadName("msc");
  while (true) {
    chSysLock();
    while (!msc_configured)
      osalThreadSuspendS(&msc_tr);
    uint16_t epoch = msc_epoch;
    chSysUnlock();
    // Wait for a command block wrapper
    int32_t n = msc_rx(cbw_buf.raw, MSC_PACKET_SIZE, epoch);
    if (n < 0)
      continue;
    if (n == CBW_LENGTH && cbw_buf.cbw.signature == CBW_SIGNATURE) {
      msc_command(epoch);
      continue;
    }
    // Invalid CBW: stall both pipes until the host does a Bulk-Only Mass Storage Reset
    chSysLock();
    if (msc_epoch == epoch) {
      msc_need_reset = true;
      usbStallTransmitI(&USBD1, MSC_EP);
      usbStallReceiveI(&USBD1, MSC_EP);
      while (msc_epoch == epoch)
        osalThreadSuspendS(&msc_tr);
    }
    chSysUnlock();
  }
}

void msc_init(void)
{
  chThdCreateStatic(waMSC, sizeof(waMSC), NORMALPRIO, msc_thread, NULL);
}

//*******************************************************
//  USB DISK mode control (sweep thread)
//*******************************************************
// The caller must have initialized the SD card and must not use SPI1 until msc_medium_stop()
void msc_medium_start(uint32_t blocks)
{
  chSysLock();
  msc_blocks = blocks;
  msc_eject  = false;
  msc_ua     = true;
  msc_ready  = true;
  chSysUnlock();
}

// Returns when the MSC thread no longer uses SPI1 and spi_buffer
void msc_medium_stop(void)
{
  chSysLock();
  msc_ready = false;
  msc_ua    = false;
  chSysUnlock();
  // A READ/WRITE in progress stops at the next chunk
  systime_t start = chVTGetSystemTimeX();
  while (msc_io && chVTGetSystemTimeX() - start < MSC_IO_STOP_TIMEOUT)
    chThdSleepMilliseconds(5);
  if (msc_io) {
    // The host does not move data (asleep, cable pulled): abort the transfer. Stalling both pipes
    // until a BOT reset makes a host that is still connected recover at once.
    chSysLock();
    if (msc_configured) {
      msc_abort_ep_I();
      msc_need_reset = true;
      usbStallTransmitI(&USBD1, MSC_EP);
      usbStallReceiveI(&USBD1, MSC_EP);
    }
    msc_new_epoch_I();
    chSchRescheduleS();
    chSysUnlock();
    // Only a SD card operation can still be running, it has its own timeouts
    while (msc_io)
      chThdSleepMilliseconds(5);
  }
}

bool msc_eject_requested(void)
{
  return msc_eject;
}

#endif // __USE_USB_MSC__
