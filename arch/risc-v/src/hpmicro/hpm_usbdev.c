/****************************************************************************
 * arch/arm/src/hpmicro/hpm_usbdev.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "riscv_internal.h"
#include "chip.h"
#include "hpm_common.h"
#include "hpm_usb_drv.h"
#include "hpm_l1c_drv.h"
#include "board.h"

#ifdef CONFIG_HPM_USBDEV

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_USBDEV_EP0_MAXSIZE
#  define CONFIG_USBDEV_EP0_MAXSIZE 64
#endif

#ifndef  CONFIG_USBDEV_MAXPOWER
#  define CONFIG_USBDEV_MAXPOWER 100  /* mA */
#endif

/* Enable reading SOF from interrupt handler vs. simply reading on demand.
 * Probably a bad idea... Unless there is some issue with sampling the SOF
 * from hardware asynchronously.
 */

#ifdef CONFIG_HPM_USBDEV_FRAME_INTERRUPT
#  define USB_FRAME_INT USB_USBINTR_SRE_MASK
#else
#  define USB_FRAME_INT 0
#endif

#ifdef CONFIG_DEBUG_FEATURES
#  define USB_ERROR_INT USB_USBINTR_UEE_MASK
#else
#  define USB_ERROR_INT 0
#endif

/* Debug ********************************************************************/

/* Trace error codes */

#define HPM_TRACEERR_ALLOCFAIL            0x0001
#define HPM_TRACEERR_BADCLEARFEATURE      0x0002
#define HPM_TRACEERR_BADDEVGETSTATUS      0x0003
#define HPM_TRACEERR_BADEPNO              0x0004
#define HPM_TRACEERR_BADEPGETSTATUS       0x0005
#define HPM_TRACEERR_BADEPTYPE            0x0006
#define HPM_TRACEERR_BADGETCONFIG         0x0007
#define HPM_TRACEERR_BADGETSETDESC        0x0008
#define HPM_TRACEERR_BADGETSTATUS         0x0009
#define HPM_TRACEERR_BADSETADDRESS        0x000a
#define HPM_TRACEERR_BADSETCONFIG         0x000b
#define HPM_TRACEERR_BADSETFEATURE        0x000c
#define HPM_TRACEERR_BINDFAILED           0x000d
#define HPM_TRACEERR_DISPATCHSTALL        0x000e
#define HPM_TRACEERR_DRIVER               0x000f
#define HPM_TRACEERR_DRIVERREGISTERED     0x0010
#define HPM_TRACEERR_EP0SETUPSTALLED      0x0011
#define HPM_TRACEERR_EPINNULLPACKET       0x0012
#define HPM_TRACEERR_EPOUTNULLPACKET      0x0013
#define HPM_TRACEERR_INVALIDCTRLREQ       0x0014
#define HPM_TRACEERR_INVALIDPARMS         0x0015
#define HPM_TRACEERR_IRQREGISTRATION      0x0016
#define HPM_TRACEERR_NOEP                 0x0017
#define HPM_TRACEERR_NOTCONFIGURED        0x0018
#define HPM_TRACEERR_REQABORTED           0x0019

/* Trace interrupt codes */

#define HPM_TRACEINTID_USB                0x0001
#define HPM_TRACEINTID_CLEARFEATURE       0x0002
#define HPM_TRACEINTID_DEVGETSTATUS       0x0003
#define HPM_TRACEINTID_DEVRESET           0x0004
#define HPM_TRACEINTID_DISPATCH           0x0005
#define HPM_TRACEINTID_EP0COMPLETE        0x0006
#define HPM_TRACEINTID_EP0NAK             0x0007
#define HPM_TRACEINTID_EP0SETUP           0x0008
#define HPM_TRACEINTID_EPGETSTATUS        0x0009
#define HPM_TRACEINTID_EPIN               0x000a
#define HPM_TRACEINTID_EPINQEMPTY         0x000b
#define HPM_TRACEINTID_EP0INSETADDRESS    0x000c
#define HPM_TRACEINTID_EPOUT              0x000d
#define HPM_TRACEINTID_EPOUTQEMPTY        0x000e
#define HPM_TRACEINTID_EP0SETUPSETADDRESS 0x000f
#define HPM_TRACEINTID_FRAME              0x0010
#define HPM_TRACEINTID_GETCONFIG          0x0011
#define HPM_TRACEINTID_GETSETDESC         0x0012
#define HPM_TRACEINTID_GETSETIF           0x0013
#define HPM_TRACEINTID_GETSTATUS          0x0014
#define HPM_TRACEINTID_IFGETSTATUS        0x0015
#define HPM_TRACEINTID_SETCONFIG          0x0016
#define HPM_TRACEINTID_SETFEATURE         0x0017
#define HPM_TRACEINTID_SUSPENDED          0x0018
#define HPM_TRACEINTID_RESUMED            0x0019
#define HPM_TRACEINTID_SYNCHFRAME         0x001a

#ifdef CONFIG_USBDEV_TRACE_STRINGS
const struct trace_msg_t g_usb_trace_strings_deverror[] =
{
  TRACE_STR(HPM_TRACEERR_ALLOCFAIL),
  TRACE_STR(HPM_TRACEERR_BADCLEARFEATURE),
  TRACE_STR(HPM_TRACEERR_BADDEVGETSTATUS),
  TRACE_STR(HPM_TRACEERR_BADEPNO),
  TRACE_STR(HPM_TRACEERR_BADEPGETSTATUS),
  TRACE_STR(HPM_TRACEERR_BADEPTYPE),
  TRACE_STR(HPM_TRACEERR_BADGETCONFIG),
  TRACE_STR(HPM_TRACEERR_BADGETSETDESC),
  TRACE_STR(HPM_TRACEERR_BADGETSTATUS),
  TRACE_STR(HPM_TRACEERR_BADSETADDRESS),
  TRACE_STR(HPM_TRACEERR_BADSETCONFIG),
  TRACE_STR(HPM_TRACEERR_BADSETFEATURE),
  TRACE_STR(HPM_TRACEERR_BINDFAILED),
  TRACE_STR(HPM_TRACEERR_DISPATCHSTALL),
  TRACE_STR(HPM_TRACEERR_DRIVER),
  TRACE_STR(HPM_TRACEERR_DRIVERREGISTERED),
  TRACE_STR(HPM_TRACEERR_EP0SETUPSTALLED),
  TRACE_STR(HPM_TRACEERR_EPINNULLPACKET),
  TRACE_STR(HPM_TRACEERR_EPOUTNULLPACKET),
  TRACE_STR(HPM_TRACEERR_INVALIDCTRLREQ),
  TRACE_STR(HPM_TRACEERR_INVALIDPARMS),
  TRACE_STR(HPM_TRACEERR_IRQREGISTRATION),
  TRACE_STR(HPM_TRACEERR_NOEP),
  TRACE_STR(HPM_TRACEERR_NOTCONFIGURED),
  TRACE_STR(HPM_TRACEERR_REQABORTED),
  TRACE_STR_END
};

const struct trace_msg_t g_usb_trace_strings_intdecode[] =
{
  TRACE_STR(HPM_TRACEINTID_USB),
  TRACE_STR(HPM_TRACEINTID_CLEARFEATURE),
  TRACE_STR(HPM_TRACEINTID_DEVGETSTATUS),
  TRACE_STR(HPM_TRACEINTID_DEVRESET),
  TRACE_STR(HPM_TRACEINTID_DISPATCH),
  TRACE_STR(HPM_TRACEINTID_EP0COMPLETE),
  TRACE_STR(HPM_TRACEINTID_EP0NAK),
  TRACE_STR(HPM_TRACEINTID_EP0SETUP),
  TRACE_STR(HPM_TRACEINTID_EPGETSTATUS),
  TRACE_STR(HPM_TRACEINTID_EPIN),
  TRACE_STR(HPM_TRACEINTID_EPINQEMPTY),
  TRACE_STR(HPM_TRACEINTID_EP0INSETADDRESS),
  TRACE_STR(HPM_TRACEINTID_EPOUT),
  TRACE_STR(HPM_TRACEINTID_EPOUTQEMPTY),
  TRACE_STR(HPM_TRACEINTID_EP0SETUPSETADDRESS),
  TRACE_STR(HPM_TRACEINTID_FRAME),
  TRACE_STR(HPM_TRACEINTID_GETCONFIG),
  TRACE_STR(HPM_TRACEINTID_GETSETDESC),
  TRACE_STR(HPM_TRACEINTID_GETSETIF),
  TRACE_STR(HPM_TRACEINTID_GETSTATUS),
  TRACE_STR(HPM_TRACEINTID_IFGETSTATUS),
  TRACE_STR(HPM_TRACEINTID_SETCONFIG),
  TRACE_STR(HPM_TRACEINTID_SETFEATURE),
  TRACE_STR(HPM_TRACEINTID_SUSPENDED),
  TRACE_STR(HPM_TRACEINTID_RESUMED),
  TRACE_STR(HPM_TRACEINTID_SYNCHFRAME),
  TRACE_STR_END
};
#endif

#if defined(CONFIG_ARCH_DCACHE)
#  define cache_aligned_alloc(s) kmm_memalign(HPM_L1C_CACHELINE_SIZE,(s))
#  define CACHE_ALIGNED_DATA     aligned_data(HPM_L1C_CACHELINE_SIZE)
#else
#  define cache_aligned_alloc kmm_malloc
#  define CACHE_ALIGNED_DATA
#endif

/* DMA **********************************************************************/

/* For now, we are assuming an identity mapping between physical and virtual
 * address spaces.
 */

#define hpm_physramaddr(a) core_local_mem_to_sys_address(0, a)
#define hpm_virtramaddr(a) sys_address_to_core_local_mem(0, a)

/* Hardware interface *******************************************************/

/* This represents a Endpoint Transfer Descriptor - note these must be 32
 * byte aligned.
 */

struct hpm_dtd_s
{
  volatile uint32_t       nextdesc;      /* Address of the next DMA descripto in RAM */
  volatile uint32_t       config;        /* Misc. bit encoded configuration information */
  uint32_t                buffer0;       /* Buffer start address */
  uint32_t                buffer1;       /* Buffer start address */
  uint32_t                buffer2;       /* Buffer start address */
  uint32_t                buffer3;       /* Buffer start address */
  uint32_t                buffer4;       /* Buffer start address */
  uint32_t                xfer_len;      /* Software only - transfer len that was queued */
};

/* This represents a Endpoint Transfer buffer address and length
 */

struct hpm_ep_buf_s
{
  uint32_t                address;       /* Buffer start address */
  uint32_t                xfer_len;      /* Software only - transfer len that was queued */
};


/* DTD nextdesc field */

#define DTD_NEXTDESC_INVALID         (1 << 0)    /* Bit 0     : Next Descriptor Invalid. The "Terminate" bit. */

/* DTD config field */

#define DTD_CONFIG_LENGTH(n)         ((n) << 16) /* Bits 16-31 : Total bytes to transfer */
#define DTD_CONFIG_IOC               (1 << 15)   /* Bit 15     : Interrupt on Completion */
#define DTD_CONFIG_MULT_VARIABLE     (0 << 10)   /* Bits 10-11 : Number of packets executed per transacation descriptor (override) */
#define DTD_CONFIG_MULT_NUM(n)       ((n) << 10)
#define DTD_CONFIG_ACTIVE            (1 << 7)    /* Bit 7      : Status Active */
#define DTD_CONFIG_HALTED            (1 << 6)    /* Bit 6      : Status Halted */
#define DTD_CONFIG_BUFFER_ERROR      (1 << 5)    /* Bit 6      : Status Buffer Error */
#define DTD_CONFIG_TRANSACTION_ERROR (1 << 3)    /* Bit 3      : Status Transaction Error */

/* This represents a queue head  - note these must be aligned to a 2048 byte
 * boundary
 */

struct hpm_dqh_s
{
  uint32_t                capability;  /* Endpoint capability */
  uint32_t                currdesc;    /* Current dTD pointer */
  struct hpm_dtd_s        overlay;     /* DTD overlay */
  volatile uint32_t       setup[2];    /* Set-up buffer */
  uint32_t                gap[4];      /* align to 64 bytes */
};

/* DQH capability field */

#define DQH_CAPABILITY_MULT_VARIABLE (0 << 30)    /* Bits 30-31 : Number of packets executed per transaction descriptor */
#define DQH_CAPABILITY_MULT_NUM(n)   ((n) << 30)
#define DQH_CAPABILITY_ZLT           (1 << 29)    /* Bit 29     : Zero Length Termination Select */
#define DQH_CAPABILITY_MAX_PACKET(n) ((n) << 16)  /* Bits 16-29 : Maximum packet size of associated endpoint (<1024) */
#define DQH_CAPABILITY_IOS           (1 << 15)    /* Bit 15     : Interrupt on Setup */

/* Endpoints ****************************************************************/

/* Number of endpoints */

#define HPM_NLOGENDPOINTS          (8)          /* ep0-7 */
#define HPM_NPHYSENDPOINTS         (16)         /* x2 for IN and OUT */

/* Odd physical endpoint numbers are IN; even are OUT */

#define HPM_EPPHYIN(epphy)         (((epphy) & 1) != 0)
#define HPM_EPPHYOUT(epphy)        (((epphy) & 1) == 0)

#define HPM_EPPHYIN2LOG(epphy)     (((uint8_t)(epphy) >> 1) | USB_DIR_IN)
#define HPM_EPPHYOUT2LOG(epphy)    (((uint8_t)(epphy) >> 1) | USB_DIR_OUT)

/* Endpoint 0 is special... */

#define HPM_EP0_OUT                (0)
#define HPM_EP0_IN                 (1)

/* Each endpoint has somewhat different characteristics */

#define HPM_EPALLSET               (0xffff)       /* All endpoints */
#define HPM_EPOUTSET               (0x5555)       /* Even phy endpoint numbers are OUT EPs */
#define HPM_EPINSET                (0xaaaa)       /* Odd endpoint numbers are IN EPs */
#define HPM_EPCTRLSET              (0x0003)       /* EP0 IN/OUT are control endpoints */
#define HPM_EPINTRSET              (0x000c)       /* Interrupt endpoints */
#define HPM_EPBULKSET              (0x0ff0)       /* Bulk endpoints */
#define HPM_EPISOCSET              (0xf000)       /* Isochronous endpoints */

/* Maximum packet sizes for endpoints */

#define HPM_EP0MAXPACKET           (64)         /* EP0 max packet size (1-64) */
#define HPM_BULKMAXPACKET          (512)        /* Bulk endpoint max packet (8/16/32/64/512) */
#define HPM_INTRMAXPACKET          (1024)       /* Interrupt endpoint max packet (1 to 1024) */
#define HPM_ISOCMAXPACKET          (512)        /* Acutally 1..1023 */

/* Endpoint bit position in SETUPSTAT, PRIME, FLUSH, STAT, COMPLETE
 * registers
 */

#define HPM_ENDPTSHIFT(epphy)      (HPM_EPPHYIN(epphy) ? (16 + ((epphy) >> 1)) : ((epphy) >> 1))
#define HPM_ENDPTMASK(epphy)       (1 << HPM_ENDPTSHIFT(epphy))
#define HPM_ENDPTMASK_ALL          0x00ff00ff

/* Request queue operations *************************************************/

#define hpm_rqempty(ep)            ((ep)->head == NULL)
#define hpm_rqpeek(ep)             ((ep)->head)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* A container for a request so that the request may be retained in a list */

struct hpm_req_s
{
  struct usbdev_req_s  req;           /* Standard USB request */
  struct hpm_req_s  *flink;         /* Supports a singly linked list */
};

/* This is the internal representation of an endpoint */

struct hpm_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct hpm_ep_s.
   */

  struct usbdev_ep_s      ep;          /* Standard endpoint structure */

  /* HPMXX-specific fields */

  struct hpm_usbdev_s   *dev;          /* Reference to private driver data */
  struct hpm_req_s      *head;         /* Request list for this endpoint */
  struct hpm_req_s      *tail;
  uint8_t                epphy;        /* Physical EP address */
  uint8_t                stalled:1;    /* 1: Endpoint is stalled */
};

/* This structure retains the state of the USB device controller */

struct hpm_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to struct hpm_usbdev_s.
   */

  struct usbdev_s         usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* HPM-specific fields */

  uint8_t                 ep0state;      /* State of certain EP0 operations */
                                         /* buffer for EP0 short transfers */
  uint8_t                 ep0buf[512] CACHE_ALIGNED_DATA;
  uint8_t                 paddr;         /* Address assigned by SETADDRESS */
  uint8_t                 stalled:1;     /* 1: Protocol stalled */
  uint8_t                 selfpowered:1; /* 1: Device is self powered */
  uint8_t                 paddrset:1;    /* 1: Peripheral addr has been set */
  uint8_t                 attached:1;    /* 1: Host attached */
  uint8_t                 suspended:1;   /* 1: Suspended */
  uint32_t                softprio;      /* Bitset of high priority interrupts */
  uint32_t                epavail;       /* Bitset of available endpoints */
#ifdef CONFIG_HPM_USBDEV_FRAME_INTERRUPT
  uint32_t                sof;           /* Last start-of-frame */
#endif

  uint16_t                ep0buf_len;
  struct usb_ctrlreq_s    ep0ctrl;

  /* The endpoint list */

  struct hpm_ep_s       eplist[HPM_NPHYSENDPOINTS];
};

#define EP0STATE_IDLE             0        /* Idle State, leave on receiving a setup packet or epsubmit */
#define EP0STATE_SETUP_OUT        1        /* Setup Packet received - SET/CLEAR */
#define EP0STATE_SETUP_IN         2        /* Setup Packet received - GET */
#define EP0STATE_SHORTREAD        3        /* Short read without a usb_request */
#define EP0STATE_SHORTWRITE       4        /* Short write without a usb_request */
#define EP0STATE_WAIT_NAK_OUT     5        /* Waiting for Host to illicit status phase (GET) */
#define EP0STATE_WAIT_NAK_IN      6        /* Waiting for Host to illicit status phase (SET/CLEAR) */
#define EP0STATE_WAIT_STATUS_OUT  7        /* Wait for status phase to complete */
#define EP0STATE_WAIT_STATUS_IN   8        /* Wait for status phase to complete */
#define EP0STATE_DATA_IN          9
#define EP0STATE_DATA_OUT         10

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_HPM_USBDEV_REGDEBUG
static uint32_t hpm_getreg(uint32_t addr);
static void hpm_putreg(uint32_t val, uint32_t addr);
#else
# define hpm_getreg(addr)     getreg32(addr)
# define hpm_putreg(val,addr) putreg32(val,addr)
#endif

static inline void hpm_clrbits(uint32_t mask, uint32_t addr);
static inline void hpm_setbits(uint32_t mask, uint32_t addr);
static inline void hpm_chgbits(uint32_t mask, uint32_t val, uint32_t addr);

/* Request queue operations *************************************************/

static struct hpm_req_s *hpm_rqdequeue(
    struct hpm_ep_s *privep);
static bool       hpm_rqenqueue(struct hpm_ep_s *privep,
                    struct hpm_req_s *req);

/* Low level data transfers and request operations **************************/

static inline void hpm_writedtd(struct hpm_dtd_s *dtd,
                                  const uint8_t *data,
                                  uint32_t nbytes);

static inline void hpm_record_epbuf(struct hpm_ep_buf_s *buf,
                                    const uint8_t *data,
                                    uint32_t nbytes);

static inline void hpm_queuedtd(uint8_t epphy, struct hpm_dtd_s *dtd);
static inline void hpm_ep0xfer(uint8_t epphy, uint8_t *data,
                                 uint32_t nbytes);
static void        hpm_readsetup(uint8_t epphy,
                                   struct usb_ctrlreq_s *ctrl);

static inline void hpm_set_address(struct hpm_usbdev_s *priv,
                                     uint16_t address);

static void        hpm_flushep(struct hpm_ep_s *privep);

static int         hpm_progressep(struct hpm_ep_s *privep);
static void        hpm_reqcomplete(struct hpm_ep_s *privep,
                     struct hpm_req_s *privreq, int16_t result);

static void        hpm_cancelrequests(struct hpm_ep_s *privep,
                                        int16_t status);

/* Interrupt handling *******************************************************/

static struct hpm_ep_s *hpm_epfindbyaddr(struct hpm_usbdev_s *priv,
                     uint16_t eplog);
static void        hpm_dispatchrequest(struct hpm_usbdev_s *priv,
                     const struct usb_ctrlreq_s *ctrl);
static void        hpm_ep0configure(struct hpm_usbdev_s *priv);
static void        hpm_usbreset(struct hpm_usbdev_s *priv);

static inline void hpm_ep0state(struct hpm_usbdev_s *priv,
                                  uint16_t state);
static void        hpm_ep0setup(struct hpm_usbdev_s *priv);

static void        hpm_ep0complete(struct hpm_usbdev_s *priv,
                                     uint8_t epphy);
static void        hpm_ep0nak(struct hpm_usbdev_s *priv, uint8_t epphy);
static bool        hpm_epcomplete(struct hpm_usbdev_s *priv,
                                    uint8_t epphy);

static int         hpm_usbinterrupt(int irq, void *context,
                                      void *arg);

/* Endpoint operations ******************************************************/

/* USB device controller operations *****************************************/

static int         hpm_epconfigure(struct usbdev_ep_s *ep,
                     const struct usb_epdesc_s *desc, bool last);
static int         hpm_epdisable(struct usbdev_ep_s *ep);
static struct usbdev_req_s *hpm_epallocreq(struct usbdev_ep_s *ep);
static void        hpm_epfreereq(struct usbdev_ep_s *ep,
                     struct usbdev_req_s *);
#ifdef CONFIG_USBDEV_DMA
static void       *hpm_epallocbuffer(struct usbdev_ep_s *ep,
                     uint16_t bytes);
static void        hpm_epfreebuffer(struct usbdev_ep_s *ep,
                     void *buf);
#endif
static int         hpm_epsubmit(struct usbdev_ep_s *ep,
                     struct usbdev_req_s *req);
static int         hpm_epcancel(struct usbdev_ep_s *ep,
                     struct usbdev_req_s *req);
static int         hpm_epstall(struct usbdev_ep_s *ep, bool resume);

static struct usbdev_ep_s *hpm_allocep(struct usbdev_s *dev,
                     uint8_t epno, bool in, uint8_t eptype);
static void        hpm_freeep(struct usbdev_s *dev,
                                struct usbdev_ep_s *ep);
static int         hpm_getframe(struct usbdev_s *dev);
static int         hpm_wakeup(struct usbdev_s *dev);
static int         hpm_selfpowered(struct usbdev_s *dev, bool selfpowered);
static int         hpm_pullup(struct usbdev_s *dev, bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Since there is only a single USB interface, all status information can be
 * be simply retained in a single global instance.
 */

static USB_Type *s_usb_instance;

static int s_irq_num;

ATTR_PLACE_AT_NONCACHEABLE struct hpm_usbdev_s g_usbdev;

ATTR_PLACE_AT_NONCACHEABLE struct hpm_dqh_s g_qh[HPM_NPHYSENDPOINTS]
                               aligned_data(2048);

ATTR_PLACE_AT_NONCACHEABLE struct hpm_dtd_s g_td[HPM_NPHYSENDPOINTS]
                               aligned_data(32);

static struct hpm_ep_buf_s s_ep_buf[HPM_NPHYSENDPOINTS];

static const struct usbdev_epops_s g_epops =
{
  .configure   = hpm_epconfigure,
  .disable     = hpm_epdisable,
  .allocreq    = hpm_epallocreq,
  .freereq     = hpm_epfreereq,
#ifdef CONFIG_USBDEV_DMA
  .allocbuffer = hpm_epallocbuffer,
  .freebuffer  = hpm_epfreebuffer,
#endif
  .submit      = hpm_epsubmit,
  .cancel      = hpm_epcancel,
  .stall       = hpm_epstall,
};

static const struct usbdev_ops_s g_devops =
{
  .allocep     = hpm_allocep,
  .freeep      = hpm_freeep,
  .getframe    = hpm_getframe,
  .wakeup      = hpm_wakeup,
  .selfpowered = hpm_selfpowered,
  .pullup      = hpm_pullup,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_getreg
 *
 * Description:
 *   Get the contents of an HPM3x register
 *
 ****************************************************************************/

#ifdef CONFIG_HPM_USBDEV_REGDEBUG
static uint32_t hpm_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval = 0;
  static uint32_t count = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              uinfo("...\n");
            }

          return val;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          uinfo("[repeats %d more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = val;
      count    = 1;
    }

  /* Show the register value read */

  uinfo("%08x->%08x\n", addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: hpm_putreg
 *
 * Description:
 *   Set the contents of an HPM3x register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_HPM_USBDEV_REGDEBUG
static void hpm_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  uinfo("%08x<-%08x\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: hpm_clrbits
 *
 * Description:
 *   Clear bits in a register
 *
 ****************************************************************************/

static inline void hpm_clrbits(uint32_t mask, uint32_t addr)
{
  uint32_t reg = hpm_getreg(addr);
  reg &= ~mask;
  hpm_putreg(reg, addr);
}

/****************************************************************************
 * Name: hpm_setbits
 *
 * Description:
 *   Set bits in a register
 *
 ****************************************************************************/

static inline void hpm_setbits(uint32_t mask, uint32_t addr)
{
  uint32_t reg = hpm_getreg(addr);
  reg |= mask;
  hpm_putreg(reg, addr);
}

/****************************************************************************
 * Name: hpm_chgbits
 *
 * Description:
 *   Change bits in a register
 *
 ****************************************************************************/

static inline void hpm_chgbits(uint32_t mask, uint32_t val, uint32_t addr)
{
  uint32_t reg = hpm_getreg(addr);
  reg &= ~mask;
  reg |= val;
  hpm_putreg(reg, addr);
}

/****************************************************************************
 * Name: hpm_rqdequeue
 *
 * Description:
 *   Remove a request from an endpoint request queue
 *
 ****************************************************************************/

static struct hpm_req_s *hpm_rqdequeue(struct hpm_ep_s *privep)
{
  struct hpm_req_s *ret = privep->head;

  if (ret)
    {
      privep->head = ret->flink;
      if (!privep->head)
        {
          privep->tail = NULL;
        }

      ret->flink = NULL;
    }

  return ret;
}

/****************************************************************************
 * Name: hpm_rqenqueue
 *
 * Description:
 *   Add a request from an endpoint request queue
 *
 ****************************************************************************/

static bool hpm_rqenqueue(struct hpm_ep_s *privep,
                            struct hpm_req_s *req)
{
  bool is_empty = !privep->head;

  req->flink = NULL;
  if (is_empty)
    {
      privep->head = req;
      privep->tail = req;
    }
  else
    {
      privep->tail->flink = req;
      privep->tail        = req;
    }

  return is_empty;
}

/****************************************************************************
 * Name: hpm_writedtd
 *
 * Description:
 *   Initialise a DTD to transfer the data
 *
 ****************************************************************************/

static inline void hpm_writedtd(struct hpm_dtd_s *dtd,
                                  const uint8_t *data,
                                  uint32_t nbytes)
{
  dtd->nextdesc  = DTD_NEXTDESC_INVALID;
  dtd->config    = DTD_CONFIG_LENGTH(nbytes) | DTD_CONFIG_IOC |
      DTD_CONFIG_ACTIVE;
  dtd->buffer0   = hpm_physramaddr((uint32_t) data);
  dtd->buffer1   = hpm_physramaddr(((uint32_t) data) + 0x1000) & 0xfffff000;
  dtd->buffer2   = hpm_physramaddr(((uint32_t) data) + 0x2000) & 0xfffff000;
  dtd->buffer3   = hpm_physramaddr(((uint32_t) data) + 0x3000) & 0xfffff000;
  dtd->buffer4   = hpm_physramaddr(((uint32_t) data) + 0x4000) & 0xfffff000;
  dtd->xfer_len  = nbytes;

  up_flush_dcache((uintptr_t)data,
                  (uintptr_t)HPM_L1C_CACHELINE_ALIGN_UP((uint32_t)data + nbytes));
}

/****************************************************************************
 * Name: hpm_record_epbuf
 *
 * Description:
 *   Record Ep Buffer Address and Length
 *
 ****************************************************************************/

static inline void hpm_record_epbuf(struct hpm_ep_buf_s *buf,
                                    const uint8_t *data,
                                    uint32_t nbytes)
{
  buf->address   = hpm_physramaddr((uint32_t) data);
  buf->xfer_len  = nbytes;
}

/****************************************************************************
 * Name: hpm_queuedtd
 *
 * Description:
 *   Add the DTD to the device list
 *
 * Assumptions:
 *   DTD is already flushed to RAM.
 *
 ****************************************************************************/

static void hpm_queuedtd(uint8_t epphy, struct hpm_dtd_s *dtd)
{
  struct hpm_dqh_s *dqh = &g_qh[epphy];

  /* Queue the DTD onto the Endpoint
   * NOTE - this only works when no DTD is currently queued
   */

  dqh->overlay.nextdesc = hpm_physramaddr((uint32_t)dtd);
  dqh->overlay.config  &= ~(DTD_CONFIG_ACTIVE | DTD_CONFIG_HALTED);

  uint32_t bit = HPM_ENDPTMASK(epphy);

  hpm_setbits(bit, (uint32_t)(&s_usb_instance->ENDPTPRIME));

  while (hpm_getreg((uint32_t)(&s_usb_instance->ENDPTPRIME)) & bit)
    ;
}

/****************************************************************************
 * Name: hpm_ep0xfer
 *
 * Description:
 *   Schedule a short transfer for Endpoint 0 (IN or OUT)
 *
 ****************************************************************************/

static inline void hpm_ep0xfer(uint8_t epphy, uint8_t *buf,
                                 uint32_t nbytes)
{
  struct hpm_dtd_s *dtd = &g_td[epphy];
  struct hpm_ep_buf_s *ep_buf = &s_ep_buf[epphy];

  hpm_writedtd(dtd, buf, nbytes);

  hpm_record_epbuf(ep_buf, buf, nbytes);

  hpm_queuedtd(epphy, dtd);
}

/****************************************************************************
 * Name: hpm_readsetup
 *
 * Description:
 *   Read a Setup packet from the DTD.
 *
 ****************************************************************************/

static void hpm_readsetup(uint8_t epphy, struct usb_ctrlreq_s *ctrl)
{
  struct hpm_dqh_s *dqh = &g_qh[epphy];
  int i;

  do
    {
      /* Set the trip wire */

      hpm_setbits(USB_USBCMD_SUTW_MASK, (uint32_t)(&s_usb_instance->USBCMD));

      /* Copy the request... */

      for (i = 0; i < 8; i++)
        {
          ((uint8_t *) ctrl)[i] = ((uint8_t *) dqh->setup)[i];
        }
    }
  while (!(hpm_getreg((uint32_t)(&s_usb_instance->USBCMD)) & USB_USBCMD_SUTW_MASK));

  /* Clear the trip wire */

  hpm_clrbits(USB_USBCMD_SUTW_MASK, (uint32_t)(&s_usb_instance->USBCMD));

  /* Clear the Setup Interrupt */

  hpm_putreg(HPM_ENDPTMASK(HPM_EP0_OUT), (uint32_t)(&s_usb_instance->ENDPTSETUPSTAT));
}

/****************************************************************************
 * Name: hpm_set_address
 *
 * Description:
 *   Set the devices USB address
 *
 ****************************************************************************/

static inline void hpm_set_address(struct hpm_usbdev_s *priv,
                                     uint16_t address)
{
  priv->paddr    = address;
  priv->paddrset = address != 0;

  hpm_chgbits(USB_DEVICEADDR_USBADR_MASK,
                USB_DEVICEADDR_USBADR_SET(priv->paddr),
                (uint32_t)(&s_usb_instance->DEVICEADDR));
}

/****************************************************************************
 * Name: hpm_flushep
 *
 * Description:
 *   Flush any primed descriptors from this ep
 *
 ****************************************************************************/

static void hpm_flushep(struct hpm_ep_s *privep)
{
  uint32_t mask = HPM_ENDPTMASK(privep->epphy);
  do
    {
      hpm_putreg(mask, (uint32_t)(&s_usb_instance->ENDPTFLUSH));
      while ((hpm_getreg((uint32_t)(&s_usb_instance->ENDPTFLUSH)) & mask) != 0)
      ;
    }
  while ((hpm_getreg((uint32_t)(&s_usb_instance->ENDPTSTAT)) & mask) != 0);
}

/****************************************************************************
 * Name: hpm_progressep
 *
 * Description:
 *   Progress the Endpoint by priming the first request into the queue head
 *
 ****************************************************************************/

static int hpm_progressep(struct hpm_ep_s *privep)
{
  struct hpm_dtd_s *dtd = &g_td[privep->epphy];
  struct hpm_ep_buf_s *ep_buf = &s_ep_buf[privep->epphy];
  struct hpm_req_s *privreq;

  /* Check the request from the head of the endpoint request queue */

  privreq = hpm_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_EPINQEMPTY), 0);
      return OK;
    }

  /* Ignore any attempt to send a zero length packet */

  if (privreq->req.len == 0)
    {
      /* If the class driver is responding to a setup packet, then wait for
       * the host to illicit the response
       */

      if (privep->epphy == HPM_EP0_IN &&
          privep->dev->ep0state == EP0STATE_SETUP_OUT)
        {
          hpm_ep0state(privep->dev, EP0STATE_WAIT_NAK_IN);
        }
      else
        {
          if (HPM_EPPHYIN(privep->epphy))
            {
              usbtrace(TRACE_DEVERROR(HPM_TRACEERR_EPINNULLPACKET), 0);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(HPM_TRACEERR_EPOUTNULLPACKET), 0);
            }
        }

      hpm_reqcomplete(privep, hpm_rqdequeue(privep), OK);
      return OK;
    }

  if (privep->epphy == HPM_EP0_IN)
    {
      hpm_ep0state(privep->dev,  EP0STATE_DATA_IN);
    }
  else if (privep->epphy == HPM_EP0_OUT)
    {
      hpm_ep0state(privep->dev, EP0STATE_DATA_OUT);
    }

  int bytesleft = privreq->req.len - privreq->req.xfrd;

  if (HPM_EPPHYIN(privep->epphy))
    {
      usbtrace(TRACE_WRITE(privep->epphy), privreq->req.xfrd);
    }
  else
    {
      usbtrace(TRACE_READ(privep->epphy), privreq->req.xfrd);
    }

  /* Initialise the DTD to transfer the next chunk */

  hpm_writedtd(dtd, privreq->req.buf + privreq->req.xfrd, bytesleft);

  /* Record ep buf addr and length */
  hpm_record_epbuf(ep_buf, privreq->req.buf, privreq->req.xfrd + bytesleft);

  /* Then queue onto the DQH */

  hpm_queuedtd(privep->epphy, dtd);

  return OK;
}

/****************************************************************************
 * Name: hpm_reqcomplete
 *
 * Description:
 *   Handle termination of the request at the head of the endpoint request
 *   queue.
 *
 ****************************************************************************/

static void hpm_reqcomplete(struct hpm_ep_s *privep,
                              struct hpm_req_s *privreq, int16_t result)
{
  /* If endpoint 0, temporarily reflect the state of protocol stalled
   * in the callback.
   */

  bool stalled = privep->stalled;
  if (privep->epphy == HPM_EP0_IN)
    privep->stalled = privep->dev->stalled;

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);

  /* Restore the stalled indication */

  privep->stalled = stalled;
}

/****************************************************************************
 * Name: hpm_cancelrequests
 *
 * Description:
 *   Cancel all pending requests for an endpoint
 *
 ****************************************************************************/

static void hpm_cancelrequests(struct hpm_ep_s *privep, int16_t status)
{
  if (!hpm_rqempty(privep))
      hpm_flushep(privep);

  while (!hpm_rqempty(privep))
    {
      /* FIXME: the entry at the head should be sync'd with the DTD
       * FIXME: only report the error status if the transfer hasn't completed
       */

      usbtrace(TRACE_COMPLETE(privep->epphy),
               (hpm_rqpeek(privep))->req.xfrd);
      hpm_reqcomplete(privep, hpm_rqdequeue(privep), status);
    }
}

/****************************************************************************
 * Name: hpm_epfindbyaddr
 *
 * Description:
 *   Find the physical endpoint structure corresponding to a logic endpoint
 *   address
 *
 ****************************************************************************/

static struct hpm_ep_s *hpm_epfindbyaddr(struct hpm_usbdev_s *priv,
                         uint16_t eplog)
{
  struct hpm_ep_s *privep;
  int i;

  /* Endpoint zero is a special case */

  if (USB_EPNO(eplog) == 0)
    {
      return &priv->eplist[0];
    }

  /* Handle the remaining */

  for (i = 1; i < HPM_NPHYSENDPOINTS; i++)
    {
      privep = &priv->eplist[i];

      /* Same logical endpoint number? (includes direction bit) */

      if (eplog == privep->ep.eplog)
        {
          /* Return endpoint found */

          return privep;
        }
    }

  /* Return endpoint not found */

  return NULL;
}

/****************************************************************************
 * Name: hpm_dispatchrequest
 *
 * Description:
 *   Provide unhandled setup actions to the class driver. This is logically
 *   part of the USB interrupt handler.
 *
 ****************************************************************************/

static void hpm_dispatchrequest(struct hpm_usbdev_s *priv,
                                    const struct usb_ctrlreq_s *ctrl)
{
  int ret = -EIO;

  usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_DISPATCH), 0);
  if (priv->driver)
    {
      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(priv->driver, &priv->usbdev, ctrl, priv->ep0buf,
                        priv->ep0buf_len);
    }

  if (ret < 0)
    {
      /* Stall on failure */

      usbtrace(TRACE_DEVERROR(HPM_TRACEERR_DISPATCHSTALL), 0);
      priv->stalled = true;
    }
}

/****************************************************************************
 * Name: hpm_ep0configure
 *
 * Description:
 *   Reset Usb engine
 *
 ****************************************************************************/

static void hpm_ep0configure(struct hpm_usbdev_s *priv)
{
  /* Enable ep0 IN and ep0 OUT */

  g_qh[HPM_EP0_OUT].capability =
      (DQH_CAPABILITY_MAX_PACKET(CONFIG_USBDEV_EP0_MAXSIZE) |
          DQH_CAPABILITY_IOS | DQH_CAPABILITY_ZLT);

  g_qh[HPM_EP0_IN].capability =
      (DQH_CAPABILITY_MAX_PACKET(CONFIG_USBDEV_EP0_MAXSIZE) |
          DQH_CAPABILITY_IOS | DQH_CAPABILITY_ZLT);

  g_qh[HPM_EP0_OUT].currdesc = DTD_NEXTDESC_INVALID;
  g_qh[HPM_EP0_IN].currdesc = DTD_NEXTDESC_INVALID;

  /* Enable EP0 */

  hpm_setbits(USB_ENDPTCTRL_RXE_MASK | USB_ENDPTCTRL_TXE_MASK,
              (uint32_t)(&s_usb_instance->ENDPTCTRL[0]));
}

/****************************************************************************
 * Name: hpm_usbreset
 *
 * Description:
 *   Reset Usb engine
 *
 ****************************************************************************/

static void hpm_usbreset(struct hpm_usbdev_s *priv)
{
  int epphy;

  /* Disable all endpoints. Control endpoint 0 is always enabled */
  for (uint8_t i = 1; i < 8; i++)
    {
      hpm_clrbits(USB_ENDPTCTRL_RXE_MASK | USB_ENDPTCTRL_TXE_MASK,
                  (uint32_t)(&s_usb_instance->ENDPTCTRL[i]));
    }

  /* Clear all pending interrupts */

  hpm_putreg(hpm_getreg(s_usb_instance->ENDPTNAK),
               (uint32_t)(&s_usb_instance->ENDPTNAK));
  hpm_putreg(hpm_getreg(s_usb_instance->ENDPTSETUPSTAT),
               (uint32_t)(&s_usb_instance->ENDPTSETUPSTAT));
  hpm_putreg(hpm_getreg(s_usb_instance->ENDPTCOMPLETE),
               (uint32_t)(&s_usb_instance->ENDPTCOMPLETE));

  /* Wait for all prime operations to have completed and then flush all
   * DTDs
   */

  while (hpm_getreg((uint32_t)(&s_usb_instance->ENDPTPRIME)) != 0)
    ;
  hpm_putreg(HPM_ENDPTMASK_ALL, (uint32_t)(&s_usb_instance->ENDPTFLUSH));
  while (hpm_getreg((uint32_t)(&s_usb_instance->ENDPTFLUSH)) != 0)
    ;

  /* Reset endpoints */

  for (epphy = 0; epphy < HPM_NPHYSENDPOINTS; epphy++)
    {
      struct hpm_ep_s *privep = &priv->eplist[epphy];

      hpm_cancelrequests(privep, -ESHUTDOWN);

      /* Reset endpoint status */

      privep->stalled = false;
    }

  /* Tell the class driver that we are disconnected. The class
   * driver should then accept any new configurations.
   */

  if (priv->driver)
    {
      CLASS_DISCONNECT(priv->driver, &priv->usbdev);
    }

  /* Set the interrupt Threshold control interval to 0 */

  hpm_chgbits(USB_USBCMD_ITC_MASK, USB_USBCMD_ITC_SET(0),
                (uint32_t)(&s_usb_instance->USBCMD));

  /* Zero out the Endpoint queue heads */

  memset ((void *) g_qh, 0, sizeof (g_qh));
  memset ((void *) g_td, 0, sizeof (g_td));

  /* Set USB address to 0 */

  hpm_set_address(priv, 0);

  /* Initialise the Enpoint List Address */

  hpm_putreg(hpm_physramaddr((uint32_t)g_qh), (uint32_t)(&s_usb_instance->ENDPTLISTADDR));

  /* EndPoint 0 initialization */

  hpm_ep0configure(priv);

  /* Enable Device interrupts */

  hpm_putreg(USB_FRAME_INT | USB_ERROR_INT | USB_USBINTR_NAKE_MASK |
               USB_USBINTR_SLE_MASK | USB_USBINTR_URE_MASK | USB_USBINTR_PCE_MASK |
               USB_USBINTR_UE_MASK, (uint32_t)(&s_usb_instance->USBINTR));
}

/****************************************************************************
 * Name: hpm_setstate
 *
 * Description:
 *   Sets the EP0 state and manages the NAK interrupts
 *
 ****************************************************************************/

static inline void hpm_ep0state(struct hpm_usbdev_s *priv,
                                  uint16_t state)
{
  priv->ep0state = state;

  switch (state)
    {
    case EP0STATE_WAIT_NAK_IN:
      hpm_putreg(HPM_ENDPTMASK(HPM_EP0_IN), (uint32_t)(&s_usb_instance->ENDPTNAKEN));
      break;

    case EP0STATE_WAIT_NAK_OUT:
      hpm_putreg(HPM_ENDPTMASK(HPM_EP0_OUT), (uint32_t)(&s_usb_instance->ENDPTNAKEN));
      break;

    default:
      hpm_putreg(0, (uint32_t)(&s_usb_instance->ENDPTNAKEN));
      break;
    }
}

/****************************************************************************
 * Name: hpm_ep0setup
 *
 * Description:
 *   USB Ctrl EP Setup Event. This is logically part of the USB interrupt
 *   handler.  This event occurs when a setup packet is receive on EP0 OUT.
 *
 ****************************************************************************/

static inline void hpm_ep0setup(struct hpm_usbdev_s *priv)
{
  struct hpm_ep_s *privep;
  struct usb_ctrlreq_s *ctrl;
  uint16_t value;
  uint16_t index;
  uint16_t len;

  ctrl = &priv->ep0ctrl;

  /* Terminate any pending requests - since all DTDs will have been retired
   * because of the setup packet.
   */

  hpm_cancelrequests(&priv->eplist[HPM_EP0_OUT], -EPROTO);
  hpm_cancelrequests(&priv->eplist[HPM_EP0_IN],  -EPROTO);

  /* Assume NOT stalled */

  priv->eplist[HPM_EP0_OUT].stalled = false;
  priv->eplist[HPM_EP0_IN].stalled = false;
  priv->stalled = false;

  /* Read EP0 setup data */

  hpm_readsetup(HPM_EP0_OUT, ctrl);

  /* And extract the little-endian 16-bit values to host order */

  value = GETUINT16(ctrl->value);
  index = GETUINT16(ctrl->index);
  len   = GETUINT16(ctrl->len);

  priv->ep0buf_len = len;

  uinfo("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        ctrl->type, ctrl->req, value, index, len);

  /* Starting a control request - update state */

  if (ctrl->type & USB_REQ_DIR_IN)
    {
      hpm_ep0state(priv, EP0STATE_SETUP_IN);
    }
  else
    {
      hpm_ep0state(priv, EP0STATE_SETUP_OUT);

      if (len > 0)
        {
          hpm_ep0state(priv, EP0STATE_SHORTREAD);
          hpm_ep0xfer(HPM_EP0_OUT, priv->ep0buf, len);
          return;
        }
    }

  /* Dispatch any non-standard requests */

  if ((ctrl->type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      hpm_dispatchrequest(priv, ctrl);
    }
  else
    {
      /* Handle standard request.  Pick off the things of interest to the USB
       * device controller driver; pass what is left to the class driver.
       */

      switch (ctrl->req)
        {
        case USB_REQ_GETSTATUS:
          {
            /* type:  device-to-host; recipient = device, interface, endpoint
             * value: 0
             * index: zero interface endpoint
             * len:   2; data = status
             */

            usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_GETSTATUS), 0);
            if (!priv->paddrset || len != 2 ||
                (ctrl->type & USB_REQ_DIR_IN) == 0 || value != 0)
              {
                priv->stalled = true;
              }
            else
              {
                switch (ctrl->type & USB_REQ_RECIPIENT_MASK)
                  {
                  case USB_REQ_RECIPIENT_ENDPOINT:
                    {
                      usbtrace(
                          TRACE_INTDECODE(HPM_TRACEINTID_EPGETSTATUS), 0);
                      privep = hpm_epfindbyaddr(priv, index);
                      if (!privep)
                        {
                          usbtrace(
                              TRACE_DEVERROR(HPM_TRACEERR_BADEPGETSTATUS),
                              0);
                          priv->stalled = true;
                        }
                      else
                        {
                          if (privep->stalled)
                            {
                              priv->ep0buf[0] = 1; /* Stalled */
                            }
                          else
                            {
                              priv->ep0buf[0] = 0; /* Not stalled */
                            }

                          priv->ep0buf[1] = 0;

                          hpm_ep0xfer(HPM_EP0_IN, priv->ep0buf, 2);
                          hpm_ep0state(priv, EP0STATE_SHORTWRITE);
                        }
                    }
                    break;

                  case USB_REQ_RECIPIENT_DEVICE:
                    {
                      if (index == 0)
                        {
                          usbtrace(
                              TRACE_INTDECODE(HPM_TRACEINTID_DEVGETSTATUS),
                              0);

                          /* Features:  Remote Wakeup=YES; selfpowered=? */

                          priv->ep0buf[0] =
                              (priv->selfpowered <<
                                  USB_FEATURE_SELFPOWERED) |
                              (1 << USB_FEATURE_REMOTEWAKEUP);
                          priv->ep0buf[1] = 0;

                          hpm_ep0xfer(HPM_EP0_IN, priv->ep0buf, 2);
                          hpm_ep0state(priv, EP0STATE_SHORTWRITE);
                        }
                      else
                        {
                          usbtrace(
                              TRACE_DEVERROR(HPM_TRACEERR_BADDEVGETSTATUS),
                              0);
                          priv->stalled = true;
                        }
                    }
                    break;

                  case USB_REQ_RECIPIENT_INTERFACE:
                    {
                      usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_IFGETSTATUS),
                               0);
                      priv->ep0buf[0] = 0;
                      priv->ep0buf[1] = 0;

                      hpm_ep0xfer(HPM_EP0_IN, priv->ep0buf, 2);
                      hpm_ep0state(priv, EP0STATE_SHORTWRITE);
                    }
                    break;

                  default:
                    {
                      usbtrace(TRACE_DEVERROR(HPM_TRACEERR_BADGETSTATUS),
                               0);
                      priv->stalled = true;
                    }
                    break;
                }
            }
        }
        break;

      case USB_REQ_CLEARFEATURE:
        {
          /* type:  host-to-device; recipient = device, interface or endpoint
           * value: feature selector
           * index: zero interface endpoint;
           * len:   zero, data = none
           */

          usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_CLEARFEATURE), 0);
          if ((ctrl->type & USB_REQ_RECIPIENT_MASK) !=
              USB_REQ_RECIPIENT_ENDPOINT)
            {
              hpm_dispatchrequest(priv, ctrl);
            }
          else if (priv->paddrset != 0 &&
              value == USB_FEATURE_ENDPOINTHALT &&
              len == 0 && (privep = hpm_epfindbyaddr(priv, index)) != NULL)
            {
              hpm_epstall(&privep->ep, true);
              hpm_ep0state(priv, EP0STATE_WAIT_NAK_IN);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(HPM_TRACEERR_BADCLEARFEATURE), 0);
              priv->stalled = true;
            }
        }
        break;

      case USB_REQ_SETFEATURE:
        {
          /* type:  host-to-device; recipient = device, interface, endpoint
           * value: feature selector
           * index: zero interface endpoint;
           * len:   0; data = none
           */

          usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_SETFEATURE), 0);
          if (((ctrl->type & USB_REQ_RECIPIENT_MASK) ==
              USB_REQ_RECIPIENT_DEVICE) && value == USB_FEATURE_TESTMODE)
            {
              uinfo("test mode: %d\n", index);
            }
          else if ((ctrl->type & USB_REQ_RECIPIENT_MASK) !=
              USB_REQ_RECIPIENT_ENDPOINT)
            {
              hpm_dispatchrequest(priv, ctrl);
            }
          else if (priv->paddrset != 0 &&
              value == USB_FEATURE_ENDPOINTHALT &&
              len == 0 && (privep = hpm_epfindbyaddr(priv, index)) != NULL)
            {
              hpm_epstall(&privep->ep, false);
              hpm_ep0state(priv, EP0STATE_WAIT_NAK_IN);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(HPM_TRACEERR_BADSETFEATURE), 0);
              priv->stalled = true;
            }
        }
        break;

      case USB_REQ_SETADDRESS:
        {
          /* type:  host-to-device; recipient = device
           * value: device address
           * index: 0
           * len:   0; data = none
           */

          usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_EP0SETUPSETADDRESS),
                   value);
          if (((ctrl->type & USB_REQ_RECIPIENT_MASK) ==
              USB_REQ_RECIPIENT_DEVICE) &&
              index == 0 && len == 0 && value < 128)
            {
              /* Save the address.  We cannot actually change to the next
               * address until the completion of the status phase.
               */

              priv->paddr = ctrl->value[0];
              priv->paddrset = false;
              hpm_ep0state(priv, EP0STATE_WAIT_NAK_IN);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(HPM_TRACEERR_BADSETADDRESS), 0);
              priv->stalled = true;
            }
        }
        break;

      case USB_REQ_GETDESCRIPTOR:
        /* type:  device-to-host; recipient = device
         * value: descriptor type and index
         * index: 0 or language ID;
         * len:   descriptor len; data = descriptor
         */

      case USB_REQ_SETDESCRIPTOR:
        /* type:  host-to-device; recipient = device
         * value: descriptor type and index
         * index: 0 or language ID;
         * len:   descriptor len; data = descriptor
         */

        {
          usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_GETSETDESC), 0);
          if ((ctrl->type & USB_REQ_RECIPIENT_MASK) ==
              USB_REQ_RECIPIENT_DEVICE)
            {
              hpm_dispatchrequest(priv, ctrl);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(HPM_TRACEERR_BADGETSETDESC), 0);
              priv->stalled = true;
            }
        }
        break;

      case USB_REQ_GETCONFIGURATION:
        /* type:  device-to-host; recipient = device
         * value: 0;
         * index: 0;
         * len:   1; data = configuration value
         */

        {
          usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_GETCONFIG), 0);
          if (priv->paddrset &&
              ((ctrl->type & USB_REQ_RECIPIENT_MASK) ==
                  USB_REQ_RECIPIENT_DEVICE) &&
                  value == 0 && index == 0 && len == 1)
            {
              hpm_dispatchrequest(priv, ctrl);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(HPM_TRACEERR_BADGETCONFIG), 0);
              priv->stalled = true;
            }
        }
        break;

      case USB_REQ_SETCONFIGURATION:
        /* type:  host-to-device; recipient = device
         * value: configuration value
         * index: 0;
         * len:   0; data = none
         */

        {
          usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_SETCONFIG), 0);
          if (((ctrl->type & USB_REQ_RECIPIENT_MASK) ==
              USB_REQ_RECIPIENT_DEVICE) && index == 0 && len == 0)
            {
              hpm_dispatchrequest(priv, ctrl);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(HPM_TRACEERR_BADSETCONFIG), 0);
              priv->stalled = true;
            }
        }
        break;

      case USB_REQ_GETINTERFACE:
        /* type:  device-to-host; recipient = interface
         * value: 0
         * index: interface;
         * len:   1; data = alt interface
         */

      case USB_REQ_SETINTERFACE:
        /* type:  host-to-device; recipient = interface
         * value: alternate setting
         * index: interface;
         * len:   0; data = none
         */

        {
          usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_GETSETIF), 0);
          hpm_dispatchrequest(priv, ctrl);
        }
        break;

      case USB_REQ_SYNCHFRAME:
        /* type:  device-to-host; recipient = endpoint
         * value: 0
         * index: endpoint;
         * len:   2; data = frame number
         */

        {
          usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_SYNCHFRAME), 0);
        }
        break;

      default:
        {
          usbtrace(TRACE_DEVERROR(HPM_TRACEERR_INVALIDCTRLREQ), 0);
          priv->stalled = true;
        }
        break;
      }
  }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(HPM_TRACEERR_EP0SETUPSTALLED),
               priv->ep0state);
      hpm_epstall(&priv->eplist[HPM_EP0_IN].ep, false);
      hpm_epstall(&priv->eplist[HPM_EP0_OUT].ep, false);
    }
}

/****************************************************************************
 * Name: hpm_ep0complete
 *
 * Description:
 *   Transfer complete handler for Endpoint 0
 *
 ****************************************************************************/

static void hpm_ep0complete(struct hpm_usbdev_s *priv, uint8_t epphy)
{
  struct hpm_ep_s *privep = &priv->eplist[epphy];

  usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_EP0COMPLETE),
           (uint16_t)priv->ep0state);

  switch (priv->ep0state)
    {
    case EP0STATE_DATA_IN:
      if (hpm_rqempty(privep))
        {
          return;
        }

      if (hpm_epcomplete(priv, epphy))
        {
          hpm_ep0state(priv, EP0STATE_WAIT_NAK_OUT);
        }
      break;

    case EP0STATE_DATA_OUT:
      if (hpm_rqempty(privep))
        {
          return;
        }

      if (hpm_epcomplete(priv, epphy))
        {
          hpm_ep0state(priv, EP0STATE_WAIT_NAK_IN);
        }
      break;

    case EP0STATE_SHORTREAD:

      /* Make sure we have updated data after the DMA transfer.
       * This invalidation matches the flush in writedtd().
       */

      up_invalidate_dcache((uintptr_t)priv->ep0buf,
                           (uintptr_t)(priv->ep0buf + sizeof(priv->ep0buf)));

      hpm_dispatchrequest(priv, &priv->ep0ctrl);
      hpm_ep0state(priv, EP0STATE_WAIT_NAK_IN);
      break;

    case EP0STATE_SHORTWRITE:
      hpm_ep0state(priv, EP0STATE_WAIT_NAK_OUT);
      break;

    case EP0STATE_WAIT_STATUS_IN:
      hpm_ep0state(priv, EP0STATE_IDLE);

      /* If we've received a SETADDRESS packet, then we set the address
       * now that the status phase has completed
       */

      if (! priv->paddrset && priv->paddr != 0)
        {
          usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_EP0INSETADDRESS),
                   (uint16_t)priv->paddr);
          hpm_set_address(priv, priv->paddr);
        }

      break;

    case EP0STATE_WAIT_STATUS_OUT:
      hpm_ep0state(priv, EP0STATE_IDLE);
      break;

    default:
#ifdef CONFIG_DEBUG_FEATURES
      DEBUGASSERT(priv->ep0state != EP0STATE_DATA_IN &&
          priv->ep0state != EP0STATE_DATA_OUT        &&
          priv->ep0state != EP0STATE_SHORTWRITE      &&
          priv->ep0state != EP0STATE_WAIT_STATUS_IN  &&
          priv->ep0state != EP0STATE_WAIT_STATUS_OUT);
#endif
      priv->stalled = true;
      break;
    }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(HPM_TRACEERR_EP0SETUPSTALLED),
               priv->ep0state);
      hpm_epstall(&priv->eplist[HPM_EP0_IN].ep, false);
      hpm_epstall(&priv->eplist[HPM_EP0_OUT].ep, false);
    }
}

/****************************************************************************
 * Name: hpm_ep0nak
 *
 * Description:
 *   Handle a NAK interrupt on EP0
 *
 ****************************************************************************/

static void hpm_ep0nak(struct hpm_usbdev_s *priv, uint8_t epphy)
{
  usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_EP0NAK),
           (uint16_t)priv->ep0state);

  switch (priv->ep0state)
    {
    case EP0STATE_WAIT_NAK_IN:
      hpm_ep0xfer(HPM_EP0_IN, NULL, 0);
      hpm_ep0state(priv, EP0STATE_WAIT_STATUS_IN);
      break;

    case EP0STATE_WAIT_NAK_OUT:
      hpm_ep0xfer(HPM_EP0_OUT, NULL, 0);
      hpm_ep0state(priv, EP0STATE_WAIT_STATUS_OUT);
      break;

    default:
#ifdef CONFIG_DEBUG_FEATURES
      DEBUGASSERT(priv->ep0state != EP0STATE_WAIT_NAK_IN &&
                  priv->ep0state != EP0STATE_WAIT_NAK_OUT);
#endif
      priv->stalled = true;
      break;
    }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(HPM_TRACEERR_EP0SETUPSTALLED),
               priv->ep0state);
      hpm_epstall(&priv->eplist[HPM_EP0_IN].ep, false);
      hpm_epstall(&priv->eplist[HPM_EP0_OUT].ep, false);
    }
}

/****************************************************************************
 * Name: hpm_epcomplete
 *
 * Description:
 *   Transfer complete handler for Endpoints other than 0
 *   returns whether the request at the head has completed
 *
 ****************************************************************************/

bool hpm_epcomplete(struct hpm_usbdev_s *priv, uint8_t epphy)
{
  struct hpm_ep_s  *privep  = &priv->eplist[epphy];
  struct hpm_req_s *privreq = privep->head;
  struct hpm_dtd_s *dtd     = &g_td[epphy];
  struct hpm_ep_buf_s *ep_buf = &s_ep_buf[privep->epphy];

  if (privreq == NULL)        /* This shouldn't really happen */
    {
      if (HPM_EPPHYOUT(privep->epphy))
        {
          usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_EPINQEMPTY), 0);
        }
      else
        {
          usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_EPOUTQEMPTY), 0);
        }

      return true;
    }

  /* Make sure we have updated data after the DMA transfer.
   * This invalidation matches the flush in writedtd().
   */
  up_invalidate_dcache((uintptr_t)ep_buf->address, 
                       (uintptr_t)HPM_L1C_CACHELINE_ALIGN_UP(ep_buf->address + ep_buf->xfer_len));

  int xfrd = dtd->xfer_len - (dtd->config >> 16);

  privreq->req.xfrd += xfrd;

  bool complete = true;
  if (HPM_EPPHYOUT(privep->epphy))
    {
      /* read(OUT) completes when request filled, or a short transfer is
       * received
       */

      usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_EPIN), complete);
    }
  else
    {
      /* write(IN) completes when request finished, unless we need to
       * terminate with a ZLP
       */

      bool need_zlp = (xfrd == privep->ep.maxpacket) &&
          ((privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0);

      complete = (privreq->req.xfrd >= privreq->req.len && !need_zlp);

      usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_EPOUT), complete);
    }

  /* If the transfer is complete, then dequeue and progress any further
   * queued requests
   */

  if (complete)
    {
      privreq = hpm_rqdequeue(privep);
    }

  if (!hpm_rqempty(privep))
    {
      hpm_progressep(privep);
    }

  /* Now it's safe to call the completion callback as it may well submit a
   * new request
   */

  if (complete)
    {
      usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
      hpm_reqcomplete(privep, privreq, OK);
    }

  return complete;
}

/****************************************************************************
 * Name: hpm_usbinterrupt
 *
 * Description:
 *   USB interrupt handler
 *
 ****************************************************************************/

static int hpm_usbinterrupt(int irq, void *context, void *arg)
{
  struct hpm_usbdev_s *priv = &g_usbdev;
  uint32_t disr;
  uint32_t portsc1;
  uint32_t n;

  usbtrace(TRACE_INTENTRY(HPM_TRACEINTID_USB), 0);

  /* Read the interrupts and then clear them */

  disr = hpm_getreg((uint32_t)(&s_usb_instance->USBSTS));
  hpm_putreg(disr, (uint32_t)(&s_usb_instance->USBSTS));

  if (disr & USB_USBSTS_URI_MASK)
    {
      usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_DEVRESET), 0);

      hpm_usbreset(priv);

      usbtrace(TRACE_INTEXIT(HPM_TRACEINTID_USB), 0);
    }

  /* When the device controller enters a suspend state from an active state,
   * the SLI bit will be set to a one.
   */

  if (!priv->suspended && (disr & USB_USBSTS_SLI_MASK) != 0)
    {
      usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_SUSPENDED), 0);

      /* Inform the Class driver of the suspend event */

      priv->suspended = 1;
      if (priv->driver)
        {
          CLASS_SUSPEND(priv->driver, &priv->usbdev);
        }

      /* TODO: Perform power management operations here. */
    }

  /* The device controller clears the SLI bit upon exiting from a suspend
   * state. This bit can also be cleared by software writing a one to it.
   */

  else if (priv->suspended && (disr & USB_USBSTS_SLI_MASK) == 0)
    {
      usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_RESUMED), 0);

      /* Inform the Class driver of the resume event */

      priv->suspended = 0;
      if (priv->driver)
        {
          CLASS_RESUME(priv->driver, &priv->usbdev);
        }

      /* TODO: Perform power management operations here. */
    }

  if (disr & USB_USBSTS_PCI_MASK)
    {
      portsc1 = hpm_getreg((uint32_t)(&s_usb_instance->PORTSC1));

      if (portsc1 & USB_PORTSC1_HSP_MASK)
        priv->usbdev.speed = USB_SPEED_HIGH;
      else
        priv->usbdev.speed = USB_SPEED_FULL;

      if (portsc1 & USB_PORTSC1_FPR_MASK)
        {
          /* FIXME: this occurs because of a J-to-K transition detected
           *         while the port is in SUSPEND state - presumambly this
           *         is where the host is resuming the device?
           *
           *  - but do we need to "ack" the interrupt
           */
        }
    }

#ifdef CONFIG_HPM_USBDEV_FRAME_INTERRUPT
  if (disr & USB_USBSTS_SRI_MASK)
    {
      usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_FRAME), 0);

      uint32_t frindex = hpm_getreg((uint32_t)(&s_usb_instance->FRINDEX));
      uint16_t frame_num =
          (frindex & USB_FRINDEX_FRINDEX_MASK) >> 3u;

      priv->sof = frame_num;
    }
#endif

  if (disr & USB_USBSTS_UEI_MASK)
    {
      /* FIXME: these occur when a transfer results in an error condition
       *        it is set alongside USBINT if the DTD also had its IOC
       *        bit set.
       */
    }

  if (disr & USB_USBSTS_UI_MASK)
    {
      /* Handle completion interrupts */

      uint32_t mask = hpm_getreg((uint32_t)(&s_usb_instance->ENDPTCOMPLETE));

      if (mask)
        {
          /* Clear any NAK interrupt and completion interrupts */

          hpm_putreg(mask, (uint32_t)(&s_usb_instance->ENDPTNAK));
          hpm_putreg(mask, (uint32_t)(&s_usb_instance->ENDPTCOMPLETE));

          if (mask & HPM_ENDPTMASK(0))
            {
              hpm_ep0complete(priv, 0);
            }

          if (mask & HPM_ENDPTMASK(1))
            {
              hpm_ep0complete(priv, 1);
            }

          for (n = 1; n < HPM_NLOGENDPOINTS; n++)
            {
              if (mask & HPM_ENDPTMASK((n << 1)))
                {
                  hpm_epcomplete(priv, (n << 1));
                }

              if (mask & HPM_ENDPTMASK((n << 1)+1))
                {
                  hpm_epcomplete(priv, (n << 1)+1);
                }
            }
        }

      /* Handle setup interrupts */

      uint32_t setupstat = hpm_getreg((uint32_t)(&s_usb_instance->ENDPTSETUPSTAT));
      if (setupstat)
        {
          /* Clear the endpoint complete CTRL OUT and IN when a Setup is
           * received
           */

          hpm_putreg(HPM_ENDPTMASK(HPM_EP0_IN) |
                       HPM_ENDPTMASK(HPM_EP0_OUT),
                       (uint32_t)(&s_usb_instance->ENDPTCOMPLETE));

          if (setupstat & HPM_ENDPTMASK(HPM_EP0_OUT))
            {
              usbtrace(TRACE_INTDECODE(HPM_TRACEINTID_EP0SETUP),
                       setupstat);
              hpm_ep0setup(priv);
            }
        }
    }

  if (disr & USB_USBSTS_NAKI_MASK)
    {
      uint32_t pending = hpm_getreg((uint32_t)(&s_usb_instance->ENDPTNAK)) &
          hpm_getreg((uint32_t)(&s_usb_instance->ENDPTNAKEN));
      if (pending)
        {
          /* We shouldn't see NAK interrupts except on Endpoint 0 */

          if (pending & HPM_ENDPTMASK(0))
            {
              hpm_ep0nak(priv, 0);
            }

          if (pending & HPM_ENDPTMASK(1))
            {
              hpm_ep0nak(priv, 1);
            }
        }

      /* Clear the interrupts */

      hpm_putreg(pending, (uint32_t)(&s_usb_instance->ENDPTNAK));
    }

  usbtrace(TRACE_INTEXIT(HPM_TRACEINTID_USB), 0);
  return OK;
}

/****************************************************************************
 * Endpoint operations
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_epconfigure
 *
 * Description:
 *   Configure endpoint, making it usable
 *
 * Input Parameters:
 *   ep   - the struct usbdev_ep_s instance obtained from allocep()
 *   desc - A struct usb_epdesc_s instance describing the endpoint
 *   last - true if this is the last endpoint to be configured.  Some
 *          hardware needs to take special action when all of the endpoints
 *          have been configured.
 *
 ****************************************************************************/

static int hpm_epconfigure(struct usbdev_ep_s *ep,
                             const struct usb_epdesc_s *desc,
                             bool last)
{
  struct hpm_ep_s *privep = (struct hpm_ep_s *)ep;
  struct hpm_dqh_s *dqh = &g_qh[privep->epphy];

  usbtrace(TRACE_EPCONFIGURE, privep->epphy);
  DEBUGASSERT(desc->addr == ep->eplog);

  /* Initialise EP capabilities */

  uint16_t maxsize = GETUINT16(desc->mxpacketsize);
  if ((desc->attr & USB_EP_ATTR_XFERTYPE_MASK) == USB_EP_ATTR_XFER_ISOC)
    {
      dqh->capability = (DQH_CAPABILITY_MAX_PACKET(maxsize) |
                    DQH_CAPABILITY_IOS |
                    DQH_CAPABILITY_ZLT);
    }
  else
    {
      dqh->capability = (DQH_CAPABILITY_MAX_PACKET(maxsize) |
                    DQH_CAPABILITY_ZLT);
    }

  /* Setup Endpoint Control Register */

  if (HPM_EPPHYIN(privep->epphy))
    {
      /* Reset the data toggles */

      uint32_t cfg = USB_ENDPTCTRL_TXR_MASK;

      /* Set the endpoint type */

      switch (desc->attr & USB_EP_ATTR_XFERTYPE_MASK)
        {
          case USB_EP_ATTR_XFER_CONTROL:
            cfg |= USB_ENDPTCTRL_TXT_SET(usb_xfer_control); break;
          case USB_EP_ATTR_XFER_ISOC:
            cfg |= USB_ENDPTCTRL_TXT_SET(usb_xfer_isochronous); break;
          case USB_EP_ATTR_XFER_BULK:
            cfg |= USB_ENDPTCTRL_TXT_SET(usb_xfer_bulk); break;
          case USB_EP_ATTR_XFER_INT:
            cfg |= USB_ENDPTCTRL_TXT_SET(usb_xfer_interrupt); break;
        }

      hpm_chgbits(0xffff0000, cfg,
                    (uint32_t)(&s_usb_instance->ENDPTCTRL[privep->epphy >> 1]));
    }
  else
    {
      /* Reset the data toggles */

      uint32_t cfg = USB_ENDPTCTRL_RXR_MASK;

      /* Set the endpoint type */

      switch (desc->attr & USB_EP_ATTR_XFERTYPE_MASK)
        {
          case USB_EP_ATTR_XFER_CONTROL:
            cfg |= USB_ENDPTCTRL_TXT_SET(usb_xfer_control); break;
          case USB_EP_ATTR_XFER_ISOC:
            cfg |= USB_ENDPTCTRL_TXT_SET(usb_xfer_isochronous); break;
          case USB_EP_ATTR_XFER_BULK:
            cfg |= USB_ENDPTCTRL_TXT_SET(usb_xfer_bulk); break;
          case USB_EP_ATTR_XFER_INT:
            cfg |= USB_ENDPTCTRL_TXT_SET(usb_xfer_interrupt); break;
        }

      hpm_chgbits(0x0000ffff, cfg,
                    (uint32_t)(&s_usb_instance->ENDPTCTRL[privep->epphy >> 1]));
    }

  /* Reset endpoint status */

  privep->stalled = false;

  /* Enable the endpoint */

  if (HPM_EPPHYIN(privep->epphy))
    {
      hpm_setbits(USB_ENDPTCTRL_TXE_MASK,
                    (uint32_t)(&s_usb_instance->ENDPTCTRL[privep->epphy >> 1]));
    }
  else
    {
      hpm_setbits(USB_ENDPTCTRL_RXE_MASK,
                    (uint32_t)(&s_usb_instance->ENDPTCTRL[privep->epphy >> 1]));
    }

  return OK;
}

/****************************************************************************
 * Name: hpm_epdisable
 *
 * Description:
 *   The endpoint will no longer be used
 *
 ****************************************************************************/

static int hpm_epdisable(struct usbdev_ep_s *ep)
{
  struct hpm_ep_s *privep = (struct hpm_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(HPM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPDISABLE, privep->epphy);

  flags = enter_critical_section();

  /* Disable Endpoint */

  if (HPM_EPPHYIN(privep->epphy))
    {
      hpm_clrbits(USB_ENDPTCTRL_TXE_MASK,
                    (uint32_t)(&s_usb_instance->ENDPTCTRL[privep->epphy >> 1]));
    }
  else
    {
      hpm_clrbits(USB_ENDPTCTRL_RXE_MASK,
                    (uint32_t)(&s_usb_instance->ENDPTCTRL[privep->epphy >> 1]));
    }

  privep->stalled = true;

  /* Cancel any ongoing activity */

  hpm_cancelrequests(privep, -ESHUTDOWN);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: hpm_epallocreq
 *
 * Description:
 *   Allocate an I/O request
 *
 ****************************************************************************/

static struct usbdev_req_s *hpm_epallocreq(struct usbdev_ep_s *ep)
{
  struct hpm_req_s *privreq;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(HPM_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif

  usbtrace(TRACE_EPALLOCREQ, ((struct hpm_ep_s *)ep)->epphy);

  privreq = (struct hpm_req_s *)kmm_malloc(sizeof(struct hpm_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(HPM_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct hpm_req_s));
  return &privreq->req;
}

/****************************************************************************
 * Name: hpm_epfreereq
 *
 * Description:
 *   Free an I/O request
 *
 ****************************************************************************/

static void hpm_epfreereq(struct usbdev_ep_s *ep,
                            struct usbdev_req_s *req)
{
  struct hpm_req_s *privreq = (struct hpm_req_s *)req;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(HPM_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif

  usbtrace(TRACE_EPFREEREQ, ((struct hpm_ep_s *)ep)->epphy);
  kmm_free(privreq);
}

/****************************************************************************
 * Name: hpm_epallocbuffer
 *
 * Description:
 *   Allocate an I/O buffer
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void *hpm_epallocbuffer(struct usbdev_ep_s *ep, uint16_t bytes)
{
  /* The USB peripheral DMA is very forgiving, as the dTD allows the buffer
   * to start at any address. Hence, no need for alignment.
   */

  struct hpm_ep_s *privep = (struct hpm_ep_s *)ep;
  UNUSED(privep);

  usbtrace(TRACE_EPALLOCBUFFER, privep->epphy);
#ifdef CONFIG_USBDEV_DMAMEMORY
  return usbdev_dma_alloc(bytes);
#else
  return cache_aligned_alloc(HPM_L1C_CACHELINE_ALIGN_UP(bytes));
#endif
}
#endif

/****************************************************************************
 * Name: hpm_epfreebuffer
 *
 * Description:
 *   Free an I/O buffer
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void hpm_epfreebuffer(struct usbdev_ep_s *ep, void *buf)
{
  struct hpm_ep_s *privep = (struct hpm_ep_s *)ep;
  UNUSED(privep);

  usbtrace(TRACE_EPFREEBUFFER, privep->epphy);

#ifdef CONFIG_USBDEV_DMAMEMORY
  usbdev_dma_free(buf);
#else
  kmm_free(buf);
#endif
}
#endif

/****************************************************************************
 * Name: hpm_epsubmit
 *
 * Description:
 *   Submit an I/O request to the endpoint
 *
 ****************************************************************************/

static int hpm_epsubmit(struct usbdev_ep_s *ep,
                          struct usbdev_req_s *req)
{
  struct hpm_req_s *privreq = (struct hpm_req_s *)req;
  struct hpm_ep_s *privep = (struct hpm_ep_s *)ep;
  struct hpm_usbdev_s *priv;
  irqstate_t flags;
  int ret = OK;

#ifdef CONFIG_DEBUG_FEATURES
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(HPM_TRACEERR_INVALIDPARMS), 0);
      uinfo("req=%p callback=%p buf=%p ep=%p\n", req,
            req->callback, req->buf, ep);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPSUBMIT, privep->epphy);
  priv = privep->dev;

  if (!priv->driver || priv->usbdev.speed == USB_SPEED_UNKNOWN)
    {
      usbtrace(TRACE_DEVERROR(HPM_TRACEERR_NOTCONFIGURED),
               priv->usbdev.speed);
      return -ESHUTDOWN;
    }

  /* Handle the request from the class driver */

  req->result = -EINPROGRESS;
  req->xfrd   = 0;

  /* Disable Interrupts */

  flags = enter_critical_section();

  /* If we are stalled, then drop all requests on the floor */

  if (privep->stalled)
    {
      ret = -EBUSY;
    }
  else
    {
      /* Add the new request to the request queue for the endpoint */

      if (HPM_EPPHYIN(privep->epphy))
        {
          usbtrace(TRACE_INREQQUEUED(privep->epphy), privreq->req.len);
        }
      else
        {
          usbtrace(TRACE_OUTREQQUEUED(privep->epphy), privreq->req.len);
        }

      if (hpm_rqenqueue(privep, privreq))
        {
          hpm_progressep(privep);
        }
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: hpm_epcancel
 *
 * Description:
 *   Cancel an I/O request previously sent to an endpoint
 *
 ****************************************************************************/

static int hpm_epcancel(struct usbdev_ep_s *ep,
                          struct usbdev_req_s *req)
{
  struct hpm_ep_s *privep = (struct hpm_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(HPM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPCANCEL, privep->epphy);

  flags = enter_critical_section();

  /* FIXME: if the request is the first, then we need to flush the EP
   *         otherwise just remove it from the list
   *
   *  but ... all other implementations cancel all requests ...
   */

  hpm_cancelrequests(privep, -ESHUTDOWN);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: hpm_epstall
 *
 * Description:
 *   Stall or resume and endpoint
 *
 ****************************************************************************/

static int hpm_epstall(struct usbdev_ep_s *ep, bool resume)
{
  struct hpm_ep_s *privep = (struct hpm_ep_s *)ep;
  irqstate_t flags;

  /* STALL or RESUME the endpoint */

  flags = enter_critical_section();
  usbtrace(resume ? TRACE_EPRESUME : TRACE_EPSTALL, privep->epphy);

  uint32_t addr    = (uint32_t)(&s_usb_instance->ENDPTCTRL[privep->epphy >> 1]);
  uint32_t ctrl_xs = HPM_EPPHYIN(privep->epphy) ?
      USB_ENDPTCTRL_TXS_MASK : USB_ENDPTCTRL_RXS_MASK;
  uint32_t ctrl_xr = HPM_EPPHYIN(privep->epphy) ?
      USB_ENDPTCTRL_TXR_MASK : USB_ENDPTCTRL_RXR_MASK;

  if (resume)
    {
      privep->stalled = false;

      /* Clear stall and reset the data toggle */

      hpm_chgbits(ctrl_xs | ctrl_xr, ctrl_xr, addr);
    }
  else
    {
      privep->stalled = true;

      hpm_setbits(ctrl_xs, addr);
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Device operations
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_allocep
 *
 * Description:
 *   Allocate an endpoint matching the parameters.
 *
 * Input Parameters:
 *   eplog  - 7-bit logical endpoint number (direction bit ignored).  Zero
 *            means that any endpoint matching the other requirements will
 *            suffice. The assigned endpoint can be found in the eplog field.
 *   in     - true: IN (device-to-host) endpoint requested
 *   eptype - Endpoint type.  One of {USB_EP_ATTR_XFER_ISOC,
 *            USB_EP_ATTR_XFER_BULK, USB_EP_ATTR_XFER_INT}
 *
 ****************************************************************************/

static struct usbdev_ep_s *hpm_allocep(struct usbdev_s *dev,
                                         uint8_t eplog,
                                         bool in, uint8_t eptype)
{
  struct hpm_usbdev_s *priv = (struct hpm_usbdev_s *)dev;
  uint32_t epset = HPM_EPALLSET & ~HPM_EPCTRLSET;
  irqstate_t flags;
  int epndx = 0;

  usbtrace(TRACE_DEVALLOCEP, (uint16_t)eplog);

  /* Ignore any direction bits in the logical address */

  eplog = USB_EPNO(eplog);

  /* A logical address of 0 means that any endpoint will do */

  if (eplog > 0)
    {
      /* Otherwise, we will return the endpoint structure only for the
       * requested 'logical' endpoint.  All of the other checks will still be
       * performed.
       *
       * First, verify that the logical endpoint is in the range supported by
       * by the hardware.
       */

      if (eplog >= HPM_NLOGENDPOINTS)
        {
          usbtrace(TRACE_DEVERROR(HPM_TRACEERR_BADEPNO), (uint16_t)eplog);
          return NULL;
        }

      /* Convert the logical address to a physical OUT endpoint address and
       * remove all of the candidate endpoints from the bitset except for the
       * the IN/OUT pair for this logical address.
       */

      epset &= 3 << (eplog << 1);
    }

  /* Get the subset matching the requested direction */

  if (in)
    {
      epset &= HPM_EPINSET;
    }
  else
    {
      epset &= HPM_EPOUTSET;
    }

  /* Get the subset matching the requested type */

  switch (eptype)
    {
    case USB_EP_ATTR_XFER_INT: /* Interrupt endpoint */
      epset &= HPM_EPINTRSET;
      break;

    case USB_EP_ATTR_XFER_BULK: /* Bulk endpoint */
      epset &= HPM_EPBULKSET;
      break;

    case USB_EP_ATTR_XFER_ISOC: /* Isochronous endpoint */
      epset &= HPM_EPISOCSET;
      break;

    case USB_EP_ATTR_XFER_CONTROL: /* Control endpoint -- not a valid choice */
    default:
      usbtrace(TRACE_DEVERROR(HPM_TRACEERR_BADEPTYPE), (uint16_t)eptype);
      return NULL;
    }

  /* Is the resulting endpoint supported by the HPM3x? */

  if (epset)
    {
      /* Yes.. now see if any of the request endpoints are available */

      flags = enter_critical_section();
      epset &= priv->epavail;
      if (epset)
        {
          /* Select the lowest bit in the set of matching, available
           * endpoints
           */

          for (epndx = 2; epndx < HPM_NPHYSENDPOINTS; epndx++)
            {
              uint32_t bit = 1 << epndx;
              if ((epset & bit) != 0)
                {
                  /* Mark endpoint no longer available */

                  priv->epavail &= ~bit;
                  leave_critical_section(flags);

                  /* And return the pointer to the standard endpoint
                   * structure
                   */

                  return &priv->eplist[epndx].ep;
                }
            }

          /* Shouldn't get here */
        }

      leave_critical_section(flags);
    }

  usbtrace(TRACE_DEVERROR(HPM_TRACEERR_NOEP), (uint16_t)eplog);
  return NULL;
}

/****************************************************************************
 * Name: hpm_freeep
 *
 * Description:
 *   Free the previously allocated endpoint
 *
 ****************************************************************************/

static void hpm_freeep(struct usbdev_s *dev,
                         struct usbdev_ep_s *ep)
{
  struct hpm_usbdev_s *priv = (struct hpm_usbdev_s *)dev;
  struct hpm_ep_s *privep = (struct hpm_ep_s *)ep;
  irqstate_t flags;

  usbtrace(TRACE_DEVFREEEP, (uint16_t)privep->epphy);

  if (priv && privep)
    {
      /* Mark the endpoint as available */

      flags = enter_critical_section();
      priv->epavail |= (1 << privep->epphy);
      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: hpm_getframe
 *
 * Description:
 *   Returns the current frame number
 *
 ****************************************************************************/

static int hpm_getframe(struct usbdev_s *dev)
{
#ifdef CONFIG_HPM_USBDEV_FRAME_INTERRUPT
  struct hpm_usbdev_s *priv = (struct hpm_usbdev_s *)dev;

  /* Return last valid value of SOF read by the interrupt handler */

  usbtrace(TRACE_DEVGETFRAME, (uint16_t)priv->sof);
  return priv->sof;
#else
  uint32_t frindex = hpm_getreg((uint32_t)(&s_usb_instance->FRINDEX));
  uint16_t frame_num =
      (frindex & USB_FRINDEX_FRINDEX_MASK) >> 3u;

  /* Return the last frame number detected by the hardware */

  usbtrace(TRACE_DEVGETFRAME, frame_num);

  return (int)(frame_num);
#endif
}

/****************************************************************************
 * Name: hpm_wakeup
 *
 * Description:
 *   Tries to wake up the host connected to this device
 *
 ****************************************************************************/

static int hpm_wakeup(struct usbdev_s *dev)
{
  irqstate_t flags;

  usbtrace(TRACE_DEVWAKEUP, 0);

  flags = enter_critical_section();
  hpm_setbits(USB_PORTSC1_FPR_MASK, (uint32_t)(&s_usb_instance->PORTSC1));
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: hpm_selfpowered
 *
 * Description:
 *   Sets/clears the device selfpowered feature
 *
 ****************************************************************************/

static int hpm_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  struct hpm_usbdev_s *priv = (struct hpm_usbdev_s *)dev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(HPM_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  priv->selfpowered = selfpowered;
  return OK;
}

/****************************************************************************
 * Name: hpm_pullup
 *
 * Description:
 *   Software-controlled connect to/disconnect from USB host
 *
 ****************************************************************************/

static int hpm_pullup(struct usbdev_s *dev, bool enable)
{
  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);

  irqstate_t flags = enter_critical_section();
  if (enable)
    {
      hpm_setbits(USB_USBCMD_RS_MASK, (uint32_t)(&s_usb_instance->USBCMD));
    }
  else
    {
      hpm_clrbits(USB_USBCMD_RS_MASK, (uint32_t)(&s_usb_instance->USBCMD));
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_usbdev_initialize
 *
 * Description:
 *   Initialize USB hardware.
 *
 * Assumptions:
 * - This function is called very early in the initialization sequence
 * - PLL  initialization is not performed here but should been in
 *   the low-level boot logic: USB1 PLL must be configured for operation
 *   at 480MHz
 *
 ****************************************************************************/

void hpm_usbdev_initialize(int controller)
{
  struct hpm_usbdev_s *priv = &g_usbdev;
  int i;
  irqstate_t flags;

  assert((controller == 0) || (controller == 1));

  flags = enter_critical_section();

  /* Initialize the device state structure */

  memset(priv, 0, sizeof(struct hpm_usbdev_s));
  priv->usbdev.ops = &g_devops;
  priv->usbdev.ep0 = &priv->eplist[HPM_EP0_IN].ep;
  priv->epavail    = HPM_EPALLSET & ~HPM_EPCTRLSET;

  /* Initialize the endpoint list */

  for (i = 0; i < HPM_NPHYSENDPOINTS; i++)
    {
      uint32_t bit = 1 << i;

      /* Set endpoint operations, reference to driver structure (not
       * really necessary because there is only one controller), and
       * the physical endpoint number (which is just the index to the
       * endpoint).
       */

      priv->eplist[i].ep.ops       = &g_epops;
      priv->eplist[i].dev          = priv;

      /* The index, i, is the physical endpoint address;  Map this
       * to a logical endpoint address usable by the class driver.
       */

      priv->eplist[i].epphy        = i;
      if (HPM_EPPHYIN(i))
        {
          priv->eplist[i].ep.eplog = HPM_EPPHYIN2LOG(i);
        }
      else
        {
          priv->eplist[i].ep.eplog = HPM_EPPHYOUT2LOG(i);
        }

      /* The maximum packet size may depend on the type of endpoint */

      if ((HPM_EPCTRLSET & bit) != 0)
        {
          priv->eplist[i].ep.maxpacket = HPM_EP0MAXPACKET;
        }
      else if ((HPM_EPINTRSET & bit) != 0)
        {
          priv->eplist[i].ep.maxpacket = HPM_INTRMAXPACKET;
        }
      else if ((HPM_EPBULKSET & bit) != 0)
        {
          priv->eplist[i].ep.maxpacket = HPM_BULKMAXPACKET;
        }
      else /* if ((HPM_EPISOCSET & bit) != 0) */
        {
          priv->eplist[i].ep.maxpacket = HPM_ISOCMAXPACKET;
        }
    }

  /* Init usb Pins */

  board_init_usb_pins();

  /* Usb Controller Operational Registers and Init PHY */

  if (controller == 0)
    {
      s_usb_instance = HPM_USB0;
      s_irq_num = HPM_IRQn_USB0;
    }
  else if (controller == 1)
    {
      s_usb_instance = HPM_USB1;
      s_irq_num = HPM_IRQn_USB1;
    }
  else
    {
      ;
    }

  /* Disable USB interrupts */

  hpm_putreg(0, (uint32_t)(&s_usb_instance->USBINTR));

  /* Disconnect device */

  hpm_pullup(&priv->usbdev, false);

  /* Init the controller */
  usb_dcd_init(s_usb_instance);

  /* Attach USB controller interrupt handler */

  irq_attach(s_irq_num, hpm_usbinterrupt, NULL);
  up_enable_irq(s_irq_num);

  leave_critical_section(flags);

  /* Reset/Re-initialize the USB hardware */

  hpm_usbreset(priv);
}

/****************************************************************************
 * Name: hpm_usbdev_uninitialize
 ****************************************************************************/

void hpm_usbdev_uninitialize(void)
{
  struct hpm_usbdev_s *priv = &g_usbdev;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNINIT, 0);

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(HPM_TRACEERR_DRIVERREGISTERED), 0);
      usbdev_unregister(priv->driver);
    }

  flags = enter_critical_section();

  /* Disconnect device */

  hpm_pullup(&priv->usbdev, false);
  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Disable and detach IRQs */

  up_disable_irq(s_irq_num);
  irq_detach(s_irq_num);

  /* Reset the controller */

  usb_dcd_deinit(s_usb_instance);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: usbdev_register
 *
 * Description:
 *   Register a USB device class driver. The class driver's bind() method
 *   will be called to bind it to a USB device driver.
 *
 ****************************************************************************/

int usbdev_register(struct usbdevclass_driver_s *driver)
{
  int ret;

  usbtrace(TRACE_DEVREGISTER, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !driver->ops->bind || !driver->ops->unbind ||
      !driver->ops->disconnect || !driver->ops->setup)
    {
      usbtrace(TRACE_DEVERROR(HPM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (g_usbdev.driver)
    {
      usbtrace(TRACE_DEVERROR(HPM_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* First hook up the driver */

  g_usbdev.driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &g_usbdev.usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(HPM_TRACEERR_BINDFAILED), (uint16_t)-ret);
      g_usbdev.driver = NULL;
    }
  else
    {
      /* Enable USB controller interrupts */

      up_enable_irq(s_irq_num);
    }

  return ret;
}

/****************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Un-register usbdev class driver.If the USB device is connected to a USB
 *   host, it will first disconnect().  The driver is also requested to
 *   unbind() and clean up any device state, before this procedure finally
 *   returns.
 *
 ****************************************************************************/

int usbdev_unregister(struct usbdevclass_driver_s *driver)
{
  usbtrace(TRACE_DEVUNREGISTER, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (driver != g_usbdev.driver)
    {
      usbtrace(TRACE_DEVERROR(HPM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &g_usbdev.usbdev);

  /* Disable USB controller interrupts */

  up_disable_irq(s_irq_num);

  /* Unhook the driver */

  g_usbdev.driver = NULL;
  return OK;
}

#endif
