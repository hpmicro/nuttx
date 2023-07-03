/*
 * Copyright (c) 2021-2023 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */


#ifndef HPM_SEI_H
#define HPM_SEI_H

typedef struct {
    struct {
        struct {
            __RW uint32_t ENG_CTRL;            /* 0x0: Engine control register */
            __RW uint32_t ENG_CFG;             /* 0x4: Pointer configuration register */
            __RW uint32_t ENG_WDG;             /* 0x8: Watch dog configuration register */
            __R  uint8_t  RESERVED0[4];        /* 0xC - 0xF: Reserved */
            __R  uint32_t ENG_EXE;             /* 0x10: Execution status */
            __R  uint32_t ENG_PTR;             /* 0x14: Execution pointer */
            __R  uint32_t ENG_INST;            /* 0x18: Execution instruction */
            __R  uint32_t ENG_STS;             /* 0x1C: Execution status */
        } ENGINE;
        struct {
            __RW uint32_t TRX_CTRL;            /* 0x20: Transceiver control register */
            __RW uint32_t TRX_TYPE_CFG;        /* 0x24: Transceiver configuration register */
            __RW uint32_t TRX_BAUD_CFG;        /* 0x28: Transceiver baud rate register */
            __RW uint32_t TRX_DATA_CFG;        /* 0x2C: Transceiver data timing configuration */
            __RW uint32_t TRX_CLK_CFG;         /* 0x30: Transceiver clock timing configuration */
            __R  uint8_t  RESERVED0[4];        /* 0x34 - 0x37: Reserved */
            __R  uint32_t TRX_PIN;             /* 0x38: Transceiver pin status */
            __R  uint32_t TRX_STATE;           /* 0x3C:  */
        } XCVR;
        struct {
            __RW uint32_t TRG_IN_CFG;          /* 0x40: Trigger input configuration */
            __W  uint32_t TRG_SW;              /* 0x44: Software trigger */
            __RW uint32_t TRG_PRD_CFG;         /* 0x48: Period trigger configuration */
            __RW uint32_t TRG_PRD;             /* 0x4C: Trigger period */
            __RW uint32_t TRG_OUT_CFG;         /* 0x50: Trigger output configuration */
            __R  uint8_t  RESERVED0[12];       /* 0x54 - 0x5F: Reserved */
            __R  uint32_t TRG_PRD_STS;         /* 0x60: Period trigger status */
            __R  uint32_t TRG_PRD_CNT;         /* 0x64: Period trigger counter */
            __R  uint8_t  RESERVED1[24];       /* 0x68 - 0x7F: Reserved */
        } TRIGER;
        struct {
            __RW uint32_t TRG_CMD[4];          /* 0x80 - 0x8C: Trigger command */
            __R  uint8_t  RESERVED0[16];       /* 0x90 - 0x9F: Reserved */
            __R  uint32_t TRG_TIME[4];         /* 0xA0 - 0xAC: Trigger Time */
            __R  uint8_t  RESERVED1[16];       /* 0xB0 - 0xBF: Reserved */
        } TRG_TABLE;
        struct {
            __RW uint32_t CMD_MODE;            /* 0xC0: command register mode */
            __RW uint32_t CMD_IDX;             /* 0xC4: command register configuration */
            __RW uint32_t CMD_GOLD;            /* 0xC8: Command gold value */
            __RW uint32_t CMD_CRCINIT;         /* 0xCC: Command Initial value */
            __RW uint32_t CMD_CRCPOLY;         /* 0xD0: Command CRC polymial */
            __R  uint8_t  RESERVED0[12];       /* 0xD4 - 0xDF: Reserved */
            __RW uint32_t CMD;                 /* 0xE0: command */
            __RW uint32_t CMD_SET;             /* 0xE4: command bit set register */
            __RW uint32_t CMD_CLR;             /* 0xE8: command bit clear register */
            __RW uint32_t CMD_INV;             /* 0xEC: command bit invert register */
            __R  uint32_t CMD_IN;              /* 0xF0: Commad input */
            __R  uint32_t CMD_OUT;             /* 0xF4: Command output */
            __RW uint32_t CMD_STS;             /* 0xF8: Command status */
            __R  uint8_t  RESERVED1[4];        /* 0xFC - 0xFF: Reserved */
        } COMMAND;
        struct {
            __RW uint32_t CMD_MIN;             /* 0x100: command start value */
            __RW uint32_t CMD_MAX;             /* 0x104: command end value */
            __RW uint32_t CMD_MSK;             /* 0x108: command compare bit enable */
            __R  uint8_t  RESERVED0[4];        /* 0x10C - 0x10F: Reserved */
            __RW uint32_t CMD_PTRA;            /* 0x110: command pointer 0 - 3 */
            __RW uint32_t CMD_PTRB;            /* 0x114: command pointer 4 - 7 */
            __RW uint32_t CMD_PTRC;            /* 0x118: command pointer 8 - 11 */
            __RW uint32_t CMD_PTRD;            /* 0x11C: command pointer 12 - 15 */
        } COMMAND_TABLE[8];
        struct {
            __RW uint32_t TRAN_CFG[4];         /* 0x200 - 0x20C: Latch state transition configuration */
            __RW uint32_t LAT_CFG;             /* 0x210: Latch configuration */
            __R  uint8_t  RESERVED0[4];        /* 0x214 - 0x217: Reserved */
            __R  uint32_t LAT_TIME;            /* 0x218: Latch time */
            __R  uint32_t LAT_STS;             /* 0x21C: Latch status */
        } LATCH[4];
        struct {
            __RW uint32_t SMP_EN;              /* 0x280: Sample selection register */
            __RW uint32_t SMP_CFG;             /* 0x284: Sample configuration */
            __RW uint32_t SMP_DAT;             /* 0x288: Sample data */
            __R  uint8_t  RESERVED0[4];        /* 0x28C - 0x28F: Reserved */
            __RW uint32_t SMP_POS;             /* 0x290: Sample override position */
            __RW uint32_t SMP_REV;             /* 0x294: Sample override revolution */
            __RW uint32_t SMP_SPD;             /* 0x298: Sample override speed */
            __RW uint32_t SMP_ACC;             /* 0x29C: Sample override accelerate */
            __RW uint32_t UPD_EN;              /* 0x2A0: Update configuration */
            __RW uint32_t UPD_CFG;             /* 0x2A4: Update configuration */
            __RW uint32_t UPD_DAT;             /* 0x2A8: Update data */
            __RW uint32_t UPD_TIME;            /* 0x2AC: Update overide time */
            __RW uint32_t UPD_POS;             /* 0x2B0: Update override position */
            __RW uint32_t UPD_REV;             /* 0x2B4: Update override revolution */
            __RW uint32_t UPD_SPD;             /* 0x2B8: Update override speed */
            __RW uint32_t UPD_ACC;             /* 0x2BC: Update override accelerate */
            __R  uint32_t SMP_VAL;             /* 0x2C0: Sample valid */
            __R  uint32_t SMP_STS;             /* 0x2C4: Sample status */
            __R  uint8_t  RESERVED1[4];        /* 0x2C8 - 0x2CB: Reserved */
            __R  uint32_t TIME_IN;             /* 0x2CC: input time */
            __R  uint32_t POS_IN;              /* 0x2D0: Input position */
            __R  uint32_t REV_IN;              /* 0x2D4: Input revolution */
            __R  uint32_t SPD_IN;              /* 0x2D8: Input speed */
            __R  uint32_t ACC_IN;              /* 0x2DC: Input accelerate */
            __R  uint8_t  RESERVED2[4];        /* 0x2E0 - 0x2E3: Reserved */
            __R  uint32_t UPD_STS;             /* 0x2E4: Update status */
            __R  uint8_t  RESERVED3[24];       /* 0x2E8 - 0x2FF: Reserved */
        } POSITION;
        struct {
            __RW uint32_t INT_EN;              /* 0x300: Interrupt Enable */
            __W  uint32_t INT_FLAG;            /* 0x304: Interrupt flag */
            __R  uint32_t INT_STS;             /* 0x308: Interrupt status */
            __R  uint8_t  RESERVED0[4];        /* 0x30C - 0x30F: Reserved */
            __RW uint32_t POINTER0;            /* 0x310: Match pointer 0 */
            __RW uint32_t POINTER1;            /* 0x314: Match pointer 1 */
            __RW uint32_t INSTR0;              /* 0x318: Match instruction 0 */
            __RW uint32_t INSTR1;              /* 0x31C: Match instruction 1 */
        } IRQ;
        __R  uint8_t  RESERVED0[224];          /* 0x320 - 0x3FF: Reserved */
    } CTRL[13];
    __RW uint32_t INSTR[256];                  /* 0x3400 - 0x37FC: Instructions */
    struct {
        __RW uint32_t DATA_MODE;               /* 0x3800:  */
        __RW uint32_t DATA_IDX;                /* 0x3804: Data register bit index */
        __RW uint32_t DATA_GOLD;               /* 0x3808: Gold data for data check */
        __RW uint32_t DATA_CRCINIT;            /* 0x380C: CRC calculation initial vector */
        __RW uint32_t DATA_CRCPOLY;            /* 0x3810: CRC calculation polynomial */
        __R  uint8_t  RESERVED0[12];           /* 0x3814 - 0x381F: Reserved */
        __RW uint32_t DATA;                    /* 0x3820: Data value */
        __RW uint32_t DATA_SET;                /* 0x3824: Data bit set */
        __RW uint32_t DATA_CLR;                /* 0x3828: Data bit clear */
        __RW uint32_t DATA_INV;                /* 0x382C: Data bit invert */
        __R  uint32_t DATA_IN;                 /* 0x3830: Data input */
        __R  uint32_t DATA_OUT;                /* 0x3834: Data output */
        __RW uint32_t DATA_STS;                /* 0x3838: Data status */
        __R  uint8_t  RESERVED1[4];            /* 0x383C - 0x383F: Reserved */
    } DAT[32];
} SEI_Type;


/* Bitfield definition for register of struct array CTRL: ENG_CTRL */
/*
 * WATCH (RW)
 *
 * Enable watch dog
 * 0: Watch dog disabled
 * 1: Watch dog enabled
 */
#define SEI_CTRL_ENGINE_ENG_CTRL_WATCH_MASK (0x1000000UL)
#define SEI_CTRL_ENGINE_ENG_CTRL_WATCH_SHIFT (24U)
#define SEI_CTRL_ENGINE_ENG_CTRL_WATCH_SET(x) (((uint32_t)(x) << SEI_CTRL_ENGINE_ENG_CTRL_WATCH_SHIFT) & SEI_CTRL_ENGINE_ENG_CTRL_WATCH_MASK)
#define SEI_CTRL_ENGINE_ENG_CTRL_WATCH_GET(x) (((uint32_t)(x) & SEI_CTRL_ENGINE_ENG_CTRL_WATCH_MASK) >> SEI_CTRL_ENGINE_ENG_CTRL_WATCH_SHIFT)

/*
 * ARMING (RW)
 *
 * Wait for trigger before excuting
 * 0: Execute on enable
 * 1: Wait trigger before exection after enabled
 */
#define SEI_CTRL_ENGINE_ENG_CTRL_ARMING_MASK (0x10000UL)
#define SEI_CTRL_ENGINE_ENG_CTRL_ARMING_SHIFT (16U)
#define SEI_CTRL_ENGINE_ENG_CTRL_ARMING_SET(x) (((uint32_t)(x) << SEI_CTRL_ENGINE_ENG_CTRL_ARMING_SHIFT) & SEI_CTRL_ENGINE_ENG_CTRL_ARMING_MASK)
#define SEI_CTRL_ENGINE_ENG_CTRL_ARMING_GET(x) (((uint32_t)(x) & SEI_CTRL_ENGINE_ENG_CTRL_ARMING_MASK) >> SEI_CTRL_ENGINE_ENG_CTRL_ARMING_SHIFT)

/*
 * EXCEPT (RW)
 *
 * Explain timout as exception
 * 0: when timeout, pointer move to next instruction
 * 1: when timeout, pointer jump to timeout vector
 */
#define SEI_CTRL_ENGINE_ENG_CTRL_EXCEPT_MASK (0x100U)
#define SEI_CTRL_ENGINE_ENG_CTRL_EXCEPT_SHIFT (8U)
#define SEI_CTRL_ENGINE_ENG_CTRL_EXCEPT_SET(x) (((uint32_t)(x) << SEI_CTRL_ENGINE_ENG_CTRL_EXCEPT_SHIFT) & SEI_CTRL_ENGINE_ENG_CTRL_EXCEPT_MASK)
#define SEI_CTRL_ENGINE_ENG_CTRL_EXCEPT_GET(x) (((uint32_t)(x) & SEI_CTRL_ENGINE_ENG_CTRL_EXCEPT_MASK) >> SEI_CTRL_ENGINE_ENG_CTRL_EXCEPT_SHIFT)

/*
 * REWIND (RW)
 *
 * Rewind execution pointer
 * 0: run
 * 1: clean status and rewind
 */
#define SEI_CTRL_ENGINE_ENG_CTRL_REWIND_MASK (0x10U)
#define SEI_CTRL_ENGINE_ENG_CTRL_REWIND_SHIFT (4U)
#define SEI_CTRL_ENGINE_ENG_CTRL_REWIND_SET(x) (((uint32_t)(x) << SEI_CTRL_ENGINE_ENG_CTRL_REWIND_SHIFT) & SEI_CTRL_ENGINE_ENG_CTRL_REWIND_MASK)
#define SEI_CTRL_ENGINE_ENG_CTRL_REWIND_GET(x) (((uint32_t)(x) & SEI_CTRL_ENGINE_ENG_CTRL_REWIND_MASK) >> SEI_CTRL_ENGINE_ENG_CTRL_REWIND_SHIFT)

/*
 * ENABLE (RW)
 *
 * Enable
 * 0: disable
 * 1: enable
 */
#define SEI_CTRL_ENGINE_ENG_CTRL_ENABLE_MASK (0x1U)
#define SEI_CTRL_ENGINE_ENG_CTRL_ENABLE_SHIFT (0U)
#define SEI_CTRL_ENGINE_ENG_CTRL_ENABLE_SET(x) (((uint32_t)(x) << SEI_CTRL_ENGINE_ENG_CTRL_ENABLE_SHIFT) & SEI_CTRL_ENGINE_ENG_CTRL_ENABLE_MASK)
#define SEI_CTRL_ENGINE_ENG_CTRL_ENABLE_GET(x) (((uint32_t)(x) & SEI_CTRL_ENGINE_ENG_CTRL_ENABLE_MASK) >> SEI_CTRL_ENGINE_ENG_CTRL_ENABLE_SHIFT)

/* Bitfield definition for register of struct array CTRL: ENG_CFG */
/*
 * DAT_CDM (RW)
 *
 * Select DATA register to receive CDM bit in BiSSC slave mode
 * 0: ignore
 * 1: command
 * 2: data register 2
 * 3: data register 3
 * ...
 * 29:data register 29
 * 30: value 0 when send, ignore in receive
 * 31: value1 when send, ignore in receive
 */
#define SEI_CTRL_ENGINE_ENG_CFG_DAT_CDM_MASK (0x1F000000UL)
#define SEI_CTRL_ENGINE_ENG_CFG_DAT_CDM_SHIFT (24U)
#define SEI_CTRL_ENGINE_ENG_CFG_DAT_CDM_SET(x) (((uint32_t)(x) << SEI_CTRL_ENGINE_ENG_CFG_DAT_CDM_SHIFT) & SEI_CTRL_ENGINE_ENG_CFG_DAT_CDM_MASK)
#define SEI_CTRL_ENGINE_ENG_CFG_DAT_CDM_GET(x) (((uint32_t)(x) & SEI_CTRL_ENGINE_ENG_CFG_DAT_CDM_MASK) >> SEI_CTRL_ENGINE_ENG_CFG_DAT_CDM_SHIFT)

/*
 * DAT_BASE (RW)
 *
 * Bias for data register access, if calculated index bigger than 32, index will wrap around
 * 0: real data index
 * 1: access index is 1 greater than instruction address
 * 2: access index is 2 greater than instruction address
 * ...
 * 31: access index is 31 greater than instruction address
 */
#define SEI_CTRL_ENGINE_ENG_CFG_DAT_BASE_MASK (0x1F0000UL)
#define SEI_CTRL_ENGINE_ENG_CFG_DAT_BASE_SHIFT (16U)
#define SEI_CTRL_ENGINE_ENG_CFG_DAT_BASE_SET(x) (((uint32_t)(x) << SEI_CTRL_ENGINE_ENG_CFG_DAT_BASE_SHIFT) & SEI_CTRL_ENGINE_ENG_CFG_DAT_BASE_MASK)
#define SEI_CTRL_ENGINE_ENG_CFG_DAT_BASE_GET(x) (((uint32_t)(x) & SEI_CTRL_ENGINE_ENG_CFG_DAT_BASE_MASK) >> SEI_CTRL_ENGINE_ENG_CFG_DAT_BASE_SHIFT)

/*
 * POINTER_WDOG (RW)
 *
 * Pointer to the instruction that the program starts executing after the instruction timeout. The timeout is WDOG_TIME
 */
#define SEI_CTRL_ENGINE_ENG_CFG_POINTER_WDOG_MASK (0xFF00U)
#define SEI_CTRL_ENGINE_ENG_CFG_POINTER_WDOG_SHIFT (8U)
#define SEI_CTRL_ENGINE_ENG_CFG_POINTER_WDOG_SET(x) (((uint32_t)(x) << SEI_CTRL_ENGINE_ENG_CFG_POINTER_WDOG_SHIFT) & SEI_CTRL_ENGINE_ENG_CFG_POINTER_WDOG_MASK)
#define SEI_CTRL_ENGINE_ENG_CFG_POINTER_WDOG_GET(x) (((uint32_t)(x) & SEI_CTRL_ENGINE_ENG_CFG_POINTER_WDOG_MASK) >> SEI_CTRL_ENGINE_ENG_CFG_POINTER_WDOG_SHIFT)

/*
 * POINTER_INIT (RW)
 *
 * Initial execute pointer
 */
#define SEI_CTRL_ENGINE_ENG_CFG_POINTER_INIT_MASK (0xFFU)
#define SEI_CTRL_ENGINE_ENG_CFG_POINTER_INIT_SHIFT (0U)
#define SEI_CTRL_ENGINE_ENG_CFG_POINTER_INIT_SET(x) (((uint32_t)(x) << SEI_CTRL_ENGINE_ENG_CFG_POINTER_INIT_SHIFT) & SEI_CTRL_ENGINE_ENG_CFG_POINTER_INIT_MASK)
#define SEI_CTRL_ENGINE_ENG_CFG_POINTER_INIT_GET(x) (((uint32_t)(x) & SEI_CTRL_ENGINE_ENG_CFG_POINTER_INIT_MASK) >> SEI_CTRL_ENGINE_ENG_CFG_POINTER_INIT_SHIFT)

/* Bitfield definition for register of struct array CTRL: ENG_WDG */
/*
 * WDOG_TIME (RW)
 *
 * Time out count for each instruction, counter in bit time.
 */
#define SEI_CTRL_ENGINE_ENG_WDG_WDOG_TIME_MASK (0xFFFFU)
#define SEI_CTRL_ENGINE_ENG_WDG_WDOG_TIME_SHIFT (0U)
#define SEI_CTRL_ENGINE_ENG_WDG_WDOG_TIME_SET(x) (((uint32_t)(x) << SEI_CTRL_ENGINE_ENG_WDG_WDOG_TIME_SHIFT) & SEI_CTRL_ENGINE_ENG_WDG_WDOG_TIME_MASK)
#define SEI_CTRL_ENGINE_ENG_WDG_WDOG_TIME_GET(x) (((uint32_t)(x) & SEI_CTRL_ENGINE_ENG_WDG_WDOG_TIME_MASK) >> SEI_CTRL_ENGINE_ENG_WDG_WDOG_TIME_SHIFT)

/* Bitfield definition for register of struct array CTRL: ENG_EXE */
/*
 * TRIGERED (RO)
 *
 * Execution has been triggered
 * 0: Execution not triggered
 * 1: Execution triggered
 */
#define SEI_CTRL_ENGINE_ENG_EXE_TRIGERED_MASK (0x100000UL)
#define SEI_CTRL_ENGINE_ENG_EXE_TRIGERED_SHIFT (20U)
#define SEI_CTRL_ENGINE_ENG_EXE_TRIGERED_GET(x) (((uint32_t)(x) & SEI_CTRL_ENGINE_ENG_EXE_TRIGERED_MASK) >> SEI_CTRL_ENGINE_ENG_EXE_TRIGERED_SHIFT)

/*
 * ARMED (RO)
 *
 * Waiting for trigger for execution
 * 0: Not in waiting status
 * 1: In waiting status
 */
#define SEI_CTRL_ENGINE_ENG_EXE_ARMED_MASK (0x10000UL)
#define SEI_CTRL_ENGINE_ENG_EXE_ARMED_SHIFT (16U)
#define SEI_CTRL_ENGINE_ENG_EXE_ARMED_GET(x) (((uint32_t)(x) & SEI_CTRL_ENGINE_ENG_EXE_ARMED_MASK) >> SEI_CTRL_ENGINE_ENG_EXE_ARMED_SHIFT)

/*
 * EXPIRE (RO)
 *
 * Watchdog timer expired
 * 0: Not expired
 * 1: Expired
 */
#define SEI_CTRL_ENGINE_ENG_EXE_EXPIRE_MASK (0x100U)
#define SEI_CTRL_ENGINE_ENG_EXE_EXPIRE_SHIFT (8U)
#define SEI_CTRL_ENGINE_ENG_EXE_EXPIRE_GET(x) (((uint32_t)(x) & SEI_CTRL_ENGINE_ENG_EXE_EXPIRE_MASK) >> SEI_CTRL_ENGINE_ENG_EXE_EXPIRE_SHIFT)

/*
 * STALL (RO)
 *
 * Program finished
 * 0: Program is executing
 * 1: Program finished
 */
#define SEI_CTRL_ENGINE_ENG_EXE_STALL_MASK (0x1U)
#define SEI_CTRL_ENGINE_ENG_EXE_STALL_SHIFT (0U)
#define SEI_CTRL_ENGINE_ENG_EXE_STALL_GET(x) (((uint32_t)(x) & SEI_CTRL_ENGINE_ENG_EXE_STALL_MASK) >> SEI_CTRL_ENGINE_ENG_EXE_STALL_SHIFT)

/* Bitfield definition for register of struct array CTRL: ENG_PTR */
/*
 * HALT_CNT (RO)
 *
 * Halt count in halt instrution
 */
#define SEI_CTRL_ENGINE_ENG_PTR_HALT_CNT_MASK (0x1F000000UL)
#define SEI_CTRL_ENGINE_ENG_PTR_HALT_CNT_SHIFT (24U)
#define SEI_CTRL_ENGINE_ENG_PTR_HALT_CNT_GET(x) (((uint32_t)(x) & SEI_CTRL_ENGINE_ENG_PTR_HALT_CNT_MASK) >> SEI_CTRL_ENGINE_ENG_PTR_HALT_CNT_SHIFT)

/*
 * BIT_CNT (RO)
 *
 * Bit count in send and receive instruction execution
 */
#define SEI_CTRL_ENGINE_ENG_PTR_BIT_CNT_MASK (0x1F0000UL)
#define SEI_CTRL_ENGINE_ENG_PTR_BIT_CNT_SHIFT (16U)
#define SEI_CTRL_ENGINE_ENG_PTR_BIT_CNT_GET(x) (((uint32_t)(x) & SEI_CTRL_ENGINE_ENG_PTR_BIT_CNT_MASK) >> SEI_CTRL_ENGINE_ENG_PTR_BIT_CNT_SHIFT)

/*
 * POINTER (RO)
 *
 * Current program pointer
 */
#define SEI_CTRL_ENGINE_ENG_PTR_POINTER_MASK (0xFFU)
#define SEI_CTRL_ENGINE_ENG_PTR_POINTER_SHIFT (0U)
#define SEI_CTRL_ENGINE_ENG_PTR_POINTER_GET(x) (((uint32_t)(x) & SEI_CTRL_ENGINE_ENG_PTR_POINTER_MASK) >> SEI_CTRL_ENGINE_ENG_PTR_POINTER_SHIFT)

/* Bitfield definition for register of struct array CTRL: ENG_INST */
/*
 * INST (RO)
 *
 * Current instruction
 */
#define SEI_CTRL_ENGINE_ENG_INST_INST_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_ENGINE_ENG_INST_INST_SHIFT (0U)
#define SEI_CTRL_ENGINE_ENG_INST_INST_GET(x) (((uint32_t)(x) & SEI_CTRL_ENGINE_ENG_INST_INST_MASK) >> SEI_CTRL_ENGINE_ENG_INST_INST_SHIFT)

/* Bitfield definition for register of struct array CTRL: ENG_STS */
/*
 * WDOG_CNT (RO)
 *
 * Current watch dog counter value
 */
#define SEI_CTRL_ENGINE_ENG_STS_WDOG_CNT_MASK (0xFFFFU)
#define SEI_CTRL_ENGINE_ENG_STS_WDOG_CNT_SHIFT (0U)
#define SEI_CTRL_ENGINE_ENG_STS_WDOG_CNT_GET(x) (((uint32_t)(x) & SEI_CTRL_ENGINE_ENG_STS_WDOG_CNT_MASK) >> SEI_CTRL_ENGINE_ENG_STS_WDOG_CNT_SHIFT)

/* Bitfield definition for register of struct array CTRL: TRX_CTRL */
/*
 * TRISMP (RW)
 *
 * Tipple sampe
 * 0: sample 1 time for data transition
 * 1: sample 3 times in receive and result in 2oo3
 */
#define SEI_CTRL_XCVR_TRX_CTRL_TRISMP_MASK (0x1000U)
#define SEI_CTRL_XCVR_TRX_CTRL_TRISMP_SHIFT (12U)
#define SEI_CTRL_XCVR_TRX_CTRL_TRISMP_SET(x) (((uint32_t)(x) << SEI_CTRL_XCVR_TRX_CTRL_TRISMP_SHIFT) & SEI_CTRL_XCVR_TRX_CTRL_TRISMP_MASK)
#define SEI_CTRL_XCVR_TRX_CTRL_TRISMP_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_CTRL_TRISMP_MASK) >> SEI_CTRL_XCVR_TRX_CTRL_TRISMP_SHIFT)

/*
 * PAR_CLR (WC)
 *
 * Clear parity error, this is a self clear bit
 * 0: no effect
 * 1: clear parity error
 */
#define SEI_CTRL_XCVR_TRX_CTRL_PAR_CLR_MASK (0x100U)
#define SEI_CTRL_XCVR_TRX_CTRL_PAR_CLR_SHIFT (8U)
#define SEI_CTRL_XCVR_TRX_CTRL_PAR_CLR_SET(x) (((uint32_t)(x) << SEI_CTRL_XCVR_TRX_CTRL_PAR_CLR_SHIFT) & SEI_CTRL_XCVR_TRX_CTRL_PAR_CLR_MASK)
#define SEI_CTRL_XCVR_TRX_CTRL_PAR_CLR_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_CTRL_PAR_CLR_MASK) >> SEI_CTRL_XCVR_TRX_CTRL_PAR_CLR_SHIFT)

/*
 * RESTART (WC)
 *
 * Restart tranceiver, this is a self clear bit
 * 0: no effect
 * 1: reset tranceiver
 */
#define SEI_CTRL_XCVR_TRX_CTRL_RESTART_MASK (0x10U)
#define SEI_CTRL_XCVR_TRX_CTRL_RESTART_SHIFT (4U)
#define SEI_CTRL_XCVR_TRX_CTRL_RESTART_SET(x) (((uint32_t)(x) << SEI_CTRL_XCVR_TRX_CTRL_RESTART_SHIFT) & SEI_CTRL_XCVR_TRX_CTRL_RESTART_MASK)
#define SEI_CTRL_XCVR_TRX_CTRL_RESTART_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_CTRL_RESTART_MASK) >> SEI_CTRL_XCVR_TRX_CTRL_RESTART_SHIFT)

/*
 * MODE (RW)
 *
 * Tranceiver mode
 * 0: synchronous maaster
 * 1: synchronous slave
 * 2: asynchronous mode
 * 3: asynchronous mode
 */
#define SEI_CTRL_XCVR_TRX_CTRL_MODE_MASK (0x3U)
#define SEI_CTRL_XCVR_TRX_CTRL_MODE_SHIFT (0U)
#define SEI_CTRL_XCVR_TRX_CTRL_MODE_SET(x) (((uint32_t)(x) << SEI_CTRL_XCVR_TRX_CTRL_MODE_SHIFT) & SEI_CTRL_XCVR_TRX_CTRL_MODE_MASK)
#define SEI_CTRL_XCVR_TRX_CTRL_MODE_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_CTRL_MODE_MASK) >> SEI_CTRL_XCVR_TRX_CTRL_MODE_SHIFT)

/* Bitfield definition for register of struct array CTRL: TRX_TYPE_CFG */
/*
 * WAIT_LEN (RW)
 *
 * Number of extra stop bit for asynchronous mode
 * 0: 1 bit
 * 1: 2 bit
 * ...
 * 255: 256 bit
 */
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_WAIT_LEN_MASK (0xFF000000UL)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_WAIT_LEN_SHIFT (24U)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_WAIT_LEN_SET(x) (((uint32_t)(x) << SEI_CTRL_XCVR_TRX_TYPE_CFG_WAIT_LEN_SHIFT) & SEI_CTRL_XCVR_TRX_TYPE_CFG_WAIT_LEN_MASK)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_WAIT_LEN_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_TYPE_CFG_WAIT_LEN_MASK) >> SEI_CTRL_XCVR_TRX_TYPE_CFG_WAIT_LEN_SHIFT)

/*
 * DATA_LEN (RW)
 *
 * Number of data bit for asynchronous mode
 * 0: 1 bit
 * 1: 2 bit
 * ...
 * 31: 32 bit
 */
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_DATA_LEN_MASK (0x1F0000UL)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_DATA_LEN_SHIFT (16U)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_DATA_LEN_SET(x) (((uint32_t)(x) << SEI_CTRL_XCVR_TRX_TYPE_CFG_DATA_LEN_SHIFT) & SEI_CTRL_XCVR_TRX_TYPE_CFG_DATA_LEN_MASK)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_DATA_LEN_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_TYPE_CFG_DATA_LEN_MASK) >> SEI_CTRL_XCVR_TRX_TYPE_CFG_DATA_LEN_SHIFT)

/*
 * PAR_POL (RW)
 *
 * Polarity of parity for asynchronous mode
 * 0: even
 * 1: odd
 */
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_PAR_POL_MASK (0x200U)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_PAR_POL_SHIFT (9U)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_PAR_POL_SET(x) (((uint32_t)(x) << SEI_CTRL_XCVR_TRX_TYPE_CFG_PAR_POL_SHIFT) & SEI_CTRL_XCVR_TRX_TYPE_CFG_PAR_POL_MASK)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_PAR_POL_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_TYPE_CFG_PAR_POL_MASK) >> SEI_CTRL_XCVR_TRX_TYPE_CFG_PAR_POL_SHIFT)

/*
 * PAR_EN (RW)
 *
 * enable parity check for asynchronous mode
 * 0: disable
 * 1: enable
 */
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_PAR_EN_MASK (0x100U)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_PAR_EN_SHIFT (8U)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_PAR_EN_SET(x) (((uint32_t)(x) << SEI_CTRL_XCVR_TRX_TYPE_CFG_PAR_EN_SHIFT) & SEI_CTRL_XCVR_TRX_TYPE_CFG_PAR_EN_MASK)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_PAR_EN_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_TYPE_CFG_PAR_EN_MASK) >> SEI_CTRL_XCVR_TRX_TYPE_CFG_PAR_EN_SHIFT)

/*
 * DA_IDLEZ (RW)
 *
 * Idle state driver of data line
 * 0: output
 * 1: high-Z
 */
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEZ_MASK (0x8U)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEZ_SHIFT (3U)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEZ_SET(x) (((uint32_t)(x) << SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEZ_SHIFT) & SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEZ_MASK)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEZ_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEZ_MASK) >> SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEZ_SHIFT)

/*
 * CK_IDLEZ (RW)
 *
 * Idle state driver of clock line
 * 0: output
 * 1: high-Z
 */
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_CK_IDLEZ_MASK (0x4U)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_CK_IDLEZ_SHIFT (2U)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_CK_IDLEZ_SET(x) (((uint32_t)(x) << SEI_CTRL_XCVR_TRX_TYPE_CFG_CK_IDLEZ_SHIFT) & SEI_CTRL_XCVR_TRX_TYPE_CFG_CK_IDLEZ_MASK)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_CK_IDLEZ_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_TYPE_CFG_CK_IDLEZ_MASK) >> SEI_CTRL_XCVR_TRX_TYPE_CFG_CK_IDLEZ_SHIFT)

/*
 * DA_IDLEV (RW)
 *
 * Idle state value of data line
 * 0: data'0'
 * 1: data'1'
 */
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEV_MASK (0x2U)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEV_SHIFT (1U)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEV_SET(x) (((uint32_t)(x) << SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEV_SHIFT) & SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEV_MASK)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEV_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEV_MASK) >> SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEV_SHIFT)

/*
 * CK_IDLEV (RW)
 *
 * Idle state value of clock line
 * 0: data'0'
 * 1: data'1'
 */
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_CK_IDLEV_MASK (0x1U)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_CK_IDLEV_SHIFT (0U)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_CK_IDLEV_SET(x) (((uint32_t)(x) << SEI_CTRL_XCVR_TRX_TYPE_CFG_CK_IDLEV_SHIFT) & SEI_CTRL_XCVR_TRX_TYPE_CFG_CK_IDLEV_MASK)
#define SEI_CTRL_XCVR_TRX_TYPE_CFG_CK_IDLEV_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_TYPE_CFG_CK_IDLEV_MASK) >> SEI_CTRL_XCVR_TRX_TYPE_CFG_CK_IDLEV_SHIFT)

/* Bitfield definition for register of struct array CTRL: TRX_BAUD_CFG */
/*
 * SYNC_POINT (RW)
 *
 * Baud synchronous time, minmum bit time
 */
#define SEI_CTRL_XCVR_TRX_BAUD_CFG_SYNC_POINT_MASK (0xFFFF0000UL)
#define SEI_CTRL_XCVR_TRX_BAUD_CFG_SYNC_POINT_SHIFT (16U)
#define SEI_CTRL_XCVR_TRX_BAUD_CFG_SYNC_POINT_SET(x) (((uint32_t)(x) << SEI_CTRL_XCVR_TRX_BAUD_CFG_SYNC_POINT_SHIFT) & SEI_CTRL_XCVR_TRX_BAUD_CFG_SYNC_POINT_MASK)
#define SEI_CTRL_XCVR_TRX_BAUD_CFG_SYNC_POINT_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_BAUD_CFG_SYNC_POINT_MASK) >> SEI_CTRL_XCVR_TRX_BAUD_CFG_SYNC_POINT_SHIFT)

/*
 * BAUD_DIV (RW)
 *
 * Baud rate, bit time in system clock cycle
 */
#define SEI_CTRL_XCVR_TRX_BAUD_CFG_BAUD_DIV_MASK (0xFFFFU)
#define SEI_CTRL_XCVR_TRX_BAUD_CFG_BAUD_DIV_SHIFT (0U)
#define SEI_CTRL_XCVR_TRX_BAUD_CFG_BAUD_DIV_SET(x) (((uint32_t)(x) << SEI_CTRL_XCVR_TRX_BAUD_CFG_BAUD_DIV_SHIFT) & SEI_CTRL_XCVR_TRX_BAUD_CFG_BAUD_DIV_MASK)
#define SEI_CTRL_XCVR_TRX_BAUD_CFG_BAUD_DIV_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_BAUD_CFG_BAUD_DIV_MASK) >> SEI_CTRL_XCVR_TRX_BAUD_CFG_BAUD_DIV_SHIFT)

/* Bitfield definition for register of struct array CTRL: TRX_DATA_CFG */
/*
 * TXD_POINT (RW)
 *
 * data transmit point  in system clcok cycle
 */
#define SEI_CTRL_XCVR_TRX_DATA_CFG_TXD_POINT_MASK (0xFFFF0000UL)
#define SEI_CTRL_XCVR_TRX_DATA_CFG_TXD_POINT_SHIFT (16U)
#define SEI_CTRL_XCVR_TRX_DATA_CFG_TXD_POINT_SET(x) (((uint32_t)(x) << SEI_CTRL_XCVR_TRX_DATA_CFG_TXD_POINT_SHIFT) & SEI_CTRL_XCVR_TRX_DATA_CFG_TXD_POINT_MASK)
#define SEI_CTRL_XCVR_TRX_DATA_CFG_TXD_POINT_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_DATA_CFG_TXD_POINT_MASK) >> SEI_CTRL_XCVR_TRX_DATA_CFG_TXD_POINT_SHIFT)

/*
 * RXD_POINT (RW)
 *
 * data receive point in system clcok cycle
 */
#define SEI_CTRL_XCVR_TRX_DATA_CFG_RXD_POINT_MASK (0xFFFFU)
#define SEI_CTRL_XCVR_TRX_DATA_CFG_RXD_POINT_SHIFT (0U)
#define SEI_CTRL_XCVR_TRX_DATA_CFG_RXD_POINT_SET(x) (((uint32_t)(x) << SEI_CTRL_XCVR_TRX_DATA_CFG_RXD_POINT_SHIFT) & SEI_CTRL_XCVR_TRX_DATA_CFG_RXD_POINT_MASK)
#define SEI_CTRL_XCVR_TRX_DATA_CFG_RXD_POINT_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_DATA_CFG_RXD_POINT_MASK) >> SEI_CTRL_XCVR_TRX_DATA_CFG_RXD_POINT_SHIFT)

/* Bitfield definition for register of struct array CTRL: TRX_CLK_CFG */
/*
 * CK1_POINT (RW)
 *
 * clock point 1 in system clcok cycle
 */
#define SEI_CTRL_XCVR_TRX_CLK_CFG_CK1_POINT_MASK (0xFFFF0000UL)
#define SEI_CTRL_XCVR_TRX_CLK_CFG_CK1_POINT_SHIFT (16U)
#define SEI_CTRL_XCVR_TRX_CLK_CFG_CK1_POINT_SET(x) (((uint32_t)(x) << SEI_CTRL_XCVR_TRX_CLK_CFG_CK1_POINT_SHIFT) & SEI_CTRL_XCVR_TRX_CLK_CFG_CK1_POINT_MASK)
#define SEI_CTRL_XCVR_TRX_CLK_CFG_CK1_POINT_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_CLK_CFG_CK1_POINT_MASK) >> SEI_CTRL_XCVR_TRX_CLK_CFG_CK1_POINT_SHIFT)

/*
 * CK0_POINT (RW)
 *
 * clock point 0 in system clcok cycle
 */
#define SEI_CTRL_XCVR_TRX_CLK_CFG_CK0_POINT_MASK (0xFFFFU)
#define SEI_CTRL_XCVR_TRX_CLK_CFG_CK0_POINT_SHIFT (0U)
#define SEI_CTRL_XCVR_TRX_CLK_CFG_CK0_POINT_SET(x) (((uint32_t)(x) << SEI_CTRL_XCVR_TRX_CLK_CFG_CK0_POINT_SHIFT) & SEI_CTRL_XCVR_TRX_CLK_CFG_CK0_POINT_MASK)
#define SEI_CTRL_XCVR_TRX_CLK_CFG_CK0_POINT_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_CLK_CFG_CK0_POINT_MASK) >> SEI_CTRL_XCVR_TRX_CLK_CFG_CK0_POINT_SHIFT)

/* Bitfield definition for register of struct array CTRL: TRX_PIN */
/*
 * OE_CK (RO)
 *
 * CK drive state
 * 0: input
 * 1: output
 */
#define SEI_CTRL_XCVR_TRX_PIN_OE_CK_MASK (0x4000000UL)
#define SEI_CTRL_XCVR_TRX_PIN_OE_CK_SHIFT (26U)
#define SEI_CTRL_XCVR_TRX_PIN_OE_CK_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_PIN_OE_CK_MASK) >> SEI_CTRL_XCVR_TRX_PIN_OE_CK_SHIFT)

/*
 * DI_CK (RO)
 *
 * CK state
 * 0: data 0
 * 1: data 1
 */
#define SEI_CTRL_XCVR_TRX_PIN_DI_CK_MASK (0x2000000UL)
#define SEI_CTRL_XCVR_TRX_PIN_DI_CK_SHIFT (25U)
#define SEI_CTRL_XCVR_TRX_PIN_DI_CK_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_PIN_DI_CK_MASK) >> SEI_CTRL_XCVR_TRX_PIN_DI_CK_SHIFT)

/*
 * DO_CK (RO)
 *
 * CK output
 * 0: data 0
 * 1: data 1
 */
#define SEI_CTRL_XCVR_TRX_PIN_DO_CK_MASK (0x1000000UL)
#define SEI_CTRL_XCVR_TRX_PIN_DO_CK_SHIFT (24U)
#define SEI_CTRL_XCVR_TRX_PIN_DO_CK_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_PIN_DO_CK_MASK) >> SEI_CTRL_XCVR_TRX_PIN_DO_CK_SHIFT)

/*
 * OE_RX (RO)
 *
 * RX drive state
 * 0: input
 * 1: output
 */
#define SEI_CTRL_XCVR_TRX_PIN_OE_RX_MASK (0x40000UL)
#define SEI_CTRL_XCVR_TRX_PIN_OE_RX_SHIFT (18U)
#define SEI_CTRL_XCVR_TRX_PIN_OE_RX_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_PIN_OE_RX_MASK) >> SEI_CTRL_XCVR_TRX_PIN_OE_RX_SHIFT)

/*
 * DI_RX (RO)
 *
 * RX state
 * 0: data 0
 * 1: data 1
 */
#define SEI_CTRL_XCVR_TRX_PIN_DI_RX_MASK (0x20000UL)
#define SEI_CTRL_XCVR_TRX_PIN_DI_RX_SHIFT (17U)
#define SEI_CTRL_XCVR_TRX_PIN_DI_RX_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_PIN_DI_RX_MASK) >> SEI_CTRL_XCVR_TRX_PIN_DI_RX_SHIFT)

/*
 * DO_RX (RO)
 *
 * RX output
 * 0: data 0
 * 1: data 1
 */
#define SEI_CTRL_XCVR_TRX_PIN_DO_RX_MASK (0x10000UL)
#define SEI_CTRL_XCVR_TRX_PIN_DO_RX_SHIFT (16U)
#define SEI_CTRL_XCVR_TRX_PIN_DO_RX_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_PIN_DO_RX_MASK) >> SEI_CTRL_XCVR_TRX_PIN_DO_RX_SHIFT)

/*
 * OE_DE (RO)
 *
 * DE drive state
 * 0: input
 * 1: output
 */
#define SEI_CTRL_XCVR_TRX_PIN_OE_DE_MASK (0x400U)
#define SEI_CTRL_XCVR_TRX_PIN_OE_DE_SHIFT (10U)
#define SEI_CTRL_XCVR_TRX_PIN_OE_DE_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_PIN_OE_DE_MASK) >> SEI_CTRL_XCVR_TRX_PIN_OE_DE_SHIFT)

/*
 * DI_DE (RO)
 *
 * DE state
 * 0: data 0
 * 1: data 1
 */
#define SEI_CTRL_XCVR_TRX_PIN_DI_DE_MASK (0x200U)
#define SEI_CTRL_XCVR_TRX_PIN_DI_DE_SHIFT (9U)
#define SEI_CTRL_XCVR_TRX_PIN_DI_DE_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_PIN_DI_DE_MASK) >> SEI_CTRL_XCVR_TRX_PIN_DI_DE_SHIFT)

/*
 * DO_DE (RO)
 *
 * DE output
 * 0: data 0
 * 1: data 1
 */
#define SEI_CTRL_XCVR_TRX_PIN_DO_DE_MASK (0x100U)
#define SEI_CTRL_XCVR_TRX_PIN_DO_DE_SHIFT (8U)
#define SEI_CTRL_XCVR_TRX_PIN_DO_DE_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_PIN_DO_DE_MASK) >> SEI_CTRL_XCVR_TRX_PIN_DO_DE_SHIFT)

/*
 * OE_TX (RO)
 *
 * TX drive state
 * 0: input
 * 1: output
 */
#define SEI_CTRL_XCVR_TRX_PIN_OE_TX_MASK (0x4U)
#define SEI_CTRL_XCVR_TRX_PIN_OE_TX_SHIFT (2U)
#define SEI_CTRL_XCVR_TRX_PIN_OE_TX_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_PIN_OE_TX_MASK) >> SEI_CTRL_XCVR_TRX_PIN_OE_TX_SHIFT)

/*
 * DI_TX (RO)
 *
 * TX state
 * 0: data 0
 * 1: data 1
 */
#define SEI_CTRL_XCVR_TRX_PIN_DI_TX_MASK (0x2U)
#define SEI_CTRL_XCVR_TRX_PIN_DI_TX_SHIFT (1U)
#define SEI_CTRL_XCVR_TRX_PIN_DI_TX_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_PIN_DI_TX_MASK) >> SEI_CTRL_XCVR_TRX_PIN_DI_TX_SHIFT)

/*
 * DO_TX (RO)
 *
 * TX output
 * 0: data 0
 * 1: data 1
 */
#define SEI_CTRL_XCVR_TRX_PIN_DO_TX_MASK (0x1U)
#define SEI_CTRL_XCVR_TRX_PIN_DO_TX_SHIFT (0U)
#define SEI_CTRL_XCVR_TRX_PIN_DO_TX_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_PIN_DO_TX_MASK) >> SEI_CTRL_XCVR_TRX_PIN_DO_TX_SHIFT)

/* Bitfield definition for register of struct array CTRL: TRX_STATE */
/*
 * RECV_STATE (RO)
 *
 * FSM of asynchronous receive
 */
#define SEI_CTRL_XCVR_TRX_STATE_RECV_STATE_MASK (0x7000000UL)
#define SEI_CTRL_XCVR_TRX_STATE_RECV_STATE_SHIFT (24U)
#define SEI_CTRL_XCVR_TRX_STATE_RECV_STATE_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_STATE_RECV_STATE_MASK) >> SEI_CTRL_XCVR_TRX_STATE_RECV_STATE_SHIFT)

/*
 * SEND_STATE (RO)
 *
 * FSM of asynchronous transmit
 */
#define SEI_CTRL_XCVR_TRX_STATE_SEND_STATE_MASK (0x70000UL)
#define SEI_CTRL_XCVR_TRX_STATE_SEND_STATE_SHIFT (16U)
#define SEI_CTRL_XCVR_TRX_STATE_SEND_STATE_GET(x) (((uint32_t)(x) & SEI_CTRL_XCVR_TRX_STATE_SEND_STATE_MASK) >> SEI_CTRL_XCVR_TRX_STATE_SEND_STATE_SHIFT)

/* Bitfield definition for register of struct array CTRL: TRG_IN_CFG */
/*
 * PRD_EN (RW)
 *
 * Enable period trigger (tigger 2)
 * 0: periodical trigger disabled
 * 1: periodical trigger enabled
 */
#define SEI_CTRL_TRIGER_TRG_IN_CFG_PRD_EN_MASK (0x800000UL)
#define SEI_CTRL_TRIGER_TRG_IN_CFG_PRD_EN_SHIFT (23U)
#define SEI_CTRL_TRIGER_TRG_IN_CFG_PRD_EN_SET(x) (((uint32_t)(x) << SEI_CTRL_TRIGER_TRG_IN_CFG_PRD_EN_SHIFT) & SEI_CTRL_TRIGER_TRG_IN_CFG_PRD_EN_MASK)
#define SEI_CTRL_TRIGER_TRG_IN_CFG_PRD_EN_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_IN_CFG_PRD_EN_MASK) >> SEI_CTRL_TRIGER_TRG_IN_CFG_PRD_EN_SHIFT)

/*
 * SYNC_SEL (RW)
 *
 * Synchronize sigal selection (tigger 2)
 * 0: trigger in 0
 * 1: trigger in 1
 * ...
 * 7: trigger in 7
 */
#define SEI_CTRL_TRIGER_TRG_IN_CFG_SYNC_SEL_MASK (0x70000UL)
#define SEI_CTRL_TRIGER_TRG_IN_CFG_SYNC_SEL_SHIFT (16U)
#define SEI_CTRL_TRIGER_TRG_IN_CFG_SYNC_SEL_SET(x) (((uint32_t)(x) << SEI_CTRL_TRIGER_TRG_IN_CFG_SYNC_SEL_SHIFT) & SEI_CTRL_TRIGER_TRG_IN_CFG_SYNC_SEL_MASK)
#define SEI_CTRL_TRIGER_TRG_IN_CFG_SYNC_SEL_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_IN_CFG_SYNC_SEL_MASK) >> SEI_CTRL_TRIGER_TRG_IN_CFG_SYNC_SEL_SHIFT)

/*
 * IN1_EN (RW)
 *
 * Enable trigger 1
 * 0: disable trigger 1
 * 1: enable trigger 1
 */
#define SEI_CTRL_TRIGER_TRG_IN_CFG_IN1_EN_MASK (0x8000U)
#define SEI_CTRL_TRIGER_TRG_IN_CFG_IN1_EN_SHIFT (15U)
#define SEI_CTRL_TRIGER_TRG_IN_CFG_IN1_EN_SET(x) (((uint32_t)(x) << SEI_CTRL_TRIGER_TRG_IN_CFG_IN1_EN_SHIFT) & SEI_CTRL_TRIGER_TRG_IN_CFG_IN1_EN_MASK)
#define SEI_CTRL_TRIGER_TRG_IN_CFG_IN1_EN_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_IN_CFG_IN1_EN_MASK) >> SEI_CTRL_TRIGER_TRG_IN_CFG_IN1_EN_SHIFT)

/*
 * IN1_SEL (RW)
 *
 * Trigger 1 sigal selection
 * 0: trigger in 0
 * 1: trigger in 1
 * ...
 * 7: trigger in 7
 */
#define SEI_CTRL_TRIGER_TRG_IN_CFG_IN1_SEL_MASK (0x700U)
#define SEI_CTRL_TRIGER_TRG_IN_CFG_IN1_SEL_SHIFT (8U)
#define SEI_CTRL_TRIGER_TRG_IN_CFG_IN1_SEL_SET(x) (((uint32_t)(x) << SEI_CTRL_TRIGER_TRG_IN_CFG_IN1_SEL_SHIFT) & SEI_CTRL_TRIGER_TRG_IN_CFG_IN1_SEL_MASK)
#define SEI_CTRL_TRIGER_TRG_IN_CFG_IN1_SEL_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_IN_CFG_IN1_SEL_MASK) >> SEI_CTRL_TRIGER_TRG_IN_CFG_IN1_SEL_SHIFT)

/*
 * IN0_EN (RW)
 *
 * Enable trigger 0
 * 0: disable trigger 1
 * 1: enable trigger 1
 */
#define SEI_CTRL_TRIGER_TRG_IN_CFG_IN0_EN_MASK (0x80U)
#define SEI_CTRL_TRIGER_TRG_IN_CFG_IN0_EN_SHIFT (7U)
#define SEI_CTRL_TRIGER_TRG_IN_CFG_IN0_EN_SET(x) (((uint32_t)(x) << SEI_CTRL_TRIGER_TRG_IN_CFG_IN0_EN_SHIFT) & SEI_CTRL_TRIGER_TRG_IN_CFG_IN0_EN_MASK)
#define SEI_CTRL_TRIGER_TRG_IN_CFG_IN0_EN_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_IN_CFG_IN0_EN_MASK) >> SEI_CTRL_TRIGER_TRG_IN_CFG_IN0_EN_SHIFT)

/*
 * IN0_SEL (RW)
 *
 * Trigger 0 sigal selection
 * 0: trigger in 0
 * 1: trigger in 1
 * ...
 * 7: trigger in 7
 */
#define SEI_CTRL_TRIGER_TRG_IN_CFG_IN0_SEL_MASK (0x7U)
#define SEI_CTRL_TRIGER_TRG_IN_CFG_IN0_SEL_SHIFT (0U)
#define SEI_CTRL_TRIGER_TRG_IN_CFG_IN0_SEL_SET(x) (((uint32_t)(x) << SEI_CTRL_TRIGER_TRG_IN_CFG_IN0_SEL_SHIFT) & SEI_CTRL_TRIGER_TRG_IN_CFG_IN0_SEL_MASK)
#define SEI_CTRL_TRIGER_TRG_IN_CFG_IN0_SEL_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_IN_CFG_IN0_SEL_MASK) >> SEI_CTRL_TRIGER_TRG_IN_CFG_IN0_SEL_SHIFT)

/* Bitfield definition for register of struct array CTRL: TRG_SW */
/*
 * SOFT (WC)
 *
 * Software trigger (tigger 3). this bit is self-clear
 * 0: trigger source disabled
 * 1: trigger source enabled
 */
#define SEI_CTRL_TRIGER_TRG_SW_SOFT_MASK (0x1U)
#define SEI_CTRL_TRIGER_TRG_SW_SOFT_SHIFT (0U)
#define SEI_CTRL_TRIGER_TRG_SW_SOFT_SET(x) (((uint32_t)(x) << SEI_CTRL_TRIGER_TRG_SW_SOFT_SHIFT) & SEI_CTRL_TRIGER_TRG_SW_SOFT_MASK)
#define SEI_CTRL_TRIGER_TRG_SW_SOFT_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_SW_SOFT_MASK) >> SEI_CTRL_TRIGER_TRG_SW_SOFT_SHIFT)

/* Bitfield definition for register of struct array CTRL: TRG_PRD_CFG */
/*
 * ARMING (RW)
 *
 * Wait for trigger synchronous before trigger
 * 0: Trigger directly
 * 1: Wait trigger source before period trigger
 */
#define SEI_CTRL_TRIGER_TRG_PRD_CFG_ARMING_MASK (0x10000UL)
#define SEI_CTRL_TRIGER_TRG_PRD_CFG_ARMING_SHIFT (16U)
#define SEI_CTRL_TRIGER_TRG_PRD_CFG_ARMING_SET(x) (((uint32_t)(x) << SEI_CTRL_TRIGER_TRG_PRD_CFG_ARMING_SHIFT) & SEI_CTRL_TRIGER_TRG_PRD_CFG_ARMING_MASK)
#define SEI_CTRL_TRIGER_TRG_PRD_CFG_ARMING_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_PRD_CFG_ARMING_MASK) >> SEI_CTRL_TRIGER_TRG_PRD_CFG_ARMING_SHIFT)

/*
 * SYNC (RW)
 *
 * Synchronous
 * 0: Not synchronous
 * 1: Synchronous every trigger source
 */
#define SEI_CTRL_TRIGER_TRG_PRD_CFG_SYNC_MASK (0x1U)
#define SEI_CTRL_TRIGER_TRG_PRD_CFG_SYNC_SHIFT (0U)
#define SEI_CTRL_TRIGER_TRG_PRD_CFG_SYNC_SET(x) (((uint32_t)(x) << SEI_CTRL_TRIGER_TRG_PRD_CFG_SYNC_SHIFT) & SEI_CTRL_TRIGER_TRG_PRD_CFG_SYNC_MASK)
#define SEI_CTRL_TRIGER_TRG_PRD_CFG_SYNC_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_PRD_CFG_SYNC_MASK) >> SEI_CTRL_TRIGER_TRG_PRD_CFG_SYNC_SHIFT)

/* Bitfield definition for register of struct array CTRL: TRG_PRD */
/*
 * PERIOD (RW)
 *
 * Trigger period
 */
#define SEI_CTRL_TRIGER_TRG_PRD_PERIOD_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_TRIGER_TRG_PRD_PERIOD_SHIFT (0U)
#define SEI_CTRL_TRIGER_TRG_PRD_PERIOD_SET(x) (((uint32_t)(x) << SEI_CTRL_TRIGER_TRG_PRD_PERIOD_SHIFT) & SEI_CTRL_TRIGER_TRG_PRD_PERIOD_MASK)
#define SEI_CTRL_TRIGER_TRG_PRD_PERIOD_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_PRD_PERIOD_MASK) >> SEI_CTRL_TRIGER_TRG_PRD_PERIOD_SHIFT)

/* Bitfield definition for register of struct array CTRL: TRG_OUT_CFG */
/*
 * OUT3_EN (RW)
 *
 * Enable trigger 3
 * 0: disable trigger 3
 * 1: enable trigger 3
 */
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT3_EN_MASK (0x80000000UL)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT3_EN_SHIFT (31U)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT3_EN_SET(x) (((uint32_t)(x) << SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT3_EN_SHIFT) & SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT3_EN_MASK)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT3_EN_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT3_EN_MASK) >> SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT3_EN_SHIFT)

/*
 * OUT3_SEL (RW)
 *
 * Trigger 3 sigal selection
 * 0: trigger out 0
 * 1: trigger out 1
 * ...
 * 7: trigger out 7
 */
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT3_SEL_MASK (0x7000000UL)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT3_SEL_SHIFT (24U)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT3_SEL_SET(x) (((uint32_t)(x) << SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT3_SEL_SHIFT) & SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT3_SEL_MASK)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT3_SEL_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT3_SEL_MASK) >> SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT3_SEL_SHIFT)

/*
 * OUT2_EN (RW)
 *
 * Enable trigger 2
 * 0: disable trigger 2
 * 1: enable trigger 2
 */
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT2_EN_MASK (0x800000UL)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT2_EN_SHIFT (23U)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT2_EN_SET(x) (((uint32_t)(x) << SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT2_EN_SHIFT) & SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT2_EN_MASK)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT2_EN_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT2_EN_MASK) >> SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT2_EN_SHIFT)

/*
 * OUT2_SEL (RW)
 *
 * Trigger 2 sigal selection
 * 0: trigger out 0
 * 1: trigger out 1
 * ...
 * 7: trigger out 7
 */
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT2_SEL_MASK (0x70000UL)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT2_SEL_SHIFT (16U)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT2_SEL_SET(x) (((uint32_t)(x) << SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT2_SEL_SHIFT) & SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT2_SEL_MASK)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT2_SEL_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT2_SEL_MASK) >> SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT2_SEL_SHIFT)

/*
 * OUT1_EN (RW)
 *
 * Enable trigger 1
 * 0: disable trigger 1
 * 1: enable trigger 1
 */
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT1_EN_MASK (0x8000U)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT1_EN_SHIFT (15U)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT1_EN_SET(x) (((uint32_t)(x) << SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT1_EN_SHIFT) & SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT1_EN_MASK)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT1_EN_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT1_EN_MASK) >> SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT1_EN_SHIFT)

/*
 * OUT1_SEL (RW)
 *
 * Trigger 1 sigal selection
 * 0: trigger out 0
 * 1: trigger out 1
 * ...
 * 7: trigger out 7
 */
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT1_SEL_MASK (0x700U)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT1_SEL_SHIFT (8U)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT1_SEL_SET(x) (((uint32_t)(x) << SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT1_SEL_SHIFT) & SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT1_SEL_MASK)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT1_SEL_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT1_SEL_MASK) >> SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT1_SEL_SHIFT)

/*
 * OUT0_EN (RW)
 *
 * Enable trigger 0
 * 0: disable trigger 1
 * 1: enable trigger 1
 */
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT0_EN_MASK (0x80U)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT0_EN_SHIFT (7U)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT0_EN_SET(x) (((uint32_t)(x) << SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT0_EN_SHIFT) & SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT0_EN_MASK)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT0_EN_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT0_EN_MASK) >> SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT0_EN_SHIFT)

/*
 * OUT0_SEL (RW)
 *
 * Trigger 0 sigal selection
 * 0: trigger out 0
 * 1: trigger out 1
 * ...
 * 7: trigger out 7
 */
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT0_SEL_MASK (0x7U)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT0_SEL_SHIFT (0U)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT0_SEL_SET(x) (((uint32_t)(x) << SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT0_SEL_SHIFT) & SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT0_SEL_MASK)
#define SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT0_SEL_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT0_SEL_MASK) >> SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT0_SEL_SHIFT)

/* Bitfield definition for register of struct array CTRL: TRG_PRD_STS */
/*
 * TRIGERED (RO)
 *
 * Period has been triggered
 * 0: Not triggered
 * 1: Triggered
 */
#define SEI_CTRL_TRIGER_TRG_PRD_STS_TRIGERED_MASK (0x100000UL)
#define SEI_CTRL_TRIGER_TRG_PRD_STS_TRIGERED_SHIFT (20U)
#define SEI_CTRL_TRIGER_TRG_PRD_STS_TRIGERED_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_PRD_STS_TRIGERED_MASK) >> SEI_CTRL_TRIGER_TRG_PRD_STS_TRIGERED_SHIFT)

/*
 * ARMED (RO)
 *
 * Waiting for trigger
 * 0: Not in waiting status
 * 1: In waiting status
 */
#define SEI_CTRL_TRIGER_TRG_PRD_STS_ARMED_MASK (0x10000UL)
#define SEI_CTRL_TRIGER_TRG_PRD_STS_ARMED_SHIFT (16U)
#define SEI_CTRL_TRIGER_TRG_PRD_STS_ARMED_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_PRD_STS_ARMED_MASK) >> SEI_CTRL_TRIGER_TRG_PRD_STS_ARMED_SHIFT)

/* Bitfield definition for register of struct array CTRL: TRG_PRD_CNT */
/*
 * PERIOD_CNT (RO)
 *
 * Trigger period counter
 */
#define SEI_CTRL_TRIGER_TRG_PRD_CNT_PERIOD_CNT_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_TRIGER_TRG_PRD_CNT_PERIOD_CNT_SHIFT (0U)
#define SEI_CTRL_TRIGER_TRG_PRD_CNT_PERIOD_CNT_GET(x) (((uint32_t)(x) & SEI_CTRL_TRIGER_TRG_PRD_CNT_PERIOD_CNT_MASK) >> SEI_CTRL_TRIGER_TRG_PRD_CNT_PERIOD_CNT_SHIFT)

/* Bitfield definition for register of struct array CTRL: COMMAND_TRIGGER0 */
/*
 * CMD_TRIGGER0 (RW)
 *
 * Trigger command
 */
#define SEI_CTRL_TRG_TABLE_TRG_CMD_CMD_TRIGGER0_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_TRG_TABLE_TRG_CMD_CMD_TRIGGER0_SHIFT (0U)
#define SEI_CTRL_TRG_TABLE_TRG_CMD_CMD_TRIGGER0_SET(x) (((uint32_t)(x) << SEI_CTRL_TRG_TABLE_TRG_CMD_CMD_TRIGGER0_SHIFT) & SEI_CTRL_TRG_TABLE_TRG_CMD_CMD_TRIGGER0_MASK)
#define SEI_CTRL_TRG_TABLE_TRG_CMD_CMD_TRIGGER0_GET(x) (((uint32_t)(x) & SEI_CTRL_TRG_TABLE_TRG_CMD_CMD_TRIGGER0_MASK) >> SEI_CTRL_TRG_TABLE_TRG_CMD_CMD_TRIGGER0_SHIFT)

/* Bitfield definition for register of struct array CTRL: TIME_TRIGGER0 */
/*
 * TRIGGER0_TIME (RO)
 *
 * Trigger time
 */
#define SEI_CTRL_TRG_TABLE_TRG_TIME_TRIGGER0_TIME_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_TRG_TABLE_TRG_TIME_TRIGGER0_TIME_SHIFT (0U)
#define SEI_CTRL_TRG_TABLE_TRG_TIME_TRIGGER0_TIME_GET(x) (((uint32_t)(x) & SEI_CTRL_TRG_TABLE_TRG_TIME_TRIGGER0_TIME_MASK) >> SEI_CTRL_TRG_TABLE_TRG_TIME_TRIGGER0_TIME_SHIFT)

/* Bitfield definition for register of struct array CTRL: CMD_MODE */
/*
 * WLEN (RW)
 *
 * word length
 * 0: 1 bit
 * 1: 2 bit
 * ...
 * 31: 32 bit
 */
#define SEI_CTRL_COMMAND_CMD_MODE_WLEN_MASK (0x1F0000UL)
#define SEI_CTRL_COMMAND_CMD_MODE_WLEN_SHIFT (16U)
#define SEI_CTRL_COMMAND_CMD_MODE_WLEN_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_CMD_MODE_WLEN_SHIFT) & SEI_CTRL_COMMAND_CMD_MODE_WLEN_MASK)
#define SEI_CTRL_COMMAND_CMD_MODE_WLEN_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_MODE_WLEN_MASK) >> SEI_CTRL_COMMAND_CMD_MODE_WLEN_SHIFT)

/*
 * WORDER (RW)
 *
 * word order
 * 0: sample as bit order
 * 1: different from bit order
 */
#define SEI_CTRL_COMMAND_CMD_MODE_WORDER_MASK (0x800U)
#define SEI_CTRL_COMMAND_CMD_MODE_WORDER_SHIFT (11U)
#define SEI_CTRL_COMMAND_CMD_MODE_WORDER_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_CMD_MODE_WORDER_SHIFT) & SEI_CTRL_COMMAND_CMD_MODE_WORDER_MASK)
#define SEI_CTRL_COMMAND_CMD_MODE_WORDER_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_MODE_WORDER_MASK) >> SEI_CTRL_COMMAND_CMD_MODE_WORDER_SHIFT)

/*
 * BORDER (RW)
 *
 * bit order
 * 0: LSB first
 * 1: MSB first
 */
#define SEI_CTRL_COMMAND_CMD_MODE_BORDER_MASK (0x400U)
#define SEI_CTRL_COMMAND_CMD_MODE_BORDER_SHIFT (10U)
#define SEI_CTRL_COMMAND_CMD_MODE_BORDER_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_CMD_MODE_BORDER_SHIFT) & SEI_CTRL_COMMAND_CMD_MODE_BORDER_MASK)
#define SEI_CTRL_COMMAND_CMD_MODE_BORDER_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_MODE_BORDER_MASK) >> SEI_CTRL_COMMAND_CMD_MODE_BORDER_SHIFT)

/*
 * SIGNED (RW)
 *
 * Signed
 * 0: unsigned value
 * 1: signed value
 */
#define SEI_CTRL_COMMAND_CMD_MODE_SIGNED_MASK (0x200U)
#define SEI_CTRL_COMMAND_CMD_MODE_SIGNED_SHIFT (9U)
#define SEI_CTRL_COMMAND_CMD_MODE_SIGNED_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_CMD_MODE_SIGNED_SHIFT) & SEI_CTRL_COMMAND_CMD_MODE_SIGNED_MASK)
#define SEI_CTRL_COMMAND_CMD_MODE_SIGNED_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_MODE_SIGNED_MASK) >> SEI_CTRL_COMMAND_CMD_MODE_SIGNED_SHIFT)

/*
 * REWIND (WC)
 *
 * Write 1 to rewind read/write pointer, this is a self clear bit
 */
#define SEI_CTRL_COMMAND_CMD_MODE_REWIND_MASK (0x100U)
#define SEI_CTRL_COMMAND_CMD_MODE_REWIND_SHIFT (8U)
#define SEI_CTRL_COMMAND_CMD_MODE_REWIND_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_CMD_MODE_REWIND_SHIFT) & SEI_CTRL_COMMAND_CMD_MODE_REWIND_MASK)
#define SEI_CTRL_COMMAND_CMD_MODE_REWIND_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_MODE_REWIND_MASK) >> SEI_CTRL_COMMAND_CMD_MODE_REWIND_SHIFT)

/*
 * MODE (RW)
 *
 * Data mode
 * 0: data mode
 * 1: check mode
 * 2: CRC mode
 */
#define SEI_CTRL_COMMAND_CMD_MODE_MODE_MASK (0x3U)
#define SEI_CTRL_COMMAND_CMD_MODE_MODE_SHIFT (0U)
#define SEI_CTRL_COMMAND_CMD_MODE_MODE_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_CMD_MODE_MODE_SHIFT) & SEI_CTRL_COMMAND_CMD_MODE_MODE_MASK)
#define SEI_CTRL_COMMAND_CMD_MODE_MODE_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_MODE_MODE_MASK) >> SEI_CTRL_COMMAND_CMD_MODE_MODE_SHIFT)

/* Bitfield definition for register of struct array CTRL: CMD_IDX */
/*
 * LAST_BIT (RW)
 *
 * Last bit index for tranceive
 */
#define SEI_CTRL_COMMAND_CMD_IDX_LAST_BIT_MASK (0x1F000000UL)
#define SEI_CTRL_COMMAND_CMD_IDX_LAST_BIT_SHIFT (24U)
#define SEI_CTRL_COMMAND_CMD_IDX_LAST_BIT_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_CMD_IDX_LAST_BIT_SHIFT) & SEI_CTRL_COMMAND_CMD_IDX_LAST_BIT_MASK)
#define SEI_CTRL_COMMAND_CMD_IDX_LAST_BIT_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_IDX_LAST_BIT_MASK) >> SEI_CTRL_COMMAND_CMD_IDX_LAST_BIT_SHIFT)

/*
 * FIRST_BIT (RW)
 *
 * First bit index for tranceive
 */
#define SEI_CTRL_COMMAND_CMD_IDX_FIRST_BIT_MASK (0x1F0000UL)
#define SEI_CTRL_COMMAND_CMD_IDX_FIRST_BIT_SHIFT (16U)
#define SEI_CTRL_COMMAND_CMD_IDX_FIRST_BIT_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_CMD_IDX_FIRST_BIT_SHIFT) & SEI_CTRL_COMMAND_CMD_IDX_FIRST_BIT_MASK)
#define SEI_CTRL_COMMAND_CMD_IDX_FIRST_BIT_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_IDX_FIRST_BIT_MASK) >> SEI_CTRL_COMMAND_CMD_IDX_FIRST_BIT_SHIFT)

/*
 * MAX_BIT (RW)
 *
 * Highest bit index
 */
#define SEI_CTRL_COMMAND_CMD_IDX_MAX_BIT_MASK (0x1F00U)
#define SEI_CTRL_COMMAND_CMD_IDX_MAX_BIT_SHIFT (8U)
#define SEI_CTRL_COMMAND_CMD_IDX_MAX_BIT_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_CMD_IDX_MAX_BIT_SHIFT) & SEI_CTRL_COMMAND_CMD_IDX_MAX_BIT_MASK)
#define SEI_CTRL_COMMAND_CMD_IDX_MAX_BIT_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_IDX_MAX_BIT_MASK) >> SEI_CTRL_COMMAND_CMD_IDX_MAX_BIT_SHIFT)

/*
 * MIN_BIT (RW)
 *
 * Lowest bit index
 */
#define SEI_CTRL_COMMAND_CMD_IDX_MIN_BIT_MASK (0x1FU)
#define SEI_CTRL_COMMAND_CMD_IDX_MIN_BIT_SHIFT (0U)
#define SEI_CTRL_COMMAND_CMD_IDX_MIN_BIT_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_CMD_IDX_MIN_BIT_SHIFT) & SEI_CTRL_COMMAND_CMD_IDX_MIN_BIT_MASK)
#define SEI_CTRL_COMMAND_CMD_IDX_MIN_BIT_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_IDX_MIN_BIT_MASK) >> SEI_CTRL_COMMAND_CMD_IDX_MIN_BIT_SHIFT)

/* Bitfield definition for register of struct array CTRL: CMD_GOLD */
/*
 * GOLD_VALUE (RW)
 *
 * Gold value for check mode
 */
#define SEI_CTRL_COMMAND_CMD_GOLD_GOLD_VALUE_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_COMMAND_CMD_GOLD_GOLD_VALUE_SHIFT (0U)
#define SEI_CTRL_COMMAND_CMD_GOLD_GOLD_VALUE_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_CMD_GOLD_GOLD_VALUE_SHIFT) & SEI_CTRL_COMMAND_CMD_GOLD_GOLD_VALUE_MASK)
#define SEI_CTRL_COMMAND_CMD_GOLD_GOLD_VALUE_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_GOLD_GOLD_VALUE_MASK) >> SEI_CTRL_COMMAND_CMD_GOLD_GOLD_VALUE_SHIFT)

/* Bitfield definition for register of struct array CTRL: CMD_CRCINIT */
/*
 * CRC_INIT (RW)
 *
 * CRC initial value
 */
#define SEI_CTRL_COMMAND_CMD_CRCINIT_CRC_INIT_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_COMMAND_CMD_CRCINIT_CRC_INIT_SHIFT (0U)
#define SEI_CTRL_COMMAND_CMD_CRCINIT_CRC_INIT_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_CMD_CRCINIT_CRC_INIT_SHIFT) & SEI_CTRL_COMMAND_CMD_CRCINIT_CRC_INIT_MASK)
#define SEI_CTRL_COMMAND_CMD_CRCINIT_CRC_INIT_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_CRCINIT_CRC_INIT_MASK) >> SEI_CTRL_COMMAND_CMD_CRCINIT_CRC_INIT_SHIFT)

/* Bitfield definition for register of struct array CTRL: CMD_CRCPOLY */
/*
 * CRC_POLY (RW)
 *
 * CRC polymonial
 */
#define SEI_CTRL_COMMAND_CMD_CRCPOLY_CRC_POLY_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_COMMAND_CMD_CRCPOLY_CRC_POLY_SHIFT (0U)
#define SEI_CTRL_COMMAND_CMD_CRCPOLY_CRC_POLY_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_CMD_CRCPOLY_CRC_POLY_SHIFT) & SEI_CTRL_COMMAND_CMD_CRCPOLY_CRC_POLY_MASK)
#define SEI_CTRL_COMMAND_CMD_CRCPOLY_CRC_POLY_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_CRCPOLY_CRC_POLY_MASK) >> SEI_CTRL_COMMAND_CMD_CRCPOLY_CRC_POLY_SHIFT)

/* Bitfield definition for register of struct array CTRL: CMD */
/*
 * DATA (RW)
 *
 * DATA
 */
#define SEI_CTRL_COMMAND_CMD_DATA_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_COMMAND_CMD_DATA_SHIFT (0U)
#define SEI_CTRL_COMMAND_CMD_DATA_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_CMD_DATA_SHIFT) & SEI_CTRL_COMMAND_CMD_DATA_MASK)
#define SEI_CTRL_COMMAND_CMD_DATA_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_DATA_MASK) >> SEI_CTRL_COMMAND_CMD_DATA_SHIFT)

/* Bitfield definition for register of struct array CTRL: CMD_SET */
/*
 * DATA_SET (RW)
 *
 * DATA bit set
 */
#define SEI_CTRL_COMMAND_CMD_SET_DATA_SET_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_COMMAND_CMD_SET_DATA_SET_SHIFT (0U)
#define SEI_CTRL_COMMAND_CMD_SET_DATA_SET_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_CMD_SET_DATA_SET_SHIFT) & SEI_CTRL_COMMAND_CMD_SET_DATA_SET_MASK)
#define SEI_CTRL_COMMAND_CMD_SET_DATA_SET_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_SET_DATA_SET_MASK) >> SEI_CTRL_COMMAND_CMD_SET_DATA_SET_SHIFT)

/* Bitfield definition for register of struct array CTRL: CMD_CLR */
/*
 * DATA_CLR (RW)
 *
 * DATA bit clear
 */
#define SEI_CTRL_COMMAND_CMD_CLR_DATA_CLR_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_COMMAND_CMD_CLR_DATA_CLR_SHIFT (0U)
#define SEI_CTRL_COMMAND_CMD_CLR_DATA_CLR_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_CMD_CLR_DATA_CLR_SHIFT) & SEI_CTRL_COMMAND_CMD_CLR_DATA_CLR_MASK)
#define SEI_CTRL_COMMAND_CMD_CLR_DATA_CLR_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_CLR_DATA_CLR_MASK) >> SEI_CTRL_COMMAND_CMD_CLR_DATA_CLR_SHIFT)

/* Bitfield definition for register of struct array CTRL: CMD_INV */
/*
 * DATA_TGL (RW)
 *
 * DATA bit toggle
 */
#define SEI_CTRL_COMMAND_CMD_INV_DATA_TGL_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_COMMAND_CMD_INV_DATA_TGL_SHIFT (0U)
#define SEI_CTRL_COMMAND_CMD_INV_DATA_TGL_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_CMD_INV_DATA_TGL_SHIFT) & SEI_CTRL_COMMAND_CMD_INV_DATA_TGL_MASK)
#define SEI_CTRL_COMMAND_CMD_INV_DATA_TGL_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_INV_DATA_TGL_MASK) >> SEI_CTRL_COMMAND_CMD_INV_DATA_TGL_SHIFT)

/* Bitfield definition for register of struct array CTRL: CMD_IN */
/*
 * DATA_IN (RO)
 *
 * Commad input
 */
#define SEI_CTRL_COMMAND_CMD_IN_DATA_IN_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_COMMAND_CMD_IN_DATA_IN_SHIFT (0U)
#define SEI_CTRL_COMMAND_CMD_IN_DATA_IN_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_IN_DATA_IN_MASK) >> SEI_CTRL_COMMAND_CMD_IN_DATA_IN_SHIFT)

/* Bitfield definition for register of struct array CTRL: CMD_OUT */
/*
 * DATA_OUT (RO)
 *
 * Command output
 */
#define SEI_CTRL_COMMAND_CMD_OUT_DATA_OUT_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_COMMAND_CMD_OUT_DATA_OUT_SHIFT (0U)
#define SEI_CTRL_COMMAND_CMD_OUT_DATA_OUT_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_OUT_DATA_OUT_MASK) >> SEI_CTRL_COMMAND_CMD_OUT_DATA_OUT_SHIFT)

/* Bitfield definition for register of struct array CTRL: CMD_STS */
/*
 * CRC_IDX (RO)
 *
 * CRC index
 */
#define SEI_CTRL_COMMAND_CMD_STS_CRC_IDX_MASK (0x1F000000UL)
#define SEI_CTRL_COMMAND_CMD_STS_CRC_IDX_SHIFT (24U)
#define SEI_CTRL_COMMAND_CMD_STS_CRC_IDX_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_STS_CRC_IDX_MASK) >> SEI_CTRL_COMMAND_CMD_STS_CRC_IDX_SHIFT)

/*
 * WORD_IDX (RO)
 *
 * Word index
 */
#define SEI_CTRL_COMMAND_CMD_STS_WORD_IDX_MASK (0x1F0000UL)
#define SEI_CTRL_COMMAND_CMD_STS_WORD_IDX_SHIFT (16U)
#define SEI_CTRL_COMMAND_CMD_STS_WORD_IDX_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_STS_WORD_IDX_MASK) >> SEI_CTRL_COMMAND_CMD_STS_WORD_IDX_SHIFT)

/*
 * WORD_CNT (RO)
 *
 * Word counter
 */
#define SEI_CTRL_COMMAND_CMD_STS_WORD_CNT_MASK (0x1F00U)
#define SEI_CTRL_COMMAND_CMD_STS_WORD_CNT_SHIFT (8U)
#define SEI_CTRL_COMMAND_CMD_STS_WORD_CNT_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_STS_WORD_CNT_MASK) >> SEI_CTRL_COMMAND_CMD_STS_WORD_CNT_SHIFT)

/*
 * BIT_IDX (RO)
 *
 * Bit index
 */
#define SEI_CTRL_COMMAND_CMD_STS_BIT_IDX_MASK (0x1FU)
#define SEI_CTRL_COMMAND_CMD_STS_BIT_IDX_SHIFT (0U)
#define SEI_CTRL_COMMAND_CMD_STS_BIT_IDX_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_CMD_STS_BIT_IDX_MASK) >> SEI_CTRL_COMMAND_CMD_STS_BIT_IDX_SHIFT)

/* Bitfield definition for register of struct array CTRL: CMD_MIN */
/*
 * CMD_MIN (RW)
 *
 * minimum command value
 */
#define SEI_CTRL_COMMAND_TABLE_CMD_MIN_CMD_MIN_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_COMMAND_TABLE_CMD_MIN_CMD_MIN_SHIFT (0U)
#define SEI_CTRL_COMMAND_TABLE_CMD_MIN_CMD_MIN_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_TABLE_CMD_MIN_CMD_MIN_SHIFT) & SEI_CTRL_COMMAND_TABLE_CMD_MIN_CMD_MIN_MASK)
#define SEI_CTRL_COMMAND_TABLE_CMD_MIN_CMD_MIN_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_TABLE_CMD_MIN_CMD_MIN_MASK) >> SEI_CTRL_COMMAND_TABLE_CMD_MIN_CMD_MIN_SHIFT)

/* Bitfield definition for register of struct array CTRL: CMD_MAX */
/*
 * CMD_MAX (RW)
 *
 * maximum command value
 */
#define SEI_CTRL_COMMAND_TABLE_CMD_MAX_CMD_MAX_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_COMMAND_TABLE_CMD_MAX_CMD_MAX_SHIFT (0U)
#define SEI_CTRL_COMMAND_TABLE_CMD_MAX_CMD_MAX_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_TABLE_CMD_MAX_CMD_MAX_SHIFT) & SEI_CTRL_COMMAND_TABLE_CMD_MAX_CMD_MAX_MASK)
#define SEI_CTRL_COMMAND_TABLE_CMD_MAX_CMD_MAX_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_TABLE_CMD_MAX_CMD_MAX_MASK) >> SEI_CTRL_COMMAND_TABLE_CMD_MAX_CMD_MAX_SHIFT)

/* Bitfield definition for register of struct array CTRL: CMD_MSK */
/*
 * CMD_MASK (RW)
 *
 * compare mask
 */
#define SEI_CTRL_COMMAND_TABLE_CMD_MSK_CMD_MASK_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_COMMAND_TABLE_CMD_MSK_CMD_MASK_SHIFT (0U)
#define SEI_CTRL_COMMAND_TABLE_CMD_MSK_CMD_MASK_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_TABLE_CMD_MSK_CMD_MASK_SHIFT) & SEI_CTRL_COMMAND_TABLE_CMD_MSK_CMD_MASK_MASK)
#define SEI_CTRL_COMMAND_TABLE_CMD_MSK_CMD_MASK_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_TABLE_CMD_MSK_CMD_MASK_MASK) >> SEI_CTRL_COMMAND_TABLE_CMD_MSK_CMD_MASK_SHIFT)

/* Bitfield definition for register of struct array CTRL: CMD_PTRA */
/*
 * PTR3 (RW)
 *
 * pointer3
 */
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR3_MASK (0xFF000000UL)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR3_SHIFT (24U)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR3_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR3_SHIFT) & SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR3_MASK)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR3_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR3_MASK) >> SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR3_SHIFT)

/*
 * PTR2 (RW)
 *
 * pointer2
 */
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR2_MASK (0xFF0000UL)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR2_SHIFT (16U)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR2_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR2_SHIFT) & SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR2_MASK)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR2_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR2_MASK) >> SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR2_SHIFT)

/*
 * PTR1 (RW)
 *
 * pointer1
 */
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR1_MASK (0xFF00U)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR1_SHIFT (8U)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR1_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR1_SHIFT) & SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR1_MASK)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR1_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR1_MASK) >> SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR1_SHIFT)

/*
 * PTR0 (RW)
 *
 * pointer0
 */
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR0_MASK (0xFFU)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR0_SHIFT (0U)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR0_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR0_SHIFT) & SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR0_MASK)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR0_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR0_MASK) >> SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR0_SHIFT)

/* Bitfield definition for register of struct array CTRL: CMD_PTRB */
/*
 * PTR7 (RW)
 *
 * pointer7
 */
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR7_MASK (0xFF000000UL)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR7_SHIFT (24U)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR7_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR7_SHIFT) & SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR7_MASK)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR7_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR7_MASK) >> SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR7_SHIFT)

/*
 * PTR6 (RW)
 *
 * pointer6
 */
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR6_MASK (0xFF0000UL)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR6_SHIFT (16U)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR6_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR6_SHIFT) & SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR6_MASK)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR6_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR6_MASK) >> SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR6_SHIFT)

/*
 * PTR5 (RW)
 *
 * pointer5
 */
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR5_MASK (0xFF00U)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR5_SHIFT (8U)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR5_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR5_SHIFT) & SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR5_MASK)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR5_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR5_MASK) >> SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR5_SHIFT)

/*
 * PTR4 (RW)
 *
 * pointer4
 */
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR4_MASK (0xFFU)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR4_SHIFT (0U)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR4_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR4_SHIFT) & SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR4_MASK)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR4_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR4_MASK) >> SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR4_SHIFT)

/* Bitfield definition for register of struct array CTRL: CMD_PTRC */
/*
 * PTR11 (RW)
 *
 * pointer11
 */
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR11_MASK (0xFF000000UL)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR11_SHIFT (24U)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR11_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR11_SHIFT) & SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR11_MASK)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR11_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR11_MASK) >> SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR11_SHIFT)

/*
 * PTR10 (RW)
 *
 * pointer10
 */
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR10_MASK (0xFF0000UL)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR10_SHIFT (16U)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR10_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR10_SHIFT) & SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR10_MASK)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR10_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR10_MASK) >> SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR10_SHIFT)

/*
 * PTR9 (RW)
 *
 * pointer9
 */
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR9_MASK (0xFF00U)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR9_SHIFT (8U)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR9_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR9_SHIFT) & SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR9_MASK)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR9_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR9_MASK) >> SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR9_SHIFT)

/*
 * PTR8 (RW)
 *
 * pointer8
 */
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR8_MASK (0xFFU)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR8_SHIFT (0U)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR8_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR8_SHIFT) & SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR8_MASK)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR8_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR8_MASK) >> SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR8_SHIFT)

/* Bitfield definition for register of struct array CTRL: CMD_PTRD */
/*
 * PTR15 (RW)
 *
 * pointer15
 */
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR15_MASK (0xFF000000UL)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR15_SHIFT (24U)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR15_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR15_SHIFT) & SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR15_MASK)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR15_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR15_MASK) >> SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR15_SHIFT)

/*
 * PTR14 (RW)
 *
 * pointer14
 */
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR14_MASK (0xFF0000UL)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR14_SHIFT (16U)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR14_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR14_SHIFT) & SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR14_MASK)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR14_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR14_MASK) >> SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR14_SHIFT)

/*
 * PTR13 (RW)
 *
 * pointer13
 */
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR13_MASK (0xFF00U)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR13_SHIFT (8U)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR13_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR13_SHIFT) & SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR13_MASK)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR13_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR13_MASK) >> SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR13_SHIFT)

/*
 * PTR12 (RW)
 *
 * pointer12
 */
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR12_MASK (0xFFU)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR12_SHIFT (0U)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR12_SET(x) (((uint32_t)(x) << SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR12_SHIFT) & SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR12_MASK)
#define SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR12_GET(x) (((uint32_t)(x) & SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR12_MASK) >> SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR12_SHIFT)

/* Bitfield definition for register of struct array CTRL: TRAN_0_1 */
/*
 * POINTER (RW)
 *
 * pointer
 */
#define SEI_CTRL_LATCH_TRAN_CFG_POINTER_MASK (0xFF000000UL)
#define SEI_CTRL_LATCH_TRAN_CFG_POINTER_SHIFT (24U)
#define SEI_CTRL_LATCH_TRAN_CFG_POINTER_SET(x) (((uint32_t)(x) << SEI_CTRL_LATCH_TRAN_CFG_POINTER_SHIFT) & SEI_CTRL_LATCH_TRAN_CFG_POINTER_MASK)
#define SEI_CTRL_LATCH_TRAN_CFG_POINTER_GET(x) (((uint32_t)(x) & SEI_CTRL_LATCH_TRAN_CFG_POINTER_MASK) >> SEI_CTRL_LATCH_TRAN_CFG_POINTER_SHIFT)

/*
 * STATE (RW)
 *
 * enable state by set bit
 * 1: state0
 * 2: state1
 * 4: state2
 * 8: state3
 */
#define SEI_CTRL_LATCH_TRAN_CFG_STATE_MASK (0xF00000UL)
#define SEI_CTRL_LATCH_TRAN_CFG_STATE_SHIFT (20U)
#define SEI_CTRL_LATCH_TRAN_CFG_STATE_SET(x) (((uint32_t)(x) << SEI_CTRL_LATCH_TRAN_CFG_STATE_SHIFT) & SEI_CTRL_LATCH_TRAN_CFG_STATE_MASK)
#define SEI_CTRL_LATCH_TRAN_CFG_STATE_GET(x) (((uint32_t)(x) & SEI_CTRL_LATCH_TRAN_CFG_STATE_MASK) >> SEI_CTRL_LATCH_TRAN_CFG_STATE_SHIFT)

/*
 * CFG_TM (RW)
 *
 * timeout
 * 0: high
 * 1: low
 * 2: rise
 * 3: fall
 */
#define SEI_CTRL_LATCH_TRAN_CFG_CFG_TM_MASK (0x30000UL)
#define SEI_CTRL_LATCH_TRAN_CFG_CFG_TM_SHIFT (16U)
#define SEI_CTRL_LATCH_TRAN_CFG_CFG_TM_SET(x) (((uint32_t)(x) << SEI_CTRL_LATCH_TRAN_CFG_CFG_TM_SHIFT) & SEI_CTRL_LATCH_TRAN_CFG_CFG_TM_MASK)
#define SEI_CTRL_LATCH_TRAN_CFG_CFG_TM_GET(x) (((uint32_t)(x) & SEI_CTRL_LATCH_TRAN_CFG_CFG_TM_MASK) >> SEI_CTRL_LATCH_TRAN_CFG_CFG_TM_SHIFT)

/*
 * CFG_RXD (RW)
 *
 * data received
 * 0: high
 * 1: low
 * 2: rise
 * 3: fall
 */
#define SEI_CTRL_LATCH_TRAN_CFG_CFG_RXD_MASK (0xC000U)
#define SEI_CTRL_LATCH_TRAN_CFG_CFG_RXD_SHIFT (14U)
#define SEI_CTRL_LATCH_TRAN_CFG_CFG_RXD_SET(x) (((uint32_t)(x) << SEI_CTRL_LATCH_TRAN_CFG_CFG_RXD_SHIFT) & SEI_CTRL_LATCH_TRAN_CFG_CFG_RXD_MASK)
#define SEI_CTRL_LATCH_TRAN_CFG_CFG_RXD_GET(x) (((uint32_t)(x) & SEI_CTRL_LATCH_TRAN_CFG_CFG_RXD_MASK) >> SEI_CTRL_LATCH_TRAN_CFG_CFG_RXD_SHIFT)

/*
 * CFG_TXD (RW)
 *
 * data send
 * 0: high
 * 1: low
 * 2: rise
 * 3: fall
 */
#define SEI_CTRL_LATCH_TRAN_CFG_CFG_TXD_MASK (0x3000U)
#define SEI_CTRL_LATCH_TRAN_CFG_CFG_TXD_SHIFT (12U)
#define SEI_CTRL_LATCH_TRAN_CFG_CFG_TXD_SET(x) (((uint32_t)(x) << SEI_CTRL_LATCH_TRAN_CFG_CFG_TXD_SHIFT) & SEI_CTRL_LATCH_TRAN_CFG_CFG_TXD_MASK)
#define SEI_CTRL_LATCH_TRAN_CFG_CFG_TXD_GET(x) (((uint32_t)(x) & SEI_CTRL_LATCH_TRAN_CFG_CFG_TXD_MASK) >> SEI_CTRL_LATCH_TRAN_CFG_CFG_TXD_SHIFT)

/*
 * CFG_CLK (RW)
 *
 * clock
 * 0: high
 * 1: low
 * 2: rise
 * 3: fall
 */
#define SEI_CTRL_LATCH_TRAN_CFG_CFG_CLK_MASK (0xC00U)
#define SEI_CTRL_LATCH_TRAN_CFG_CFG_CLK_SHIFT (10U)
#define SEI_CTRL_LATCH_TRAN_CFG_CFG_CLK_SET(x) (((uint32_t)(x) << SEI_CTRL_LATCH_TRAN_CFG_CFG_CLK_SHIFT) & SEI_CTRL_LATCH_TRAN_CFG_CFG_CLK_MASK)
#define SEI_CTRL_LATCH_TRAN_CFG_CFG_CLK_GET(x) (((uint32_t)(x) & SEI_CTRL_LATCH_TRAN_CFG_CFG_CLK_MASK) >> SEI_CTRL_LATCH_TRAN_CFG_CFG_CLK_SHIFT)

/*
 * CFG_PTR (RW)
 *
 * pointer
 * 0: match
 * 1: not match
 * 2:entry
 * 3:leave
 */
#define SEI_CTRL_LATCH_TRAN_CFG_CFG_PTR_MASK (0x300U)
#define SEI_CTRL_LATCH_TRAN_CFG_CFG_PTR_SHIFT (8U)
#define SEI_CTRL_LATCH_TRAN_CFG_CFG_PTR_SET(x) (((uint32_t)(x) << SEI_CTRL_LATCH_TRAN_CFG_CFG_PTR_SHIFT) & SEI_CTRL_LATCH_TRAN_CFG_CFG_PTR_MASK)
#define SEI_CTRL_LATCH_TRAN_CFG_CFG_PTR_GET(x) (((uint32_t)(x) & SEI_CTRL_LATCH_TRAN_CFG_CFG_PTR_MASK) >> SEI_CTRL_LATCH_TRAN_CFG_CFG_PTR_SHIFT)

/*
 * OV_TM (RW)
 *
 * override timeout check
 */
#define SEI_CTRL_LATCH_TRAN_CFG_OV_TM_MASK (0x10U)
#define SEI_CTRL_LATCH_TRAN_CFG_OV_TM_SHIFT (4U)
#define SEI_CTRL_LATCH_TRAN_CFG_OV_TM_SET(x) (((uint32_t)(x) << SEI_CTRL_LATCH_TRAN_CFG_OV_TM_SHIFT) & SEI_CTRL_LATCH_TRAN_CFG_OV_TM_MASK)
#define SEI_CTRL_LATCH_TRAN_CFG_OV_TM_GET(x) (((uint32_t)(x) & SEI_CTRL_LATCH_TRAN_CFG_OV_TM_MASK) >> SEI_CTRL_LATCH_TRAN_CFG_OV_TM_SHIFT)

/*
 * OV_RXD (RW)
 *
 * override RX data check
 */
#define SEI_CTRL_LATCH_TRAN_CFG_OV_RXD_MASK (0x8U)
#define SEI_CTRL_LATCH_TRAN_CFG_OV_RXD_SHIFT (3U)
#define SEI_CTRL_LATCH_TRAN_CFG_OV_RXD_SET(x) (((uint32_t)(x) << SEI_CTRL_LATCH_TRAN_CFG_OV_RXD_SHIFT) & SEI_CTRL_LATCH_TRAN_CFG_OV_RXD_MASK)
#define SEI_CTRL_LATCH_TRAN_CFG_OV_RXD_GET(x) (((uint32_t)(x) & SEI_CTRL_LATCH_TRAN_CFG_OV_RXD_MASK) >> SEI_CTRL_LATCH_TRAN_CFG_OV_RXD_SHIFT)

/*
 * OV_TXD (RW)
 *
 * override TX data check
 */
#define SEI_CTRL_LATCH_TRAN_CFG_OV_TXD_MASK (0x4U)
#define SEI_CTRL_LATCH_TRAN_CFG_OV_TXD_SHIFT (2U)
#define SEI_CTRL_LATCH_TRAN_CFG_OV_TXD_SET(x) (((uint32_t)(x) << SEI_CTRL_LATCH_TRAN_CFG_OV_TXD_SHIFT) & SEI_CTRL_LATCH_TRAN_CFG_OV_TXD_MASK)
#define SEI_CTRL_LATCH_TRAN_CFG_OV_TXD_GET(x) (((uint32_t)(x) & SEI_CTRL_LATCH_TRAN_CFG_OV_TXD_MASK) >> SEI_CTRL_LATCH_TRAN_CFG_OV_TXD_SHIFT)

/*
 * OV_CLK (RW)
 *
 * override clock check
 */
#define SEI_CTRL_LATCH_TRAN_CFG_OV_CLK_MASK (0x2U)
#define SEI_CTRL_LATCH_TRAN_CFG_OV_CLK_SHIFT (1U)
#define SEI_CTRL_LATCH_TRAN_CFG_OV_CLK_SET(x) (((uint32_t)(x) << SEI_CTRL_LATCH_TRAN_CFG_OV_CLK_SHIFT) & SEI_CTRL_LATCH_TRAN_CFG_OV_CLK_MASK)
#define SEI_CTRL_LATCH_TRAN_CFG_OV_CLK_GET(x) (((uint32_t)(x) & SEI_CTRL_LATCH_TRAN_CFG_OV_CLK_MASK) >> SEI_CTRL_LATCH_TRAN_CFG_OV_CLK_SHIFT)

/*
 * OV_PTR (RW)
 *
 * override pointer check
 */
#define SEI_CTRL_LATCH_TRAN_CFG_OV_PTR_MASK (0x1U)
#define SEI_CTRL_LATCH_TRAN_CFG_OV_PTR_SHIFT (0U)
#define SEI_CTRL_LATCH_TRAN_CFG_OV_PTR_SET(x) (((uint32_t)(x) << SEI_CTRL_LATCH_TRAN_CFG_OV_PTR_SHIFT) & SEI_CTRL_LATCH_TRAN_CFG_OV_PTR_MASK)
#define SEI_CTRL_LATCH_TRAN_CFG_OV_PTR_GET(x) (((uint32_t)(x) & SEI_CTRL_LATCH_TRAN_CFG_OV_PTR_MASK) >> SEI_CTRL_LATCH_TRAN_CFG_OV_PTR_SHIFT)

/* Bitfield definition for register of struct array CTRL: LAT_CFG */
/*
 * EN (RW)
 *
 * Enable latch
 * 0: disable
 * 1: enable
 */
#define SEI_CTRL_LATCH_LAT_CFG_EN_MASK (0x80000000UL)
#define SEI_CTRL_LATCH_LAT_CFG_EN_SHIFT (31U)
#define SEI_CTRL_LATCH_LAT_CFG_EN_SET(x) (((uint32_t)(x) << SEI_CTRL_LATCH_LAT_CFG_EN_SHIFT) & SEI_CTRL_LATCH_LAT_CFG_EN_MASK)
#define SEI_CTRL_LATCH_LAT_CFG_EN_GET(x) (((uint32_t)(x) & SEI_CTRL_LATCH_LAT_CFG_EN_MASK) >> SEI_CTRL_LATCH_LAT_CFG_EN_SHIFT)

/*
 * SELECT (RW)
 *
 * Output select
 * 0: state0-state1
 * 1: state1-state2
 * 2: state2-state3
 * 3: state3-state0
 */
#define SEI_CTRL_LATCH_LAT_CFG_SELECT_MASK (0x7000000UL)
#define SEI_CTRL_LATCH_LAT_CFG_SELECT_SHIFT (24U)
#define SEI_CTRL_LATCH_LAT_CFG_SELECT_SET(x) (((uint32_t)(x) << SEI_CTRL_LATCH_LAT_CFG_SELECT_SHIFT) & SEI_CTRL_LATCH_LAT_CFG_SELECT_MASK)
#define SEI_CTRL_LATCH_LAT_CFG_SELECT_GET(x) (((uint32_t)(x) & SEI_CTRL_LATCH_LAT_CFG_SELECT_MASK) >> SEI_CTRL_LATCH_LAT_CFG_SELECT_SHIFT)

/*
 * DELAY (RW)
 *
 * Delay in system clock cycle, for state transition
 */
#define SEI_CTRL_LATCH_LAT_CFG_DELAY_MASK (0xFFFFU)
#define SEI_CTRL_LATCH_LAT_CFG_DELAY_SHIFT (0U)
#define SEI_CTRL_LATCH_LAT_CFG_DELAY_SET(x) (((uint32_t)(x) << SEI_CTRL_LATCH_LAT_CFG_DELAY_SHIFT) & SEI_CTRL_LATCH_LAT_CFG_DELAY_MASK)
#define SEI_CTRL_LATCH_LAT_CFG_DELAY_GET(x) (((uint32_t)(x) & SEI_CTRL_LATCH_LAT_CFG_DELAY_MASK) >> SEI_CTRL_LATCH_LAT_CFG_DELAY_SHIFT)

/* Bitfield definition for register of struct array CTRL: LAT_TIME */
/*
 * LAT_TIME (RO)
 *
 * Latch time
 */
#define SEI_CTRL_LATCH_LAT_TIME_LAT_TIME_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_LATCH_LAT_TIME_LAT_TIME_SHIFT (0U)
#define SEI_CTRL_LATCH_LAT_TIME_LAT_TIME_GET(x) (((uint32_t)(x) & SEI_CTRL_LATCH_LAT_TIME_LAT_TIME_MASK) >> SEI_CTRL_LATCH_LAT_TIME_LAT_TIME_SHIFT)

/* Bitfield definition for register of struct array CTRL: LAT_STS */
/*
 * STATE (RO)
 *
 * State
 */
#define SEI_CTRL_LATCH_LAT_STS_STATE_MASK (0x7000000UL)
#define SEI_CTRL_LATCH_LAT_STS_STATE_SHIFT (24U)
#define SEI_CTRL_LATCH_LAT_STS_STATE_GET(x) (((uint32_t)(x) & SEI_CTRL_LATCH_LAT_STS_STATE_MASK) >> SEI_CTRL_LATCH_LAT_STS_STATE_SHIFT)

/*
 * LAT_CNT (RO)
 *
 * Latch counter
 */
#define SEI_CTRL_LATCH_LAT_STS_LAT_CNT_MASK (0xFFFFU)
#define SEI_CTRL_LATCH_LAT_STS_LAT_CNT_SHIFT (0U)
#define SEI_CTRL_LATCH_LAT_STS_LAT_CNT_GET(x) (((uint32_t)(x) & SEI_CTRL_LATCH_LAT_STS_LAT_CNT_MASK) >> SEI_CTRL_LATCH_LAT_STS_LAT_CNT_SHIFT)

/* Bitfield definition for register of struct array CTRL: SMP_EN */
/*
 * ACC_EN (RW)
 *
 * Position include acceleration
 */
#define SEI_CTRL_POSITION_SMP_EN_ACC_EN_MASK (0x80000000UL)
#define SEI_CTRL_POSITION_SMP_EN_ACC_EN_SHIFT (31U)
#define SEI_CTRL_POSITION_SMP_EN_ACC_EN_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_SMP_EN_ACC_EN_SHIFT) & SEI_CTRL_POSITION_SMP_EN_ACC_EN_MASK)
#define SEI_CTRL_POSITION_SMP_EN_ACC_EN_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_EN_ACC_EN_MASK) >> SEI_CTRL_POSITION_SMP_EN_ACC_EN_SHIFT)

/*
 * ACC_SEL (RW)
 *
 * Data register for acceleration transfer
 */
#define SEI_CTRL_POSITION_SMP_EN_ACC_SEL_MASK (0x1F000000UL)
#define SEI_CTRL_POSITION_SMP_EN_ACC_SEL_SHIFT (24U)
#define SEI_CTRL_POSITION_SMP_EN_ACC_SEL_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_SMP_EN_ACC_SEL_SHIFT) & SEI_CTRL_POSITION_SMP_EN_ACC_SEL_MASK)
#define SEI_CTRL_POSITION_SMP_EN_ACC_SEL_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_EN_ACC_SEL_MASK) >> SEI_CTRL_POSITION_SMP_EN_ACC_SEL_SHIFT)

/*
 * SPD_EN (RW)
 *
 * Position include speed
 */
#define SEI_CTRL_POSITION_SMP_EN_SPD_EN_MASK (0x800000UL)
#define SEI_CTRL_POSITION_SMP_EN_SPD_EN_SHIFT (23U)
#define SEI_CTRL_POSITION_SMP_EN_SPD_EN_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_SMP_EN_SPD_EN_SHIFT) & SEI_CTRL_POSITION_SMP_EN_SPD_EN_MASK)
#define SEI_CTRL_POSITION_SMP_EN_SPD_EN_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_EN_SPD_EN_MASK) >> SEI_CTRL_POSITION_SMP_EN_SPD_EN_SHIFT)

/*
 * SPD_SEL (RW)
 *
 * Data register for speed transfer
 */
#define SEI_CTRL_POSITION_SMP_EN_SPD_SEL_MASK (0x1F0000UL)
#define SEI_CTRL_POSITION_SMP_EN_SPD_SEL_SHIFT (16U)
#define SEI_CTRL_POSITION_SMP_EN_SPD_SEL_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_SMP_EN_SPD_SEL_SHIFT) & SEI_CTRL_POSITION_SMP_EN_SPD_SEL_MASK)
#define SEI_CTRL_POSITION_SMP_EN_SPD_SEL_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_EN_SPD_SEL_MASK) >> SEI_CTRL_POSITION_SMP_EN_SPD_SEL_SHIFT)

/*
 * REV_EN (RW)
 *
 * Position include revolution
 */
#define SEI_CTRL_POSITION_SMP_EN_REV_EN_MASK (0x8000U)
#define SEI_CTRL_POSITION_SMP_EN_REV_EN_SHIFT (15U)
#define SEI_CTRL_POSITION_SMP_EN_REV_EN_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_SMP_EN_REV_EN_SHIFT) & SEI_CTRL_POSITION_SMP_EN_REV_EN_MASK)
#define SEI_CTRL_POSITION_SMP_EN_REV_EN_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_EN_REV_EN_MASK) >> SEI_CTRL_POSITION_SMP_EN_REV_EN_SHIFT)

/*
 * REV_SEL (RW)
 *
 * Data register for revolution transfer
 */
#define SEI_CTRL_POSITION_SMP_EN_REV_SEL_MASK (0x1F00U)
#define SEI_CTRL_POSITION_SMP_EN_REV_SEL_SHIFT (8U)
#define SEI_CTRL_POSITION_SMP_EN_REV_SEL_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_SMP_EN_REV_SEL_SHIFT) & SEI_CTRL_POSITION_SMP_EN_REV_SEL_MASK)
#define SEI_CTRL_POSITION_SMP_EN_REV_SEL_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_EN_REV_SEL_MASK) >> SEI_CTRL_POSITION_SMP_EN_REV_SEL_SHIFT)

/*
 * POS_EN (RW)
 *
 * Position include position
 */
#define SEI_CTRL_POSITION_SMP_EN_POS_EN_MASK (0x80U)
#define SEI_CTRL_POSITION_SMP_EN_POS_EN_SHIFT (7U)
#define SEI_CTRL_POSITION_SMP_EN_POS_EN_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_SMP_EN_POS_EN_SHIFT) & SEI_CTRL_POSITION_SMP_EN_POS_EN_MASK)
#define SEI_CTRL_POSITION_SMP_EN_POS_EN_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_EN_POS_EN_MASK) >> SEI_CTRL_POSITION_SMP_EN_POS_EN_SHIFT)

/*
 * POS_SEL (RW)
 *
 * Data register for position transfer
 */
#define SEI_CTRL_POSITION_SMP_EN_POS_SEL_MASK (0x1FU)
#define SEI_CTRL_POSITION_SMP_EN_POS_SEL_SHIFT (0U)
#define SEI_CTRL_POSITION_SMP_EN_POS_SEL_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_SMP_EN_POS_SEL_SHIFT) & SEI_CTRL_POSITION_SMP_EN_POS_SEL_MASK)
#define SEI_CTRL_POSITION_SMP_EN_POS_SEL_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_EN_POS_SEL_MASK) >> SEI_CTRL_POSITION_SMP_EN_POS_SEL_SHIFT)

/* Bitfield definition for register of struct array CTRL: SMP_CFG */
/*
 * ONCE (RW)
 *
 * Sample one time
 * 0: Sample during windows time
 * 1: Close sample window after first sample
 */
#define SEI_CTRL_POSITION_SMP_CFG_ONCE_MASK (0x1000000UL)
#define SEI_CTRL_POSITION_SMP_CFG_ONCE_SHIFT (24U)
#define SEI_CTRL_POSITION_SMP_CFG_ONCE_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_SMP_CFG_ONCE_SHIFT) & SEI_CTRL_POSITION_SMP_CFG_ONCE_MASK)
#define SEI_CTRL_POSITION_SMP_CFG_ONCE_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_CFG_ONCE_MASK) >> SEI_CTRL_POSITION_SMP_CFG_ONCE_SHIFT)

/*
 * LAT_SEL (RW)
 *
 * Latch selection
 * 0: latch 0
 * 1: latch 1
 * 2: latch 2
 * 3: latch 3
 */
#define SEI_CTRL_POSITION_SMP_CFG_LAT_SEL_MASK (0x30000UL)
#define SEI_CTRL_POSITION_SMP_CFG_LAT_SEL_SHIFT (16U)
#define SEI_CTRL_POSITION_SMP_CFG_LAT_SEL_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_SMP_CFG_LAT_SEL_SHIFT) & SEI_CTRL_POSITION_SMP_CFG_LAT_SEL_MASK)
#define SEI_CTRL_POSITION_SMP_CFG_LAT_SEL_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_CFG_LAT_SEL_MASK) >> SEI_CTRL_POSITION_SMP_CFG_LAT_SEL_SHIFT)

/*
 * WINDOW (RW)
 *
 * Sample window, in clock cycle
 */
#define SEI_CTRL_POSITION_SMP_CFG_WINDOW_MASK (0xFFFFU)
#define SEI_CTRL_POSITION_SMP_CFG_WINDOW_SHIFT (0U)
#define SEI_CTRL_POSITION_SMP_CFG_WINDOW_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_SMP_CFG_WINDOW_SHIFT) & SEI_CTRL_POSITION_SMP_CFG_WINDOW_MASK)
#define SEI_CTRL_POSITION_SMP_CFG_WINDOW_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_CFG_WINDOW_MASK) >> SEI_CTRL_POSITION_SMP_CFG_WINDOW_SHIFT)

/* Bitfield definition for register of struct array CTRL: SMP_DAT */
/*
 * DAT_SEL (RW)
 *
 * Data register sampled, each bit represent a data register
 */
#define SEI_CTRL_POSITION_SMP_DAT_DAT_SEL_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_POSITION_SMP_DAT_DAT_SEL_SHIFT (0U)
#define SEI_CTRL_POSITION_SMP_DAT_DAT_SEL_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_SMP_DAT_DAT_SEL_SHIFT) & SEI_CTRL_POSITION_SMP_DAT_DAT_SEL_MASK)
#define SEI_CTRL_POSITION_SMP_DAT_DAT_SEL_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_DAT_DAT_SEL_MASK) >> SEI_CTRL_POSITION_SMP_DAT_DAT_SEL_SHIFT)

/* Bitfield definition for register of struct array CTRL: SMP_POS */
/*
 * POS (RW)
 *
 * Sample override position
 */
#define SEI_CTRL_POSITION_SMP_POS_POS_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_POSITION_SMP_POS_POS_SHIFT (0U)
#define SEI_CTRL_POSITION_SMP_POS_POS_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_SMP_POS_POS_SHIFT) & SEI_CTRL_POSITION_SMP_POS_POS_MASK)
#define SEI_CTRL_POSITION_SMP_POS_POS_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_POS_POS_MASK) >> SEI_CTRL_POSITION_SMP_POS_POS_SHIFT)

/* Bitfield definition for register of struct array CTRL: SMP_REV */
/*
 * REV (RW)
 *
 * Sample override revolution
 */
#define SEI_CTRL_POSITION_SMP_REV_REV_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_POSITION_SMP_REV_REV_SHIFT (0U)
#define SEI_CTRL_POSITION_SMP_REV_REV_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_SMP_REV_REV_SHIFT) & SEI_CTRL_POSITION_SMP_REV_REV_MASK)
#define SEI_CTRL_POSITION_SMP_REV_REV_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_REV_REV_MASK) >> SEI_CTRL_POSITION_SMP_REV_REV_SHIFT)

/* Bitfield definition for register of struct array CTRL: SMP_SPD */
/*
 * SPD (RW)
 *
 * Sample override speed
 */
#define SEI_CTRL_POSITION_SMP_SPD_SPD_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_POSITION_SMP_SPD_SPD_SHIFT (0U)
#define SEI_CTRL_POSITION_SMP_SPD_SPD_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_SMP_SPD_SPD_SHIFT) & SEI_CTRL_POSITION_SMP_SPD_SPD_MASK)
#define SEI_CTRL_POSITION_SMP_SPD_SPD_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_SPD_SPD_MASK) >> SEI_CTRL_POSITION_SMP_SPD_SPD_SHIFT)

/* Bitfield definition for register of struct array CTRL: SMP_ACC */
/*
 * ACC (RW)
 *
 * Sample override accelerate
 */
#define SEI_CTRL_POSITION_SMP_ACC_ACC_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_POSITION_SMP_ACC_ACC_SHIFT (0U)
#define SEI_CTRL_POSITION_SMP_ACC_ACC_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_SMP_ACC_ACC_SHIFT) & SEI_CTRL_POSITION_SMP_ACC_ACC_MASK)
#define SEI_CTRL_POSITION_SMP_ACC_ACC_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_ACC_ACC_MASK) >> SEI_CTRL_POSITION_SMP_ACC_ACC_SHIFT)

/* Bitfield definition for register of struct array CTRL: UPD_EN */
/*
 * ACC_EN (RW)
 *
 * Position include acceleration
 */
#define SEI_CTRL_POSITION_UPD_EN_ACC_EN_MASK (0x80000000UL)
#define SEI_CTRL_POSITION_UPD_EN_ACC_EN_SHIFT (31U)
#define SEI_CTRL_POSITION_UPD_EN_ACC_EN_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_UPD_EN_ACC_EN_SHIFT) & SEI_CTRL_POSITION_UPD_EN_ACC_EN_MASK)
#define SEI_CTRL_POSITION_UPD_EN_ACC_EN_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_UPD_EN_ACC_EN_MASK) >> SEI_CTRL_POSITION_UPD_EN_ACC_EN_SHIFT)

/*
 * ACC_SEL (RW)
 *
 * Data register for acceleration transfer
 */
#define SEI_CTRL_POSITION_UPD_EN_ACC_SEL_MASK (0x1F000000UL)
#define SEI_CTRL_POSITION_UPD_EN_ACC_SEL_SHIFT (24U)
#define SEI_CTRL_POSITION_UPD_EN_ACC_SEL_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_UPD_EN_ACC_SEL_SHIFT) & SEI_CTRL_POSITION_UPD_EN_ACC_SEL_MASK)
#define SEI_CTRL_POSITION_UPD_EN_ACC_SEL_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_UPD_EN_ACC_SEL_MASK) >> SEI_CTRL_POSITION_UPD_EN_ACC_SEL_SHIFT)

/*
 * SPD_EN (RW)
 *
 * Position include speed
 */
#define SEI_CTRL_POSITION_UPD_EN_SPD_EN_MASK (0x800000UL)
#define SEI_CTRL_POSITION_UPD_EN_SPD_EN_SHIFT (23U)
#define SEI_CTRL_POSITION_UPD_EN_SPD_EN_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_UPD_EN_SPD_EN_SHIFT) & SEI_CTRL_POSITION_UPD_EN_SPD_EN_MASK)
#define SEI_CTRL_POSITION_UPD_EN_SPD_EN_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_UPD_EN_SPD_EN_MASK) >> SEI_CTRL_POSITION_UPD_EN_SPD_EN_SHIFT)

/*
 * SPD_SEL (RW)
 *
 * Data register for speed transfer
 */
#define SEI_CTRL_POSITION_UPD_EN_SPD_SEL_MASK (0x1F0000UL)
#define SEI_CTRL_POSITION_UPD_EN_SPD_SEL_SHIFT (16U)
#define SEI_CTRL_POSITION_UPD_EN_SPD_SEL_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_UPD_EN_SPD_SEL_SHIFT) & SEI_CTRL_POSITION_UPD_EN_SPD_SEL_MASK)
#define SEI_CTRL_POSITION_UPD_EN_SPD_SEL_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_UPD_EN_SPD_SEL_MASK) >> SEI_CTRL_POSITION_UPD_EN_SPD_SEL_SHIFT)

/*
 * REV_EN (RW)
 *
 * Position include revolution
 */
#define SEI_CTRL_POSITION_UPD_EN_REV_EN_MASK (0x8000U)
#define SEI_CTRL_POSITION_UPD_EN_REV_EN_SHIFT (15U)
#define SEI_CTRL_POSITION_UPD_EN_REV_EN_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_UPD_EN_REV_EN_SHIFT) & SEI_CTRL_POSITION_UPD_EN_REV_EN_MASK)
#define SEI_CTRL_POSITION_UPD_EN_REV_EN_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_UPD_EN_REV_EN_MASK) >> SEI_CTRL_POSITION_UPD_EN_REV_EN_SHIFT)

/*
 * REV_SEL (RW)
 *
 * Data register for revolution transfer
 */
#define SEI_CTRL_POSITION_UPD_EN_REV_SEL_MASK (0x1F00U)
#define SEI_CTRL_POSITION_UPD_EN_REV_SEL_SHIFT (8U)
#define SEI_CTRL_POSITION_UPD_EN_REV_SEL_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_UPD_EN_REV_SEL_SHIFT) & SEI_CTRL_POSITION_UPD_EN_REV_SEL_MASK)
#define SEI_CTRL_POSITION_UPD_EN_REV_SEL_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_UPD_EN_REV_SEL_MASK) >> SEI_CTRL_POSITION_UPD_EN_REV_SEL_SHIFT)

/*
 * POS_EN (RW)
 *
 * Position include position
 */
#define SEI_CTRL_POSITION_UPD_EN_POS_EN_MASK (0x80U)
#define SEI_CTRL_POSITION_UPD_EN_POS_EN_SHIFT (7U)
#define SEI_CTRL_POSITION_UPD_EN_POS_EN_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_UPD_EN_POS_EN_SHIFT) & SEI_CTRL_POSITION_UPD_EN_POS_EN_MASK)
#define SEI_CTRL_POSITION_UPD_EN_POS_EN_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_UPD_EN_POS_EN_MASK) >> SEI_CTRL_POSITION_UPD_EN_POS_EN_SHIFT)

/*
 * POS_SEL (RW)
 *
 * Data register for position transfer
 */
#define SEI_CTRL_POSITION_UPD_EN_POS_SEL_MASK (0x1FU)
#define SEI_CTRL_POSITION_UPD_EN_POS_SEL_SHIFT (0U)
#define SEI_CTRL_POSITION_UPD_EN_POS_SEL_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_UPD_EN_POS_SEL_SHIFT) & SEI_CTRL_POSITION_UPD_EN_POS_SEL_MASK)
#define SEI_CTRL_POSITION_UPD_EN_POS_SEL_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_UPD_EN_POS_SEL_MASK) >> SEI_CTRL_POSITION_UPD_EN_POS_SEL_SHIFT)

/* Bitfield definition for register of struct array CTRL: UPD_CFG */
/*
 * OVRD (RW)
 *
 * Use override value
 * 0: use received data
 * 1: use override data
 */
#define SEI_CTRL_POSITION_UPD_CFG_OVRD_MASK (0x80000000UL)
#define SEI_CTRL_POSITION_UPD_CFG_OVRD_SHIFT (31U)
#define SEI_CTRL_POSITION_UPD_CFG_OVRD_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_UPD_CFG_OVRD_SHIFT) & SEI_CTRL_POSITION_UPD_CFG_OVRD_MASK)
#define SEI_CTRL_POSITION_UPD_CFG_OVRD_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_UPD_CFG_OVRD_MASK) >> SEI_CTRL_POSITION_UPD_CFG_OVRD_SHIFT)

/*
 * ONERR (RW)
 *
 * Sample one time
 * 0: Sample during windows time
 * 1: Close sample window after first sample
 */
#define SEI_CTRL_POSITION_UPD_CFG_ONERR_MASK (0x1000000UL)
#define SEI_CTRL_POSITION_UPD_CFG_ONERR_SHIFT (24U)
#define SEI_CTRL_POSITION_UPD_CFG_ONERR_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_UPD_CFG_ONERR_SHIFT) & SEI_CTRL_POSITION_UPD_CFG_ONERR_MASK)
#define SEI_CTRL_POSITION_UPD_CFG_ONERR_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_UPD_CFG_ONERR_MASK) >> SEI_CTRL_POSITION_UPD_CFG_ONERR_SHIFT)

/*
 * LAT_SEL (RW)
 *
 * Latch selection
 * 0: latch 0
 * 1: latch 1
 * 2: latch 2
 * 3: latch 3
 */
#define SEI_CTRL_POSITION_UPD_CFG_LAT_SEL_MASK (0x30000UL)
#define SEI_CTRL_POSITION_UPD_CFG_LAT_SEL_SHIFT (16U)
#define SEI_CTRL_POSITION_UPD_CFG_LAT_SEL_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_UPD_CFG_LAT_SEL_SHIFT) & SEI_CTRL_POSITION_UPD_CFG_LAT_SEL_MASK)
#define SEI_CTRL_POSITION_UPD_CFG_LAT_SEL_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_UPD_CFG_LAT_SEL_MASK) >> SEI_CTRL_POSITION_UPD_CFG_LAT_SEL_SHIFT)

/* Bitfield definition for register of struct array CTRL: UPD_DAT */
/*
 * DAT_SEL (RW)
 *
 * Data register sampled, each bit represent a data register
 */
#define SEI_CTRL_POSITION_UPD_DAT_DAT_SEL_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_POSITION_UPD_DAT_DAT_SEL_SHIFT (0U)
#define SEI_CTRL_POSITION_UPD_DAT_DAT_SEL_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_UPD_DAT_DAT_SEL_SHIFT) & SEI_CTRL_POSITION_UPD_DAT_DAT_SEL_MASK)
#define SEI_CTRL_POSITION_UPD_DAT_DAT_SEL_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_UPD_DAT_DAT_SEL_MASK) >> SEI_CTRL_POSITION_UPD_DAT_DAT_SEL_SHIFT)

/* Bitfield definition for register of struct array CTRL: UPD_TIME */
/*
 * TIME (RW)
 *
 * Update override time
 */
#define SEI_CTRL_POSITION_UPD_TIME_TIME_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_POSITION_UPD_TIME_TIME_SHIFT (0U)
#define SEI_CTRL_POSITION_UPD_TIME_TIME_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_UPD_TIME_TIME_SHIFT) & SEI_CTRL_POSITION_UPD_TIME_TIME_MASK)
#define SEI_CTRL_POSITION_UPD_TIME_TIME_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_UPD_TIME_TIME_MASK) >> SEI_CTRL_POSITION_UPD_TIME_TIME_SHIFT)

/* Bitfield definition for register of struct array CTRL: UPD_POS */
/*
 * POS (RW)
 *
 * Update override position
 */
#define SEI_CTRL_POSITION_UPD_POS_POS_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_POSITION_UPD_POS_POS_SHIFT (0U)
#define SEI_CTRL_POSITION_UPD_POS_POS_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_UPD_POS_POS_SHIFT) & SEI_CTRL_POSITION_UPD_POS_POS_MASK)
#define SEI_CTRL_POSITION_UPD_POS_POS_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_UPD_POS_POS_MASK) >> SEI_CTRL_POSITION_UPD_POS_POS_SHIFT)

/* Bitfield definition for register of struct array CTRL: UPD_REV */
/*
 * REV (RW)
 *
 * Update override revolution
 */
#define SEI_CTRL_POSITION_UPD_REV_REV_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_POSITION_UPD_REV_REV_SHIFT (0U)
#define SEI_CTRL_POSITION_UPD_REV_REV_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_UPD_REV_REV_SHIFT) & SEI_CTRL_POSITION_UPD_REV_REV_MASK)
#define SEI_CTRL_POSITION_UPD_REV_REV_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_UPD_REV_REV_MASK) >> SEI_CTRL_POSITION_UPD_REV_REV_SHIFT)

/* Bitfield definition for register of struct array CTRL: UPD_SPD */
/*
 * SPD (RW)
 *
 * Update override speed
 */
#define SEI_CTRL_POSITION_UPD_SPD_SPD_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_POSITION_UPD_SPD_SPD_SHIFT (0U)
#define SEI_CTRL_POSITION_UPD_SPD_SPD_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_UPD_SPD_SPD_SHIFT) & SEI_CTRL_POSITION_UPD_SPD_SPD_MASK)
#define SEI_CTRL_POSITION_UPD_SPD_SPD_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_UPD_SPD_SPD_MASK) >> SEI_CTRL_POSITION_UPD_SPD_SPD_SHIFT)

/* Bitfield definition for register of struct array CTRL: UPD_ACC */
/*
 * ACC (RW)
 *
 * Update override accelerate
 */
#define SEI_CTRL_POSITION_UPD_ACC_ACC_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_POSITION_UPD_ACC_ACC_SHIFT (0U)
#define SEI_CTRL_POSITION_UPD_ACC_ACC_SET(x) (((uint32_t)(x) << SEI_CTRL_POSITION_UPD_ACC_ACC_SHIFT) & SEI_CTRL_POSITION_UPD_ACC_ACC_MASK)
#define SEI_CTRL_POSITION_UPD_ACC_ACC_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_UPD_ACC_ACC_MASK) >> SEI_CTRL_POSITION_UPD_ACC_ACC_SHIFT)

/* Bitfield definition for register of struct array CTRL: SMP_VAL */
/*
 * ACC (RO)
 *
 * Position include acceleration
 */
#define SEI_CTRL_POSITION_SMP_VAL_ACC_MASK (0x80000000UL)
#define SEI_CTRL_POSITION_SMP_VAL_ACC_SHIFT (31U)
#define SEI_CTRL_POSITION_SMP_VAL_ACC_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_VAL_ACC_MASK) >> SEI_CTRL_POSITION_SMP_VAL_ACC_SHIFT)

/*
 * SPD (RO)
 *
 * Position include speed
 */
#define SEI_CTRL_POSITION_SMP_VAL_SPD_MASK (0x800000UL)
#define SEI_CTRL_POSITION_SMP_VAL_SPD_SHIFT (23U)
#define SEI_CTRL_POSITION_SMP_VAL_SPD_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_VAL_SPD_MASK) >> SEI_CTRL_POSITION_SMP_VAL_SPD_SHIFT)

/*
 * REV (RO)
 *
 * Position include revolution
 */
#define SEI_CTRL_POSITION_SMP_VAL_REV_MASK (0x8000U)
#define SEI_CTRL_POSITION_SMP_VAL_REV_SHIFT (15U)
#define SEI_CTRL_POSITION_SMP_VAL_REV_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_VAL_REV_MASK) >> SEI_CTRL_POSITION_SMP_VAL_REV_SHIFT)

/*
 * POS (RO)
 *
 * Position include position
 */
#define SEI_CTRL_POSITION_SMP_VAL_POS_MASK (0x80U)
#define SEI_CTRL_POSITION_SMP_VAL_POS_SHIFT (7U)
#define SEI_CTRL_POSITION_SMP_VAL_POS_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_VAL_POS_MASK) >> SEI_CTRL_POSITION_SMP_VAL_POS_SHIFT)

/* Bitfield definition for register of struct array CTRL: SMP_STS */
/*
 * OCCUR (RO)
 *
 * Sample occured
 * 0: Sample not happened
 * 1: Sample occured
 */
#define SEI_CTRL_POSITION_SMP_STS_OCCUR_MASK (0x1000000UL)
#define SEI_CTRL_POSITION_SMP_STS_OCCUR_SHIFT (24U)
#define SEI_CTRL_POSITION_SMP_STS_OCCUR_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_STS_OCCUR_MASK) >> SEI_CTRL_POSITION_SMP_STS_OCCUR_SHIFT)

/*
 * WIN_CNT (RO)
 *
 * Sample window counter
 */
#define SEI_CTRL_POSITION_SMP_STS_WIN_CNT_MASK (0xFFFFU)
#define SEI_CTRL_POSITION_SMP_STS_WIN_CNT_SHIFT (0U)
#define SEI_CTRL_POSITION_SMP_STS_WIN_CNT_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SMP_STS_WIN_CNT_MASK) >> SEI_CTRL_POSITION_SMP_STS_WIN_CNT_SHIFT)

/* Bitfield definition for register of struct array CTRL: TIME_IN */
/*
 * TIME (RO)
 *
 * input time
 */
#define SEI_CTRL_POSITION_TIME_IN_TIME_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_POSITION_TIME_IN_TIME_SHIFT (0U)
#define SEI_CTRL_POSITION_TIME_IN_TIME_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_TIME_IN_TIME_MASK) >> SEI_CTRL_POSITION_TIME_IN_TIME_SHIFT)

/* Bitfield definition for register of struct array CTRL: POS_IN */
/*
 * POS (RO)
 *
 * Input position
 */
#define SEI_CTRL_POSITION_POS_IN_POS_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_POSITION_POS_IN_POS_SHIFT (0U)
#define SEI_CTRL_POSITION_POS_IN_POS_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_POS_IN_POS_MASK) >> SEI_CTRL_POSITION_POS_IN_POS_SHIFT)

/* Bitfield definition for register of struct array CTRL: REV_IN */
/*
 * REV (RO)
 *
 * Input revolution
 */
#define SEI_CTRL_POSITION_REV_IN_REV_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_POSITION_REV_IN_REV_SHIFT (0U)
#define SEI_CTRL_POSITION_REV_IN_REV_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_REV_IN_REV_MASK) >> SEI_CTRL_POSITION_REV_IN_REV_SHIFT)

/* Bitfield definition for register of struct array CTRL: SPD_IN */
/*
 * SPD (RO)
 *
 * Input speed
 */
#define SEI_CTRL_POSITION_SPD_IN_SPD_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_POSITION_SPD_IN_SPD_SHIFT (0U)
#define SEI_CTRL_POSITION_SPD_IN_SPD_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_SPD_IN_SPD_MASK) >> SEI_CTRL_POSITION_SPD_IN_SPD_SHIFT)

/* Bitfield definition for register of struct array CTRL: ACC_IN */
/*
 * ACC (RO)
 *
 * Input accelerate
 */
#define SEI_CTRL_POSITION_ACC_IN_ACC_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_POSITION_ACC_IN_ACC_SHIFT (0U)
#define SEI_CTRL_POSITION_ACC_IN_ACC_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_ACC_IN_ACC_MASK) >> SEI_CTRL_POSITION_ACC_IN_ACC_SHIFT)

/* Bitfield definition for register of struct array CTRL: UPD_STS */
/*
 * UPD_ERR (RO)
 *
 * Update  error
 * 0: data receive normally
 * 1: data receive error
 */
#define SEI_CTRL_POSITION_UPD_STS_UPD_ERR_MASK (0x1000000UL)
#define SEI_CTRL_POSITION_UPD_STS_UPD_ERR_SHIFT (24U)
#define SEI_CTRL_POSITION_UPD_STS_UPD_ERR_GET(x) (((uint32_t)(x) & SEI_CTRL_POSITION_UPD_STS_UPD_ERR_MASK) >> SEI_CTRL_POSITION_UPD_STS_UPD_ERR_SHIFT)

/* Bitfield definition for register of struct array CTRL: INT_EN */
/*
 * TRG_ERR3 (RW)
 *
 * Trigger3 failed
 */
#define SEI_CTRL_IRQ_INT_EN_TRG_ERR3_MASK (0x80000000UL)
#define SEI_CTRL_IRQ_INT_EN_TRG_ERR3_SHIFT (31U)
#define SEI_CTRL_IRQ_INT_EN_TRG_ERR3_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_TRG_ERR3_SHIFT) & SEI_CTRL_IRQ_INT_EN_TRG_ERR3_MASK)
#define SEI_CTRL_IRQ_INT_EN_TRG_ERR3_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_TRG_ERR3_MASK) >> SEI_CTRL_IRQ_INT_EN_TRG_ERR3_SHIFT)

/*
 * TRG_ERR2 (RW)
 *
 * Trigger2 failed
 */
#define SEI_CTRL_IRQ_INT_EN_TRG_ERR2_MASK (0x40000000UL)
#define SEI_CTRL_IRQ_INT_EN_TRG_ERR2_SHIFT (30U)
#define SEI_CTRL_IRQ_INT_EN_TRG_ERR2_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_TRG_ERR2_SHIFT) & SEI_CTRL_IRQ_INT_EN_TRG_ERR2_MASK)
#define SEI_CTRL_IRQ_INT_EN_TRG_ERR2_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_TRG_ERR2_MASK) >> SEI_CTRL_IRQ_INT_EN_TRG_ERR2_SHIFT)

/*
 * TRG_ERR1 (RW)
 *
 * Trigger1 failed
 */
#define SEI_CTRL_IRQ_INT_EN_TRG_ERR1_MASK (0x20000000UL)
#define SEI_CTRL_IRQ_INT_EN_TRG_ERR1_SHIFT (29U)
#define SEI_CTRL_IRQ_INT_EN_TRG_ERR1_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_TRG_ERR1_SHIFT) & SEI_CTRL_IRQ_INT_EN_TRG_ERR1_MASK)
#define SEI_CTRL_IRQ_INT_EN_TRG_ERR1_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_TRG_ERR1_MASK) >> SEI_CTRL_IRQ_INT_EN_TRG_ERR1_SHIFT)

/*
 * TRG_ERR0 (RW)
 *
 * Trigger0 failed
 */
#define SEI_CTRL_IRQ_INT_EN_TRG_ERR0_MASK (0x10000000UL)
#define SEI_CTRL_IRQ_INT_EN_TRG_ERR0_SHIFT (28U)
#define SEI_CTRL_IRQ_INT_EN_TRG_ERR0_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_TRG_ERR0_SHIFT) & SEI_CTRL_IRQ_INT_EN_TRG_ERR0_MASK)
#define SEI_CTRL_IRQ_INT_EN_TRG_ERR0_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_TRG_ERR0_MASK) >> SEI_CTRL_IRQ_INT_EN_TRG_ERR0_SHIFT)

/*
 * TRIGER3 (RW)
 *
 * Trigger3
 */
#define SEI_CTRL_IRQ_INT_EN_TRIGER3_MASK (0x8000000UL)
#define SEI_CTRL_IRQ_INT_EN_TRIGER3_SHIFT (27U)
#define SEI_CTRL_IRQ_INT_EN_TRIGER3_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_TRIGER3_SHIFT) & SEI_CTRL_IRQ_INT_EN_TRIGER3_MASK)
#define SEI_CTRL_IRQ_INT_EN_TRIGER3_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_TRIGER3_MASK) >> SEI_CTRL_IRQ_INT_EN_TRIGER3_SHIFT)

/*
 * TRIGER2 (RW)
 *
 * Trigger2
 */
#define SEI_CTRL_IRQ_INT_EN_TRIGER2_MASK (0x4000000UL)
#define SEI_CTRL_IRQ_INT_EN_TRIGER2_SHIFT (26U)
#define SEI_CTRL_IRQ_INT_EN_TRIGER2_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_TRIGER2_SHIFT) & SEI_CTRL_IRQ_INT_EN_TRIGER2_MASK)
#define SEI_CTRL_IRQ_INT_EN_TRIGER2_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_TRIGER2_MASK) >> SEI_CTRL_IRQ_INT_EN_TRIGER2_SHIFT)

/*
 * TRIGER1 (RW)
 *
 * Trigger1
 */
#define SEI_CTRL_IRQ_INT_EN_TRIGER1_MASK (0x2000000UL)
#define SEI_CTRL_IRQ_INT_EN_TRIGER1_SHIFT (25U)
#define SEI_CTRL_IRQ_INT_EN_TRIGER1_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_TRIGER1_SHIFT) & SEI_CTRL_IRQ_INT_EN_TRIGER1_MASK)
#define SEI_CTRL_IRQ_INT_EN_TRIGER1_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_TRIGER1_MASK) >> SEI_CTRL_IRQ_INT_EN_TRIGER1_SHIFT)

/*
 * TRIGER0 (RW)
 *
 * Trigger0
 */
#define SEI_CTRL_IRQ_INT_EN_TRIGER0_MASK (0x1000000UL)
#define SEI_CTRL_IRQ_INT_EN_TRIGER0_SHIFT (24U)
#define SEI_CTRL_IRQ_INT_EN_TRIGER0_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_TRIGER0_SHIFT) & SEI_CTRL_IRQ_INT_EN_TRIGER0_MASK)
#define SEI_CTRL_IRQ_INT_EN_TRIGER0_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_TRIGER0_MASK) >> SEI_CTRL_IRQ_INT_EN_TRIGER0_SHIFT)

/*
 * SMP_ERR (RW)
 *
 * Sample error
 */
#define SEI_CTRL_IRQ_INT_EN_SMP_ERR_MASK (0x100000UL)
#define SEI_CTRL_IRQ_INT_EN_SMP_ERR_SHIFT (20U)
#define SEI_CTRL_IRQ_INT_EN_SMP_ERR_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_SMP_ERR_SHIFT) & SEI_CTRL_IRQ_INT_EN_SMP_ERR_MASK)
#define SEI_CTRL_IRQ_INT_EN_SMP_ERR_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_SMP_ERR_MASK) >> SEI_CTRL_IRQ_INT_EN_SMP_ERR_SHIFT)

/*
 * LATCH3 (RW)
 *
 * Latch3
 */
#define SEI_CTRL_IRQ_INT_EN_LATCH3_MASK (0x80000UL)
#define SEI_CTRL_IRQ_INT_EN_LATCH3_SHIFT (19U)
#define SEI_CTRL_IRQ_INT_EN_LATCH3_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_LATCH3_SHIFT) & SEI_CTRL_IRQ_INT_EN_LATCH3_MASK)
#define SEI_CTRL_IRQ_INT_EN_LATCH3_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_LATCH3_MASK) >> SEI_CTRL_IRQ_INT_EN_LATCH3_SHIFT)

/*
 * LATCH2 (RW)
 *
 * Latch2
 */
#define SEI_CTRL_IRQ_INT_EN_LATCH2_MASK (0x40000UL)
#define SEI_CTRL_IRQ_INT_EN_LATCH2_SHIFT (18U)
#define SEI_CTRL_IRQ_INT_EN_LATCH2_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_LATCH2_SHIFT) & SEI_CTRL_IRQ_INT_EN_LATCH2_MASK)
#define SEI_CTRL_IRQ_INT_EN_LATCH2_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_LATCH2_MASK) >> SEI_CTRL_IRQ_INT_EN_LATCH2_SHIFT)

/*
 * LATCH1 (RW)
 *
 * Latch1
 */
#define SEI_CTRL_IRQ_INT_EN_LATCH1_MASK (0x20000UL)
#define SEI_CTRL_IRQ_INT_EN_LATCH1_SHIFT (17U)
#define SEI_CTRL_IRQ_INT_EN_LATCH1_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_LATCH1_SHIFT) & SEI_CTRL_IRQ_INT_EN_LATCH1_MASK)
#define SEI_CTRL_IRQ_INT_EN_LATCH1_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_LATCH1_MASK) >> SEI_CTRL_IRQ_INT_EN_LATCH1_SHIFT)

/*
 * LATCH0 (RW)
 *
 * Latch0
 */
#define SEI_CTRL_IRQ_INT_EN_LATCH0_MASK (0x10000UL)
#define SEI_CTRL_IRQ_INT_EN_LATCH0_SHIFT (16U)
#define SEI_CTRL_IRQ_INT_EN_LATCH0_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_LATCH0_SHIFT) & SEI_CTRL_IRQ_INT_EN_LATCH0_MASK)
#define SEI_CTRL_IRQ_INT_EN_LATCH0_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_LATCH0_MASK) >> SEI_CTRL_IRQ_INT_EN_LATCH0_SHIFT)

/*
 * TIMEOUT (RW)
 *
 * Timeout
 */
#define SEI_CTRL_IRQ_INT_EN_TIMEOUT_MASK (0x2000U)
#define SEI_CTRL_IRQ_INT_EN_TIMEOUT_SHIFT (13U)
#define SEI_CTRL_IRQ_INT_EN_TIMEOUT_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_TIMEOUT_SHIFT) & SEI_CTRL_IRQ_INT_EN_TIMEOUT_MASK)
#define SEI_CTRL_IRQ_INT_EN_TIMEOUT_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_TIMEOUT_MASK) >> SEI_CTRL_IRQ_INT_EN_TIMEOUT_SHIFT)

/*
 * TRX_ERR (RW)
 *
 * Transfer error
 */
#define SEI_CTRL_IRQ_INT_EN_TRX_ERR_MASK (0x1000U)
#define SEI_CTRL_IRQ_INT_EN_TRX_ERR_SHIFT (12U)
#define SEI_CTRL_IRQ_INT_EN_TRX_ERR_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_TRX_ERR_SHIFT) & SEI_CTRL_IRQ_INT_EN_TRX_ERR_MASK)
#define SEI_CTRL_IRQ_INT_EN_TRX_ERR_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_TRX_ERR_MASK) >> SEI_CTRL_IRQ_INT_EN_TRX_ERR_SHIFT)

/*
 * INSTR1_END (RW)
 *
 * Instruction 1 end
 */
#define SEI_CTRL_IRQ_INT_EN_INSTR1_END_MASK (0x800U)
#define SEI_CTRL_IRQ_INT_EN_INSTR1_END_SHIFT (11U)
#define SEI_CTRL_IRQ_INT_EN_INSTR1_END_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_INSTR1_END_SHIFT) & SEI_CTRL_IRQ_INT_EN_INSTR1_END_MASK)
#define SEI_CTRL_IRQ_INT_EN_INSTR1_END_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_INSTR1_END_MASK) >> SEI_CTRL_IRQ_INT_EN_INSTR1_END_SHIFT)

/*
 * INSTR0_END (RW)
 *
 * Instruction 0 end
 */
#define SEI_CTRL_IRQ_INT_EN_INSTR0_END_MASK (0x400U)
#define SEI_CTRL_IRQ_INT_EN_INSTR0_END_SHIFT (10U)
#define SEI_CTRL_IRQ_INT_EN_INSTR0_END_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_INSTR0_END_SHIFT) & SEI_CTRL_IRQ_INT_EN_INSTR0_END_MASK)
#define SEI_CTRL_IRQ_INT_EN_INSTR0_END_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_INSTR0_END_MASK) >> SEI_CTRL_IRQ_INT_EN_INSTR0_END_SHIFT)

/*
 * PTR1_END (RW)
 *
 * Pointer 1 end
 */
#define SEI_CTRL_IRQ_INT_EN_PTR1_END_MASK (0x200U)
#define SEI_CTRL_IRQ_INT_EN_PTR1_END_SHIFT (9U)
#define SEI_CTRL_IRQ_INT_EN_PTR1_END_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_PTR1_END_SHIFT) & SEI_CTRL_IRQ_INT_EN_PTR1_END_MASK)
#define SEI_CTRL_IRQ_INT_EN_PTR1_END_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_PTR1_END_MASK) >> SEI_CTRL_IRQ_INT_EN_PTR1_END_SHIFT)

/*
 * PTR0_END (RW)
 *
 * Pointer 0 end
 */
#define SEI_CTRL_IRQ_INT_EN_PTR0_END_MASK (0x100U)
#define SEI_CTRL_IRQ_INT_EN_PTR0_END_SHIFT (8U)
#define SEI_CTRL_IRQ_INT_EN_PTR0_END_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_PTR0_END_SHIFT) & SEI_CTRL_IRQ_INT_EN_PTR0_END_MASK)
#define SEI_CTRL_IRQ_INT_EN_PTR0_END_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_PTR0_END_MASK) >> SEI_CTRL_IRQ_INT_EN_PTR0_END_SHIFT)

/*
 * INSTR1_ST (RW)
 *
 * Instruction 1 start
 */
#define SEI_CTRL_IRQ_INT_EN_INSTR1_ST_MASK (0x80U)
#define SEI_CTRL_IRQ_INT_EN_INSTR1_ST_SHIFT (7U)
#define SEI_CTRL_IRQ_INT_EN_INSTR1_ST_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_INSTR1_ST_SHIFT) & SEI_CTRL_IRQ_INT_EN_INSTR1_ST_MASK)
#define SEI_CTRL_IRQ_INT_EN_INSTR1_ST_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_INSTR1_ST_MASK) >> SEI_CTRL_IRQ_INT_EN_INSTR1_ST_SHIFT)

/*
 * INSTR0_ST (RW)
 *
 * Instruction 0 start
 */
#define SEI_CTRL_IRQ_INT_EN_INSTR0_ST_MASK (0x40U)
#define SEI_CTRL_IRQ_INT_EN_INSTR0_ST_SHIFT (6U)
#define SEI_CTRL_IRQ_INT_EN_INSTR0_ST_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_INSTR0_ST_SHIFT) & SEI_CTRL_IRQ_INT_EN_INSTR0_ST_MASK)
#define SEI_CTRL_IRQ_INT_EN_INSTR0_ST_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_INSTR0_ST_MASK) >> SEI_CTRL_IRQ_INT_EN_INSTR0_ST_SHIFT)

/*
 * PTR1_ST (RW)
 *
 * Pointer 1 end
 */
#define SEI_CTRL_IRQ_INT_EN_PTR1_ST_MASK (0x20U)
#define SEI_CTRL_IRQ_INT_EN_PTR1_ST_SHIFT (5U)
#define SEI_CTRL_IRQ_INT_EN_PTR1_ST_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_PTR1_ST_SHIFT) & SEI_CTRL_IRQ_INT_EN_PTR1_ST_MASK)
#define SEI_CTRL_IRQ_INT_EN_PTR1_ST_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_PTR1_ST_MASK) >> SEI_CTRL_IRQ_INT_EN_PTR1_ST_SHIFT)

/*
 * PTR0_ST (RW)
 *
 * Pointer 0 end
 */
#define SEI_CTRL_IRQ_INT_EN_PTR0_ST_MASK (0x10U)
#define SEI_CTRL_IRQ_INT_EN_PTR0_ST_SHIFT (4U)
#define SEI_CTRL_IRQ_INT_EN_PTR0_ST_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_PTR0_ST_SHIFT) & SEI_CTRL_IRQ_INT_EN_PTR0_ST_MASK)
#define SEI_CTRL_IRQ_INT_EN_PTR0_ST_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_PTR0_ST_MASK) >> SEI_CTRL_IRQ_INT_EN_PTR0_ST_SHIFT)

/*
 * WDOG (RW)
 *
 * Watch dog
 */
#define SEI_CTRL_IRQ_INT_EN_WDOG_MASK (0x4U)
#define SEI_CTRL_IRQ_INT_EN_WDOG_SHIFT (2U)
#define SEI_CTRL_IRQ_INT_EN_WDOG_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_WDOG_SHIFT) & SEI_CTRL_IRQ_INT_EN_WDOG_MASK)
#define SEI_CTRL_IRQ_INT_EN_WDOG_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_WDOG_MASK) >> SEI_CTRL_IRQ_INT_EN_WDOG_SHIFT)

/*
 * EXECPT (RW)
 *
 * Exception
 */
#define SEI_CTRL_IRQ_INT_EN_EXECPT_MASK (0x2U)
#define SEI_CTRL_IRQ_INT_EN_EXECPT_SHIFT (1U)
#define SEI_CTRL_IRQ_INT_EN_EXECPT_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_EXECPT_SHIFT) & SEI_CTRL_IRQ_INT_EN_EXECPT_MASK)
#define SEI_CTRL_IRQ_INT_EN_EXECPT_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_EXECPT_MASK) >> SEI_CTRL_IRQ_INT_EN_EXECPT_SHIFT)

/*
 * STALL (RW)
 *
 * Stall
 */
#define SEI_CTRL_IRQ_INT_EN_STALL_MASK (0x1U)
#define SEI_CTRL_IRQ_INT_EN_STALL_SHIFT (0U)
#define SEI_CTRL_IRQ_INT_EN_STALL_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_EN_STALL_SHIFT) & SEI_CTRL_IRQ_INT_EN_STALL_MASK)
#define SEI_CTRL_IRQ_INT_EN_STALL_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_EN_STALL_MASK) >> SEI_CTRL_IRQ_INT_EN_STALL_SHIFT)

/* Bitfield definition for register of struct array CTRL: INT_FLAG */
/*
 * TRG_ERR3 (W1C)
 *
 * Trigger3 failed
 */
#define SEI_CTRL_IRQ_INT_FLAG_TRG_ERR3_MASK (0x80000000UL)
#define SEI_CTRL_IRQ_INT_FLAG_TRG_ERR3_SHIFT (31U)
#define SEI_CTRL_IRQ_INT_FLAG_TRG_ERR3_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_TRG_ERR3_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_TRG_ERR3_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_TRG_ERR3_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_TRG_ERR3_MASK) >> SEI_CTRL_IRQ_INT_FLAG_TRG_ERR3_SHIFT)

/*
 * TRG_ERR2 (W1C)
 *
 * Trigger2 failed
 */
#define SEI_CTRL_IRQ_INT_FLAG_TRG_ERR2_MASK (0x40000000UL)
#define SEI_CTRL_IRQ_INT_FLAG_TRG_ERR2_SHIFT (30U)
#define SEI_CTRL_IRQ_INT_FLAG_TRG_ERR2_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_TRG_ERR2_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_TRG_ERR2_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_TRG_ERR2_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_TRG_ERR2_MASK) >> SEI_CTRL_IRQ_INT_FLAG_TRG_ERR2_SHIFT)

/*
 * TRG_ERR1 (W1C)
 *
 * Trigger1 failed
 */
#define SEI_CTRL_IRQ_INT_FLAG_TRG_ERR1_MASK (0x20000000UL)
#define SEI_CTRL_IRQ_INT_FLAG_TRG_ERR1_SHIFT (29U)
#define SEI_CTRL_IRQ_INT_FLAG_TRG_ERR1_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_TRG_ERR1_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_TRG_ERR1_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_TRG_ERR1_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_TRG_ERR1_MASK) >> SEI_CTRL_IRQ_INT_FLAG_TRG_ERR1_SHIFT)

/*
 * TRG_ERR0 (W1C)
 *
 * Trigger0 failed
 */
#define SEI_CTRL_IRQ_INT_FLAG_TRG_ERR0_MASK (0x10000000UL)
#define SEI_CTRL_IRQ_INT_FLAG_TRG_ERR0_SHIFT (28U)
#define SEI_CTRL_IRQ_INT_FLAG_TRG_ERR0_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_TRG_ERR0_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_TRG_ERR0_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_TRG_ERR0_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_TRG_ERR0_MASK) >> SEI_CTRL_IRQ_INT_FLAG_TRG_ERR0_SHIFT)

/*
 * TRIGER3 (W1C)
 *
 * Trigger3
 */
#define SEI_CTRL_IRQ_INT_FLAG_TRIGER3_MASK (0x8000000UL)
#define SEI_CTRL_IRQ_INT_FLAG_TRIGER3_SHIFT (27U)
#define SEI_CTRL_IRQ_INT_FLAG_TRIGER3_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_TRIGER3_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_TRIGER3_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_TRIGER3_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_TRIGER3_MASK) >> SEI_CTRL_IRQ_INT_FLAG_TRIGER3_SHIFT)

/*
 * TRIGER2 (W1C)
 *
 * Trigger2
 */
#define SEI_CTRL_IRQ_INT_FLAG_TRIGER2_MASK (0x4000000UL)
#define SEI_CTRL_IRQ_INT_FLAG_TRIGER2_SHIFT (26U)
#define SEI_CTRL_IRQ_INT_FLAG_TRIGER2_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_TRIGER2_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_TRIGER2_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_TRIGER2_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_TRIGER2_MASK) >> SEI_CTRL_IRQ_INT_FLAG_TRIGER2_SHIFT)

/*
 * TRIGER1 (W1C)
 *
 * Trigger1
 */
#define SEI_CTRL_IRQ_INT_FLAG_TRIGER1_MASK (0x2000000UL)
#define SEI_CTRL_IRQ_INT_FLAG_TRIGER1_SHIFT (25U)
#define SEI_CTRL_IRQ_INT_FLAG_TRIGER1_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_TRIGER1_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_TRIGER1_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_TRIGER1_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_TRIGER1_MASK) >> SEI_CTRL_IRQ_INT_FLAG_TRIGER1_SHIFT)

/*
 * TRIGER0 (W1C)
 *
 * Trigger0
 */
#define SEI_CTRL_IRQ_INT_FLAG_TRIGER0_MASK (0x1000000UL)
#define SEI_CTRL_IRQ_INT_FLAG_TRIGER0_SHIFT (24U)
#define SEI_CTRL_IRQ_INT_FLAG_TRIGER0_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_TRIGER0_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_TRIGER0_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_TRIGER0_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_TRIGER0_MASK) >> SEI_CTRL_IRQ_INT_FLAG_TRIGER0_SHIFT)

/*
 * SMP_ERR (W1C)
 *
 * Sample error
 */
#define SEI_CTRL_IRQ_INT_FLAG_SMP_ERR_MASK (0x100000UL)
#define SEI_CTRL_IRQ_INT_FLAG_SMP_ERR_SHIFT (20U)
#define SEI_CTRL_IRQ_INT_FLAG_SMP_ERR_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_SMP_ERR_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_SMP_ERR_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_SMP_ERR_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_SMP_ERR_MASK) >> SEI_CTRL_IRQ_INT_FLAG_SMP_ERR_SHIFT)

/*
 * LATCH3 (W1C)
 *
 * Latch3
 */
#define SEI_CTRL_IRQ_INT_FLAG_LATCH3_MASK (0x80000UL)
#define SEI_CTRL_IRQ_INT_FLAG_LATCH3_SHIFT (19U)
#define SEI_CTRL_IRQ_INT_FLAG_LATCH3_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_LATCH3_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_LATCH3_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_LATCH3_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_LATCH3_MASK) >> SEI_CTRL_IRQ_INT_FLAG_LATCH3_SHIFT)

/*
 * LATCH2 (W1C)
 *
 * Latch2
 */
#define SEI_CTRL_IRQ_INT_FLAG_LATCH2_MASK (0x40000UL)
#define SEI_CTRL_IRQ_INT_FLAG_LATCH2_SHIFT (18U)
#define SEI_CTRL_IRQ_INT_FLAG_LATCH2_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_LATCH2_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_LATCH2_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_LATCH2_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_LATCH2_MASK) >> SEI_CTRL_IRQ_INT_FLAG_LATCH2_SHIFT)

/*
 * LATCH1 (W1C)
 *
 * Latch1
 */
#define SEI_CTRL_IRQ_INT_FLAG_LATCH1_MASK (0x20000UL)
#define SEI_CTRL_IRQ_INT_FLAG_LATCH1_SHIFT (17U)
#define SEI_CTRL_IRQ_INT_FLAG_LATCH1_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_LATCH1_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_LATCH1_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_LATCH1_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_LATCH1_MASK) >> SEI_CTRL_IRQ_INT_FLAG_LATCH1_SHIFT)

/*
 * LATCH0 (W1C)
 *
 * Latch0
 */
#define SEI_CTRL_IRQ_INT_FLAG_LATCH0_MASK (0x10000UL)
#define SEI_CTRL_IRQ_INT_FLAG_LATCH0_SHIFT (16U)
#define SEI_CTRL_IRQ_INT_FLAG_LATCH0_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_LATCH0_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_LATCH0_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_LATCH0_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_LATCH0_MASK) >> SEI_CTRL_IRQ_INT_FLAG_LATCH0_SHIFT)

/*
 * TIMEOUT (W1C)
 *
 * Timeout
 */
#define SEI_CTRL_IRQ_INT_FLAG_TIMEOUT_MASK (0x2000U)
#define SEI_CTRL_IRQ_INT_FLAG_TIMEOUT_SHIFT (13U)
#define SEI_CTRL_IRQ_INT_FLAG_TIMEOUT_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_TIMEOUT_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_TIMEOUT_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_TIMEOUT_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_TIMEOUT_MASK) >> SEI_CTRL_IRQ_INT_FLAG_TIMEOUT_SHIFT)

/*
 * TRX_ERR (W1C)
 *
 * Transfer error
 */
#define SEI_CTRL_IRQ_INT_FLAG_TRX_ERR_MASK (0x1000U)
#define SEI_CTRL_IRQ_INT_FLAG_TRX_ERR_SHIFT (12U)
#define SEI_CTRL_IRQ_INT_FLAG_TRX_ERR_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_TRX_ERR_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_TRX_ERR_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_TRX_ERR_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_TRX_ERR_MASK) >> SEI_CTRL_IRQ_INT_FLAG_TRX_ERR_SHIFT)

/*
 * INSTR1_END (W1C)
 *
 * Instruction 1 end
 */
#define SEI_CTRL_IRQ_INT_FLAG_INSTR1_END_MASK (0x800U)
#define SEI_CTRL_IRQ_INT_FLAG_INSTR1_END_SHIFT (11U)
#define SEI_CTRL_IRQ_INT_FLAG_INSTR1_END_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_INSTR1_END_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_INSTR1_END_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_INSTR1_END_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_INSTR1_END_MASK) >> SEI_CTRL_IRQ_INT_FLAG_INSTR1_END_SHIFT)

/*
 * INSTR0_END (W1C)
 *
 * Instruction 0 end
 */
#define SEI_CTRL_IRQ_INT_FLAG_INSTR0_END_MASK (0x400U)
#define SEI_CTRL_IRQ_INT_FLAG_INSTR0_END_SHIFT (10U)
#define SEI_CTRL_IRQ_INT_FLAG_INSTR0_END_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_INSTR0_END_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_INSTR0_END_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_INSTR0_END_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_INSTR0_END_MASK) >> SEI_CTRL_IRQ_INT_FLAG_INSTR0_END_SHIFT)

/*
 * PTR1_END (W1C)
 *
 * Pointer 1 end
 */
#define SEI_CTRL_IRQ_INT_FLAG_PTR1_END_MASK (0x200U)
#define SEI_CTRL_IRQ_INT_FLAG_PTR1_END_SHIFT (9U)
#define SEI_CTRL_IRQ_INT_FLAG_PTR1_END_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_PTR1_END_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_PTR1_END_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_PTR1_END_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_PTR1_END_MASK) >> SEI_CTRL_IRQ_INT_FLAG_PTR1_END_SHIFT)

/*
 * PTR0_END (W1C)
 *
 * Pointer 0 end
 */
#define SEI_CTRL_IRQ_INT_FLAG_PTR0_END_MASK (0x100U)
#define SEI_CTRL_IRQ_INT_FLAG_PTR0_END_SHIFT (8U)
#define SEI_CTRL_IRQ_INT_FLAG_PTR0_END_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_PTR0_END_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_PTR0_END_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_PTR0_END_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_PTR0_END_MASK) >> SEI_CTRL_IRQ_INT_FLAG_PTR0_END_SHIFT)

/*
 * INSTR1_ST (W1C)
 *
 * Instruction 1 start
 */
#define SEI_CTRL_IRQ_INT_FLAG_INSTR1_ST_MASK (0x80U)
#define SEI_CTRL_IRQ_INT_FLAG_INSTR1_ST_SHIFT (7U)
#define SEI_CTRL_IRQ_INT_FLAG_INSTR1_ST_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_INSTR1_ST_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_INSTR1_ST_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_INSTR1_ST_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_INSTR1_ST_MASK) >> SEI_CTRL_IRQ_INT_FLAG_INSTR1_ST_SHIFT)

/*
 * INSTR0_ST (W1C)
 *
 * Instruction 0 start
 */
#define SEI_CTRL_IRQ_INT_FLAG_INSTR0_ST_MASK (0x40U)
#define SEI_CTRL_IRQ_INT_FLAG_INSTR0_ST_SHIFT (6U)
#define SEI_CTRL_IRQ_INT_FLAG_INSTR0_ST_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_INSTR0_ST_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_INSTR0_ST_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_INSTR0_ST_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_INSTR0_ST_MASK) >> SEI_CTRL_IRQ_INT_FLAG_INSTR0_ST_SHIFT)

/*
 * PTR1_ST (W1C)
 *
 * Pointer 1 end
 */
#define SEI_CTRL_IRQ_INT_FLAG_PTR1_ST_MASK (0x20U)
#define SEI_CTRL_IRQ_INT_FLAG_PTR1_ST_SHIFT (5U)
#define SEI_CTRL_IRQ_INT_FLAG_PTR1_ST_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_PTR1_ST_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_PTR1_ST_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_PTR1_ST_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_PTR1_ST_MASK) >> SEI_CTRL_IRQ_INT_FLAG_PTR1_ST_SHIFT)

/*
 * PTR0_ST (W1C)
 *
 * Pointer 0 end
 */
#define SEI_CTRL_IRQ_INT_FLAG_PTR0_ST_MASK (0x10U)
#define SEI_CTRL_IRQ_INT_FLAG_PTR0_ST_SHIFT (4U)
#define SEI_CTRL_IRQ_INT_FLAG_PTR0_ST_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_PTR0_ST_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_PTR0_ST_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_PTR0_ST_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_PTR0_ST_MASK) >> SEI_CTRL_IRQ_INT_FLAG_PTR0_ST_SHIFT)

/*
 * WDOG (W1C)
 *
 * Watch dog
 */
#define SEI_CTRL_IRQ_INT_FLAG_WDOG_MASK (0x4U)
#define SEI_CTRL_IRQ_INT_FLAG_WDOG_SHIFT (2U)
#define SEI_CTRL_IRQ_INT_FLAG_WDOG_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_WDOG_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_WDOG_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_WDOG_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_WDOG_MASK) >> SEI_CTRL_IRQ_INT_FLAG_WDOG_SHIFT)

/*
 * EXECPT (W1C)
 *
 * Exception
 */
#define SEI_CTRL_IRQ_INT_FLAG_EXECPT_MASK (0x2U)
#define SEI_CTRL_IRQ_INT_FLAG_EXECPT_SHIFT (1U)
#define SEI_CTRL_IRQ_INT_FLAG_EXECPT_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_EXECPT_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_EXECPT_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_EXECPT_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_EXECPT_MASK) >> SEI_CTRL_IRQ_INT_FLAG_EXECPT_SHIFT)

/*
 * STALL (W1C)
 *
 * Stall
 */
#define SEI_CTRL_IRQ_INT_FLAG_STALL_MASK (0x1U)
#define SEI_CTRL_IRQ_INT_FLAG_STALL_SHIFT (0U)
#define SEI_CTRL_IRQ_INT_FLAG_STALL_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INT_FLAG_STALL_SHIFT) & SEI_CTRL_IRQ_INT_FLAG_STALL_MASK)
#define SEI_CTRL_IRQ_INT_FLAG_STALL_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_FLAG_STALL_MASK) >> SEI_CTRL_IRQ_INT_FLAG_STALL_SHIFT)

/* Bitfield definition for register of struct array CTRL: INT_STS */
/*
 * TRG_ERR3 (RO)
 *
 * Trigger3 failed
 */
#define SEI_CTRL_IRQ_INT_STS_TRG_ERR3_MASK (0x80000000UL)
#define SEI_CTRL_IRQ_INT_STS_TRG_ERR3_SHIFT (31U)
#define SEI_CTRL_IRQ_INT_STS_TRG_ERR3_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_TRG_ERR3_MASK) >> SEI_CTRL_IRQ_INT_STS_TRG_ERR3_SHIFT)

/*
 * TRG_ERR2 (RO)
 *
 * Trigger2 failed
 */
#define SEI_CTRL_IRQ_INT_STS_TRG_ERR2_MASK (0x40000000UL)
#define SEI_CTRL_IRQ_INT_STS_TRG_ERR2_SHIFT (30U)
#define SEI_CTRL_IRQ_INT_STS_TRG_ERR2_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_TRG_ERR2_MASK) >> SEI_CTRL_IRQ_INT_STS_TRG_ERR2_SHIFT)

/*
 * TRG_ERR1 (RO)
 *
 * Trigger1 failed
 */
#define SEI_CTRL_IRQ_INT_STS_TRG_ERR1_MASK (0x20000000UL)
#define SEI_CTRL_IRQ_INT_STS_TRG_ERR1_SHIFT (29U)
#define SEI_CTRL_IRQ_INT_STS_TRG_ERR1_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_TRG_ERR1_MASK) >> SEI_CTRL_IRQ_INT_STS_TRG_ERR1_SHIFT)

/*
 * TRG_ERR0 (RO)
 *
 * Trigger0 failed
 */
#define SEI_CTRL_IRQ_INT_STS_TRG_ERR0_MASK (0x10000000UL)
#define SEI_CTRL_IRQ_INT_STS_TRG_ERR0_SHIFT (28U)
#define SEI_CTRL_IRQ_INT_STS_TRG_ERR0_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_TRG_ERR0_MASK) >> SEI_CTRL_IRQ_INT_STS_TRG_ERR0_SHIFT)

/*
 * TRIGER3 (RO)
 *
 * Trigger3
 */
#define SEI_CTRL_IRQ_INT_STS_TRIGER3_MASK (0x8000000UL)
#define SEI_CTRL_IRQ_INT_STS_TRIGER3_SHIFT (27U)
#define SEI_CTRL_IRQ_INT_STS_TRIGER3_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_TRIGER3_MASK) >> SEI_CTRL_IRQ_INT_STS_TRIGER3_SHIFT)

/*
 * TRIGER2 (RO)
 *
 * Trigger2
 */
#define SEI_CTRL_IRQ_INT_STS_TRIGER2_MASK (0x4000000UL)
#define SEI_CTRL_IRQ_INT_STS_TRIGER2_SHIFT (26U)
#define SEI_CTRL_IRQ_INT_STS_TRIGER2_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_TRIGER2_MASK) >> SEI_CTRL_IRQ_INT_STS_TRIGER2_SHIFT)

/*
 * TRIGER1 (RO)
 *
 * Trigger1
 */
#define SEI_CTRL_IRQ_INT_STS_TRIGER1_MASK (0x2000000UL)
#define SEI_CTRL_IRQ_INT_STS_TRIGER1_SHIFT (25U)
#define SEI_CTRL_IRQ_INT_STS_TRIGER1_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_TRIGER1_MASK) >> SEI_CTRL_IRQ_INT_STS_TRIGER1_SHIFT)

/*
 * TRIGER0 (RO)
 *
 * Trigger0
 */
#define SEI_CTRL_IRQ_INT_STS_TRIGER0_MASK (0x1000000UL)
#define SEI_CTRL_IRQ_INT_STS_TRIGER0_SHIFT (24U)
#define SEI_CTRL_IRQ_INT_STS_TRIGER0_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_TRIGER0_MASK) >> SEI_CTRL_IRQ_INT_STS_TRIGER0_SHIFT)

/*
 * SMP_ERR (RO)
 *
 * Sample error
 */
#define SEI_CTRL_IRQ_INT_STS_SMP_ERR_MASK (0x100000UL)
#define SEI_CTRL_IRQ_INT_STS_SMP_ERR_SHIFT (20U)
#define SEI_CTRL_IRQ_INT_STS_SMP_ERR_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_SMP_ERR_MASK) >> SEI_CTRL_IRQ_INT_STS_SMP_ERR_SHIFT)

/*
 * LATCH3 (RO)
 *
 * Latch3
 */
#define SEI_CTRL_IRQ_INT_STS_LATCH3_MASK (0x80000UL)
#define SEI_CTRL_IRQ_INT_STS_LATCH3_SHIFT (19U)
#define SEI_CTRL_IRQ_INT_STS_LATCH3_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_LATCH3_MASK) >> SEI_CTRL_IRQ_INT_STS_LATCH3_SHIFT)

/*
 * LATCH2 (RO)
 *
 * Latch2
 */
#define SEI_CTRL_IRQ_INT_STS_LATCH2_MASK (0x40000UL)
#define SEI_CTRL_IRQ_INT_STS_LATCH2_SHIFT (18U)
#define SEI_CTRL_IRQ_INT_STS_LATCH2_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_LATCH2_MASK) >> SEI_CTRL_IRQ_INT_STS_LATCH2_SHIFT)

/*
 * LATCH1 (RO)
 *
 * Latch1
 */
#define SEI_CTRL_IRQ_INT_STS_LATCH1_MASK (0x20000UL)
#define SEI_CTRL_IRQ_INT_STS_LATCH1_SHIFT (17U)
#define SEI_CTRL_IRQ_INT_STS_LATCH1_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_LATCH1_MASK) >> SEI_CTRL_IRQ_INT_STS_LATCH1_SHIFT)

/*
 * LATCH0 (RO)
 *
 * Latch0
 */
#define SEI_CTRL_IRQ_INT_STS_LATCH0_MASK (0x10000UL)
#define SEI_CTRL_IRQ_INT_STS_LATCH0_SHIFT (16U)
#define SEI_CTRL_IRQ_INT_STS_LATCH0_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_LATCH0_MASK) >> SEI_CTRL_IRQ_INT_STS_LATCH0_SHIFT)

/*
 * TIMEOUT (RO)
 *
 * Timeout
 */
#define SEI_CTRL_IRQ_INT_STS_TIMEOUT_MASK (0x2000U)
#define SEI_CTRL_IRQ_INT_STS_TIMEOUT_SHIFT (13U)
#define SEI_CTRL_IRQ_INT_STS_TIMEOUT_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_TIMEOUT_MASK) >> SEI_CTRL_IRQ_INT_STS_TIMEOUT_SHIFT)

/*
 * TRX_ERR (RO)
 *
 * Transfer error
 */
#define SEI_CTRL_IRQ_INT_STS_TRX_ERR_MASK (0x1000U)
#define SEI_CTRL_IRQ_INT_STS_TRX_ERR_SHIFT (12U)
#define SEI_CTRL_IRQ_INT_STS_TRX_ERR_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_TRX_ERR_MASK) >> SEI_CTRL_IRQ_INT_STS_TRX_ERR_SHIFT)

/*
 * INSTR1_END (RO)
 *
 * Instruction 1 end
 */
#define SEI_CTRL_IRQ_INT_STS_INSTR1_END_MASK (0x800U)
#define SEI_CTRL_IRQ_INT_STS_INSTR1_END_SHIFT (11U)
#define SEI_CTRL_IRQ_INT_STS_INSTR1_END_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_INSTR1_END_MASK) >> SEI_CTRL_IRQ_INT_STS_INSTR1_END_SHIFT)

/*
 * INSTR0_END (RO)
 *
 * Instruction 0 end
 */
#define SEI_CTRL_IRQ_INT_STS_INSTR0_END_MASK (0x400U)
#define SEI_CTRL_IRQ_INT_STS_INSTR0_END_SHIFT (10U)
#define SEI_CTRL_IRQ_INT_STS_INSTR0_END_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_INSTR0_END_MASK) >> SEI_CTRL_IRQ_INT_STS_INSTR0_END_SHIFT)

/*
 * PTR1_END (RO)
 *
 * Pointer 1 end
 */
#define SEI_CTRL_IRQ_INT_STS_PTR1_END_MASK (0x200U)
#define SEI_CTRL_IRQ_INT_STS_PTR1_END_SHIFT (9U)
#define SEI_CTRL_IRQ_INT_STS_PTR1_END_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_PTR1_END_MASK) >> SEI_CTRL_IRQ_INT_STS_PTR1_END_SHIFT)

/*
 * PTR0_END (RO)
 *
 * Pointer 0 end
 */
#define SEI_CTRL_IRQ_INT_STS_PTR0_END_MASK (0x100U)
#define SEI_CTRL_IRQ_INT_STS_PTR0_END_SHIFT (8U)
#define SEI_CTRL_IRQ_INT_STS_PTR0_END_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_PTR0_END_MASK) >> SEI_CTRL_IRQ_INT_STS_PTR0_END_SHIFT)

/*
 * INSTR1_ST (RO)
 *
 * Instruction 1 start
 */
#define SEI_CTRL_IRQ_INT_STS_INSTR1_ST_MASK (0x80U)
#define SEI_CTRL_IRQ_INT_STS_INSTR1_ST_SHIFT (7U)
#define SEI_CTRL_IRQ_INT_STS_INSTR1_ST_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_INSTR1_ST_MASK) >> SEI_CTRL_IRQ_INT_STS_INSTR1_ST_SHIFT)

/*
 * INSTR0_ST (RO)
 *
 * Instruction 0 start
 */
#define SEI_CTRL_IRQ_INT_STS_INSTR0_ST_MASK (0x40U)
#define SEI_CTRL_IRQ_INT_STS_INSTR0_ST_SHIFT (6U)
#define SEI_CTRL_IRQ_INT_STS_INSTR0_ST_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_INSTR0_ST_MASK) >> SEI_CTRL_IRQ_INT_STS_INSTR0_ST_SHIFT)

/*
 * PTR1_ST (RO)
 *
 * Pointer 1 end
 */
#define SEI_CTRL_IRQ_INT_STS_PTR1_ST_MASK (0x20U)
#define SEI_CTRL_IRQ_INT_STS_PTR1_ST_SHIFT (5U)
#define SEI_CTRL_IRQ_INT_STS_PTR1_ST_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_PTR1_ST_MASK) >> SEI_CTRL_IRQ_INT_STS_PTR1_ST_SHIFT)

/*
 * PTR0_ST (RO)
 *
 * Pointer 0 end
 */
#define SEI_CTRL_IRQ_INT_STS_PTR0_ST_MASK (0x10U)
#define SEI_CTRL_IRQ_INT_STS_PTR0_ST_SHIFT (4U)
#define SEI_CTRL_IRQ_INT_STS_PTR0_ST_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_PTR0_ST_MASK) >> SEI_CTRL_IRQ_INT_STS_PTR0_ST_SHIFT)

/*
 * WDOG (RO)
 *
 * Watch dog
 */
#define SEI_CTRL_IRQ_INT_STS_WDOG_MASK (0x4U)
#define SEI_CTRL_IRQ_INT_STS_WDOG_SHIFT (2U)
#define SEI_CTRL_IRQ_INT_STS_WDOG_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_WDOG_MASK) >> SEI_CTRL_IRQ_INT_STS_WDOG_SHIFT)

/*
 * EXECPT (RO)
 *
 * Exception
 */
#define SEI_CTRL_IRQ_INT_STS_EXECPT_MASK (0x2U)
#define SEI_CTRL_IRQ_INT_STS_EXECPT_SHIFT (1U)
#define SEI_CTRL_IRQ_INT_STS_EXECPT_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_EXECPT_MASK) >> SEI_CTRL_IRQ_INT_STS_EXECPT_SHIFT)

/*
 * STALL (RO)
 *
 * Stall
 */
#define SEI_CTRL_IRQ_INT_STS_STALL_MASK (0x1U)
#define SEI_CTRL_IRQ_INT_STS_STALL_SHIFT (0U)
#define SEI_CTRL_IRQ_INT_STS_STALL_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INT_STS_STALL_MASK) >> SEI_CTRL_IRQ_INT_STS_STALL_SHIFT)

/* Bitfield definition for register of struct array CTRL: POINTER0 */
/*
 * POINTER (RW)
 *
 * Match pointer 0
 */
#define SEI_CTRL_IRQ_POINTER0_POINTER_MASK (0xFFU)
#define SEI_CTRL_IRQ_POINTER0_POINTER_SHIFT (0U)
#define SEI_CTRL_IRQ_POINTER0_POINTER_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_POINTER0_POINTER_SHIFT) & SEI_CTRL_IRQ_POINTER0_POINTER_MASK)
#define SEI_CTRL_IRQ_POINTER0_POINTER_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_POINTER0_POINTER_MASK) >> SEI_CTRL_IRQ_POINTER0_POINTER_SHIFT)

/* Bitfield definition for register of struct array CTRL: POINTER1 */
/*
 * POINTER (RW)
 *
 * Match pointer 1
 */
#define SEI_CTRL_IRQ_POINTER1_POINTER_MASK (0xFFU)
#define SEI_CTRL_IRQ_POINTER1_POINTER_SHIFT (0U)
#define SEI_CTRL_IRQ_POINTER1_POINTER_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_POINTER1_POINTER_SHIFT) & SEI_CTRL_IRQ_POINTER1_POINTER_MASK)
#define SEI_CTRL_IRQ_POINTER1_POINTER_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_POINTER1_POINTER_MASK) >> SEI_CTRL_IRQ_POINTER1_POINTER_SHIFT)

/* Bitfield definition for register of struct array CTRL: INSTR0 */
/*
 * INSTR (RW)
 *
 * Match instruction 0
 */
#define SEI_CTRL_IRQ_INSTR0_INSTR_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_IRQ_INSTR0_INSTR_SHIFT (0U)
#define SEI_CTRL_IRQ_INSTR0_INSTR_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INSTR0_INSTR_SHIFT) & SEI_CTRL_IRQ_INSTR0_INSTR_MASK)
#define SEI_CTRL_IRQ_INSTR0_INSTR_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INSTR0_INSTR_MASK) >> SEI_CTRL_IRQ_INSTR0_INSTR_SHIFT)

/* Bitfield definition for register of struct array CTRL: INSTR1 */
/*
 * INSTR (RW)
 *
 * Match instruction 1
 */
#define SEI_CTRL_IRQ_INSTR1_INSTR_MASK (0xFFFFFFFFUL)
#define SEI_CTRL_IRQ_INSTR1_INSTR_SHIFT (0U)
#define SEI_CTRL_IRQ_INSTR1_INSTR_SET(x) (((uint32_t)(x) << SEI_CTRL_IRQ_INSTR1_INSTR_SHIFT) & SEI_CTRL_IRQ_INSTR1_INSTR_MASK)
#define SEI_CTRL_IRQ_INSTR1_INSTR_GET(x) (((uint32_t)(x) & SEI_CTRL_IRQ_INSTR1_INSTR_MASK) >> SEI_CTRL_IRQ_INSTR1_INSTR_SHIFT)

/* Bitfield definition for register array: INSTR */
/*
 * OP (RW)
 *
 * operation
 * 0: halt
 * 1: jump
 * 2: send with timeout check
 * 3: send without timout check
 * 4: wait with timeout check
 * 5: wait without timout check
 * 6: receive with timeout check
 * 7: receive without timout check
 */
#define SEI_INSTR_OP_MASK (0x1C000000UL)
#define SEI_INSTR_OP_SHIFT (26U)
#define SEI_INSTR_OP_SET(x) (((uint32_t)(x) << SEI_INSTR_OP_SHIFT) & SEI_INSTR_OP_MASK)
#define SEI_INSTR_OP_GET(x) (((uint32_t)(x) & SEI_INSTR_OP_MASK) >> SEI_INSTR_OP_SHIFT)

/*
 * CK (RW)
 *
 * clock
 * 0: low
 * 1: rise-fall
 * 2: fall-rise
 * 3:high
 */
#define SEI_INSTR_CK_MASK (0x3000000UL)
#define SEI_INSTR_CK_SHIFT (24U)
#define SEI_INSTR_CK_SET(x) (((uint32_t)(x) << SEI_INSTR_CK_SHIFT) & SEI_INSTR_CK_MASK)
#define SEI_INSTR_CK_GET(x) (((uint32_t)(x) & SEI_INSTR_CK_MASK) >> SEI_INSTR_CK_SHIFT)

/*
 * CRC (RW)
 *
 * CRC register
 * 0: don't calculate CRC
 * 1: do not set this value
 * 2: data register 2
 * 3: data register 3
 * ...
 * 29: data register 29
 * 30: value 0 when send, wait 0 in receive
 * 31: value1 when send,  wait 1 in receive
 */
#define SEI_INSTR_CRC_MASK (0x1F0000UL)
#define SEI_INSTR_CRC_SHIFT (16U)
#define SEI_INSTR_CRC_SET(x) (((uint32_t)(x) << SEI_INSTR_CRC_SHIFT) & SEI_INSTR_CRC_MASK)
#define SEI_INSTR_CRC_GET(x) (((uint32_t)(x) & SEI_INSTR_CRC_MASK) >> SEI_INSTR_CRC_SHIFT)

/*
 * DAT (RW)
 *
 * DATA register
 * 0: ignore data
 * 1: command
 * 2: data register 2
 * 3: data register 3
 * ...
 * 29: data register 29
 * 30: value 0 when send, wait 0 in receive
 * 31: value1 when send,  wait 1 in receive
 */
#define SEI_INSTR_DAT_MASK (0x1F00U)
#define SEI_INSTR_DAT_SHIFT (8U)
#define SEI_INSTR_DAT_SET(x) (((uint32_t)(x) << SEI_INSTR_DAT_SHIFT) & SEI_INSTR_DAT_MASK)
#define SEI_INSTR_DAT_GET(x) (((uint32_t)(x) & SEI_INSTR_DAT_MASK) >> SEI_INSTR_DAT_SHIFT)

/*
 * OPR (RW)
 *
 * When OP is 0, this area is the halt time in baudrate, 0 represents infinite time.
 * When OP is 1, this area is the the pointer to the command table.
 * When OP is 2-7, this area is the data length as fellow:
 * 0: 1 bit
 * 1: 2 bit
 *  ...
 * 31: 32 bit
 */
#define SEI_INSTR_OPR_MASK (0x1FU)
#define SEI_INSTR_OPR_SHIFT (0U)
#define SEI_INSTR_OPR_SET(x) (((uint32_t)(x) << SEI_INSTR_OPR_SHIFT) & SEI_INSTR_OPR_MASK)
#define SEI_INSTR_OPR_GET(x) (((uint32_t)(x) & SEI_INSTR_OPR_MASK) >> SEI_INSTR_OPR_SHIFT)

/* Bitfield definition for register of struct array DAT: DATA_MODE */
/*
 * CRC_LEN (RW)
 *
 * CRC length
 * 0: 1 bit
 * 1: 2 bit
 * ...
 * 31: 32 bit
 */
#define SEI_DAT_DATA_MODE_CRC_LEN_MASK (0x1F000000UL)
#define SEI_DAT_DATA_MODE_CRC_LEN_SHIFT (24U)
#define SEI_DAT_DATA_MODE_CRC_LEN_SET(x) (((uint32_t)(x) << SEI_DAT_DATA_MODE_CRC_LEN_SHIFT) & SEI_DAT_DATA_MODE_CRC_LEN_MASK)
#define SEI_DAT_DATA_MODE_CRC_LEN_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_MODE_CRC_LEN_MASK) >> SEI_DAT_DATA_MODE_CRC_LEN_SHIFT)

/*
 * WLEN (RW)
 *
 * word length
 * 0: 1 bit
 * 1: 2 bit
 * ...
 * 31: 32 bit
 */
#define SEI_DAT_DATA_MODE_WLEN_MASK (0x1F0000UL)
#define SEI_DAT_DATA_MODE_WLEN_SHIFT (16U)
#define SEI_DAT_DATA_MODE_WLEN_SET(x) (((uint32_t)(x) << SEI_DAT_DATA_MODE_WLEN_SHIFT) & SEI_DAT_DATA_MODE_WLEN_MASK)
#define SEI_DAT_DATA_MODE_WLEN_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_MODE_WLEN_MASK) >> SEI_DAT_DATA_MODE_WLEN_SHIFT)

/*
 * CRC_SHIFT (RW)
 *
 * CRC shift mode, this mode is used to perform repeat code check
 * 0: CRC
 * 1: shift mode
 */
#define SEI_DAT_DATA_MODE_CRC_SHIFT_MASK (0x2000U)
#define SEI_DAT_DATA_MODE_CRC_SHIFT_SHIFT (13U)
#define SEI_DAT_DATA_MODE_CRC_SHIFT_SET(x) (((uint32_t)(x) << SEI_DAT_DATA_MODE_CRC_SHIFT_SHIFT) & SEI_DAT_DATA_MODE_CRC_SHIFT_MASK)
#define SEI_DAT_DATA_MODE_CRC_SHIFT_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_MODE_CRC_SHIFT_MASK) >> SEI_DAT_DATA_MODE_CRC_SHIFT_SHIFT)

/*
 * CRC_INV (RW)
 *
 * CRC invert
 * 0: use CRC
 * 1: use inverted CRC
 */
#define SEI_DAT_DATA_MODE_CRC_INV_MASK (0x1000U)
#define SEI_DAT_DATA_MODE_CRC_INV_SHIFT (12U)
#define SEI_DAT_DATA_MODE_CRC_INV_SET(x) (((uint32_t)(x) << SEI_DAT_DATA_MODE_CRC_INV_SHIFT) & SEI_DAT_DATA_MODE_CRC_INV_MASK)
#define SEI_DAT_DATA_MODE_CRC_INV_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_MODE_CRC_INV_MASK) >> SEI_DAT_DATA_MODE_CRC_INV_SHIFT)

/*
 * WORDER (RW)
 *
 * word order
 * 0: sample as bit order
 * 1: different from bit order
 */
#define SEI_DAT_DATA_MODE_WORDER_MASK (0x800U)
#define SEI_DAT_DATA_MODE_WORDER_SHIFT (11U)
#define SEI_DAT_DATA_MODE_WORDER_SET(x) (((uint32_t)(x) << SEI_DAT_DATA_MODE_WORDER_SHIFT) & SEI_DAT_DATA_MODE_WORDER_MASK)
#define SEI_DAT_DATA_MODE_WORDER_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_MODE_WORDER_MASK) >> SEI_DAT_DATA_MODE_WORDER_SHIFT)

/*
 * BORDER (RW)
 *
 * bit order
 * 0: LSB first
 * 1: MSB first
 */
#define SEI_DAT_DATA_MODE_BORDER_MASK (0x400U)
#define SEI_DAT_DATA_MODE_BORDER_SHIFT (10U)
#define SEI_DAT_DATA_MODE_BORDER_SET(x) (((uint32_t)(x) << SEI_DAT_DATA_MODE_BORDER_SHIFT) & SEI_DAT_DATA_MODE_BORDER_MASK)
#define SEI_DAT_DATA_MODE_BORDER_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_MODE_BORDER_MASK) >> SEI_DAT_DATA_MODE_BORDER_SHIFT)

/*
 * SIGNED (RW)
 *
 * Signed
 * 0: unsigned value
 * 1: signed value
 */
#define SEI_DAT_DATA_MODE_SIGNED_MASK (0x200U)
#define SEI_DAT_DATA_MODE_SIGNED_SHIFT (9U)
#define SEI_DAT_DATA_MODE_SIGNED_SET(x) (((uint32_t)(x) << SEI_DAT_DATA_MODE_SIGNED_SHIFT) & SEI_DAT_DATA_MODE_SIGNED_MASK)
#define SEI_DAT_DATA_MODE_SIGNED_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_MODE_SIGNED_MASK) >> SEI_DAT_DATA_MODE_SIGNED_SHIFT)

/*
 * REWIND (RW)
 *
 * Write 1 to rewind read/write pointer, this is a self clear bit
 */
#define SEI_DAT_DATA_MODE_REWIND_MASK (0x100U)
#define SEI_DAT_DATA_MODE_REWIND_SHIFT (8U)
#define SEI_DAT_DATA_MODE_REWIND_SET(x) (((uint32_t)(x) << SEI_DAT_DATA_MODE_REWIND_SHIFT) & SEI_DAT_DATA_MODE_REWIND_MASK)
#define SEI_DAT_DATA_MODE_REWIND_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_MODE_REWIND_MASK) >> SEI_DAT_DATA_MODE_REWIND_SHIFT)

/*
 * MODE (RW)
 *
 * Data mode
 * 0: data mode
 * 1: check mode
 * 2: CRC mode
 */
#define SEI_DAT_DATA_MODE_MODE_MASK (0x3U)
#define SEI_DAT_DATA_MODE_MODE_SHIFT (0U)
#define SEI_DAT_DATA_MODE_MODE_SET(x) (((uint32_t)(x) << SEI_DAT_DATA_MODE_MODE_SHIFT) & SEI_DAT_DATA_MODE_MODE_MASK)
#define SEI_DAT_DATA_MODE_MODE_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_MODE_MODE_MASK) >> SEI_DAT_DATA_MODE_MODE_SHIFT)

/* Bitfield definition for register of struct array DAT: DATA_IDX */
/*
 * LAST_BIT (RW)
 *
 * Last bit index for tranceive
 */
#define SEI_DAT_DATA_IDX_LAST_BIT_MASK (0x1F000000UL)
#define SEI_DAT_DATA_IDX_LAST_BIT_SHIFT (24U)
#define SEI_DAT_DATA_IDX_LAST_BIT_SET(x) (((uint32_t)(x) << SEI_DAT_DATA_IDX_LAST_BIT_SHIFT) & SEI_DAT_DATA_IDX_LAST_BIT_MASK)
#define SEI_DAT_DATA_IDX_LAST_BIT_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_IDX_LAST_BIT_MASK) >> SEI_DAT_DATA_IDX_LAST_BIT_SHIFT)

/*
 * FIRST_BIT (RW)
 *
 * First bit index for tranceive
 */
#define SEI_DAT_DATA_IDX_FIRST_BIT_MASK (0x1F0000UL)
#define SEI_DAT_DATA_IDX_FIRST_BIT_SHIFT (16U)
#define SEI_DAT_DATA_IDX_FIRST_BIT_SET(x) (((uint32_t)(x) << SEI_DAT_DATA_IDX_FIRST_BIT_SHIFT) & SEI_DAT_DATA_IDX_FIRST_BIT_MASK)
#define SEI_DAT_DATA_IDX_FIRST_BIT_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_IDX_FIRST_BIT_MASK) >> SEI_DAT_DATA_IDX_FIRST_BIT_SHIFT)

/*
 * MAX_BIT (RW)
 *
 * Highest bit index
 */
#define SEI_DAT_DATA_IDX_MAX_BIT_MASK (0x1F00U)
#define SEI_DAT_DATA_IDX_MAX_BIT_SHIFT (8U)
#define SEI_DAT_DATA_IDX_MAX_BIT_SET(x) (((uint32_t)(x) << SEI_DAT_DATA_IDX_MAX_BIT_SHIFT) & SEI_DAT_DATA_IDX_MAX_BIT_MASK)
#define SEI_DAT_DATA_IDX_MAX_BIT_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_IDX_MAX_BIT_MASK) >> SEI_DAT_DATA_IDX_MAX_BIT_SHIFT)

/*
 * MIN_BIT (RW)
 *
 * Lowest bit index
 */
#define SEI_DAT_DATA_IDX_MIN_BIT_MASK (0x1FU)
#define SEI_DAT_DATA_IDX_MIN_BIT_SHIFT (0U)
#define SEI_DAT_DATA_IDX_MIN_BIT_SET(x) (((uint32_t)(x) << SEI_DAT_DATA_IDX_MIN_BIT_SHIFT) & SEI_DAT_DATA_IDX_MIN_BIT_MASK)
#define SEI_DAT_DATA_IDX_MIN_BIT_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_IDX_MIN_BIT_MASK) >> SEI_DAT_DATA_IDX_MIN_BIT_SHIFT)

/* Bitfield definition for register of struct array DAT: DATA_GOLD */
/*
 * GOLD_VALUE (RW)
 *
 * Gold value for check mode
 */
#define SEI_DAT_DATA_GOLD_GOLD_VALUE_MASK (0xFFFFFFFFUL)
#define SEI_DAT_DATA_GOLD_GOLD_VALUE_SHIFT (0U)
#define SEI_DAT_DATA_GOLD_GOLD_VALUE_SET(x) (((uint32_t)(x) << SEI_DAT_DATA_GOLD_GOLD_VALUE_SHIFT) & SEI_DAT_DATA_GOLD_GOLD_VALUE_MASK)
#define SEI_DAT_DATA_GOLD_GOLD_VALUE_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_GOLD_GOLD_VALUE_MASK) >> SEI_DAT_DATA_GOLD_GOLD_VALUE_SHIFT)

/* Bitfield definition for register of struct array DAT: DATA_CRCINIT */
/*
 * CRC_INIT (RW)
 *
 * CRC initial value
 */
#define SEI_DAT_DATA_CRCINIT_CRC_INIT_MASK (0xFFFFFFFFUL)
#define SEI_DAT_DATA_CRCINIT_CRC_INIT_SHIFT (0U)
#define SEI_DAT_DATA_CRCINIT_CRC_INIT_SET(x) (((uint32_t)(x) << SEI_DAT_DATA_CRCINIT_CRC_INIT_SHIFT) & SEI_DAT_DATA_CRCINIT_CRC_INIT_MASK)
#define SEI_DAT_DATA_CRCINIT_CRC_INIT_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_CRCINIT_CRC_INIT_MASK) >> SEI_DAT_DATA_CRCINIT_CRC_INIT_SHIFT)

/* Bitfield definition for register of struct array DAT: DATA_CRCPOLY */
/*
 * CRC_POLY (RW)
 *
 * CRC polymonial
 */
#define SEI_DAT_DATA_CRCPOLY_CRC_POLY_MASK (0xFFFFFFFFUL)
#define SEI_DAT_DATA_CRCPOLY_CRC_POLY_SHIFT (0U)
#define SEI_DAT_DATA_CRCPOLY_CRC_POLY_SET(x) (((uint32_t)(x) << SEI_DAT_DATA_CRCPOLY_CRC_POLY_SHIFT) & SEI_DAT_DATA_CRCPOLY_CRC_POLY_MASK)
#define SEI_DAT_DATA_CRCPOLY_CRC_POLY_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_CRCPOLY_CRC_POLY_MASK) >> SEI_DAT_DATA_CRCPOLY_CRC_POLY_SHIFT)

/* Bitfield definition for register of struct array DAT: DATA */
/*
 * DATA (RW)
 *
 * DATA
 */
#define SEI_DAT_DATA_DATA_MASK (0xFFFFFFFFUL)
#define SEI_DAT_DATA_DATA_SHIFT (0U)
#define SEI_DAT_DATA_DATA_SET(x) (((uint32_t)(x) << SEI_DAT_DATA_DATA_SHIFT) & SEI_DAT_DATA_DATA_MASK)
#define SEI_DAT_DATA_DATA_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_DATA_MASK) >> SEI_DAT_DATA_DATA_SHIFT)

/* Bitfield definition for register of struct array DAT: DATA_SET */
/*
 * DATA_SET (RW)
 *
 * DATA bit set
 */
#define SEI_DAT_DATA_SET_DATA_SET_MASK (0xFFFFFFFFUL)
#define SEI_DAT_DATA_SET_DATA_SET_SHIFT (0U)
#define SEI_DAT_DATA_SET_DATA_SET_SET(x) (((uint32_t)(x) << SEI_DAT_DATA_SET_DATA_SET_SHIFT) & SEI_DAT_DATA_SET_DATA_SET_MASK)
#define SEI_DAT_DATA_SET_DATA_SET_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_SET_DATA_SET_MASK) >> SEI_DAT_DATA_SET_DATA_SET_SHIFT)

/* Bitfield definition for register of struct array DAT: DATA_CLR */
/*
 * DATA_CLR (RW)
 *
 * DATA bit clear
 */
#define SEI_DAT_DATA_CLR_DATA_CLR_MASK (0xFFFFFFFFUL)
#define SEI_DAT_DATA_CLR_DATA_CLR_SHIFT (0U)
#define SEI_DAT_DATA_CLR_DATA_CLR_SET(x) (((uint32_t)(x) << SEI_DAT_DATA_CLR_DATA_CLR_SHIFT) & SEI_DAT_DATA_CLR_DATA_CLR_MASK)
#define SEI_DAT_DATA_CLR_DATA_CLR_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_CLR_DATA_CLR_MASK) >> SEI_DAT_DATA_CLR_DATA_CLR_SHIFT)

/* Bitfield definition for register of struct array DAT: DATA_INV */
/*
 * DATA_INV (RW)
 *
 * DATA bit toggle
 */
#define SEI_DAT_DATA_INV_DATA_INV_MASK (0xFFFFFFFFUL)
#define SEI_DAT_DATA_INV_DATA_INV_SHIFT (0U)
#define SEI_DAT_DATA_INV_DATA_INV_SET(x) (((uint32_t)(x) << SEI_DAT_DATA_INV_DATA_INV_SHIFT) & SEI_DAT_DATA_INV_DATA_INV_MASK)
#define SEI_DAT_DATA_INV_DATA_INV_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_INV_DATA_INV_MASK) >> SEI_DAT_DATA_INV_DATA_INV_SHIFT)

/* Bitfield definition for register of struct array DAT: DATA_IN */
/*
 * DATA_IN (RO)
 *
 * Data input
 */
#define SEI_DAT_DATA_IN_DATA_IN_MASK (0xFFFFFFFFUL)
#define SEI_DAT_DATA_IN_DATA_IN_SHIFT (0U)
#define SEI_DAT_DATA_IN_DATA_IN_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_IN_DATA_IN_MASK) >> SEI_DAT_DATA_IN_DATA_IN_SHIFT)

/* Bitfield definition for register of struct array DAT: DATA_OUT */
/*
 * DATA_OUT (RO)
 *
 * Data output
 */
#define SEI_DAT_DATA_OUT_DATA_OUT_MASK (0xFFFFFFFFUL)
#define SEI_DAT_DATA_OUT_DATA_OUT_SHIFT (0U)
#define SEI_DAT_DATA_OUT_DATA_OUT_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_OUT_DATA_OUT_MASK) >> SEI_DAT_DATA_OUT_DATA_OUT_SHIFT)

/* Bitfield definition for register of struct array DAT: DATA_STS */
/*
 * CRC_IDX (RO)
 *
 * CRC index
 */
#define SEI_DAT_DATA_STS_CRC_IDX_MASK (0x1F000000UL)
#define SEI_DAT_DATA_STS_CRC_IDX_SHIFT (24U)
#define SEI_DAT_DATA_STS_CRC_IDX_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_STS_CRC_IDX_MASK) >> SEI_DAT_DATA_STS_CRC_IDX_SHIFT)

/*
 * WORD_IDX (RO)
 *
 * Word index
 */
#define SEI_DAT_DATA_STS_WORD_IDX_MASK (0x1F0000UL)
#define SEI_DAT_DATA_STS_WORD_IDX_SHIFT (16U)
#define SEI_DAT_DATA_STS_WORD_IDX_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_STS_WORD_IDX_MASK) >> SEI_DAT_DATA_STS_WORD_IDX_SHIFT)

/*
 * WORD_CNT (RO)
 *
 * Word counter
 */
#define SEI_DAT_DATA_STS_WORD_CNT_MASK (0x1F00U)
#define SEI_DAT_DATA_STS_WORD_CNT_SHIFT (8U)
#define SEI_DAT_DATA_STS_WORD_CNT_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_STS_WORD_CNT_MASK) >> SEI_DAT_DATA_STS_WORD_CNT_SHIFT)

/*
 * BIT_IDX (RO)
 *
 * Bit index
 */
#define SEI_DAT_DATA_STS_BIT_IDX_MASK (0x1FU)
#define SEI_DAT_DATA_STS_BIT_IDX_SHIFT (0U)
#define SEI_DAT_DATA_STS_BIT_IDX_GET(x) (((uint32_t)(x) & SEI_DAT_DATA_STS_BIT_IDX_MASK) >> SEI_DAT_DATA_STS_BIT_IDX_SHIFT)



/* TRG_CMD register group index macro definition */
#define SEI_CTRL_TRG_TABLE_TRG_CMD_COMMAND_TRIGGER0 (0UL)
#define SEI_CTRL_TRG_TABLE_TRG_CMD_COMMAND_TRIGGER1 (1UL)
#define SEI_CTRL_TRG_TABLE_TRG_CMD_COMMAND_TRIGGER2 (2UL)
#define SEI_CTRL_TRG_TABLE_TRG_CMD_COMMAND_TRIGGER3 (3UL)

/* TRG_TIME register group index macro definition */
#define SEI_CTRL_TRG_TABLE_TRG_TIME_TIME_TRIGGER0 (0UL)
#define SEI_CTRL_TRG_TABLE_TRG_TIME_TIME_TRIGGER1 (1UL)
#define SEI_CTRL_TRG_TABLE_TRG_TIME_TIME_TRIGGER2 (2UL)
#define SEI_CTRL_TRG_TABLE_TRG_TIME_TIME_TRIGGER3 (3UL)

/* COMMAND_TABLE register group index macro definition */
#define SEI_COMMAND_TABLE_0 (0UL)
#define SEI_COMMAND_TABLE_1 (1UL)
#define SEI_COMMAND_TABLE_2 (2UL)
#define SEI_COMMAND_TABLE_3 (3UL)
#define SEI_COMMAND_TABLE_4 (4UL)
#define SEI_COMMAND_TABLE_5 (5UL)
#define SEI_COMMAND_TABLE_6 (6UL)
#define SEI_COMMAND_TABLE_7 (7UL)

/* TRAN_CFG register group index macro definition */
#define SEI_CTRL_LATCH_TRAN_CFG_TRAN_0_1 (0UL)
#define SEI_CTRL_LATCH_TRAN_CFG_TRAN_1_2 (1UL)
#define SEI_CTRL_LATCH_TRAN_CFG_TRAN_2_3 (2UL)
#define SEI_CTRL_LATCH_TRAN_CFG_TRAN_3_0 (3UL)

/* LATCH register group index macro definition */
#define SEI_LATCH_0 (0UL)
#define SEI_LATCH_1 (1UL)
#define SEI_LATCH_2 (2UL)
#define SEI_LATCH_3 (3UL)

/* CTRL register group index macro definition */
#define SEI_CTRL_0 (0UL)
#define SEI_CTRL_1 (1UL)
#define SEI_CTRL_2 (2UL)
#define SEI_CTRL_3 (3UL)
#define SEI_CTRL_4 (4UL)
#define SEI_CTRL_5 (5UL)
#define SEI_CTRL_6 (6UL)
#define SEI_CTRL_7 (7UL)
#define SEI_CTRL_8 (8UL)
#define SEI_CTRL_9 (9UL)
#define SEI_CTRL_10 (10UL)
#define SEI_CTRL_11 (11UL)
#define SEI_CTRL_12 (12UL)

/* INSTR register group index macro definition */
#define SEI_INSTR_INSTR0 (0UL)
#define SEI_INSTR_INSTR1 (1UL)
#define SEI_INSTR_INSTR2 (2UL)
#define SEI_INSTR_INSTR3 (3UL)
#define SEI_INSTR_INSTR4 (4UL)
#define SEI_INSTR_INSTR5 (5UL)
#define SEI_INSTR_INSTR6 (6UL)
#define SEI_INSTR_INSTR7 (7UL)
#define SEI_INSTR_INSTR8 (8UL)
#define SEI_INSTR_INSTR9 (9UL)
#define SEI_INSTR_INSTR10 (10UL)
#define SEI_INSTR_INSTR11 (11UL)
#define SEI_INSTR_INSTR12 (12UL)
#define SEI_INSTR_INSTR13 (13UL)
#define SEI_INSTR_INSTR14 (14UL)
#define SEI_INSTR_INSTR15 (15UL)
#define SEI_INSTR_INSTR16 (16UL)
#define SEI_INSTR_INSTR17 (17UL)
#define SEI_INSTR_INSTR18 (18UL)
#define SEI_INSTR_INSTR19 (19UL)
#define SEI_INSTR_INSTR20 (20UL)
#define SEI_INSTR_INSTR21 (21UL)
#define SEI_INSTR_INSTR22 (22UL)
#define SEI_INSTR_INSTR23 (23UL)
#define SEI_INSTR_INSTR24 (24UL)
#define SEI_INSTR_INSTR25 (25UL)
#define SEI_INSTR_INSTR26 (26UL)
#define SEI_INSTR_INSTR27 (27UL)
#define SEI_INSTR_INSTR28 (28UL)
#define SEI_INSTR_INSTR29 (29UL)
#define SEI_INSTR_INSTR30 (30UL)
#define SEI_INSTR_INSTR31 (31UL)
#define SEI_INSTR_INSTR32 (32UL)
#define SEI_INSTR_INSTR33 (33UL)
#define SEI_INSTR_INSTR34 (34UL)
#define SEI_INSTR_INSTR35 (35UL)
#define SEI_INSTR_INSTR36 (36UL)
#define SEI_INSTR_INSTR37 (37UL)
#define SEI_INSTR_INSTR38 (38UL)
#define SEI_INSTR_INSTR39 (39UL)
#define SEI_INSTR_INSTR40 (40UL)
#define SEI_INSTR_INSTR41 (41UL)
#define SEI_INSTR_INSTR42 (42UL)
#define SEI_INSTR_INSTR43 (43UL)
#define SEI_INSTR_INSTR44 (44UL)
#define SEI_INSTR_INSTR45 (45UL)
#define SEI_INSTR_INSTR46 (46UL)
#define SEI_INSTR_INSTR47 (47UL)
#define SEI_INSTR_INSTR48 (48UL)
#define SEI_INSTR_INSTR49 (49UL)
#define SEI_INSTR_INSTR50 (50UL)
#define SEI_INSTR_INSTR51 (51UL)
#define SEI_INSTR_INSTR52 (52UL)
#define SEI_INSTR_INSTR53 (53UL)
#define SEI_INSTR_INSTR54 (54UL)
#define SEI_INSTR_INSTR55 (55UL)
#define SEI_INSTR_INSTR56 (56UL)
#define SEI_INSTR_INSTR57 (57UL)
#define SEI_INSTR_INSTR58 (58UL)
#define SEI_INSTR_INSTR59 (59UL)
#define SEI_INSTR_INSTR60 (60UL)
#define SEI_INSTR_INSTR61 (61UL)
#define SEI_INSTR_INSTR62 (62UL)
#define SEI_INSTR_INSTR63 (63UL)
#define SEI_INSTR_INSTR64 (64UL)
#define SEI_INSTR_INSTR65 (65UL)
#define SEI_INSTR_INSTR66 (66UL)
#define SEI_INSTR_INSTR67 (67UL)
#define SEI_INSTR_INSTR68 (68UL)
#define SEI_INSTR_INSTR69 (69UL)
#define SEI_INSTR_INSTR70 (70UL)
#define SEI_INSTR_INSTR71 (71UL)
#define SEI_INSTR_INSTR72 (72UL)
#define SEI_INSTR_INSTR73 (73UL)
#define SEI_INSTR_INSTR74 (74UL)
#define SEI_INSTR_INSTR75 (75UL)
#define SEI_INSTR_INSTR76 (76UL)
#define SEI_INSTR_INSTR77 (77UL)
#define SEI_INSTR_INSTR78 (78UL)
#define SEI_INSTR_INSTR79 (79UL)
#define SEI_INSTR_INSTR80 (80UL)
#define SEI_INSTR_INSTR81 (81UL)
#define SEI_INSTR_INSTR82 (82UL)
#define SEI_INSTR_INSTR83 (83UL)
#define SEI_INSTR_INSTR84 (84UL)
#define SEI_INSTR_INSTR85 (85UL)
#define SEI_INSTR_INSTR86 (86UL)
#define SEI_INSTR_INSTR87 (87UL)
#define SEI_INSTR_INSTR88 (88UL)
#define SEI_INSTR_INSTR89 (89UL)
#define SEI_INSTR_INSTR90 (90UL)
#define SEI_INSTR_INSTR91 (91UL)
#define SEI_INSTR_INSTR92 (92UL)
#define SEI_INSTR_INSTR93 (93UL)
#define SEI_INSTR_INSTR94 (94UL)
#define SEI_INSTR_INSTR95 (95UL)
#define SEI_INSTR_INSTR96 (96UL)
#define SEI_INSTR_INSTR97 (97UL)
#define SEI_INSTR_INSTR98 (98UL)
#define SEI_INSTR_INSTR99 (99UL)
#define SEI_INSTR_INSTR100 (100UL)
#define SEI_INSTR_INSTR101 (101UL)
#define SEI_INSTR_INSTR102 (102UL)
#define SEI_INSTR_INSTR103 (103UL)
#define SEI_INSTR_INSTR104 (104UL)
#define SEI_INSTR_INSTR105 (105UL)
#define SEI_INSTR_INSTR106 (106UL)
#define SEI_INSTR_INSTR107 (107UL)
#define SEI_INSTR_INSTR108 (108UL)
#define SEI_INSTR_INSTR109 (109UL)
#define SEI_INSTR_INSTR110 (110UL)
#define SEI_INSTR_INSTR111 (111UL)
#define SEI_INSTR_INSTR112 (112UL)
#define SEI_INSTR_INSTR113 (113UL)
#define SEI_INSTR_INSTR114 (114UL)
#define SEI_INSTR_INSTR115 (115UL)
#define SEI_INSTR_INSTR116 (116UL)
#define SEI_INSTR_INSTR117 (117UL)
#define SEI_INSTR_INSTR118 (118UL)
#define SEI_INSTR_INSTR119 (119UL)
#define SEI_INSTR_INSTR120 (120UL)
#define SEI_INSTR_INSTR121 (121UL)
#define SEI_INSTR_INSTR122 (122UL)
#define SEI_INSTR_INSTR123 (123UL)
#define SEI_INSTR_INSTR124 (124UL)
#define SEI_INSTR_INSTR125 (125UL)
#define SEI_INSTR_INSTR126 (126UL)
#define SEI_INSTR_INSTR127 (127UL)
#define SEI_INSTR_INSTR128 (128UL)
#define SEI_INSTR_INSTR129 (129UL)
#define SEI_INSTR_INSTR130 (130UL)
#define SEI_INSTR_INSTR131 (131UL)
#define SEI_INSTR_INSTR132 (132UL)
#define SEI_INSTR_INSTR133 (133UL)
#define SEI_INSTR_INSTR134 (134UL)
#define SEI_INSTR_INSTR135 (135UL)
#define SEI_INSTR_INSTR136 (136UL)
#define SEI_INSTR_INSTR137 (137UL)
#define SEI_INSTR_INSTR138 (138UL)
#define SEI_INSTR_INSTR139 (139UL)
#define SEI_INSTR_INSTR140 (140UL)
#define SEI_INSTR_INSTR141 (141UL)
#define SEI_INSTR_INSTR142 (142UL)
#define SEI_INSTR_INSTR143 (143UL)
#define SEI_INSTR_INSTR144 (144UL)
#define SEI_INSTR_INSTR145 (145UL)
#define SEI_INSTR_INSTR146 (146UL)
#define SEI_INSTR_INSTR147 (147UL)
#define SEI_INSTR_INSTR148 (148UL)
#define SEI_INSTR_INSTR149 (149UL)
#define SEI_INSTR_INSTR150 (150UL)
#define SEI_INSTR_INSTR151 (151UL)
#define SEI_INSTR_INSTR152 (152UL)
#define SEI_INSTR_INSTR153 (153UL)
#define SEI_INSTR_INSTR154 (154UL)
#define SEI_INSTR_INSTR155 (155UL)
#define SEI_INSTR_INSTR156 (156UL)
#define SEI_INSTR_INSTR157 (157UL)
#define SEI_INSTR_INSTR158 (158UL)
#define SEI_INSTR_INSTR159 (159UL)
#define SEI_INSTR_INSTR160 (160UL)
#define SEI_INSTR_INSTR161 (161UL)
#define SEI_INSTR_INSTR162 (162UL)
#define SEI_INSTR_INSTR163 (163UL)
#define SEI_INSTR_INSTR164 (164UL)
#define SEI_INSTR_INSTR165 (165UL)
#define SEI_INSTR_INSTR166 (166UL)
#define SEI_INSTR_INSTR167 (167UL)
#define SEI_INSTR_INSTR168 (168UL)
#define SEI_INSTR_INSTR169 (169UL)
#define SEI_INSTR_INSTR170 (170UL)
#define SEI_INSTR_INSTR171 (171UL)
#define SEI_INSTR_INSTR172 (172UL)
#define SEI_INSTR_INSTR173 (173UL)
#define SEI_INSTR_INSTR174 (174UL)
#define SEI_INSTR_INSTR175 (175UL)
#define SEI_INSTR_INSTR176 (176UL)
#define SEI_INSTR_INSTR177 (177UL)
#define SEI_INSTR_INSTR178 (178UL)
#define SEI_INSTR_INSTR179 (179UL)
#define SEI_INSTR_INSTR180 (180UL)
#define SEI_INSTR_INSTR181 (181UL)
#define SEI_INSTR_INSTR182 (182UL)
#define SEI_INSTR_INSTR183 (183UL)
#define SEI_INSTR_INSTR184 (184UL)
#define SEI_INSTR_INSTR185 (185UL)
#define SEI_INSTR_INSTR186 (186UL)
#define SEI_INSTR_INSTR187 (187UL)
#define SEI_INSTR_INSTR188 (188UL)
#define SEI_INSTR_INSTR189 (189UL)
#define SEI_INSTR_INSTR190 (190UL)
#define SEI_INSTR_INSTR191 (191UL)
#define SEI_INSTR_INSTR192 (192UL)
#define SEI_INSTR_INSTR193 (193UL)
#define SEI_INSTR_INSTR194 (194UL)
#define SEI_INSTR_INSTR195 (195UL)
#define SEI_INSTR_INSTR196 (196UL)
#define SEI_INSTR_INSTR197 (197UL)
#define SEI_INSTR_INSTR198 (198UL)
#define SEI_INSTR_INSTR199 (199UL)
#define SEI_INSTR_INSTR200 (200UL)
#define SEI_INSTR_INSTR201 (201UL)
#define SEI_INSTR_INSTR202 (202UL)
#define SEI_INSTR_INSTR203 (203UL)
#define SEI_INSTR_INSTR204 (204UL)
#define SEI_INSTR_INSTR205 (205UL)
#define SEI_INSTR_INSTR206 (206UL)
#define SEI_INSTR_INSTR207 (207UL)
#define SEI_INSTR_INSTR208 (208UL)
#define SEI_INSTR_INSTR209 (209UL)
#define SEI_INSTR_INSTR210 (210UL)
#define SEI_INSTR_INSTR211 (211UL)
#define SEI_INSTR_INSTR212 (212UL)
#define SEI_INSTR_INSTR213 (213UL)
#define SEI_INSTR_INSTR214 (214UL)
#define SEI_INSTR_INSTR215 (215UL)
#define SEI_INSTR_INSTR216 (216UL)
#define SEI_INSTR_INSTR217 (217UL)
#define SEI_INSTR_INSTR218 (218UL)
#define SEI_INSTR_INSTR219 (219UL)
#define SEI_INSTR_INSTR220 (220UL)
#define SEI_INSTR_INSTR221 (221UL)
#define SEI_INSTR_INSTR222 (222UL)
#define SEI_INSTR_INSTR223 (223UL)
#define SEI_INSTR_INSTR224 (224UL)
#define SEI_INSTR_INSTR225 (225UL)
#define SEI_INSTR_INSTR226 (226UL)
#define SEI_INSTR_INSTR227 (227UL)
#define SEI_INSTR_INSTR228 (228UL)
#define SEI_INSTR_INSTR229 (229UL)
#define SEI_INSTR_INSTR230 (230UL)
#define SEI_INSTR_INSTR231 (231UL)
#define SEI_INSTR_INSTR232 (232UL)
#define SEI_INSTR_INSTR233 (233UL)
#define SEI_INSTR_INSTR234 (234UL)
#define SEI_INSTR_INSTR235 (235UL)
#define SEI_INSTR_INSTR236 (236UL)
#define SEI_INSTR_INSTR237 (237UL)
#define SEI_INSTR_INSTR238 (238UL)
#define SEI_INSTR_INSTR239 (239UL)
#define SEI_INSTR_INSTR240 (240UL)
#define SEI_INSTR_INSTR241 (241UL)
#define SEI_INSTR_INSTR242 (242UL)
#define SEI_INSTR_INSTR243 (243UL)
#define SEI_INSTR_INSTR244 (244UL)
#define SEI_INSTR_INSTR245 (245UL)
#define SEI_INSTR_INSTR246 (246UL)
#define SEI_INSTR_INSTR247 (247UL)
#define SEI_INSTR_INSTR248 (248UL)
#define SEI_INSTR_INSTR249 (249UL)
#define SEI_INSTR_INSTR250 (250UL)
#define SEI_INSTR_INSTR251 (251UL)
#define SEI_INSTR_INSTR252 (252UL)
#define SEI_INSTR_INSTR253 (253UL)
#define SEI_INSTR_INSTR254 (254UL)
#define SEI_INSTR_INSTR255 (255UL)

/* DAT register group index macro definition */
#define SEI_DAT_0 (0UL)
#define SEI_DAT_1 (1UL)
#define SEI_DAT_2 (2UL)
#define SEI_DAT_3 (3UL)
#define SEI_DAT_4 (4UL)
#define SEI_DAT_5 (5UL)
#define SEI_DAT_6 (6UL)
#define SEI_DAT_7 (7UL)
#define SEI_DAT_8 (8UL)
#define SEI_DAT_9 (9UL)
#define SEI_DAT_10 (10UL)
#define SEI_DAT_11 (11UL)
#define SEI_DAT_12 (12UL)
#define SEI_DAT_13 (13UL)
#define SEI_DAT_14 (14UL)
#define SEI_DAT_15 (15UL)
#define SEI_DAT_16 (16UL)
#define SEI_DAT_17 (17UL)
#define SEI_DAT_18 (18UL)
#define SEI_DAT_19 (19UL)
#define SEI_DAT_20 (20UL)
#define SEI_DAT_21 (21UL)
#define SEI_DAT_22 (22UL)
#define SEI_DAT_23 (23UL)
#define SEI_DAT_24 (24UL)
#define SEI_DAT_25 (25UL)
#define SEI_DAT_26 (26UL)
#define SEI_DAT_27 (27UL)
#define SEI_DAT_28 (28UL)
#define SEI_DAT_29 (29UL)
#define SEI_DAT_30 (30UL)
#define SEI_DAT_31 (31UL)


#endif /* HPM_SEI_H */