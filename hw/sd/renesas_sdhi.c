/*
 * Renesas SDHI, SD/MMC Host Interface (TMIO core) for R-Car Gen1 (R8A7778).
 *
 * The Clarion QY8XXX board has two controllers:
 *
 *     0xFFE4C000  SDHI0  - main slot, "Main Slot SD Card" (DSK1)
 *     0xFFE4D000  SDHI1  - second slot, "Sub Slot SD Card" (DSK11)
 *
 * Slot names come from the device registry
 * (SYSTEM\StorageManager\Profiles\SDProfile / SDProfile2); addresses come
 * from r8a7778.dtsi (mmc@ffe4c000, mmc@ffe4d000) and live QEMU traces of
 * `SDHC.dll` accesses (vbase 0xefa20000).
 *
 * --- Register semantics sources -----------------------------------------
 *
 * Derived from `SDHC.dll` (disassembled section 0, VA 0xefa21000), not a
 * datasheet. Established points:
 *
 *   0xefa30dd4  WaitForSclkDivEn:  ldrh r3,[base+0x1e]; tst r3,#0x2000
 *               - retry twice with a 1 ms delay, otherwise error 0xFFFFFBB2
 *               (mvn r3,#0x400; eor r0,r3,#0x4d @0xefa30f54).
 *   0xefa30e20  Init: (rd16(0x20) & 0x0018) | 0x0305 -> 0x20,
 *               0x8b7f -> 0x22, 0xc007 -> 0x38, 0 -> 0xd8.
 *   0xefa30654  Reset: 0 -> 0xe0, delay, 1 -> 0xe0.
 *   0xefa30f28  SetClock: after WaitForSclkDivEn, updates 0x24
 *               (bit 8 = SCLKEN, bits 7..0 = divider; 0xFF is special).
 *   0xefa307f0  (rd16(0x28) & 0xFFF0) | TOP - SD_OPTION timeout field;
 *   0xefa3081c  (rd16(0x28) & 0x00FF) | 0x8000 - 1-bit bus width.
 *   0xefa318d8  WaitEvent(mask32): polls 0x1c (low 16) and 0x1e (high 16),
 *               clears matched bits by writing ~bits, and always preserves
 *               0x0800 in 0x1e, so this hardware bit is always set.
 *   0xefa31c00  Command: 0x0a=block count, 0x08=0x100 (SEC), 0x26=block
 *               length, 0x04/0x06=argument, 0x00=SD_CMD (bits 5..0 index,
 *               6 ACMD, 11 data, 12 read, 13 multi).
 *   0xefa32340/0xefa32534  PIO through 0x30, 16 bits at a time.
 *
 * --- Model scope ---------------------------------------------------------
 *
 * This is a REAL controller model: commands use QEMU's sd-bus rather than
 * faking success. SD_INFO2.SCLKDIVEN is not always 1; it follows controller
 * state (no transfer in progress), as do CMD_BUSY and DAT0. An empty slot
 * stays empty: commands receive no response and the controller sets CMDTIMEOUT.
 *
 * Register reset values are not confirmed by a datasheet. Safe values are
 * used where the driver performs RMW and are documented by the register array.
 *
 * The model exposes an interrupt line, but the board does NOT connect it yet:
 * no evidence establishes the SDHI GIC SPI, and a wrong line could cause an
 * interrupt storm. The driver tolerates this: its main wait path
 * (0xefa318d8, mode 1) polls with Sleep(10).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/sd/sd.h"
#include "hw/sd/renesas_sdhi.h"
#include "hw/dma/clarion_hpbdma.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "trace.h"

#define TYPE_RENESAS_SDHI_BUS "renesas-sdhi-bus"
DECLARE_INSTANCE_CHECKER(SDBus, RENESAS_SDHI_BUS, TYPE_RENESAS_SDHI_BUS)

OBJECT_DECLARE_SIMPLE_TYPE(RenesasSDHIState, RENESAS_SDHI)

/* --- registers (16-bit, as in TMIO) ----------------------------------- */

#define SD_CMD              0x00
#define SD_PORTSEL          0x02
#define SD_ARG0             0x04
#define SD_ARG1             0x06
#define SD_STOP             0x08
#define SD_SECCNT           0x0a
#define SD_RSP0             0x0c    /* 0x0c..0x1a: eight response words */
#define SD_RSP7             0x1a
#define SD_INFO1            0x1c
#define SD_INFO2            0x1e
#define SD_INFO1_MASK       0x20
#define SD_INFO2_MASK       0x22
#define SD_CLK_CTRL         0x24
#define SD_SIZE             0x26
#define SD_OPTION           0x28
#define SD_ERR_STS1         0x2c
#define SD_ERR_STS2         0x2e
#define SD_BUF0             0x30
#define SDIO_MODE           0x34
#define SDIO_INFO1          0x36
#define SDIO_INFO1_MASK     0x38
#define CC_EXT_MODE         0xd8
#define SD_SOFT_RST         0xe0
#define SD_VERSION          0xe2
#define SD_HOST_MODE        0xe4
#define SD_SDIF_MODE        0xe6

/* SD_CMD */
#define CMD_IDX             0x003f
#define CMD_ACMD            0x0040
#define CMD_RSPTP           0x0700
#define CMD_RSPTP_NONE      0x0300
#define CMD_DATA            0x0800
#define CMD_READ            0x1000
#define CMD_MULTI           0x2000
#define CMD_NOAUTOSTOP      0x4000      /* 0 = controller sends CMD12 itself */

/* SD_STOP */
#define STOP_STP            0x0001
#define STOP_SEC            0x0100

/* SD_INFO1: bits cleared by writing zero */
#define INFO1_RESPEND       0x0001
#define INFO1_ACCEND        0x0004
#define INFO1_REMOVE        0x0008
#define INFO1_INSERT        0x0010
#define INFO1_SIGSTATE      0x0020      /* CD line level, not an event */
#define INFO1_WRPROTECT     0x0080      /* set = writes allowed (TMIO) */
#define INFO1_REMOVE_A      0x0100
#define INFO1_INSERT_A      0x0200
#define INFO1_SIGSTATE_A    0x0400      /* level, not an event */

#define INFO1_W0C   (INFO1_RESPEND | INFO1_ACCEND | INFO1_REMOVE | \
                     INFO1_INSERT | INFO1_REMOVE_A | INFO1_INSERT_A)
#define INFO1_IRQ   INFO1_W0C

/* SD_INFO2 */
#define INFO2_CMD_IDX_ERR   0x0001
#define INFO2_CRCFAIL       0x0002
#define INFO2_STOPBIT_ERR   0x0004
#define INFO2_DATATIMEOUT   0x0008
#define INFO2_RXOVERFLOW    0x0010
#define INFO2_TXUNDERRUN    0x0020
#define INFO2_CMDTIMEOUT    0x0040
#define INFO2_DAT0          0x0080      /* DAT0 level: 1 = card is idle */
#define INFO2_RXRDY         0x0100
#define INFO2_TXRQ          0x0200
#define INFO2_ALWAYS        0x0800      /* always 1 in hardware (see above) */
#define INFO2_SCLKDIVEN     0x2000      /* clock divider can be changed */
#define INFO2_CMD_BUSY      0x4000
#define INFO2_ILL_ACCESS    0x8000

#define INFO2_ERRORS    (INFO2_CMD_IDX_ERR | INFO2_CRCFAIL | \
                         INFO2_STOPBIT_ERR | INFO2_DATATIMEOUT | \
                         INFO2_RXOVERFLOW | INFO2_TXUNDERRUN | \
                         INFO2_CMDTIMEOUT)
#define INFO2_W0C       (INFO2_ERRORS | INFO2_RXRDY | INFO2_TXRQ | \
                         INFO2_ILL_ACCESS)
/*
 * Only events generate interrupts. DAT0, ALWAYS, SCLKDIVEN and CMD_BUSY are
 * levels; they never raise the line even though the driver leaves them
 * unmasked (SD_INFO2_MASK = 0x8b7f masks bit 11 but not 13/14).
 */
#define INFO2_IRQ       INFO2_W0C

/* SD_CLK_CTRL */
#define CLK_SCLKEN          0x0100

/* SD_SOFT_RST: bit 0 = 0 holds the controller in reset */
#define SOFT_RST_RELEASE    0x0001

/* CC_EXT_MODE: bit 1 selects DMA instead of SD_BUF0 data transfers */
#define CC_EXT_DMASDRW      0x0002

#define SDHI_MAX_BLOCK      0x800

struct RenesasSDHIState {
    SysBusDevice parent_obj;

    SDBus sdbus;
    MemoryRegion iomem;
    qemu_irq irq;
    bool irq_level;
    bool irq_level_valid;

    QEMUTimer *wire_timer;
    uint32_t input_clock_hz;
    /* 0: idle, 1: command, 2: read, 3: write, 4: stop, 5: DMA read burst */
    uint8_t wire_phase;
    bool burst;                 /* remaining blocks are already due */
    uint64_t wire_cycles;
    int64_t wire_started_ns;
    int64_t wire_deadline_ns;

    uint8_t unit;               /* controller number, for traces only */

    /*
     * CC_EXT_MODE.DMASDRW uses HPB-DMAC for both transfers. It reads
     * SD_BUF0 for module-to-memory and writes it for memory-to-module.
     * Both directions use the same FIFO path as PIO.
     */
    DeviceState *dmac;
    uint64_t dma_buf_addr;      /* physical address of this controller's SD_BUF0 */
    char *read_log_path;
    uint32_t read_sector;
    uint32_t read_index;
    uint32_t read_total;

    uint16_t cmd, portsel, arg0, arg1, stop, seccnt;
    uint16_t rsp[8];
    uint16_t info1, info2;
    uint16_t info1_mask, info2_mask;
    uint16_t clk_ctrl, size, option;
    uint16_t err_sts1, err_sts2;
    uint16_t sdio_mode, sdio_info1, sdio_info1_mask;
    uint16_t cc_ext_mode, soft_rst, host_mode, sdif_mode;

    /* data transfer state (PIO through SD_BUF0) */
    uint8_t  buf[SDHI_MAX_BLOCK];
    uint32_t blen;              /* block length in bytes */
    uint32_t pos;               /* bytes transferred in current block */
    uint32_t blocks;            /* blocks remaining, including current */
    bool     reading, writing;
    bool     cmd_deferred;
};

/* --- helpers ----------------------------------------------------------- */

static bool sdhi_idle(RenesasSDHIState *s)
{
    return !s->reading && !s->writing && !s->wire_phase && !s->cmd_deferred;
}

static bool sdhi_card_present(RenesasSDHIState *s)
{
    return sdbus_get_inserted(&s->sdbus);
}

/* SD_INFO1 with line levels added */
static uint16_t sdhi_info1_live(RenesasSDHIState *s)
{
    uint16_t v = s->info1 & INFO1_W0C;

    if (sdhi_card_present(s)) {
        v |= INFO1_SIGSTATE | INFO1_SIGSTATE_A;
        if (!sdbus_get_readonly(&s->sdbus)) {
            v |= INFO1_WRPROTECT;
        }
    }
    return v;
}

/* SD_INFO2 with levels added */
static uint16_t sdhi_info2_live(RenesasSDHIState *s)
{
    uint16_t v = (s->info2 & INFO2_W0C) | INFO2_ALWAYS;

    if (sdhi_idle(s)) {
        /*
         * No transfer is in progress, so the clock divider can change and
         * the bus is idle. WaitForSclkDivEn waits for this bit.
         */
        v |= INFO2_SCLKDIVEN;
        if (sdhi_card_present(s)) {
            v |= INFO2_DAT0;
        }
    } else {
        v |= INFO2_CMD_BUSY;
    }
    return v;
}

static void sdhi_update_irq(RenesasSDHIState *s)
{
    int level = ((sdhi_info1_live(s) & INFO1_IRQ & ~s->info1_mask) ||
                 (sdhi_info2_live(s) & INFO2_IRQ & ~s->info2_mask) ||
                 (s->sdio_info1 & ~s->sdio_info1_mask & 0x0001));

    trace_renesas_sdhi_irq(s->unit, level);
    /* Repeated high levels can refresh the guest's INT2 source table. */
    if (!s->irq_level_valid || s->irq_level != level || level) {
        qemu_set_irq(s->irq, level);
        s->irq_level = level;
        s->irq_level_valid = true;
    }
}

static void sdhi_abort_data(RenesasSDHIState *s)
{
    timer_del(s->wire_timer);
    s->wire_phase = 0;
    s->burst = false;
    s->reading = s->writing = false;
    s->pos = 0;
    s->blocks = 0;
    s->info2 &= ~(INFO2_RXRDY | INFO2_TXRQ);
    s->cmd_deferred = false;
}

/* --- commands ---------------------------------------------------------- */

/*
 * The driver usually leaves RSPTP = 000 ("automatic"), so the controller
 * selects response type from the command index. This is the SD specification's
 * list of commands without a response.
 */
static bool sdhi_cmd_has_response(unsigned idx)
{
    switch (idx) {
    case 0:     /* GO_IDLE_STATE */
    case 4:     /* SET_DSR */
    case 15:    /* GO_INACTIVE_STATE */
        return false;
    default:
        return true;
    }
}

/*
 * TMIO clock encoding: 0xff bypasses the divider, zero divides by two,
 * and a one-hot value divides by four times that value. The board supplies
 * the input clock; zero preserves the existing untimed controller.
 */
static unsigned sdhi_clock_divisor(RenesasSDHIState *s)
{
    unsigned div = s->clk_ctrl & 0xff;

    return div == 0xff ? 1 : (div ? 4 * div : 2);
}

static uint64_t sdhi_cycles_ns(RenesasSDHIState *s, uint64_t cycles)
{
    return DIV_ROUND_UP(cycles * sdhi_clock_divisor(s) * NANOSECONDS_PER_SECOND,
                        s->input_clock_hz);
}

/*
 * Start a bus phase. A chained phase continues the previous one on the
 * bus (next block of a multi-block read, auto-stop after the last block):
 * it starts when the previous phase was due, not when the guest got round
 * to asking, so host timer latency does not accumulate across blocks.
 */
static void sdhi_wire_wait(RenesasSDHIState *s, uint8_t phase,
                           uint64_t cycles, bool chain)
{
    uint64_t ns;

    s->wire_phase = phase;
    s->wire_cycles = cycles;
    s->wire_started_ns = chain ? s->wire_deadline_ns :
                         qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_del(s->wire_timer);
    if (!(s->clk_ctrl & CLK_SCLKEN)) {
        return;
    }
    ns = sdhi_cycles_ns(s, cycles);
    s->wire_deadline_ns = s->wire_started_ns + ns;
    timer_mod(s->wire_timer, s->wire_deadline_ns);
}

static uint64_t sdhi_block_cycles(RenesasSDHIState *s)
{
    unsigned width = s->option & 0x8000 ? 1 : 4;

    /* A start bit, 16 CRC bits and an end bit on each data line. */
    return DIV_ROUND_UP((uint64_t)s->blen * 8, width) + 18;
}

static void sdhi_fill_block(RenesasSDHIState *s)
{
    bool ready = sdbus_data_ready(&s->sdbus);

    memset(s->buf, 0, s->blen);
    if (ready) {
        sdbus_read_data(&s->sdbus, s->buf, s->blen);
    } else {
        /* Card stopped supplying data: transfer error, not silence. */
        s->info2 |= INFO2_DATATIMEOUT;
    }

    if (s->read_log_path && *s->read_log_path &&
        ((s->cmd & CMD_IDX) == 17 || (s->cmd & CMD_IDX) == 18)) {
        FILE *file = fopen(s->read_log_path, "a");
        bool all_zero = ready;

        for (uint32_t i = 0; i < s->blen; i++) {
            all_zero &= s->buf[i] == 0;
        }
        if (file) {
            fprintf(file,
                    "{\"now_ns\":%" PRId64 ",\"unit\":%u,"
                    "\"cmd\":%u,\"block\":%u,"
                    "\"block_index\":%u,\"blocks\":%u,"
                    "\"bytes\":%u,\"ready\":%s,\"all_zero\":%s,"
                    "\"result\":\"%s\"}\n",
                    qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                    s->unit, s->cmd & CMD_IDX, s->read_sector,
                    s->read_index, s->read_total, s->blen,
                    ready ? "true" : "false",
                    all_zero ? "true" : "false",
                    ready ? "ok" : "not-ready");
            fclose(file);
        }
    }
    if (!ready) {
        sdhi_abort_data(s);
        return;
    }

    s->read_sector++;
    s->read_index++;
    s->pos = 0;
    s->info2 |= INFO2_RXRDY;
}

/*
 * DMA request from the controller: "data is ready in the buffer now".
 *
 * This is a request, not permission: DMAC queries it before each transfer
 * unit and reads SD_BUF0 only while it is true. The model needs no separate
 * counter and cannot return more bytes than the card supplied.
 */
static bool sdhi_dma_ready(void *opaque)
{
    RenesasSDHIState *s = opaque;

    return !s->wire_phase && s->dmac &&
           (s->cc_ext_mode & CC_EXT_DMASDRW) &&
           (s->reading || s->writing) && s->pos < s->blen;
}

/*
 * Multi-block DMA read: the guest only looks at the controller when the
 * command ends, and the DMAC drains the FIFO on its own. One timer covers
 * every remaining block plus the automatic CMD12, ending at the same
 * virtual instant as the per-block chain (the sum of identical per-block
 * delays). When it fires, sdhi_wire_expired() hands all blocks to the DMAC.
 */
static void sdhi_wire_burst(RenesasSDHIState *s)
{
    bool stop = (s->cmd & CMD_MULTI) && (s->stop & STOP_SEC) &&
                !(s->cmd & CMD_NOAUTOSTOP);
    uint64_t bc = sdhi_block_cycles(s);
    uint64_t ns = s->blocks * sdhi_cycles_ns(s, bc) +
                  (stop ? sdhi_cycles_ns(s, 48 + 48) : 0);

    s->wire_phase = 5;
    s->wire_cycles = s->blocks * bc + (stop ? 48 + 48 : 0);
    s->wire_started_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->wire_deadline_ns = s->wire_started_ns + ns;
    timer_mod(s->wire_timer, s->wire_deadline_ns);
}

static void sdhi_start_data(RenesasSDHIState *s)
{
    s->blen = s->size ? s->size : 512;
    if (s->blen > SDHI_MAX_BLOCK) {
        qemu_log_mask(LOG_GUEST_ERROR, "renesas-sdhi: block length %u > %u\n",
                      s->blen, SDHI_MAX_BLOCK);
        s->blen = SDHI_MAX_BLOCK;
    }
    s->blocks = (s->stop & STOP_SEC) ? s->seccnt : 1;
    if (!s->blocks) {
        s->blocks = 1;
    }
    s->pos = 0;

    if ((s->cc_ext_mode & CC_EXT_DMASDRW) && !s->dmac) {
        /*
         * DMA was requested, but no DMA controller is connected (machine
         * lacks HPB-DMAC or its property is unset). Do not silently fake
         * success: no one would transfer the data. The driver always has a
         * PIO path (0xefa32340), so continue through it.
         */
        qemu_log_mask(LOG_UNIMP, "renesas-sdhi: DMA transfer requested, but "
                      "no DMAC is connected (CC_EXT_MODE=%#x)\n",
                      s->cc_ext_mode);
    }

    if (s->cmd & CMD_READ) {
        s->reading = true;
        if (s->input_clock_hz) {
            s->info2 &= ~INFO2_RXRDY;
            if (s->blocks > 1 && s->dmac &&
                (s->cc_ext_mode & CC_EXT_DMASDRW) &&
                (s->clk_ctrl & CLK_SCLKEN)) {
                sdhi_wire_burst(s);
            } else {
                sdhi_wire_wait(s, 2, sdhi_block_cycles(s), false);
            }
        } else {
            sdhi_fill_block(s);
        }
        /*
         * The block is buffered, but this board arms DMAC AFTER the command
         * (trace: DCR/DSAR/DDAR/DTCR -> CMD17 -> DCMDR.DMEN). Kick DMAC for
         * either order: an already armed channel consumes the block now;
         * otherwise it will do so on DMEN after querying sdhi_dma_ready().
         */
        if (sdhi_dma_ready(s)) {
            clarion_hpbdma_module_poke(s->dmac, s->dma_buf_addr);
        }
    } else {
        s->writing = true;
        s->info2 |= INFO2_TXRQ;
        if (sdhi_dma_ready(s)) {
            clarion_hpbdma_module_poke(s->dmac, s->dma_buf_addr);
        }
    }
}

static void sdhi_execute_command(RenesasSDHIState *s)
{
    unsigned idx = s->cmd & CMD_IDX;
    SDRequest req;
    uint8_t rsp[16];
    size_t rlen;
    bool want_rsp;
    int i;

    if (!(s->soft_rst & SOFT_RST_RELEASE)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "renesas-sdhi: CMD%u while in reset\n", idx);
        return;
    }

    sdhi_abort_data(s);
    s->info2 &= ~INFO2_ERRORS;
    memset(s->rsp, 0, sizeof(s->rsp));

    req.cmd = idx;
    req.arg = ((uint32_t)s->arg1 << 16) | s->arg0;
    if ((s->cmd & (CMD_DATA | CMD_READ)) == (CMD_DATA | CMD_READ)) {
        s->read_sector = req.arg;
        s->read_index = 0;
        s->read_total = (s->stop & STOP_SEC) ? s->seccnt : 1;
        if (!s->read_total) {
            s->read_total = 1;
        }
    }
    trace_renesas_sdhi_command(s->unit, idx, req.arg, s->cmd);

    want_rsp = (s->cmd & CMD_RSPTP) != CMD_RSPTP_NONE &&
               sdhi_cmd_has_response(idx);

    if (!sdhi_card_present(s)) {
        /* Empty slot: no card is present to respond on the bus. */
        if (want_rsp) {
            s->info2 |= INFO2_CMDTIMEOUT;
        } else {
            s->info1 |= INFO1_RESPEND;
        }
        sdhi_update_irq(s);
        return;
    }

    rlen = sdbus_do_command(&s->sdbus, &req, rsp, sizeof(rsp));

    if (rlen == 4) {
        uint32_t v = ldl_be_p(&rsp[0]);

        s->rsp[0] = v & 0xffff;
        s->rsp[1] = v >> 16;
    } else if (rlen == 16) {
        /*
         * R2: the registers hold bits [127:8] without the CRC, right-aligned;
         * tmio_mmc_cmd_irq() shifts them back left by 8 bits.
         */
        uint8_t r2[16] = { 0 };

        memcpy(&r2[1], rsp, 15);
        for (i = 0; i < 4; i++) {
            uint32_t v = ldl_be_p(&r2[12 - 4 * i]);

            s->rsp[2 * i] = v & 0xffff;
            s->rsp[2 * i + 1] = v >> 16;
        }
    } else if (want_rsp) {
        trace_renesas_sdhi_timeout(s->unit, idx);
        s->info2 |= INFO2_CMDTIMEOUT;
        sdhi_update_irq(s);
        return;
    }

    s->info1 |= INFO1_RESPEND;

    if (s->cmd & CMD_DATA) {
        sdhi_start_data(s);
    }
    sdhi_update_irq(s);
}

static void sdhi_send_command(RenesasSDHIState *s)
{
    unsigned idx = s->cmd & CMD_IDX;
    unsigned response_bits;

    if (!s->input_clock_hz) {
        if (!(s->soft_rst & SOFT_RST_RELEASE)) {
            return;
        }
        if (!(s->clk_ctrl & CLK_SCLKEN)) {
            s->cmd_deferred = true;
            return;
        }
        s->cmd_deferred = false;
        sdhi_execute_command(s);
        return;
    }
    if (!(s->soft_rst & SOFT_RST_RELEASE)) {
        return;
    }
    sdhi_abort_data(s);
    response_bits = (s->cmd & CMD_RSPTP) == CMD_RSPTP_NONE ||
                    !sdhi_cmd_has_response(idx) ? 0 :
                    (idx == 2 || idx == 9 || idx == 10 ? 136 : 48);
    /* Command packet and response packet, excluding card-internal latency. */
    sdhi_wire_wait(s, 1, 48 + response_bits, false);
}

/* --- data port --------------------------------------------------------- */

static void sdhi_block_done(RenesasSDHIState *s)
{
    s->blocks--;
    if (s->blocks) {
        if (s->reading) {
            if (s->burst) {
                sdhi_fill_block(s);     /* already due, no bus time left */
            } else if (s->input_clock_hz) {
                s->info2 &= ~INFO2_RXRDY;
                sdhi_wire_wait(s, 2, sdhi_block_cycles(s), true);
            } else {
                sdhi_fill_block(s);
            }
        } else {
            s->pos = 0;
        }
        return;
    }
    if ((s->cmd & CMD_MULTI) && (s->stop & STOP_SEC) &&
        !(s->cmd & CMD_NOAUTOSTOP)) {
        if (s->input_clock_hz && !s->burst) {
            sdhi_wire_wait(s, 4, 48 + 48, true);
            return;
        }
        SDRequest req = { .cmd = 12, .arg = 0 };
        uint8_t rsp[16];

        sdbus_do_command(&s->sdbus, &req, rsp, sizeof(rsp));
    }
    sdhi_abort_data(s);
    s->info1 |= INFO1_ACCEND;
}

static uint64_t sdhi_buf_read(RenesasSDHIState *s, unsigned size)
{
    uint64_t v = 0;
    unsigned i;

    if (!s->reading || s->wire_phase) {
        s->info2 |= INFO2_ILL_ACCESS;
        sdhi_update_irq(s);
        return 0;
    }
    for (i = 0; i < size; i++) {
        v |= (uint64_t)s->buf[s->pos++] << (8 * i);
        if (s->pos >= s->blen) {
            sdhi_block_done(s);
            break;
        }
    }
    sdhi_update_irq(s);
    return v;
}

static void sdhi_buf_write(RenesasSDHIState *s, uint64_t val, unsigned size)
{
    unsigned i;

    if (!s->writing || s->wire_phase) {
        s->info2 |= INFO2_ILL_ACCESS;
        sdhi_update_irq(s);
        return;
    }
    for (i = 0; i < size; i++) {
        s->buf[s->pos++] = (val >> (8 * i)) & 0xff;
        if (s->pos >= s->blen) {
            if (s->input_clock_hz) {
                s->info2 &= ~INFO2_TXRQ;
                sdhi_wire_wait(s, 3, sdhi_block_cycles(s), false);
            } else {
                sdbus_write_data(&s->sdbus, s->buf, s->blen);
                sdhi_block_done(s);
            }
            break;
        }
    }
    sdhi_update_irq(s);
}

static void sdhi_wire_expired(void *opaque)
{
    RenesasSDHIState *s = opaque;
    uint8_t phase = s->wire_phase;

    s->wire_phase = 0;
    switch (phase) {
    case 1:
        sdhi_execute_command(s);
        break;
    case 5:
        s->burst = true;
        /* fall through */
    case 2:
        sdhi_fill_block(s);
        if (sdhi_dma_ready(s)) {
            clarion_hpbdma_module_poke(s->dmac, s->dma_buf_addr);
        }
        break;
    case 3:
        sdbus_write_data(&s->sdbus, s->buf, s->blen);
        sdhi_block_done(s);
        if (sdhi_dma_ready(s)) {
            clarion_hpbdma_module_poke(s->dmac, s->dma_buf_addr);
        }
        if (s->writing) {
            s->info2 |= INFO2_TXRQ;
        }
        break;
    case 4: {
        SDRequest req = { .cmd = 12, .arg = 0 };
        uint8_t rsp[16];

        sdbus_do_command(&s->sdbus, &req, rsp, sizeof(rsp));
        sdhi_abort_data(s);
        s->info1 |= INFO1_ACCEND;
        break;
    }
    default:
        g_assert_not_reached();
    }
    sdhi_update_irq(s);
}

/* --- MMIO ------------------------------------------------------------- */

static uint64_t sdhi_read(void *opaque, hwaddr addr, unsigned size)
{
    RenesasSDHIState *s = opaque;
    uint64_t v;

    if (addr == SD_BUF0) {
        return sdhi_buf_read(s, size);
    }

    switch (addr) {
    case SD_CMD:            v = s->cmd;             break;
    case SD_PORTSEL:        v = s->portsel;         break;
    case SD_ARG0:           v = s->arg0;            break;
    case SD_ARG1:           v = s->arg1;            break;
    case SD_STOP:           v = s->stop;            break;
    case SD_SECCNT:         v = s->seccnt;          break;
    case SD_RSP0 ... SD_RSP7:
        v = s->rsp[(addr - SD_RSP0) / 2];
        break;
    case SD_INFO1:          v = sdhi_info1_live(s); break;
    case SD_INFO2:          v = sdhi_info2_live(s); break;
    case SD_INFO1_MASK:     v = s->info1_mask;      break;
    case SD_INFO2_MASK:     v = s->info2_mask;      break;
    case SD_CLK_CTRL:       v = s->clk_ctrl;        break;
    case SD_SIZE:           v = s->size;            break;
    case SD_OPTION:         v = s->option;          break;
    case SD_ERR_STS1:       v = s->err_sts1;        break;
    case SD_ERR_STS2:       v = s->err_sts2;        break;
    case SDIO_MODE:         v = s->sdio_mode;       break;
    case SDIO_INFO1:        v = s->sdio_info1;      break;
    case SDIO_INFO1_MASK:   v = s->sdio_info1_mask; break;
    case CC_EXT_MODE:       v = s->cc_ext_mode;     break;
    case SD_SOFT_RST:       v = s->soft_rst;        break;
    case SD_VERSION:        v = 0;                  break;
    case SD_HOST_MODE:      v = s->host_mode;       break;
    case SD_SDIF_MODE:      v = s->sdif_mode;       break;
    default:
        qemu_log_mask(LOG_UNIMP, "renesas-sdhi: read from unknown "
                      "register +%#" HWADDR_PRIx "\n", addr);
        v = 0;
        break;
    }
    trace_renesas_sdhi_read(s->unit, addr, size, v);
    return v;
}

static void sdhi_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    RenesasSDHIState *s = opaque;

    trace_renesas_sdhi_write(s->unit, addr, size, val);

    if (addr == SD_BUF0) {
        sdhi_buf_write(s, val, size);
        return;
    }

    switch (addr) {
    case SD_CMD:
        s->cmd = val;
        sdhi_send_command(s);
        break;
    case SD_PORTSEL:        s->portsel = val;   break;
    case SD_ARG0:           s->arg0 = val;      break;
    case SD_ARG1:           s->arg1 = val;      break;
    case SD_STOP:
        s->stop = val;
        if (val & STOP_STP) {
            /* force-stop transfer */
            sdhi_abort_data(s);
            sdhi_update_irq(s);
        }
        break;
    case SD_SECCNT:         s->seccnt = val;    break;
    case SD_INFO1:
        /* writing zero clears the bit; writing one leaves it unchanged */
        s->info1 &= (val | ~INFO1_W0C);
        sdhi_update_irq(s);
        break;
    case SD_INFO2:
        s->info2 &= (val | ~INFO2_W0C);
        sdhi_update_irq(s);
        break;
    case SD_INFO1_MASK:
        s->info1_mask = val;
        sdhi_update_irq(s);
        break;
    case SD_INFO2_MASK:
        s->info2_mask = val;
        sdhi_update_irq(s);
        break;
    case SD_CLK_CTRL:
        if (s->input_clock_hz && s->wire_phase) {
            uint64_t elapsed = 0;
            uint64_t cycles;

            if (s->clk_ctrl & CLK_SCLKEN) {
                elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                          s->wire_started_ns;
            }
            cycles = elapsed * s->input_clock_hz /
                     ((uint64_t)sdhi_clock_divisor(s) *
                      NANOSECONDS_PER_SECOND);
            cycles = MIN(cycles, s->wire_cycles);
            s->clk_ctrl = val;
            sdhi_wire_wait(s, s->wire_phase, s->wire_cycles - cycles, false);
        } else {
            s->clk_ctrl = val;
            if (!s->input_clock_hz && s->cmd_deferred && (val & CLK_SCLKEN)) {
                s->cmd_deferred = false;
                trace_renesas_sdhi_deferred_start(s->unit, s->cmd & CMD_IDX);
                sdhi_execute_command(s);
            }
        }
        break;
    case SD_SIZE:           s->size = val;      break;
    case SD_OPTION:         s->option = val;    break;
    case SDIO_MODE:         s->sdio_mode = val; break;
    case SDIO_INFO1:
        s->sdio_info1 &= val;
        sdhi_update_irq(s);
        break;
    case SDIO_INFO1_MASK:
        s->sdio_info1_mask = val;
        sdhi_update_irq(s);
        break;
    case CC_EXT_MODE:       s->cc_ext_mode = val;   break;
    case SD_SOFT_RST:
        s->soft_rst = val;
        if (!(val & SOFT_RST_RELEASE)) {
            /* controller reset clears transfer state and events */
            sdhi_abort_data(s);
            s->info1 = 0;
            s->info2 = 0;
            sdhi_update_irq(s);
        }
        break;
    case SD_HOST_MODE:      s->host_mode = val; break;
    case SD_SDIF_MODE:      s->sdif_mode = val; break;
    case SD_RSP0 ... SD_RSP7:
    case SD_ERR_STS1:
    case SD_ERR_STS2:
    case SD_VERSION:
        qemu_log_mask(LOG_GUEST_ERROR, "renesas-sdhi: write to read-only "
                      "register +%#" HWADDR_PRIx "\n", addr);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "renesas-sdhi: write to unknown register "
                      "+%#" HWADDR_PRIx " = %#" PRIx64 "\n", addr, val);
        break;
    }
}

static const MemoryRegionOps sdhi_ops = {
    .read = sdhi_read,
    .write = sdhi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    /*
     * All registers are 16-bit and the driver accesses them as halfwords.
     * Accept byte accesses to avoid a bus error on accidental accesses;
     * QEMU composes them from our 16-bit operations.
     */
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 2,
    .impl.max_access_size = 4,
};

/* --- slot state changes ------------------------------------------------ */

static void sdhi_set_inserted(DeviceState *dev, bool inserted)
{
    RenesasSDHIState *s = RENESAS_SDHI(dev);

    trace_renesas_sdhi_inserted(s->unit, inserted);
    s->info1 |= inserted ? (INFO1_INSERT | INFO1_INSERT_A)
                         : (INFO1_REMOVE | INFO1_REMOVE_A);
    if (!inserted) {
        sdhi_abort_data(s);
    }
    sdhi_update_irq(s);
}

/*
 * SD_INFO1.WRPROTECT follows TMIO polarity: the bit set means writes are
 * allowed. The model takes it from the bus (see sdhi_info1_live); only the
 * interrupt line needs re-evaluating here.
 */
static void sdhi_set_readonly(DeviceState *dev, bool readonly)
{
    RenesasSDHIState *s = RENESAS_SDHI(dev);

    sdhi_update_irq(s);
}

/* --- QOM -------------------------------------------------------------- */

static int sdhi_post_load(void *opaque, int version_id)
{
    RenesasSDHIState *s = opaque;

    (void)version_id;
    s->irq_level_valid = false;
    sdhi_update_irq(s);
    return 0;
}

static void sdhi_reset_hold(Object *obj, ResetType type)
{
    RenesasSDHIState *s = RENESAS_SDHI(obj);

    s->cmd = s->portsel = s->arg0 = s->arg1 = s->stop = s->seccnt = 0;
    memset(s->rsp, 0, sizeof(s->rsp));
    s->info1 = 0;
    s->info2 = 0;
    /*
     * ⚠ Reset values for masks are not confirmed by a datasheet. Start with
     * everything masked, the only choice that prevents spurious interrupts
     * before driver setup. The driver later writes its expected value:
     * (0xffff & 0x0018) | 0x0305 = 0x031d (0xefa30e38), and writes the full
     * SD_INFO2_MASK word 0x8b7f.
     */
    s->info1_mask = 0xffff;
    s->info2_mask = 0xffff;
    /*
     * ⚠ Reset values for SD_CLK_CTRL, SD_OPTION and SD_SIZE are also unknown.
     * Zeros here do not reproduce power-on state; the driver programs all
     * three fields before the first transfer.
     */
    s->clk_ctrl = 0;
    s->size = 0;
    s->option = 0;
    s->err_sts1 = s->err_sts2 = 0;
    s->sdio_mode = s->sdio_info1 = 0;
    s->sdio_info1_mask = 0xffff;
    s->cc_ext_mode = 0;
    s->soft_rst = SOFT_RST_RELEASE;
    s->host_mode = s->sdif_mode = 0;
    sdhi_abort_data(s);
    s->blen = 0;
    s->irq_level = false;
    s->irq_level_valid = true;
    qemu_set_irq(s->irq, 0);
}

static void sdhi_init(Object *obj)
{
    RenesasSDHIState *s = RENESAS_SDHI(obj);

    s->wire_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sdhi_wire_expired, s);
    qbus_init(&s->sdbus, sizeof(s->sdbus), TYPE_RENESAS_SDHI_BUS,
              DEVICE(s), "sd-bus");
    memory_region_init_io(&s->iomem, obj, &sdhi_ops, s,
                          TYPE_RENESAS_SDHI, RENESAS_SDHI_SIZE);
    /* an SD_CMD write wakes the DMAC, which reads SD_BUF0 inside that write */
    s->iomem.disable_reentrancy_guard = true;
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
}

static void sdhi_finalize(Object *obj)
{
    RenesasSDHIState *s = RENESAS_SDHI(obj);

    timer_free(s->wire_timer);
}

static const VMStateDescription vmstate_renesas_sdhi = {
    .name = "renesas-sdhi",
    .version_id = 5,
    .minimum_version_id = 1,
    .post_load = sdhi_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(cmd, RenesasSDHIState),
        VMSTATE_UINT16(portsel, RenesasSDHIState),
        VMSTATE_UINT16(arg0, RenesasSDHIState),
        VMSTATE_UINT16(arg1, RenesasSDHIState),
        VMSTATE_UINT16(stop, RenesasSDHIState),
        VMSTATE_UINT16(seccnt, RenesasSDHIState),
        VMSTATE_UINT16_ARRAY(rsp, RenesasSDHIState, 8),
        VMSTATE_UINT16(info1, RenesasSDHIState),
        VMSTATE_UINT16(info2, RenesasSDHIState),
        VMSTATE_UINT16(info1_mask, RenesasSDHIState),
        VMSTATE_UINT16(info2_mask, RenesasSDHIState),
        VMSTATE_UINT16(clk_ctrl, RenesasSDHIState),
        VMSTATE_UINT16(size, RenesasSDHIState),
        VMSTATE_UINT16(option, RenesasSDHIState),
        VMSTATE_UINT16(err_sts1, RenesasSDHIState),
        VMSTATE_UINT16(err_sts2, RenesasSDHIState),
        VMSTATE_UINT16(sdio_mode, RenesasSDHIState),
        VMSTATE_UINT16(sdio_info1, RenesasSDHIState),
        VMSTATE_UINT16(sdio_info1_mask, RenesasSDHIState),
        VMSTATE_UINT16(cc_ext_mode, RenesasSDHIState),
        VMSTATE_UINT16(soft_rst, RenesasSDHIState),
        VMSTATE_UINT16(host_mode, RenesasSDHIState),
        VMSTATE_UINT16(sdif_mode, RenesasSDHIState),
        VMSTATE_UINT8_ARRAY(buf, RenesasSDHIState, SDHI_MAX_BLOCK),
        VMSTATE_UINT32(blen, RenesasSDHIState),
        VMSTATE_UINT32(pos, RenesasSDHIState),
        VMSTATE_UINT32(blocks, RenesasSDHIState),
        VMSTATE_BOOL(reading, RenesasSDHIState),
        VMSTATE_BOOL(writing, RenesasSDHIState),
        VMSTATE_UINT32_V(read_sector, RenesasSDHIState, 2),
        VMSTATE_UINT32_V(read_index, RenesasSDHIState, 2),
        VMSTATE_UINT32_V(read_total, RenesasSDHIState, 2),
        VMSTATE_UINT8_V(wire_phase, RenesasSDHIState, 3),
        VMSTATE_UINT64_V(wire_cycles, RenesasSDHIState, 3),
        VMSTATE_INT64_V(wire_started_ns, RenesasSDHIState, 3),
        VMSTATE_INT64_V(wire_deadline_ns, RenesasSDHIState, 3),
        VMSTATE_TIMER_PTR_V(wire_timer, RenesasSDHIState, 3),
        VMSTATE_BOOL_V(burst, RenesasSDHIState, 4),
        VMSTATE_BOOL_V(cmd_deferred, RenesasSDHIState, 5),
        VMSTATE_END_OF_LIST()
    }
};

static void sdhi_realize(DeviceState *dev, Error **errp)
{
    RenesasSDHIState *s = RENESAS_SDHI(dev);

    if (s->dmac) {
        if (!s->dma_buf_addr) {
            error_setg(errp, "renesas-sdhi: dma-buf-addr (physical SD_BUF0 "
                             "address) is required with dmac");
            return;
        }
        /*
         * Register as a buffered module: DMAC queries sdhi_dma_ready() and
         * reads SD_BUF0 at this address.
         */
        clarion_hpbdma_module_attach(s->dmac, s->dma_buf_addr,
                                     sdhi_dma_ready, s);
    }
}

static const Property sdhi_properties[] = {
    DEFINE_PROP_UINT32("input-clock-hz", RenesasSDHIState, input_clock_hz, 0),
    DEFINE_PROP_UINT8("unit", RenesasSDHIState, unit, 0),
    /*
     * Board DMA controller and physical address of this SDHI's SD_BUF0.
     * Both are optional; without them, the model uses PIO as before.
     */
    DEFINE_PROP_LINK("dmac", RenesasSDHIState, dmac, TYPE_DEVICE,
                     DeviceState *),
    DEFINE_PROP_UINT64("dma-buf-addr", RenesasSDHIState, dma_buf_addr, 0),
    DEFINE_PROP_STRING("read-log", RenesasSDHIState, read_log_path),
};

static void sdhi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    device_class_set_props(dc, sdhi_properties);
    dc->realize = sdhi_realize;
    dc->desc = "Renesas SDHI (TMIO) SD/MMC host controller";
    dc->vmsd = &vmstate_renesas_sdhi;
    rc->phases.hold = sdhi_reset_hold;
}

static void sdhi_bus_class_init(ObjectClass *klass, const void *data)
{
    SDBusClass *sbc = SD_BUS_CLASS(klass);

    sbc->set_inserted = sdhi_set_inserted;
    sbc->set_readonly = sdhi_set_readonly;
}

static const TypeInfo renesas_sdhi_types[] = {
    {
        .name           = TYPE_RENESAS_SDHI,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(RenesasSDHIState),
        .instance_init  = sdhi_init,
        .instance_finalize = sdhi_finalize,
        .class_init     = sdhi_class_init,
    },
    {
        .name           = TYPE_RENESAS_SDHI_BUS,
        .parent         = TYPE_SD_BUS,
        .instance_size  = sizeof(SDBus),
        .class_init     = sdhi_bus_class_init,
    },
};

DEFINE_TYPES(renesas_sdhi_types)
