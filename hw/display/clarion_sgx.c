/*
 * PowerVR SGX on the Clarion QY8XXX board (WinCE 7.0) —
 * current QY8 baseline: PDS/USE execution and declared null-render B2.
 *
 * Current baseline: supported PDS/USE programs execute, and behavioral
 * kernel-CCB B2 synthetically completes the proven TA+3D sync word. This
 * does not rasterize the scene; its pixels remain undefined. Both modes are
 * enabled by default only on the QY8 machine and can each be disabled with `=off`.
 *
 * Execution finds data through guest registers, the MMU, and structures;
 * B2 runs at the proven dispatcher point and reads live CCB inputs. No general
 * IRQ model or arbitrary completions are added. Other observation flags and
 * READBACK remain separate.
 *
 * The initial init chain is derived from execution of the documented DMA,
 * not from a hard-coded address or signature search:
 *
 *   EUR_CR_PDS_EXEC_BASE      (0x0AB8) = 0x0DC00000       <- MMIO
 *   EUR_CR_EVENT_OTHER_PDS_EXEC(0x0A68) = 0x0080C180      <- MMIO
 *   sum                                = 0x0E40C180        = PDS program #13
 *   #13 = sgxinit_primary: MOVS DOUTU  -> USE task @0x0E400BA0
 *   USE code: emitpds                  -> PDS program #14 @0x0E40C1B0
 *   #14 = sgxinit_secondary: MOVS DOUTD -> 128 B from 0x0F003000 to
 *                                          secondary attributes (SGX_UKERNEL_NUM_SEC_ATTRIB)
 *   secondary attribute sa[1] = R_HostCtl = 0x0F003120  = HOST_CTL
 *   HOST_CTL +0x00                      = ui32InitStatus
 *
 * Thus execution of the documented DMA discovers the GPU address; it is not
 * hard-coded in the model. `PVRSRV_SGX_EDMPROG_SECATTR` defines the secondary
 * attribute layout (`sgx_mkif.h:144`); `usedefs.h:70` names `R_HostCtl = SA(sHostCtl)`.
 *
 * With QY8_SGX_EXEC, the model executes supported PDS and USE programs.
 * In particular, the kernel-CCB dispatcher provides B2's live inputs. Only
 * the additional QY8_SGX_NULLRENDER flag enables a synthetic write of one
 * proven sync word; SGX_EXEC alone adds no B2 writes.
 *
 * Bases, sizes, offsets, and masks are documented in clarion_sgx.h; no value
 * here is guessed by analogy with another SoC.
 *
 * Environment switches:
 *
 *   QY8_SGX=off              disable the model (A/B against the interceptor)
 *   QY8_SGX_KICKS=N          number of initial kicks to decode in detail (2)
 *   QY8_SGX_READBACK=1       ⚠ return written values on reads. CHANGES
 *                            guest-visible behavior; for experiments only.
 *   QY8_SGX_DUMP=VA:LEN,...  also dump these GPU-VA ranges on each kick
 *   QY8_SGX_EXEC=off         disable PDS/USE execution (enabled by default).
 *   QY8_SGX_NULLRENDER=off   disable live B2 (enabled by default);
 *                            completion is synthetic and has no pixels.
 *   QY8_SGX_NULLRENDER=all   ⚠ MIRROR-only: apply the same dst-sync rule to
 *                            EVERY READY TA command between TA-CCB ReadOffset
 *                            and WriteOffset. Does not alter offsets, render
 *                            details locks, or queues. Default behavior stays.
 *   QY8_SGX_PDS_RUN=VA[:rows[:ir0]]
 *                            ⚠ diagnostic direct PDS launch, not the normal
 *                            chain; needed until USE can let `emitpds` launch
 *                            sgxinit_secondary itself.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/cutils.h"
#include "qemu/timer.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/clarion_sgx.h"
#include "system/address-spaces.h"
#include "qom/object.h"
#include "trace.h"

#define SGX_NREGS           (CLARION_SGX_SIZE / 4)
#define SGX_DUMP_RANGES     8
#define SGX_WR_JOURNAL      512
#define SGX_FIND_TARGETS    8
#define SGX_USE_QUEUE       16
#define SGX_MAX_DEPTH       8
#define SGX_USE_MAX_STEPS   4096
#define SGX_USE_POLL_REPEATS 256 /* diagnostic threshold, not hardware timing */
#define SGX_USE_POLL_SPAN    64  /* maximum instructions between poll reads */
#define SGX_PDS_MAX_STEPS   4096
#define SGX_PDS_MAX_PROG    0x2000
#define SGX_B2_SYNTH_MAX    8
#define SGX_B2_WALK_MAX     8       /* TA-CCB commands per kick in all mode */
#define SGX_B2_CMDTA_SIZE   0x280   /* live-measured TA command size */
#define SGX_B2_STORE_HISTORY 512
#define SGX_CR_EVENT_TIMER 0x0ACC
#define SGX_CR_EVENT_TIMER_ENABLE (1U << 24)
#define SGX_CR_EVENT_TIMER_VALUE_MASK 0x00FFFFFFU
#define SGX_CR_USE0_SERV_EVENT 0x0B10
#define SGX_CR_USE1_SERV_EVENT 0x0B1C
/*
 * Conditional conversion scale: target core clock is not statically known.
 * At the recorded reload of 200,000 this produces a 1.8 ms virtual tick.
 * Any tick below the watchdog's roughly 90 ms sample interval is equivalent
 * for acceptance; this is not a measured target clock.
 */
#define SGX_CORE_CLOCK_HZ 111000000ULL

typedef struct ClarionSgxSyntheticWord {
    uint32_t dva;
    uint32_t pa;
    uint32_t value;
    bool valid;
} ClarionSgxSyntheticWord;

typedef struct ClarionSgxGuestStore {
    uint32_t dva;
    uint32_t pa;
    uint32_t value;
    uint8_t size;
    bool synthetic;
} ClarionSgxGuestStore;

typedef struct ClarionSgxDump {
    uint32_t va;
    uint32_t len;
} ClarionSgxDump;

typedef struct ClarionSgxWrite {
    uint32_t off;
    uint32_t val;
} ClarionSgxWrite;

struct ClarionSgxState {
    SysBusDevice parent_obj;

    MemoryRegion mr;

    /* Everything written by the guest. Nothing here has its own semantics. */
    uint32_t regs[SGX_NREGS];
    /*
     * Which value bits are known. A full guest/USE write makes all 32 known;
     * a selective hardware side effect can justify only specific bits.
     * Unknown bits never become implicit zeros during `ldr` or a later TEST.
     */
    uint32_t regs_known_mask[SGX_NREGS];
    uint32_t regs_version[SGX_NREGS];
    uint32_t regs_epoch;

    uint32_t kicks;             /* number of EVENT_KICK2 events received */
    uint32_t kick_reports;      /* number to decode in detail */
    bool readback;              /* ⚠ return written values on reads */
    bool nullrender;            /* QY8_SGX_NULLRENDER: synthetic scene completion */
    bool nullrender_all;        /* =all: dst-sync rule for every READY TA command */
    bool b2_disabled;           /* image guard failed; disabled for this run */
    QEMUTimer *heartbeat;       /* stands in for the microkernel's timer task */
    bool edm_task_register_model;
    uint64_t edm_timer_start_ns;
    uint64_t edm_timer_period_ns;
    ClarionSgxSyntheticWord b2_synthetic[SGX_B2_SYNTH_MAX];
    unsigned nb2_synthetic;
    ClarionSgxGuestStore b2_stores[SGX_B2_STORE_HISTORY];
    unsigned nb2_stores;
    bool b2_store_overflow;

    ClarionSgxDump dump[SGX_DUMP_RANGES];
    unsigned ndump;

    /* Write journal in arrival order, including writes absent from the
     * init script (for example, SGXReset writes BIF_DIR_LIST_BASE0 itself). */
    ClarionSgxWrite wr[SGX_WR_JOURNAL];
    unsigned nwr;
    bool wr_overflow;
    bool show_writes;

    uint32_t find[SGX_FIND_TARGETS];   /* QY8_SGX_FIND */
    unsigned nfind;
    bool graph;                        /* QY8_SGX_GRAPH */

    /* --- M3-B1: PDS execution (QY8_SGX_EXEC) ------------------------- */

    bool exec;                  /* execute PDS programs instead of only decoding */

    /*
     * Microkernel secondary-attribute bank. This is MODEL state, not guest
     * memory: DOUTD fills it, and USE code (M3-B2) reads the base for
     * `stad [sa1,+#0]` from it. The guest cannot see this bank, so M3-B1 does
     * not change guest behavior.
     */
    uint32_t sa[SGX_SA_DWORDS];
    bool sa_known[SGX_SA_DWORDS];
    uint32_t sa_known_mask[SGX_SA_DWORDS];
    unsigned sa_count;

    /*
     * PRIMARY attribute bank, filled by the event handler's `MOVS DOUTA`,
     * which copies event flags from input registers here.
     */
    uint32_t pa[SGX_PA_DWORDS];
    bool pa_known[SGX_PA_DWORDS];
    uint32_t pa_known_mask[SGX_PA_DWORDS];
    unsigned pa_count;
    uint32_t sa_sbase;          /* SBASE from the last DOUTD, for validation */

    /* Entry points of USE tasks launched by DOUTU. */
    uint32_t use_queue[SGX_USE_QUEUE];
    unsigned nuse;
    unsigned nstores;           /* number of stad writes to guest memory */

    /* --- M5-C: core identity (QY8_SGX_CORE_REV / QY8_SGX_CORE_ID) --- */
    uint32_t core_rev;
    uint32_t core_id;
    bool core_id_warned;

    /* QY8_SGX_PDS_RUN — diagnostic launch of a named PDS program. */
    uint32_t run_va;
    uint32_t run_rows;
    uint32_t run_ir0;
    bool run_set;
};
typedef struct ClarionSgxState ClarionSgxState;

OBJECT_DECLARE_SIMPLE_TYPE(ClarionSgxState, CLARION_SGX)

/*
 * Trace output. Execution and report detail are separate: the guest can
 * issue EVENT_KICK2 many times, so execute the chain every time but print a
 * full decode only for the first QY8_SGX_KICKS events.
 */
static bool sgx_silent;

#define sgx_fprintf(...) \
    do { \
        if (!sgx_silent) { \
            fprintf(stderr, __VA_ARGS__); \
        } \
    } while (0)

static void G_GNUC_PRINTF(1, 2) sgx_pr(const char *fmt, ...)
{
    va_list ap;

    if (sgx_silent) {
        return;
    }
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

static uint32_t sgx_reg(ClarionSgxState *s, hwaddr off)
{
    return s->regs[off / 4];
}

static uint32_t sgx_edm_task_count(ClarionSgxState *s)
{
    uint64_t elapsed;

    if (!s->edm_task_register_model || !s->edm_timer_period_ns ||
        !(sgx_reg(s, SGX_CR_EVENT_TIMER) & SGX_CR_EVENT_TIMER_ENABLE)) {
        return 0;
    }

    elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->edm_timer_start_ns;
    return (uint32_t)(elapsed / s->edm_timer_period_ns);
}

static void sgx_set_reg(ClarionSgxState *s, hwaddr off, uint32_t val)
{
    if (s->regs_known_mask[off / 4] != UINT32_MAX ||
        s->regs[off / 4] != val) {
        s->regs_version[off / 4]++;
        s->regs_epoch++;
    }
    s->regs[off / 4] = val;
    s->regs_known_mask[off / 4] = UINT32_MAX;
}

static void sgx_event_timer_write(ClarionSgxState *s, uint32_t val)
{
    uint32_t ticks = val & SGX_CR_EVENT_TIMER_VALUE_MASK;

    s->edm_timer_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->edm_timer_period_ns =
        ((uint64_t)ticks * NANOSECONDS_PER_SECOND) / SGX_CORE_CLOCK_HZ;
}

/* Establish only justified bits; unrelated register bits retain their state. */
static void G_GNUC_UNUSED sgx_set_known_bits(ClarionSgxState *s, hwaddr off,
                                              uint32_t mask, uint32_t val)
{
    uint32_t i = off / 4;
    uint32_t next = (s->regs[i] & ~mask) | (val & mask);

    if ((s->regs_known_mask[i] & mask) != mask ||
        ((s->regs[i] ^ next) & mask)) {
        s->regs_version[i]++;
        s->regs_epoch++;
    }
    s->regs[i] = next;
    s->regs_known_mask[i] |= mask;
}

/* Target USE image writes only SW_EVENT. Reject unsupported bits before mutation. */
static bool sgx_write_event_status(ClarionSgxState *s, uint32_t val)
{
    if (val & ~SGX_CR_EVENT_STATUS_SW_EVENT) {
        return false;
    }
    if (val & SGX_CR_EVENT_STATUS_SW_EVENT) {
        sgx_set_known_bits(s, SGX_CR_EVENT_STATUS,
                           SGX_CR_EVENT_STATUS_SW_EVENT,
                           SGX_CR_EVENT_STATUS_SW_EVENT);
    }
    return true;
}

/*
 * Register-write side effects are the same regardless of writer: the guest
 * via MMIO or the microkernel via `str`. This handles MNE and PDS system-cache nodes.
 *
 * This is faithful because the model has no cache: "invalidate all" really
 * completes immediately and fully. The header describes the only way to
 * report this to the microkernel: set `INVAL` in `MNE_CR_EVENT_STATUS`, then
 * clear it by writing `MNE_CR_EVENT_CLEAR`. Otherwise the microkernel loops
 * forever in `ISLC_WaitForInvalidate`, making a model gap look like its own bug.
 *
 * ⚠ The guest does NOT see this bit: SGX-window reads still return zero
 * (see `sgx_read`), keeping A/B runs clean. The bit is for internal register
 * reads by USE code through `ldr`.
 */
static void sgx_reg_side_effects(ClarionSgxState *s, hwaddr off, uint32_t val)
{
    if (off == SGX_CR_EVENT_TIMER) {
        sgx_event_timer_write(s, val);
    }

    switch (off) {
    case SGX_CR_CACHE_CTRL:
        if (val & SGX_CR_CACHE_CTRL_INVALIDATE) {
            sgx_set_known_bits(s, SGX_CR_EVENT_STATUS,
                               SGX_CR_EVENT_STATUS_MADD_INVAL,
                               SGX_CR_EVENT_STATUS_MADD_INVAL);
        }
        break;
    case SGX_CR_TE_TPCCONTROL:
        if (val & SGX_CR_TE_TPCCONTROL_FLUSH) {
            sgx_set_known_bits(s, SGX_CR_EVENT_STATUS,
                               SGX_CR_EVENT_STATUS_TPC_FLUSH,
                               SGX_CR_EVENT_STATUS_TPC_FLUSH);
        }
        if (val & SGX_CR_TE_TPCCONTROL_CLEAR) {
            sgx_set_known_bits(s, SGX_CR_EVENT_STATUS,
                               SGX_CR_EVENT_STATUS_TPC_CLEAR,
                               SGX_CR_EVENT_STATUS_TPC_CLEAR);
        }
        break;
    case SGX_CR_EVENT_HOST_CLEAR: {
        uint32_t mask = val & SGX_CR_EVENT_STATUS_MODELED &
            s->regs_known_mask[SGX_CR_EVENT_STATUS / 4];
        if (mask) {
            sgx_set_known_bits(s, SGX_CR_EVENT_STATUS, mask, 0);
        }
        break;
    }
    case SGX_CR_PDS_INV0:
    case SGX_CR_PDS_INV1:
    case SGX_CR_PDS_INV3:
    case SGX_CR_PDS_INV_CSC:
        if (val & SGX_CR_PDS_INV_REQUEST) {
            uint32_t bit = off == SGX_CR_PDS_INV0
                ? SGX_CR_PDS_CACHE_STATUS_DSC_INV0
                : off == SGX_CR_PDS_INV1
                ? SGX_CR_PDS_CACHE_STATUS_DSC_INV1
                : off == SGX_CR_PDS_INV3
                ? SGX_CR_PDS_CACHE_STATUS_DSC_INV3
                : SGX_CR_PDS_CACHE_STATUS_CSC_INV;

            /* No modeled PDS cache: completion is immediate for level polling.
             * The physical latency and internal state machine are unknown.
             * First request establishes knownness; reset value stays unknown.
             */
            sgx_set_reg(s, SGX_CR_PDS_CACHE_STATUS,
                        sgx_reg(s, SGX_CR_PDS_CACHE_STATUS) | bit);
        }
        break;
    case SGX_CR_PDS_CACHE_HOST_CLEAR:
        if (s->regs_known_mask[SGX_CR_PDS_CACHE_STATUS / 4]) {
            sgx_set_reg(s, SGX_CR_PDS_CACHE_STATUS,
                        sgx_reg(s, SGX_CR_PDS_CACHE_STATUS) &
                        ~(val & SGX_CR_PDS_CACHE_STATUS_MASK));
        }
        break;
    case SGX_CR_MNE_CTRL:
        if (val & SGX_CR_MNE_CTRL_INVAL_ALL) {
            sgx_set_reg(s, SGX_CR_MNE_EVENT_STATUS,
                        sgx_reg(s, SGX_CR_MNE_EVENT_STATUS) |
                        SGX_CR_MNE_EVENT_STATUS_INVAL);
        }
        break;
    case SGX_CR_MNE_EVENT_CLEAR:
        if (val & SGX_CR_MNE_EVENT_CLEAR_INVAL) {
            sgx_set_reg(s, SGX_CR_MNE_EVENT_STATUS,
                        sgx_reg(s, SGX_CR_MNE_EVENT_STATUS) &
                        ~SGX_CR_MNE_EVENT_STATUS_INVAL);
        }
        break;
    default:
        break;
    }
}

/* --- SGX MMU traversal ------------------------------------------------ */

/*
 * Read a word from the guest's PHYSICAL memory, not our register mirror;
 * otherwise the report would confirm itself.
 */
static uint32_t sgx_phys_ld32(uint32_t pa)
{
    uint32_t v = 0;

    address_space_read(&address_space_memory, pa, MEMTXATTRS_UNSPECIFIED,
                       &v, sizeof(v));
    return le32_to_cpu(v);
}

/*
 * Write a word to the guest's PHYSICAL memory.
 *
 * ⚠ The only place where the model changes guest-visible state. Reached only
 * when the microkernel actually executes `stad`, and only with QY8_SGX_EXEC.
 * Nothing is written speculatively.
 */
static void sgx_phys_st32(uint32_t pa, uint32_t val)
{
    uint32_t v = cpu_to_le32(val);

    address_space_write(&address_space_memory, pa, MEMTXATTRS_UNSPECIFIED,
                        &v, sizeof(v));
}

/*
 * Same for a halfword. The 2-byte width and byte order are established:
 * DTYPE=16 in `sgxdefs.h`, IMG decoder/encoder use `ldaw`/`staw`, and on QY8
 * the low byte is at the lower address.
 *
 * ⚠ Exactly two bytes are written; the model does not touch the adjacent
 * halfword, so preserving neighboring data is not at issue.
 */
static uint32_t sgx_phys_ld16(uint32_t pa)
{
    uint16_t v = 0;

    address_space_read(&address_space_memory, pa, MEMTXATTRS_UNSPECIFIED,
                       &v, sizeof(v));
    return le16_to_cpu(v);
}

static void sgx_phys_st16(uint32_t pa, uint16_t val)
{
    uint16_t v = cpu_to_le16(val);

    address_space_write(&address_space_memory, pa, MEMTXATTRS_UNSPECIFIED,
                        &v, sizeof(v));
}

/*
 * Translate device VA to a physical address using the pd_pa directory.
 * Return false for an invalid PDE or PTE; do not guess a translation.
 */
static bool sgx_translate(uint32_t pd_pa, uint32_t va, uint32_t *pa_out)
{
    uint32_t pde = sgx_phys_ld32(pd_pa + 4 * (va >> SGX_MMU_PD_SHIFT));
    uint32_t pt_pa, pte, idx;

    if (!(pde & SGX_MMU_ENTRY_VALID)) {
        return false;
    }
    pt_pa = pde & SGX_MMU_ENTRY_ADDR_MASK;
    idx = (va >> SGX_MMU_PAGE_SHIFT) & (SGX_MMU_ENTRIES - 1);
    pte = sgx_phys_ld32(pt_pa + 4 * idx);
    if (!(pte & SGX_MMU_ENTRY_VALID)) {
        return false;
    }
    *pa_out = (pte & SGX_MMU_ENTRY_ADDR_MASK) | (va & (SGX_MMU_PAGE_SIZE - 1));
    return true;
}

static bool sgx_b2_read32(uint32_t pd, uint32_t dva, uint32_t *value,
                          uint32_t *pa)
{
    if ((dva & 3) || !sgx_translate(pd, dva, pa)) {
        return false;
    }
    *value = sgx_phys_ld32(*pa);
    return true;
}

static bool sgx_b2_prior_writer(ClarionSgxState *s, uint32_t pa,
                                unsigned size);
static void sgx_b2_record_store(ClarionSgxState *s, uint32_t dva,
                               uint32_t pa, uint32_t value, unsigned size,
                               bool synthetic);
static bool sgx_b2_model_store_conflict(ClarionSgxState *s, uint32_t dva,
                                        uint32_t pa, unsigned size);

/* S3/E2 read-only dispatch snapshot. Keep this outside the B2 decision path. */
static void sgx_e2_ta_dispatch(uint32_t pd, ClarionSgxState *s,
                               uint32_t context, uint32_t kick)
{
    uint32_t slot, base, ro, wo, ctxctl, ctxbase, ctlwo, ctlro;
    uint32_t pa, list, pending, sync, syncv;
    uint32_t off, size, cmd, ready, stop = 0;
    unsigned i, j;

    if (!s->nullrender) return;
    if (s->sa_known[SGX_SA_TA3DCTL] && s->sa_known[SGX_SA_CCBCTL] &&
        sgx_b2_read32(pd, s->sa[SGX_SA_TA3DCTL] + 0x88, &base, &slot) &&
        sgx_b2_read32(pd, s->sa[SGX_SA_CCBCTL], &wo, &pa) &&
        sgx_b2_read32(pd, s->sa[SGX_SA_CCBCTL] + 4, &ro, &pa)) {
        sgx_fprintf("{\"kind\":\"e2_ta_dispatch\",\"kick\":%u,"
                "\"kernel_ccb\":{\"slot_dva\":\"0x%08x\","
                "\"write_offset\":\"0x%08x\",\"read_offset\":\"0x%08x\",\"words\":[",
                kick, s->sa[SGX_SA_TA3DCTL] + 0x88, wo, ro);
        for (i = 0; i < 8; i++) {
            uint32_t w = 0;
            bool ok = sgx_b2_read32(pd, base + ro * 32 + i * 4, &w, &pa);
            sgx_fprintf("%s{\"ok\":%s,\"value\":\"0x%08x\"}",
                    i ? "," : "", ok ? "true" : "false", w);
        }
        sgx_fprintf("]},\"context\":\"0x%08x\",\"context_raw\":\"",
                context);
        for (i = 0; context && i < 0x48; i += 4) {
            uint32_t w = 0;
            bool ok = sgx_b2_read32(pd, context + i, &w, &pa);
            if (!ok) break;
            for (j = 0; j < 4; j++) {
                sgx_fprintf("%02x", (w >> (8 * j)) & 0xff);
            }
        }
        sgx_fprintf("\",\"ta_ccb\":");
        if (!context) {
            sgx_fprintf("null");
        } else if (sgx_b2_read32(pd, context + 0x10, &ctxctl, &pa) &&
            sgx_b2_read32(pd, context + 0x0c, &ctxbase, &pa) &&
            sgx_b2_read32(pd, ctxctl, &ctlwo, &pa) &&
            sgx_b2_read32(pd, ctxctl + 4, &ctlro, &pa)) {
            sgx_fprintf("{\"ctl_dva\":\"0x%08x\",\"base_dva\":\"0x%08x\","
                    "\"write_offset\":\"0x%08x\",\"read_offset\":\"0x%08x\",\"commands\":[",
                    ctxctl, ctxbase, ctlwo, ctlro);
            off = ctlro;
            if (ctlwo < ctlro) stop = 4;
            for (i = 0; !stop && off != ctlwo && i < 8; i++) {
                cmd = ctxbase + off;
                if (!sgx_b2_read32(pd, cmd, &size, &pa)) { stop = 5; break; }
                if (i && !sgx_silent) {
                    fputc(',', stderr);
                }
                sgx_fprintf("{\"offset\":\"0x%08x\",\"raw\":\"", off);
                for (j = 0; j < 0x60; j += 4) {
                    uint32_t w = 0;
                    if (!sgx_b2_read32(pd, cmd + j, &w, &pa)) break;
                    for (unsigned b = 0; b < 4; b++) {
                        sgx_fprintf("%02x", (w >> (8 * b)) & 0xff);
                    }
                }
                ready = list = pending = sync = syncv = 0;
                sgx_b2_read32(pd, cmd + 0x50, &ready, &pa);
                if (sgx_b2_read32(pd, cmd + 0x3c, &list, &pa) && list) {
                    sgx_b2_read32(pd, list + 0x10, &pending, &pa);
                    if (sgx_b2_read32(pd, list + 0x14, &sync, &pa) && sync)
                        sgx_b2_read32(pd, sync, &syncv, &pa);
                }
                sgx_fprintf("\",\"ready\":\"0x%08x\",\"list\":\"0x%08x\","
                        "\"pending\":\"0x%08x\",\"sync_dva\":\"0x%08x\","
                        "\"sync_word\":\"0x%08x\",\"size\":\"0x%08x\"}",
                        ready, list, pending, sync, syncv, size);
                if (!size) { stop = 1; break; }
                if (size & 3) { stop = 2; break; }
                if (off > ctlwo || size > ctlwo - off) { stop = 3; break; }
                off += size;
            }
            sgx_fprintf("],\"walk_stop\":%u,\"walk_end\":\"0x%08x\"}",
                        stop, off);
        } else {
            sgx_fprintf("null");
        }
        sgx_fprintf("}\n");
    } else {
        sgx_fprintf("{\"kind\":\"e2_ta_dispatch\",\"kick\":%u,"
                      "\"read_error\":true}\n", kick);
    }
}

static void sgx_b2_skip(uint32_t kick, const char *reason, uint32_t dva)
{
    static const char banner[] = "render completion is SYNTHETIC; no rasterization occurred; pixel contents are undefined";

    sgx_fprintf("{\"kind\":\"b2_skip\",\"kick\":%u,"
            "\"reason\":\"%s\",\"dva\":\"0x%08x\","
            "\"banner\":\"%s\"}\n", kick, reason, dva, banner);
}

/*
 * dst-sync rule for one TA command: READY, list, PendingVal + 1, and a
 * check of current complete. true allows traversal to continue (the word
 * was written or already completed); false fails closed and stops traversal.
 */
static bool sgx_b2_complete_cmd(ClarionSgxState *s, uint32_t pd,
                                uint32_t code_base, uint32_t kick,
                                uint32_t cmdta, const char *prefix,
                                const char *extra)
{
    static const char banner[] = "render completion is SYNTHETIC; no rasterization occurred; pixel contents are undefined";
    uint32_t cmdta_pa;
    uint32_t ready, ready_pa, list_dva = 0, pending, pending_pa;
    uint32_t dva, dva_pa, before, sync_pa, after;
    ClarionSgxSyntheticWord *own = NULL;
    char chain[2048];
    size_t used = 0;

    if (cmdta > UINT32_MAX - 0x50) {
        sgx_b2_skip(kick, "cmdta_address_overflow", cmdta);
        return false;
    }
    if (!sgx_b2_read32(pd, cmdta + 0x50, &ready, &ready_pa)) {
        sgx_b2_skip(kick, "cmdta_not_mapped", cmdta + 0x50);
        return false;
    }
    if (!(ready & 1)) {
        sgx_b2_skip(kick, "not_ready", cmdta + 0x50);
        return false;
    }
    if (!sgx_b2_read32(pd, cmdta + 0x3c, &list_dva, &cmdta_pa)) {
        sgx_b2_skip(kick, "list_pointer_unmapped", cmdta + 0x3c);
        return false;
    }
    if (!list_dva) {
        sgx_b2_skip(kick, "null_list", cmdta + 0x3c);
        return false;
    }
    if (list_dva > UINT32_MAX - 0x14) {
        sgx_b2_skip(kick, "list_address_overflow", list_dva);
        return false;
    }
    if (!sgx_b2_read32(pd, list_dva + 0x10, &pending, &pending_pa) ||
        !sgx_b2_read32(pd, list_dva + 0x14, &dva, &dva_pa)) {
        sgx_b2_skip(kick, "list_unmapped", list_dva);
        return false;
    }
    if (!dva) {
        sgx_b2_skip(kick, "null_sync_dva", list_dva + 0x14);
        return false;
    }
    if (!sgx_b2_read32(pd, dva, &before, &sync_pa)) {
        sgx_b2_skip(kick, "sync_unmapped", dva);
        return false;
    }
    /* word already ahead: the command is done; if behind, catch up to pending + 1 */
    if (before != pending && (int32_t)(pending - before) < 0) {
        sgx_b2_skip(kick, "unexpected_complete", dva);
        return true;
    }
    if (s->b2_store_overflow) {
        sgx_b2_skip(kick, "store_history_overflow", dva);
        return false;
    }
    for (unsigned i = 0; i < s->nb2_synthetic; i++) {
        if (s->b2_synthetic[i].valid && s->b2_synthetic[i].pa == sync_pa) {
            own = &s->b2_synthetic[i];
        }
    }
    /* our own word may move on: each later TA on the same sync */
    if (!own && sgx_b2_prior_writer(s, sync_pa, 4)) {
        sgx_b2_skip(kick, "prior_model_writer", dva);
        return false;
    }
    if (!own && s->nb2_synthetic >= SGX_B2_SYNTH_MAX) {
        sgx_b2_skip(kick, "synthetic_provenance_full", dva);
        return false;
    }
    after = pending + 1;
    sgx_phys_st32(sync_pa, after);
    if (own) {
        own->value = after;
    } else {
        sgx_b2_record_store(s, dva, sync_pa, after, 4, true);
        s->b2_synthetic[s->nb2_synthetic++] =
            (ClarionSgxSyntheticWord) { dva, sync_pa, after, true };
    }
    s->nstores++;
    used += snprintf(chain + used, sizeof(chain) - used, "%s", prefix);
    used += snprintf(chain + used, sizeof(chain) - used,
                     "{\"step\":\"CMDTA+0x50\",\"dva\":\"0x%08x\",\"pa\":\"0x%08x\",\"value\":\"0x%08x\"},",
                     cmdta + 0x50, ready_pa, ready);
    used += snprintf(chain + used, sizeof(chain) - used,
                     "{\"step\":\"CMDTA+0x3c\",\"dva\":\"0x%08x\",\"pa\":\"0x%08x\",\"value\":\"0x%08x\"},",
                     cmdta + 0x3c, cmdta_pa, list_dva);
    used += snprintf(chain + used, sizeof(chain) - used,
                     "{\"step\":\"list+0x10\",\"dva\":\"0x%08x\",\"pa\":\"0x%08x\",\"value\":\"0x%08x\"},",
                     list_dva + 0x10, pending_pa, pending);
    used += snprintf(chain + used, sizeof(chain) - used,
                     "{\"step\":\"list+0x14\",\"dva\":\"0x%08x\",\"pa\":\"0x%08x\",\"value\":\"0x%08x\"},",
                     list_dva + 0x14, dva_pa, dva);
    snprintf(chain + used, sizeof(chain) - used,
             "{\"step\":\"sync\",\"dva\":\"0x%08x\",\"pa\":\"0x%08x\",\"before\":\"0x%08x\",\"after\":\"0x%08x\"}]",
             dva, sync_pa, before, after);
    sgx_fprintf("{\"kind\":\"b2_write\",\"kick\":%u,"
            "\"class\":\"TA\",\"class_index\":0,\"image_entry\":\"0x%08x\","
            "\"chain\":%s,\"field_dva\":\"0x%08x\",\"field_pa\":\"0x%08x\","
            "\"before\":\"0x%08x\",\"after\":\"0x%08x\","
            "\"rule\":\"0x0e4206b8..0x0e4206e8\","
            "\"list_source\":\"CMDTA+0x3c directly; RD+0xa8 not observed live\","
            "\"synthetic\":true,%s\"banner\":\"%s\"}\n",
            kick, code_base + 0x728, chain, dva, sync_pa, before, after,
            extra, banner);
    return true;
}


/*
 * Minimal B2 at the target dispatcher. This consumes only live USE attributes
 * and the selected record/context/list chain; it never advances a queue.
 */
static void sgx_b2_dispatch(ClarionSgxState *s, uint32_t pd,
                            uint32_t code_base, uint32_t dispatch_pc,
                            uint32_t target, uint32_t context, uint32_t kick)
{
    uint32_t dispatch_pa, target_pa, word, record_base, read_offset;
    uint32_t kernel_ccb_slot, ccb_read_offset;
    uint32_t kernel_ccb_base, ccb_pa;
    uint32_t record_dva, record_pa, record_context;
    uint32_t ctl_dva, ctl_pa, base_dva;
    uint32_t ctx_ctl_pa, ctx_base_pa;
    uint32_t write_offset, ctl_wo_pa, off, size = 0, size_pa;
    unsigned n;
    char chain[1024];
    size_t used = 0;

    if (!s->nullrender || dispatch_pc != code_base + 0x458) {
        return;
    }
    if (s->b2_disabled) {
        sgx_b2_skip(kick, "image_mismatch", dispatch_pc);
        return;
    }
    if (!sgx_b2_read32(pd, code_base + 0x458, &word, &dispatch_pa) ||
        word != 0x00000200 ||
        !sgx_b2_read32(pd, code_base + 0x45c, &word, &target_pa) ||
        word != 0xf80000c0) {
        s->b2_disabled = true;
        sgx_b2_skip(kick, "image_mismatch", code_base + 0x458);
        return;
    }
    if (target != code_base + 0x728) {
        sgx_b2_skip(kick, "unsupported_class", target);
        return;
    }
    sgx_e2_ta_dispatch(pd, s, context, kick);
    if (!sgx_b2_read32(pd, code_base + 0x728, &word, &target_pa) ||
        word != 0x20200380 ||
        !sgx_b2_read32(pd, code_base + 0x72c, &word, &target_pa) ||
        word != 0x28011000 ||
        !sgx_b2_read32(pd, code_base + 0x730, &word, &target_pa) ||
        word != 0xa0404280 ||
        !sgx_b2_read32(pd, code_base + 0x734, &word, &target_pa) ||
        word != 0xe82b0000) {
        s->b2_disabled = true;
        sgx_b2_skip(kick, "image_mismatch", code_base + 0x728);
        return;
    }
    if (!context || !s->sa_known[SGX_SA_TA3DCTL] ||
        !s->sa_known[SGX_SA_CCBCTL]) {
        sgx_b2_skip(kick, "missing_live_attributes", context);
        return;
    }
    if (s->sa[SGX_SA_TA3DCTL] > UINT32_MAX - 0x88 ||
        s->sa[SGX_SA_CCBCTL] > UINT32_MAX - 4 ||
        !sgx_b2_read32(pd, s->sa[SGX_SA_TA3DCTL] + 0x88,
                       &kernel_ccb_base, &kernel_ccb_slot) ||
        !sgx_b2_read32(pd, s->sa[SGX_SA_CCBCTL] + 4,
                       &ccb_read_offset, &ccb_pa)) {
        sgx_b2_skip(kick, "live_ccb_inputs_unmapped", context);
        return;
    }
    if (ccb_read_offset > (UINT32_MAX - kernel_ccb_base) / 32) {
        sgx_b2_skip(kick, "record_address_overflow", kernel_ccb_base);
        return;
    }
    record_base = kernel_ccb_base;
    read_offset = ccb_read_offset;
    record_dva = record_base + read_offset * 32;
    if (record_dva > UINT32_MAX - 0x0c) {
        sgx_b2_skip(kick, "record_address_overflow", record_dva);
        return;
    }
    if (!sgx_b2_read32(pd, record_dva + 0x0c, &record_context,
                       &record_pa) || record_context != context) {
        sgx_b2_skip(kick, "record_context_unmapped_or_mismatch",
                    record_dva + 0x0c);
        return;
    }
    if (context > UINT32_MAX - 0x10 ||
        !sgx_b2_read32(pd, context + 0x10, &ctl_dva, &ctx_ctl_pa) ||
        !sgx_b2_read32(pd, context + 0x0c, &base_dva, &ctx_base_pa) ||
        ctl_dva > UINT32_MAX - 4 ||
        !sgx_b2_read32(pd, ctl_dva + 4, &read_offset, &ctl_pa)) {
        sgx_b2_skip(kick, "context_or_ctl_unmapped", context);
        return;
    }
    used += snprintf(chain + used, sizeof(chain) - used,
                     "[{\"step\":\"sa0+0x88 kernel CCB base\",\"dva\":\"0x%08x\",\"pa\":\"0x%08x\",\"value\":\"0x%08x\"},",
                     s->sa[SGX_SA_TA3DCTL] + 0x88, kernel_ccb_slot,
                     kernel_ccb_base);
    used += snprintf(chain + used, sizeof(chain) - used,
                     "{\"step\":\"sa2+4 ReadOffset\",\"dva\":\"0x%08x\",\"pa\":\"0x%08x\",\"value\":\"0x%08x\"},",
                     s->sa[SGX_SA_CCBCTL] + 4, ccb_pa, ccb_read_offset);
    used += snprintf(chain + used, sizeof(chain) - used,
                     "{\"step\":\"record+0x0c\",\"dva\":\"0x%08x\",\"pa\":\"0x%08x\",\"value\":\"0x%08x\"},",
                     record_dva + 0x0c, record_pa, record_context);
    used += snprintf(chain + used, sizeof(chain) - used,
                     "{\"step\":\"ctx+0x10\",\"dva\":\"0x%08x\",\"pa\":\"0x%08x\",\"value\":\"0x%08x\"},",
                     context + 0x10, ctx_ctl_pa, ctl_dva);
    used += snprintf(chain + used, sizeof(chain) - used,
                     "{\"step\":\"ctx+0x0c\",\"dva\":\"0x%08x\",\"pa\":\"0x%08x\",\"value\":\"0x%08x\"},",
                     context + 0x0c, ctx_base_pa, base_dva);
    used += snprintf(chain + used, sizeof(chain) - used,
                     "{\"step\":\"ctl+4\",\"dva\":\"0x%08x\",\"pa\":\"0x%08x\",\"value\":\"0x%08x\"},",
                     ctl_dva + 4, ctl_pa, read_offset);
    if (!s->nullrender_all) {
        /*
         * Nothing advances ReadOffset (there is no microkernel), so it always
         * points at the first command; take the newest, just before WriteOffset.
         */
        if (sgx_b2_read32(pd, ctl_dva, &write_offset, &ctl_wo_pa) &&
            write_offset >= read_offset + SGX_B2_CMDTA_SIZE) {
            read_offset = write_offset - SGX_B2_CMDTA_SIZE;
        }
        if (read_offset > UINT32_MAX - base_dva) {
            sgx_b2_skip(kick, "cmdta_address_overflow", base_dva);
            return;
        }
        sgx_b2_complete_cmd(s, pd, code_base, kick, base_dva + read_offset,
                            chain, "");
        return;
    }
    /*
     * In all mode, walk commands from ReadOffset to WriteOffset without
     * writing ctl. Stop traversal on any unexpected condition.
     */
    if (!sgx_b2_read32(pd, ctl_dva, &write_offset, &ctl_wo_pa)) {
        sgx_b2_skip(kick, "ctl_write_offset_unmapped", ctl_dva);
        return;
    }
    if (write_offset < read_offset) {
        sgx_b2_skip(kick, "ta_ccb_wrapped", ctl_dva);
        return;
    }
    for (off = read_offset, n = 0; off != write_offset; off += size, n++) {
        char extra[96];

        if (n >= SGX_B2_WALK_MAX) {
            sgx_b2_skip(kick, "walk_limit", base_dva + off);
            return;
        }
        if (off > UINT32_MAX - base_dva) {
            sgx_b2_skip(kick, "cmdta_address_overflow", base_dva);
            return;
        }
        if (!sgx_b2_read32(pd, base_dva + off, &size, &size_pa)) {
            sgx_b2_skip(kick, "cmdta_size_unmapped", base_dva + off);
            return;
        }
        if (size != SGX_B2_CMDTA_SIZE) {
            sgx_b2_skip(kick, "unexpected_command_size", base_dva + off);
            return;
        }
        if (size > write_offset - off) {
            sgx_b2_skip(kick, "command_past_write_offset", base_dva + off);
            return;
        }
        snprintf(extra, sizeof(extra),
                 "\"mode\":\"all\",\"ta_ccb_offset\":\"0x%08x\","
                 "\"ta_ccb_write_offset\":\"0x%08x\",", off, write_offset);
        if (!sgx_b2_complete_cmd(s, pd, code_base, kick, base_dva + off,
                                 chain, extra)) {
            return;
        }
    }
}

static bool sgx_b2_synthetic_pa(ClarionSgxState *s, uint32_t pa,
                                uint32_t *value)
{
    unsigned i;

    for (i = 0; i < s->nb2_synthetic; i++) {
        ClarionSgxSyntheticWord *sw = &s->b2_synthetic[i];
        if (sw->valid && sw->pa == pa) {
            *value = sw->value;
            return true;
        }
    }
    return false;
}

static bool sgx_b2_store_overlap(uint32_t pa, unsigned size,
                                 uint32_t other_pa, unsigned other_size)
{
    return (uint64_t)pa < (uint64_t)other_pa + other_size &&
           (uint64_t)other_pa < (uint64_t)pa + size;
}

static bool sgx_b2_prior_writer(ClarionSgxState *s, uint32_t pa,
                                unsigned size)
{
    unsigned i;

    if (s->b2_store_overflow) {
        return true;
    }
    for (i = 0; i < s->nb2_stores; i++) {
        ClarionSgxGuestStore *st = &s->b2_stores[i];
        if (sgx_b2_store_overlap(pa, size, st->pa, st->size)) {
            return true;
        }
    }
    return false;
}

static void sgx_b2_record_store(ClarionSgxState *s, uint32_t dva,
                               uint32_t pa, uint32_t value, unsigned size,
                               bool synthetic)
{
    if (!s->nullrender) {
        return;
    }
    if (s->nb2_stores >= SGX_B2_STORE_HISTORY) {
        s->b2_store_overflow = true;
        return;
    }
    s->b2_stores[s->nb2_stores++] = (ClarionSgxGuestStore) {
        dva, pa, value, size, synthetic
    };
}

static bool sgx_b2_model_store_conflict(ClarionSgxState *s, uint32_t dva,
                                        uint32_t pa, unsigned size)
{
    unsigned i;

    for (i = 0; i < s->nb2_synthetic; i++) {
        ClarionSgxSyntheticWord *sw = &s->b2_synthetic[i];
        if (sw->valid && sgx_b2_store_overlap(pa, size, sw->pa, 4)) {
            sgx_fprintf(
                    "{\"kind\":\"b2_conflict\",\"kick\":%u,"
                    "\"dva\":\"0x%08x\",\"pa\":\"0x%08x\","
                    "\"reason\":\"model_store_overlaps_synthetic_completion\"}\n",
                    s->kicks, dva, pa);
            return true;
        }
    }
    return false;
}

/* List valid PDEs and the number of mapped pages in each. */
static void sgx_report_pd(uint32_t pd_pa)
{
    unsigned i, total = 0, ndir = 0;

    for (i = 0; i < SGX_MMU_ENTRIES; i++) {
        uint32_t pde = sgx_phys_ld32(pd_pa + 4 * i);
        uint32_t pt_pa;
        unsigned j, n = 0;

        if (!(pde & SGX_MMU_ENTRY_VALID)) {
            continue;
        }
        pt_pa = pde & SGX_MMU_ENTRY_ADDR_MASK;
        for (j = 0; j < SGX_MMU_ENTRIES; j++) {
            if (sgx_phys_ld32(pt_pa + 4 * j) & SGX_MMU_ENTRY_VALID) {
                n++;
            }
        }
        ndir++;
        total += n;
        sgx_fprintf("[sgx]   PDE[%4u] = %08x  table PA %08x  "
                "VA %08x..%08x  pages %u\n",
                i, pde, pt_pa, i << SGX_MMU_PD_SHIFT,
                (i << SGX_MMU_PD_SHIFT) + 0x3FFFFF, n);
    }
    sgx_fprintf("[sgx]   valid PDEs %u, mapped pages %u\n",
            ndir, total);
}

/* Show a GPU-VA range. Cap len to keep the report bounded. */
static void sgx_report_range(uint32_t pd_pa, const char *what,
                             uint32_t va, uint32_t len)
{
    uint32_t off;

    len = MIN(len, 0x200);
    sgx_fprintf("[sgx]   %s GPU VA %08x, %u bytes:\n", what, va, len);
    for (off = 0; off < len; off += 16) {
        uint32_t pa, w[4];
        unsigned k, n = MIN(4, (len - off + 3) / 4);

        if (!sgx_translate(pd_pa, va + off, &pa)) {
            sgx_fprintf("[sgx]     +0x%03x: unmapped\n", off);
            continue;
        }
        for (k = 0; k < n; k++) {
            w[k] = sgx_phys_ld32(pa + 4 * k);
        }
        sgx_fprintf("[sgx]     +0x%03x (PA %08x):", off, pa);
        for (k = 0; k < n; k++) {
            sgx_fprintf(" %08x", w[k]);
        }
        sgx_fprintf("\n");
    }
}

/* --- search tools (diagnostic; do not affect model behavior) ----------- */

/*
 * Possible encodings for an address written to SGX. Two are established by
 * a run: `EVENT_KICKER` holds a device VA directly, while register 0x0A68
 * holds a byte offset from `PDS_EXEC_BASE`. The others are hypotheses to
 * check, not assumptions; this is why this is a diagnostic tool, not a mechanism.
 */
typedef struct ClarionSgxEnc {
    const char *name;
    uint32_t value;
    bool valid;
} ClarionSgxEnc;

static unsigned sgx_encodings(uint32_t target, uint32_t pds, uint32_t use,
                              ClarionSgxEnc *out, unsigned max)
{
    unsigned n = 0;

    if (n < max) { out[n++] = (ClarionSgxEnc){"absolute", target, true}; }
    if (pds && target >= pds) {
        uint32_t d = target - pds;
        if (n < max) { out[n++] = (ClarionSgxEnc){"PDS-rel", d, true}; }
        if (n < max) { out[n++] = (ClarionSgxEnc){"PDS-rel>>2", d >> 2, true}; }
        if (n < max) { out[n++] = (ClarionSgxEnc){"PDS-rel>>4", d >> 4, true}; }
    }
    if (use && target >= use) {
        uint32_t d = target - use;
        if (n < max) { out[n++] = (ClarionSgxEnc){"USE-rel", d, true}; }
        if (n < max) { out[n++] = (ClarionSgxEnc){"USE-rel>>2", d >> 2, true}; }
        if (n < max) { out[n++] = (ClarionSgxEnc){"USE-rel>>4", d >> 4, true}; }
    }
    return n;
}

/*
 * QY8_SGX_FIND=VA — find references to this device VA. Inspect recorded
 * register writes and every aligned word in all mapped pages. This does not
 * find anything for the model itself; it reports findings to the engineer.
 */
static void sgx_find_target(ClarionSgxState *s, uint32_t pd, uint32_t pds,
                            uint32_t use, uint32_t target)
{
    ClarionSgxEnc enc[8];
    unsigned nenc = sgx_encodings(target, pds, use, enc, ARRAY_SIZE(enc));
    uint32_t field = use && target >= use ? ((target - use) >> 4) & 0xFFF : 0;
    unsigned i, hits = 0;

    sgx_fprintf("[sgx]   FIND %08x — encodings:", target);
    for (i = 0; i < nenc; i++) {
        sgx_fprintf(" %s=%08x", enc[i].name, enc[i].value);
    }
    sgx_fprintf("\n");

    for (i = 0; i < SGX_NREGS; i++) {
        unsigned k;

        if (!s->regs[i]) {
            continue;
        }
        for (k = 0; k < nenc; k++) {
            if (enc[k].value && s->regs[i] == enc[k].value) {
                sgx_fprintf("[sgx]     register +0x%04x = %08x  (%s)\n",
                        i * 4, s->regs[i], enc[k].name);
                hits++;
            }
        }
    }

    for (i = 0; i < SGX_MMU_ENTRIES; i++) {
        uint32_t pde = sgx_phys_ld32(pd + 4 * i);
        uint32_t pt_pa;
        unsigned j;

        if (!(pde & SGX_MMU_ENTRY_VALID)) {
            continue;
        }
        pt_pa = pde & SGX_MMU_ENTRY_ADDR_MASK;
        for (j = 0; j < SGX_MMU_ENTRIES; j++) {
            uint32_t pte = sgx_phys_ld32(pt_pa + 4 * j);
            uint32_t page_va = (i << SGX_MMU_PD_SHIFT) | (j << SGX_MMU_PAGE_SHIFT);
            uint32_t pa;
            unsigned o;

            if (!(pte & SGX_MMU_ENTRY_VALID)) {
                continue;
            }
            pa = pte & SGX_MMU_ENTRY_ADDR_MASK;
            for (o = 0; o < SGX_MMU_PAGE_SIZE; o += 4) {
                uint32_t w = sgx_phys_ld32(pa + o);
                unsigned k;

                if (!w) {
                    continue;
                }
                for (k = 0; k < nenc; k++) {
                    if (enc[k].value && w == enc[k].value) {
                        sgx_fprintf("[sgx]     GPU memory VA %08x = %08x"
                                "  (%s)\n", page_va + o, w, enc[k].name);
                        hits++;
                    }
                }
                /*
                 * Weaker 12-bit form: field [15:4] in a pair like
                 * 0x0020XXYF/0x04000000. Report separately and label it weak;
                 * 12 bits alone do not prove a match.
                 */
                if (field && (w & 0xFFFF000F) == 0x0020000F &&
                    ((w >> 4) & 0xFFF) == field) {
                    sgx_fprintf("[sgx]     GPU memory VA %08x = %08x"
                            "  (field[15:4]=%03x, WEAK match)\n",
                            page_va + o, w, field);
                    hits++;
                }
            }
        }
    }
    sgx_fprintf("[sgx]     matches: %u\n", hits);
}

/*
 * QY8_SGX_GRAPH=1 — each aligned word in mapped memory that is itself a
 * valid device VA in the current directory, forming an edge in the pointer
 * graph. This reveals references that have not yet been queried directly.
 */
static void sgx_report_graph(uint32_t pd)
{
    unsigned i, edges = 0;

    sgx_fprintf("[sgx]   pointer graph (word = valid device VA):\n");
    for (i = 0; i < SGX_MMU_ENTRIES; i++) {
        uint32_t pde = sgx_phys_ld32(pd + 4 * i);
        uint32_t pt_pa;
        unsigned j;

        if (!(pde & SGX_MMU_ENTRY_VALID)) {
            continue;
        }
        pt_pa = pde & SGX_MMU_ENTRY_ADDR_MASK;
        for (j = 0; j < SGX_MMU_ENTRIES; j++) {
            uint32_t pte = sgx_phys_ld32(pt_pa + 4 * j);
            uint32_t page_va = (i << SGX_MMU_PD_SHIFT) | (j << SGX_MMU_PAGE_SHIFT);
            uint32_t pa, tgt;
            unsigned o;

            if (!(pte & SGX_MMU_ENTRY_VALID)) {
                continue;
            }
            pa = pte & SGX_MMU_ENTRY_ADDR_MASK;
            for (o = 0; o < SGX_MMU_PAGE_SIZE; o += 4) {
                uint32_t w = sgx_phys_ld32(pa + o);

                /* Four-byte alignment filters out most code. */
                if (!w || (w & 3) || !sgx_translate(pd, w, &tgt)) {
                    continue;
                }
                sgx_fprintf("[sgx]     %08x -> %08x (PA %08x)\n",
                        page_va + o, w, tgt);
                edges++;
            }
        }
    }
    sgx_fprintf("[sgx]     edges: %u\n", edges);
}

/* --- PDS program: decoding and execution (public DDK, SGX540) ---------- */

/*
 * PDS program execution state.
 *
 * The datastore has two banks of PDS_DATASTORE_PERBANKSIZE doublewords.
 * Indices 0..47 are constants physically stored in the program data segment;
 * 48..63 are temporaries that do not exist in memory. Both layouts follow
 * `PDSGetDS0ConstantOffset`/`PDSGetDS1ConstantOffset` from `codegen/pds/pds.c`.
 */
typedef struct ClarionPdsCtx {
    ClarionSgxState *s;
    uint32_t pd;                /* SGX page directory */
    uint32_t data_va;           /* data segment base = object base */
    uint32_t ndwords;           /* data segment size in doublewords */
    uint32_t code_va;           /* code segment base */
    uint32_t use_base[16];      /* decoded EUR_CR_USE_CODE_BASE_0..15 */

    bool exec;                  /* execute, rather than only decode */

    uint32_t temp[2][PDS_DATASTORE_PERBANKSIZE];
    bool temp_known[2][PDS_DATASTORE_PERBANKSIZE];

    bool pred[3];               /* p0..p2 */
    bool pred_known[3];
    uint32_t ir[2];             /* task input registers */
    bool ir_known[2];

    unsigned depth;             /* PDS -> USE -> PDS nesting depth */
    const char *stop;           /* reason execution stopped, or NULL */
} ClarionPdsCtx;

/*
 * PDS and USE launch each other: `MOVS DOUTU` starts a USE task, and its
 * `emitpds` launches the next PDS program. This requires mutual recursion
 * and a forward declaration.
 */
static void sgx_use_run(ClarionSgxState *s, uint32_t pd, uint32_t code_base,
                        uint32_t page_base, unsigned cbase, unsigned coff,
                        uint32_t entry, unsigned depth);

/*
 * Offset of constant ds<bank>[k] in the data segment, in doublewords. This
 * follows
 * `PDSGetDS0ConstantOffset()`/`PDSGetDS1ConstantOffset()` from
 * `eurasia/codegen/pds/pds.c`:
 *
 *     row    = k / PDS_NUM_DWORDS_PER_ROW;
 *     column = k % PDS_NUM_DWORDS_PER_ROW;
 *     ds0: (2 * row)     * PDS_NUM_DWORDS_PER_ROW + column;
 *     ds1: (2 * row + 1) * PDS_NUM_DWORDS_PER_ROW + column;
 *
 * On SGX540, `PDS_NUM_DWORDS_PER_ROW = 2`, so ds0/ds1 banks alternate in
 * pairs of doublewords. Thus ds0[2] is at word 4, not 2.
 */
static uint32_t pds_ds_dword(unsigned bank, uint32_t k)
{
    uint32_t row = k / PDS_NUM_DWORDS_PER_ROW;
    uint32_t col = k % PDS_NUM_DWORDS_PER_ROW;

    return (2 * row + (bank ? 1 : 0)) * PDS_NUM_DWORDS_PER_ROW + col;
}

/*
 * Read ds<bank>[k]. Constants come from the data segment; temporaries come
 * from our state. If a value is unknown, return false and do not guess; the
 * caller stops and reports the reason.
 */
static bool pds_ds_read(ClarionPdsCtx *c, unsigned bank, uint32_t k,
                        uint32_t *out)
{
    uint32_t dw, pa;

    if (bank > 1 || k >= PDS_DATASTORE_PERBANKSIZE) {
        return false;
    }
    if (k >= PDS_DATASTORE_TEMPSTART) {
        if (!c->temp_known[bank][k]) {
            return false;
        }
        *out = c->temp[bank][k];
        return true;
    }
    dw = pds_ds_dword(bank, k);
    if (dw >= c->ndwords || !sgx_translate(c->pd, c->data_va + 4 * dw, &pa)) {
        return false;
    }
    *out = sgx_phys_ld32(pa);
    return true;
}

/*
 * Write ds<bank>[k]. Only temporaries are writable: constants reside in
 * guest memory, which the model does not modify (M3-B1 is guest-transparent).
 */
static bool pds_ds_write(ClarionPdsCtx *c, unsigned bank, uint32_t k,
                         uint32_t val)
{
    if (bank > 1 || k < PDS_DATASTORE_TEMPSTART ||
        k >= PDS_DATASTORE_PERBANKSIZE) {
        return false;
    }
    c->temp[bank][k] = val;
    c->temp_known[bank][k] = true;
    return true;
}

/*
 * Whether an instruction passes its cc field (`sgxdefs.h:2653..2662`).
 * Set `known` false when the condition depends on unknown state; execution
 * then stops instead of guessing the branch.
 */
static bool pds_cc_true(ClarionPdsCtx *c, uint32_t cc, bool *known)
{
    *known = true;
    switch (cc) {
    case PDS_CC_ALWAYS:
        return true;
    case PDS_CC_P0:
    case PDS_CC_P1:
    case PDS_CC_P2:
        *known = c->pred_known[cc];
        return c->pred[cc];
    default:
        /*
         * IF0/IF1 (external interface state) and ALUZ/ALUN (arithmetic flags)
         * do not occur in this task queue. Do not guess their values.
         */
        *known = false;
        return false;
    }
}

/* `EUR_CR_USE_CODE_BASE(x)`: ADDR field is bits 23:0; device VA = ADDR << 8. */
static uint32_t pds_use_code_base(ClarionSgxState *s, unsigned i)
{
    uint32_t v = sgx_reg(s, SGX_CR_USE_CODE_BASE(i));

    return (v & SGX_USE_CODE_BASE_ADDR_MASK) << 8;
}

/*
 * DOUTD — DMA into the secondary-attribute bank.
 *
 * Fields are in `sgxdefs.h:4211..4275`, the non-SGX545/543 variant: BSIZE
 * bits 3:0, BLINES 7:4, AO 18:8, INSTR 20:19, STRIDE 29:21, STYPE bit 30.
 * All three sizes are encoded as "minus one", as shown by our program's
 * builder at `srvinit/devices/sgx/sgxinit.c:1961..1973`, which writes
 * `ui32DMABurstSize - 1`, `ui32DMABurstLines - 1`, and again
 * `ui32DMABurstSize - 1` for STRIDE.
 *
 * Source is `SBASE` (device VA); destination is the secondary-attribute
 * doubleword at AO. Transfer BLINES rows, each with BSIZE doublewords and a
 * STRIDE-doubleword step between rows.
 */
static void pds_doutd(ClarionPdsCtx *c, const uint32_t *emit, unsigned n)
{
    uint32_t sbase, ctl, bsize, blines, ao, stride, instr, bytes, pa;
    unsigned line, i, copied = 0;

    if (n < PDS_NUM_DMA_CONTROL_WORDS) {
        sgx_pr("[sgx]       DOUTD: operands unresolved\n");
        c->stop = "DOUTD operands unresolved";
        return;
    }
    sbase = emit[0];
    ctl = emit[1];
    bsize = (ctl & PDS_DOUTD1_BSIZE_MASK) + 1;
    blines = ((ctl >> PDS_DOUTD1_BLINES_SHIFT) & PDS_DOUTD1_BLINES_MASK) + 1;
    ao = (ctl >> PDS_DOUTD1_AO_SHIFT) & PDS_DOUTD1_AO_MASK;
    stride = ((ctl >> PDS_DOUTD1_STRIDE_SHIFT) & PDS_DOUTD1_STRIDE_MASK) + 1;
    instr = (ctl >> PDS_DOUTD1_INSTR_SHIFT) & 3;
    bytes = bsize * blines * 4;

    sgx_pr("[sgx]       DOUTD: SBASE=%08x  DOUTD1=%08x\n", sbase, ctl);
    sgx_pr("[sgx]              BSIZE=%u BLINES=%u AO=%u STRIDE=%u"
            " INSTR=%u STYPE=%u -> %u bytes from %08x\n", bsize, blines, ao, stride,
            instr, !!(ctl & PDS_DOUTD1_STYPE), bytes, sbase);
    if (!sgx_translate(c->pd, sbase, &pa)) {
        sgx_pr("[sgx]              ⚠ DMA source is NOT mapped in this page directory\n");
        if (c->exec) {
            c->stop = "DOUTD source is unmapped";
        }
        return;
    }
    sgx_pr("[sgx]              DMA source mapped: PA %08x\n", pa);

    if (!c->exec) {
        return;
    }
    if (instr != PDS_DOUTD1_INSTR_NORMAL) {
        sgx_pr("[sgx]              ⚠ INSTR=%u is not NORMAL; not executing\n",
                instr);
        c->stop = "DOUTD mode is not NORMAL";
        return;
    }
    if (ao + bsize * blines > SGX_SA_DWORDS) {
        sgx_pr("[sgx]              ⚠ DMA exceeds the attribute bank\n");
        c->stop = "DOUTD exceeds secondary-attribute bank";
        return;
    }

    c->s->sa_sbase = sbase;
    for (line = 0; line < blines; line++) {
        for (i = 0; i < bsize; i++) {
            uint32_t src_va = sbase + 4 * (line * stride + i);
            uint32_t src_pa;

            if (!sgx_translate(c->pd, src_va, &src_pa)) {
                sgx_pr("[sgx]              ⚠ row %u word %u: VA %08x"
                        " is unmapped; DMA stopped\n", line, i, src_va);
                c->stop = "unmapped DOUTD source";
                return;
            }
            c->s->sa[ao + line * bsize + i] = sgx_phys_ld32(src_pa);
            c->s->sa_known[ao + line * bsize + i] = true;
            copied++;
        }
    }
    if (ao + copied > c->s->sa_count) {
        c->s->sa_count = ao + copied;
    }
    sgx_pr("[sgx]              ✔ EXECUTED: %u doublewords -> sa[%u..%u]\n",
            copied, ao, ao + copied - 1);
}

/*
 * DOUTA writes to the USE task's primary-attribute bank.
 *
 * Word 0 contains data; word 1 carries `AO` (bank offset in doublewords).
 * Fields are in `sgxdefs.h:3654..3662`. This is the first thing the
 * microkernel event handler does: place `ir0` and `ir1` where USE code reads them
 * (`eventhandler.pds.asm:84..85`).
 */
static void pds_douta(ClarionPdsCtx *c, const uint32_t *emit, unsigned n)
{
    uint32_t data, ctl, ao;

    if (n < PDS_NUM_ATTRIB_CONTROL_WORDS) {
        sgx_pr("[sgx]       DOUTA: operands unresolved\n");
        if (c->exec) {
            c->stop = "DOUTA operands unresolved";
        }
        return;
    }
    data = emit[0];
    ctl = emit[1];
    ao = (ctl >> PDS_DOUTA1_AO_SHIFT) & PDS_DOUTA1_AO_MASK;

    sgx_pr("[sgx]       DOUTA: DATA=%08x DOUTA1=%08x -> pa[%u]\n",
           data, ctl, ao);
    if (!c->exec) {
        return;
    }
    if (ao >= SGX_PA_DWORDS) {
            c->stop = "DOUTA exceeds primary-attribute bank";
        return;
    }
    c->s->pa[ao] = data;
    c->s->pa_known[ao] = true;
    if (ao + 1 > c->s->pa_count) {
        c->s->pa_count = ao + 1;
    }
    sgx_pr("[sgx]              ✔ EXECUTED: pa[%u] = %08x\n", ao, data);
}

/*
 * DOUTU launches a USE task. The interpreter executes supported instructions
 * and stops unsupported paths with an explicit reason. B2 runs only at the
 * proven dispatcher point and only with QY8_SGX_NULLRENDER=1.
 */
static void pds_doutu(ClarionPdsCtx *c, const uint32_t *emit, unsigned n)
{
    uint32_t w0, w1, cbase, coff, exe, exeaddr, va;

    if (n < PDS_NUM_USE_TASK_CONTROL_WORDS) {
        sgx_pr("[sgx]       DOUTU: operands unresolved\n");
        if (c->exec) {
            c->stop = "DOUTU operands unresolved";
        }
        return;
    }
    w0 = emit[0];
    w1 = emit[1];
    cbase = w0 & PDS_DOUTU0_CBASE_MASK;
    coff = (w0 >> PDS_DOUTU0_COFF_SHIFT) & PDS_DOUTU0_COFF_MASK;
    exe = (w0 >> PDS_DOUTU0_EXE_SHIFT) & PDS_DOUTU0_EXE_MASK;
    exeaddr = (coff << PDS_DOUTU0_COFF_ALIGNSHIFT) |
              (exe << PDS_DOUTU0_EXE_ALIGNSHIFT);
    va = c->use_base[cbase] + exeaddr;

    sgx_pr("[sgx]       DOUTU: %08x %08x %08x\n", w0, w1, emit[2]);
    sgx_pr("[sgx]              CBASE=%u -> USE_CODE_BASE_%u=%08x;"
            " COFF=%x EXE=%03x -> offset 0x%05x\n",
            cbase, cbase, c->use_base[cbase], coff, exe, exeaddr);
    sgx_pr("[sgx]              ➜ USE TASK ENTRY POINT: device VA %08x%s\n",
            va, (w0 & PDS_DOUTU0_PDSDMADEP) ? "  [PDSDMADEPENDENCY]" : "");
    sgx_pr("[sgx]              MODE=%s\n",
            (w1 >> PDS_DOUTU1_MODE_SHIFT) & 1 ? "PERINSTANCE" : "PARALLEL");

    if (!c->exec) {
        return;
    }
    /*
     * PDSDMADEPENDENCY means "the task waits for this program's DMA to finish".
     * Our DOUTD is synchronous and has completed, so the dependency is met by
     * construction; report this explicitly.
     */
    if (c->s->nuse < SGX_USE_QUEUE) {
        c->s->use_queue[c->s->nuse++] = va;
    }
    sgx_pr("[sgx]              ✔ LAUNCH USE task @%08x\n", va);
    sgx_use_run(c->s, c->pd, c->use_base[cbase],
                c->use_base[cbase] + (coff << PDS_DOUTU0_COFF_ALIGNSHIFT),
                cbase, coff, va, c->depth + 1);
}

/*
 * Walk a PDS program, decoding every instruction and executing its effects
 * when QY8_SGX_EXEC is enabled.
 *
 * This intentionally uses one pass, not two, so the trace shows exactly what
 * the model did. Without QY8_SGX_EXEC, behavior is unchanged: decode only,
 * with no side effects.
 *
 * Control flow is real: cc gates each instruction, TSTZ/TSTN set predicates,
 * and BRA/CALL/RTN update the counter. Unsupported behavior is never guessed;
 * execution stops at the first unknown instruction or condition with a reason.
 */
static void sgx_pds_run(ClarionSgxState *s, uint32_t pd, uint32_t data_va,
                        uint32_t rows, const char *what, bool have_ir,
                        uint32_t ir0, uint32_t ir1, unsigned depth)
{
    ClarionPdsCtx c;
    uint32_t pc = 0, i;
    uint32_t link[8];
    unsigned nlink = 0, steps = 0;
    unsigned n;

    memset(&c, 0, sizeof(c));
    c.s = s;
    c.pd = pd;
    c.data_va = data_va;
    /* A row occupies PDS_NUM_DWORDS_PER_ROW doublewords in EACH of the two banks. */
    c.ndwords = rows * 2 * PDS_NUM_DWORDS_PER_ROW;
    c.code_va = data_va + 4 * c.ndwords;
    c.exec = s->exec;
    c.depth = depth;
    for (i = 0; i < 16; i++) {
        c.use_base[i] = pds_use_code_base(s, i);
    }
    if (have_ir) {
        c.ir[0] = ir0;
        c.ir_known[0] = true;
        c.ir[1] = ir1;
        c.ir_known[1] = true;
    }

    if (depth > SGX_MAX_DEPTH) {
        sgx_pr("[sgx]   ⛔ PDS/USE depth > %u; stopped\n",
                SGX_MAX_DEPTH);
        return;
    }
    sgx_pr("[sgx]   %*s%s PDS program (%s):\n", 2 * depth, "",
            c.exec ? "EXECUTION" : "decode", what);
    sgx_pr("[sgx]     data segment %08x, %u rows -> %u doublewords"
            " (%u bytes)\n", c.data_va, rows, c.ndwords, 4 * c.ndwords);
    sgx_pr("[sgx]     code at %08x\n", c.code_va);
    if (have_ir) {
        sgx_pr("[sgx]     ir0 = %08x  ir1 = %08x%s\n", ir0, ir1,
               (ir1 & PDS_IR1_EDM_EVENT_SWEVENT) ? "  [SWEVENT]" : "");
    }

    /*
     * ⚠ The window was 0x100 bytes and 256 steps, enough for init programs,
     * but the event handler jumps to +0x220 (the ClearLine chain), causing a
     * silent exit. Silent exits are undesirable, so the limit is explicit and
     * reaching it is reported.
     */
    while (steps++ < SGX_PDS_MAX_STEPS) {
        if (pc >= SGX_PDS_MAX_PROG) {
            sgx_pr("[sgx]     +0x%02x: outside decode window (%#x)\n", pc,
                   SGX_PDS_MAX_PROG);
            c.stop = "outside PDS program window";
            break;
        }
        uint32_t pa, w, group, type, cc;
        bool cc_known, taken;

        if (!sgx_translate(pd, c.code_va + pc, &pa)) {
            sgx_pr("[sgx]     +0x%02x: unmapped; stopped\n", pc);
            c.stop = "code is unmapped";
            break;
        }
        w = sgx_phys_ld32(pa);
        group = (w >> PDS_INST_SHIFT) & 3;
        type = (w >> PDS_TYPE_SHIFT) & 7;
        cc = (w >> PDS_CC_SHIFT) & 7;

        sgx_pr("[sgx]     +0x%02x: %08x  group=%u type=%u cc=%u", pc, w,
                group, type, cc);

        taken = pds_cc_true(&c, cc, &cc_known);
        if (c.exec && !cc_known) {
            sgx_pr("  ⚠ condition unknown; stopped\n");
            c.stop = "execution condition unknown";
            break;
        }
        if (c.exec && !taken) {
            sgx_pr("  (condition false; skipped)\n");
            pc += PDS_INSTRUCTION_SIZE;
            continue;
        }

        if (group == PDS_INST_FLOW && type == PDS_TYPE_HALT) {
            sgx_pr("  HALT\n");
            break;
        }
        if (group == PDS_INST_FLOW && type == PDS_TYPE_NOP) {
            sgx_pr("  NOP\n");
            pc += PDS_INSTRUCTION_SIZE;
            continue;
        }
        if (group == PDS_INST_FLOW &&
            (type == PDS_TYPE_TSTZ || type == PDS_TYPE_TSTN)) {
            uint32_t dst = w & PDS_TST_DEST_MASK;
            uint32_t src1 = (w >> PDS_TST_SRC1_SHIFT) & PDS_TST_SRC1_MASK;
            uint32_t src2 = (w >> PDS_TST_SRC2_SHIFT) & PDS_TST_SRC2_MASK;
            bool use_src2 = ((w >> PDS_TST_SRCSEL_SHIFT) & 1) ==
                            PDS_TST_SRCSEL_SRC2;
            bool src1_reg = ((w >> PDS_TST_SRC1SEL_SHIFT) & 1) ==
                            PDS_TST_SRC1SEL_REG;
            uint32_t val = 0;
            bool known = false;

            if (use_src2) {
                known = pds_ds_read(&c, 1, src2, &val);
                sgx_pr("  %s p%u, ds1[%u]",
                        type == PDS_TYPE_TSTZ ? "TSTZ" : "TSTN", dst, src2);
            } else if (src1_reg) {
                known = src1 < 2 && c.ir_known[src1];
                val = src1 < 2 ? c.ir[src1] : 0;
                sgx_pr("  %s p%u, ir%u",
                        type == PDS_TYPE_TSTZ ? "TSTZ" : "TSTN", dst, src1);
            } else {
                known = pds_ds_read(&c, 0, src1, &val);
                sgx_pr("  %s p%u, ds0[%u]",
                        type == PDS_TYPE_TSTZ ? "TSTZ" : "TSTN", dst, src1);
            }
            if (known) {
                sgx_pr(" = %08x", val);
            }
            if (dst < 3) {
                c.pred_known[dst] = known;
                c.pred[dst] = known &&
                    (type == PDS_TYPE_TSTZ ? val == 0 : (int32_t)val < 0);
                if (known) {
                    sgx_pr(" -> p%u=%u", dst, c.pred[dst]);
                }
            }
            if (c.exec && !known) {
                sgx_pr("  ⚠ source unknown; stopped\n");
                c.stop = "TST source unknown";
                break;
            }
            sgx_pr("\n");
            pc += PDS_INSTRUCTION_SIZE;
            continue;
        }
        if (group == PDS_INST_FLOW &&
            (type == PDS_TYPE_BRA || type == PDS_TYPE_CALL)) {
            uint32_t dest = (w & PDS_FLOW_DEST_MASK) << PDS_FLOW_DEST_ALIGNSHIFT;

            sgx_pr("  %s -> instruction %u (+0x%02x)\n",
                    type == PDS_TYPE_BRA ? "BRA" : "CALL",
                    dest >> PDS_FLOW_DEST_ALIGNSHIFT, dest);
            if (!c.exec) {
                pc += PDS_INSTRUCTION_SIZE;
                continue;
            }
            if (type == PDS_TYPE_CALL) {
                if (nlink >= ARRAY_SIZE(link)) {
                    c.stop = "CALL stack overflow";
                    break;
                }
                link[nlink++] = pc + PDS_INSTRUCTION_SIZE;
            }
            pc = dest;
            continue;
        }
        if (group == PDS_INST_FLOW && type == PDS_TYPE_RTN) {
            sgx_pr("  RTN\n");
            if (!c.exec) {
                pc += PDS_INSTRUCTION_SIZE;
                continue;
            }
            if (!nlink) {
                c.stop = "RTN without CALL";
                break;
            }
            pc = link[--nlink];
            continue;
        }
        if (group == PDS_INST_LOGIC && type <= PDS_TYPE_NAND) {
            uint32_t src1 = (w >> PDS_LOGIC_SRC1_SHIFT) & PDS_LOGIC_SRC1_MASK;
            uint32_t src2 = (w >> PDS_LOGIC_SRC2_SHIFT) & PDS_LOGIC_SRC2_MASK;
            bool s1_reg = ((w >> PDS_LOGIC_SRC1SEL_SHIFT) & 1) ==
                          PDS_LOGIC_SRC1SEL_REG;
            uint32_t dstsel = (w >> PDS_LOGIC_DESTSEL_SHIFT) &
                              PDS_LOGIC_DESTSEL_MASK;
            uint32_t dst = w & PDS_LOGIC_DEST_MASK;
            static const char *const names[] = {
                "OR", "AND", "XOR", "NOT", "NOR", "NAND"
            };
            uint32_t a = 0, b = 0, res = 0;
            bool ka, kb;

            if (s1_reg) {
                unsigned k = src1 == PDS_LOGIC_SRC1_IR1 ? 1 : 0;

                ka = (src1 == PDS_LOGIC_SRC1_IR0 ||
                      src1 == PDS_LOGIC_SRC1_IR1) && c.ir_known[k];
                a = ka ? c.ir[k] : 0;
                sgx_pr("  %s ds%u[%u], ir%u, ds1[%u]", names[type], dstsel,
                       dst, src1, src2);
            } else {
                ka = pds_ds_read(&c, 0, src1, &a);
                sgx_pr("  %s ds%u[%u], ds0[%u], ds1[%u]", names[type], dstsel,
                       dst, src1, src2);
            }
            kb = pds_ds_read(&c, 1, src2, &b);
            if (!ka || !kb) {
                sgx_pr("  ⚠ source unknown\n");
                if (c.exec) {
                    c.stop = "logical operation source unknown";
                    break;
                }
                pc += PDS_INSTRUCTION_SIZE;
                continue;
            }
            switch (type) {
            case PDS_TYPE_OR:   res = a | b;    break;
            case PDS_TYPE_AND:  res = a & b;    break;
            case PDS_TYPE_XOR:  res = a ^ b;    break;
            case PDS_TYPE_NOT:  res = ~a;       break;
            case PDS_TYPE_NOR:  res = ~(a | b); break;
            default:            res = ~(a & b); break;
            }
            sgx_pr("  = %08x & %08x -> %08x\n", a, b, res);
            if (c.exec && !pds_ds_write(&c, dstsel, dst, res)) {
                sgx_pr("[sgx]       ⚠ destination is not temporary; stopped\n");
                c.stop = "logical operation writes to a constant";
                break;
            }
            pc += PDS_INSTRUCTION_SIZE;
            continue;
        }
        if (group == PDS_INST_MOV && type == PDS_TYPE_MOV32) {
            uint32_t srcsel = (w >> PDS_MOV32_SRCSEL_SHIFT) &
                              PDS_MOV32_SRCSEL_MASK;
            uint32_t src = (w >> PDS_MOV32_SRC_SHIFT) & PDS_MOV32_SRC_MASK;
            uint32_t dstsel = (w >> PDS_MOV32_DESTSEL_SHIFT) &
                              PDS_MOV32_DESTSEL_MASK;
            uint32_t dst = (w >> PDS_MOV32_DEST_SHIFT) & PDS_MOV32_DEST_MASK;
            uint32_t val = 0;
            bool known = false;

            sgx_pr("  MOV32 ds%u[%u] <- ", dstsel, dst);
            if (srcsel == PDS_MOV32_SRCSEL_REG) {
                /*
                 * ir0/ir1 are encoded here as 0x00/0x02 (`sgxdefs.h:2829`),
                 * while PC and TIM are not modeled or simulated.
                 */
                if (src == PDS_MOV32_SRC_IR0 || src == PDS_MOV32_SRC_IR1) {
                    unsigned k = src == PDS_MOV32_SRC_IR0 ? 0 : 1;

                    known = c.ir_known[k];
                    val = c.ir[k];
                    sgx_pr("ir%u", k);
                } else {
                    sgx_pr("reg[%u]", src);
                }
            } else {
                known = pds_ds_read(&c, srcsel, src, &val);
                sgx_pr("ds%u[%u]", srcsel, src);
            }
            if (known) {
                sgx_pr(" = %08x", val);
                if (!pds_ds_write(&c, dstsel, dst, val) && c.exec) {
                    sgx_pr("  ⚠ destination is not temporary; stopped\n");
                    c.stop = "MOV32 writes to a constant";
                    break;
                }
            } else if (c.exec) {
                sgx_pr("  ⚠ source unknown; stopped\n");
                c.stop = "MOV32 source unknown";
                break;
            }
            sgx_pr("\n");
            pc += PDS_INSTRUCTION_SIZE;
            continue;
        }
        if (group == PDS_INST_MOV && type == PDS_TYPE_MOVS) {
            uint32_t src1 = (w >> PDS_MOVS_SRC1_SHIFT) & PDS_MOVS_SRC1_MASK;
            uint32_t src2 = (w >> PDS_MOVS_SRC2_SHIFT) & PDS_MOVS_SRC2_MASK;
            bool s1_ds0 = !((w >> PDS_MOVS_SRC1SEL_SHIFT) & 1);
            uint32_t dest = w & PDS_MOVS_DEST_MASK;
            /* Doubleword indices: sources address QUADWORDS. */
            uint32_t d1 = src1 * PDS_NUM_DWORDS_PER_QWORD;
            uint32_t d2 = src2 * PDS_NUM_DWORDS_PER_QWORD;
            uint32_t pair[4] = { 0, 0, 0, 0 };
            bool have[4] = { false, false, false, false };
            uint32_t emit[4] = { 0, 0, 0, 0 };
            unsigned k;

            /*
             * SRC1 is either a pair of adjacent DS0 values or a task INPUT
             * REGISTER (`SRC1SEL = REG`, `sgxdefs.h:2668,2674..2676`). The
             * model previously lacked this case and stopped at the event
             * handler's `movs douta, ir0, ...` instruction.
             */
            if (s1_ds0) {
                have[PDS_MOVS_SWIZ_SRC1L] = pds_ds_read(&c, 0, d1, &pair[0]);
                have[PDS_MOVS_SWIZ_SRC1H] = pds_ds_read(&c, 0, d1 + 1, &pair[1]);
            } else if (src1 == PDS_MOVS_SRC1_IR0 || src1 == PDS_MOVS_SRC1_IR1) {
                unsigned k0 = src1 == PDS_MOVS_SRC1_IR0 ? 0 : 1;

                have[PDS_MOVS_SWIZ_SRC1L] = c.ir_known[k0];
                pair[0] = c.ir[k0];
                if (k0 + 1 < 2) {
                    have[PDS_MOVS_SWIZ_SRC1H] = c.ir_known[k0 + 1];
                    pair[1] = c.ir[k0 + 1];
                }
            }
            /*
             * `tim` (timer counter) is not modeled or simulated: its value
             * stays unknown, and execution stops if it is used.
             */
            /* SRC2 always uses bank DS1: constants or temporaries. */
            have[PDS_MOVS_SWIZ_SRC2L] = pds_ds_read(&c, 1, d2, &pair[2]);
            have[PDS_MOVS_SWIZ_SRC2H] = pds_ds_read(&c, 1, d2 + 1, &pair[3]);

            n = 0;
            for (k = 0; k < 4; k++) {
                uint32_t sw = (w >> PDS_MOVS_SWIZ_SHIFT(k)) & 3;

                if (!have[sw]) {
                    break;
                }
                emit[k] = pair[sw];
                n++;
            }

            sgx_pr("  MOVS dest=%u src1=%s[%u] src2=ds1[%u]"
                    " (resolved %u)\n",
                    dest, s1_ds0 ? "ds0" : "ir", s1_ds0 ? d1 : src1, d2, n);
            if (dest == PDS_MOVS_DEST_DOUTA) {
                pds_douta(&c, emit, n);
            } else if (dest == PDS_MOVS_DEST_DOUTD) {
                pds_doutd(&c, emit, n);
            } else if (dest == PDS_MOVS_DEST_DOUTU) {
                pds_doutu(&c, emit, n);
            } else if (c.exec) {
                sgx_pr("[sgx]       ⚠ MOVS destination %u unsupported; stopped\n", dest);
                c.stop = "unsupported MOVS destination";
            }
            if (c.stop) {
                break;
            }
            pc += PDS_INSTRUCTION_SIZE;
            continue;
        }
        sgx_pr("  (unsupported)\n");
        if (c.exec) {
            c.stop = "unsupported instruction";
            break;
        }
        pc += PDS_INSTRUCTION_SIZE;
    }

    if (c.stop) {
        sgx_pr("[sgx]     ⛔ execution stopped: %s\n", c.stop);
    } else if (steps >= SGX_PDS_MAX_STEPS) {
        sgx_pr("[sgx]     ⛔ step limit (%u); stopped\n",
               SGX_PDS_MAX_STEPS);
    }
}


/* --- USE task: decoding and execution (public DDK, SGX540) ------------ */

/*
 * USE register banks. The model supports exactly those found on the
 * measured path: temporaries, secondary attributes, and immediates. Other
 * cases fail explicitly instead of silently returning zero.
 */
typedef enum {
    USE_BANK_TEMP,
    USE_BANK_SECATTR,
    USE_BANK_PRIMATTR,
    USE_BANK_IMMEDIATE,
    USE_BANK_INDEX,             /* i.l/i.h themselves as a `mov` destination */
    USE_BANK_INDEXED,           /* operand whose index must still be resolved */
    USE_BANK_UNSUPPORTED,
} ClarionUseBank;

typedef struct ClarionUseCtx {
    ClarionSgxState *s;
    uint32_t pd;
    unsigned depth;

    uint32_t r[USE_NUM_TEMPS];
    bool r_known[USE_NUM_TEMPS];
    uint32_t r_known_mask[USE_NUM_TEMPS];
    bool pred[USE_NUM_PREDICATES];
    bool pred_known[USE_NUM_PREDICATES];

    uint32_t poll_pc, poll_off, poll_value, poll_mask;
    uint32_t poll_version, poll_epoch;
    unsigned poll_steps, poll_repeats, poll_stores;
    bool poll_valid;

    /*
     * Task index registers. UNKNOWN on entry: the model does not know what
     * the previous owner left in the pipeline and must not invent zero.
     */
    uint32_t idx[USE_INDEX_BANK_SIZE];
    bool idx_known[USE_INDEX_BANK_SIZE];

    /* Notes for the current instruction, printed below its line. */
    char note[2][96];
    unsigned notes;

    uint32_t code_base;         /* this task's EUR_CR_USE_CODE_BASE_n window */
    uint32_t page_base;         /* code_base + COFF * USE_PAGE_SIZE */
    unsigned cbase;
    unsigned coff;
    uint32_t link;              /* link register for ba.savelink/lapc */
    bool link_known;

    const char *stop;
} ClarionUseCtx;

/*
 * S0 source bank: one bit. Not all instructions allow bank extension via
 * S0BEXT; in the integer group the same bit 18 means END (`usedisasm.c:10856`
 * passes `DecodeSrc0(..., FALSE, 0, ...)`), so extension must NOT be decoded there.
 */
static ClarionUseBank use_bank_s0_ex(uint32_t w1, bool allow_ext)
{
    uint32_t b = (w1 >> USE1_S0BANK_SHIFT) & USE1_S0BANK_MASK;

    if (allow_ext && (w1 & USE1_S0BEXT)) {
        return b == USE_S0EXTBANK_SECATTR ? USE_BANK_SECATTR
                                          : USE_BANK_UNSUPPORTED;
    }
    switch (b) {
    case USE_S0STDBANK_TEMP:     return USE_BANK_TEMP;
    case USE_S0STDBANK_PRIMATTR: return USE_BANK_PRIMATTR;
    default:                     return USE_BANK_UNSUPPORTED;
    }
}

static ClarionUseBank use_bank_s0(uint32_t w1)
{
    return use_bank_s0_ex(w1, true);
}

/* S1/S2 source bank: two bits, extended by S1BEXT/S2BEXT. */
static ClarionUseBank use_bank_s12(uint32_t bank, bool ext)
{
    if (ext) {
        switch (bank) {
        case USE_S12EXTBANK_IMMEDIATE: return USE_BANK_IMMEDIATE;
        case USE_S12EXTBANK_INDEXED:   return USE_BANK_INDEXED;
        default:                       return USE_BANK_UNSUPPORTED;
        }
    }
    switch (bank) {
    case USE_S12STDBANK_TEMP:     return USE_BANK_TEMP;
    case USE_S12STDBANK_SECATTR:  return USE_BANK_SECATTR;
    case USE_S12STDBANK_PRIMATTR: return USE_BANK_PRIMATTR;
    default:                      return USE_BANK_UNSUPPORTED;
    }
}

static const char *use_bank_name(ClarionUseBank b)
{
    switch (b) {
    case USE_BANK_TEMP:      return "r";
    case USE_BANK_SECATTR:   return "sa";
    case USE_BANK_PRIMATTR:  return "pa";
    case USE_BANK_IMMEDIATE: return "#";
    case USE_BANK_INDEX:     return "i";
    case USE_BANK_INDEXED:   return "idx";
    default:                 return "?";
    }
}

/* Index-register name selected by destination mask (`useasm.c:1920..1927`). */
static const char *use_index_name(uint32_t mask)
{
    switch (mask) {
    case USE_INDEX_MASK_L: return "i.l";
    case USE_INDEX_MASK_H: return "i.h";
    case USE_INDEX_MASK_L | USE_INDEX_MASK_H: return "i.lh";
    default: return "i?";
    }
}

/* Note below the instruction line: a trace of what was resolved. */
static void use_note(ClarionUseCtx *c, const char *fmt, ...)
{
    va_list ap;

    if (c->notes >= ARRAY_SIZE(c->note)) {
        return;
    }
    va_start(ap, fmt);
    vsnprintf(c->note[c->notes], sizeof(c->note[0]), fmt, ap);
    va_end(ap);
    c->notes++;
}

/*
 * Convert an indexed operand to a "bank + number" pair.
 *
 * The number field carries {bank, IDXSEL, offset} (`sgxdefs.h:7719..7733`).
 * The effective register number is `index + offset`: the field is named OFFSET, and the
 * reference assembler calls it "Offset into indexed register bank"
 * (`useasm.c:1048..1060`). This remains an inference.
 *
 * An unknown index fails with a log explanation instead of returning zero.
 */
static bool use_index_resolve(ClarionUseCtx *c, ClarionUseBank *bank,
                              uint32_t *num)
{
    uint32_t enc, off, sel;
    ClarionUseBank target;
    unsigned reg;

    if (*bank != USE_BANK_INDEXED) {
        return true;
    }
    enc = *num;
    off = enc & USE_INDEX_OFFSET_MASK;
    reg = (enc & USE_INDEX_IDXSEL) ? 1 : 0;
    sel = (enc & USE_INDEX_IDXSEL) ? USE_INDEX_MASK_H : USE_INDEX_MASK_L;

    switch ((enc >> USE_INDEX_BANK_SHIFT) & USE_INDEX_BANK_MASK) {
    case USE_INDEX_BANK_TEMP:     target = USE_BANK_TEMP; break;
    case USE_INDEX_BANK_PRIMATTR: target = USE_BANK_PRIMATTR; break;
    case USE_INDEX_BANK_SECATTR:  target = USE_BANK_SECATTR; break;
    default:
        /* OUTPUT: the model has no output bank; report it explicitly. */
        use_note(c, "indexed OUTPUT bank is not modeled");
        return false;
    }
    if (!c->idx_known[reg]) {
        use_note(c, "index %s is unknown; cannot resolve operand",
                 use_index_name(sel));
        return false;
    }
    *bank = target;
    *num = c->idx[reg] + off;
    use_note(c, "%s[%s+%u] -> %s%u   (%s = %u)", use_bank_name(target),
             use_index_name(sel), off, use_bank_name(target), *num,
             use_index_name(sel), c->idx[reg]);
    return true;
}

/* Read an operand. Unknown values return false; do not invent zero. */
static bool use_read(ClarionUseCtx *c, ClarionUseBank bank, uint32_t num,
                     uint32_t *out)
{
    if (!use_index_resolve(c, &bank, &num)) {
        return false;
    }
    switch (bank) {
    case USE_BANK_IMMEDIATE:
        *out = num;
        return true;
    case USE_BANK_TEMP:
        if (num >= USE_NUM_TEMPS || !c->r_known[num]) {
            return false;
        }
        *out = c->r[num];
        return true;
    case USE_BANK_SECATTR:
        if (num >= SGX_SA_DWORDS || !c->s->sa_known[num]) {
            return false;
        }
        *out = c->s->sa[num];
        return true;
    case USE_BANK_PRIMATTR:
        /*
         * Primary attributes are filled by the PDS program's `MOVS DOUTA`.
         * The event handler reads the flags written there by `ir0`/`ir1`.
         */
        if (num >= SGX_PA_DWORDS || !c->s->pa_known[num]) {
            return false;
        }
        *out = c->s->pa[num];
        return true;
    default:
        return false;
    }
}

static bool use_write(ClarionUseCtx *c, ClarionUseBank bank, uint32_t num,
                      uint32_t val)
{
    if (!use_index_resolve(c, &bank, &num)) {
        return false;
    }
    if (bank == USE_BANK_INDEX) {
        /* This is a register mask, not a register number. */
        if (num == 0 || num > (USE_INDEX_MASK_L | USE_INDEX_MASK_H)) {
            return false;
        }
        if (num & USE_INDEX_MASK_L) {
            c->idx[0] = val;
            c->idx_known[0] = true;
        }
        if (num & USE_INDEX_MASK_H) {
            c->idx[1] = val;
            c->idx_known[1] = true;
        }
        use_note(c, "%s = %u", use_index_name(num), val);
        return true;
    }
    if (bank == USE_BANK_TEMP && num < USE_NUM_TEMPS) {
        c->r[num] = val;
        c->r_known[num] = true;
        c->r_known_mask[num] = UINT32_MAX;
        return true;
    }
    if (bank == USE_BANK_PRIMATTR && num < SGX_PA_DWORDS) {
        c->s->pa[num] = val;
        c->s->pa_known[num] = true;
        c->s->pa_known_mask[num] = UINT32_MAX;
        if (num + 1 > c->s->pa_count) {
            c->s->pa_count = num + 1;
        }
        return true;
    }
    if (bank == USE_BANK_SECATTR && num < SGX_SA_DWORDS) {
        c->s->sa[num] = val;
        c->s->sa_known[num] = true;
        c->s->sa_known_mask[num] = UINT32_MAX;
        if (num + 1 > c->s->sa_count) {
            c->s->sa_count = num + 1;
        }
        return true;
    }
    return false;
}

typedef struct ClarionUseBits {
    uint32_t value;
    uint32_t known;
} ClarionUseBits;

static ClarionUseBits use_bits(uint32_t value, uint32_t known)
{
    return (ClarionUseBits) { value, known };
}

static bool use_read_bits(ClarionUseCtx *c, ClarionUseBank bank, uint32_t num,
                          ClarionUseBits *out)
{
    if (!use_index_resolve(c, &bank, &num)) {
        return false;
    }
    switch (bank) {
    case USE_BANK_IMMEDIATE:
        *out = use_bits(num, UINT32_MAX);
        return true;
    case USE_BANK_TEMP:
        if (num >= USE_NUM_TEMPS) { return false; }
        *out = use_bits(c->r[num], c->r_known[num] ? UINT32_MAX
                                                   : c->r_known_mask[num]);
        return true;
    case USE_BANK_SECATTR:
        if (num >= SGX_SA_DWORDS) { return false; }
        *out = use_bits(c->s->sa[num], c->s->sa_known[num] ? UINT32_MAX
                                      : c->s->sa_known_mask[num]);
        return true;
    case USE_BANK_PRIMATTR:
        if (num >= SGX_PA_DWORDS) { return false; }
        *out = use_bits(c->s->pa[num], c->s->pa_known[num] ? UINT32_MAX
                                      : c->s->pa_known_mask[num]);
        return true;
    default:
        return false;
    }
}

static bool use_write_bits(ClarionUseCtx *c, ClarionUseBank bank, uint32_t num,
                           ClarionUseBits val)
{
    if (val.known == UINT32_MAX) {
        return use_write(c, bank, num, val.value);
    }
    if (!use_index_resolve(c, &bank, &num)) {
        return false;
    }
    switch (bank) {
    case USE_BANK_TEMP:
        if (num >= USE_NUM_TEMPS) { return false; }
        c->r[num] = val.value;
        c->r_known[num] = false;
        c->r_known_mask[num] = val.known;
        return true;
    case USE_BANK_SECATTR:
        if (num >= SGX_SA_DWORDS) { return false; }
        c->s->sa[num] = val.value;
        c->s->sa_known[num] = false;
        c->s->sa_known_mask[num] = val.known;
        if (num + 1 > c->s->sa_count) { c->s->sa_count = num + 1; }
        return true;
    case USE_BANK_PRIMATTR:
        if (num >= SGX_PA_DWORDS) { return false; }
        c->s->pa[num] = val.value;
        c->s->pa_known[num] = false;
        c->s->pa_known_mask[num] = val.known;
        if (num + 1 > c->s->pa_count) { c->s->pa_count = num + 1; }
        return true;
    default:
        return false;
    }
}

static ClarionUseBits use_bits_and(ClarionUseBits a, ClarionUseBits b)
{
    uint32_t az = a.known & ~a.value, bz = b.known & ~b.value;
    uint32_t ao = a.known & a.value, bo = b.known & b.value;

    return use_bits(a.value & b.value, az | bz | (ao & bo));
}

static ClarionUseBits use_bits_or(ClarionUseBits a, ClarionUseBits b)
{
    uint32_t az = a.known & ~a.value, bz = b.known & ~b.value;
    uint32_t ao = a.known & a.value, bo = b.known & b.value;

    return use_bits(a.value | b.value, ao | bo | (az & bz));
}

static ClarionUseBits use_bits_xor(ClarionUseBits a, ClarionUseBits b)
{
    return use_bits(a.value ^ b.value, a.known & b.known);
}

static ClarionUseBits use_bits_shl(ClarionUseBits a, unsigned n)
{
    uint32_t zeros = n ? UINT32_MAX >> (32 - n) : 0;

    return use_bits(a.value << n, (a.known << n) | zeros);
}

static ClarionUseBits use_bits_shr(ClarionUseBits a, unsigned n)
{
    uint32_t zeros = n ? UINT32_MAX << (32 - n) : 0;

    return use_bits(a.value >> n, (a.known >> n) | zeros);
}

static ClarionUseBits use_bits_asr(ClarionUseBits a, unsigned n)
{
    uint32_t high = n && (a.known & 0x80000000U)
        ? UINT32_MAX << (32 - n) : 0;

    return use_bits((uint32_t)((int32_t)a.value >> n),
                    (a.known >> n) | high);
}

static ClarionUseBits use_bits_rol(ClarionUseBits a, unsigned n)
{
    if (!n) { return a; }
    return use_bits((a.value << n) | (a.value >> (32 - n)),
                    (a.known << n) | (a.known >> (32 - n)));
}

/* A known one proves nonzero even when other bits are unknown. */
static uint32_t use_test_zero_unknown(ClarionUseBits result)
{
    return (result.value & result.known) || result.known == UINT32_MAX
        ? 0 : ~result.known;
}

static bool use_test_require_known(ClarionUseCtx *c, uint32_t pc,
                                   const char *name, ClarionUseBits a,
                                   ClarionUseBits b, ClarionUseBits result,
                                   bool zero_check, bool sign_check)
{
    uint32_t missing = zero_check ? use_test_zero_unknown(result) : 0;

    if (missing) {
        sgx_pr("cannot evaluate TEST pc=%08x %s:"
               " a=%08x/%08x b=%08x/%08x result=%08x/%08x"
               " required-unknown=%08x\n", pc, name,
               a.value, a.known, b.value, b.known,
               result.value, result.known, missing);
        c->stop = "unknown bits in TEST";
        return false;
    }
    if (sign_check && !(result.known & 0x80000000U)) {
        sgx_pr("cannot evaluate TEST pc=%08x %s:"
               " sign bit unknown result=%08x/%08x\n",
               pc, name, result.value, result.known);
        c->stop = "unknown sign in TEST";
        return false;
    }
    return true;
}

/*
 * Whether a slot exists is separate from whether its value is known. This
 * matters when an unknown value must be propagated but a missing bank must
 * stop execution; these cases must not be conflated.
 */
static bool use_slot_ok(ClarionUseCtx *c, ClarionUseBank bank, uint32_t num)
{
    if (!use_index_resolve(c, &bank, &num)) {
        return false;
    }
    switch (bank) {
    case USE_BANK_IMMEDIATE: return true;
    case USE_BANK_INDEX:
        return num != 0 && num <= (USE_INDEX_MASK_L | USE_INDEX_MASK_H);
    case USE_BANK_TEMP:      return num < USE_NUM_TEMPS;
    case USE_BANK_SECATTR:   return num < SGX_SA_DWORDS;
    case USE_BANK_PRIMATTR:  return num < SGX_PA_DWORDS;
    default:                 return false;
    }
}

/* Mark a slot unknown rather than writing an invented zero. */
static bool use_forget(ClarionUseCtx *c, ClarionUseBank bank, uint32_t num)
{
    if (!use_index_resolve(c, &bank, &num)) {
        return false;
    }
    if (bank == USE_BANK_INDEX) {
        if (num == 0 || num > (USE_INDEX_MASK_L | USE_INDEX_MASK_H)) {
            return false;
        }
        if (num & USE_INDEX_MASK_L) {
            c->idx_known[0] = false;
        }
        if (num & USE_INDEX_MASK_H) {
            c->idx_known[1] = false;
        }
        return true;
    }
    if (bank == USE_BANK_TEMP && num < USE_NUM_TEMPS) {
        c->r_known[num] = false;
        c->r_known_mask[num] = 0;
        return true;
    }
    if (bank == USE_BANK_PRIMATTR && num < SGX_PA_DWORDS) {
        c->s->pa_known[num] = false;
        c->s->pa_known_mask[num] = 0;
        return true;
    }
    if (bank == USE_BANK_SECATTR && num < SGX_SA_DWORDS) {
        c->s->sa_known[num] = false;
        c->s->sa_known_mask[num] = 0;
        return true;
    }
    return false;
}

/* D1 destination bank. */
static ClarionUseBank use_bank_dst(uint32_t w1)
{
    uint32_t b = (w1 >> USE1_D1BANK_SHIFT) & USE1_D1BANK_MASK;

    if (w1 & USE1_DBEXT) {
        switch (b) {
        case USE_D1EXTBANK_SECATTR: return USE_BANK_SECATTR;
        case USE_D1EXTBANK_INDEX:   return USE_BANK_INDEX;
        default:                    return USE_BANK_UNSUPPORTED;
        }
    }
    switch (b) {
    case USE_D1STDBANK_TEMP:     return USE_BANK_TEMP;
    case USE_D1STDBANK_PRIMATTR: return USE_BANK_PRIMATTR;
    case USE_D1STDBANK_INDEXED:  return USE_BANK_INDEXED;
    default:                     return USE_BANK_UNSUPPORTED;
    }
}

/*
 * Instruction predicate (`sgxdefs.h:5166..5176`). `PNMOD4` depends on the
 * task instance number, which is not modeled, so do not guess.
 */
static bool use_pred_true(ClarionUseCtx *c, uint32_t epred, bool *known)
{
    *known = true;
    switch (epred) {
    case USE1_EPRED_ALWAYS: return true;
    case USE1_EPRED_P0:     *known = c->pred_known[0]; return c->pred[0];
    case USE1_EPRED_P1:     *known = c->pred_known[1]; return c->pred[1];
    case USE1_EPRED_P2:     *known = c->pred_known[2]; return c->pred[2];
    case USE1_EPRED_P3:     *known = c->pred_known[3]; return c->pred[3];
    case USE1_EPRED_NOTP0:  *known = c->pred_known[0]; return !c->pred[0];
    case USE1_EPRED_NOTP1:  *known = c->pred_known[1]; return !c->pred[1];
    default:
        *known = false;
        return false;
    }
}

/*
 * Compact integer-group predicate (`sgxdefs.h:5184..5190`,
 * `usedisasm.c:1932..1938`). This is a different field from EPRED and has
 * only four values; there are no unsupported values here.
 */
static bool use_spred_true(ClarionUseCtx *c, uint32_t spred, bool *known)
{
    *known = true;
    switch (spred) {
    case USE1_SPRED_P0:    *known = c->pred_known[0]; return c->pred[0];
    case USE1_SPRED_P1:    *known = c->pred_known[1]; return c->pred[1];
    case USE1_SPRED_NOTP0: *known = c->pred_known[0]; return !c->pred[0];
    default:               return true;   /* ALWAYS */
    }
}

/* ldad/stad offset multiplier: one address unit equals the access width. */
static uint32_t use_ldst_scale(uint32_t w1)
{
    switch ((w1 >> USE1_LDST_DTYPE_SHIFT) & USE1_LDST_DTYPE_MASK) {
    case USE1_LDST_DTYPE_32BIT: return 4;
    case USE1_LDST_DTYPE_16BIT: return 2;
    case USE1_LDST_DTYPE_8BIT:  return 1;
    default:                    return 0;
    }
}

/*
 * Execute a USE task from its entry point.
 *
 * Supports exactly the instruction classes measured by T4 on the reachable
 * path. Everything else stops execution with a named reason; the model
 * reports unsupported behavior instead of pretending to execute it.
 */
#define USE_PC_INDEX_MASK ((1U << SGX_FEATURE_USE_NUMBER_PC_BITS) - 1U)

static uint32_t use_page_pc(uint32_t page_base, uint32_t index)
{
    return page_base + ((index & USE_PC_INDEX_MASK) * USE_INST_SIZE);
}

static uint32_t use_next_pc(const ClarionUseCtx *c, uint32_t pc)
{
    uint32_t index = (pc - c->page_base) / USE_INST_SIZE;

    return use_page_pc(c->page_base, index + 1U);
}

/* Model-safety detector. It does not change SGX or guest-visible state. */
static bool use_poll_observe(ClarionUseCtx *c, uint32_t pc, uint32_t off,
                             ClarionUseBits value, unsigned steps)
{
    uint32_t i = off / 4;
    bool same = c->poll_valid && c->poll_pc == pc && c->poll_off == off &&
        c->poll_value == value.value && c->poll_mask == value.known &&
        c->poll_version == c->s->regs_version[i] &&
        c->poll_epoch == c->s->regs_epoch &&
        c->poll_stores == c->s->nstores &&
        steps - c->poll_steps <= SGX_USE_POLL_SPAN;

    if (same) {
        c->poll_repeats++;
    } else if (!c->poll_valid || c->poll_pc == pc) {
        c->poll_repeats = 1;
    } else {
        /* A different LDR may be part of the same short polling loop. */
        return false;
    }
    c->poll_valid = true;
    c->poll_pc = pc;
    c->poll_off = off;
    c->poll_value = value.value;
    c->poll_mask = value.known;
    c->poll_version = c->s->regs_version[i];
    c->poll_epoch = c->s->regs_epoch;
    c->poll_stores = c->s->nstores;
    c->poll_steps = steps;
    if (c->poll_repeats >= SGX_USE_POLL_REPEATS) {
        sgx_pr("[sgx] ⛔ repeated-LDR poll pc=%08x reg=+0x%04x"
               " repeats=%u value=%08x known=%08x\n", pc, off,
               c->poll_repeats, value.value, value.known);
        c->stop = "repeated LDR without register-state change";
        return true;
    }
    return false;
}

/*
 * Operand 0 of LDRSTR is the address of a global SGX register.
 *
 * ⚠ Canonical `DecodeLDRSTRInstruction` (`usedisasm.c:12068..12094`) handles
 * it the SAME for `ldr` and `str`: the immediate form is used only when
 * `S2BEXT` is set and S2's bank is IMMEDIATE. Otherwise S2 is a normal source,
 * and the register index is its **runtime value**, not the encoded field.
 * Previously both `str` and `ldr` violated this rule: the model used the
 * source number (for example, `+0x000C`) instead of r3's value in
 * `ldr r7, r3` (`sgx_timer.use.asm:107`), reading or writing the wrong register.
 */
static bool use_ldrstr_addr(ClarionUseCtx *c, const char *op, uint32_t w0,
                            uint32_t w1, uint32_t src2, uint32_t *num_out,
                            ClarionUseBank *bank_out)
{
    ClarionUseBank ab = use_bank_s12((w0 >> USE0_S2BANK_SHIFT) & USE0_BANK_MASK,
                                     (w1 & USE1_S2BEXT) != 0);
    uint32_t num;

    if (ab == USE_BANK_IMMEDIATE) {
        num = src2 | (((w0 >> USE0_LDRSTR_SRC2EXT_SHIFT) &
                       USE0_LDRSTR_SRC2EXT_MASK)
                      << USE_LDRSTR_SRC2EXT_INTSHIFT);
    } else if (ab == USE_BANK_TEMP || ab == USE_BANK_SECATTR ||
               ab == USE_BANK_PRIMATTR) {
        if (!use_read(c, ab, src2, &num)) {
            sgx_pr("%s: address %s%u  ⚠ value unknown\n", op,
                    use_bank_name(ab), src2);
            c->stop = "ldr/str address unknown";
            return false;
        }
    } else {
        sgx_pr("%s: unsupported address bank %s\n", op, use_bank_name(ab));
        c->stop = "unsupported ldr/str address bank";
        return false;
    }
    if (num > UINT32_MAX / 4) {
        sgx_pr("%s: register index 0x%08x too large for offset\n", op, num);
        c->stop = "ldr/str address too large";
        return false;
    }
    *num_out = num;
    *bank_out = ab;
    return true;
}

static void sgx_use_run(ClarionSgxState *s, uint32_t pd, uint32_t code_base,
                        uint32_t page_base, unsigned cbase, unsigned coff,
                        uint32_t entry, unsigned depth)
{
    ClarionUseCtx c;
    uint32_t pc = use_page_pc(page_base,
                              (entry - page_base) / USE_INST_SIZE);
    unsigned steps = 0;
    const char *ind;

    if (depth > SGX_MAX_DEPTH) {
        sgx_pr("[sgx]   ⛔ PDS/USE depth > %u; stopped\n",
                SGX_MAX_DEPTH);
        return;
    }

    memset(&c, 0, sizeof(c));
    c.s = s;
    c.pd = pd;
    c.depth = depth;
    c.code_base = code_base;
    c.page_base = page_base;
    c.cbase = cbase;
    c.coff = coff;
    ind = "";

    sgx_pr("[sgx]   %*sEXECUTING USE task @%08x (CBASE=%u COFF=%u;"
            " page_base=%08x; code window %08x):\n",
            2 * depth, ind, entry, cbase, coff, page_base, code_base);

    while (steps++ < SGX_USE_MAX_STEPS) {
        uint32_t pa, w0, w1, op, epred, dst, src0, src1, src2;
        bool pred_known, taken, is_end = false;
        unsigned note;

        if (c.poll_valid &&
            ((pc > c.poll_pc &&
              pc - c.poll_pc > SGX_USE_POLL_SPAN * USE_INST_SIZE) ||
             (pc < c.poll_pc &&
              c.poll_pc - pc > SGX_USE_POLL_SPAN * USE_INST_SIZE))) {
            c.poll_valid = false;
        }

        if (!sgx_translate(pd, pc, &pa)) {
            sgx_pr("[sgx]     %*s%08x: unmapped; stopped\n",
                    2 * depth, ind, pc);
            c.stop = "USE code is unmapped";
            break;
        }
        w0 = sgx_phys_ld32(pa);
        w1 = sgx_phys_ld32(pa + 4);
        op = (w1 >> USE1_OP_SHIFT) & 0x1F;
        epred = (w1 >> USE1_EPRED_SHIFT) & USE1_EPRED_MASK;
        dst = (w0 >> USE0_DST_SHIFT) & USE0_REG_MASK;
        src0 = (w0 >> USE0_SRC0_SHIFT) & USE0_REG_MASK;
        src1 = (w0 >> USE0_SRC1_SHIFT) & USE0_REG_MASK;
        src2 = (w0 >> USE0_SRC2_SHIFT) & USE0_REG_MASK;

        sgx_pr("[sgx]     %*s%08x: %08x %08x ", 2 * depth, ind,
                pc, w0, w1);
        c.notes = 0;

        /*
         * LIMM's predicate is in a different field (`sgxdefs.h:7443`), while
         * branches use the normal field. To avoid interpreting unrelated
         * bits as a predicate, gate only opcodes that actually have EPRED:
         * TEST, LD, ST, and flow control.
         */
        if (op == USE1_OP_TEST || op == USE1_OP_LD || op == USE1_OP_ST ||
            (op == USE1_OP_SPECIAL &&
             ((w1 >> USE1_SPECIAL_OPCAT_SHIFT) & USE1_SPECIAL_OPCAT_MASK) ==
             USE1_SPECIAL_OPCAT_FLOWCTRL)) {
            taken = use_pred_true(&c, epred, &pred_known);
            if (!pred_known) {
                sgx_pr(" ⚠ predicate %u is not modeled; stopped\n",
                        epred);
                c.stop = "USE predicate is not modeled";
                break;
            }
            if (!taken) {
                sgx_pr(" (predicate false; skipped)\n");
                pc = use_next_pc(&c, pc);
                continue;
            }
        }

        switch (op) {
        case USE1_OP_SPECIAL: {
            uint32_t cat = (w1 >> USE1_SPECIAL_OPCAT_SHIFT) &
                           USE1_SPECIAL_OPCAT_MASK;

            if (cat == USE1_SPECIAL_OPCAT_FLOWCTRL) {
                uint32_t op2 = (w1 >> USE1_FLOWCTRL_OP2_SHIFT) &
                               USE1_FLOWCTRL_OP2_MASK;

                if (op2 == USE1_FLOWCTRL_OP2_NOP) {
                    is_end = (w1 & USE1_END) != 0;
                    sgx_pr("nop%s\n", is_end ? ".end" : "");
                    break;
                }
                if (op2 == USE1_FLOWCTRL_OP2_BA) {
                    /* SGX540 BA target is a 12-bit instruction index within
                     * this task's COFF-selected code page (DDK
                     * SGX_FEATURE_USE_NUMBER_PC_BITS). */
                    uint32_t target = use_page_pc(c.page_base,
                                      w0 & USE_PC_INDEX_MASK);

                    if (w1 & USE1_BRANCH_SAVELINK) {
                        c.link = use_next_pc(&c, pc);
                        c.link_known = true;
                        sgx_pr("ba.savelink -> %08x (link=%08x)\n",
                                target, c.link);
                    } else {
                        sgx_pr("ba -> %08x\n", target);
                    }
                    pc = target;
                    continue;
                }
                if (op2 == USE1_FLOWCTRL_OP2_BR) {
                    uint32_t imm = w0 & USE0_BRANCH_OFFSET_MASK;
                    int32_t disp = (imm & (1U <<
                                           (SGX_FEATURE_USE_NUMBER_PC_BITS - 1)))
                                   ? (int32_t)imm -
                                     (1 << SGX_FEATURE_USE_NUMBER_PC_BITS)
                                   : (int32_t)imm;
                    uint32_t index = (pc - c.page_base) / USE_INST_SIZE;
                    uint32_t target;

                    /* Only the plain relative branch is modeled here.
                     * SAVELINK and ordering modifiers need separate behavior. */
                    if ((w0 & ~USE0_BRANCH_OFFSET_MASK) ||
                        (w1 & USE1_BRANCH_MODIFIER_MASK)) {
                        sgx_pr("br extension bits w0=%08x w1=%08x"
                               "; unsupported\n",
                               w0 & ~USE0_BRANCH_OFFSET_MASK,
                               w1 & USE1_BRANCH_MODIFIER_MASK);
                        c.stop = "unsupported BR modifiers";
                        break;
                    }
                    target = use_page_pc(c.page_base, index + disp);
                    sgx_pr("br #%+d pairs -> %08x\n", disp, target);
                    pc = target;
                    continue;
                }
                if (op2 == USE1_FLOWCTRL_OP2_LAPC) {
                    if (!c.link_known) {
                        sgx_pr("lapc  ⚠ link register is empty\n");
                        c.stop = "lapc without ba.savelink";
                        break;
                    }
                    sgx_pr("lapc -> %08x\n", c.link);
                    pc = c.link;
                    c.link_known = false;
                    continue;
                }
                if (op2 == USE1_FLOWCTRL_OP2_SETL) {
                    /*
                     * `mov pclink, src1`. The value is an INSTRUCTION NUMBER
                     * in the code window (see header), as stored by the host
                     * in `ui32ServiceAddress`. Convert it to an address using
                     * the same rule as the `ba` target so both paths agree.
                     */
                    ClarionUseBank sb = use_bank_s12(
                        (w0 >> USE0_S1BANK_SHIFT) & USE0_BANK_MASK,
                        (w1 & USE1_S1BEXT) != 0);
                    uint32_t val;

                    if (!use_slot_ok(&c, sb, src1)) {
                        sgx_pr("mov pclink, %s%u  ⚠ unsupported bank\n",
                                use_bank_name(sb), src1);
                        c.stop = "unsupported setl source bank";
                        break;
                    }
                    if (!use_read(&c, sb, src1, &val)) {
                        /*
                         * This is how the call macro RESTORES the link:
                         * `PVRSRV_SGXUTILS_CALL` (`usedefs.h:502..505`):
                         * `mov R_UTILS_PCLINK, pclink` -> `bal` ->
                         * `mov pclink, R_UTILS_PCLINK`. If the saved
                         * if the saved value is unknown, the link register is
                         * unknown too. That is accurate, not a zero value.
                         */
                        c.link_known = false;
                        sgx_pr("mov pclink, %s%u — value unknown;"
                                " pclink becomes unknown\n",
                                use_bank_name(sb), src1);
                        break;
                    }
                    c.link = use_page_pc(c.page_base, val);
                    c.link_known = true;
                    sgx_pr("mov pclink, %s%u = %08x  -> link=%08x\n",
                            use_bank_name(sb), src1, val, c.link);
                    if (c.r_known[7]) {
                        sgx_b2_dispatch(s, pd, c.code_base, pc, c.link,
                                        c.r[7], s->kicks);
                    } else if (s->nullrender &&
                               pc == c.code_base + 0x458) {
                        sgx_e2_ta_dispatch(pd, s, 0, s->kicks);
                        sgx_b2_skip(s->kicks, "context_unknown", pc);
                    }
                    break;
                }
                if (op2 == USE1_FLOWCTRL_OP2_SAVL) {
                    ClarionUseBank db = use_bank_dst(w1);

                    if (!c.link_known) {
                        /*
                         * The event handler starts via DOUTU, not a branch, so
                         * pclink contains whatever the previous pipeline owner
                         * left there. The call macro saves and restores it
                         * without inspecting it, so the unknown value can be
                         * propagated safely; stop only if code uses it.
                         */
                        if (!use_forget(&c, db, dst)) {
                            sgx_pr("mov %s%u, pclink  ⚠ unsupported destination\n",
                                    use_bank_name(db), dst);
                            c.stop = "unsupported savl destination";
                            break;
                        }
                        sgx_pr("mov %s%u, pclink — pclink unknown;"
                                " destination becomes unknown\n",
                                use_bank_name(db), dst);
                        break;
                    }
                    sgx_pr("mov %s%u, pclink = %08x (instruction %u)\n",
                            use_bank_name(db), dst, c.link,
                            (c.link - c.page_base) / USE_INST_SIZE);
                    if (!use_write(&c, db, dst,
                                   (c.link - c.page_base) / USE_INST_SIZE)) {
                        c.stop = "unsupported savl destination";
                    }
                    break;
                }
                sgx_pr("flowctrl op2=%u; unsupported\n", op2);
                c.stop = "unsupported USE flow control";
                break;
            }

            if (cat == USE1_SPECIAL_OPCAT_MOECTRL) {
                uint32_t op2 = (w1 >> USE1_MOECTRL_OP2_SHIFT) &
                               USE1_MOECTRL_OP2_MASK;

                /*
                 * SMLSI sets MOE increment modes for REPEATED instructions.
                 * No instruction on our path has a repeat count, so MOE state
                 * has no effect and is only recorded in the trace. If repeats
                 * appear, this must be modeled properly.
                 */
                if (op2 == USE1_MOECTRL_OP2_SMLSI) {
                    sgx_pr("smlsi (MOE state; no repeats on path, no effect)\n");
                    break;
                }
                sgx_pr("moectrl op2=%u; unsupported\n", op2);
                c.stop = "unsupported MOE control";
                break;
            }

            if (cat == USE1_SPECIAL_OPCAT_OTHER) {
                uint32_t op2 = (w1 >> USE1_OTHER_OP2_SHIFT) &
                               USE1_OTHER_OP2_MASK;

                if (op2 == USE1_OTHER_OP2_LIMM) {
                    uint32_t imm = (w0 & USE0_LIMM_IMML21_MASK) |
                        (((w1 >> USE1_LIMM_IMM2521_SHIFT) &
                          USE1_LIMM_IMM2521_MASK) << 21) |
                        (((w1 >> USE1_LIMM_IMM3126_SHIFT) &
                          USE1_LIMM_IMM3126_MASK) << 26);
                    ClarionUseBank db = use_bank_dst(w1);
                    uint32_t lpred = (w1 >> USE1_LIMM_EPRED_SHIFT) &
                                     USE1_LIMM_EPRED_MASK;
                    bool lpred_known, lpred_taken;

                    /*
                     * LIMM's predicate is in its own field and must be
                     * checked: the microkernel loop counter (`0x0e402c40`)
                     * uses a predicated `mov`; without the check it is reset
                     * on every iteration, causing an infinite loop.
                     */
                    lpred_taken = use_pred_true(&c, lpred, &lpred_known);
                    if (!lpred_known) {
                        sgx_pr("mov #0x%08x  ⚠ predicate %u is not modeled\n",
                                imm, lpred);
                        c.stop = "LIMM predicate is not modeled";
                        break;
                    }
                    if (!lpred_taken) {
                        sgx_pr("mov %s%u, #0x%08x"
                                " (predicate false; skipped)\n",
                                use_bank_name(db), dst, imm);
                        pc = use_next_pc(&c, pc);
                        continue;
                    }
                    sgx_pr("mov %s%u, #0x%08x", use_bank_name(db),
                            dst, imm);
                    if (!use_write(&c, db, dst, imm)) {
                        sgx_pr("  ⚠ unsupported destination\n");
                        c.stop = "unsupported LIMM destination";
                        break;
                    }
                    is_end = (w1 & USE1_END) != 0;
                    sgx_pr("%s\n", is_end ? "  .end" : "");
                    break;
                }
                if (op2 == USE1_OTHER_OP2_IDF || op2 == USE1_OTHER_OP2_WDF) {
                    /*
                     * Memory accesses are synchronous, so the data queue is
                     * always empty and the barrier has no work. This is not
                     * skipped; it is trivially satisfied.
                     */
                    sgx_pr("%s drc%u (synchronous accesses)\n",
                            op2 == USE1_OTHER_OP2_IDF ? "idf" : "wdf", w1 & 3);
                    break;
                }
                if (op2 == USE1_OTHER_OP2_LDRSTR) {
                    bool is_store = (w1 & USE1_LDRSTR_DSEL_STORE) != 0;
                    const char *op_name = is_store ? "str" : "ldr";
                    ClarionUseBank ab;
                    uint32_t num, off, val;
                    char addr_src[32];

                    if (!use_ldrstr_addr(&c, op_name, w0, w1, src2,
                                         &num, &ab)) {
                        break;
                    }
                    off = num * 4;
                    if (ab == USE_BANK_IMMEDIATE) {
                        addr_src[0] = '\0';
                    } else {
                        snprintf(addr_src, sizeof(addr_src), "  (address from %s%u)",
                                 use_bank_name(ab), src2);
                    }

                    if (!is_store) {
                        /*
                         * `ldr dst, #number` reads an SGX register from the
                         * same space written by `str`. Return a value only if
                         * the model knows it; otherwise report the register
                         * and stop. Inventing zero is worse: the microkernel
                         * polls status bits and could loop forever.
                         */
                        /* LDRSTR overlays the generic D1 bank fields: bit 7
                         * selects TEMP versus PRIMATTR; bits 1:0 select DRC.
                         * See SGX540 usedisasm DecodeLDRSTRInstruction and
                         * T20b. Do not decode this as a generic D1 destination.
                         */
                        ClarionUseBank db = (w1 & 0x80)
                            ? USE_BANK_PRIMATTR : USE_BANK_TEMP;
                        if (off + 4 > CLARION_SGX_SIZE) {
                            sgx_pr("ldr #%u%s — outside register window\n",
                                    num, addr_src);
                            c.stop = "ldr outside register window";
                            break;
                        }
                        if (!s->regs_known_mask[off / 4]) {
                            sgx_pr("ldr %s%u, #%u%s  ⚠ register +0x%04x is"
                                    " not modeled (no writes observed)\n",
                                    use_bank_name(db), dst, num, addr_src,
                                    off);
                            c.stop = "SGX register value unknown";
                            break;
                        }
                        sgx_pr("ldr %s%u, #%u%s = %08x  (reg +0x%04x)%s\n",
                                use_bank_name(db), dst, num, addr_src,
                                s->regs[off / 4], off,
                                off == SGX_CR_CORE_ID
                                    ? "  ⚠ EUR_CR_CORE_ID: value has no"
                                      " established basis; contract does not read it"
                                    : off == SGX_CR_CORE_REVISION
                                    ? "  EUR_CR_CORE_REVISION: constant reported"
                                      " by the firmware itself"
                                    : "");
                        ClarionUseBits loaded = use_bits(
                            s->regs[off / 4], s->regs_known_mask[off / 4]);
                        if (use_poll_observe(&c, pc, off, loaded, steps)) {
                            break;
                        }
                        if (!use_write_bits(&c, db, dst, loaded)) {
                            c.stop = "unsupported ldr destination";
                        }
                        break;
                    }
                    /*
                     * ⚠ Store data comes from SRC1, not the destination
                     * field: `usedisasm.c:12093..12099` decodes it with
                     * `DecodeSrc12(..., 1, ..., S1BEXT)`. Using DST made the
                     * model write r0 where the IMG reference disassembler
                     * specifies r1 or even #1.
                     */
                    ClarionUseBank sb = use_bank_s12(
                        (w0 >> USE0_S1BANK_SHIFT) & USE0_BANK_MASK,
                        (w1 & USE1_S1BEXT) != 0);

                    if (!use_read(&c, sb, src1, &val)) {
                        sgx_pr("str #%u%s, %s%u  ⚠ source unknown\n",
                                num, addr_src, use_bank_name(sb), src1);
                        c.stop = "str source unknown";
                        break;
                    }
                    /*
                     * Register number × 4 is its SGX byte offset. Write to
                     * the register mirror, the same bank the guest writes via
                     * MMIO and where the program later reads task-control.
                     */
                    sgx_pr("str #%u%s, %s%u = %08x  -> reg +0x%04x\n",
                            num, addr_src, use_bank_name(sb), src1, val, off);
                    if (off + 4 <= CLARION_SGX_SIZE) {
                        if (off == SGX_CR_EVENT_STATUS) {
                            if (!sgx_write_event_status(s, val)) {
                                sgx_pr("str EUR_CR_EVENT_STATUS +0x%04x = %08x"
                                       " unsupported mask %08x USE PC %08x\n",
                                       off, val,
                                       val & ~SGX_CR_EVENT_STATUS_SW_EVENT,
                                       pc);
                                c.stop = "unsupported EVENT_STATUS write";
                            }
                        } else {
                            sgx_set_reg(s, off, val);
                            sgx_reg_side_effects(s, off, val);
                        }
                    } else {
                        c.stop = "str outside register window";
                    }
                    break;
                }
                if (op2 == USE1_OTHER_OP2_EMIT) {
                    uint32_t target = (w1 >> USE1_EMIT_TARGET_SHIFT) &
                                      USE1_EMIT_TARGET_MASK;
                    uint32_t sb0 = 0, addr = 0, ir0 = 0, rows, prog;
                    uint32_t pds_base = sgx_reg(s, SGX_CR_PDS_EXEC_BASE) &
                                        SGX_PDS_EXEC_BASE_ADDR_MASK;

                    if (target != USE1_EMIT_TARGET_PDS) {
                        sgx_pr("emit target=%u; unsupported\n",
                                target);
                        c.stop = "emit target is not PDS";
                        break;
                    }
                    if (!use_read(&c, USE_BANK_TEMP, src0, &sb0) ||
                        !use_read(&c, USE_BANK_TEMP, src1, &addr) ||
                        !use_read(&c, USE_BANK_TEMP, src2, &ir0)) {
                        sgx_pr("emitpds  ⚠ operand unknown\n");
                        c.stop = "emitpds operand unknown";
                        break;
                    }
                    /*
                     * Address uses the same encoding as EUR_CR_EVENT_OTHER_PDS_EXEC:
                     * field << 4 under the ADDR mask. Data segment size comes
                     * from PDSDATASIZE in sideband word 0, in 16-byte units;
                     * a parser "row" is also 16 bytes, so the counts match.
                     */
                    prog = pds_base +
                           ((addr << 4) & SGX_EVENT_OTHER_PDS_EXEC_ADDR_MASK);
                    rows = (sb0 >> PDSSB0_PDSDATASIZE_SHIFT) &
                           PDSSB0_PDSDATASIZE_MASK;
                    sgx_pr("emitpds r%u=%08x r%u=%08x r%u=%08x\n",
                            src0, sb0, src1, addr, src2, ir0);
                    sgx_pr("[sgx]     %*s  -> PDS program %08x,"
                            " data %u × 16 bytes, ir0=%08x\n",
                            2 * depth, ind, prog, rows, ir0);
                    sgx_pds_run(s, pd, prog, rows, "launched by emitpds",
                                true, ir0, 0, depth + 1);
                    break;
                }
                sgx_pr("other op2=%u; unsupported\n", op2);
                c.stop = "unsupported OTHER instruction";
                break;
            }
            sgx_pr("special cat=%u; unsupported\n", cat);
            c.stop = "unsupported SPECIAL instruction";
            break;
        }

        case USE1_OP_TEST: {
            uint32_t alusel = (w0 >> USE0_TEST_ALUSEL_SHIFT) &
                              USE0_TEST_ALUSEL_MASK;
            uint32_t aluop = (w0 >> USE0_TEST_ALUOP_SHIFT) &
                             USE0_TEST_ALUOP_MASK;
            uint32_t ztst = (w1 >> USE1_TEST_ZTST_SHIFT) & USE1_TEST_ZTST_MASK;
            uint32_t stst = (w1 >> USE1_TEST_STST_SHIFT) & USE1_TEST_STST_MASK;
            uint32_t pdst = (w1 >> USE1_TEST_PDST_SHIFT) & USE1_TEST_PDST_MASK;
            ClarionUseBank b1 = use_bank_s12((w0 >> USE0_S1BANK_SHIFT) &
                                             USE0_BANK_MASK,
                                             (w1 & USE1_S1BEXT) != 0);
            ClarionUseBank b2 = use_bank_s12((w0 >> USE0_S2BANK_SHIFT) &
                                             USE0_BANK_MASK,
                                             (w1 & USE1_S2BEXT) != 0);
            ClarionUseBits a, b, res;
            const char *name;

            if (alusel != USE0_TEST_ALUSEL_BITWISE &&
                !(alusel == USE0_TEST_ALUSEL_I16 &&
                  aluop == USE0_TEST_ALUOP_I16_ISUB)) {
                sgx_pr("test alusel=%u aluop=%u; unsupported\n",
                        alusel, aluop);
                c.stop = "unsupported ALUSEL in TEST";
                break;
            }
            if (!use_read_bits(&c, b1, src1, &a) ||
                !use_read_bits(&c, b2, src2, &b)) {
                sgx_pr("test  ⚠ source unknown\n");
                c.stop = "TEST source unknown";
                break;
            }
            if (alusel == USE0_TEST_ALUSEL_I16) {
                uint32_t channel = (w1 >> USE1_TEST_CHANCC_SHIFT) &
                                   USE1_TEST_CHANCC_MASK;

                /* SGX540 ISUB16 computes signed halfword lanes. TEST channel 0
                 * observes the low halfword; this instruction does not write
                 * back the arithmetic result (WBEN is clear in the target). */
                if (channel != USE1_TEST_CHANCC_SELECT0) {
                    sgx_pr("isub16.test channel=%u; unsupported\n", channel);
                    c.stop = "unsupported I16 TEST channel";
                    break;
                }
                if (a.known != UINT32_MAX || b.known != UINT32_MAX) {
                    sgx_pr("isub16.test pc=%08x unknown source masks"
                           " a=%08x/%08x b=%08x/%08x\n", pc,
                           a.value, a.known, b.value, b.known);
                    c.stop = "unknown bits in arithmetic TEST";
                    break;
                }
                res = use_bits((uint32_t)(int32_t)(int16_t)
                               (uint16_t)(a.value - b.value), UINT32_MAX);
                name = "isub16";
            } else switch (aluop) {
            case USE0_TEST_ALUOP_BW_AND: res = use_bits_and(a, b); name = "and"; break;
            case USE0_TEST_ALUOP_BW_OR:  res = use_bits_or(a, b);  name = "or"; break;
            case USE0_TEST_ALUOP_BW_XOR: res = use_bits_xor(a, b); name = "xor"; break;
            case USE0_TEST_ALUOP_BW_SHL:
                if (b.known != UINT32_MAX) { c.stop = "TEST shift unknown"; break; }
                res = use_bits_shl(a, b.value & 31); name = "shl"; break;
            case USE0_TEST_ALUOP_BW_SHR:
                if (b.known != UINT32_MAX) { c.stop = "TEST shift unknown"; break; }
                res = use_bits_shr(a, b.value & 31); name = "shr"; break;
            case USE0_TEST_ALUOP_BW_ASR:
                if (b.known != UINT32_MAX) { c.stop = "TEST shift unknown"; break; }
                res = use_bits_asr(a, b.value & 31); name = "asr"; break;
            default:
                sgx_pr("test aluop=%u; unsupported\n", aluop);
                c.stop = "unsupported TEST operation";
                res = use_bits(0, 0); name = "?";
                break;
            }
            if (c.stop) {
                break;
            }
            /*
             * Two independent checks (zero and sign) combine according to
             * CRCOMB (see header). `NONE` means "always true", not "no check".
             */
            {
                bool zt, st, comb_and = (w1 & USE1_TEST_CRCOMB_AND) != 0;
                const char *zn, *sn;

                switch (ztst) {
                case USE1_TEST_ZTST_NONE:
                    zt = true;  zn = "-";   break;
                case USE1_TEST_ZTST_ZERO:
                    zt = res.value == 0; zn = "==0"; break;
                case USE1_TEST_ZTST_NOTZERO:
                    zt = res.value != 0; zn = "!=0"; break;
                default:
                    sgx_pr("  ⚠ reserved zero condition\n");
                    c.stop = "reserved zero condition in TEST";
                    zt = false; zn = "?";
                    break;
                }
                if (c.stop) {
                    break;
                }
                switch (stst) {
                case USE1_TEST_STST_NONE:
                    st = true; sn = "-"; break;
                case USE1_TEST_STST_NEGATIVE:
                    st = (int32_t)res.value < 0;  sn = "sign"; break;
                case USE1_TEST_STST_POSITIVE:
                    st = (int32_t)res.value >= 0; sn = "!sign"; break;
                default:
                    sgx_pr("  ⚠ reserved sign condition\n");
                    c.stop = "reserved sign condition in TEST";
                    st = false; sn = "?";
                    break;
                }
                if (c.stop) {
                    break;
                }
                if (!use_test_require_known(&c, pc, name, a, b, res,
                                            ztst != USE1_TEST_ZTST_NONE,
                                            stst != USE1_TEST_STST_NONE)) {
                    break;
                }
                c.pred[pdst] = comb_and ? (zt && st) : (zt || st);
                c.pred_known[pdst] = true;
                sgx_pr("%s.test %s%u, %s%u = %08x -> p%u=%u (%s %s %s)\n",
                        name, use_bank_name(b1), src1, use_bank_name(b2),
                        src2, res.value, pdst, c.pred[pdst], sn,
                        comb_and ? "and" : "or", zn);
            }
            if (w0 & USE0_TEST_WBEN) {
                ClarionUseBank db = use_bank_dst(w1);

                if (!use_write_bits(&c, db, dst, res)) {
                    c.stop = "unsupported TEST destination";
                }
            }
            break;
        }

        case USE1_OP_MOVC: {
            uint32_t tst = (w1 >> USE1_MOVC_TSTDTYPE_SHIFT) &
                           USE1_MOVC_TSTDTYPE_MASK;
            ClarionUseBank b1 = use_bank_s12((w0 >> USE0_S1BANK_SHIFT) &
                                             USE0_BANK_MASK,
                                             (w1 & USE1_S1BEXT) != 0);
            ClarionUseBank db = use_bank_dst(w1);
            ClarionUseBits val;

            if (tst != USE1_MOVC_TSTDTYPE_UNCOND) {
                sgx_pr("conditional movc (tstdtype=%u); unsupported\n",
                        tst);
                c.stop = "conditional movc";
                break;
            }
            if (!use_read_bits(&c, b1, src1, &val)) {
                sgx_pr("mov %s%u, %s%u  ⚠ source unknown\n",
                        use_bank_name(db), dst, use_bank_name(b1), src1);
                c.stop = "mov source unknown";
                break;
            }
            is_end = (w1 & USE1_END) != 0;
            sgx_pr("mov %s%u, %s%u = %08x%s\n", use_bank_name(db),
                    dst, use_bank_name(b1), src1, val.value,
                    is_end ? "  .end" : "");
            if (!use_write_bits(&c, db, dst, val)) {
                c.stop = "unsupported mov destination";
            }
            break;
        }

        case USE1_OP_ANDOR:
        case USE1_OP_XOR:
        case USE1_OP_SHLROL:
        case USE1_OP_SHRASR: {
            uint32_t op2 = (w1 >> USE1_BITWISE_OP2_SHIFT) &
                           USE1_BITWISE_OP2_MASK;
            uint32_t rot = (w1 >> USE1_BITWISE_SRC2ROT_SHIFT) &
                           USE1_BITWISE_SRC2ROT_MASK;
            ClarionUseBank b1 = use_bank_s12((w0 >> USE0_S1BANK_SHIFT) &
                                             USE0_BANK_MASK,
                                             (w1 & USE1_S1BEXT) != 0);
            ClarionUseBank b2 = use_bank_s12((w0 >> USE0_S2BANK_SHIFT) &
                                             USE0_BANK_MASK,
                                             (w1 & USE1_S2BEXT) != 0);
            ClarionUseBank db = use_bank_dst(w1);
            uint32_t b;
            ClarionUseBits a, bv, res;
            const char *name = "?";

            if (w1 & USE1_BITWISE_PARTIAL) {
                sgx_pr("bitwise operation with PARTIAL; unsupported\n");
                c.stop = "PARTIAL bitwise operation";
                break;
            }
            if (!use_read_bits(&c, b1, src1, &a)) {
                sgx_pr("bitwise operation  ⚠ source unknown\n");
                c.stop = "bitwise operation source unknown";
                break;
            }
            if (b2 == USE_BANK_IMMEDIATE) {
                /*
                 * The immediate is a 16-bit value assembled from three fields
                 * (see header), then rotated LEFT. The reference decoder does
                 * this in `usedisasm.c:9018..9024` with
                 * `RotateLeft(uNumber, uRot)`. Register sources are not rotated;
                 * rotation applies only to the immediate operand.
                 */
                b = src2 |
                    (((w0 >> USE0_BITWISE_SRC2IEXTLPSEL_SHIFT) &
                      USE0_BITWISE_SRC2IEXTLPSEL_MASK) << 7) |
                    (((w1 >> USE1_BITWISE_SRC2IEXTH_SHIFT) &
                      USE1_BITWISE_SRC2IEXTH_MASK) << 14);
                if (rot) {
                    b = (b << rot) | (b >> (32 - rot));
                }
                bv = use_bits(b, UINT32_MAX);
            } else {
                if (!use_read_bits(&c, b2, src2, &bv)) {
                    sgx_pr("bitwise operation  ⚠ unsupported source\n");
                    c.stop = "unsupported bitwise operation source";
                    break;
                }
                b = bv.value;
            }
            if (w1 & USE1_BITWISE_SRC2INV) {
                bv.value = ~bv.value;
                b = bv.value;
            }
            switch (op) {
            case USE1_OP_ANDOR:
                res = op2 ? use_bits_or(a, bv) : use_bits_and(a, bv);
                name = op2 ? "or" : "and";
                break;
            case USE1_OP_XOR:
                res = use_bits_xor(a, bv);
                name = "xor";
                break;
            case USE1_OP_SHLROL:
                if (bv.known != UINT32_MAX) {
                    c.stop = "bitwise operation shift unknown";
                    break;
                }
                if (op2) {
                    res = use_bits_rol(a, b & 31);
                    name = "rol";
                } else {
                    res = use_bits_shl(a, b & 31);
                    name = "shl";
                }
                break;
            default:
                if (bv.known != UINT32_MAX) {
                    c.stop = "bitwise operation shift unknown";
                    break;
                }
                res = op2 ? use_bits_asr(a, b & 31)
                          : use_bits_shr(a, b & 31);
                name = op2 ? "asr" : "shr";
                break;
            }
            if (c.stop) { break; }
            if (b2 == USE_BANK_IMMEDIATE) {
                sgx_pr("%s %s%u, %s%u, #0x%x = %08x\n", name,
                       use_bank_name(db), dst, use_bank_name(b1), src1,
                       b, res.value);
            } else {
                sgx_pr("%s %s%u, %s%u, %s%u = %08x\n", name,
                       use_bank_name(db), dst, use_bank_name(b1), src1,
                       use_bank_name(b2), src2, res.value);
            }
            if (!use_write_bits(&c, db, dst, res)) {
                c.stop = "unsupported bitwise operation destination";
            }
            break;
        }

        case USE1_OP_LD:
        case USE1_OP_ST: {
            bool store = op == USE1_OP_ST;
            ClarionUseBank b0 = use_bank_s0(w1);
            ClarionUseBank b1 = use_bank_s12((w0 >> USE0_S1BANK_SHIFT) &
                                             USE0_BANK_MASK,
                                             (w1 & USE1_S1BEXT) != 0);
            ClarionUseBank b2 = use_bank_s12((w0 >> USE0_S2BANK_SHIFT) &
                                             USE0_BANK_MASK,
                                             (w1 & USE1_S2BEXT) != 0);
            uint32_t imode = (w1 >> USE1_LDST_IMODE_SHIFT) &
                             USE1_LDST_IMODE_MASK;
            /* One counter; meaning depends on MOEEXPAND (see header). */
            uint32_t count = ((w1 >> USE1_RMSKCNT_SHIFT) &
                              USE1_RMSKCNT_MASK) + 1;
            bool fetch = (w1 & USE1_LDST_MOEEXPAND) == 0;
            uint32_t base, off, scale, addr, step, pa2, val = 0;
            const char *mnem = store ? "stad" : "ldad";
            char suffix[16];
            unsigned i;

            if (((w1 >> USE1_LDST_AMODE_SHIFT) & USE1_LDST_AMODE_MASK) !=
                USE1_LDST_AMODE_ABSOLUTE) {
                sgx_pr("ld/st with unsupported addressing mode\n");
                c.stop = "ld/st addressing mode";
                break;
            }
            if (imode == USE1_LDST_IMODE_RESERVED) {
                sgx_pr("ld/st with reserved increment mode\n");
                c.stop = "reserved ld/st increment mode";
                break;
            }
            if (w1 & USE1_LDST_RANGEENABLE) {
                /*
                 * RANGEENABLE adds a third source, the range limit; hardware
                 * may disable the instruction when the address is outside it.
                 * This has not been measured, so do not guess.
                 */
                sgx_pr("ld/st with rangeenable; unsupported\n");
                c.stop = "rangeenable in ld/st";
                break;
            }
            if (!fetch && count > 1) {
                /*
                 * MOE expands repeats, but its state is not modeled (see
                 * SMLSI above). A single access does not use MOE; larger
                 * counts are unsupported.
                 */
                sgx_pr("ld/st with MOE repeat ×%u; unsupported\n",
                        count);
                c.stop = "MOE repeat in ld/st";
                break;
            }
            scale = use_ldst_scale(w1);
            if (scale != 4 && scale != 2) {
                sgx_pr("ld/st width %u bytes; unsupported\n", scale);
                c.stop = "ld/st access width";
                break;
            }
            if (scale == 2) {
                /* Same OP/AMODE, different DTYPE; IMG uses different mnemonics. */
                mnem = store ? "staw" : "ldaw";
                if (count > 1) {
                    /*
                     * The number of registers filled by a halfword fetch and
                     * their packing have not been measured. One access is known.
                     */
                    sgx_pr("%s with count ×%u; not measured\n", mnem, count);
                    c.stop = "halfword ld/st count";
                    break;
                }
            }
            if (!use_read(&c, b0, src0, &base) ||
                !use_read(&c, b1, src1, &off)) {
                sgx_pr("%s  ⚠ address unknown\n", mnem);
                c.stop = "ld/st address unknown";
                break;
            }
            /*
             * ⚠ `INCSGN` sets the increment sign, not the address-offset sign.
             * Do not apply it to `IMODE_NONE`, which has no increment; there
             * the offset is added to the address as-is.
             */
            step = off * scale;
            if (w1 & USE1_LDST_INCSGN) {
                step = (uint32_t)-(int32_t)step;
            }
            /*
             * ⚠ Increment placement is established by measurement, not
             * convention. The event handler performs these accesses in order:
             *
             *   ldad.fcfill CCB(ui32ServiceAddress), [r0, #1++]
             *   ldad.f7     CCB(ui32CacheControl),   [r0, #0++]
             *
             * `CCB(x)` is `r[4 + DOFFSET(SGXMKIF_COMMAND.x)]` (`usedefs.h:59`),
             * namely r4 and r5..r11. To load command doubleword 0 into r4 and
             * doublewords 1..7 into r5..r11, the first access must use the
             * base itself, then add 4. POST uses base then increments; PRE
             * increments first, then accesses.
             */
            addr = (imode == USE1_LDST_IMODE_PRE)  ? base + step :
                   (imode == USE1_LDST_IMODE_POST) ? base :
                                                     base + off * scale;
            if (scale == 2 && (addr & 1)) {
                /*
                 * Hardware behavior at an odd address is UNKNOWN. Do not
                 * touch registers or memory; stop rather than invent alignment semantics.
                 */
                sgx_pr("%s ⚠ odd address VA %08x; semantics unproven\n",
                        mnem, addr);
                c.stop = "odd halfword ld/st address";
                break;
            }
            if (fetch && count > 1 && imode != USE1_LDST_IMODE_NONE &&
                step != 0) {
                /*
                 * The number of base increments for multi-doubleword fetches
                 * is unmeasured (the offset is zero on our path). Do not guess.
                 */
                sgx_pr("ld/st: fetch ×%u with increment #%u; not measured\n",
                        count, off);
                c.stop = "ld/st fetch with nonzero increment";
                break;
            }

            suffix[0] = '\0';
            if (fetch) {
                snprintf(suffix, sizeof(suffix), ".f%u", count);
            }
            if (w1 & USE1_LDST_FCLFILL) {
                /* No cache exists, so this requirement is trivially met. */
                pstrcat(suffix, sizeof(suffix), ".fcfill");
            }

            if (store) {
                if (scale == 2) {
                    /*
                     * `staw` reads exactly one 16-bit source channel, proven
                     * by USC (`dce.c:3074..3143`). The upper 16 bits may be
                     * unknown because they do not affect the stored halfword.
                     */
                    ClarionUseBits sval;

                    if (!use_read_bits(&c, b2, src2, &sval) ||
                        (sval.known & 0xffff) != 0xffff) {
                        sgx_pr("staw ⚠ lower source halfword unknown\n");
                        c.stop = "staw value unknown";
                        break;
                    }
                    val = sval.value & 0xffff;
                } else if (!use_read(&c, b2, src2, &val)) {
                    sgx_pr("stad  ⚠ value unknown\n");
                    c.stop = "stad value unknown";
                    break;
                }
                if (!sgx_translate(pd, addr, &pa2)) {
                    sgx_pr("%s%s [%s%u,+#%u] -> VA %08x is NOT mapped\n",
                            mnem, suffix, use_bank_name(b0), src0, off, addr);
                    c.stop = "ld/st target is unmapped";
                    break;
                }
                sgx_pr("%s%s [%s%u,+#%u] <- %0*x  (VA %08x, PA %08x)",
                        mnem, suffix, use_bank_name(b0), src0, off,
                        scale == 2 ? 4 : 8, val, addr, pa2);
                if (s->nullrender &&
                    sgx_b2_model_store_conflict(s, addr, pa2, scale)) {
                    c.stop = "model store conflicts with synthetic completion";
                    break;
                }
                if (scale == 2) {
                    sgx_phys_st16(pa2, (uint16_t)val);
                } else {
                    sgx_phys_st32(pa2, val);
                }
                sgx_b2_record_store(s, addr, pa2, val, scale, false);
                s->nstores++;
                sgx_pr("  ✔ WRITTEN TO GUEST MEMORY\n");
            } else {
                ClarionUseBank db = (w1 & USE1_LDST_DBANK_PRIMATTR)
                                    ? USE_BANK_PRIMATTR : USE_BANK_TEMP;

                if (scale == 2) {
                    /*
                     * ⚠⚠ Key point. Evidence establishes only that
                     * destination bits 15:0 receive the selected halfword.
                     * No source determines bits 31:16 (zero-extend,
                     * sign-extend, preserve old value, or undefined behavior).
                     *
                     * The model therefore performs no extension. It marks
                     * exactly 16 bits known and leaves the upper half unknown.
                     * If the microkernel depends on it, execution stops with
                     * the location; an invented zero would hide the issue.
                     */
                    uint32_t hw;

                    if (!sgx_translate(pd, addr, &pa2)) {
                        sgx_pr("ldaw%s %s%u <- [%s%u,+#%u] -> VA %08x NOT"
                                " mapped\n", suffix, use_bank_name(db),
                                dst, use_bank_name(b0), src0, off, addr);
                        c.stop = "ld/st target is unmapped";
                        break;
                    }
                    hw = sgx_phys_ld16(pa2);
                    sgx_pr("ldaw%s %s%u <- [%s%u,+#%u] = %04x  (VA %08x,"
                            " PA %08x; bits 31:16 UNKNOWN)\n",
                            suffix, use_bank_name(db), dst,
                            use_bank_name(b0), src0, off, hw, addr, pa2);
                    if (!use_write_bits(&c, db, dst, use_bits(hw, 0xffff))) {
                        c.stop = "unsupported ldaw destination";
                        break;
                    }
                } else {
                    sgx_pr("ldad%s %s%u..+%u <- [%s%u,+#%u]  (VA %08x)\n",
                            suffix, use_bank_name(db), dst, count - 1,
                            use_bank_name(b0), src0, off, addr);
                    for (i = 0; i < count; i++) {
                        if (!sgx_translate(pd, addr + i * 4, &pa2)) {
                            sgx_pr("[sgx]     %*s  VA %08x is NOT mapped\n",
                                    2 * depth, ind, addr + i * 4);
                            c.stop = "ld/st target is unmapped";
                            break;
                        }
                        val = sgx_phys_ld32(pa2);
                        sgx_pr("[sgx]     %*s  %s%u = %08x  (VA %08x)\n",
                                2 * depth, ind, use_bank_name(db), dst + i,
                                val, addr + i * 4);
                        {
                            uint32_t synthetic_value;
                            if (sgx_b2_synthetic_pa(s, pa2,
                                                    &synthetic_value)) {
                                sgx_pr("[sgx]     %*s  provenance:"
                                       " synthetic B2 completion%s\n",
                                       2 * depth, ind,
                                       val == synthetic_value
                                           ? " (value matches)"
                                           : " (value changed since B2)");
                            }
                        }
                        if (!use_write(&c, db, dst + i, val)) {
                            c.stop = "unsupported ldad destination";
                            break;
                        }
                    }
                    if (c.stop) {
                        break;
                    }
                }
            }

            if (imode != USE1_LDST_IMODE_NONE && step != 0) {
                uint32_t nb = base + step;

                if (!use_write(&c, b0, src0, nb)) {
                    sgx_pr("[sgx]     %*s  ⚠ cannot update base %s%u\n",
                            2 * depth, ind, use_bank_name(b0), src0);
                    c.stop = "cannot update ld/st base";
                    break;
                }
                sgx_pr("[sgx]     %*s  %s%u = %08x (increment %s#%u)\n",
                        2 * depth, ind, use_bank_name(b0), src0, nb,
                        imode == USE1_LDST_IMODE_PRE ? "before, " : "after, ",
                        off);
            }
            break;
        }

        case USE1_OP_IMAE: {
            /*
             * dst = src0(halfword) × src1(halfword) + src2.
             * The microkernel uses this to calculate a Kernel CCB slot address:
             * `imae r0, r1.low, #SIZEOF(SGXMKIF_COMMAND), r0, u32`
             * (`eventhandler.use.asm:383`) — 32 bytes per command because `SIZEOF`
             * in the assembler lexer is in BYTES (`use.l:1017` divides by 4 only
             * `DOFFSET`/`DSIZEOF`).
             */
            uint32_t spred = (w1 >> USE1_SPRED_SHIFT) & USE1_SPRED_MASK;
            uint32_t s2type = (w1 >> USE1_IMAE_SRC2TYPE_SHIFT) &
                              USE1_IMAE_SRC2TYPE_MASK;
            uint32_t orshift = (w1 >> USE1_IMAE_ORSHIFT_SHIFT) &
                               USE1_IMAE_ORSHIFT_MASK;
            uint32_t rcount = ((w1 >> USE1_INT_RCOUNT_SHIFT) &
                               USE1_INT_RCOUNT_MASK) + 1;
            /* ⚠ S0 has no bank extension here: bit 18 is END, not S0BEXT. */
            ClarionUseBank b0 = use_bank_s0_ex(w1, false);
            ClarionUseBank b1 = use_bank_s12((w0 >> USE0_S1BANK_SHIFT) &
                                             USE0_BANK_MASK,
                                             (w1 & USE1_S1BEXT) != 0);
            ClarionUseBank b2 = use_bank_s12((w0 >> USE0_S2BANK_SHIFT) &
                                             USE0_BANK_MASK,
                                             (w1 & USE1_S2BEXT) != 0);
            ClarionUseBank db = use_bank_dst(w1);
            uint32_t a, b, acc, res;
            bool spred_known, spred_taken;

            spred_taken = use_spred_true(&c, spred, &spred_known);
            if (!spred_known) {
                sgx_pr("imae  ⚠ short predicate %u unknown\n", spred);
                c.stop = "IMAE predicate unknown";
                break;
            }
            if (!spred_taken) {
                sgx_pr("imae (predicate false; skipped)\n");
                break;
            }
            if (w1 & USE1_IMAE_SIGNED) {
                sgx_pr("signed imae; unsupported\n");
                c.stop = "signed imae";
                break;
            }
            if (w1 & USE1_IMAE_SATURATE) {
                sgx_pr("saturating imae; unsupported\n");
                c.stop = "saturating imae";
                break;
            }
            if (w1 & (USE1_IMAE_CARRYINENABLE | USE1_IMAE_CARRYOUTENABLE)) {
                /* The model does not track internal i0/i1 registers. */
                sgx_pr("imae with carry; unsupported\n");
                c.stop = "imae with carry";
                break;
            }
            if (orshift != 0) {
                sgx_pr("imae with result shift by %u; unsupported\n",
                        orshift);
                c.stop = "imae result shift";
                break;
            }
            if (rcount != 1) {
                sgx_pr("imae repeat ×%u; unsupported\n", rcount);
                c.stop = "imae repeat";
                break;
            }
            if (s2type == USE1_IMAE_SRC2TYPE_MASK) {
                sgx_pr("imae with reserved src2 type\n");
                c.stop = "reserved src2 type in imae";
                break;
            }
            if (!use_read(&c, b0, src0, &a) ||
                !use_read(&c, b1, src1, &b) ||
                !use_read(&c, b2, src2, &acc)) {
                sgx_pr("imae  ⚠ source unknown\n");
                c.stop = "imae source unknown";
                break;
            }
            a = (w1 & USE1_IMAE_SRC0H_SELECTHIGH) ? (a >> 16) : (a & 0xFFFF);
            b = (w1 & USE1_IMAE_SRC1H_SELECTHIGH) ? (b >> 16) : (b & 0xFFFF);
            if (s2type != USE1_IMAE_SRC2TYPE_32BIT) {
                uint32_t h = (w1 & USE1_IMAE_SRC2H_SELECTHIGH)
                             ? (acc >> 16) : (acc & 0xFFFF);

                acc = (s2type == USE1_IMAE_SRC2TYPE_16BITSEXT)
                      ? (uint32_t)(int32_t)(int16_t)h : h;
            }
            res = a * b + acc;
            sgx_pr("imae %s%u, %s%u.%s, %s%u.%s, %s%u = %08x\n",
                    use_bank_name(db), dst,
                    use_bank_name(b0), src0,
                    (w1 & USE1_IMAE_SRC0H_SELECTHIGH) ? "high" : "low",
                    use_bank_name(b1), src1,
                    (w1 & USE1_IMAE_SRC1H_SELECTHIGH) ? "high" : "low",
                    use_bank_name(b2), src2, res);
            is_end = (w1 & USE1_END) != 0;
            if (!use_write(&c, db, dst, res)) {
                c.stop = "unsupported imae destination";
            }
            break;
        }

        default:
            sgx_pr("opcode %u; unsupported\n", op);
            c.stop = "unsupported USE opcode";
            break;
        }

        for (note = 0; note < c.notes; note++) {
            sgx_pr("[sgx]     %*s  ↳ %s\n", 2 * depth, ind,
                    c.note[note]);
        }

        if (c.stop) {
            break;
        }
        if (is_end) {
            sgx_pr("[sgx]     %*s— USE task completed\n",
                    2 * depth, ind);
            break;
        }
        pc = use_next_pc(&c, pc);
    }

    if (c.stop) {
        sgx_pr("[sgx]     %*s⛔ USE task stopped: %s\n",
                2 * depth, ind, c.stop);
    } else if (steps >= SGX_USE_MAX_STEPS) {
        sgx_pr("[sgx]     %*s⛔ USE task step limit reached\n",
                2 * depth, ind);
    }
}

/*
 * Report the secondary-attribute bank after execution.
 *
 * Also perform a self-check without relying on any hard-coded address. The
 * `PVRSRV_SGX_EDMPROG_SECATTR` layout (`sgx_mkif.h:144..156`) starts with
 * `sTA3DCtl`; our program's builder (`sgxinit.c:1961`) puts the device VA of
 * that same TA3D control block in `SBASE`. After a correct DMA, `sa[0]` must
 * equal the `SBASE` source. A match validates the offset, stride, and copy
 * direction; a mismatch exposes an error.
 */
static void sgx_report_sa(ClarionSgxState *s)
{
    unsigned i;

    if (!s->sa_count) {
        sgx_pr("[sgx]   secondary attributes: DMA was not executed\n");
        return;
    }
    sgx_pr("[sgx]   microkernel secondary attributes (%u doublewords):\n",
            s->sa_count);
    for (i = 0; i < s->sa_count; i += 4) {
        unsigned k;

        sgx_pr("[sgx]     sa[%2u]:", i);
        for (k = 0; k < 4 && i + k < s->sa_count; k++) {
            if (s->sa_known[i + k]) {
                sgx_pr(" %08x", s->sa[i + k]);
            } else {
                sgx_pr(" --------");
            }
        }
        sgx_pr("\n");
    }
    sgx_pr("[sgx]     sa[%u] sTA3DCtl = %08x\n", SGX_SA_TA3DCTL,
            s->sa[SGX_SA_TA3DCTL]);
    sgx_pr("[sgx]     sa[%u] sHostCtl = %08x   ← R_HostCtl in USE code\n",
            SGX_SA_HOSTCTL, s->sa[SGX_SA_HOSTCTL]);
    sgx_pr("[sgx]     sa[%u] sCCBCtl  = %08x\n", SGX_SA_CCBCTL,
            s->sa[SGX_SA_CCBCTL]);
    if (s->sa_known[SGX_SA_TA3DCTL] && s->sa[SGX_SA_TA3DCTL] == s->sa_sbase) {
        sgx_pr("[sgx]     ✔ self-check: sa[0] == DMA SBASE (%08x);"
                " copy offset and stride are correct\n", s->sa_sbase);
    } else if (s->sa_known[SGX_SA_TA3DCTL]) {
        sgx_pr("[sgx]     ⛔ self-check FAILED: sa[0]=%08x, expected"
                " SBASE DMA = %08x\n", s->sa[SGX_SA_TA3DCTL], s->sa_sbase);
    }
}

/* --- report on kick --------------------------------------------------- */

static void sgx_report_kick(ClarionSgxState *s)
{
    uint32_t pds   = sgx_reg(s, SGX_CR_PDS_EXEC_BASE) &
                     SGX_PDS_EXEC_BASE_ADDR_MASK;
    uint32_t task  = sgx_reg(s, SGX_CR_EVENT_OTHER_PDS_EXEC);
    uint32_t use15 = sgx_reg(s, SGX_CR_USE_CODE_BASE(15));
    uint32_t kicker = sgx_reg(s, SGX_CR_EVENT_KICKER) &
                      SGX_EVENT_KICKER_ADDR_MASK;
    uint32_t pd    = sgx_reg(s, SGX_CR_BIF_DIR_LIST_BASE0) &
                     SGX_BIF_DIR_LIST_BASE_ADDR_MASK;
    uint32_t root  = pds + task;
    uint32_t pa;
    unsigned i;

    sgx_pr("[sgx] EVENT_KICK2 #%u — decoding visible state\n",
            s->kicks);
    if (s->nullrender && !s->exec) {
        sgx_b2_skip(s->kicks, "no_exec", 0);
    }
    sgx_pr("[sgx]   EVENT_HOST_ENABLE  = %08x   (bit 14 = SW event)\n",
            sgx_reg(s, SGX_CR_EVENT_HOST_ENABLE));
    sgx_pr("[sgx]   EVENT_KICKER       = %08x   counter device VA\n",
            kicker);
    sgx_pr("[sgx]   PDS_EXEC_BASE      = %08x\n", pds);
    sgx_pr("[sgx]   USE_CODE_BASE_15   = %08x   DM=%u ADDR<<8=%08x\n",
            use15,
            (use15 & SGX_USE_CODE_BASE_DM_MASK) >> SGX_USE_CODE_BASE_DM_SHIFT,
            (use15 & SGX_USE_CODE_BASE_ADDR_MASK) << 8);
    sgx_pr("[sgx]   EVENT_OTHER_PDS EXEC/DATA/INFO + 0x0A74 ="
            " %08x %08x %08x %08x\n",
            task, sgx_reg(s, SGX_CR_EVENT_OTHER_PDS_DATA),
            sgx_reg(s, SGX_CR_EVENT_OTHER_PDS_INFO), sgx_reg(s, SGX_CR_QY8_TASK_W3));
    sgx_pr("[sgx]   root = PDS_EXEC_BASE + reg 0x0A68 = %08x\n",
            root);
    sgx_pr("[sgx]   BIF_DIR_LIST_BASE0 = %08x\n", pd);

    if (s->show_writes) {
        /*
         * Full journal in arrival order. The init script is not the only
         * source of writes: `SGXReset` writes some registers (including
         * BIF_DIR_LIST_BASE0) directly, absent from the script table.
         */
        sgx_pr("[sgx]   write journal (%u%s):\n", s->nwr,
                s->wr_overflow ? ", OVERFLOW" : "");
        for (i = 0; i < s->nwr; i++) {
            sgx_pr("[sgx]     %3u  +0x%04x <- %08x\n",
                    i, s->wr[i].off, s->wr[i].val);
        }
    }

    if (!pd) {
        /*
         * The guest did not write a directory to this window. Without a
         * directory no device VA can be translated; report that directly.
         */
        sgx_pr("[sgx]   no page directory written to this block; MMU traversal unavailable\n");
        return;
    }

    sgx_report_pd(pd);

    if (!sgx_translate(pd, root, &pa)) {
        sgx_pr("[sgx]   root %08x is NOT mapped in this directory\n", root);
    } else {
        sgx_report_range(pd, "root", root, 0x40);
    }

    for (i = 0; i < s->ndump; i++) {
        sgx_report_range(pd, "QY8_SGX_DUMP", s->dump[i].va, s->dump[i].len);
    }
    for (i = 0; i < s->nfind; i++) {
        sgx_find_target(s, pd, pds, (use15 & SGX_USE_CODE_BASE_ADDR_MASK) << 8,
                        s->find[i]);
    }
    if (s->graph) {
        sgx_report_graph(pd);
    }
    if (pds && task) {
        /*
         * Program referenced by the "Other" event task-control. On the first
         * kick this is `sgxinit_primary` (#13); later it is the event handler
         * that the microkernel installed itself using instructions
         * `str #666/#667`.
         *
         * Event input registers. `ir1` carries event flags, including the
         * software event. The guest enables `SW_EVENT` in
         * `EUR_CR_EVENT_PDS_ENABLE` (bit 14) before hardware delivers it to
         * PDS. Register and `ir1` bit numbers differ (14 versus 8), so map by
         * event name.
         *
         * If the guest did not enable the PDS software event, mark `ir`
         * unknown. Execution stops if the program tries to read it.
         */
        uint32_t pdsen = sgx_reg(s, SGX_CR_EVENT_PDS_ENABLE);
        bool sw = (pdsen & SGX_EVENT_SW_EVENT_MASK) != 0;

        sgx_pr("[sgx]   EVENT_PDS_ENABLE   = %08x   SW_EVENT(bit 14) %s\n",
               pdsen, sw ? "enabled by guest" : "NOT enabled");
        sgx_pds_run(s, pd, root, sgx_reg(s, SGX_CR_EVENT_OTHER_PDS_DATA),
                    "Other event task-control", sw, 0,
                    sw ? PDS_IR1_EDM_EVENT_SWEVENT : 0, 0);
    }

    /*
     * ⚠ Diagnostic override, not part of the normal chain. The engineer
     * takes the program address from the trace; the model does not search or
     * guess it. Needed until USE can let microkernel `emitpds` launch
     * `sgxinit_secondary` (#14), whose DOUTD transfers the secondary attributes.
     */
    if (s->run_set) {
        sgx_pr("[sgx]   ⚠ QY8_SGX_PDS_RUN — diagnostic launch, not part of the normal chain\n");
        sgx_pds_run(s, pd, s->run_va, s->run_rows, "QY8_SGX_PDS_RUN",
                    true, s->run_ir0, 0, 0);
    }

    if (s->exec) {
        sgx_report_sa(s);
        if (s->nuse) {
            unsigned k;

            sgx_pr("[sgx]   launched USE tasks:\n");
            for (k = 0; k < s->nuse; k++) {
                sgx_pr("[sgx]     #%u  device VA %08x\n",
                        k, s->use_queue[k]);
            }
        }
    }
}

/* --- MMIO ------------------------------------------------------------- */

static uint64_t sgx_read(void *opaque, hwaddr addr, unsigned size)
{
    ClarionSgxState *s = opaque;
    uint64_t val = 0;

    if (size == 4 && (addr == SGX_CR_USE0_SERV_EVENT ||
                      addr == SGX_CR_USE1_SERV_EVENT)) {
        val = addr == SGX_CR_USE0_SERV_EVENT ? sgx_edm_task_count(s) : 0;
    } else if (s->readback && size == 4 && addr + 4 <= CLARION_SGX_SIZE) {
        val = sgx_reg(s, addr);
    }

    /*
     * Reads remain logged under LOG_UNIMP because reads really are not
     * modeled. `-d unimp` was also used for earlier traces of this block.
     * Keep this line in the broad-interceptor form for A/B comparisons.
     */
    qemu_log_mask(LOG_UNIMP, "clarion-sgx: read  (size %d, offset 0x%04"
                  HWADDR_PRIx ") -> 0x%0*" PRIx64 "\n",
                  size, addr, size << 1, val);
    trace_clarion_sgx_read((uint32_t)addr, size, val);
    return val;
}

static void sgx_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    ClarionSgxState *s = opaque;

    qemu_log_mask(LOG_UNIMP, "clarion-sgx: write (size %d, offset 0x%04"
                  HWADDR_PRIx ", value 0x%0*" PRIx64 ")\n",
                  size, addr, size << 1, val);
    trace_clarion_sgx_write((uint32_t)addr, size, val);

    if (size == 4 && addr + 4 <= CLARION_SGX_SIZE) {
        /*
         * Identity registers are hardware read-only; do not change their
         * values or the model would forget which core it represents. The
         * write is still journaled because the journal records guest actions.
         */
        if (addr == SGX_CR_CORE_ID || addr == SGX_CR_CORE_REVISION) {
            qemu_log_mask(LOG_GUEST_ERROR, "clarion-sgx: write to identity register"
                          " identity register 0x%04" HWADDR_PRIx
                          " — ignored\n", addr);
        } else {
            if (addr == SGX_CR_EVENT_STATUS) {
                if (val > UINT32_MAX ||
                    !sgx_write_event_status(s, (uint32_t)val)) {
                    error_report("clarion-sgx: EUR_CR_EVENT_STATUS +0x%04x"
                                 " value 0x%08" PRIx64 " unsupported mask"
                                 " 0x%08" PRIx64 " origin MMIO",
                                 (unsigned)addr, val,
                                 val & ~((uint64_t)SGX_CR_EVENT_STATUS_SW_EVENT));
                    abort();
                }
            } else {
                sgx_set_reg(s, addr, (uint32_t)val);
                sgx_reg_side_effects(s, addr, (uint32_t)val);
            }
        }
        if (s->nwr < SGX_WR_JOURNAL) {
            s->wr[s->nwr].off = (uint32_t)addr;
            s->wr[s->nwr].val = (uint32_t)val;
            s->nwr++;
        } else {
            s->wr_overflow = true;
        }
    }

    if (addr == SGX_CR_EVENT_KICK2 && (val & SGX_CR_EVENT_KICK2_NOW)) {
        s->kicks++;
        if (s->kicks > s->kick_reports) {
            /*
             * Continue execution silently. Otherwise the microkernel would
             * run only on the initial kicks, while the guest can kick as
             * often as needed.
             */
            if (s->kicks == s->kick_reports + 1) {
                sgx_fprintf("[sgx] EVENT_KICK2 #%u — decode suppressed"
                        " (QY8_SGX_KICKS=%u); execution continues\n",
                        s->kicks, s->kick_reports);
            }
            sgx_silent = true;
        }
        sgx_report_kick(s);
        sgx_silent = s->kick_reports == 0;
    }
}

static const MemoryRegionOps sgx_ops = {
    .read = sgx_read,
    .write = sgx_write,
    /*
     * Keep the same access range as the board's broad interceptor. Firmware
     * uses more than 32-bit accesses here (traces include 8-byte reads at
     * offsets 0x100..0x1F8); the model must not narrow guest behavior.
     */
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* --- QOM -------------------------------------------------------------- */

static void clarion_sgx_reset_hold(Object *obj, ResetType type)
{
    ClarionSgxState *s = CLARION_SGX(obj);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->regs_known_mask, 0, sizeof(s->regs_known_mask));
    memset(s->regs_version, 0, sizeof(s->regs_version));
    s->regs_epoch = 0;
    s->kicks = 0;
    s->nwr = 0;
    s->wr_overflow = false;
    memset(s->sa, 0, sizeof(s->sa));
    memset(s->sa_known, 0, sizeof(s->sa_known));
    memset(s->sa_known_mask, 0, sizeof(s->sa_known_mask));
    s->sa_count = 0;
    memset(s->pa, 0, sizeof(s->pa));
    memset(s->pa_known, 0, sizeof(s->pa_known));
    memset(s->pa_known_mask, 0, sizeof(s->pa_known_mask));
    s->pa_count = 0;
    s->sa_sbase = 0;
    s->nuse = 0;
    s->nstores = 0;
    memset(s->b2_synthetic, 0, sizeof(s->b2_synthetic));
    s->nb2_synthetic = 0;
    memset(s->b2_stores, 0, sizeof(s->b2_stores));
    s->nb2_stores = 0;
    s->b2_store_overflow = false;
    s->b2_disabled = false;
    s->edm_timer_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->edm_timer_period_ns = 0;

    /*
     * Identity registers always exist; they are not written, but intrinsic
     * hardware values. They are therefore KNOWN after reset, unlike the rest
     * of the register window.
     */
    s->regs[SGX_CR_CORE_REVISION / 4] = s->core_rev;
    s->regs_known_mask[SGX_CR_CORE_REVISION / 4] = UINT32_MAX;
    s->regs[SGX_CR_CORE_ID / 4] = s->core_id;
    s->regs_known_mask[SGX_CR_CORE_ID / 4] = UINT32_MAX;
    s->core_id_warned = false;
}

/* QY8_SGX_DUMP="0x0F003000:0x104,0x0E40C1B0:0x4C" */
static void sgx_parse_dump(ClarionSgxState *s, const char *spec)
{
    while (spec && *spec && s->ndump < SGX_DUMP_RANGES) {
        char *end;
        uint64_t va = strtoull(spec, &end, 0);
        uint64_t len = 0x40;

        if (end == spec) {
            break;
        }
        if (*end == ':') {
            spec = end + 1;
            len = strtoull(spec, &end, 0);
        }
        s->dump[s->ndump].va = (uint32_t)va;
        s->dump[s->ndump].len = (uint32_t)len;
        s->ndump++;
        if (*end != ',') {
            break;
        }
        spec = end + 1;
    }
}

/*
 * QY8_SGX_PDS_RUN="VA[:rows[:ir0]]" — execute the named PDS program.
 *
 * The default is 3 rows (`sgxinit_secondary` has exactly this: #14
 * = #13 + 0x30, and 3 rows × 2 banks × 2 doublewords = 0x30 bytes of data).
 * The default ir0 is 0; this is a specific case, not an arbitrary default:
 * in
 * `sgx_init.use.asm:93..104` ir0 comes from
 * `HOST_CTL.ui32InterruptClearFlags & PVRSRV_USSE_EDM_INTERRUPT_HWR`, meaning
 * zero means "cold start, not hardware recovery"; only then does secondary
 * perform DMA.
 */
static void sgx_parse_pds_run(ClarionSgxState *s, const char *spec)
{
    char *end;

    if (!spec || !*spec) {
        return;
    }
    s->run_va = (uint32_t)strtoull(spec, &end, 0);
    if (end == spec) {
        return;
    }
    s->run_rows = 3;
    s->run_ir0 = 0;
    if (*end == ':') {
        spec = end + 1;
        s->run_rows = (uint32_t)strtoull(spec, &end, 0);
        if (*end == ':') {
            spec = end + 1;
            s->run_ir0 = (uint32_t)strtoull(spec, &end, 0);
        }
    }
    s->run_set = true;
}

/* QY8_SGX_FIND="0x0E40C1B0,0x0F003000" */
static void sgx_parse_find(ClarionSgxState *s, const char *spec)
{
    while (spec && *spec && s->nfind < SGX_FIND_TARGETS) {
        char *end;
        uint64_t va = strtoull(spec, &end, 0);

        if (end == spec) {
            break;
        }
        s->find[s->nfind++] = (uint32_t)va;
        if (*end != ',') {
            break;
        }
        spec = end + 1;
    }
}

/*
 * Target DDK 1.7 disassembly proves that SGXOSTimer reads HOST_CTL+0x40,
 * remembers changes, and decrements an unchanged nonzero value to skip
 * recovery. DDK 1.8 names this field ui32OpenCLDelayCount. This legacy
 * opt-in therefore remains a diagnostic shim, not the modeled task counter.
 *
 * Target static disassembly shows the timer callback reading two configured
 * USE service-event offsets, XORing their values, and checking after three
 * unchanged samples. DDK 1.8 maps these offsets to USE0/USE1_SERV_EVENT and
 * initializes the second offset to zero. The target's EVENT_TIMER write is
 * observed in the existing trace; its use as the microkernel tick source is
 * inferred from DDK 1.8, not proven by target disassembly.
 */
#define SGX_HOSTCTL_UKERNEL_CLOCK   0x40
#define SGX_HEARTBEAT_MS            10

static void sgx_heartbeat(void *opaque)
{
    ClarionSgxState *s = opaque;
    uint32_t pd = sgx_reg(s, SGX_CR_BIF_DIR_LIST_BASE0) &
                  SGX_BIF_DIR_LIST_BASE_ADDR_MASK;
    uint32_t pa;

    if (s->sa_known[SGX_SA_HOSTCTL] && pd &&
        sgx_translate(pd, s->sa[SGX_SA_HOSTCTL] + SGX_HOSTCTL_UKERNEL_CLOCK,
                      &pa)) {
        uint32_t v = sgx_phys_ld32(pa) + 1;

        sgx_phys_st32(pa, v ? v : 1);
    }
    timer_mod(s->heartbeat,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + SGX_HEARTBEAT_MS);
}

static void clarion_sgx_realize(DeviceState *dev, Error **errp)
{
    ClarionSgxState *s = CLARION_SGX(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    const char *e;

    e = getenv("QY8_SGX_KICKS");
    s->kick_reports = e ? (uint32_t)atoi(e) : 0;
    sgx_silent = s->kick_reports == 0;
    s->readback = getenv("QY8_SGX_READBACK") != NULL;
    e = getenv("QY8_SGX_NULLRENDER");
    s->nullrender = !e || (strcmp(e, "off") && strcmp(e, "0"));
    s->nullrender_all = e && !strcmp(e, "all");
    s->show_writes = getenv("QY8_SGX_WRITES") != NULL;
    s->graph = getenv("QY8_SGX_GRAPH") != NULL;
    e = getenv("QY8_SGX_EXEC");
    s->exec = !e || (strcmp(e, "off") && strcmp(e, "0"));
    e = getenv("QY8_SGX_CORE_REV");
    s->core_rev = e ? (uint32_t)strtoul(e, NULL, 0) : SGX_CORE_REVISION_QY8;
    e = getenv("QY8_SGX_CORE_ID");
    s->core_id = e ? (uint32_t)strtoul(e, NULL, 0) : SGX_CORE_ID_UNKNOWN;
    sgx_fprintf("[sgx] core identity: EUR_CR_CORE_REVISION = %08x"
            " (SGX540 r%u.%u.%u — firmware constant; see header),"
            " EUR_CR_CORE_ID = %08x%s\n",
            s->core_rev,
            (s->core_rev >> 16) & 0xFF, (s->core_rev >> 8) & 0xFF,
            s->core_rev & 0xFF, s->core_id,
            s->core_id == SGX_CORE_ID_UNKNOWN
                ? " ⚠ NO BASIS (no check reads this value)" : "");

    if (s->nullrender) {
        sgx_fprintf("[sgx] QY8_SGX_NULLRENDER: render completion is"
                " SYNTHETIC; no rasterization occurred; pixel contents are"
                " undefined\n");
    }
    if (s->nullrender_all) {
        warn_report("clarion-sgx: QY8_SGX_NULLRENDER=all: MIRROR-only;"
                    " dst-sync completion for every READY TA command;"
                    " no queue offset is advanced");
    }

    sgx_parse_dump(s, getenv("QY8_SGX_DUMP"));
    sgx_parse_find(s, getenv("QY8_SGX_FIND"));
    sgx_parse_pds_run(s, getenv("QY8_SGX_PDS_RUN"));

    if (s->exec) {
        /*
         * DOUTD/USE execute in the model. Guest memory is modified only by
         * modeled USE stores and, under a separate flag, minimal B2. This
         * option does not perform real rasterization.
         */
        if (s->nullrender) {
            sgx_fprintf("[sgx] QY8_SGX_EXEC: PDS/USE programs EXECUTE"
                    " (B2 may synthetically complete a sync word)\n");
        } else {
            sgx_fprintf("[sgx] QY8_SGX_EXEC: PDS programs EXECUTE"
                    " (effects stay in model state; guest sees no changes)\n");
        }
    }
    if (s->readback) {
        warn_report("clarion-sgx: QY8_SGX_READBACK: reads return written"
                    " values; guest-visible behavior CHANGED");
    }

    e = getenv("QY8_SGX_HEARTBEAT");
    if (e && (!strcmp(e, "on") || !strcmp(e, "1"))) {
        s->heartbeat = timer_new_ms(QEMU_CLOCK_VIRTUAL, sgx_heartbeat, s);
        timer_mod(s->heartbeat,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + SGX_HEARTBEAT_MS);
    }

    memory_region_init_io(&s->mr, OBJECT(dev), &sgx_ops, s,
                          "clarion-sgx", CLARION_SGX_SIZE);
    sysbus_init_mmio(sbd, &s->mr);
}

static const Property clarion_sgx_properties[] = {
    DEFINE_PROP_BOOL("edm-task-register-model", ClarionSgxState,
                     edm_task_register_model, true),
};

static void clarion_sgx_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = clarion_sgx_realize;
    dc->desc = "Clarion QY8XXX PowerVR SGX (passive MMIO + BIF/MMU tracer)";
    device_class_set_props(dc, clarion_sgx_properties);
    rc->phases.hold = clarion_sgx_reset_hold;
}

static const TypeInfo clarion_sgx_types[] = {
    {
        .name          = TYPE_CLARION_SGX,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ClarionSgxState),
        .class_init    = clarion_sgx_class_init,
    },
};

DEFINE_TYPES(clarion_sgx_types)
