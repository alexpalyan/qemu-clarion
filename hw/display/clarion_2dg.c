/*
 * Clarion 2DG model: Renesas M2DG-style command list executor.
 *
 * The list format follows the open-source DirectFB sh772x driver
 * (gfxdrivers/sh772x/sh7723_blt.[ch]) plus the extensions seen in lists
 * submitted by the QY8 guest (sublist call/return, indirect register
 * loads, centre-relative rectangles, clip paths, textured polygons).
 * Every extension is derived from captured lists only; fields that could
 * not be established are noted next to the code that skips them.
 *
 * The 1 ms completion delay and the +0x0c enable-bit meaning are SYNTHETIC.
 */
#include "qemu/osdep.h"
#include "hw/display/clarion_2dg.h"
#include "system/address-spaces.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/bswap.h"

#define CLARION_2DG_SIZE 0x200
#define CLARION_2DG_REGS (CLARION_2DG_SIZE / sizeof(uint32_t))
#define CLARION_2DG_STATUS 0x04
#define CLARION_2DG_ACK 0x08
#define CLARION_2DG_IRQ_ENABLE 0x0c
#define CLARION_2DG_LIST 0x48
#define CLARION_2DG_DONE 1
#define CLARION_2DG_DELAY_NS 1000000
#define CLARION_2DG_LOG_FALLBACK_WORDS 256
#define CLARION_2DG_LOG_PRE_WORDS 64

/* List execution. */
#define CLARION_2DG_LREGS 0x200
#define CLARION_2DG_HDR_MAGIC 0x52473130u /* 'RG10', 0x70 bytes before list */
#define CLARION_2DG_HDR_BACK 0x70
#define CLARION_2DG_HDR_LEN 0x08 /* byte offset of the length field */
#define CLARION_2DG_MAX_WORDS 65536
#define CLARION_2DG_FALLBACK_WORDS 16384
#define CLARION_2DG_MAX_PATH 256
#define CLARION_2DG_MAX_DIM 8192
#define CLARION_2DG_MAX_TEX 128
#define CLARION_2DG_MAX_CALL 4

/* Command opcodes (bits 31..24 of the first word). */
#define M2DG_TRAP 0x00
#define M2DG_NOP 0x08
#define M2DG_SYNC 0x12
#define M2DG_WPR 0x18
#define M2DG_DATA 0x28
#define M2DG_CALL 0x30
#define M2DG_RET 0x38
#define M2DG_LCOFS 0x40
#define M2DG_MOVE 0x48
#define M2DG_POLY4C 0x80
#define M2DG_POLY4T 0x82
#define M2DG_BITBLTC 0xa0
#define M2DG_BITBLTA 0xa2
#define M2DG_BITBLTK 0xa8
#define M2DG_BITBLTR 0xaa
#define M2DG_LINE 0xb0
#define M2DG_LINE_NC 0xb3
#define M2DG_CLIPPATH 0xd0
#define M2DG_CLIPRECT 0xe0

/* Header flag bits. */
#define M2DG_F_LINE_IND 0x4000
#define M2DG_F_WPR_IND 0x0400
#define M2DG_F_STRANS 0x0800

/* List registers (byte offsets). */
#define M2DG_R_SRC 0x4c
#define M2DG_R_DST 0x50
#define M2DG_R_SRC_STRIDE 0x58
#define M2DG_R_DST_STRIDE 0x5c
#define M2DG_R_CTRL 0xc0
#define M2DG_R_SYSCLIP 0xd0
#define M2DG_R_CLIP_MIN 0xdc
#define M2DG_R_CLIP_MAX 0xe0

struct Clarion2DGState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
    uint32_t regs[CLARION_2DG_REGS];
    char *log_path;
    bool exec;
    /* Command-list register file; persists across lists, not migrated. */
    uint32_t lreg[CLARION_2DG_LREGS / 4];
    bool lreg_set[CLARION_2DG_LREGS / 4];
};

static void clarion_2dg_update_irq(Clarion2DGState *s)
{
    qemu_set_irq(s->irq, (s->regs[CLARION_2DG_STATUS / 4] & CLARION_2DG_DONE) &&
                             (s->regs[CLARION_2DG_IRQ_ENABLE / 4] & 1));
}

typedef struct ListCtx {
    Clarion2DGState *s;
    /* Mapped destination canvas. */
    uint8_t *dst;
    hwaddr dst_base;
    hwaddr dst_len;
    uint32_t dst_stride;
    uint32_t dst_h;
    /* Clip rectangle and clip path set by 0xe0 / 0xd0, one-shot. */
    bool rc_on;
    int rx0, ry0, rx1, ry1;
    bool path_on;
    unsigned path_n;
    int px[CLARION_2DG_MAX_PATH];
    int py[CLARION_2DG_MAX_PATH];
    /* Statistics. */
    unsigned done[256];
    unsigned skipped[256];
    unsigned stopped_op;
} ListCtx;

static bool clarion_2dg_rd32(hwaddr addr, uint32_t *value)
{
    uint32_t word;

    if (address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                           &word, sizeof(word)) != MEMTX_OK) {
        *value = 0;
        return false;
    }
    *value = le32_to_cpu(word);
    return true;
}

static void clarion_2dg_dst_release(ListCtx *c)
{
    if (c->dst) {
        address_space_unmap(&address_space_memory, c->dst, c->dst_len, true,
                            c->dst_len);
        c->dst = NULL;
    }
}

/* Map the destination canvas described by list registers 0x50/0x5c/0xd0. */
static bool clarion_2dg_dst_map(ListCtx *c)
{
    Clarion2DGState *s = c->s;
    hwaddr base = s->lreg[M2DG_R_DST / 4];
    uint32_t stride = s->lreg[M2DG_R_DST_STRIDE / 4];
    uint32_t height = (s->lreg[M2DG_R_SYSCLIP / 4] & 0xffff) + 1;
    hwaddr len, want;
    void *ptr;

    if (c->dst && c->dst_base == base && c->dst_stride == stride &&
        c->dst_h == height) {
        return true;
    }
    clarion_2dg_dst_release(c);
    if (!s->lreg_set[M2DG_R_DST / 4] || !s->lreg_set[M2DG_R_DST_STRIDE / 4] ||
        !s->lreg_set[M2DG_R_SYSCLIP / 4] || !stride ||
        stride > CLARION_2DG_MAX_DIM || height > CLARION_2DG_MAX_DIM) {
        return false;
    }
    want = (hwaddr)stride * height * 2;
    len = want;
    ptr = address_space_map(&address_space_memory, base, &len, true,
                            MEMTXATTRS_UNSPECIFIED);
    if (!ptr) {
        return false;
    }
    if (len != want) {
        address_space_unmap(&address_space_memory, ptr, len, true, 0);
        return false;
    }
    c->dst = ptr;
    c->dst_base = base;
    c->dst_len = len;
    c->dst_stride = stride;
    c->dst_h = height;
    return true;
}

/* Effective clip box (inclusive) from system clip and window clip. */
static bool clarion_2dg_clip(ListCtx *c, int *x0, int *y0, int *x1, int *y1)
{
    Clarion2DGState *s = c->s;
    uint32_t sys = s->lreg[M2DG_R_SYSCLIP / 4];

    *x0 = 0;
    *y0 = 0;
    *x1 = MIN((int)(sys >> 16), (int)c->dst_stride - 1);
    *y1 = MIN((int)(sys & 0xffff), (int)c->dst_h - 1);
    if (s->lreg_set[M2DG_R_CLIP_MIN / 4] && s->lreg_set[M2DG_R_CLIP_MAX / 4]) {
        uint32_t mn = s->lreg[M2DG_R_CLIP_MIN / 4];
        uint32_t mx = s->lreg[M2DG_R_CLIP_MAX / 4];

        *x0 = MAX(*x0, (int)(mn >> 16));
        *y0 = MAX(*y0, (int)(mn & 0xffff));
        *x1 = MIN(*x1, (int)(mx >> 16));
        *y1 = MIN(*y1, (int)(mx & 0xffff));
    }
    return *x0 <= *x1 && *y0 <= *y1;
}

static inline void clarion_2dg_put(ListCtx *c, int x, int y, uint16_t v)
{
    stw_le_p(c->dst + ((size_t)y * c->dst_stride + x) * 2, v);
}

/* Even-odd containment, plus pixels within half a pixel of an edge. */
static bool clarion_2dg_in_poly(const int *vx, const int *vy, unsigned n,
                                int x, int y)
{
    bool inside = false;
    unsigned i, j;

    for (i = 0, j = n - 1; i < n; j = i++) {
        double xi = vx[i], yi = vy[i], xj = vx[j], yj = vy[j];
        double dx = xj - xi, dy = yj - yi, len2 = dx * dx + dy * dy;
        double t = len2 ? ((x - xi) * dx + (y - yi) * dy) / len2 : 0;
        double ex, ey;

        if (t < 0) {
            t = 0;
        } else if (t > 1) {
            t = 1;
        }
        ex = xi + t * dx - x;
        ey = yi + t * dy - y;
        if (ex * ex + ey * ey <= 0.25) {
            return true;
        }
        if ((yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) {
            inside = !inside;
        }
    }
    return inside;
}

/* Fill a polygon; tex (w x h, or NULL) selects texture or solid colour. */
static void clarion_2dg_fill_poly(ListCtx *c, const int *vx, const int *vy,
                                  unsigned n, uint16_t colour,
                                  const uint16_t *tex, unsigned tw, unsigned th)
{
    int cx0, cy0, cx1, cy1, minx = INT_MAX, miny = INT_MAX;
    int maxx = INT_MIN, maxy = INT_MIN, x, y;
    unsigned i;

    if (!clarion_2dg_dst_map(c) ||
        !clarion_2dg_clip(c, &cx0, &cy0, &cx1, &cy1)) {
        return;
    }
    for (i = 0; i < n; i++) {
        minx = MIN(minx, vx[i]);
        maxx = MAX(maxx, vx[i]);
        miny = MIN(miny, vy[i]);
        maxy = MAX(maxy, vy[i]);
    }
    if (c->rc_on) {
        cx0 = MAX(cx0, c->rx0);
        cy0 = MAX(cy0, c->ry0);
        cx1 = MIN(cx1, c->rx1);
        cy1 = MIN(cy1, c->ry1);
    }
    minx = MAX(minx, cx0);
    miny = MAX(miny, cy0);
    maxx = MIN(maxx, cx1);
    maxy = MIN(maxy, cy1);
    for (y = miny; y <= maxy; y++) {
        for (x = minx; x <= maxx; x++) {
            if (!clarion_2dg_in_poly(vx, vy, n, x, y)) {
                continue;
            }
            if (c->path_on &&
                !clarion_2dg_in_poly(c->px, c->py, c->path_n, x, y)) {
                continue;
            }
            clarion_2dg_put(c, x, y,
                            tex ? tex[(y % th) * tw + (x % tw)] : colour);
        }
    }
}

static void clarion_2dg_fill_rect(ListCtx *c, int x0, int y0, int x1, int y1,
                                  uint16_t colour)
{
    int cx0, cy0, cx1, cy1, x, y;

    if (!clarion_2dg_dst_map(c) ||
        !clarion_2dg_clip(c, &cx0, &cy0, &cx1, &cy1)) {
        return;
    }
    x0 = MAX(x0, cx0);
    y0 = MAX(y0, cy0);
    x1 = MIN(x1, cx1);
    y1 = MIN(y1, cy1);
    for (y = y0; y <= y1; y++) {
        for (x = x0; x <= x1; x++) {
            clarion_2dg_put(c, x, y, colour);
        }
    }
}

/*
 * Source-to-canvas blit. key >= 0 selects colour-key mode: pixels with the
 * ARGB1555 alpha bit clear are skipped. The literal key word (0x7fff in all
 * captured lists) left the 0x0000 background opaque, so it is not compared.
 */
static void clarion_2dg_blit(ListCtx *c, int sx, int sy, int w, int h, int dx,
                             int dy, int key)
{
    Clarion2DGState *s = c->s;
    hwaddr src = s->lreg[M2DG_R_SRC / 4];
    uint32_t sstride = s->lreg[M2DG_R_SRC_STRIDE / 4];
    uint16_t row[CLARION_2DG_MAX_DIM];
    int cx0, cy0, cx1, cy1, i, j;

    if (!clarion_2dg_dst_map(c) || !clarion_2dg_clip(c, &cx0, &cy0, &cx1, &cy1)
        || !s->lreg_set[M2DG_R_SRC / 4] || !sstride ||
        w <= 0 || h <= 0 || w > CLARION_2DG_MAX_DIM) {
        return;
    }
    for (j = 0; j < h; j++) {
        int y = dy + j;

        if (y < cy0 || y > cy1) {
            continue;
        }
        if (address_space_read(
                &address_space_memory,
                src + (((hwaddr)(sy + j) * sstride) + sx) * 2,
                MEMTXATTRS_UNSPECIFIED, row, w * 2) != MEMTX_OK) {
            continue;
        }
        for (i = 0; i < w; i++) {
            int x = dx + i;
            uint16_t v = lduw_le_p(&row[i]);

            if (x < cx0 || x > cx1 || (key >= 0 && !(v & 0x8000))) {
                continue;
            }
            clarion_2dg_put(c, x, y, v);
        }
    }
}

/* Polyline with a square brush of thickness t. */
static void clarion_2dg_polyline(ListCtx *c, const int *vx, const int *vy,
                                 unsigned n, uint16_t colour, int t)
{
    int cx0, cy0, cx1, cy1, lo, hi;
    unsigned k;

    if (!clarion_2dg_dst_map(c) ||
        !clarion_2dg_clip(c, &cx0, &cy0, &cx1, &cy1)) {
        return;
    }
    lo = -((t - 1) / 2);
    hi = t / 2;
    for (k = 0; k + 1 < n || (n == 1 && k == 0); k++) {
        int x0 = vx[k], y0 = vy[k];
        int x1 = vx[n == 1 ? 0 : k + 1], y1 = vy[n == 1 ? 0 : k + 1];
        int dx = abs(x1 - x0), dy = -abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;

        for (;;) {
            int bx, by, e2;

            for (by = lo; by <= hi; by++) {
                for (bx = lo; bx <= hi; bx++) {
                    int x = x0 + bx, y = y0 + by;

                    if (x >= cx0 && x <= cx1 && y >= cy0 && y <= cy1) {
                        clarion_2dg_put(c, x, y, colour);
                    }
                }
            }
            if (x0 == x1 && y0 == y1) {
                break;
            }
            e2 = 2 * err;
            if (e2 >= dy) {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y0 += sy;
            }
        }
    }
}

static int clarion_2dg_xy_x(uint32_t w)
{
    return (int16_t)(w >> 16);
}

static int clarion_2dg_xy_y(uint32_t w)
{
    return (int16_t)(w & 0xffff);
}

/* Read n packed XY vertices at addr into vx/vy; false on a bad read. */
static bool clarion_2dg_read_xy(hwaddr addr, unsigned n, int *vx, int *vy)
{
    unsigned i;
    uint32_t w;

    for (i = 0; i < n; i++) {
        if (!clarion_2dg_rd32(addr + i * 4, &w)) {
            return false;
        }
        vx[i] = clarion_2dg_xy_x(w);
        vy[i] = clarion_2dg_xy_y(w);
    }
    return true;
}

static void clarion_2dg_set_lreg(Clarion2DGState *s, uint32_t reg,
                                 uint32_t value, uint32_t mask)
{
    unsigned b;
    uint32_t cur;

    if (reg >= CLARION_2DG_LREGS || (reg & 3)) {
        return;
    }
    cur = s->lreg[reg / 4];
    /* Mask bit set = byte is kept (only byte 2 of 0xc0 etc. is changed). */
    for (b = 0; b < 4; b++) {
        if (!(mask & (1u << b))) {
            cur = (cur & ~(0xffu << (b * 8))) | (value & (0xffu << (b * 8)));
        }
    }
    s->lreg[reg / 4] = cur;
    s->lreg_set[reg / 4] = true;
}

/*
 * Walk and execute one command list. Returns the number of words walked.
 * max_words bounds the walk; the list normally ends with a TRAP word.
 */
static unsigned clarion_2dg_run(Clarion2DGState *s, hwaddr list,
                                unsigned max_words)
{
    ListCtx *c = g_new0(ListCtx, 1);
    hwaddr pc = list;
    hwaddr ret = 0;
    unsigned depth = 0, walked = 0;
    bool stop = false;
    uint16_t t16[CLARION_2DG_MAX_TEX * CLARION_2DG_MAX_TEX];

    c->s = s;
    while (!stop && walked < max_words) {
        uint32_t w, a1 = 0, a2 = 0, a3 = 0, a4 = 0, a5 = 0;
        unsigned op, lo, words = 1;

        if (!clarion_2dg_rd32(pc, &w)) {
            c->stopped_op = 0x100;
            break;
        }
        op = w >> 24;
        lo = w & 0xff;
        clarion_2dg_rd32(pc + 4, &a1);
        clarion_2dg_rd32(pc + 8, &a2);
        clarion_2dg_rd32(pc + 12, &a3);
        clarion_2dg_rd32(pc + 16, &a4);
        clarion_2dg_rd32(pc + 20, &a5);

        switch (op) {
        case M2DG_TRAP:
            if (w) {
                goto unknown;
            }
            stop = true;
            break;
        case M2DG_NOP:
        case M2DG_SYNC:
            break;
        case M2DG_LCOFS:
        case M2DG_MOVE:
            words = 2; /* offset observed as 0 only; not applied */
            c->skipped[op]++;
            break;
        case M2DG_WPR:
            if (w & M2DG_F_WPR_IND) {
                /* a1 = (count - 1) << 16 | reg; a2 = offset of the values. */
                unsigned n = (a1 >> 16) + 1, i;
                uint32_t v;

                words = 3;
                for (i = 0; i < n; i++) {
                    if (clarion_2dg_rd32(pc + (int32_t)a2 + i * 4, &v)) {
                        clarion_2dg_set_lreg(s, (a1 & 0xffff) + i * 4, v, lo);
                    }
                }
            } else if (lo) {
                words = 3;
                if (a1 >> 16) {
                    goto unknown;
                }
                clarion_2dg_set_lreg(s, a1 & 0xffff, a2, lo);
            } else {
                unsigned n = (a1 >> 16) + 1, i;
                uint32_t v;

                words = 2 + n;
                for (i = 0; i < n; i++) {
                    clarion_2dg_rd32(pc + 8 + i * 4, &v);
                    clarion_2dg_set_lreg(s, (a1 & 0xffff) + i * 4, v, 0);
                }
            }
            break;
        case M2DG_DATA:
            words = a1 / 4;
            if (words < 2) {
                goto unknown;
            }
            break;
        case M2DG_CALL:
            words = 2;
            if (depth >= CLARION_2DG_MAX_CALL) {
                goto unknown;
            }
            if (!depth) {
                ret = pc + 8;
            }
            depth++;
            pc = pc + (int32_t)a1;
            walked++;
            c->done[op]++;
            continue;
        case M2DG_RET:
            if (!depth) {
                goto unknown;
            }
            depth--;
            pc = ret;
            walked++;
            c->done[op]++;
            continue;
        case M2DG_CLIPRECT:
            words = 3;
            c->rc_on = true;
            c->rx0 = clarion_2dg_xy_x(a1);
            c->ry0 = clarion_2dg_xy_y(a1);
            c->rx1 = clarion_2dg_xy_x(a2);
            c->ry1 = clarion_2dg_xy_y(a2);
            break;
        case M2DG_CLIPPATH:
            words = 4 + a1;
            if (a1 > CLARION_2DG_MAX_PATH || a1 < 3) {
                goto unknown;
            }
            c->path_n = a1;
            c->path_on = clarion_2dg_read_xy(pc + 16, a1, c->px, c->py);
            break;
        case M2DG_BITBLTC:
        case M2DG_BITBLTA:
        case M2DG_BITBLTK:
        case M2DG_BITBLTR: {
            /*
             * a1 = 0xcc selector / colour key, a2 = colour or source XY,
             * a3 = (left << 16 | right) extents, a4 = (up << 16 | down)
             * extents, a5 = centre XY. Centre-relative form is inferred:
             * it covers exactly the whole canvas in every captured fill.
             */
            int cx = clarion_2dg_xy_x(a5), cy = clarion_2dg_xy_y(a5);
            int l = a3 >> 16, r = a3 & 0xffff, u = a4 >> 16, d = a4 & 0xffff;
            bool drawn = true;

            words = 6;
            if (op == M2DG_BITBLTC) {
                clarion_2dg_fill_rect(c, cx - l, cy - u, cx + r, cy + d,
                                      a2 & 0xffff);
            } else if (op == M2DG_BITBLTK) {
                clarion_2dg_blit(c, clarion_2dg_xy_x(a2), clarion_2dg_xy_y(a2),
                                 l + r + 1, u + d + 1, cx - l, cy - u,
                                 a1 & 0xffff);
            } else if (op == M2DG_BITBLTA && !(w & M2DG_F_STRANS)) {
                clarion_2dg_blit(c, clarion_2dg_xy_x(a2), clarion_2dg_xy_y(a2),
                                 l + r + 1, u + d + 1, cx - l, cy - u, -1);
            } else {
                drawn = false; /* STRANS key and rotated blit unestablished */
            }
            c->done[op] += drawn;
            c->skipped[op] += !drawn;
            c->rc_on = c->path_on = false;
            pc += words * 4;
            walked += words;
            continue;
        }
        case M2DG_POLY4C:
        case M2DG_POLY4T: {
            int vx[4], vy[4];
            unsigned i;
            bool drawn = true;

            words = op == M2DG_POLY4C ? 6 : 8;
            if (op == M2DG_POLY4C) {
                if (!clarion_2dg_read_xy(pc + 8, 4, vx, vy)) {
                    goto unknown;
                }
                clarion_2dg_fill_poly(c, vx, vy, 4, a1 & 0xffff, NULL, 0, 0);
            } else {
                /* a1 = texture offset from this header, a2 = (w << 16 | h). */
                unsigned tw = a2 >> 16, th = a2 & 0xffff;

                if (!tw || !th || tw > CLARION_2DG_MAX_TEX ||
                    th > CLARION_2DG_MAX_TEX ||
                    !clarion_2dg_read_xy(pc + 16, 4, vx, vy) ||
                    address_space_read(&address_space_memory,
                                       pc + (int32_t)a1,
                                       MEMTXATTRS_UNSPECIFIED, t16,
                                       tw * th * 2) != MEMTX_OK) {
                    drawn = false;
                } else {
                    for (i = 0; i < tw * th; i++) {
                        t16[i] = lduw_le_p(&t16[i]);
                    }
                    /* Tiling by absolute canvas coordinates: assumed. */
                    clarion_2dg_fill_poly(c, vx, vy, 4, 0, t16, tw, th);
                }
            }
            c->done[op] += drawn;
            c->skipped[op] += !drawn;
            c->rc_on = c->path_on = false;
            break;
        }
        case M2DG_LINE:
            if (w & M2DG_F_LINE_IND) {
                /* a1 = colour << 16 | n; a2 = width; a3 = vertex offset. */
                unsigned n = a1 & 0xffff;
                int vx[CLARION_2DG_MAX_PATH], vy[CLARION_2DG_MAX_PATH];

                words = 4;
                if (n && n <= CLARION_2DG_MAX_PATH &&
                    clarion_2dg_read_xy(pc + (int32_t)a3, n, vx, vy)) {
                    clarion_2dg_polyline(c, vx, vy, n, a1 >> 16,
                                         MAX(1, (int)a2));
                    c->done[op]++;
                } else {
                    c->skipped[op]++;
                }
            } else {
                unsigned n = a1 & 0xffff;
                int vx[CLARION_2DG_MAX_PATH], vy[CLARION_2DG_MAX_PATH];

                words = 3 + n;
                if (n && n <= CLARION_2DG_MAX_PATH &&
                    clarion_2dg_read_xy(pc + 12, n, vx, vy)) {
                    /* a2 = width (0 draws one pixel; scale unestablished). */
                    clarion_2dg_polyline(c, vx, vy, n, a1 >> 16,
                                         MAX(1, (int)a2));
                    c->done[op]++;
                } else {
                    c->skipped[op]++;
                }
            }
            break;
        case M2DG_LINE_NC:
            words = 3; /* no colour word; colour source not established */
            c->skipped[op]++;
            break;
        default:
            goto unknown;
        }
        if (!stop) {
            c->done[op] += (op != M2DG_POLY4C && op != M2DG_POLY4T &&
                            op != M2DG_LINE && op != M2DG_LINE_NC &&
                            op != M2DG_LCOFS && op != M2DG_MOVE);
        }
        pc += words * 4;
        walked += words;
        continue;

    unknown:
        qemu_log_mask(LOG_UNIMP,
                      "clarion-2dg: unimplemented command %08x at %08"
                      HWADDR_PRIx " (list %08" HWADDR_PRIx
                      "), list abandoned\n",
                      w, pc, list);
        c->stopped_op = op;
        break;
    }
    clarion_2dg_dst_release(c);
    if (qemu_loglevel_mask(LOG_UNIMP)) {
        GString *st = g_string_new(NULL);
        unsigned i;

        for (i = 0; i < 256; i++) {
            if (c->done[i] || c->skipped[i]) {
                g_string_append_printf(st, " %02x:%u/%u", i, c->done[i],
                                       c->skipped[i]);
            }
        }
        qemu_log_mask(LOG_UNIMP,
                      "clarion-2dg: list %08" HWADDR_PRIx " walked=%u "
                      "done/skipped:%s%s\n", list, walked, st->str,
                      c->stopped_op ? " ABANDONED" : "");
        g_string_free(st, true);
    }
    g_free(c);
    return walked;
}

/* Words of the list from its 'RG10' header, or 0 if there is no header. */
static unsigned clarion_2dg_hdr_words(hwaddr list)
{
    uint32_t magic, len;

    if (!clarion_2dg_rd32(list - CLARION_2DG_HDR_BACK, &magic) ||
        magic != CLARION_2DG_HDR_MAGIC ||
        !clarion_2dg_rd32(list - CLARION_2DG_HDR_BACK + CLARION_2DG_HDR_LEN,
                          &len)) {
        return 0;
    }
    return MIN(len / 4, CLARION_2DG_MAX_WORDS);
}

static void clarion_2dg_log_list(Clarion2DGState *s, uint32_t address,
                                 unsigned nwords)
{
    FILE *file;
    g_autofree uint32_t *words = g_new0(uint32_t, nwords);
    char *json;
    GString *line;
    unsigned i;

    if (!s->log_path || !*s->log_path) {
        return;
    }
    for (i = 0; i < nwords; i++) {
        uint32_t word = 0;

        address_space_read(
            &address_space_memory, (hwaddr)address + i * sizeof(word),
            MEMTXATTRS_UNSPECIFIED, (uint8_t *)&word, sizeof(word));
        words[i] = le32_to_cpu(word);
    }
    line = g_string_new(NULL);
    g_string_append_printf(
        line, "{\"time_ns\":%" PRId64 ",\"address\":%u,\"words\":[",
        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), address);
    for (i = 0; i < nwords; i++) {
        g_string_append_printf(line, "%s%u", i ? "," : "", words[i]);
    }
    g_string_append(line, "],\"pre\":[");
    for (i = 0; i < CLARION_2DG_LOG_PRE_WORDS; i++) {
        uint32_t word = 0;

        address_space_read(
            &address_space_memory,
            (hwaddr)address - (CLARION_2DG_LOG_PRE_WORDS - i) * sizeof(word),
            MEMTXATTRS_UNSPECIFIED, (uint8_t *)&word, sizeof(word));
        g_string_append_printf(line, "%s%u", i ? "," : "", le32_to_cpu(word));
    }
    g_string_append(line, "]}\n");
    json = g_string_free(line, FALSE);
    file = fopen(s->log_path, "a");
    if (file) {
        fputs(json, file);
        fclose(file);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "clarion-2dg: cannot write %s: %s\n",
                      s->log_path, strerror(errno));
    }
    g_free(json);
}

static void clarion_2dg_complete(void *opaque)
{
    Clarion2DGState *s = opaque;

    s->regs[CLARION_2DG_STATUS / 4] |= CLARION_2DG_DONE;
    clarion_2dg_update_irq(s);
    qemu_log_mask(LOG_UNIMP, "clarion-2dg: completion status=%08x irq=%u\n",
                  s->regs[CLARION_2DG_STATUS / 4],
                  !!((s->regs[CLARION_2DG_STATUS / 4] & CLARION_2DG_DONE) &&
                     (s->regs[CLARION_2DG_IRQ_ENABLE / 4] & 1)));
}

static uint64_t clarion_2dg_read(void *opaque, hwaddr offset, unsigned size)
{
    Clarion2DGState *s = opaque;
    uint32_t value = 0;

    if (size == 4 && offset < CLARION_2DG_SIZE && !(offset & 3)) {
        value = s->regs[offset / 4];
    }
    qemu_log_mask(LOG_UNIMP,
                  "clarion-2dg: read offset=%03" HWADDR_PRIx
                  " size=%u value=%08x\n",
                  offset, size, value);
    return value;
}

static void clarion_2dg_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    Clarion2DGState *s = opaque;
    uint32_t old;
    unsigned hdr_words;

    if (size != 4 || offset >= CLARION_2DG_SIZE || (offset & 3)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "clarion-2dg: invalid write offset=%03" HWADDR_PRIx
                      " size=%u value=%08" PRIx64 "\n",
                      offset, size, value);
        return;
    }
    qemu_log_mask(LOG_UNIMP,
                  "clarion-2dg: write offset=%03" HWADDR_PRIx
                  " value=%08" PRIx64 "\n",
                  offset, value);
    if (offset == CLARION_2DG_STATUS) {
        return;
    }
    if (offset == CLARION_2DG_ACK) {
        if ((value & CLARION_2DG_DONE) &&
            (s->regs[CLARION_2DG_STATUS / 4] & CLARION_2DG_DONE)) {
            s->regs[CLARION_2DG_STATUS / 4] &= ~CLARION_2DG_DONE;
            clarion_2dg_update_irq(s);
            qemu_log_mask(LOG_UNIMP, "clarion-2dg: acknowledge\n");
        }
        s->regs[offset / 4] = value;
        return;
    }
    old = s->regs[offset / 4];
    s->regs[offset / 4] = value;
    if (offset == CLARION_2DG_IRQ_ENABLE) {
        clarion_2dg_update_irq(s);
    }
    if (offset == 0 && (value & 1)) {
        if (timer_pending(s->timer)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "clarion-2dg: start while previous blit pending\n");
        }
        qemu_log_mask(LOG_UNIMP, "clarion-2dg: start list=%08x first_words=",
                      s->regs[CLARION_2DG_LIST / 4]);
        for (unsigned i = 0; i < 16; i++) {
            uint32_t word = 0;

            address_space_read(&address_space_memory,
                               s->regs[CLARION_2DG_LIST / 4] + i * 4,
                               MEMTXATTRS_UNSPECIFIED, (uint8_t *)&word, 4);
            qemu_log_mask(LOG_UNIMP, "%s%08x", i ? "," : " ",
                          le32_to_cpu(word));
        }
        qemu_log_mask(LOG_UNIMP, "\n");
        hdr_words = clarion_2dg_hdr_words(s->regs[CLARION_2DG_LIST / 4]);
        clarion_2dg_log_list(s, s->regs[CLARION_2DG_LIST / 4],
                             hdr_words ? hdr_words :
                                         CLARION_2DG_LOG_FALLBACK_WORDS);
        if (s->exec) {
            clarion_2dg_run(s, s->regs[CLARION_2DG_LIST / 4],
                            hdr_words ? hdr_words :
                                        CLARION_2DG_FALLBACK_WORDS);
        }
        s->regs[CLARION_2DG_STATUS / 4] &= ~CLARION_2DG_DONE;
        clarion_2dg_update_irq(s);
        timer_mod(s->timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + CLARION_2DG_DELAY_NS);
    }
    if (old != s->regs[offset / 4]) {
        clarion_2dg_update_irq(s);
    }
}

static const MemoryRegionOps clarion_2dg_ops = {
    .read = clarion_2dg_read,
    .write = clarion_2dg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4, .unaligned = false },
    .impl = { .min_access_size = 4, .max_access_size = 4, .unaligned = false },
};

static void clarion_2dg_reset(DeviceState *dev)
{
    Clarion2DGState *s = CLARION_2DG(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->lreg, 0, sizeof(s->lreg));
    memset(s->lreg_set, 0, sizeof(s->lreg_set));
    timer_del(s->timer);
    qemu_set_irq(s->irq, 0);
}

static void clarion_2dg_realize(DeviceState *dev, Error **errp)
{
    Clarion2DGState *s = CLARION_2DG(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    (void)errp;
    memory_region_init_io(&s->iomem, OBJECT(s), &clarion_2dg_ops, s,
                          TYPE_CLARION_2DG, CLARION_2DG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, clarion_2dg_complete, s);
}

static const VMStateDescription vmstate_clarion_2dg = {
    .name = TYPE_CLARION_2DG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields =
        (const VMStateField[]){
            VMSTATE_UINT32_ARRAY(regs, Clarion2DGState, CLARION_2DG_REGS),
            VMSTATE_TIMER_PTR(timer, Clarion2DGState), VMSTATE_END_OF_LIST() }
};

static const Property clarion_2dg_properties[] = {
    DEFINE_PROP_STRING("log", Clarion2DGState, log_path),
    DEFINE_PROP_BOOL("exec", Clarion2DGState, exec, true),
};

static void clarion_2dg_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    (void)data;
    dc->realize = clarion_2dg_realize;
    device_class_set_legacy_reset(dc, clarion_2dg_reset);
    dc->vmsd = &vmstate_clarion_2dg;
    device_class_set_props(dc, clarion_2dg_properties);
}

static const TypeInfo clarion_2dg_info = {
    .name = TYPE_CLARION_2DG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Clarion2DGState),
    .class_init = clarion_2dg_class_init,
};

static void clarion_2dg_register_types(void)
{
    type_register_static(&clarion_2dg_info);
}

type_init(clarion_2dg_register_types)
