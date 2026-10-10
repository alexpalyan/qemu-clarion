/*
 * qy8gl: trace (and later forward) the QY8 head unit's OpenGL ES / EGL calls.
 *
 * The guest's libGLESv2.dll, libEGL.dll and libIMGEGL.dll sit at fixed ROM
 * addresses, so every entry point is known up front. Load with
 *   -plugin build/contrib/plugins/libqy8gl.dylib,syms=FILE,log=FILE
 * where FILE lines are "<ordinal> <hex address> <name>".
 *
 * Return values are caught at the caller's return address: a call's first
 * entry happens before the code after its `bl` is translated, so adding that
 * address to the watch set is enough to instrument it.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <glib.h>

#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

typedef struct {
    uint32_t addr;
    char *name;
    uint64_t calls;
} Sym;

typedef struct {
    Sym *sym;
    uint32_t sp, ttbr;
    uint32_t a[4];
} Pending;

static GHashTable *by_addr;         /* entry addr -> Sym* */
static GHashTable *ret_sites;       /* return addr -> GPtrArray of Pending* */
static GPtrArray *syms;
static FILE *tracef;
static uint64_t log_limit = 500000;
static uint64_t logged;
static GMutex lock;

static struct qemu_plugin_register *reg_r[4], *reg_sp, *reg_lr, *reg_ttbr;
static bool have_ttbr;

static uint32_t rd(struct qemu_plugin_register *r)
{
    g_autoptr(GByteArray) b = g_byte_array_new();
    uint32_t v = 0;

    /* handles are register numbers, so r0's handle is NULL */
    if (qemu_plugin_read_register(r, b) && b->len >= 4) {
        memcpy(&v, b->data, 4);
    }
    return v;
}

static bool rmem(uint32_t va, void *dst, size_t len)
{
    g_autoptr(GByteArray) b = g_byte_array_new();

    if (!len || !qemu_plugin_read_memory_vaddr(va, b, len)) {
        return false;
    }
    memcpy(dst, b->data, len);
    return true;
}

static uint32_t rd32(uint32_t va)
{
    uint32_t v = 0;
    rmem(va, &v, 4);
    return v;
}

static void rstr(uint32_t va, char *out, size_t max)
{
    size_t i;

    for (i = 0; i + 1 < max; i++) {
        char c;
        if (!rmem(va + i, &c, 1) || !c) {
            break;
        }
        out[i] = c;
    }
    out[i] = 0;
}

static float f32(uint32_t v)
{
    float f;
    memcpy(&f, &v, 4);
    return f;
}

static Sym *add_sym(uint32_t addr, const char *name)
{
    Sym *s = g_hash_table_lookup(by_addr, GUINT_TO_POINTER(addr));

    if (!s) {
        s = g_new0(Sym, 1);
        s->addr = addr;
        s->name = g_strdup(name);
        g_ptr_array_add(syms, s);
        g_hash_table_insert(by_addr, GUINT_TO_POINTER(addr), s);
    }
    return s;
}

static void dump_attribs(const char *tag, uint32_t lst, int max)
{
    uint32_t kv[32];

    if (!lst || max > 32 || !rmem(lst, kv, max * 4)) {
        return;
    }
    fprintf(tracef, "    %s", tag);
    for (int i = 0; i < max && kv[i] != 0x3038; i++) {
        fprintf(tracef, " %x", kv[i]);
    }
    fprintf(tracef, "\n");
}

static void rwstr(uint32_t va, char *out, size_t max)
{
    size_t i;

    for (i = 0; i + 1 < max; i++) {
        uint16_t c = 0;
        if (!rmem(va + i * 2, &c, 2) || !c) {
            break;
        }
        out[i] = c < 0x80 ? c : '?';
    }
    out[i] = 0;
}

/* format a CE debug message: wide format, args from @next() */
static void wprintf_guest(uint32_t fmtva, uint32_t (*next)(void *), void *ctx,
                          GString *out)
{
    char fmt[512];

    rwstr(fmtva, fmt, sizeof(fmt));
    for (const char *f = fmt; *f; f++) {
        char spec[16], buf[256];
        int n = 0;

        if (*f != '%') {
            g_string_append_c(out, *f);
            continue;
        }
        spec[n++] = *f++;
        while (*f && strchr("-+ #0123456789.lhwI", *f) && n < 12) {
            if (*f != 'l' && *f != 'h' && *f != 'w' && *f != 'I') {
                spec[n++] = *f;
            }
            f++;
        }
        if (!*f) {
            break;
        }
        switch (*f) {
        case '%':
            g_string_append_c(out, '%');
            break;
        case 's':
            rwstr(next(ctx), buf, sizeof(buf));
            g_string_append(out, buf);
            break;
        case 'S':
            rstr(next(ctx), buf, sizeof(buf));
            g_string_append(out, buf);
            break;
        case 'c': case 'C':
            g_string_append_c(out, (char)next(ctx));
            break;
        case 'd': case 'i': case 'u': case 'x': case 'X': case 'o': case 'p':
            spec[n++] = *f == 'p' ? 'x' : *f;
            spec[n] = 0;
            snprintf(buf, sizeof(buf), spec, next(ctx));
            g_string_append(out, buf);
            break;
        default:
            g_string_append_c(out, *f);
        }
    }
}

typedef struct {
    const uint32_t *reg;
    int nreg, idx;
    uint32_t mem;
} ArgIt;

static uint32_t arg_next(void *p)
{
    ArgIt *it = p;
    uint32_t v;

    if (it->idx < it->nreg) {
        return it->reg[it->idx++];
    }
    v = rd32(it->mem);
    it->mem += 4;
    return v;
}

static void debug_msg(const char *n, const uint32_t *a, uint32_t sp, uint32_t lr)
{
    g_autoptr(GString) out = g_string_new(NULL);
    char buf[512];

    if (!strcmp(n, "OutputDebugStringW")) {
        rwstr(a[0], buf, sizeof(buf));
        g_string_append(out, buf);
    } else if (!strcmp(n, "NKDbgPrintfW")) {
        ArgIt it = { a + 1, 3, 0, sp };
        wprintf_guest(a[0], arg_next, &it, out);
    } else if (!strcmp(n, "NKvDbgPrintfW")) {
        ArgIt it = { NULL, 0, 0, a[1] };
        wprintf_guest(a[0], arg_next, &it, out);
    } else {
        return;
    }
    while (out->len && (out->str[out->len - 1] == '\n' || out->str[out->len - 1] == '\r')) {
        g_string_truncate(out, out->len - 1);
    }
    fprintf(tracef, "DBG %08x %s\n", lr, out->str);
}

/* extra detail for the calls whose arguments point at data */
static void detail(Sym *s, const uint32_t *a, uint32_t sp)
{
    char str[128];
    const char *n = s->name;

    if (!strcmp(n, "eglGetProcAddress")) {
        rstr(a[0], str, sizeof(str));
        fprintf(tracef, "    name \"%s\"\n", str);
    } else if (!strcmp(n, "glGetUniformLocation") ||
               !strcmp(n, "glGetAttribLocation")) {
        rstr(a[1], str, sizeof(str));
        fprintf(tracef, "    name \"%s\"\n", str);
    } else if (!strcmp(n, "glUniformMatrix4fv")) {
        uint32_t m[16];
        if (rmem(a[3], m, sizeof(m))) {
            fprintf(tracef, "    m");
            for (int i = 0; i < 16; i++) {
                fprintf(tracef, " %g", f32(m[i]));
            }
            fprintf(tracef, "\n");
        }
    } else if (!strcmp(n, "glUniform1f")) {
        fprintf(tracef, "    v %g\n", f32(a[1]));
    } else if (!strcmp(n, "glUniform4f")) {
        fprintf(tracef, "    v %g %g %g %g\n", f32(a[1]), f32(a[2]), f32(a[3]),
                f32(rd32(sp)));
    } else if (!strcmp(n, "glVertexAttribPointer")) {
        /* index, size, type, normalized | stride, ptr */
        uint32_t stride = rd32(sp), ptr = rd32(sp + 4), v[12];
        uint32_t cnt = a[1] * 4;
        if (cnt <= 12 && !stride && rmem(ptr, v, cnt * 4)) {
            fprintf(tracef, "    attr%u @%08x:", a[0], ptr);
            for (uint32_t i = 0; i < cnt; i++) {
                fprintf(tracef, " %g", f32(v[i]));
            }
            fprintf(tracef, "\n");
        }
    } else if (!strcmp(n, "eglCreateImageKHR")) {
        /* dpy, ctx, target, buffer | attribs */
        uint32_t buf[16];
        dump_attribs("attribs", rd32(sp), 16);
        if (a[3] && rmem(a[3], buf, sizeof(buf))) {
            fprintf(tracef, "    buffer");
            for (int i = 0; i < 16; i++) {
                fprintf(tracef, " %08x", buf[i]);
            }
            fprintf(tracef, "\n");
        }
    } else if (!strcmp(n, "CreateDIBSection")) {
        /* hdc, BITMAPINFO*, usage, void **bits | section, offset */
        uint32_t h[13];
        if (rmem(a[1], h, sizeof(h))) {
            fprintf(tracef, "    bmi w %d h %d bpp %u comp %u masks %08x %08x %08x\n",
                    (int32_t)h[1], (int32_t)h[2], h[3] >> 16, h[4],
                    h[10], h[11], h[12]);
        }
    } else if (!strcmp(n, "DispatchMessageW")) {
        uint32_t m[4];
        if (rmem(a[0], m, sizeof(m))) {
            fprintf(tracef, "    msg hwnd=%08x msg=%04x wp=%08x lp=%08x\n",
                    m[0], m[1], m[2], m[3]);
        }
    } else if (!strcmp(n, "eglChooseConfig")) {
        dump_attribs("attribs", a[1], 24);
    } else if (!strcmp(n, "eglCreatePbufferSurface")) {
        dump_attribs("attribs", a[2], 24);
    } else if (!strcmp(n, "eglCreateContext") ||
               !strcmp(n, "eglCreateWindowSurface")) {
        dump_attribs("attribs", a[3], 24);
    }
}

/* ---- software renderer: shadows the AUI's GL state and draws its quads ---- */

typedef struct {
    uint32_t bits;
    int w, h, bpp, comp;
    uint32_t mask[4];
} Bmp;

typedef struct {
    int w, h;
    uint8_t *px;                    /* RGBA, row 0 = GL y 0 */
} Surf;

typedef struct {
    uint32_t hbm;                   /* source bitmap, or 0 */
    uint32_t surf;                  /* pbuffer bound with eglBindTexImage */
} TexSrc;

typedef struct {
    uint32_t ttbr;
    GHashTable *bmps;               /* hbm -> Bmp* */
    GHashTable *images;             /* EGLImage -> hbm */
    GHashTable *texs;               /* texture name -> TexSrc* */
    GHashTable *surfs;              /* EGLSurface -> Surf* */
    GHashTable *shader_bin;         /* shader -> binary address */
    GHashTable *prog_frag;          /* program -> fragment binary address */
    GHashTable *uni_name;           /* (prog << 8 | loc) -> name */
    GHashTable *attr_name;          /* (prog << 8 | loc) -> name */
    GHashTable *uni_val;            /* (prog << 8 | loc) -> float[16] */
    uint32_t prog, tex, draw;
    uint32_t attr_ptr[8], attr_size[8];
    bool blend;
    uint32_t bf[4];
    int vp[4];
    float clear[4];
    int frame;
    uint32_t ctx;
    GHashTable *ctx_vp;             /* EGLContext -> int[4] */
} Proc;

static GHashTable *procs;           /* ttbr -> Proc* */
static char *outdir;

/*
 * Fragment shader binaries inside auirtdll.dll, told apart by address. The
 * defaults are G218ENNI's; a syms file can override them with "@fs_*" lines.
 */
static uint32_t FS_TEXCOLOR = 0x412a3a18u;  /* u_TextureEnabled ? tex : u_Color */
static uint32_t FS_KEYED_A  = 0x412a3f40u;  /* tex with optional colour key */
static uint32_t FS_KEYED_B  = 0x412a4648u;
static uint32_t FS_TINT     = 0x412a4ce0u;  /* u_Color tinted by tex alpha */

static Proc *proc_get(uint32_t ttbr)
{
    Proc *p = g_hash_table_lookup(procs, GUINT_TO_POINTER(ttbr));

    if (!p) {
        p = g_new0(Proc, 1);
        p->ttbr = ttbr;
        p->bmps = g_hash_table_new(NULL, NULL);
        p->images = g_hash_table_new(NULL, NULL);
        p->texs = g_hash_table_new(NULL, NULL);
        p->surfs = g_hash_table_new(NULL, NULL);
        p->shader_bin = g_hash_table_new(NULL, NULL);
        p->prog_frag = g_hash_table_new(NULL, NULL);
        p->uni_name = g_hash_table_new(NULL, NULL);
        p->attr_name = g_hash_table_new(NULL, NULL);
        p->uni_val = g_hash_table_new(NULL, NULL);
        p->vp[2] = 800;
        p->vp[3] = 480;
        p->ctx_vp = g_hash_table_new(NULL, NULL);
        g_hash_table_insert(procs, GUINT_TO_POINTER(ttbr), p);
    }
    return p;
}

static Surf *surf_get(Proc *p, uint32_t h, int w, int ht)
{
    Surf *s = g_hash_table_lookup(p->surfs, GUINT_TO_POINTER(h));

    if (!s && w > 0 && ht > 0) {
        s = g_new0(Surf, 1);
        s->w = w;
        s->h = ht;
        s->px = g_malloc0((size_t)w * ht * 4);
        g_hash_table_insert(p->surfs, GUINT_TO_POINTER(h), s);
    }
    return s;
}

static float *uval(Proc *p, uint32_t loc)
{
    gpointer k = GUINT_TO_POINTER(p->prog << 8 | (loc & 0xff));
    float *v = g_hash_table_lookup(p->uni_val, k);

    if (!v) {
        v = g_new0(float, 16);
        g_hash_table_insert(p->uni_val, k, v);
    }
    return v;
}

/* value of the named uniform in the current program, or NULL */
static float *uni(Proc *p, const char *name)
{
    for (uint32_t loc = 0; loc < 32; loc++) {
        const char *n = g_hash_table_lookup(p->uni_name,
                                            GUINT_TO_POINTER(p->prog << 8 | loc));
        if (n && !strcmp(n, name)) {
            return uval(p, loc);
        }
    }
    return NULL;
}

static int attr_index(Proc *p, const char *name)
{
    for (uint32_t loc = 0; loc < 8; loc++) {
        const char *n = g_hash_table_lookup(p->attr_name,
                                            GUINT_TO_POINTER(p->prog << 8 | loc));
        if (n && !strcmp(n, name)) {
            return loc;
        }
    }
    return -1;
}

static uint8_t chan(uint32_t v, uint32_t mask)
{
    int sh = 0, bits = 0;

    if (!mask) {
        return 255;
    }
    while (!(mask >> sh & 1)) {
        sh++;
    }
    while (mask >> (sh + bits) & 1) {
        bits++;
    }
    return (((v & mask) >> sh) * 255) / ((1u << bits) - 1);
}

/* decode a guest DIB section into RGBA, row 0 = first row in memory */
static uint8_t *bmp_rgba(Bmp *b)
{
    int bpp = b->bpp / 8, stride = (b->w * bpp + 3) & ~3;
    size_t len = (size_t)stride * b->h;
    uint8_t *raw = g_malloc(len), *out = g_malloc((size_t)b->w * b->h * 4);

    if (!rmem(b->bits, raw, len)) {
        memset(raw, 0, len);
    }
    for (int y = 0; y < b->h; y++) {
        for (int x = 0; x < b->w; x++) {
            uint8_t *o = out + ((size_t)y * b->w + x) * 4;
            const uint8_t *s = raw + (size_t)y * stride + x * bpp;
            if (bpp == 4) {
                o[0] = s[2]; o[1] = s[1]; o[2] = s[0]; o[3] = s[3];
            } else {
                uint32_t v = s[0] | s[1] << 8;
                o[0] = chan(v, b->mask[0]);
                o[1] = chan(v, b->mask[1]);
                o[2] = chan(v, b->mask[2]);
                o[3] = chan(v, b->mask[3]);
            }
        }
    }
    g_free(raw);
    return out;
}

static float bfac(uint32_t f, const float *s, const float *d, int c)
{
    switch (f) {
    case 0x0000: return 0;
    case 0x0001: return 1;
    case 0x0300: return s[c];
    case 0x0301: return 1 - s[c];
    case 0x0302: return s[3];
    case 0x0303: return 1 - s[3];
    case 0x0304: return d[3];
    case 0x0305: return 1 - d[3];
    case 0x0306: return d[c];
    case 0x0307: return 1 - d[c];
    default:     return 1;
    }
}

static void draw_quad(Proc *p, int count)
{
    Surf *dst = g_hash_table_lookup(p->surfs, GUINT_TO_POINTER(p->draw));
    uint32_t frag = GPOINTER_TO_UINT(g_hash_table_lookup(p->prog_frag,
                                                         GUINT_TO_POINTER(p->prog)));
    int ipos = attr_index(p, "a_Position"), itc = attr_index(p, "a_TexCoord");
    float *mv = uni(p, "u_Modelview"), *col = uni(p, "u_Color");
    float *ten = uni(p, "u_TextureEnabled"), *tr = uni(p, "u_Transparency");
    float *key = uni(p, "u_TransparentColorEnabled");
    float pos[4][4], tc[4][2] = { { 0 } }, sx[4], sy[4];
    TexSrc *ts = g_hash_table_lookup(p->texs, GUINT_TO_POINTER(p->tex));
    uint8_t *tex = NULL;
    int tw = 0, th = 0;
    bool own = false;

    if (!dst || ipos < 0 || count != 4) {
        return;
    }
    if (tracef) {
        fprintf(tracef, "    draw prog %x frag %x tex %x hbm %x surf %x vp %d,%d,%d,%d\n",
                p->prog, frag, p->tex, ts ? ts->hbm : 0, ts ? ts->surf : 0,
                p->vp[0], p->vp[1], p->vp[2], p->vp[3]);
    }
    for (int v = 0; v < 4; v++) {
        uint32_t n = p->attr_size[ipos] ? p->attr_size[ipos] : 3;
        uint32_t raw[4] = { 0, 0, 0, 0x3f800000 };
        float in[4];
        rmem(p->attr_ptr[ipos] + v * n * 4, raw, n * 4);
        memcpy(in, raw, 16);
        if (n < 4) {
            in[3] = 1;
        }
        for (int r = 0; r < 4; r++) {
            pos[v][r] = mv ? mv[r] * in[0] + mv[4 + r] * in[1] +
                             mv[8 + r] * in[2] + mv[12 + r] * in[3] : in[r];
        }
        if (itc >= 0) {
            uint32_t t[2];
            rmem(p->attr_ptr[itc] + v * 8, t, 8);
            memcpy(tc[v], t, 8);
        }
        float w = pos[v][3] ? pos[v][3] : 1;
        sx[v] = p->vp[0] + (pos[v][0] / w + 1) * p->vp[2] / 2;
        sy[v] = p->vp[1] + (pos[v][1] / w + 1) * p->vp[3] / 2;
    }

    if (ts && ts->hbm) {
        Bmp *b = g_hash_table_lookup(p->bmps, GUINT_TO_POINTER(ts->hbm));
        if (b) {
            tex = bmp_rgba(b);
            tw = b->w;
            th = b->h;
            own = true;
            if (outdir && b->w * b->h >= 800 * 400) {
                g_autofree char *path = g_strdup_printf("%s/bmp-%08x-%dx%d.rgba",
                                                        outdir, ts->hbm, tw, th);
                FILE *f = fopen(path, "wb");
                if (f) {
                    fwrite(tex, 4, (size_t)tw * th, f);
                    fclose(f);
                }
            }
        }
    } else if (ts && ts->surf) {
        Surf *s = g_hash_table_lookup(p->surfs, GUINT_TO_POINTER(ts->surf));
        if (s) {
            tex = s->px;
            tw = s->w;
            th = s->h;
        }
    }

    static const int tri[2][3] = { { 0, 1, 2 }, { 1, 3, 2 } };
    for (int t = 0; t < 2; t++) {
        int i0 = tri[t][0], i1 = tri[t][1], i2 = tri[t][2];
        float area = (sx[i1] - sx[i0]) * (sy[i2] - sy[i0]) -
                     (sx[i2] - sx[i0]) * (sy[i1] - sy[i0]);
        if (fabsf(area) < 1e-6f) {
            continue;
        }
        int x0 = MAX(0, (int)floorf(MIN(sx[i0], MIN(sx[i1], sx[i2]))));
        int x1 = MIN(dst->w - 1, (int)ceilf(MAX(sx[i0], MAX(sx[i1], sx[i2]))));
        int y0 = MAX(0, (int)floorf(MIN(sy[i0], MIN(sy[i1], sy[i2]))));
        int y1 = MIN(dst->h - 1, (int)ceilf(MAX(sy[i0], MAX(sy[i1], sy[i2]))));
        for (int y = y0; y <= y1; y++) {
            for (int x = x0; x <= x1; x++) {
                float px = x + 0.5f, py = y + 0.5f;
                float w0 = ((sx[i1] - px) * (sy[i2] - py) -
                            (sx[i2] - px) * (sy[i1] - py)) / area;
                float w1 = ((sx[i2] - px) * (sy[i0] - py) -
                            (sx[i0] - px) * (sy[i2] - py)) / area;
                float w2 = 1 - w0 - w1;
                if (w0 < 0 || w1 < 0 || w2 < 0) {
                    continue;
                }
                float u = w0 * tc[i0][0] + w1 * tc[i1][0] + w2 * tc[i2][0];
                float vv = w0 * tc[i0][1] + w1 * tc[i1][1] + w2 * tc[i2][1];
                float s[4] = { 1, 1, 1, 1 }, c[4];
                if (tex) {
                    int tx = CLAMP((int)floorf(u * tw), 0, tw - 1);
                    int ty = CLAMP((int)floorf(vv * th), 0, th - 1);
                    const uint8_t *t8 = tex + ((size_t)ty * tw + tx) * 4;
                    for (int k = 0; k < 4; k++) {
                        s[k] = t8[k] / 255.0f;
                    }
                }
                if (frag == FS_TEXCOLOR) {
                    if (!(ten && ten[0] != 0) && col) {
                        memcpy(s, col, sizeof(s));
                    }
                } else if (frag == FS_KEYED_A || frag == FS_KEYED_B) {
                    /* opaque except the key colour; texture alpha is ignored */
                    if (key && key[0] != 0 && col &&
                        fabsf(s[0] - col[0]) < 0.02f &&
                        fabsf(s[1] - col[1]) < 0.02f &&
                        fabsf(s[2] - col[2]) < 0.02f) {
                        continue;
                    }
                    s[3] = 1;
                } else if (frag == FS_TINT && col) {
                    for (int k = 0; k < 4; k++) {
                        s[k] *= col[k];
                    }
                }
                if (tr) {
                    s[3] *= tr[0];
                }
                uint8_t *d8 = dst->px + ((size_t)y * dst->w + x) * 4;
                float d[4] = { d8[0] / 255.0f, d8[1] / 255.0f,
                               d8[2] / 255.0f, d8[3] / 255.0f };
                for (int k = 0; k < 4; k++) {
                    if (p->blend) {
                        int sf = k < 3 ? 0 : 2, df = k < 3 ? 1 : 3;
                        c[k] = s[k] * bfac(p->bf[sf], s, d, k) +
                               d[k] * bfac(p->bf[df], s, d, k);
                    } else {
                        c[k] = s[k];
                    }
                    d8[k] = (uint8_t)(CLAMP(c[k], 0.0f, 1.0f) * 255 + 0.5f);
                }
            }
        }
    }
    if (own) {
        g_free(tex);
    }
}

/* hand the frame to the display model through the QY8_GL_FRAME file */
static void publish(Proc *p, Surf *s)
{
    static uint8_t *map;
    static size_t len;
    static uint32_t frame;
    const char *path = getenv("QY8_GL_FRAME");
    size_t need = 16 + (size_t)s->w * s->h * 4;
    uint32_t hdr[4];

    /* only the AUI: the map renderer uses shaders this file doesn't model */
    if (!path || !g_hash_table_size(p->prog_frag)) {
        return;
    }
    if (!map || len != need) {
        int fd = open(path, O_RDWR | O_CREAT, 0644);
        if (fd < 0 || ftruncate(fd, need) < 0) {
            if (fd >= 0) {
                close(fd);
            }
            return;
        }
        if (map) {
            munmap(map, len);
        }
        map = mmap(NULL, need, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        close(fd);
        if (map == MAP_FAILED) {
            map = NULL;
            return;
        }
        len = need;
    }
    memcpy(map + 16, s->px, need - 16);
    hdr[0] = 0x4c475951;            /* "QYGL" */
    hdr[1] = ++frame;
    hdr[2] = s->w;
    hdr[3] = s->h;
    memcpy(map, hdr, sizeof(hdr));
}

static void dump_surf(Proc *p, uint32_t h)
{
    Surf *s = g_hash_table_lookup(p->surfs, GUINT_TO_POINTER(h));
    g_autofree char *path = NULL;
    FILE *f;

    if (!s) {
        return;
    }
    publish(p, s);
    if (!outdir) {
        return;
    }
    path = g_strdup_printf("%s/%08x-%08x-%04d-%dx%d.rgba", outdir, p->ttbr, h,
                           p->frame++, s->w, s->h);
    f = fopen(path, "wb");
    if (f) {
        fwrite(s->px, 4, (size_t)s->w * s->h, f);
        fclose(f);
    }
}

static int attrib_lookup(uint32_t lst, uint32_t key, int dflt)
{
    for (int i = 0; lst && i < 32; i += 2) {
        uint32_t k = rd32(lst + i * 4);
        if (k == 0x3038) {
            break;
        }
        if (k == key) {
            return rd32(lst + i * 4 + 4);
        }
    }
    return dflt;
}

static void gl_call(uint32_t ttbr, const char *n, const uint32_t *a, uint32_t sp)
{
    Proc *p = proc_get(ttbr);

    if (!strcmp(n, "glShaderBinary")) {
        g_hash_table_insert(p->shader_bin, GUINT_TO_POINTER(rd32(a[1])),
                            GUINT_TO_POINTER(a[3]));
    } else if (!strcmp(n, "glAttachShader")) {
        uint32_t bin = GPOINTER_TO_UINT(g_hash_table_lookup(p->shader_bin,
                                                            GUINT_TO_POINTER(a[1])));
        if (bin == FS_TEXCOLOR || bin == FS_KEYED_A || bin == FS_KEYED_B ||
            bin == FS_TINT) {
            g_hash_table_insert(p->prog_frag, GUINT_TO_POINTER(a[0]),
                                GUINT_TO_POINTER(bin));
        }
    } else if (!strcmp(n, "glUseProgram")) {
        p->prog = a[0];
    } else if (!strcmp(n, "glUniform1f")) {
        uval(p, a[0])[0] = f32(a[1]);
    } else if (!strcmp(n, "glUniform1i")) {
        uval(p, a[0])[0] = (float)(int32_t)a[1];
    } else if (!strcmp(n, "glUniform4f")) {
        float *v = uval(p, a[0]);
        v[0] = f32(a[1]); v[1] = f32(a[2]); v[2] = f32(a[3]); v[3] = f32(rd32(sp));
    } else if (!strcmp(n, "glUniformMatrix4fv")) {
        uint32_t m[16];
        if (rmem(a[3], m, sizeof(m))) {
            memcpy(uval(p, a[0]), m, sizeof(m));
        }
    } else if (!strcmp(n, "glVertexAttribPointer")) {
        if (a[0] < 8) {
            p->attr_size[a[0]] = a[1];
            p->attr_ptr[a[0]] = rd32(sp + 4);
        }
    } else if (!strcmp(n, "glBindTexture")) {
        p->tex = a[1];
    } else if (!strcmp(n, "glEGLImageTargetTexture2DOES")) {
        TexSrc *t = g_new0(TexSrc, 1);
        t->hbm = GPOINTER_TO_UINT(g_hash_table_lookup(p->images,
                                                      GUINT_TO_POINTER(a[1])));
        g_hash_table_replace(p->texs, GUINT_TO_POINTER(p->tex), t);
    } else if (!strcmp(n, "eglBindTexImage")) {
        TexSrc *t = g_new0(TexSrc, 1);
        t->surf = a[1];
        g_hash_table_replace(p->texs, GUINT_TO_POINTER(p->tex), t);
    } else if (!strcmp(n, "glEnable") && a[0] == 0x0be2) {
        p->blend = true;
    } else if (!strcmp(n, "glDisable") && a[0] == 0x0be2) {
        p->blend = false;
    } else if (!strcmp(n, "glBlendFuncSeparate")) {
        memcpy(p->bf, a, sizeof(p->bf));
    } else if (!strcmp(n, "glBlendFunc")) {
        p->bf[0] = p->bf[2] = a[0];
        p->bf[1] = p->bf[3] = a[1];
    } else if (!strcmp(n, "glViewport")) {
        int *v = g_hash_table_lookup(p->ctx_vp, GUINT_TO_POINTER(p->ctx));
        if (!v) {
            v = g_new0(int, 4);
            g_hash_table_insert(p->ctx_vp, GUINT_TO_POINTER(p->ctx), v);
        }
        for (int i = 0; i < 4; i++) {
            p->vp[i] = v[i] = a[i];
        }
    } else if (!strcmp(n, "glClearColor")) {
        p->clear[0] = f32(a[0]); p->clear[1] = f32(a[1]);
        p->clear[2] = f32(a[2]); p->clear[3] = f32(a[3]);
    } else if (!strcmp(n, "glClear") && (a[0] & 0x4000)) {
        Surf *s = g_hash_table_lookup(p->surfs, GUINT_TO_POINTER(p->draw));
        for (size_t i = 0; s && i < (size_t)s->w * s->h; i++) {
            for (int k = 0; k < 4; k++) {
                s->px[i * 4 + k] = (uint8_t)(p->clear[k] * 255 + 0.5f);
            }
        }
    } else if (!strcmp(n, "glDrawArrays")) {
        draw_quad(p, a[2]);
    } else if (!strcmp(n, "eglMakeCurrent")) {
        int *v = g_hash_table_lookup(p->ctx_vp, GUINT_TO_POINTER(a[3]));
        Surf *sf = g_hash_table_lookup(p->surfs, GUINT_TO_POINTER(a[1]));
        p->draw = a[1];
        p->ctx = a[3];
        if (v) {
            memcpy(p->vp, v, sizeof(p->vp));
        } else if (sf) {
            /* a fresh context's viewport is its first surface */
            p->vp[0] = p->vp[1] = 0;
            p->vp[2] = sf->w;
            p->vp[3] = sf->h;
        }
    } else if (!strcmp(n, "eglSwapBuffers")) {
        dump_surf(p, a[1]);
    }
}

static void gl_ret(uint32_t ttbr, const char *n, const uint32_t *a, uint32_t r0,
                   uint32_t sp_args)
{
    Proc *p = proc_get(ttbr);

    if (!strcmp(n, "CreateDIBSection") && r0) {
        uint32_t h[14];
        if (rmem(a[1], h, sizeof(h))) {
            Bmp *b = g_new0(Bmp, 1);
            b->bits = rd32(a[3]);
            b->w = (int32_t)h[1];
            b->h = abs((int32_t)h[2]);
            b->bpp = h[3] >> 16;
            b->comp = h[4];
            if (b->comp == 3 || b->comp == 6) {
                b->mask[0] = h[10]; b->mask[1] = h[11]; b->mask[2] = h[12];
                b->mask[3] = b->comp == 6 ? h[13] : 0;
            }
            g_hash_table_replace(p->bmps, GUINT_TO_POINTER(r0), b);
            if (tracef) {
                fprintf(tracef, "    bmp %x %dx%d bpp %d comp %d masks %x %x %x %x\n",
                        r0, b->w, b->h, b->bpp, b->comp, b->mask[0], b->mask[1],
                        b->mask[2], b->mask[3]);
            }
        }
    } else if (!strcmp(n, "eglCreateImageKHR") && r0) {
        g_hash_table_replace(p->images, GUINT_TO_POINTER(r0),
                             GUINT_TO_POINTER(a[3]));
    } else if (!strcmp(n, "glGetUniformLocation") && r0 != 0xffffffff) {
        char nm[64];
        rstr(a[1], nm, sizeof(nm));
        g_hash_table_replace(p->uni_name, GUINT_TO_POINTER(a[0] << 8 | (r0 & 0xff)),
                             g_strdup(nm));
    } else if (!strcmp(n, "glGetAttribLocation") && r0 != 0xffffffff) {
        char nm[64];
        rstr(a[1], nm, sizeof(nm));
        g_hash_table_replace(p->attr_name, GUINT_TO_POINTER(a[0] << 8 | (r0 & 0xff)),
                             g_strdup(nm));
    } else if (!strcmp(n, "eglCreateWindowSurface") && r0) {
        surf_get(p, r0, 800, 480);
    } else if (!strcmp(n, "eglCreatePbufferSurface") && r0) {
        surf_get(p, r0, attrib_lookup(a[2], 0x3057, 1), attrib_lookup(a[2], 0x3056, 1));
    }
}

static void on_ret(unsigned int vcpu, void *udata)
{
    uint32_t pc = GPOINTER_TO_UINT(udata);
    uint32_t sp = rd(reg_sp), r0 = rd(reg_r[0]);
    uint32_t ttbr = have_ttbr ? rd(reg_ttbr) : 0;
    GPtrArray *list;

    g_mutex_lock(&lock);
    list = g_hash_table_lookup(ret_sites, GUINT_TO_POINTER(pc));
    for (guint i = 0; list && i < list->len; i++) {
        Pending *p = g_ptr_array_index(list, i);
        if (p->sp != sp || p->ttbr != ttbr) {
            continue;
        }
        if (tracef && logged < log_limit) {
            fprintf(tracef, "  -> %s = %08x\n", p->sym->name, r0);
            if (!strcmp(p->sym->name, "glGenTextures")) {
                fprintf(tracef, "    tex %08x\n", rd32(p->a[1]));
            }
            if (!strcmp(p->sym->name, "CreateDIBSection")) {
                fprintf(tracef, "    bits %08x\n", rd32(p->a[3]));
            }
        }
        gl_ret(p->ttbr, p->sym->name, p->a, r0, 0);
        if (!strcmp(p->sym->name, "eglGetProcAddress") && r0) {
            char nm[96];
            rstr(p->a[0], nm, sizeof(nm));
            add_sym(r0, nm);
        }
        g_ptr_array_remove_index_fast(list, i);
        g_free(p);
        break;
    }
    g_mutex_unlock(&lock);
}

static bool native_gpu;            /* native=1: let the AUI's GPU work run too */

/*
 * GPU work the bridge already drew. The guest driver waits for the SGX to
 * finish it (eglSwapBuffers never returns once a frame stalls), so the AUI
 * thread hangs and stops reading input; skip it and return success instead.
 */
static bool skip_in_guest(uint32_t ttbr, const char *n, uint32_t *ret)
{
    Proc *p = g_hash_table_lookup(procs, GUINT_TO_POINTER(ttbr));

    /*
     * Each call waits a second for an SGX blit that never completes. Navi's
     * map engine makes ten of them per surface DC it fails to get, so a map
     * draw takes ~40 s and the map screen shows up only after minutes.
     */
    if (!native_gpu && !strcmp(n, "DDWaitForBltDone")) {
        *ret = 0;
        return true;
    }
    if (native_gpu || !getenv("QY8_GL_FRAME") || !p ||
        !g_hash_table_size(p->prog_frag)) {
        return false;
    }
    *ret = 0;
    if (!strcmp(n, "eglSwapBuffers")) {
        *ret = 1;
        return true;
    }
    return !strcmp(n, "glDrawArrays") || !strcmp(n, "glDrawElements") ||
           !strcmp(n, "glClear") || !strcmp(n, "glFlush") ||
           !strcmp(n, "glFinish");
}

/*
 * cnf=CODE:VALUE answers SYS_CNF_read(CODE, buf, 1) with VALUE without
 * reading the store. leafsdtools disables the immobiliser check by writing
 * code 0x0f = 0 with SYS_CNF_write; flash programming isn't modelled well
 * enough for that write to land, so the read is answered instead.
 */
static int cnf_code = -1;
static uint8_t cnf_value;

static void on_call(unsigned int vcpu, void *udata)
{
    Sym *s = udata;
    uint32_t a[4] = { rd(reg_r[0]), rd(reg_r[1]), rd(reg_r[2]), rd(reg_r[3]) };
    uint32_t sp = rd(reg_sp), lr = rd(reg_lr);
    uint32_t ttbr = have_ttbr ? rd(reg_ttbr) : 0;
    Pending *p;
    GPtrArray *list;

    g_mutex_lock(&lock);
    s->calls++;
    if (strstr(s->name, "DebugString") || strstr(s->name, "DbgPrintf")) {
        if (tracef) {
            debug_msg(s->name, a, sp, lr);
        }
    } else if (tracef && logged < log_limit) {
        logged++;
        fprintf(tracef, "[%08x] %s(%08x, %08x, %08x, %08x | %08x %08x) lr=%08x\n",
                ttbr, s->name, a[0], a[1], a[2], a[3], rd32(sp), rd32(sp + 4), lr);
        detail(s, a, sp);
    }
    gl_call(ttbr, s->name, a, sp);

    if (cnf_code >= 0 && a[0] == (uint32_t)cnf_code && a[2] == 1 && !(lr & 1) &&
        !strcmp(s->name, "SYS_CNF_read")) {
        g_autoptr(GByteArray) v = g_byte_array_new();
        g_autoptr(GByteArray) r = g_byte_array_new();
        uint32_t ok = 0;

        g_byte_array_append(v, &cnf_value, 1);
        g_byte_array_append(r, (const guint8 *)&ok, 4);
        g_mutex_unlock(&lock);
        qemu_plugin_write_memory_vaddr(a[1], v);
        qemu_plugin_write_register(reg_r[0], r);
        qemu_plugin_set_pc(lr);
    }

    uint32_t ret;
    if (!(lr & 1) && skip_in_guest(ttbr, s->name, &ret)) {
        g_autoptr(GByteArray) b = g_byte_array_new();

        g_byte_array_append(b, (const guint8 *)&ret, 4);
        g_mutex_unlock(&lock);
        qemu_plugin_write_register(reg_r[0], b);
        qemu_plugin_set_pc(lr);
    }

    p = g_new0(Pending, 1);
    p->sym = s;
    p->sp = sp;
    p->ttbr = ttbr;
    memcpy(p->a, a, sizeof(a));
    list = g_hash_table_lookup(ret_sites, GUINT_TO_POINTER(lr & ~1u));
    if (!list) {
        list = g_ptr_array_new();
        g_hash_table_insert(ret_sites, GUINT_TO_POINTER(lr & ~1u), list);
    }
    if (list->len < 64) {
        g_ptr_array_add(list, p);
    } else {
        g_free(p);
    }
    g_mutex_unlock(&lock);
}

static void tb_trans(struct qemu_plugin_tb *tb, void *udata)
{
    size_t n = qemu_plugin_tb_n_insns(tb);

    g_mutex_lock(&lock);
    for (size_t i = 0; i < n; i++) {
        struct qemu_plugin_insn *insn = qemu_plugin_tb_get_insn(tb, i);
        uint32_t va = qemu_plugin_insn_vaddr(insn);
        Sym *s = g_hash_table_lookup(by_addr, GUINT_TO_POINTER(va));

        if (s) {
            qemu_plugin_register_vcpu_insn_exec_cb(insn, on_call,
                                                   QEMU_PLUGIN_CB_RW_REGS, s);
        }
        if (g_hash_table_contains(ret_sites, GUINT_TO_POINTER(va))) {
            qemu_plugin_register_vcpu_insn_exec_cb(insn, on_ret,
                                                   QEMU_PLUGIN_CB_R_REGS,
                                                   GUINT_TO_POINTER(va));
        }
    }
    g_mutex_unlock(&lock);
}

static void vcpu_init(unsigned int vcpu, void *udata)
{
    g_autoptr(GArray) regs = qemu_plugin_get_registers();

    for (guint i = 0; i < regs->len; i++) {
        qemu_plugin_reg_descriptor *d =
            &g_array_index(regs, qemu_plugin_reg_descriptor, i);

        for (int k = 0; k < 4; k++) {
            char nm[4];
            snprintf(nm, sizeof(nm), "r%d", k);
            if (!strcmp(d->name, nm)) {
                reg_r[k] = d->handle;
            }
        }
        if (!strcmp(d->name, "sp")) {
            reg_sp = d->handle;
        }
        if (!strcmp(d->name, "lr")) {
            reg_lr = d->handle;
        }
        /* one translation table per process: tells the GL clients apart */
        if (!g_ascii_strcasecmp(d->name, "TTBR0") ||
            !g_ascii_strcasecmp(d->name, "TTBR0_EL1")) {
            reg_ttbr = d->handle;
            have_ttbr = true;
        }
    }
    if (tracef) {
        fprintf(tracef, "# ttbr register %s\n", have_ttbr ? "found" : "missing");
    }
}

static void at_exit(void *p)
{
    if (!tracef) {
        return;
    }
    fprintf(tracef, "# summary\n");
    for (guint i = 0; i < syms->len; i++) {
        Sym *s = g_ptr_array_index(syms, i);
        if (s->calls) {
            fprintf(tracef, "# %10" PRIu64 " %s\n", s->calls, s->name);
        }
    }
    fclose(tracef);
}

static bool load_syms(const char *path)
{
    g_autofree char *text = NULL;
    g_auto(GStrv) lines = NULL;

    if (!g_file_get_contents(path, &text, NULL, NULL)) {
        return false;
    }
    lines = g_strsplit(text, "\n", -1);
    for (int i = 0; lines[i]; i++) {
        unsigned ord, addr;
        char name[128];

        if (sscanf(lines[i], "%u %x %127s", &ord, &addr, name) != 3) {
            continue;
        }
        if (!strcmp(name, "@fs_texcolor")) {
            FS_TEXCOLOR = addr;
        } else if (!strcmp(name, "@fs_keyed_a")) {
            FS_KEYED_A = addr;
        } else if (!strcmp(name, "@fs_keyed_b")) {
            FS_KEYED_B = addr;
        } else if (!strcmp(name, "@fs_tint")) {
            FS_TINT = addr;
        } else {
            add_sym(addr, name);
        }
    }
    return syms->len > 0;
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                           const qemu_info_t *info,
                                           int argc, char **argv)
{
    const char *sympath = NULL, *logpath = "qy8gl.log";

    by_addr = g_hash_table_new(NULL, NULL);
    procs = g_hash_table_new(NULL, NULL);
    ret_sites = g_hash_table_new(NULL, NULL);
    syms = g_ptr_array_new();

    for (int i = 0; i < argc; i++) {
        g_auto(GStrv) kv = g_strsplit(argv[i], "=", 2);
        if (!g_strcmp0(kv[0], "syms")) {
            sympath = g_strdup(kv[1]);
        } else if (!g_strcmp0(kv[0], "log")) {
            logpath = g_strdup(kv[1]);
        } else if (!g_strcmp0(kv[0], "cnf")) {
            char *v = strchr(kv[1], ':');
            if (v) {
                cnf_code = g_ascii_strtoull(kv[1], NULL, 0);
                cnf_value = g_ascii_strtoull(v + 1, NULL, 0);
            }
        } else if (!g_strcmp0(kv[0], "native")) {
            native_gpu = g_ascii_strtoull(kv[1], NULL, 0) != 0;
        } else if (!g_strcmp0(kv[0], "out")) {
            outdir = g_strdup(kv[1]);
        } else if (!g_strcmp0(kv[0], "limit")) {
            log_limit = g_ascii_strtoull(kv[1], NULL, 0);
        }
    }
    if (!sympath || !load_syms(sympath)) {
        fprintf(stderr, "qy8gl: need syms=FILE with '<ord> <addr> <name>'\n");
        return -1;
    }
    tracef = fopen(logpath, "w");
    setvbuf(tracef, NULL, _IOLBF, 0);

    qemu_plugin_register_vcpu_init_cb(id, vcpu_init, NULL);
    qemu_plugin_register_vcpu_tb_trans_cb(id, tb_trans, NULL);
    qemu_plugin_register_atexit_cb(id, at_exit, NULL);
    return 0;
}
