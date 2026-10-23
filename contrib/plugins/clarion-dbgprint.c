/*
 * Print Windows CE debug strings emitted by Clarion guests.
 *
 * Load with -plugin libclarion-dbgprint.dylib,image=FILE[,out=FILE]. Image
 * paths may be repeated. A syms=FILE option can supply symbols manually.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <glib.h>
#include <qemu-plugin.h>

#define E32_EXP 0x24

static const char *wanted[] = {
    "NKDbgPrintfW", "NKvDbgPrintfW", "OutputDebugStringW"
};

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

typedef struct {
    uint32_t addr;
    char *name;
} DbgSymbol;

static GHashTable *symbols;
static FILE *output;
static bool arm_target;
static GMutex output_lock;
static struct qemu_plugin_register *reg_arg[4];
static struct qemu_plugin_register *reg_sp;
static struct qemu_plugin_register *reg_lr;

typedef struct {
    const guint8 *data;
    gsize size;
    gsize base;
    uint32_t physfirst;
    gsize romhdr;
    uint32_t count;
} Xip;

typedef struct {
    uint32_t rva;
    uint32_t size;
    uint32_t dataptr;
    bool compressed;
} XipSection;

static bool image_u16(const guint8 *data, gsize size, gsize off,
                      uint16_t *value)
{
    if (off > size || size - off < 2) {
        return false;
    }
    *value = data[off] | ((uint16_t)data[off + 1] << 8);
    return true;
}

static bool image_u32(const guint8 *data, gsize size, gsize off,
                      uint32_t *value)
{
    if (off > size || size - off < 4) {
        return false;
    }
    *value = data[off] | ((uint32_t)data[off + 1] << 8) |
             ((uint32_t)data[off + 2] << 16) |
             ((uint32_t)data[off + 3] << 24);
    return true;
}

static bool xip_file_offset(const Xip *xip, uint32_t va, gsize *offset)
{
    uint64_t off = (uint64_t)va - xip->physfirst + xip->base;
    if (off >= xip->size) {
        return false;
    }
    *offset = off;
    return true;
}

static bool xip_rva_offset(const Xip *xip, const XipSection *sections,
                           size_t nsections, uint32_t rva, size_t length,
                           gsize *offset)
{
    for (size_t i = 0; i < nsections; i++) {
        uint64_t end = (uint64_t)rva + length;
        uint64_t sec_end = (uint64_t)sections[i].rva + sections[i].size;
        if (rva >= sections[i].rva && end <= sec_end &&
            !sections[i].compressed) {
            uint64_t dataptr = (uint64_t)sections[i].dataptr +
                               rva - sections[i].rva;
            if (dataptr > UINT32_MAX ||
                !xip_file_offset(xip, dataptr, offset) ||
                length > xip->size - *offset) {
                return false;
            }
            return true;
        }
    }
    return false;
}

static bool coredll_exports(const Xip *xip, uint32_t addresses[3])
{
    gsize toc = xip->romhdr + 0x54;
    for (uint32_t i = 0; i < xip->count; i++) {
        uint32_t name_va, e32_va, o32_va;
        gsize name_off, e32, o32;
        char *name;
        uint16_t nsections;
        XipSection *sections;
        uint32_t vbase, exp_rva;
        gsize exp;
        uint32_t nfunc, nname, funcs_rva, names_rva, ords_rva;
        gsize funcs, names, ords;
        char **export_names;
        bool found[3] = { false, false, false };
        bool valid = false;

        if (!image_u32(xip->data, xip->size, toc + i * 0x20 + 0x10,
                       &name_va) ||
            !xip_file_offset(xip, name_va, &name_off)) {
            continue;
        }
        name = g_strndup((const char *)xip->data + name_off,
                         strnlen((const char *)xip->data + name_off,
                                 xip->size - name_off));
        if (g_ascii_strcasecmp(name, "coredll.dll")) {
            g_free(name);
            continue;
        }
        g_free(name);
        if (!image_u32(xip->data, xip->size, toc + i * 0x20 + 0x14,
                       &e32_va) ||
            !image_u32(xip->data, xip->size, toc + i * 0x20 + 0x18,
                       &o32_va) ||
            !xip_file_offset(xip, e32_va, &e32) ||
            !xip_file_offset(xip, o32_va, &o32) ||
            !image_u16(xip->data, xip->size, e32, &nsections) ||
            nsections > 96 || o32 > xip->size ||
            (size_t)nsections > (xip->size - o32) / 0x18 ||
            !image_u32(xip->data, xip->size, e32 + 8, &vbase) ||
            !image_u32(xip->data, xip->size, e32 + E32_EXP, &exp_rva)) {
            continue;
        }
        sections = g_new0(XipSection, nsections);
        for (uint16_t j = 0; j < nsections; j++) {
            uint32_t vsize, rva, psize, dataptr, flags;
            gsize sec = o32 + (gsize)j * 0x18;
            if (!image_u32(xip->data, xip->size, sec, &vsize) ||
                !image_u32(xip->data, xip->size, sec + 4, &rva) ||
                !image_u32(xip->data, xip->size, sec + 8, &psize) ||
                !image_u32(xip->data, xip->size, sec + 12, &dataptr) ||
                !image_u32(xip->data, xip->size, sec + 20, &flags)) {
                goto section_done;
            }
            sections[j] = (XipSection){ rva, psize, dataptr,
                                        !!(flags & 0x2000) };
        }
        if (!xip_rva_offset(xip, sections, nsections, exp_rva, 40, &exp) ||
            !image_u32(xip->data, xip->size, exp + 20, &nfunc) ||
            !image_u32(xip->data, xip->size, exp + 24, &nname) ||
            !image_u32(xip->data, xip->size, exp + 28, &funcs_rva) ||
            !image_u32(xip->data, xip->size, exp + 32, &names_rva) ||
            !image_u32(xip->data, xip->size, exp + 36, &ords_rva) ||
            nfunc > 100000 || nname > 100000 ||
            !xip_rva_offset(xip, sections, nsections, funcs_rva,
                            (size_t)nfunc * 4, &funcs) ||
            !xip_rva_offset(xip, sections, nsections, names_rva,
                            (size_t)nname * 4, &names) ||
            !xip_rva_offset(xip, sections, nsections, ords_rva,
                            (size_t)nname * 2, &ords)) {
            goto section_done;
        }
        export_names = g_new0(char *, nfunc);
        for (uint32_t j = 0; j < nname; j++) {
            uint32_t nrva;
            uint16_t ordinal;
            gsize noff;
            if (!image_u32(xip->data, xip->size, names + (gsize)j * 4,
                           &nrva) ||
                !image_u16(xip->data, xip->size, ords + (gsize)j * 2,
                           &ordinal) || ordinal >= nfunc ||
                !xip_rva_offset(xip, sections, nsections, nrva, 1, &noff)) {
                continue;
            }
            export_names[ordinal] = g_strndup(
                (const char *)xip->data + noff,
                strnlen((const char *)xip->data + noff,
                        xip->size - noff));
        }
        for (uint32_t j = 0; j < nfunc; j++) {
            uint32_t rva;
            if (!export_names[j] ||
                !image_u32(xip->data, xip->size, funcs + (gsize)j * 4,
                           &rva) || !rva) {
                continue;
            }
            for (size_t k = 0; k < G_N_ELEMENTS(wanted); k++) {
                if (!strcmp(export_names[j], wanted[k])) {
                    addresses[k] = vbase + rva;
                    found[k] = true;
                }
            }
        }
        valid = found[0] && found[1] && found[2];
        for (uint32_t j = 0; j < nfunc; j++) {
            g_free(export_names[j]);
        }
        g_free(export_names);
section_done:
        g_free(sections);
        if (valid) {
            return true;
        }
    }
    return false;
}

static bool load_image_symbols(const char *path)
{
    g_autoptr(GError) error = NULL;
    g_autoptr(GMappedFile) mapped = NULL;
    const gchar *contents;
    gsize size;
    bool loaded = false;
    mapped = g_mapped_file_new(path, FALSE, &error);
    if (!mapped) {
        fprintf(stderr, "clarion-dbgprint: cannot map image %s: %s\n", path,
                error->message);
        return false;
    }
    contents = g_mapped_file_get_contents(mapped);
    size = g_mapped_file_get_length(mapped);
    for (gsize marker = 0; marker + 4 <= size; marker++) {
        Xip xip;
        uint32_t romhdr_va, physfirst32, count, physlast;
        bool valid = false;
        if (memcmp(contents + marker, "ECEC", 4) || marker < 0x40) {
            continue;
        }
        xip = (Xip){ (const guint8 *)contents, size, marker - 0x40, 0,
                     0, 0 };
        if (!image_u32(xip.data, size, xip.base + 0x44, &romhdr_va)) {
            continue;
        }
        for (uint64_t physfirst = 0x80000000; physfirst < 0x90000000;
             physfirst += 0x10000) {
            int64_t rh = (int64_t)(uint64_t)romhdr_va - physfirst + xip.base;
            if (rh < 0 || (uint64_t)rh > size || size - rh < 0x100 ||
                !image_u32(xip.data, size, rh + 8, &physfirst32) ||
                physfirst32 != physfirst ||
                !image_u32(xip.data, size, rh + 0x10, &count) ||
                !count || count >= 2000 ||
                !image_u32(xip.data, size, rh + 0xc, &physlast) ||
                physfirst >= physlast) {
                continue;
            }
            xip.physfirst = physfirst;
            xip.romhdr = rh;
            xip.count = count;
            valid = true;
            break;
        }
        if (valid) {
            uint32_t addresses[3] = { 0 };
            if (coredll_exports(&xip, addresses)) {
                for (size_t i = 0; i < G_N_ELEMENTS(wanted); i++) {
                    DbgSymbol *sym = g_new0(DbgSymbol, 1);
                    sym->addr = addresses[i];
                    sym->name = g_strdup(wanted[i]);
                    g_hash_table_replace(symbols,
                                         GUINT_TO_POINTER(addresses[i]), sym);
                }
                loaded = true;
            }
        }
    }
    return loaded;
}

static void free_symbol(gpointer value)
{
    DbgSymbol *sym = value;

    g_free(sym->name);
    g_free(sym);
}

static struct qemu_plugin_register *find_register(const char *name)
{
    g_autoptr(GArray) regs = qemu_plugin_get_registers();
    for (guint i = 0; i < regs->len; i++) {
        qemu_plugin_reg_descriptor *reg =
            &g_array_index(regs, qemu_plugin_reg_descriptor, i);
        if (!strcmp(reg->name, name)) {
            return reg->handle;
        }
    }
    return NULL;
}

static uint32_t read_reg(struct qemu_plugin_register *reg)
{
    g_autoptr(GByteArray) value = g_byte_array_new();
    uint32_t result = 0;
    if (qemu_plugin_read_register(reg, value) && value->len >= 4) {
        memcpy(&result, value->data, sizeof(result));
    }
    return result;
}

static bool read_guest(uint32_t addr, void *dst, size_t len)
{
    g_autoptr(GByteArray) value = g_byte_array_new();
    if (!addr || !len || !qemu_plugin_read_memory_vaddr(addr, value, len) ||
        value->len < len) {
        return false;
    }
    memcpy(dst, value->data, len);
    return true;
}

static uint32_t read_u32(uint32_t addr)
{
    uint32_t value = 0;
    read_guest(addr, &value, sizeof(value));
    return value;
}

static void read_wide(uint32_t addr, char *dst, size_t size)
{
    size_t i;
    for (i = 0; i + 1 < size; i++) {
        uint16_t ch = 0;
        if (!read_guest(addr + i * 2, &ch, sizeof(ch)) || !ch) {
            break;
        }
        dst[i] = ch < 0x80 ? (char)ch : '?';
    }
    dst[i] = '\0';
}

static void read_ascii(uint32_t addr, char *dst, size_t size)
{
    size_t i;
    for (i = 0; i + 1 < size; i++) {
        if (!read_guest(addr + i, &dst[i], 1) || !dst[i]) {
            break;
        }
    }
    dst[i] = '\0';
}

typedef struct {
    const uint32_t *regs;
    unsigned int count;
    unsigned int index;
    uint32_t stack;
} ArgIter;

static uint32_t next_arg(ArgIter *it)
{
    if (it->index < it->count) {
        return it->regs[it->index++];
    }
    uint32_t value = read_u32(it->stack);
    it->stack += sizeof(value);
    return value;
}

static void format_message(uint32_t fmt_addr, ArgIter *args, GString *message)
{
    char fmt[512];
    read_wide(fmt_addr, fmt, sizeof(fmt));
    for (const char *p = fmt; *p; p++) {
        char buf[256], spec[16];
        size_t n = 0;
        if (*p != '%') {
            g_string_append_c(message, *p);
            continue;
        }
        spec[n++] = *p++;
        while (*p && strchr("-+ #0123456789.lhwI", *p) && n < 12) {
            if (*p != 'l' && *p != 'h' && *p != 'w' && *p != 'I') {
                spec[n++] = *p;
            }
            p++;
        }
        if (!*p) {
            break;
        }
        switch (*p) {
        case '%':
            g_string_append_c(message, '%');
            break;
        case 's':
            read_wide(next_arg(args), buf, sizeof(buf));
            g_string_append(message, buf);
            break;
        case 'S':
            read_ascii(next_arg(args), buf, sizeof(buf));
            g_string_append(message, buf);
            break;
        case 'c':
        case 'C':
            g_string_append_c(message, (char)next_arg(args));
            break;
        case 'd':
        case 'i':
        case 'u':
        case 'x':
        case 'X':
        case 'o':
        case 'p':
            spec[n++] = *p == 'p' ? 'x' : *p;
            spec[n] = '\0';
            snprintf(buf, sizeof(buf), spec, next_arg(args));
            g_string_append(message, buf);
            break;
        default:
            g_string_append_c(message, *p);
            break;
        }
    }
}

/*
 * NKDbgPrintfW forwards to NKvDbgPrintfW a few instructions after its entry.
 * Skip calls returning that close to a hooked entry so each message is
 * printed once.
 */
#define NESTED_CALL_WINDOW 0x40

static bool called_from_hook(uint32_t lr)
{
    GHashTableIter iter;
    gpointer key;

    lr &= ~1u;
    g_hash_table_iter_init(&iter, symbols);
    while (g_hash_table_iter_next(&iter, &key, NULL)) {
        if (lr - GPOINTER_TO_UINT(key) < NESTED_CALL_WINDOW) {
            return true;
        }
    }
    return false;
}

/*
 * A card can hold several XIP copies of coredll.dll. An entry of an inactive
 * copy may fall a few instructions into the same function of the active one,
 * even inside a loop, so one call can reach several hooks with the same
 * return address. Compare each hit with the entry the call started at.
 */
static bool same_call_as_last(uint32_t addr, uint32_t lr)
{
    static uint32_t entry_addr, entry_lr;

    if (lr == entry_lr && addr - entry_addr - 1 < NESTED_CALL_WINDOW) {
        return true;
    }
    entry_addr = addr;
    entry_lr = lr;
    return false;
}

static void print_debug(unsigned int vcpu_index, void *userdata)
{
    DbgSymbol *sym = userdata;
    uint32_t args[4] = { 0 }, sp, lr, fmt_addr;
    bool skip;
    g_autoptr(GString) message = g_string_new(NULL);
    (void)vcpu_index;
    for (unsigned int i = 0; i < 4; i++) {
        args[i] = read_reg(reg_arg[i]);
    }
    sp = read_reg(reg_sp);
    lr = read_reg(reg_lr);
    if (called_from_hook(lr)) {
        return;
    }
    g_mutex_lock(&output_lock);
    skip = same_call_as_last(sym->addr, lr);
    g_mutex_unlock(&output_lock);
    if (skip) {
        return;
    }
    if (!strcmp(sym->name, "OutputDebugStringW")) {
        char text[512];
        read_wide(args[0], text, sizeof(text));
        g_string_assign(message, text);
    } else {
        ArgIter iter;
        fmt_addr = args[0];
        if (!strcmp(sym->name, "NKvDbgPrintfW")) {
            iter = (ArgIter){ NULL, 0, 0, args[1] };
        } else if (arm_target) {
            iter = (ArgIter){ args + 1, 3, 0, sp };
        } else {
            iter = (ArgIter){ args + 1, 3, 0, sp + 16 };
        }
        format_message(fmt_addr, &iter, message);
    }
    while (message->len && (message->str[message->len - 1] == '\n' ||
                            message->str[message->len - 1] == '\r')) {
        g_string_truncate(message, message->len - 1);
    }
    if (message->len) {
        g_mutex_lock(&output_lock);
        fprintf(output, "DBG %08" PRIx32 " %s\n", lr, message->str);
        fflush(output);
        g_mutex_unlock(&output_lock);
    }
}

static void vcpu_init(unsigned int vcpu_index, void *data)
{
    (void)vcpu_index;
    (void)data;
    for (unsigned int i = 0; i < 4; i++) {
        char name[16];
        if (arm_target) {
            snprintf(name, sizeof(name), "r%u", i);
        } else {
            snprintf(name, sizeof(name), "r%u", i + 4);
        }
        reg_arg[i] = find_register(name);
    }
    reg_sp = find_register(arm_target ? "sp" : "r15");
    reg_lr = find_register(arm_target ? "lr" : "pr");
}

static void tb_trans(struct qemu_plugin_tb *tb, void *userdata)
{
    (void)userdata;
    size_t n = qemu_plugin_tb_n_insns(tb);
    for (size_t i = 0; i < n; i++) {
        struct qemu_plugin_insn *insn = qemu_plugin_tb_get_insn(tb, i);
        DbgSymbol *sym = g_hash_table_lookup(
            symbols, GUINT_TO_POINTER((uint32_t)qemu_plugin_insn_vaddr(insn)));
        if (sym) {
            qemu_plugin_register_vcpu_insn_exec_cb(insn, print_debug,
                                                   QEMU_PLUGIN_CB_R_REGS, sym);
        }
    }
}

static bool load_symbols(const char *path)
{
    g_autofree char *contents = NULL;
    g_auto(GStrv) lines = NULL;
    if (!g_file_get_contents(path, &contents, NULL, NULL)) {
        return false;
    }
    lines = g_strsplit(contents, "\n", -1);
    for (size_t i = 0; lines[i]; i++) {
        uint32_t addr;
        char name[128];
        if (sscanf(lines[i], "%" SCNx32 " %127s", &addr, name) != 2) {
            continue;
        }
        DbgSymbol *sym = g_new0(DbgSymbol, 1);
        sym->addr = addr;
        sym->name = g_strdup(name);
        g_hash_table_insert(symbols, GUINT_TO_POINTER(addr), sym);
    }
    return g_hash_table_size(symbols) > 0;
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                           const qemu_info_t *info, int argc,
                                           char **argv)
{
    g_autofree char *sym_path = NULL;
    g_autofree char *out_path = NULL;
    GPtrArray *images = g_ptr_array_new_with_free_func(g_free);
    if (!strcmp(info->target_name, "arm")) {
        arm_target = true;
    } else if (!strcmp(info->target_name, "sh4")) {
        arm_target = false;
    } else {
        fprintf(stderr, "clarion-dbgprint: unsupported target %s\n",
                info->target_name);
        return -1;
    }
    for (int i = 0; i < argc; i++) {
        g_auto(GStrv) option = g_strsplit(argv[i], "=", 2);
        if (!option[1]) {
            return -1;
        }
        if (!strcmp(option[0], "syms")) {
            g_free(sym_path);
            sym_path = g_strdup(option[1]);
        } else if (!strcmp(option[0], "out")) {
            g_free(out_path);
            out_path = g_strdup(option[1]);
        } else if (!strcmp(option[0], "image")) {
            g_ptr_array_add(images, g_strdup(option[1]));
        } else {
            fprintf(stderr, "clarion-dbgprint: unknown option %s\n", option[0]);
            return -1;
        }
    }
    if (!sym_path && !images->len) {
        fprintf(stderr, "clarion-dbgprint: requires image=FILE or syms=FILE\n");
        return -1;
    }
    symbols =
        g_hash_table_new_full(g_direct_hash, g_direct_equal, NULL, free_symbol);
    if ((sym_path && !load_symbols(sym_path))) {
        fprintf(stderr, "clarion-dbgprint: cannot load symbols from %s\n",
                sym_path);
        return -1;
    }
    for (guint i = 0; i < images->len; i++) {
        load_image_symbols(g_ptr_array_index(images, i));
    }
    if (!g_hash_table_size(symbols)) {
        fprintf(stderr, "clarion-dbgprint: no debug-print exports found; "
                "guest will continue without debug output\n");
        g_ptr_array_free(images, TRUE);
        return 0;
    }
    {
        GHashTableIter iter;
        gpointer value;
        g_hash_table_iter_init(&iter, symbols);
        while (g_hash_table_iter_next(&iter, NULL, &value)) {
            DbgSymbol *sym = value;
            fprintf(stderr, "clarion-dbgprint: %s at %08" PRIx32 "\n",
                    sym->name, sym->addr);
        }
    }
    g_ptr_array_free(images, TRUE);
    output = out_path ? fopen(out_path, "w") : stderr;
    if (!output) {
        fprintf(stderr, "clarion-dbgprint: cannot open output file\n");
        return -1;
    }
    setvbuf(output, NULL, _IOLBF, 0);
    qemu_plugin_register_vcpu_init_cb(id, vcpu_init, NULL);
    qemu_plugin_register_vcpu_tb_trans_cb(id, tb_trans, NULL);
    return 0;
}
