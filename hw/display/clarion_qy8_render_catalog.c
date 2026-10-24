/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "plugins/qemu-plugin.h"
#include "qemu/error-report.h"
#include "clarion_qy8_render_catalog.h"

static gboolean read_guest(uint64_t address, size_t size, GByteArray *bytes)
{
    return qemu_plugin_read_memory_vaddr(address, bytes, size) &&
           bytes->len == size;
}

static gboolean read_guest_u16(uint64_t address, uint16_t *value)
{
    GByteArray *bytes = g_byte_array_new();
    gboolean ok = read_guest(address, 2, bytes);

    if (ok) {
        memcpy(value, bytes->data, sizeof(*value));
    }
    g_byte_array_unref(bytes);
    return ok;
}

static gboolean read_guest_u32(uint64_t address, uint32_t *value)
{
    GByteArray *bytes = g_byte_array_new();
    gboolean ok = read_guest(address, 4, bytes);

    if (ok) {
        memcpy(value, bytes->data, sizeof(*value));
    }
    g_byte_array_unref(bytes);
    return ok;
}

static gboolean read_guest_string(uint32_t address, char *value, size_t size)
{
    size_t used = 0;

    while (used + 1 < size) {
        GByteArray *bytes = g_byte_array_new();
        size_t chunk = MIN(size - used - 1, 32);
        gboolean ok = read_guest((uint64_t)address + used, chunk, bytes);
        const uint8_t *nul;

        if (!ok) {
            g_byte_array_unref(bytes);
            value[used] = 0;
            return FALSE;
        }
        nul = memchr(bytes->data, 0, chunk);
        if (nul) {
            size_t length = nul - bytes->data;

            memcpy(value + used, bytes->data, length);
            value[used + length] = 0;
            g_byte_array_unref(bytes);
            return TRUE;
        }
        memcpy(value + used, bytes->data, chunk);
        used += chunk;
        g_byte_array_unref(bytes);
    }
    value[size - 1] = 0;
    return FALSE;
}

static gboolean map_rva(const Qy8RenderModule *module, uint32_t rva,
                        uint32_t *address)
{
    for (uint32_t i = 0; i < module->sections; i++) {
        uint64_t section = (uint64_t)module->o32 + i * 0x18;
        uint32_t vsize, section_rva, psize, data_va;
        uint32_t extent;

        if (!read_guest_u32(section, &vsize) ||
            !read_guest_u32(section + 4, &section_rva) ||
            !read_guest_u32(section + 8, &psize) ||
            !read_guest_u32(section + 12, &data_va)) {
            continue;
        }
        extent = MAX(vsize, psize);
        if (rva >= section_rva &&
            (uint64_t)rva < (uint64_t)section_rva + extent && data_va) {
            *address = data_va + rva - section_rva;
            return TRUE;
        }
    }
    return FALSE;
}

static Qy8RenderModule *find_module(uint32_t romhdr, const char *wanted)
{
    uint32_t count;

    if (!read_guest_u32(romhdr + 0x10, &count) || !count || count > 4096) {
        return NULL;
    }
    for (uint32_t i = 0; i < count; i++) {
        uint64_t entry = (uint64_t)romhdr + 0x54 + i * 0x20;
        uint32_t name_va, e32, o32, sections;
        Qy8RenderModule *module;
        char name[96];

        if (!read_guest_u32(entry + 0x10, &name_va) ||
            !read_guest_u32(entry + 0x14, &e32) ||
            !read_guest_u32(entry + 0x18, &o32) ||
            !read_guest_string(name_va, name, sizeof(name)) ||
            g_ascii_strcasecmp(name, wanted)) {
            continue;
        }
        if (!read_guest_u32(e32, &sections)) {
            return NULL;
        }
        module = g_new0(Qy8RenderModule, 1);
        module->name = g_strdup(name);
        module->e32 = e32;
        module->o32 = o32;
        module->sections = sections & 0xffff;
        if (!read_guest_u32(e32 + 8, &module->vbase) ||
            !read_guest_u32(e32 + 0x14, &module->vsize) ||
            !read_guest_u32(e32 + 0x24, &module->export_rva) ||
            !read_guest_u32(e32 + 0x28, &module->export_size)) {
            g_free(module->name);
            g_free(module);
            return NULL;
        }
        return module;
    }
    return NULL;
}

Qy8RenderModule *qy8_render_module_by_name(GPtrArray *modules, const char *name)
{
    for (guint i = 0; i < modules->len; i++) {
        Qy8RenderModule *module = g_ptr_array_index(modules, i);

        if (!g_ascii_strcasecmp(module->name, name)) {
            return module;
        }
    }
    return NULL;
}

Qy8RenderModule *qy8_render_module_for_pc(GPtrArray *modules, uint32_t pc)
{
    pc &= ~1u;
    for (guint i = 0; i < modules->len; i++) {
        Qy8RenderModule *module = g_ptr_array_index(modules, i);

        if (pc >= module->vbase &&
            (uint64_t)pc < (uint64_t)module->vbase + module->vsize) {
            return module;
        }
    }
    return NULL;
}

static gboolean find_romhdr(uint32_t *romhdr, GPtrArray *modules)
{
    for (uint32_t base = 0x80000000; base < 0x90000000; base += 0x10000) {
        uint32_t signature, candidate, count;
        Qy8RenderModule *egl, *gles, *aui;

        if (!read_guest_u32((uint64_t)base + 0x40, &signature) ||
            signature != 0x43454345 ||
            !read_guest_u32((uint64_t)base + 0x44, &candidate) ||
            !read_guest_u32((uint64_t)candidate + 0x10, &count) || !count ||
            count > 4096) {
            continue;
        }
        egl = find_module(candidate, "libEGL.dll");
        gles = find_module(candidate, "libGLESv2.dll");
        aui = find_module(candidate, "auirtdll.dll");
        if (egl && gles && aui) {
            *romhdr = candidate;
            g_ptr_array_add(modules, egl);
            g_ptr_array_add(modules, gles);
            g_ptr_array_add(modules, aui);
            return TRUE;
        }
        g_free(egl);
        g_free(gles);
        g_free(aui);
    }
    return FALSE;
}

static gboolean lookup_export(const Qy8RenderModule *module, const char *wanted,
                              uint32_t *rva, uint32_t *word)
{
    uint32_t exp, nfunc, nname, af_rva, an_rva, ao_rva;
    uint32_t af, an, ao;

    if (!map_rva(module, module->export_rva, &exp) ||
        !read_guest_u32(exp + 20, &nfunc) ||
        !read_guest_u32(exp + 24, &nname) ||
        !read_guest_u32(exp + 28, &af_rva) ||
        !read_guest_u32(exp + 32, &an_rva) ||
        !read_guest_u32(exp + 36, &ao_rva) || nfunc > 65536 || nname > 65536 ||
        !map_rva(module, af_rva, &af) || !map_rva(module, an_rva, &an) ||
        !map_rva(module, ao_rva, &ao)) {
        return FALSE;
    }
    for (uint32_t i = 0; i < nname; i++) {
        uint32_t name_rva, name_va;
        uint16_t ordinal_index;
        char name[128];

        if (!read_guest_u32((uint64_t)an + i * 4, &name_rva) ||
            !map_rva(module, name_rva, &name_va) ||
            !read_guest_string(name_va, name, sizeof(name))) {
            continue;
        }
        if (strcmp(name, wanted)) {
            continue;
        }
        if (!read_guest_u16((uint64_t)ao + i * 2, &ordinal_index) ||
            ordinal_index >= nfunc ||
            !read_guest_u32((uint64_t)af + ordinal_index * 4, rva) ||
            !read_guest_u32((uint64_t)module->vbase + *rva, word)) {
            return FALSE;
        }
        return TRUE;
    }
    return FALSE;
}

gboolean qy8_render_catalog_ensure(const char *const *names, size_t count,
                                   GHashTable *exports, GPtrArray *modules)
{
    uint32_t romhdr;
    Qy8RenderModule *egl, *gles;

    if (modules->len) {
        return TRUE;
    }
    if (!find_romhdr(&romhdr, modules)) {
        return FALSE;
    }
    egl = qy8_render_module_by_name(modules, "libEGL.dll");
    gles = qy8_render_module_by_name(modules, "libGLESv2.dll");
    for (size_t i = 0; i < count; i++) {
        Qy8RenderModule *module =
            g_str_has_prefix(names[i], "egl") ? egl : gles;
        Qy8RenderExport *entry = g_new0(Qy8RenderExport, 1);

        entry->name = names[i];
        entry->base = module->vbase;
        entry->index = i;
        if (!lookup_export(module, names[i], &entry->rva, &entry->word)) {
            error_report_once("clarion-qy8-render: export %s missing from %s",
                         names[i], module->name);
            g_hash_table_remove_all(exports);
            g_free(entry);
            g_ptr_array_set_size(modules, 0);
            return FALSE;
        }
        g_hash_table_insert(exports, GUINT_TO_POINTER(entry->base + entry->rva),
                            entry);
    }
    return TRUE;
}

Qy8RenderExport *qy8_render_export_by_name(GHashTable *exports,
                                           const char *name)
{
    GHashTableIter iter;
    gpointer value;

    g_hash_table_iter_init(&iter, exports);
    while (g_hash_table_iter_next(&iter, NULL, &value)) {
        Qy8RenderExport *entry = value;

        if (!strcmp(entry->name, name)) {
            return entry;
        }
    }
    return NULL;
}
