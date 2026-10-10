#include <errno.h>
#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "hw/misc/clarion_boards.h"

static const ClarionBoardInfo *find_model(const char *model)
{
    for (size_t i = 0; i < G_N_ELEMENTS(clarion_boards); i++) {
        if (!strcmp(model, clarion_boards[i].model)) {
            return &clarion_boards[i];
        }
    }
    return NULL;
}

static gboolean read_model(const char *path,
                           char model[CLARION_PROD_MODEL_LEN + 1])
{
    FILE *file = fopen(path, "rb");

    if (!file) {
        return FALSE;
    }
    for (size_t i = 0; i < G_N_ELEMENTS(clarion_prod_offsets); i++) {
        char signature[4];
        if (fseeko(file, clarion_prod_offsets[i], SEEK_SET) ||
            fread(signature, 1, sizeof(signature), file) != sizeof(signature) ||
            memcmp(signature, "PROD", sizeof(signature)) ||
            fseeko(file, clarion_prod_offsets[i] + CLARION_PROD_MODEL_OFF,
                   SEEK_SET) ||
            fread(model, 1, CLARION_PROD_MODEL_LEN, file) !=
                CLARION_PROD_MODEL_LEN) {
            continue;
        }
        for (size_t j = 0; j < CLARION_PROD_MODEL_LEN; j++) {
            if (!g_ascii_isgraph(model[j])) {
                model[j] = '.';
            }
        }
        model[CLARION_PROD_MODEL_LEN] = '\0';
        fclose(file);
        return TRUE;
    }
    fclose(file);
    return FALSE;
}

static void usage(const char *program)
{
    fprintf(stderr,
            "usage: %s [-n] [--read-only] [--headless] NOR [SD] "
            "[QEMU arguments...]\n",
            program);
}

static char *drive_argument(const char *path, const char *interface,
                            gboolean read_only)
{
    GString *drive = g_string_new(NULL);

    if (!strcmp(interface, "pflash")) {
        g_string_append(drive, "if=pflash,format=raw,file=");
    } else {
        g_string_append(drive, "if=sd,index=0,format=raw,file=");
    }
    for (const char *p = path; *p; p++) {
        g_string_append_c(drive, *p);
        if (*p == ',') {
            g_string_append_c(drive, ',');
        }
    }
    if (read_only) {
        g_string_append(drive, ",snapshot=on");
    }
    return g_string_free(drive, FALSE);
}

static gboolean has_console_argument(int argc, char **argv)
{
    for (int i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "-serial") || !strcmp(argv[i], "-display") ||
            !strcmp(argv[i], "-nographic")) {
            return TRUE;
        }
    }
    return FALSE;
}

static gboolean has_argument(int argc, char **argv, const char *name)
{
    for (int i = 0; i < argc; i++) {
        if (!strcmp(argv[i], name)) {
            return TRUE;
        }
    }
    return FALSE;
}

static gboolean check_writable(const char *path, gboolean read_only)
{
    if (!read_only && access(path, W_OK)) {
        fprintf(stderr, "qemu-clarion: %s is not writable; use --read-only\n",
                path);
        return FALSE;
    }
    return TRUE;
}

int main(int argc, char **argv)
{
    gboolean dry_run = FALSE;
    gboolean read_only = FALSE;
    gboolean headless = FALSE;
    int first_arg = 1;
    const char *flash;
    const char *sd = NULL;
    char model[CLARION_PROD_MODEL_LEN + 1] = "";
    const ClarionBoardInfo *board;
    char *program_path;
    char *directory;
    char *emulator;
    char **args;
    int arg_count = 0;
    gboolean user_console;

    while (first_arg < argc) {
        if (!strcmp(argv[first_arg], "-n")) {
            dry_run = TRUE;
        } else if (!strcmp(argv[first_arg], "--read-only")) {
            read_only = TRUE;
        } else if (!strcmp(argv[first_arg], "--headless")) {
            headless = TRUE;
        } else {
            break;
        }
        first_arg++;
    }
    if (first_arg >= argc) {
        usage(argv[0]);
        return 2;
    }
    flash = argv[first_arg++];
    if (first_arg < argc && argv[first_arg][0] != '-') {
        sd = argv[first_arg++];
    }
    if (!read_model(flash, model)) {
        fprintf(stderr, "qemu-clarion: no PROD block in %s\n", flash);
        return 1;
    }
    board = find_model(model);
    if (!board) {
        fprintf(stderr, "qemu-clarion: unknown model %s; supported: ", model);
        for (size_t i = 0; i < G_N_ELEMENTS(clarion_boards); i++) {
            fprintf(stderr, "%s%s", i ? ", " : "", clarion_boards[i].model);
        }
        fputc('\n', stderr);
        return 1;
    }
    if (sd && !board->has_sd) {
        fprintf(stderr, "qemu-clarion: %s does not support an SD card\n",
                model);
        return 1;
    }
    if (!check_writable(flash, read_only) ||
        (sd && !check_writable(sd, read_only))) {
        return 1;
    }

    program_path = g_find_program_in_path(argv[0]);
    if (!program_path) {
        program_path = g_strdup(argv[0]);
    }
    directory = g_path_get_dirname(program_path);
    emulator = g_strdup_printf("%s/qemu-system-%s", directory, board->target);
    args = g_new0(char *, argc + 12);
    args[arg_count++] = emulator;
    args[arg_count++] = (char *)"-M";
    args[arg_count++] = (char *)board->machine;
    args[arg_count++] = (char *)"-drive";
    args[arg_count++] = drive_argument(flash, "pflash", read_only);
    if (sd) {
        args[arg_count++] = (char *)"-drive";
        args[arg_count++] = drive_argument(sd, "sd", read_only);
    }
    user_console = has_console_argument(argc - first_arg, argv + first_arg);
    if (!user_console) {
        args[arg_count++] = (char *)(headless ? "-nographic" : "-serial");
        if (!headless) {
            args[arg_count++] = (char *)"mon:stdio";
        }
    }
    if (board->icount &&
        !has_argument(argc - first_arg, argv + first_arg, "-icount")) {
        args[arg_count++] = (char *)"-icount";
        args[arg_count++] = (char *)board->icount;
    }
    for (int i = first_arg; i < argc; i++) {
        args[arg_count++] = argv[i];
    }
    args[arg_count] = NULL;
    if (dry_run) {
        printf("%s", emulator);
        for (int i = 1; i < arg_count; i++) {
            printf(" %s", args[i]);
        }
        putchar('\n');
        return 0;
    }
    execv(emulator, args);
    fprintf(stderr, "qemu-clarion: cannot execute %s: %s\n", emulator,
            strerror(errno));
    return 1;
}
