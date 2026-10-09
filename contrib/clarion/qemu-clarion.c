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
    fprintf(stderr, "usage: %s [-n] FLASH [QEMU arguments...]\n", program);
}

int main(int argc, char **argv)
{
    gboolean dry_run = FALSE;
    int first_arg = 1;
    const char *flash;
    char model[CLARION_PROD_MODEL_LEN + 1] = "";
    const ClarionBoardInfo *board;
    char *program_path;
    char *directory;
    char *emulator;
    char **args;
    int arg_count;

    if (argc > 1 && !strcmp(argv[1], "-n")) {
        dry_run = TRUE;
        first_arg++;
    }
    if (first_arg >= argc) {
        usage(argv[0]);
        return 2;
    }
    flash = argv[first_arg++];
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
    program_path = g_find_program_in_path(argv[0]);
    if (!program_path) {
        program_path = g_strdup(argv[0]);
    }
    directory = g_path_get_dirname(program_path);
    emulator = g_strdup_printf("%s/qemu-system-%s", directory, board->target);
    args = g_new0(char *, argc + 6);
    arg_count = 0;
    args[arg_count++] = emulator;
    args[arg_count++] = (char *)"-M";
    args[arg_count++] = (char *)board->machine;
    args[arg_count++] = (char *)"-drive";
    args[arg_count++] = g_strdup_printf("if=pflash,format=raw,file=%s", flash);
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
