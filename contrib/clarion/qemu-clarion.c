#include "config-host.h"
#include <errno.h>
#include <fcntl.h>
#include <glib.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "hw/misc/clarion_boards.h"

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

/*
 * Every refusal to launch goes through here so a caller can grep one string
 * ("qemu-clarion: refused:") instead of matching each message by hand.
 */
static int refuse(const char *fmt, ...)
{
    va_list ap;

    fprintf(stderr, "qemu-clarion: refused: ");
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
    return 1;
}

static void usage(const char *program)
{
    fprintf(stderr,
            "usage: %s [-n] [--writable] [--headless] [--log FILE] "
            "[--dbgprint FILE] NOR [SD] [QEMU arguments...]\n"
            "       %s --help\n",
            program, program);
}

static void help(const char *program)
{
    usage(program);
    fprintf(stderr,
            "\n"
            "  NOR            flash dump to boot; its PROD block picks the board\n"
            "  SD             SD card image, if the board has a slot\n"
            "  -n             print the resolved qemu-system-* command line, don't run it\n"
            "  --writable     write NOR and SD in place (default: snapshot=on,\n"
            "                 an in-memory overlay discarded on exit)\n"
            "  --headless     -nographic instead of a display window\n"
            "  --log FILE     send this process's stdout and stderr, and so the\n"
            "                 emulator's, to FILE instead of the console\n"
            "  --dbgprint FILE  capture guest debug output to FILE\n"
            "  --help         this text\n"
            "\n"
            "Anything else is passed straight to qemu-system-*. A passed-through\n"
            "-M replaces the board's own (so board properties, e.g. gps=off,\n"
            "can be set); -serial/-display/-nographic/-icount are likewise left\n"
            "alone if already given.\n"
            "\n"
            "Before the engine runs, this prints one status line to stdout:\n"
            "  qemu-clarion: ready             NOR and SD are readable, a board was\n"
            "                                   found, about to hand off to the engine\n"
            "  qemu-clarion: refused: REASON   on stderr, nothing was run\n"
            "There is no status line for the engine exiting: once the handoff\n"
            "happens this process becomes qemu-system-*, so the caller's own\n"
            "wait() on it carries that exit code, same as running it directly.\n");
}

static char *find_dbgprint_plugin(const char *directory)
{
#ifdef CONFIG_DARWIN
    const char *extension = "dylib";
#else
    const char *extension = "so";
#endif
    char *path = g_strdup_printf("%s/lib/libclarion-dbgprint.%s",
                                 directory, extension);
    if (g_file_test(path, G_FILE_TEST_IS_REGULAR)) {
        return path;
    }
    g_free(path);
    path = g_strdup_printf("%s/contrib/plugins/libclarion-dbgprint.%s",
                           directory, extension);
    if (g_file_test(path, G_FILE_TEST_IS_REGULAR)) {
        return path;
    }
    g_free(path);
    return NULL;
}

static char *drive_argument(const char *path, const char *interface,
                            gboolean writable)
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
    if (!writable) {
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

static gboolean check_writable(const char *path)
{
    if (access(path, W_OK)) {
        fprintf(stderr,
                "qemu-clarion: refused: %s is not writable; drop --writable "
                "to use the default read-only overlay\n",
                path);
        return FALSE;
    }
    return TRUE;
}

/* --log opens FILE once and dups it onto both streams, so the launcher's own
 * status lines and the emulator's stdio land in one file in the order they
 * were written. Two independent freopen()s would race for the same offset. */
static gboolean redirect_log(const char *path)
{
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);

    if (fd < 0) {
        fprintf(stderr, "qemu-clarion: refused: cannot open log %s: %s\n",
                path, strerror(errno));
        return FALSE;
    }
    fflush(stdout);
    fflush(stderr);
    dup2(fd, STDOUT_FILENO);
    dup2(fd, STDERR_FILENO);
    close(fd);
    return TRUE;
}

int main(int argc, char **argv)
{
    gboolean dry_run = FALSE;
    gboolean writable = FALSE;
    gboolean headless = FALSE;
    const char *dbgprint = NULL;
    const char *log_path = NULL;
    int first_arg = 1;
    const char *flash;
    const char *sd = NULL;
    char model[CLARION_PROD_MODEL_LEN + 1] = "";
    const ClarionBoardInfo *board;
    int untested;
    char *program_path;
    char *directory;
    char *emulator;
    char **args;
    int arg_count = 0;
    gboolean user_console;

    while (first_arg < argc) {
        if (!strcmp(argv[first_arg], "-n")) {
            dry_run = TRUE;
        } else if (!strcmp(argv[first_arg], "--writable")) {
            writable = TRUE;
        } else if (!strcmp(argv[first_arg], "--headless")) {
            headless = TRUE;
        } else if (!strcmp(argv[first_arg], "--log")) {
            if (first_arg + 1 >= argc) {
                usage(argv[0]);
                return 2;
            }
            log_path = argv[++first_arg];
        } else if (!strcmp(argv[first_arg], "--dbgprint")) {
            if (first_arg + 1 >= argc) {
                usage(argv[0]);
                return 2;
            }
            dbgprint = argv[++first_arg];
        } else if (!strcmp(argv[first_arg], "--help")) {
            help(argv[0]);
            return 0;
        } else {
            break;
        }
        first_arg++;
    }
    if (first_arg >= argc) {
        usage(argv[0]);
        return 2;
    }
    if (log_path && !redirect_log(log_path)) {
        return 1;
    }
    flash = argv[first_arg++];
    if (first_arg < argc && argv[first_arg][0] != '-') {
        sd = argv[first_arg++];
    }
    if (!read_model(flash, model)) {
        return refuse("no PROD block in %s", flash);
    }
    board = clarion_board_find(model, &untested);
    if (!board) {
        fprintf(stderr, "qemu-clarion: refused: unknown model %s; supported: ",
                model);
        for (size_t i = 0; i < G_N_ELEMENTS(clarion_boards); i++) {
            fprintf(stderr, "%s%s", i ? ", " : "", clarion_boards[i].model);
        }
        fputc('\n', stderr);
        return 1;
    }
    if (untested) {
        fprintf(stderr, "qemu-clarion: warning: %s has not been tested; "
                "using the %s machine\n", model, board->machine);
    }
    if (sd && !board->has_sd) {
        return refuse("%s does not support an SD card", model);
    }
    /* snapshot=on (the default) needs no write access, so only check it
     * when a write was actually asked for. */
    if (writable &&
        (!check_writable(flash) || (sd && !check_writable(sd)))) {
        return 1;
    }

    program_path = g_find_program_in_path(argv[0]);
    if (!program_path) {
        program_path = g_strdup(argv[0]);
    }
    directory = g_path_get_dirname(program_path);
    /* Release packages keep the engines in libexec/; build trees do not. */
    emulator = g_strdup_printf("%s/libexec/qemu-system-%s", directory,
                               board->target);
    if (!g_file_test(emulator, G_FILE_TEST_IS_EXECUTABLE)) {
        g_free(emulator);
        emulator = g_strdup_printf("%s/qemu-system-%s", directory,
                                   board->target);
    }
    args = g_new0(char *, argc + 14);
    args[arg_count++] = emulator;
    /* A caller who already knows the machine string they want (board
     * properties included) gets to use it, same as -serial/-display below. */
    if (!has_argument(argc - first_arg, argv + first_arg, "-M")) {
        args[arg_count++] = (char *)"-M";
        args[arg_count++] = (char *)board->machine;
    }
    args[arg_count++] = (char *)"-drive";
    args[arg_count++] = drive_argument(flash, "pflash", writable);
    if (sd) {
        args[arg_count++] = (char *)"-drive";
        args[arg_count++] = drive_argument(sd, "sd", writable);
    }
    user_console = has_console_argument(argc - first_arg, argv + first_arg);
    if (!user_console) {
        if (headless) {
            args[arg_count++] = (char *)"-nographic";
        } else {
#ifdef CONFIG_DARWIN
            /* The Cocoa window hides the pointer over the guest by default. */
            args[arg_count++] = (char *)"-display";
            args[arg_count++] = (char *)"cocoa,show-cursor=on";
#endif
            args[arg_count++] = (char *)"-serial";
            args[arg_count++] = (char *)"mon:stdio";
        }
    }
    if (board->icount &&
        !has_argument(argc - first_arg, argv + first_arg, "-icount")) {
        args[arg_count++] = (char *)"-icount";
        args[arg_count++] = (char *)board->icount;
    }
    if (dbgprint) {
        g_autofree char *plugin_path = find_dbgprint_plugin(directory);
        g_autofree char *plugin_arg = NULL;
        GString *plugin;
        if (!plugin_path) {
            fprintf(stderr, "qemu-clarion: cannot find "
                    "libclarion-dbgprint plugin beside %s\n", argv[0]);
            return 1;
        }
        plugin = g_string_new(plugin_path);
        g_string_append_printf(plugin, ",image=%s", flash);
        if (sd) {
            g_string_append_printf(plugin, ",image=%s", sd);
        }
        g_string_append_printf(plugin, ",out=%s", dbgprint);
        plugin_arg = g_string_free(plugin, FALSE);
        args[arg_count++] = (char *)"-plugin";
        args[arg_count++] = g_steal_pointer(&plugin_arg);
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
    fputs("qemu-clarion: ready\n", stdout);
    fflush(stdout);
    execv(emulator, args);
    fprintf(stderr, "qemu-clarion: refused: cannot execute %s: %s\n", emulator,
            strerror(errno));
    return 1;
}
