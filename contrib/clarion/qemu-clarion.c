#include "config-host.h"
#include <errno.h>
#include <glib.h>
#include <glib/gstdio.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
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

static void usage(const char *program)
{
    fprintf(stderr,
            "usage: %s [-n] [--read-only] [--headless] "
            "[--dbgprint FILE] NOR [SD] "
            "[QEMU arguments...]\n"
            "       %s --identify NOR\n",
            program, program);
}

/* Printable build string at @offset, e.g. "G218ENNI.120"; "" if none. */
static void read_build(const char *path, unsigned long long offset,
                       char build[17])
{
    FILE *file = fopen(path, "rb");
    size_t n = 0;

    build[0] = '\0';
    if (!file) {
        return;
    }
    if (!fseeko(file, offset, SEEK_SET)) {
        n = fread(build, 1, 16, file);
    }
    fclose(file);
    for (size_t i = 0; i < n; i++) {
        if (!g_ascii_isgraph(build[i])) {
            n = i;
            break;
        }
    }
    build[n] = '\0';
}

/*
 * Print what a hardware support request needs: the unit model and the
 * kernel build. Nothing else from PROD (serial numbers, codes) is shown.
 */
static int identify(const char *flash)
{
    char model[CLARION_PROD_MODEL_LEN + 1] = "";
    char build[17];
    const ClarionSeries *series;
    GStatBuf st;

    if (!read_model(flash, model)) {
        fprintf(stderr, "qemu-clarion: no PROD block in %s\n", flash);
        return 1;
    }
    series = clarion_series_find(model);
    printf("model:    %s\n", model);
    if (!series) {
        printf("series:   unknown (expected QY7 or QY8)\n");
        return 1;
    }
    read_build(flash, series->build_off, build);
    printf("series:   %s (machine %s, qemu-system-%s)\n", series->prefix,
           series->machine, series->target);
    printf("firmware: %s (NOR 0x%llx)\n", build[0] ? build : "not found",
           series->build_off);
    if (!g_stat(flash, &st)) {
        printf("image:    %lld bytes\n", (long long)st.st_size);
    }
    return 0;
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
                            gboolean read_only)
{
    GString *drive = g_string_new(NULL);

    if (!strcmp(interface, "pflash")) {
        g_string_append_printf(drive, "if=pflash,format=%s,file=",
                               read_only ? "qcow2" : "raw");
    } else {
        g_string_append_printf(drive, "if=sd,index=0,format=%s,file=",
                               read_only ? "qcow2" : "raw");
    }
    for (const char *p = path; *p; p++) {
        g_string_append_c(drive, *p);
        if (*p == ',') {
            g_string_append_c(drive, ',');
        }
    }
    return g_string_free(drive, FALSE);
}

static volatile sig_atomic_t active_child;
static volatile sig_atomic_t caught_signal;

static void forward_signal(int signal_number)
{
    caught_signal = signal_number;
    if (active_child > 0) {
        kill(active_child, signal_number);
    }
}

static gboolean install_signal_forwarding(void)
{
    const int signals[] = { SIGINT, SIGTERM, SIGHUP, SIGQUIT };
    struct sigaction action = { 0 };

    action.sa_handler = forward_signal;
    sigemptyset(&action.sa_mask);
    for (size_t i = 0; i < G_N_ELEMENTS(signals); i++) {
        if (sigaction(signals[i], &action, NULL)) {
            fprintf(stderr, "qemu-clarion: cannot handle signals: %s\n",
                    strerror(errno));
            return FALSE;
        }
    }
    return TRUE;
}

static int run_supervised(char **args)
{
    for (;;) {
        int status;
        caught_signal = 0;
        pid_t child = fork();

        if (child < 0) {
            fprintf(stderr, "qemu-clarion: cannot fork: %s\n",
                    strerror(errno));
            return 1;
        }
        if (!child) {
            struct sigaction action = { .sa_handler = SIG_DFL };

            sigemptyset(&action.sa_mask);
            sigaction(SIGINT, &action, NULL);
            sigaction(SIGTERM, &action, NULL);
            sigaction(SIGHUP, &action, NULL);
            sigaction(SIGQUIT, &action, NULL);
            execv(args[0], args);
            fprintf(stderr, "qemu-clarion: cannot execute %s: %s\n",
                    args[0], strerror(errno));
            _exit(errno == ENOENT ? 127 : 126);
        }

        active_child = child;
        if (caught_signal) {
            kill(child, caught_signal);
        }
        while (waitpid(child, &status, 0) < 0) {
            if (errno == EINTR) {
                continue;
            }
            fprintf(stderr, "qemu-clarion: waitpid failed: %s\n",
                    strerror(errno));
            active_child = 0;
            return 1;
        }
        active_child = 0;

        if (caught_signal) {
            return 128 + caught_signal;
        }
        if (WIFEXITED(status)) {
            int exit_code = WEXITSTATUS(status);

            if (exit_code == CLARION_POWER_CYCLE_EXIT_CODE) {
                fprintf(stderr, "qemu-clarion: power cycle; restarting QEMU\n");
                continue;
            }
            return exit_code;
        }
        if (WIFSIGNALED(status)) {
            return 128 + WTERMSIG(status);
        }
        return 1;
    }
}

static gboolean create_overlay(const char *qemu_img, const char *backing,
                               const char *overlay)
{
    char *args[] = { (char *)qemu_img, (char *)"create", (char *)"-q",
                     (char *)"-f", (char *)"qcow2", (char *)"-F",
                     (char *)"raw", (char *)"-b", (char *)backing,
                     (char *)overlay, NULL };
    g_autoptr(GError) error = NULL;
    int status;

    if (!g_spawn_sync(NULL, args, NULL, G_SPAWN_SEARCH_PATH, NULL, NULL,
                      NULL, NULL, &status, &error)) {
        fprintf(stderr, "qemu-clarion: cannot create overlay: %s\n",
                error->message);
        return FALSE;
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status)) {
        fprintf(stderr, "qemu-clarion: qemu-img failed creating %s\n",
                overlay);
        return FALSE;
    }
    return TRUE;
}

static void remove_temp_overlays(const char *temp_dir, const char *nor,
                                 const char *sd)
{
    if (nor) {
        g_unlink(nor);
    }
    if (sd) {
        g_unlink(sd);
    }
    if (temp_dir) {
        g_rmdir(temp_dir);
    }
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
    const char *dbgprint = NULL;
    int first_arg = 1;
    const char *flash;
    const char *sd = NULL;
    const char *flash_drive;
    const char *sd_drive = NULL;
    char model[CLARION_PROD_MODEL_LEN + 1] = "";
    const ClarionSeries *board;
    g_autofree char *program_path = NULL;
    g_autofree char *directory = NULL;
    g_autofree char *emulator = NULL;
    g_autofree char *qemu_img = NULL;
    g_autofree char *flash_backing = NULL;
    g_autofree char *sd_backing = NULL;
    g_autofree char *temp_dir = NULL;
    g_autofree char *nor_overlay = NULL;
    g_autofree char *sd_overlay = NULL;
    g_autofree char *nor_drive = NULL;
    g_autofree char *sd_drive_arg = NULL;
    g_autofree char **args = NULL;
    int arg_count = 0;
    gboolean user_console;

    if (argc == 3 && !strcmp(argv[1], "--identify")) {
        return identify(argv[2]);
    }
    while (first_arg < argc) {
        if (!strcmp(argv[first_arg], "-h") ||
            !strcmp(argv[first_arg], "--help")) {
            usage(argv[0]);
            return 0;
        } else if (!strcmp(argv[first_arg], "-n")) {
            dry_run = TRUE;
        } else if (!strcmp(argv[first_arg], "--read-only")) {
            read_only = TRUE;
        } else if (!strcmp(argv[first_arg], "--headless")) {
            headless = TRUE;
        } else if (!strcmp(argv[first_arg], "--dbgprint")) {
            if (first_arg + 1 >= argc) {
                usage(argv[0]);
                return 2;
            }
            dbgprint = argv[++first_arg];
        } else if (argv[first_arg][0] == '-') {
            fprintf(stderr, "qemu-clarion: unknown option %s\n",
                    argv[first_arg]);
            usage(argv[0]);
            return 2;
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
    board = clarion_series_find(model);
    if (!board) {
        fprintf(stderr, "qemu-clarion: unknown model %s; expected a QY7 or "
                "QY8 unit\n", model);
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
    /* Release packages keep the engines in libexec/; build trees do not. */
    emulator = g_strdup_printf("%s/libexec/qemu-system-%s", directory,
                               board->target);
    if (!g_file_test(emulator, G_FILE_TEST_IS_EXECUTABLE)) {
        g_free(emulator);
        emulator = g_strdup_printf("%s/qemu-system-%s", directory,
                                   board->target);
    }
    flash_drive = flash;
    sd_drive = sd;
    if (read_only) {
        if (dry_run) {
            nor_overlay = g_strdup("<temporary-nor-overlay.qcow2>");
            if (sd) {
                sd_overlay = g_strdup("<temporary-sd-overlay.qcow2>");
            }
        } else {
            g_autofree char *qemu_img_libexec =
                g_strdup_printf("%s/libexec/qemu-img", directory);
            g_autofree char *qemu_img_build =
                g_strdup_printf("%s/qemu-img", directory);

            qemu_img = g_file_test(qemu_img_libexec, G_FILE_TEST_IS_EXECUTABLE)
                ? g_steal_pointer(&qemu_img_libexec)
                : g_steal_pointer(&qemu_img_build);
            if (!g_file_test(qemu_img, G_FILE_TEST_IS_EXECUTABLE)) {
                fprintf(stderr, "qemu-clarion: qemu-img not found beside %s\n",
                        argv[0]);
                return 1;
            }
            temp_dir = g_dir_make_tmp("qemu-clarion-XXXXXX", NULL);
            if (!temp_dir) {
                fprintf(stderr, "qemu-clarion: cannot create temp directory\n");
                return 1;
            }
            nor_overlay = g_build_filename(temp_dir, "nor.qcow2", NULL);
            if (sd) {
                sd_overlay = g_build_filename(temp_dir, "sd.qcow2", NULL);
            }
            flash_backing = g_canonicalize_filename(flash, NULL);
            if (sd) {
                sd_backing = g_canonicalize_filename(sd, NULL);
            }
            if (!create_overlay(qemu_img, flash_backing, nor_overlay) ||
                (sd && !create_overlay(qemu_img, sd_backing, sd_overlay))) {
                remove_temp_overlays(temp_dir, nor_overlay, sd_overlay);
                return 1;
            }
        }
        flash_drive = nor_overlay;
        sd_drive = sd_overlay;
    }
    args = g_new0(char *, argc + 14);
    args[arg_count++] = emulator;
    args[arg_count++] = (char *)"-M";
    args[arg_count++] = (char *)board->machine;
    args[arg_count++] = (char *)"-drive";
    nor_drive = drive_argument(flash_drive, "pflash", read_only);
    args[arg_count++] = nor_drive;
    if (sd) {
        args[arg_count++] = (char *)"-drive";
        sd_drive_arg = drive_argument(sd_drive, "sd", read_only);
        args[arg_count++] = sd_drive_arg;
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
        if (read_only) {
            fprintf(stderr, "qemu-clarion: read-only mode uses temporary "
                    "qcow2 overlays\n");
        }
        printf("%s", emulator);
        for (int i = 1; i < arg_count; i++) {
            printf(" %s", args[i]);
        }
        putchar('\n');
        return 0;
    }
    if (!install_signal_forwarding()) {
        remove_temp_overlays(temp_dir, nor_overlay, sd_overlay);
        return 1;
    }
    int exit_code = run_supervised(args);
    remove_temp_overlays(temp_dir, nor_overlay, sd_overlay);
    return exit_code;
}
