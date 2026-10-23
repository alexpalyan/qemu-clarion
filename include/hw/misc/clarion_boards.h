#ifndef HW_MISC_CLARION_BOARDS_H
#define HW_MISC_CLARION_BOARDS_H

/*
 * Shared, QEMU-independent unit series table. The series comes from the unit
 * model in the PROD block; each machine picks its board from the firmware.
 */
#define CLARION_PROD_MODEL_OFF 0x40
#define CLARION_PROD_MODEL_LEN 8
/* QY7221NL: 16-byte SD card lock password stored in PROD at +0x100. */
#define CLARION_PROD_CARD_PWD_OFF 0x100
#define CLARION_PROD_CARD_PWD_LEN 16

/* Exit status for a guest power cycle; the launcher restarts QEMU on it. */
#define CLARION_POWER_CYCLE_EXIT_CODE 0xc7

static const unsigned long long clarion_prod_offsets[] = { 0x40000, 0xa000 };

typedef struct ClarionSeries {
    const char *prefix;         /* unit model prefix */
    const char *target;
    const char *machine;
    int has_sd;
    /*
     * Default -icount argument, or NULL. The QY7 kernel reprograms its tick
     * timer with a correction computed a few hundred instructions earlier;
     * with host-clock virtual time that gap is unbounded and the guest can
     * sleep forever, so its virtual time follows instructions.
     */
    const char *icount;
} ClarionSeries;

static const ClarionSeries clarion_series[] = {
    { "QY7", "sh4", "clarion-qy7", 1, "shift=2" },
    { "QY8", "arm", "clarion-qy8", 1, NULL },
};

static inline const ClarionSeries *clarion_series_find(const char *model)
{
    for (size_t i = 0; i < sizeof(clarion_series) / sizeof(*clarion_series);
         i++) {
        if (!strncmp(model, clarion_series[i].prefix,
                     strlen(clarion_series[i].prefix))) {
            return &clarion_series[i];
        }
    }
    return NULL;
}

#endif
