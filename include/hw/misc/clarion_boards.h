#ifndef HW_MISC_CLARION_BOARDS_H
#define HW_MISC_CLARION_BOARDS_H

/* Shared, QEMU-independent unit model to board mapping. */
#define CLARION_PROD_MODEL_OFF 0x40
#define CLARION_PROD_MODEL_LEN 8
/* QY7221NL: 16-byte SD card lock password stored in PROD at +0x100. */
#define CLARION_PROD_CARD_PWD_OFF 0x100
#define CLARION_PROD_CARD_PWD_LEN 16

static const unsigned long long clarion_prod_offsets[] = { 0x40000, 0xa000 };

typedef struct ClarionBoardInfo {
    const char *model;
    const char *name;
    const char *target;
    const char *machine;
    int has_sd;
    /*
     * Default -icount argument, or NULL. The QY7221NL kernel reprograms its
     * tick timer with a correction computed a few hundred instructions
     * earlier; with host-clock virtual time that gap is unbounded and the
     * guest can sleep forever, so its virtual time follows instructions.
     */
    const char *icount;
} ClarionBoardInfo;

static const ClarionBoardInfo clarion_boards[] = {
    { "QY8652NB", "qy8652nb", "arm", "clarion-qy8", 1, NULL },
    { "QY8202NA", "qy8202na", "arm", "clarion-qy8", 1, NULL },
    { "QY8602NB", "qy8602nb", "arm", "clarion-qy8", 1, NULL },
    { "QY8600NE", "qy8600ne", "arm", "clarion-qy8", 1, NULL },
    { "QY8202PB", "qy8202pb", "arm", "clarion-qy8", 1, NULL },
    { "QY7221NL", "qy7221nl", "sh4", "clarion-qy7", 1, "shift=2" },
};

/*
 * Units missing from the table above run on their series' machine, chosen
 * by the model prefix. Such a unit has not been tested; callers warn.
 */
static const ClarionBoardInfo clarion_series[] = {
    { "QY7", "qy7", "sh4", "clarion-qy7", 1, "shift=2" },
    { "QY8", "qy8", "arm", "clarion-qy8", 1, NULL },
};

/* Look up a unit model; *untested is set when only the series matched. */
static inline const ClarionBoardInfo *clarion_board_find(const char *model,
                                                         int *untested)
{
    *untested = 0;
    for (size_t i = 0; i < sizeof(clarion_boards) / sizeof(*clarion_boards);
         i++) {
        if (!strcmp(model, clarion_boards[i].model)) {
            return &clarion_boards[i];
        }
    }
    for (size_t i = 0; i < sizeof(clarion_series) / sizeof(*clarion_series);
         i++) {
        if (!strncmp(model, clarion_series[i].model,
                     strlen(clarion_series[i].model))) {
            *untested = 1;
            return &clarion_series[i];
        }
    }
    return NULL;
}

#endif
