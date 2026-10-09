#ifndef HW_MISC_CLARION_BOARDS_H
#define HW_MISC_CLARION_BOARDS_H

/* Shared, QEMU-independent unit model to board mapping. */
#define CLARION_PROD_MODEL_OFF 0x40
#define CLARION_PROD_MODEL_LEN 8

static const unsigned long long clarion_prod_offsets[] = { 0x40000, 0xa000 };

typedef struct ClarionBoardInfo {
    const char *model;
    const char *name;
    const char *target;
    const char *machine;
} ClarionBoardInfo;

static const ClarionBoardInfo clarion_boards[] = {
    { "QY8652NB", "qy8652nb", "arm", "clarion-qy8" },
    { "QY8202NA", "qy8202na", "arm", "clarion-qy8" },
    { "QY7221NL", "qy7221nl", "sh4", "clarion-qy7" },
};

#endif
