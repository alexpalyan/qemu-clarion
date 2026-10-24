/*
 * Clarion QY7221NL: synthetic body control module (BCM).
 *
 * Antitheft.exe sends a request on CAN ID 0x71E every 666 ms and waits for
 * an answer on 0x72E (docs/21-antitheft.md, statically checked against the
 * handler at Antitheft.exe+0x149E4).  Eight data bytes per frame:
 *
 *   step 1  request   14 seed F0 .. .. sum FF FF     (seed 0..31)
 *           answer    14 T T T T sum FF FF           T = key[seed]
 *   step 2  request   24 c0 c1 c2 c3 sum FF FF       (c0..c3 = vehicle code)
 *           answer    24 c0 c1 c2 c3 sum FF FF       (the code, acknowledged)
 *
 * key[] is the 32-byte table at Antitheft.exe VA 0x11D78.  The model has no
 * data of its own: the table is read out of the guest's own flash image
 * (the module is stored uncompressed there), located by the code pointers
 * that precede it, and the step-2 answer repeats the code the guest itself
 * sends (the check is Antitheft.exe+0x14C28: opcode 0x24, bytes 1..4 equal
 * to the ones it sent).  A real BCM would reject a code that does not match
 * the vehicle; this one accepts any.  Nothing about the vehicle is stored
 * or logged.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "system/memory.h"
#include "system/address-spaces.h"
#include "hw/misc/clarion_bcm.h"
#include "hw/misc/clarion_scpu.h"

#define BCM_ID_REQUEST  0x71e
#define BCM_ID_ANSWER   0x72e
#define BCM_KEY_SIZE    32

/* Bus turnaround of the real module is a few milliseconds. */
#define BCM_ANSWER_DELAY_NS (8 * 1000 * 1000)

/* Flash is mapped at physical 0. */
#define BCM_FLASH_SIZE  (8 * MiB)
#define BCM_SCAN_CHUNK  (64 * KiB)

/*
 * Pointers into Antitheft.exe that immediately precede key[] (VA 0x11D6C).
 * INFERENCE: stable for the QY7221NL image this model was written against;
 * if the anchor is not found the BCM stays silent and logs once.
 */
static const uint8_t bcm_key_anchor[] = {
    0xa8, 0x43, 0x01, 0x00, 0x00, 0x57, 0x01, 0x00, 0x38, 0x7a, 0x01, 0x00,
};

struct ClarionBcmState {
    DeviceState parent_obj;

    DeviceState *scpu;
    QEMUTimer *timer;

    uint8_t key[BCM_KEY_SIZE];
    bool key_loaded;
    bool key_failed;

    uint8_t seed;
    bool seed_valid;

    uint8_t answer[8];
};

static bool bcm_load_key(ClarionBcmState *s)
{
    uint8_t *buf;
    unsigned keep = sizeof(bcm_key_anchor) + BCM_KEY_SIZE;
    uint64_t pos;

    if (s->key_loaded) {
        return true;
    }
    if (s->key_failed) {
        return false;
    }
    /* chunks overlap by the pattern length so no match straddles a gap */
    buf = g_malloc(BCM_SCAN_CHUNK + keep);
    for (pos = 0; pos < BCM_FLASH_SIZE; pos += BCM_SCAN_CHUNK) {
        unsigned n = MIN(BCM_SCAN_CHUNK + keep, BCM_FLASH_SIZE - pos);

        if (address_space_read(&address_space_memory, pos,
                               MEMTXATTRS_UNSPECIFIED, buf, n) != MEMTX_OK) {
            break;
        }
        for (unsigned i = 0; i + keep <= n; i++) {
            if (!memcmp(buf + i, bcm_key_anchor, sizeof(bcm_key_anchor))) {
                memcpy(s->key, buf + i + sizeof(bcm_key_anchor),
                       BCM_KEY_SIZE);
                s->key_loaded = true;
                break;
            }
        }
        if (s->key_loaded) {
            break;
        }
    }
    g_free(buf);
    if (!s->key_loaded) {
        s->key_failed = true;
        qemu_log_mask(LOG_GUEST_ERROR, "clarion-bcm: key table not found in "
                      "flash, BCM stays silent\n");
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "clarion-bcm: key table located in flash\n");
    }
    return s->key_loaded;
}

static void bcm_answer_expire(void *opaque)
{
    ClarionBcmState *s = opaque;

    clarion_scpu_can_rx(s->scpu, BCM_ID_ANSWER, s->answer, sizeof(s->answer));
}

static void bcm_sum(uint8_t *d)
{
    d[5] = d[0] + d[1] + d[2] + d[3] + d[4];
}

static void bcm_can_tx(void *opaque, uint16_t id, const uint8_t *d,
                       unsigned len)
{
    ClarionBcmState *s = opaque;
    uint8_t t;

    if (id != BCM_ID_REQUEST || len != 8) {
        return;
    }
    if (!bcm_load_key(s)) {
        return;
    }
    if (d[0] == 0x14 && d[2] == 0xf0 && d[1] < BCM_KEY_SIZE) {
        /* step 1: challenge, answer with the table entry for the seed */
        s->seed = d[1];
        s->seed_valid = true;
        t = s->key[s->seed];
        s->answer[0] = 0x14;
        memset(s->answer + 1, t, 4);
    } else if (d[0] == 0x24 && s->seed_valid) {
        /* step 2: the vehicle code, acknowledged by repeating it */
        memcpy(s->answer, d, 5);
    } else {
        return;
    }
    bcm_sum(s->answer);
    s->answer[6] = 0xff;
    s->answer[7] = 0xff;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              BCM_ANSWER_DELAY_NS);
}

void clarion_bcm_attach(DeviceState *dev, DeviceState *scpu)
{
    ClarionBcmState *s = CLARION_BCM(dev);

    s->scpu = scpu;
    clarion_scpu_set_can_sink(scpu, bcm_can_tx, s);
}

static void bcm_reset_hold(Object *obj, ResetType type)
{
    ClarionBcmState *s = CLARION_BCM(obj);

    s->seed_valid = false;
    timer_del(s->timer);
}

static void bcm_realize(DeviceState *dev, Error **errp)
{
    ClarionBcmState *s = CLARION_BCM(dev);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, bcm_answer_expire, s);
}

static void bcm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = bcm_realize;
    dc->desc = "Clarion QY7221NL synthetic BCM (antitheft)";
    rc->phases.hold = bcm_reset_hold;
}

static const TypeInfo bcm_types[] = {
    {
        .name = TYPE_CLARION_BCM,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(ClarionBcmState),
        .class_init = bcm_class_init,
    },
};

DEFINE_TYPES(bcm_types)
