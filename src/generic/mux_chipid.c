// The serial mux identity, derived from the hardware chip id
//
// The device half of the serial mux answers a scan with a six-byte identity
// (mux.h mux_set_uuid) and adopts a node id only when the assignment names that
// identity, so a build that speaks the mux needs one before the console runs.
// This file is where it comes from: the same six bytes katapult's CAN uuid is
// (generic/canserial.c canserial_set_uuid - the low six bytes of fasthash64
// over the raw chip id with CAN's seed), so a board answers the same uuid
// whichever wire it is asked on.
//
// The raw chip id is arch specific, and the arch chipid.c files that read it
// are compiled into USB and CAN builds only (src/<arch>/Makefile), which is
// neither of what a UART mux build is. Hence the read lives here, next to the
// mux it feeds.
//
// Copyright (C) 2026  Feng Cheng <i@pfeng.top>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "../lib/fast-hash/fasthash.h" // fasthash64
#include "autoconf.h" // CONFIG_MACH_*
#include "mux.h" // mux_set_uuid, MUX_UUID_LEN
#include "sched.h" // DECL_INIT

#if CONFIG_MACH_STM32
#include "stm32/internal.h" // UID_BASE, through the family's CMSIS header
#define CHIP_UID_LEN 12
#elif CONFIG_MACH_RPXXXX
#include "rp2040/internal.h" // bootrom_read_unique_id
#define CHIP_UID_LEN 8
#elif CONFIG_MACH_LPC176X
#include "generic/irq.h" // irq_disable, irq_enable
// In-application programming interface, command 58 reads the unique id
// (lpc176x/chipid.c)
#define IAP_LOCATION 0x1fff1ff1
#define IAP_CMD_READ_UID 58
#define CHIP_UID_LEN 16
typedef void (*IAP)(uint32_t *, uint32_t *);
#else
#error "The serial mux needs a chip id source for this architecture"
#endif

// CAN's seed, as canserial_set_uuid uses it.
#define MUX_UUID_SEED 0xA16231A7

static void
mux_set_uuid_from_chipid(const void *chip_id, uint32_t chip_id_len)
{
    uint64_t hash = fasthash64(chip_id, chip_id_len, MUX_UUID_SEED);
    uint8_t uuid[MUX_UUID_LEN];
    memcpy(uuid, &hash, MUX_UUID_LEN);
    mux_set_uuid(uuid);
}

void
mux_chipid_init(void)
{
#if CONFIG_MACH_STM32
    mux_set_uuid_from_chipid((void*)UID_BASE, CHIP_UID_LEN);
#elif CONFIG_MACH_RPXXXX
    uint8_t uid[CHIP_UID_LEN] = {};
    bootrom_read_unique_id(uid, sizeof(uid));
    mux_set_uuid_from_chipid(uid, sizeof(uid));
#elif CONFIG_MACH_LPC176X
    uint32_t iap_cmd_uid[5] = {IAP_CMD_READ_UID, 0, 0, 0, 0};
    uint32_t iap_resp[5];
    IAP iap_entry = (IAP)IAP_LOCATION;
    irq_disable();
    iap_entry(iap_cmd_uid, iap_resp);
    irq_enable();
    mux_set_uuid_from_chipid(&iap_resp[1], CHIP_UID_LEN);
#endif
}
DECL_INIT(mux_chipid_init);
