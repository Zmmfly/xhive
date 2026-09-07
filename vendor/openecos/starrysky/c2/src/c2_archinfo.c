/* See works/docs/periphs/archinfo.md. No assumed C2 silicon reset values. */
#include "c2_archinfo.h"

static bool decode_bcd(uint32_t value, unsigned int digits, uint32_t *decoded)
{
    uint32_t result = 0u;
    uint32_t scale = 1u;
    unsigned int i;
    for (i = 0u; i < digits; ++i) {
        uint32_t digit = value & 15u;
        if (digit > 9u) {
            return false;
        }
        result += digit * scale;
        value >>= 4;
        scale *= 10u;
    }
    *decoded = result;
    return true;
}

c2_status_t c2_archinfo_read(C2_ARCHINFO_TypeDef *reg,
                             c2_archinfo_snapshot_t *snapshot)
{
    if (reg == NULL || snapshot == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    snapshot->sys = c2_mmio_read32(&reg->SYS.WORD);
    snapshot->idl = c2_mmio_read32(&reg->IDL.WORD);
    snapshot->idh = c2_mmio_read32(&reg->IDH.WORD);
    return C2_OK;
}

c2_status_t c2_archinfo_write(C2_ARCHINFO_TypeDef *reg,
                              const c2_archinfo_snapshot_t *snapshot)
{
    if (reg == NULL || snapshot == NULL ||
        (snapshot->sys & UINT32_C(0xfff00000)) != 0u ||
        (snapshot->idh & UINT32_C(0xff000000)) != 0u) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    c2_mmio_write32(&reg->SYS.WORD, snapshot->sys);
    c2_mmio_write32(&reg->IDL.WORD, snapshot->idl);
    c2_mmio_write32(&reg->IDH.WORD, snapshot->idh);
    return C2_OK;
}

c2_status_t c2_archinfo_decode(const c2_archinfo_snapshot_t *snapshot,
                               c2_archinfo_info_t *info)
{
    uint32_t clock;
    uint32_t process;
    uint32_t date;
    if (snapshot == NULL || info == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    if (!decode_bcd((snapshot->sys >> 8) & 0xfffu, 3u, &clock) ||
        !decode_bcd((snapshot->idl >> 6) & 0xffffu, 4u, &process) ||
        !decode_bcd(snapshot->idh & 0xffffffu, 6u, &date)) {
        return C2_ERROR_IO;
    }
    info->clock_code = (uint16_t)clock;
    info->process_nm = (uint16_t)process;
    info->year = (uint16_t)(date / 100u);
    info->month = (uint8_t)(date % 100u);
    info->sram_kb = (uint8_t)snapshot->sys;
    info->vendor = (uint8_t)(snapshot->idl >> 22);
    info->type = (uint8_t)(snapshot->idl >> 30);
    info->custom = (uint8_t)(snapshot->idl & 63u);
    return C2_OK;
}
