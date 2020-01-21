#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <helper/types.h>

#define VCM4_VERSION_ID 0x4004803C

/* vcm4 flash csr register */
enum vcm4_fcsr_registers {
    FCSR_BASE = 0x40020000,

#define FCSR_REG(offset) (FCSR_BASE + offset)

    FCSR_FLASH_CMD         = FCSR_REG(0x000),  // flash controller command register
    FCSR_FLASH_ADDR        = FCSR_REG(0x004),  // flash controller address regsiter
    FCSR_FLASH_CFG         = FCSR_REG(0x008),  // flash controller configuration register
    FCSR_FLASH_CACHE       = FCSR_REG(0x00C),  // flash controller cache configuration register
    FCSR_FLASH_SR          = FCSR_REG(0x010),  // flash controller SPI flash status register
    FCSR_FLASH_ID          = FCSR_REG(0x014),  // flash controller SPI flash ID register
    FCSR_FLASH_CACHEHIT    = FCSR_REG(0x018),  // cache hit rate counting register
    FCSR_FLASH_INVADDR_S   = FCSR_REG(0x020),  // invalid/flush cache start address
    FCSR_FLASH_INVADDR_E   = FCSR_REG(0x024),  // invalid/flush cache end address
    FCSR_FLASH_CACHE_INV   = FCSR_REG(0x028),  // invalid cache control register
    FCSR_FLASH_CACHE_FLUSH = FCSR_REG(0x02C),  // flush cache control register
    FCSR_FLASH_BUF0        = FCSR_REG(0x100),  // flash controller read/write buffer 0
    FCSR_FLASH_BUF63       = FCSR_REG(0x1FC),  // flash controller read/write buffer 63
};

/* vcm4 flash bit-fields */
#define FLASH_CMD_ACT_Pos (31)
#define FLASH_CMD_ACT_Msk (0x1 << FLASH_CMD_ACT_Pos)

#define FLASH_CMD_POLL_Pos (24)
#define FLASH_CMD_POLL_Msk (0x1 << FLASH_CMD_POLL_Pos)

#define FLASH_CMD_LENGTH_Pos (16)
#define FLASH_CMD_LENGTH_Msk (0xFF << FLASH_CMD_LENGTH_Pos)

#define FLASH_CMD_CMDMODE_Pos (12)
#define FLASH_CMD_CMDMODE_Msk (0x7 << FLASH_CMD_CMDMODE_Pos)

#define FLASH_CMD_CMDADDR4_Pos (11)
#define FLASH_CMD_CMDADDR4_Msk (0x1 << FLASH_CMD_CMDADDR4_Pos)

#define FLASH_CMD_CMDADDR_Pos (10)
#define FLASH_CMD_CMDADDR_Msk (0x1 << FLASH_CMD_CMDADDR_Pos)

#define FLASH_CMD_CMDWR_Pos (9)
#define FLASH_CMD_CMDWR_Msk (0x1 << FLASH_CMD_CMDWR_Pos)

#define FLASH_CMD_CMDDATA_Pos (8)
#define FLASH_CMD_CMDDATA_Msk (0x1 << FLASH_CMD_CMDDATA_Pos)

#define FLASH_CMD_CMDID_Pos (0)
#define FLASH_CMD_CMDID_Msk (0xFF << FLASH_CMD_CMDID_Pos)

#define WINBOND_CMDID_WRSR1             0x01    // write status register 1
#define WINBOND_CMDID_WRSR2             0x31    // write status register 2
#define WINBOND_CMDID_RDSR1             0x05    // read status register 1
#define WINBOND_CMDID_RDSR2             0x35    // read status register 2
#define WINBOND_CMDID_PAGE_PROG         0x02    // page program
#define WINBOND_CMDID_READ_DATA         0x03    // read data
#define WINBOND_CMDID_WRITE_DISABLE     0x04    // write disable
#define WINBOND_CMDID_WRITE_ENABLE      0x06    // write enable
#define WINBOND_CMDID_FAST_READ         0x0B    // fast read
#define WINBOND_CMDID_RDCR              0x15    // read configuration register
#define WINBOND_CMDID_SECTOR_ERASE      0x20    // sector erase
#define WINBOND_CMDID_QUADPAGE_PROG     0x32    // quad page program
#define WINBOND_CMDID_BLOCK_ERASE_32K   0x52    // 32K block erase
#define WINBOND_CMDID_BLOCK_ERASE_64K   0xD8    // 64K block erase
#define WINBOND_CMDID_CHIP_ERASE        0xC7    // chip erase

#define WINBOND_MF  0xEF

struct vcm4_info {
    uint32_t code_page_size;
    bool probed;
    struct target *target;
};

struct vcm4_device_spec {
    uint32_t version_id;
    const char *variant;
    uint8_t sector_size_kb;
    unsigned int flash_size_kb;
};

static const struct vcm4_device_spec vcm4_known_device_table[] = {
    {
        .version_id = 0x19061001,
        .variant = "phoenix",
        .sector_size_kb = 4,
        .flash_size_kb = 2048,
    },
};

static bool vcm4_bank_is_probed(struct flash_bank *bank)
{
    struct vcm4_info *chip = bank->driver_priv;
    assert(chip != NULL);
    return chip->probed;
}

static int vcm4_probe(struct flash_bank *bank);

static int vcm4_get_probed_chip_if_halted(struct flash_bank *bank, struct vcm4_info **chip)
{
    if (bank->target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    *chip = bank->driver_priv;

    int probed = vcm4_bank_is_probed(bank);

    if (probed < 0) {
        return probed;
    } else if (!probed) {
        return vcm4_probe(bank);
    } else {
        return ERROR_OK;
    }
}

static int vcm4_flash_wait_for_status_idle(struct vcm4_info *chip, int timeout)
{
    int res = ERROR_OK;
    uint32_t temp = 0;
    uint32_t counter = timeout;

    do {
        res = target_read_u32(chip->target, FCSR_FLASH_SR, &temp);
        if (res != ERROR_OK) {
            LOG_ERROR("can't read FLASH SR register");
            return res;
        }
        if ((temp & 0x1) == 0) {
            return ERROR_OK;
        }
        alive_sleep(1);
    } while (counter--);

    LOG_WARNING("timeout waiting for FLASH STATUS idle");
    return ERROR_FLASH_BUSY;
}

static int vcm4_flash_wait_for_action_done(struct vcm4_info *chip, int timeout)
{
    int res = ERROR_OK;
    uint32_t temp = 0;
    uint32_t counter = timeout;

    do {
        res = target_read_u32(chip->target, FCSR_FLASH_CMD, &temp);
        if (res != ERROR_OK) {
            LOG_ERROR("can't read FLASH CMD register");
            return res;
        }
        if ((temp & FLASH_CMD_ACT_Msk) == 0) {
            return ERROR_OK;
        }
        alive_sleep(1);
    } while (counter--);

    LOG_WARNING("timeout waiting for FLASH ACTION done");
    return ERROR_FLASH_BUSY;
}

static int vcm4_flash_set_cache(struct vcm4_info *chip, int enable)
{
    int res = ERROR_OK;

    uint32_t temp = 0;

    res = target_read_u32(chip->target, FCSR_FLASH_CACHE, &temp);

    if (res != ERROR_OK) {
        LOG_ERROR("failed to read FLASH CACHE register");
        return res;
    }

    if (enable) {
        temp |= (1 << 0);
    } else {
        temp &= ~(1 << 0);
    }

    res = target_write_u32(chip->target, FCSR_FLASH_CACHE, temp);

    return res;
}

static int vcm4_flash_config(struct vcm4_info *chip)
{
    int res = ERROR_OK;

    uint32_t temp = 0;

    // config clock to run gppll 150 mhz

#if 0
    res = target_read_u32(chip->target, 0x4004b004, &temp); // ANA_RG_GPPLL_CTRL0
    if (res != ERROR_OK) {
        LOG_ERROR("failed to read RG_GPPLL_CTRL0 register");
        return res;
    }

    // set CTRL0 GPPLL POSDIV divided by 4
    temp &= ~(0x3 << 5);
    temp |= (0x2 << 5);

    // set CTRL0 GPPLL EN
    temp |= (0x1 << 15);

    res = target_write_u32(chip->target, 0x4004b004, temp);
    if (res != ERROR_OK) {
        LOG_ERROR("failed to write RG_GPPLL_CTRL0 register");
        return res;
    }

    res = target_read_u32(chip->target, 0x4004b008, &temp); // ANA_RG_GPPLL_CTRL1
    if (res != ERROR_OK) {
        LOG_ERROR("failed to read RG_GPPLL_CTRL1 register");
        return res;
    }

    // set CTRL1 DDSM_IN fraction to 0
    temp &= ~(0xfffff << 0);

    // set CTRL1 DDSM_IN integer to 0x18
    temp &= ~(0x3f << 20);
    temp |= (0x18 << 20);

    // set CTRL1 DDSM EN
    temp |= (0x1 << 26);

    res = target_write_u32(chip->target, 0x4004b008, temp);
    if (res != ERROR_OK) {
        LOG_ERROR("failed to write RG_GPPLL_CTRL1 register");
        return res;
    }

    alive_sleep(100);

    // config AHB source to GPLL

    res = target_read_u32(chip->target, 0x40047004, &temp); // MISC2_CLKSEL
    if (res != ERROR_OK) {
        LOG_ERROR("failed to read MISC2_CLKSEL register");
        return res;
    }

    temp &= ~(0x3 << 0);
    temp |= (0x2 << 0); // GPLL clock source

    res = target_write_u32(chip->target, 0x40047004, temp);
    if (res != ERROR_OK) {
        LOG_ERROR("failed to write MISC2_CLKSEL register");
        return res;
    }
#endif

    // config SPIFLASH

    res = target_read_u32(chip->target, FCSR_FLASH_CFG, &temp);
    if (res != ERROR_OK) {
        LOG_ERROR("failed to read FLASH CFG register");
        return res;
    }

    // set flash size to 16 MBIT
    temp &= ~(0x7 << 8);
    temp |= (0x5 << 8);

    // set cmd latency to 1
    temp &= ~(0x3 << 16);
    temp |= (0x1 << 16);

    // set mem latency to 1
    temp &= ~(0x3 << 18);
    temp |= (0x1 << 18);

    // set memmode to 4I4O
    temp &= ~(0x7 << 12);
    temp |= (0x5 << 12);

    // set cmd clk div 3
    temp &= ~(0xF << 0);
    temp |= (0x2 << 0);

    // set mem clk div 2
    temp &= ~(0xF << 4);
    temp |= (0x1 << 4);

    res = target_write_u32(chip->target, FCSR_FLASH_CFG, temp);

    return res;
}

static int vcm4_flash_write_enable(struct vcm4_info *chip)
{
    int res = ERROR_OK;
    uint32_t temp = 0;

    temp |= (1 << FLASH_CMD_ACT_Pos);
    temp |= (1 << FLASH_CMD_POLL_Pos);
    temp |= (WINBOND_CMDID_WRITE_ENABLE << FLASH_CMD_CMDID_Pos);

    res = target_write_u32(chip->target, FCSR_FLASH_CMD, temp);

    res = vcm4_flash_wait_for_action_done(chip, 100);

    return res;
}

static uint8_t vcm4_flash_read_manu_id(struct vcm4_info *chip)
{
    int res = ERROR_OK;
    uint32_t temp = 0;

    res = target_read_u32(chip->target, FCSR_FLASH_ID, &temp);

    if (res != ERROR_OK) {
        LOG_ERROR("can't read FLASH ID register");
        return 0;
    }

    return (uint8_t)(temp >> 16);
}

static int vcm4_flash_quad_enable(struct vcm4_info *chip)
{
    int res = ERROR_OK;
    uint32_t temp = 0;
    uint32_t data = 0;

    res = vcm4_flash_wait_for_status_idle(chip, 100);
    if (res != ERROR_OK) {
        return res;
    }

    res = vcm4_flash_write_enable(chip);
    if (res != ERROR_OK) {
        return res;
    }

    if (vcm4_flash_read_manu_id(chip) == WINBOND_MF) {

        temp |= (1 << FLASH_CMD_ACT_Pos);
        temp |= (1 << FLASH_CMD_CMDWR_Pos);
        temp |= (1 << FLASH_CMD_CMDDATA_Pos);
        temp |= (1 << FLASH_CMD_POLL_Pos);
        temp |= (WINBOND_CMDID_WRSR2 << FLASH_CMD_CMDID_Pos);

        data = 0x02;

        res = target_write_u32(chip->target, FCSR_FLASH_BUF0, data);
        if (res != ERROR_OK) {
            LOG_ERROR("failed tor write FLASH BUF0 register");
            return res;
        }

        res = target_write_u32(chip->target, FCSR_FLASH_CMD, temp);
        if (res != ERROR_OK) {
            LOG_ERROR("failed to write FLASH CMD register");
            return res;
        }

        res = vcm4_flash_wait_for_status_idle(chip, 100);

    } else {

        temp |= (1 << FLASH_CMD_ACT_Pos);
        temp |= (1 << FLASH_CMD_CMDWR_Pos);
        temp |= (1 << FLASH_CMD_CMDDATA_Pos);
        temp |= (1 << FLASH_CMD_POLL_Pos);
        temp |= (1 << FLASH_CMD_LENGTH_Pos);
        temp |= (WINBOND_CMDID_WRSR1 << FLASH_CMD_CMDID_Pos);

        data |= (0x00 << 0); // byte 0
        data |= (0x02 << 8); // byte 1

        res = target_write_u32(chip->target, FCSR_FLASH_BUF0, data);
        if (res != ERROR_OK) {
            LOG_ERROR("failed to write FLASH BUF0 register");
            return res;
        }

        res = target_write_u32(chip->target, FCSR_FLASH_CMD, temp);
        if (res != ERROR_OK) {
            LOG_ERROR("failed to write FLASH CMD register");
            return res;
        }

        res = vcm4_flash_wait_for_action_done(chip, 100);
    }

    return res;
}

static int vcm4_flash_write_status_register(struct vcm4_info *chip, uint8_t cmdid, uint32_t data, uint8_t length)
{
    int res = ERROR_OK;
    uint32_t temp = 0;

    res = vcm4_flash_wait_for_status_idle(chip, 100);
    if (res != ERROR_OK) {
        return res;
    }

    res = vcm4_flash_write_enable(chip);
    if (res != ERROR_OK) {
        return res;
    }

    temp |= (1 << FLASH_CMD_ACT_Pos);
    temp |= (1 << FLASH_CMD_CMDWR_Pos);
    temp |= (1 << FLASH_CMD_CMDDATA_Pos);
    temp |= (1 << FLASH_CMD_POLL_Pos);

    if (length != 0) {
        temp |= (length << FLASH_CMD_LENGTH_Pos);
    }

    temp |= (cmdid << FLASH_CMD_CMDID_Pos);

    res = target_write_u32(chip->target, FCSR_FLASH_BUF0, data);
    if (res != ERROR_OK) {
        LOG_ERROR("failed to write FLASH BUF0 register");
        return res;
    }

    res = target_write_u32(chip->target, FCSR_FLASH_CMD, temp);
    if (res != ERROR_OK) {
        return res;
    }

    res = vcm4_flash_wait_for_status_idle(chip, 100);

    return res;
}

static int vcm4_flash_wprot_disable(struct vcm4_info *chip)
{
    int res = ERROR_OK;

    if (vcm4_flash_read_manu_id(chip) == WINBOND_MF) {
        res = vcm4_flash_write_status_register(chip, WINBOND_CMDID_WRSR2, 0x2, 0);
        if (res != ERROR_OK) {
            return res;
        }
        res = vcm4_flash_write_status_register(chip, WINBOND_CMDID_WRSR1, 0x0, 0);
        if (res != ERROR_OK) {
            return res;
        }
    } else {
        uint32_t data = 0;

        data |= (0x0 << 0); // byte 0
        data |= (0x2 << 8); // byte 1

        res = vcm4_flash_write_status_register(chip, WINBOND_CMDID_WRSR1, data, 1);
        if (res != ERROR_OK) {
            return res;
        }
    }

    return res;
}

static int vcm4_flash_chip_erase(struct vcm4_info *chip)
{
    int res = ERROR_OK;
    uint32_t temp = 0;

    res = vcm4_flash_write_enable(chip);
    if (res != ERROR_OK) {
        return res;
    }

    temp |= (1 << FLASH_CMD_ACT_Pos);
    temp |= (1 << FLASH_CMD_POLL_Pos);
    temp |= (1 << FLASH_CMD_CMDWR_Pos);
    temp |= (WINBOND_CMDID_CHIP_ERASE << FLASH_CMD_CMDID_Pos);

    res = target_write_u32(chip->target, FCSR_FLASH_CMD, temp);
    if (res != ERROR_OK) {
        return res;
    }

    res = vcm4_flash_wait_for_action_done(chip, 2000);

    return res;
}

static int vcm4_flash_sector_erase(struct vcm4_info *chip, uint32_t addr)
{
    int res = ERROR_OK;
    uint32_t temp = 0;

    res = vcm4_flash_write_enable(chip);
    if (res != ERROR_OK) {
        return res;
    }

    temp |= (1 << FLASH_CMD_ACT_Pos);
    temp |= (1 << FLASH_CMD_POLL_Pos);
    temp |= (1 << FLASH_CMD_CMDADDR_Pos);
    temp |= (1 << FLASH_CMD_CMDWR_Pos);
    temp |= (WINBOND_CMDID_SECTOR_ERASE << FLASH_CMD_CMDID_Pos);

    res = target_write_u32(chip->target, FCSR_FLASH_ADDR, addr);
    if (res != ERROR_OK) {
        LOG_ERROR("failed to write FLASH ADDR register");
        return res;
    }

    res = target_write_u32(chip->target, FCSR_FLASH_CMD, temp);
    if (res != ERROR_OK) {
        LOG_ERROR("failed to write FLASH CMD register");
        return res;
    }

    res = vcm4_flash_wait_for_action_done(chip, 1000);

    return res;
}

static int vcm4_flash_program_word(struct vcm4_info *chip, uint32_t addr, uint32_t data)
{
    int res = ERROR_OK;
    uint32_t temp = 0;

    res = vcm4_flash_write_enable(chip);
    if (res != ERROR_OK) {
        return res;
    }

    temp |= (1 << FLASH_CMD_ACT_Pos);
    temp |= (1 << FLASH_CMD_POLL_Pos);
    temp |= (1 << FLASH_CMD_CMDADDR_Pos);
    temp |= (1 << FLASH_CMD_CMDWR_Pos);
    temp |= (1 << FLASH_CMD_CMDDATA_Pos);
    temp |= (3 << FLASH_CMD_LENGTH_Pos); // 1 word (4 bytes) - 1
    temp |= (WINBOND_CMDID_PAGE_PROG << FLASH_CMD_CMDID_Pos);

    res = target_write_u32(chip->target, FCSR_FLASH_BUF0, data);
    if (res != ERROR_OK) {
        return res;
    }

    res = target_write_u32(chip->target, FCSR_FLASH_ADDR, addr);
    if (res != ERROR_OK) {
        return res;
    }

    res = target_write_u32(chip->target, FCSR_FLASH_CMD, temp);
    if (res != ERROR_OK) {
        return res;
    }

    res = vcm4_flash_wait_for_action_done(chip, 100);

    return res;
}

#if 0
static int vcm4_flash_program_page(struct vcm4_info *chip, uint32_t addr, uint32_t len, uint8_t *buf)
{
    int res = ERROR_OK;

    uint32_t i = 0;
    uint32_t temp = 0;
    uint32_t data;

    res = vcm4_flash_write_enable(chip);
    if (res != ERROR_OK) {
        return res;
    }

    uint32_t counter = len / 4;
    uint32_t remain = len % 4;

    for (i = 0; i < (counter * 4); i += 4) {
        memcpy(&data, &buf[i], sizeof(uint32_t));
        res = target_write_u32(chip->target, FCSR_FLASH_BUF0 + i, data);
        if (res != ERROR_OK) {
            return res;
        }
    }

    data = 0;

    if (remain) {
        for (i = 0; i < remain; i++) {
            data |= (buf[(counter * 4) + i] << (i * 8));
        }
        res = target_write_u32(chip->target, FCSR_FLASH_BUF0 + counter, data);
        if (res != ERROR_OK) {
            return res;
        }
    }

    temp |= (1 << FLASH_CMD_ACT_Pos);
    temp |= (1 << FLASH_CMD_POLL_Pos);
    temp |= (1 << FLASH_CMD_CMDADDR_Pos);
    temp |= (1 << FLASH_CMD_CMDWR_Pos);
    temp |= (1 << FLASH_CMD_CMDDATA_Pos);

    if (len) {
        temp |= ((len - 1) << FLASH_CMD_LENGTH_Pos);
    }

    temp |= (WINBOND_CMDID_PAGE_PROG << FLASH_CMD_CMDID_Pos);

    res = target_write_u32(chip->target, FCSR_FLASH_CMD, temp);
    if (res != ERROR_OK) {
        return res;
    }

    res = vcm4_flash_wait_for_action_done(chip, 1000);

    return res;
}
#endif

static int vcm4_protect_check(struct flash_bank *bank)
{
    int res = ERROR_OK;
    struct vcm4_info *chip = bank->driver_priv;
    assert(chip != NULL);
    // TODO
    return res;
}

static int vcm4_protect(struct flash_bank *bank, int set, int first, int last)
{
    int res = ERROR_OK;
    struct vcm4_info *chip = bank->driver_priv;
    assert(chip != NULL);
    // TODO
    return res;
}

static int vcm4_probe(struct flash_bank *bank)
{
    int res = ERROR_OK;
    struct vcm4_info *chip = bank->driver_priv;
    uint32_t version_id;

    res = target_read_u32(chip->target, VCM4_VERSION_ID, &version_id);
    if (res != ERROR_OK) {
        LOG_ERROR("can't read VERSION ID register");
        return res;
    }

    const struct vcm4_device_spec *spec = NULL;
    for (size_t i = 0; i < ARRAY_SIZE(vcm4_known_device_table); i++) {
        if (version_id == vcm4_known_device_table[i].version_id) {
            spec = &vcm4_known_device_table[i];
        }
    }

    if (!chip->probed) {
        if (spec) {
            LOG_INFO("vcm4-%s: %uKB flash", spec->variant, spec->flash_size_kb);
        } else {
            LOG_ERROR("unknown device version id (0x%08"PRIx32")", version_id);
            res = ERROR_FLASH_BANK_NOT_PROBED;
            return res;
        }
    }

    bank->size = spec->flash_size_kb * 0x400;
    bank->num_sectors = (spec->flash_size_kb / spec->sector_size_kb);
    bank->sectors = calloc(bank->num_sectors, sizeof((bank->sectors)[0]));

    chip->code_page_size = spec->sector_size_kb * 0x400;

    if (!bank->sectors) {
        res = ERROR_FLASH_BANK_NOT_PROBED;
        return res;
    }

    for (int i = 0; i < bank->num_sectors; i++) {
        bank->sectors[i].size = chip->code_page_size;
        bank->sectors[i].offset = i * chip->code_page_size;
        /* mark as unknown */
        bank->sectors[i].is_erased = -1;
        bank->sectors[i].is_protected = -1;
    }

    vcm4_protect_check(bank);

    chip->probed = true;

    vcm4_flash_set_cache(chip, 0); // disable cache

    vcm4_flash_config(chip);
    vcm4_flash_quad_enable(chip);

    vcm4_flash_wprot_disable(chip);

    vcm4_flash_set_cache(chip, 1); // enable cache

    // dump register
    uint32_t temp;

    res = target_read_u32(chip->target, FCSR_FLASH_CFG, &temp);

    LOG_INFO("FCSR_FLASH_CFG: 0x%08"PRIx32, temp);

    return res;
}

static int vcm4_auto_probe(struct flash_bank *bank)
{
    int probed = vcm4_bank_is_probed(bank);

    if (probed < 0) {
        return probed;
    } else if (probed) {
        return ERROR_OK;
    } else {
        return vcm4_probe(bank);
    }
}

static struct flash_sector *vcm4_find_sector_by_address(struct flash_bank *bank, uint32_t address)
{
    struct vcm4_info *chip = bank->driver_priv;

    for (int i = 0; i < bank->num_sectors; i++) {
        if ((bank->sectors[i].offset <= address) &&
            (address < (bank->sectors[i].offset + chip->code_page_size))) {
            return &bank->sectors[i];
        }
    }

    return NULL;
}

static int vcm4_erase_page(struct flash_bank *bank,
                           struct vcm4_info *chip,
                           struct flash_sector *sector)
{
    int res = ERROR_OK;

    if (sector->is_protected) {
        LOG_ERROR("cannot erase protected sector at 0x%"PRIx32, sector->offset);
        res = ERROR_FAIL;
        return res;
    }

    res = vcm4_flash_sector_erase(chip, sector->offset);

    if (res == ERROR_OK) {
        sector->is_erased = 1;
    }

    return res;
}

/* start a low level flash write for the specified region */
static int vcm4_flash_write(struct vcm4_info *chip, uint32_t offset, const uint8_t *buffer, uint32_t bytes)
{
    int res = ERROR_OK;

	LOG_INFO("writing buffer to flash offset=0x%"PRIx32" bytes=0x%"PRIx32, offset, bytes);

    assert(bytes % 4 == 0);

    LOG_WARNING("no working area available, falling back to slow memory writes");

    for (; bytes > 0; bytes -= 4) {
        uint32_t value;
        memcpy(&value, buffer, sizeof(uint32_t));
        vcm4_flash_program_word(chip, offset, value);
        offset += 4;
        buffer += 4;
    }

    return res;
}

/* check and erase flash sectors in specified range then start a low level page
 * write. start/end must be sector aligned.
 */
static int vcm4_write_pages(struct flash_bank *bank, uint32_t start, uint32_t end, const uint8_t *buffer)
{
    int res = ERROR_OK;
    struct vcm4_info *chip = bank->driver_priv;
    struct flash_sector *sector;
    uint32_t offset;

    assert(start % chip->code_page_size == 0);
    assert(end % chip->code_page_size == 0);

    /* erase all sectors */
    for (offset = start; offset < end; offset += chip->code_page_size) {
        sector = vcm4_find_sector_by_address(bank, offset);
        if (!sector) {
            LOG_ERROR("invalid sector at 0x%08"PRIx32, offset);
            return ERROR_FLASH_SECTOR_INVALID;
        }
        if (sector->is_protected) {
            LOG_ERROR("can't erase protected sector at 0x%08"PRIx32, offset);
            return ERROR_FAIL;
        }
        if (sector->is_erased != 1) {
            res = vcm4_erase_page(bank, chip, sector);
            if (res != ERROR_OK) {
                LOG_ERROR("failed to erase sector at 0x%08"PRIx32, sector->offset);
                return res;
            }
        }
        sector->is_erased = 0;
    }

    res = vcm4_flash_write(chip, start, buffer, (end - start));

    return res;
}

static int vcm4_erase(struct flash_bank *bank, int first, int last)
{
    int res;
    struct vcm4_info *chip;

    res = vcm4_get_probed_chip_if_halted(bank, &chip);
    if (res != ERROR_OK) {
        return res;
    }

    for (int s = first; s <= last && res == ERROR_OK; s++) {
        res = vcm4_erase_page(bank, chip, &bank->sectors[s]);
    }

    return res;
}

static int vcm4_code_flash_write(struct flash_bank *bank,
                                 struct vcm4_info *chip,
                                 const uint8_t *buffer, uint32_t offset, uint32_t count)
{
    int res;

    /* need to perform reads to fill any gaps we need to preserve in the first
     * page before the start of buffer, or in the last page, after the end of
     * buffer */

    uint32_t first_page = offset / chip->code_page_size;
    uint32_t last_page = DIV_ROUND_UP(offset + count, chip->code_page_size);

    uint32_t first_page_offset = first_page * chip->code_page_size;
    uint32_t last_page_offset = last_page * chip->code_page_size;

	LOG_DEBUG("Padding write from 0x%08"PRIx32"-0x%08"PRIx32" as 0x%08"PRIx32"-0x%08"PRIx32,
		      offset, offset+count, first_page_offset, last_page_offset);

	uint32_t page_cnt = last_page - first_page;
	uint8_t buffer_to_flash[page_cnt * chip->code_page_size];

	/* Fill in any space between start of first page and start of buffer */
	uint32_t pre = offset - first_page_offset;
	if (pre > 0) {
		res = target_read_memory(bank->target,
					first_page_offset,
					1,
					pre,
					buffer_to_flash);
		if (res != ERROR_OK)
			return res;
	}

	/* Fill in main contents of buffer */
	memcpy(buffer_to_flash+pre, buffer, count);

	/* Fill in any space between end of buffer and end of last page */
	uint32_t post = last_page_offset - (offset+count);
	if (post > 0) {
		/* Retrieve the full row contents from Flash */
		res = target_read_memory(bank->target,
					offset + count,
					1,
					post,
					buffer_to_flash+pre+count);
		if (res != ERROR_OK)
			return res;
	}

    return vcm4_write_pages(bank, first_page_offset, last_page_offset, buffer_to_flash);
}

static int vcm4_write(struct flash_bank *bank, const uint8_t *buffer,
                      uint32_t offset, uint32_t count)
{
    int res;
    struct vcm4_info *chip;

    res = vcm4_get_probed_chip_if_halted(bank, &chip);
    if (res != ERROR_OK) {
        return res;
    }

    return vcm4_code_flash_write(bank, chip, buffer, offset, count);
}

FLASH_BANK_COMMAND_HANDLER(vcm4_flash_bank_command)
{
    static struct vcm4_info *chip;

    if (bank->base != 0x00000000) {
        LOG_ERROR("invalid bank address 0x%08"PRIx32, bank->base);
        return ERROR_FAIL;
    }

    if (!chip) {
        /* create a new chip */
        chip = calloc(1, sizeof(*chip));
        if (!chip) {
            return ERROR_FAIL;
        }
        chip->target = bank->target;
    }

    chip->probed = false;
    bank->driver_priv = chip;
    
    return ERROR_OK;
}

COMMAND_HANDLER(vcm4_handle_mass_erase_command)
{
    struct flash_bank *bank = NULL;
    struct target *target = get_current_target(CMD_CTX);
    int res;

    res = get_flash_bank_by_addr(target, 0x00000000, true, &bank);
    if (res != ERROR_OK) {
        LOG_ERROR("failed to get flash bank");
        return res;
    }

    assert(bank != NULL);

    LOG_INFO("get flash bank base: 0x%08"PRIx32, bank->base);

    struct vcm4_info *chip = bank->driver_priv;

    // chip erase
    res = vcm4_flash_chip_erase(chip);
    if (res != ERROR_OK) {
        LOG_ERROR("failed to erase the chip");
        vcm4_protect_check(bank);
        return res;
    }

    for (int i = 0; i < bank->num_sectors; i++) {
        bank->sectors[i].is_erased = 1;
    }

    res = vcm4_protect_check(bank);

    return res;
}

static int vcm4_get_info(struct flash_bank *bank, char *buf, int buf_size)
{
    int res;

    struct vcm4_info *chip;

    res = vcm4_get_probed_chip_if_halted(bank, &chip);
    if (res != ERROR_OK) {
        return res;
    }

    // TODO

    return res;
}

static const struct command_registration vcm4_exec_command_handlers[] = {
    {
        .name    = "mass_erase",
        .handler = vcm4_handle_mass_erase_command,
        .mode    = COMMAND_EXEC,
        .help    = "Erase all flash content of the chip.",
    },
    COMMAND_REGISTRATION_DONE
};

static const struct command_registration vcm4_command_handlers[] = {
    {
        .name = "vcm4",
        .mode = COMMAND_ANY,
        .help = "vcm4 flash command group",
        .usage = "",
        .chain = vcm4_exec_command_handlers,
    },
    COMMAND_REGISTRATION_DONE
};

struct flash_driver vcm4_flash = {
    .name               = "vcm4",
    .commands           = vcm4_command_handlers,
    .flash_bank_command = vcm4_flash_bank_command,
    .info               = vcm4_get_info,
    .erase              = vcm4_erase,
    .protect            = vcm4_protect,
    .write              = vcm4_write,
    .read               = default_flash_read,
    .probe              = vcm4_probe,
    .auto_probe         = vcm4_auto_probe,
    .erase_check        = default_flash_blank_check,
    .protect_check      = vcm4_protect_check,
};
