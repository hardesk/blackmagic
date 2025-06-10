/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2024 hardesk <hardesk@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "buffer_utils.h"
#include "jep106.h"
#include "cortex.h"

#define MSPM0_CONFIG_FLASH_DUMP_SUPPORT (CONFIG_BMDA == 1 || ENABLE_DEBUG == 1)

#define TI_DEVID_MSPM0C           0xbba1U /* MSPM0C110[34] */
#define TI_DEVID_MSPM0L           0xbb82U /* MSPM0L110[56], MSPM0L13[04][456] */
#define TI_DEVID_MSPM0L_1227_2228 0xbb9fU /* MSPM0L[12]22[78]*/
#define TI_DEVID_MSPM0G           0xbb88U /* MSPM0G310[567], MSPM0G150[567], MSPM0G350[567] */

#define MSPM0_SRAM_BASE       0x20000000U
#define MSPM0_FLASH_MAIN      0x00000000U
#define MSPM0_FLASH_NONMAIN   0x41c00000U /* One Sector, BANK0. Device boot configuration (BCR, BSL) */
#define MSPM0_FLASH_FACTORY   0x41c40000U /* One Sector, BANK0. Non modifiable */
#define MSPM0_FLASH_DATA      0x41d00000U
#define MSPM0_FLASH_SECTOR_SZ 1024U

#define MSPM0_FACTORYREGION_DEVICEID  (MSPM0_FLASH_FACTORY + 0x4U)
#define MSPM0_FACTORYREGION_SRAMFLASH (MSPM0_FLASH_FACTORY + 0x18U)

#define MSPM0_DEVICEID_MANUFACTURER_MASK  0x00000ffeU
#define MSPM0_DEVICEID_MANUFACTURER_SHIFT 1U
#define MSPM0_DEVICEID_PARTNUM_MASK       0x0ffff000U
#define MSPM0_DEVICEID_PARTNUM_SHIFT      12U

#define MSPM0_FACTORYREGION_SRAMFLASH_MAINFLASH_SZ_MASK  0x00000fffU
#define MSPM0_FACTORYREGION_SRAMFLASH_MAINFLASH_SZ_SHIFT 0U
#define MSPM0_FACTORYREGION_SRAMFLASH_MAINNUMBANKS_MASK  0x00003000U
#define MSPM0_FACTORYREGION_SRAMFLASH_MAINNUMBANKS_SHIFT 12U
#define MSPM0_FACTORYREGION_SRAMFLASH_SRAM_SZ_MASK       0x03ff0000U
#define MSPM0_FACTORYREGION_SRAMFLASH_SRAM_SZ_SHIFT      16U
#define MSPM0_FACTORYREGION_SRAMFLASH_DATAFLASH_SZ_MASK  0xfc0000U
#define MSPM0_FACTORYREGION_SRAMFLASH_DATAFLASH_SZ_SHIFT 26U

#define MSPM0_FLASHCTL_BASE              0x400cd000U
#define MSPM0_FLASHCTL_CMDEXEC           (MSPM0_FLASHCTL_BASE + 0x1100U)
#define MSPM0_FLASHCTL_CMDTYPE           (MSPM0_FLASHCTL_BASE + 0x1104U)
#define MSPM0_FLASHCTL_CMDCTL            (MSPM0_FLASHCTL_BASE + 0x1108U)
#define MSPM0_FLASHCTL_CMDADDR           (MSPM0_FLASHCTL_BASE + 0x1120U)
#define MSPM0_FLASHCTL_BYTEN             (MSPM0_FLASHCTL_BASE + 0x1124U)
#define MSPM0_FLASHCTL_CMDDATAIDX        (MSPM0_FLASHCTL_BASE + 0x112cU)
#define MSPM0_FLASHCTL_STATCMD           (MSPM0_FLASHCTL_BASE + 0x13d0U)
#define MSPM0_FLASHCTL_CMDDATA0          (MSPM0_FLASHCTL_BASE + 0x1130U)
#define MSPM0_FLASHCTL_CMDDATA1          (MSPM0_FLASHCTL_BASE + 0x1134U)
#define MSPM0_FLASHCTL_CMDDATA2          (MSPM0_FLASHCTL_BASE + 0x1138U)
#define MSPM0_FLASHCTL_CMDDATA3          (MSPM0_FLASHCTL_BASE + 0x113cU)
#define MSPM0_FLASHCTL_CMDWEPROTA        (MSPM0_FLASHCTL_BASE + 0x11d0U)
#define MSPM0_FLASHCTL_CMDWEPROTB        (MSPM0_FLASHCTL_BASE + 0x11d4U)
#define MSPM0_FLASHCTL_CMDWEPROTC        (MSPM0_FLASHCTL_BASE + 0x11d8U)
#define MSPM0_FLASHCTL_CMDWEPROTNM       (MSPM0_FLASHCTL_BASE + 0x1210U)
#define MSPM0_FLASHCTL_CMDTYPE_NOOP      0U
#define MSPM0_FLASHCTL_CMDTYPE_PROG      1U
#define MSPM0_FLASHCTL_CMDTYPE_ERASE     2U
#define MSPM0_FLASHCTL_CMDTYPE_RDVERIFY  3U
#define MSPM0_FLASHCTL_CMDTYPE_BLVERIFY  6U
#define MSPM0_FLASHCTL_CMDTYPE_SZ_1WORD  (0U << 4U)
#define MSPM0_FLASHCTL_CMDTYPE_SZ_2WORDS (1U << 4U)
#define MSPM0_FLASHCTL_CMDTYPE_SZ_4WORDS (2U << 4U)
#define MSPM0_FLASHCTL_CMDTYPE_SZ_8WORDS (3U << 4U)
#define MSPM0_FLASHCTL_CMDTYPE_SZ_SECTOR (4U << 4U)
#define MSPM0_FLASHCTL_CMDTYPE_SZ_BANK   (5U << 4U)
#define MSPM0_FLASHCTL_CMDEXEC_EXEC      1U
#define MSPM0_FLASHCTL_STAT_DONE         0x01U
#define MSPM0_FLASHCTL_STAT_CMDPASS      0x02U

typedef struct mspm0_flash {
	target_flash_s target_flash;
	uint32_t banks;
} mspm0_flash_s;

static bool mspm0_flash_erase(target_flash_s *flash, target_addr_t addr, size_t length);
static bool mspm0_flash_write(target_flash_s *flash, target_addr_t dest, const void *src, size_t length);
static bool mspm0_mass_erase(target_s *target, platform_timeout_s *print_progess);

static bool poll_mspm0(target_s *const target, const int argc, const char **const argv);

#if MSPM0_CONFIG_FLASH_DUMP_SUPPORT
static bool mspm0_dump_factory_config(target_s *target, int argc, const char **argv);
static bool mspm0_dump_bcr_config(target_s *target, int argc, const char **argv);

static command_s mspm0_cmds_list[] = {
	{"dump_factory", mspm0_dump_factory_config, "Display FACTORY registers"},
	{"dump_bcr", mspm0_dump_bcr_config, "Display NONMAIN (BCR/BSL) registers"},
	{NULL, NULL, NULL},
};

typedef struct conf_register {
	uint16_t reg_offset;
	uint16_t size_words;
	const char *id;
} conf_register_s;

static conf_register_s mspm0_factory_regs[] = {
	{0x00U, 1U, "TRACEID"},
	{0x04U, 1U, "DEVICEID"},
	{0x08U, 1U, "USERID"},
	{0x0cU, 1U, "BSLPIN_UART"},
	{0x10U, 1U, "BSLPIN_I2C"},
	{0x14U, 1U, "BSLPIN_INVOKE"},
	{0x18U, 1U, "SRAMFLASH"},
	{0x3cU, 1U, "TEMP_SENSE0"},
	{0x7cU, 1U, "BOOTCRC"},
	{0U, 0U, NULL},
};

static conf_register_s mspm0_bcr_regs[] = {
	{0x00U, 1U, "BCRCONFIGID"},
	{0x04U, 1U, "BOOTCFG0"},
	{0x08U, 1U, "BOOTCFG1"},
	{0x0cU, 4U, "PWDDEBUGLOCK"},
	{0x1cU, 4U, "BOOTCFG2"},
	{0x20U, 1U, "BOOTCFG3"},
	{0x24U, 4U, "PWDMASSERASE"},
	{0x34U, 4U, "PWDFACTORYRESET"},
	{0x44U, 1U, "FLASHSWP0"},
	{0x48U, 1U, "FLASHSWP1"},
	{0x4cU, 1U, "BOOTCFG4"},
	{0x50U, 1U, "APPCRCSTART"},
	{0x54U, 1U, "APPCRCLENGTH"},
	{0x58U, 1U, "APPCRC"},
	{0x5cU, 1U, "BOOTCRC"},
	{0x100U, 1U, "BSLCONFIGID"},
	{0x104U, 1U, "BSLPINCFG0"},
	{0x108U, 1U, "BSLPINCFG1"},
	{0x10cU, 1U, "BSLCONFIG0"},
	{0x110U, 8U, "BSLPW"},
	{0x130U, 1U, "BSLPLUGINCFG"},
	{0x134U, 4U, "BSLPLUGINHOOK"},
	{0x144U, 1U, "PATCHHOOKID"},
	{0x148U, 1U, "SBLADDRESS"},
	{0x14cU, 1U, "BSLAPPVER"},
	{0x150U, 1U, "BSLCONFIG1"},
	{0x154U, 1U, "BSLCRC"},
	{0U, 0U, NULL},
};

static void mspm0_dump_regs(target_s *const target, const conf_register_s *regs, uint32_t base)
{
	for (const conf_register_s *reg = regs; reg->id; ++reg) {
		tc_printf(target, "%15s: ", reg->id);
		for (size_t i = 0; i < reg->size_words; ++i) {
			uint32_t value = target_mem32_read32(target, base + reg->reg_offset + i * 4U);
			tc_printf(target, "0x%08" PRIx32 "%s", value, i == reg->size_words - 1U ? "\n" : " ");
		}
	}
}

static bool mspm0_dump_factory_config(target_s *const target, const int argc, const char **const argv)
{
	(void)argc;
	(void)argv;
	mspm0_dump_regs(target, mspm0_factory_regs, MSPM0_FLASH_FACTORY);
	return true;
}

static bool mspm0_dump_bcr_config(target_s *const target, const int argc, const char **const argv)
{
	(void)argc;
	(void)argv;
	mspm0_dump_regs(target, mspm0_bcr_regs, MSPM0_FLASH_NONMAIN);
	return true;
}
#endif

static void mspm0_add_flash(target_s *const target, const uint32_t base, const size_t length, const uint32_t banks)
{
	mspm0_flash_s *const flash = calloc(1, sizeof(*flash));
	if (flash == NULL) {
		DEBUG_WARN("%s: calloc failed for %" PRIu32 " bytes\n", __func__, (uint32_t)length);
		return;
	}

	flash->banks = banks;
	target_flash_s *target_flash = &flash->target_flash;
	target_flash->start = base;
	target_flash->length = length;
	target_flash->blocksize = MSPM0_FLASH_SECTOR_SZ;
	target_flash->writesize = 8U;
	target_flash->erase = mspm0_flash_erase;
	target_flash->write = mspm0_flash_write;
	target_flash->erased = 0xffU;
	target_add_flash(target, target_flash);
}

bool mspm0_probe(target_s *const target)
{
	const uint32_t deviceid = target_mem32_read32(target, MSPM0_FACTORYREGION_DEVICEID);

	const uint32_t manufacturer = (deviceid & MSPM0_DEVICEID_MANUFACTURER_MASK) >> MSPM0_DEVICEID_MANUFACTURER_SHIFT;
	if (manufacturer != JEP106_MANUFACTURER_TEXAS)
		return false;

	const uint32_t partnum = (deviceid & MSPM0_DEVICEID_PARTNUM_MASK) >> MSPM0_DEVICEID_PARTNUM_SHIFT;
	if (partnum != TI_DEVID_MSPM0C && partnum != TI_DEVID_MSPM0L && partnum != TI_DEVID_MSPM0L_1227_2228 &&
		partnum != TI_DEVID_MSPM0G)
		return false;

	target->driver = "MSPM0";
	target->target_options |= TOPT_INHIBIT_NRST;
	target->mass_erase = mspm0_mass_erase;

	const uint32_t sramflash = target_mem32_read32(target, MSPM0_FACTORYREGION_SRAMFLASH);
	const uint32_t mainflash_size = 1024U *
		((sramflash & MSPM0_FACTORYREGION_SRAMFLASH_MAINFLASH_SZ_MASK) >>
			MSPM0_FACTORYREGION_SRAMFLASH_MAINFLASH_SZ_SHIFT);
	const uint32_t main_num_banks = (sramflash & MSPM0_FACTORYREGION_SRAMFLASH_MAINNUMBANKS_MASK) >>
		MSPM0_FACTORYREGION_SRAMFLASH_MAINNUMBANKS_SHIFT;
	const uint32_t sram_size = 1024U *
		((sramflash & MSPM0_FACTORYREGION_SRAMFLASH_SRAM_SZ_MASK) >> MSPM0_FACTORYREGION_SRAMFLASH_SRAM_SZ_SHIFT);
	const uint32_t dataflash_size = 1024U *
		((sramflash & MSPM0_FACTORYREGION_SRAMFLASH_DATAFLASH_SZ_MASK) >>
			MSPM0_FACTORYREGION_SRAMFLASH_DATAFLASH_SZ_SHIFT);

	target_add_ram32(target, MSPM0_SRAM_BASE, sram_size);
	mspm0_add_flash(target, MSPM0_FLASH_MAIN, mainflash_size, main_num_banks);
	if (dataflash_size != 0)
		mspm0_add_flash(target, MSPM0_FLASH_DATA, dataflash_size, 1U);

#if MSPM0_CONFIG_FLASH_DUMP_SUPPORT
	target_add_commands(target, mspm0_cmds_list, "MSPM0");
#endif

	return true;
}

/* Wait for flash command to finish and return the status word or UINT32_MAX if timout */
static uint32_t mspm0_flash_wait_done(target_s *const target)
{
	platform_timeout_s timeout;
	platform_timeout_set(&timeout, 500U);

	uint32_t status = 0U;
	while (!(status & MSPM0_FLASHCTL_STAT_DONE)) {
		status = target_mem32_read32(target, MSPM0_FLASHCTL_STATCMD);
		if (platform_timeout_is_expired(&timeout))
			return 0U;
	};

	return status;
}

static void mspm0_flash_unprotect(target_flash_s *const target_flash)
{
	target_mem32_write32(target_flash->t, MSPM0_FLASHCTL_CMDWEPROTA, 0U);
	target_mem32_write32(target_flash->t, MSPM0_FLASHCTL_CMDWEPROTB, 0U);
	target_mem32_write32(target_flash->t, MSPM0_FLASHCTL_CMDWEPROTC, 0U);
}

static void mspm0_flash_unprotect_sector(target_flash_s *const target_flash, const target_addr_t addr)
{
	mspm0_flash_s *mspm0_flash = (mspm0_flash_s *)target_flash;
	uint32_t sector = (addr - target_flash->start) / MSPM0_FLASH_SECTOR_SZ;

	if (sector < 32U) { /* One sector per bit */
		uint32_t mask = ~(1U << sector);
		target_mem32_write32(target_flash->t, MSPM0_FLASHCTL_CMDWEPROTA, mask);
	} else if (sector < 256U) { /* 8 sectors per bit */
		/* When main flash is single bank, PROTB covers sectors starting after PROTA which is 32k. In multibank case
		 * PROTB bits overlap PROTA and starts at sector 0. */
		uint32_t start_protb_sector = mspm0_flash->banks > 1U ? 0U : 32U;
		uint32_t mask = ~(1U << ((sector - start_protb_sector) >> 3U));
		target_mem32_write32(target_flash->t, MSPM0_FLASHCTL_CMDWEPROTB, mask);
	} else { /* 8 sectors per bit, starts at sector 256 */
		uint32_t mask = ~(1U << ((sector - 256U) >> 3U));
		target_mem32_write32(target_flash->t, MSPM0_FLASHCTL_CMDWEPROTC, mask);
	}
}

static bool mspm0_flash_erase(target_flash_s *const target_flash, const target_addr_t addr, const size_t length)
{
#ifdef DEBUG_TARGET_IS_NOOP
	(void)length;
#endif

	target_s *const target = target_flash->t;

	DEBUG_INFO("%s: Erasing flash addr %08" PRIx32 " length %08" PRIx32 "\n", __func__, addr, (uint32_t)length);

	mspm0_flash_unprotect_sector(target_flash, addr);
	target_mem32_write32(
		target, MSPM0_FLASHCTL_CMDTYPE, MSPM0_FLASHCTL_CMDTYPE_SZ_SECTOR | MSPM0_FLASHCTL_CMDTYPE_ERASE);
	target_mem32_write32(target, MSPM0_FLASHCTL_BYTEN, 0xffffffffU);
	target_mem32_write32(target, MSPM0_FLASHCTL_CMDCTL, 0U);
	target_mem32_write32(target, MSPM0_FLASHCTL_CMDADDR, addr);
	target_mem32_write32(target, MSPM0_FLASHCTL_CMDEXEC, MSPM0_FLASHCTL_CMDEXEC_EXEC);

	const uint32_t status = mspm0_flash_wait_done(target);
	if (!(status & MSPM0_FLASHCTL_STAT_CMDPASS))
		DEBUG_TARGET("%s: Failed to erase flash, status %08" PRIx32 " addr %08" PRIx32 " length %08" PRIx32 "\n",
			__func__, status, addr, (uint32_t)length);

	return status & MSPM0_FLASHCTL_STAT_CMDPASS;
}

static bool mspm0_flash_write(
	target_flash_s *const target_flash, target_addr_t dest, const void *const src, const size_t length)
{
#ifdef DEBUG_TARGET_IS_NOOP
	(void)length;
#endif

	target_s *const target = target_flash->t;

	mspm0_flash_unprotect_sector(target_flash, dest);
	target_mem32_write32(target, MSPM0_FLASHCTL_CMDTYPE, MSPM0_FLASHCTL_CMDTYPE_PROG | MSPM0_FLASHCTL_CMDTYPE_SZ_1WORD);
	target_mem32_write32(target, MSPM0_FLASHCTL_CMDCTL, 0U);
	target_mem32_write32(target, MSPM0_FLASHCTL_CMDADDR, dest);
	target_mem32_write32(target, MSPM0_FLASHCTL_BYTEN, 0xffffffffU);
	target_mem32_write32(target, MSPM0_FLASHCTL_CMDDATA0, read_le4((const uint8_t *)src, 0U));
	target_mem32_write32(target, MSPM0_FLASHCTL_CMDDATA1, read_le4((const uint8_t *)src, 4U));
	target_mem32_write32(target, MSPM0_FLASHCTL_CMDEXEC, MSPM0_FLASHCTL_CMDEXEC_EXEC);

	const uint32_t status = mspm0_flash_wait_done(target);
	if (!(status & MSPM0_FLASHCTL_STAT_CMDPASS))
		DEBUG_TARGET("%s: Failed to write to flash, status %08" PRIx32 " addr %08" PRIx32 " length %08" PRIx32 "\n",
			__func__, status, dest, (uint32_t)length);

	return status & MSPM0_FLASHCTL_STAT_CMDPASS;
}

static bool mspm0_mass_erase(target_s *target, platform_timeout_s *print_progess)
{
	bool success = true;
	for (mspm0_flash_s *flash = (mspm0_flash_s *)target->flash; flash && success;
		 flash = (mspm0_flash_s *)flash->target_flash.next) {
		/* Assume banks are of same size */
		uint32_t bank_size = flash->target_flash.length / flash->banks;
		for (uint32_t bank = 0U; bank < flash->banks; ++bank) {
			uint32_t bank_offset = bank * bank_size;
			uint32_t bank_address = flash->target_flash.start + bank_offset;
			DEBUG_INFO("%s: Mass erase flash bank starting %08" PRIx32 " length %08" PRIx32 "\n", __func__,
				bank_address, bank_size);

			mspm0_flash_unprotect(&flash->target_flash);
			target_mem32_write32(
				target, MSPM0_FLASHCTL_CMDTYPE, MSPM0_FLASHCTL_CMDTYPE_SZ_BANK | MSPM0_FLASHCTL_CMDTYPE_ERASE);
			target_mem32_write32(target, MSPM0_FLASHCTL_CMDCTL, 0U);
			target_mem32_write32(target, MSPM0_FLASHCTL_CMDADDR, bank_address);
			target_mem32_write32(target, MSPM0_FLASHCTL_CMDEXEC, MSPM0_FLASHCTL_CMDEXEC_EXEC);

			uint32_t status = 0U;
			while (status & MSPM0_FLASHCTL_STAT_DONE) {
				status = target_mem32_read32(target, MSPM0_FLASHCTL_STATCMD);
				if (print_progess)
					target_print_progress(print_progess);
			}

			if (!(status & MSPM0_FLASHCTL_STAT_CMDPASS))
				DEBUG_TARGET("%s: Failed to mass erase flash, status %08" PRIx32 " start %08" PRIx32 " length "
							 "%08" PRIx32 "\n",
					__func__, status, bank_address, (uint32_t)bank_size);

			success &= (status & MSPM0_FLASHCTL_STAT_CMDPASS) == MSPM0_FLASHCTL_STAT_CMDPASS;
		}
	}

	return success;
}

#define DEBUGSS_BASE			0x400C7000U
#define DEBUGSS_TXD				(DEBUGSS_BASE + 0x1100U)
#define DEBUGSS_TXCTL			(DEBUGSS_BASE + 0x1104U)
#define DEBUGSS_RXD 			(DEBUGSS_BASE + 0x1108U)
#define DEBUGSS_RXCTL 			(DEBUGSS_BASE + 0x110cU)
#define DEBUGSS_SPECIAL_AUTH 	(DEBUGSS_BASE + 0x1200U)
#define DEBUGSS_APP_AUTH 		(DEBUGSS_BASE + 0x1210U)

#include "timing.h"

// struct platform_timeout mspm0_poll_timeout = {0};
// static bool poll_mspm0(target_s *const cur_target, const int argc, const char **const argv)
// {
// 	while(1) {
// 		// TX data (debug probe to target device)
// 		// RX data (target device to debug probe)
// 		if (platform_timeout_is_expired(&mspm0_poll_timeout)) {
// 			platform_timeout_set(&mspm0_poll_timeout, 1000);
// 			debug_info("%s\n", cur_target ? "'" : "*");
// 		}

// 		if (target_mem32_read32(cur_target, DEBUGSS_RXCTL) & 1) {
// 			uint32_t data = target_mem32_read32(cur_target, DEBUGSS_RXD);
// 			debug_info("RXD: %c (%u)\n", (char)data, data);
// 		}
// 		sleep(1);
// 	}
// }

#define TI_SECAP_IDR 0x002e0000U
#define TI_SEC_AP_TXD			ADIV5_AP_REG(0x0u)
#define TI_SEC_AP_TXCTL			ADIV5_AP_REG(0x4u)
#define TI_SEC_AP_RXD			ADIV5_AP_REG(0x8u)
#define TI_SEC_AP_RXCTL			ADIV5_AP_REG(0xcu)

static bool poll_mspm0(target_s *const target, const int argc, const char **const argv);

static command_s mspm0_sec_ap_cmds_list[] = {
	{"poll", poll_mspm0, "Poll DEBUGSS channel"},
	{NULL, NULL, NULL},
};


static void mspm0_sec_ap_free(void *priv);
bool mspm0_sec_ap_probe(adiv5_access_port_s *ap)
{
	if (ap->idr != TI_SECAP_IDR)
		return false;

	target_s *target = target_new();
	if (!target)
		return false;

	adiv5_ap_ref(ap);
	target->priv = ap;
	target->priv_free = mspm0_sec_ap_free;

	target->driver = "MSPM0 Debug Mailbox";
	target->regs_size = 0;
	target->mass_erase = NULL;

	debug_info("SEC-AP: idr %08x apsel %d class %d type %d\n", ap->idr, ap->apsel,
		ADIV5_AP_IDR_CLASS(ap->idr), ADIV5_AP_IDR_TYPE(ap->idr) );

	// for (unsigned i=0;i<0x10; i += 4) {
	// 	uint32_t r = adiv5_ap_read(ap, 0x100 | i );
	// 	debug_info("SEC-AP: read @0x%02x: %08x\n", i, r);
	// }

	target_add_commands(target, mspm0_sec_ap_cmds_list, "MSPM0 Mailbox");

	return true;
}

static void mspm0_sec_ap_free(void *priv)
{
	adiv5_ap_unref(priv);
}

static struct platform_timeout mspm0_poll_timeout = {0};
static char const s_progress[] = "-/|\\";
static uint8_t s_progress_n = 0; 

static void mspm0_listen_mailbox(adiv5_access_port_s *ap)
{
	platform_timeout_set(&mspm0_poll_timeout, 500);

	bool progress_printed = false;
	unsigned recvd = 0;
	char buffer[256+1];
	size_t recv_max = sizeof(buffer)-1-4;
	unsigned state = 0; // 0 - wait, 1 - recv
	uint32_t rxctl, rxd, len;
	while(1) {
		// TX data (debug probe to target device)
		// RX data (target device to debug probe)
		if (platform_timeout_is_expired(&mspm0_poll_timeout)) {
			platform_timeout_set(&mspm0_poll_timeout, 500);
			// debug_info("%s", ap ? "'" : "*");
			debug_info("%s%c", progress_printed ? "\r" : "", s_progress[s_progress_n++ % (sizeof s_progress - 1)]);
			debug_flush(BMD_DEBUG_INFO);
			progress_printed = true;
		}

		do {
			switch(state) {
			default:
			case 0: // waiting
				rxctl = adiv5_ap_read(ap, TI_SEC_AP_RXCTL);
				if (rxctl & 1) {
					state = 1;
					recvd = 0;
					len = (rxctl & 0x7fu) >> 1;
					platform_timeout_set(&mspm0_poll_timeout, 500);
					goto read_data;
				}
				break;

			case 1: // reading
				rxctl = adiv5_ap_read(ap, TI_SEC_AP_RXCTL);
				if (rxctl & 1) {
read_data:
					rxd = adiv5_ap_read(ap, TI_SEC_AP_RXD);
					// debug_info("RXCTL: %08x RXD: %08x\n", rxctl, rxd);
					unsigned recvd_now = MIN(4, len);
					memcpy(buffer + recvd, &rxd, recvd_now);
					len -= recvd_now;
					recvd += recvd_now;

					if (!len || recvd >= recv_max) {
						buffer[recvd] = 0;
						if (progress_printed) progress_printed = false, debug_info("%s", "  \r");
						debug_info("%s", buffer);
						debug_flush(BMD_DEBUG_INFO);
						if (!len)
							state = 0; // done with the message
						recvd = 0;
					}
				}
				break;
			} // switch
		} while(rxctl & 1);

		usleep(10000u);
	}
}

static bool poll_mspm0(target_s *const cur_target, const int argc, const char **const argv)
{
	(void)argc;
	(void)argv;
	adiv5_access_port_s* ap = (adiv5_access_port_s*)cur_target->priv;
	mspm0_listen_mailbox(ap);
	return true;
}
