/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi_emul.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "tmc5262_emul.h"
#include "tmc5262_reg.h"

#define EMUL_NREGS   0x80
#define EMUL_LOG_MAX 256
#define PLL_CONFIG                                                                                 \
	(TMC5262_PLL_EXT_NOT_INT | TMC5262_PLL_CLK_SYS_SEL | TMC5262_PLL_CLK_FSM_ENA |             \
	 TMC5262_PLL_CLOCK_DIVIDER)
#define EV_STOP_BITS     (TMC5262_RAMP_STAT_EVENTS)
#define DECEL_DEFAULT    3
/* DRV_STATUS.stst rises this many SPI transfers after the motor stopped */
#define STST_DELAY_XFERS 20

enum emul_variant {
	EMUL_TMC5262,
	EMUL_TMC2262,
	EMUL_TMC3241,
};

struct emul_reg_range {
	uint8_t first;
	uint8_t last;
};

/* Register availability from the datasheet register maps (independent of the driver tables) */
static const struct emul_reg_range common_regs[] = {
	{0x00, 0x02}, {0x04, 0x04}, {0x0A, 0x0B}, {0x10, 0x12}, {0x14, 0x15}, {0x1A, 0x1A},
	{0x38, 0x3C}, {0x58, 0x58}, {0x5A, 0x5A}, {0x60, 0x6D}, {0x6F, 0x6F},
};
static const struct emul_reg_range ramp_regs[] = {
	{0x05, 0x06},
	{0x20, 0x31},
	{0x34, 0x36},
	{0x3E, 0x3F},
};
static const struct emul_reg_range plus_regs[] = {
	{0x13, 0x13}, {0x16, 0x19}, {0x3D, 0x3D}, {0x40, 0x52}, {0x59, 0x59}, {0x70, 0x70},
};

struct tmc5262_emul_cfg {
	uint16_t chipsel;
	enum emul_variant variant;
	struct gpio_dt_spec sleepn;
	struct gpio_dt_spec drv_enn;
};

struct tmc5262_emul_data {
	uint32_t reg[EMUL_NREGS];
	int read_addr;
	int32_t vact;
	bool asleep;
	bool silent;
	bool no_resistor;
	bool pll_fault;
	bool pll_commit_stuck;
	int stst_countdown;
	bool mirror_write;
	uint32_t last_write_val;
	uint32_t read_requests[EMUL_NREGS];
	bool fault_persistent;
	uint32_t enc_persist;
	int pll_fail_next;
	int pll_commits;
	int decel_xfers;
	int decel_countdown;
	uint32_t xfers;
	uint32_t invalid_access;
	uint32_t vmax_in_hold;
	uint32_t reverse_risk;
	uint32_t vmax_writes;
	uint8_t last_status;
	struct tmc5262_emul_wr log[EMUL_LOG_MAX];
	int log_n;
};

static bool in_ranges(const struct emul_reg_range *r, size_t n, uint8_t reg)
{
	for (size_t i = 0; i < n; i++) {
		if (reg >= r[i].first && reg <= r[i].last) {
			return true;
		}
	}

	return false;
}

static bool reg_exists(enum emul_variant v, uint8_t reg)
{
	if (in_ranges(common_regs, ARRAY_SIZE(common_regs), reg)) {
		return true;
	}

	if (v != EMUL_TMC2262 && in_ranges(ramp_regs, ARRAY_SIZE(ramp_regs), reg)) {
		return true;
	}

	return v != EMUL_TMC3241 && in_ranges(plus_regs, ARRAY_SIZE(plus_regs), reg);
}

static bool has_ramp(enum emul_variant v)
{
	return v != EMUL_TMC2262;
}

static void do_reset(const struct tmc5262_emul_cfg *cfg, struct tmc5262_emul_data *d)
{
	memset(d->reg, 0, sizeof(d->reg));

	/* Datasheet reset values */
	d->reg[TMC5262_REG_GCONF] = TMC5262_GCONF_SMALL_HYSTERESIS | TMC5262_GCONF_MULTISTEP_FILT;
	d->reg[TMC5262_REG_GSTAT] = TMC5262_GSTAT_RESET | TMC5262_GSTAT_REGISTER_RESET;
	d->reg[TMC5262_REG_DO_CONF] = TMC5262_DO_CONF_DO0_INVPP | TMC5262_DO_CONF_DO1_INVPP;
	d->reg[TMC5262_REG_DRV_CONF] = 0x3C;
	d->reg[TMC5262_REG_IHOLD_IRUN] = 0x0407F040;
	d->reg[TMC5262_REG_TPOWERDOWN] = 10;
	d->reg[TMC5262_REG_ENC_CONST] = 0x00010000;
	d->reg[TMC5262_REG_OTW_OV_VTH] = 0x01FF01FF;
	d->reg[TMC5262_REG_CHOPCONF] = cfg->variant == EMUL_TMC3241 ? 0x10010000 : 0x10410150;
	d->reg[TMC5262_REG_MSLUTSEL] = 0;

	if (cfg->variant != EMUL_TMC3241) {
		d->reg[TMC5262_REG_T_RCOIL_MEAS] = 0x1000;
		d->reg[TMC5262_REG_UDC_CONF] = 0xCE;
		d->reg[TMC5262_REG_ENCMODE] = 16U << 16;
		d->reg[TMC5262_REG_PWMCONF] = 0x000FE000;
	}

	if (cfg->variant == EMUL_TMC5262) {
		d->reg[TMC5262_REG_SW_MODE] = TMC5262_SW_MODE_HARD_STOP_CLR_CUR_INT;
	}

	d->reg[TMC5262_REG_RAMP_STAT] =
		TMC5262_RAMP_STAT_VZERO | TMC5262_RAMP_STAT_POSITION_REACHED;
	d->vact = 0;
	d->read_addr = -1;
	d->pll_fault = false;
	d->pll_commit_stuck = false;
	d->stst_countdown = 0;
	d->mirror_write = false;
	d->fault_persistent = false;
	d->enc_persist = 0U;
	d->decel_countdown = 0;
}

static const struct tmc5262_emul_cfg *ecfg(const struct emul *e)
{
	return e->cfg;
}

static struct tmc5262_emul_data *edata(const struct emul *e)
{
	return e->data;
}

static int pin_level(const struct gpio_dt_spec *spec)
{
	if (spec->port == NULL) {
		return -1;
	}

	return gpio_emul_output_get(spec->port, spec->pin);
}

void tmc5262_emul_sync_pins(const struct emul *e)
{
	const struct tmc5262_emul_cfg *cfg = ecfg(e);
	struct tmc5262_emul_data *d = edata(e);

	if (pin_level(&cfg->sleepn) == 0) {
		d->asleep = true;
	}
}

static uint8_t spi_status(const struct tmc5262_emul_cfg *cfg, struct tmc5262_emul_data *d)
{
	uint32_t gstat = d->reg[TMC5262_REG_GSTAT];
	uint32_t drv = d->reg[TMC5262_REG_DRV_STATUS];
	uint32_t rs = d->reg[TMC5262_REG_RAMP_STAT];
	uint8_t s = 0;

	/* Table 5 of the datasheet */
	if (d->vact == 0 && d->stst_countdown == 0) {
		s |= BIT(3);
	}
	if ((drv & TMC5262_DRV_STATUS_STALLGUARD) != 0U) {
		s |= BIT(2);
	}
	if ((gstat & TMC5262_GSTAT_DRV_ERR) != 0U) {
		s |= BIT(1);
	}
	if ((gstat & TMC5262_GSTAT_RESET) != 0U) {
		s |= BIT(0);
	}

	if (has_ramp(cfg->variant)) {
		s |= (rs & TMC5262_RAMP_STAT_VELOCITY_REACHED) != 0U ? BIT(4) : 0;
		s |= (rs & TMC5262_RAMP_STAT_POSITION_REACHED) != 0U ? BIT(5) : 0;
		s |= (rs & TMC5262_RAMP_STAT_STATUS_STOP_L) != 0U ? BIT(6) : 0;
		s |= (rs & TMC5262_RAMP_STAT_STATUS_STOP_R) != 0U ? BIT(7) : 0;
	}

	return s;
}

/* Time based behavior, advanced with every SPI transfer */
static void advance(const struct tmc5262_emul_cfg *cfg, struct tmc5262_emul_data *d)
{
	/* Short and overtemperature flags clear when the driver is disabled by DRV_ENN or TOFF = 0
	 */
	bool disabled = pin_level(&cfg->drv_enn) == 1 ||
			(d->reg[TMC5262_REG_CHOPCONF] & TMC5262_CHOPCONF_TOFF) == 0U;

	if (disabled && !d->fault_persistent) {
		d->reg[TMC5262_REG_DRV_STATUS] &=
			~(TMC5262_DRV_STATUS_S2GA | TMC5262_DRV_STATUS_S2GB |
			  TMC5262_DRV_STATUS_S2VSA | TMC5262_DRV_STATUS_S2VSB |
			  TMC5262_DRV_STATUS_OT);
	}

	/* Standstill (DRV_STATUS.stst) is only detected a while after the last step */
	if (d->vact != 0) {
		d->stst_countdown = STST_DELAY_XFERS;
	} else if (d->stst_countdown > 0) {
		d->stst_countdown--;
	}

	if (d->decel_countdown > 0 && --d->decel_countdown == 0) {
		/* The ramp generator overshoots while it decelerates towards a target behind it */
		d->reg[TMC5262_REG_XACTUAL] += d->vact > 0 ? 500U : (uint32_t)-500;
		d->vact = 0;
		d->reg[TMC5262_REG_RAMP_STAT] |= TMC5262_RAMP_STAT_VZERO;
	}
}

static uint32_t reg_read(const struct tmc5262_emul_cfg *cfg, struct tmc5262_emul_data *d,
			 uint8_t reg)
{
	uint32_t v = d->reg[reg];

	switch (reg) {
	case TMC5262_REG_IOIN:
		v = FIELD_PREP(TMC5262_IOIN_SILICON_RV, 2U);
		v |= d->no_resistor ? 0U : TMC5262_IOIN_EXT_RES_DET;
		v |= (d->reg[TMC5262_REG_PLL] & TMC5262_PLL_EXT_NOT_INT) != 0U
			     ? TMC5262_IOIN_EXT_CLK
			     : 0U;
		v |= pin_level(&cfg->drv_enn) == 1 ? TMC5262_IOIN_DRV_ENN : 0U;
		break;
	case TMC5262_REG_DRV_STATUS:
		v &= ~TMC5262_DRV_STATUS_STST;
		v |= (d->vact == 0 && d->stst_countdown == 0) ? TMC5262_DRV_STATUS_STST : 0U;
		break;
	case TMC5262_REG_VACTUAL:
		v = (uint32_t)d->vact & GENMASK(23, 0);
		break;
	default:
		break;
	}

	return v;
}

static void log_write(struct tmc5262_emul_data *d, uint8_t reg, uint32_t val)
{
	if (d->log_n < EMUL_LOG_MAX) {
		d->log[d->log_n].reg = reg;
		d->log[d->log_n].val = val;
	}
	d->log_n++;
}

static void pll_write(struct tmc5262_emul_data *d, uint32_t val)
{
	uint32_t *pll = &d->reg[TMC5262_REG_PLL];

	*pll = (*pll & ~PLL_CONFIG) | (val & PLL_CONFIG);

	if ((val & TMC5262_PLL_CLK_FSM_ENA) == 0U) {
		/* PLL FSM in reset: all flags cleared, the fault state is left */
		*pll &= ~(TMC5262_PLL_FLAGS | TMC5262_PLL_COMMIT);
		d->pll_fault = false;
		return;
	}

	/* Write 1 to clear the flags, a fault re-asserts them */
	*pll &= ~(val & TMC5262_PLL_FLAGS);

	if ((val & TMC5262_PLL_COMMIT) != 0U) {
		d->pll_commits++;
		/* Committing sets clk_loss and clk_is_stuck until they are cleared */
		*pll |= TMC5262_PLL_CLK_LOSS | TMC5262_PLL_CLK_IS_STUCK;
		/* The commit bit clears once the PLL is stable */
		*pll &= ~TMC5262_PLL_COMMIT;
		if (d->pll_commit_stuck) {
			/* The PLL never gets stable: commit stays set */
			*pll |= TMC5262_PLL_COMMIT;
		}

		if (d->pll_fail_next > 0) {
			d->pll_fail_next--;
			d->pll_fault = true;
		}
	}

	if (d->pll_fault) {
		*pll |= TMC5262_PLL_CLK_1M0_TMO | TMC5262_PLL_CLK_LOSS;
	}
}

static void reg_write(const struct tmc5262_emul_cfg *cfg, struct tmc5262_emul_data *d, uint8_t reg,
		      uint32_t val)
{
	uint32_t *r = &d->reg[reg];

	log_write(d, reg, val);

	if (!reg_exists(cfg->variant, reg)) {
		d->invalid_access++;
		return;
	}

	switch (reg) {
	case TMC5262_REG_GSTAT:
		/* Write 1 to clear; the driver error only clears when its cause is gone */
		*r &= ~(val & (d->fault_persistent ? ~TMC5262_GSTAT_DRV_ERR : UINT32_MAX));
		break;
	case TMC5262_REG_PLL:
		pll_write(d, val);
		break;
	case TMC5262_REG_RAMP_STAT:
		*r &= ~(val & EV_STOP_BITS);
		break;
	case TMC5262_REG_ENC_STATUS:
		/* Write 1 to clear; a persisting deviation warning cannot be cleared */
		*r &= ~(val & ~d->enc_persist);
		break;
	case TMC5262_REG_IOIN:
	case TMC5262_REG_DRV_STATUS:
	case TMC5262_REG_VACTUAL:
	case TMC5262_REG_TSTEP:
		break;
	case TMC5262_REG_VMAX:
		d->vmax_writes++;
		if (val != 0U && (d->reg[TMC5262_REG_RAMPMODE] & 3U) == TMC5262_RAMPMODE_HOLD) {
			d->vmax_in_hold++;
		}
		*r = val;
		break;
	case TMC5262_REG_TZEROWAIT:
		/* Lowering TZEROWAIT while the generator stands still with a stale target in
		 * positioning mode lets it drive back.
		 */
		if (val != TMC5262_TZEROWAIT_MASK && d->vact == 0 &&
		    (d->reg[TMC5262_REG_RAMPMODE] & 3U) == TMC5262_RAMPMODE_POSITION &&
		    d->reg[TMC5262_REG_XTARGET] != d->reg[TMC5262_REG_XACTUAL] &&
		    d->reg[TMC5262_REG_TZEROWAIT] == TMC5262_TZEROWAIT_MASK) {
			d->reverse_risk++;
		}
		*r = val;
		break;
	case TMC5262_REG_XTARGET:
		*r = val;
		if ((d->reg[TMC5262_REG_RAMPMODE] & 3U) == TMC5262_RAMPMODE_POSITION) {
			if (val == d->reg[TMC5262_REG_XACTUAL]) {
				if (d->vact != 0 && d->decel_countdown == 0) {
					d->decel_countdown = d->decel_xfers;
				} else if (d->vact == 0) {
					d->reg[TMC5262_REG_RAMP_STAT] |=
						TMC5262_RAMP_STAT_POSITION_REACHED |
						TMC5262_RAMP_STAT_EVENT_POS_REACHED;
				}
			} else if (d->reg[TMC5262_REG_VMAX] != 0U) {
				/* A move starts */
				d->vact = val > d->reg[TMC5262_REG_XACTUAL] ? 1000 : -1000;
				d->reg[TMC5262_REG_RAMP_STAT] &=
					~(TMC5262_RAMP_STAT_VZERO |
					  TMC5262_RAMP_STAT_POSITION_REACHED);
			}
		}
		break;
	case TMC5262_REG_RAMPMODE:
		*r = val & 3U;
		if (*r == TMC5262_RAMPMODE_POSITION &&
		    d->reg[TMC5262_REG_XTARGET] == d->reg[TMC5262_REG_XACTUAL] && d->vact != 0 &&
		    d->decel_countdown == 0) {
			d->decel_countdown = d->decel_xfers;
		} else if (*r == TMC5262_RAMPMODE_VELOCITY_POS ||
			   *r == TMC5262_RAMPMODE_VELOCITY_NEG) {
			if (d->reg[TMC5262_REG_VMAX] != 0U) {
				d->vact = *r == TMC5262_RAMPMODE_VELOCITY_POS ? 1000 : -1000;
				d->reg[TMC5262_REG_RAMP_STAT] &= ~TMC5262_RAMP_STAT_VZERO;
			}
		}
		break;
	default:
		*r = val;
		break;
	}
}

static int emul_io(const struct emul *target, const struct spi_config *config,
		   const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	const struct tmc5262_emul_cfg *cfg = ecfg(target);
	struct tmc5262_emul_data *d = edata(target);
	const uint8_t *tx;
	uint8_t *rx;
	uint32_t resp = 0U;
	uint8_t addr;

	ARG_UNUSED(config);

	if (tx_bufs == NULL || tx_bufs->count != 1 || tx_bufs->buffers[0].len != 5 ||
	    rx_bufs == NULL || rx_bufs->count != 1 || rx_bufs->buffers[0].len != 5) {
		return -EIO;
	}

	tx = tx_bufs->buffers[0].buf;
	rx = rx_bufs->buffers[0].buf;
	d->xfers++;

	if (d->asleep) {
		if (pin_level(&cfg->sleepn) != 0) {
			/* Woken: all registers are back at their defaults */
			do_reset(cfg, d);
			d->asleep = false;
		} else {
			memset(rx, 0, 5);
			return 0;
		}
	}

	if (d->silent) {
		memset(rx, 0, 5);
		return 0;
	}

	advance(cfg, d);

	/* The response carries the register requested by the previous datagram, or the data of
	 * the previous write
	 */
	if (d->read_addr >= 0) {
		resp = reg_read(cfg, d, (uint8_t)d->read_addr);
		d->read_requests[d->read_addr]++;
	} else if (d->mirror_write) {
		resp = d->last_write_val;
	}

	d->last_status = spi_status(cfg, d);
	rx[0] = d->last_status;
	sys_put_be32(resp, &rx[1]);

	addr = tx[0] & 0x7FU;
	if ((tx[0] & 0x80U) != 0U) {
		d->last_write_val = sys_get_be32(&tx[1]);
		d->mirror_write = true;
		reg_write(cfg, d, addr, d->last_write_val);
		d->read_addr = -1;
	} else {
		d->mirror_write = false;
		if (!reg_exists(cfg->variant, addr)) {
			d->invalid_access++;
			d->read_addr = -1;
		} else {
			d->read_addr = addr;
		}
	}

	return 0;
}

static const struct spi_emul_api emul_api = {
	.io = emul_io,
};

static int emul_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(parent);

	do_reset(ecfg(target), edata(target));
	edata(target)->decel_xfers = DECEL_DEFAULT;

	return 0;
}

/* ---- test control API ---- */

int tmc5262_emul_log_count(const struct emul *e)
{
	return edata(e)->log_n;
}

void tmc5262_emul_log_clear(const struct emul *e)
{
	edata(e)->log_n = 0;
}

const struct tmc5262_emul_wr *tmc5262_emul_log_get(const struct emul *e, int idx)
{
	struct tmc5262_emul_data *d = edata(e);

	return (idx >= 0 && idx < MIN(d->log_n, EMUL_LOG_MAX)) ? &d->log[idx] : NULL;
}

int tmc5262_emul_log_find(const struct emul *e, uint8_t reg, bool match_val, uint32_t val,
			  int start)
{
	struct tmc5262_emul_data *d = edata(e);

	for (int i = MAX(start, 0); i < MIN(d->log_n, EMUL_LOG_MAX); i++) {
		if (d->log[i].reg == reg && (!match_val || d->log[i].val == val)) {
			return i;
		}
	}

	return -1;
}

uint32_t tmc5262_emul_reg_get(const struct emul *e, uint8_t reg)
{
	return reg_read(ecfg(e), edata(e), reg);
}

void tmc5262_emul_reg_set(const struct emul *e, uint8_t reg, uint32_t val)
{
	edata(e)->reg[reg] = val;
}

uint32_t tmc5262_emul_invalid_access(const struct emul *e)
{
	return edata(e)->invalid_access;
}

uint32_t tmc5262_emul_vmax_in_hold(const struct emul *e)
{
	return edata(e)->vmax_in_hold;
}

uint32_t tmc5262_emul_reverse_risk(const struct emul *e)
{
	return edata(e)->reverse_risk;
}

uint32_t tmc5262_emul_vmax_writes(const struct emul *e)
{
	return edata(e)->vmax_writes;
}

void tmc5262_emul_set_silent(const struct emul *e, bool silent)
{
	edata(e)->silent = silent;
}

void tmc5262_emul_reset(const struct emul *e)
{
	do_reset(ecfg(e), edata(e));
}

void tmc5262_emul_set_no_resistor(const struct emul *e, bool no_resistor)
{
	edata(e)->no_resistor = no_resistor;
}

void tmc5262_emul_pll_commit_stuck(const struct emul *e, bool stuck)
{
	edata(e)->pll_commit_stuck = stuck;
}

uint32_t tmc5262_emul_read_requests(const struct emul *e, uint8_t reg)
{
	return edata(e)->read_requests[reg];
}

void tmc5262_emul_settle(const struct emul *e)
{
	edata(e)->stst_countdown = 0;
}

void tmc5262_emul_pll_fail_next(const struct emul *e, int n)
{
	edata(e)->pll_fail_next = n;
}

int tmc5262_emul_pll_commits(const struct emul *e)
{
	return edata(e)->pll_commits;
}

void tmc5262_emul_set_fault(const struct emul *e, uint32_t drv_status_bits, bool persistent)
{
	struct tmc5262_emul_data *d = edata(e);

	d->reg[TMC5262_REG_GSTAT] |= TMC5262_GSTAT_DRV_ERR;
	d->reg[TMC5262_REG_DRV_STATUS] |= drv_status_bits;
	d->fault_persistent = persistent;
}

void tmc5262_emul_set_stall(const struct emul *e, bool stall)
{
	const struct tmc5262_emul_cfg *cfg = ecfg(e);
	struct tmc5262_emul_data *d = edata(e);

	if (stall) {
		d->reg[TMC5262_REG_DRV_STATUS] |= TMC5262_DRV_STATUS_STALLGUARD;
		if (has_ramp(cfg->variant)) {
			d->reg[TMC5262_REG_RAMP_STAT] |= TMC5262_RAMP_STAT_EVENT_STOP_SG;
		}
	} else {
		d->reg[TMC5262_REG_DRV_STATUS] &= ~TMC5262_DRV_STATUS_STALLGUARD;
	}
}

void tmc5262_emul_start_motion(const struct emul *e, int32_t velocity)
{
	struct tmc5262_emul_data *d = edata(e);

	d->vact = velocity;
	d->reg[TMC5262_REG_RAMP_STAT] &=
		~(TMC5262_RAMP_STAT_VZERO | TMC5262_RAMP_STAT_POSITION_REACHED);
}

void tmc5262_emul_finish_move(const struct emul *e)
{
	struct tmc5262_emul_data *d = edata(e);

	d->reg[TMC5262_REG_XACTUAL] = d->reg[TMC5262_REG_XTARGET];
	d->vact = 0;
	d->decel_countdown = 0;
	d->stst_countdown = STST_DELAY_XFERS;
	d->reg[TMC5262_REG_RAMP_STAT] |= TMC5262_RAMP_STAT_VZERO |
					 TMC5262_RAMP_STAT_POSITION_REACHED |
					 TMC5262_RAMP_STAT_EVENT_POS_REACHED;
}

void tmc5262_emul_set_decel_transfers(const struct emul *e, int n)
{
	edata(e)->decel_xfers = n;
}

void tmc5262_emul_ramp_stat_set(const struct emul *e, uint32_t bits)
{
	edata(e)->reg[TMC5262_REG_RAMP_STAT] |= bits;
}

void tmc5262_emul_set_enc_status(const struct emul *e, uint32_t bits, bool persistent)
{
	struct tmc5262_emul_data *d = edata(e);

	d->reg[TMC5262_REG_ENC_STATUS] |= bits;
	d->enc_persist = persistent ? bits : 0U;
}

uint32_t tmc5262_emul_transfers(const struct emul *e)
{
	return edata(e)->xfers;
}

uint8_t tmc5262_emul_last_spi_status(const struct emul *e)
{
	return edata(e)->last_status;
}

/* ---- instantiation ---- */

#define TMC5262_EMUL(n, _variant)                                                                  \
	static struct tmc5262_emul_data emul_data_##_variant##_##n;                                \
	static const struct tmc5262_emul_cfg emul_cfg_##_variant##_##n = {                         \
		.chipsel = DT_INST_REG_ADDR(n),                                                    \
		.variant = _variant,                                                               \
		.sleepn = GPIO_DT_SPEC_INST_GET_OR(n, sleepn_gpios, {0}),                          \
		.drv_enn = GPIO_DT_SPEC_INST_GET_OR(n, drv_enn_gpios, {0}),                        \
	};                                                                                         \
	EMUL_DT_INST_DEFINE(n, emul_init, &emul_data_##_variant##_##n, &emul_cfg_##_variant##_##n, \
			    &emul_api, NULL)

#define DT_DRV_COMPAT adi_tmc5262
#define EMUL_5262(n)  TMC5262_EMUL(n, EMUL_TMC5262)
DT_INST_FOREACH_STATUS_OKAY(EMUL_5262)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT adi_tmc2262
#define EMUL_2262(n)  TMC5262_EMUL(n, EMUL_TMC2262)
DT_INST_FOREACH_STATUS_OKAY(EMUL_2262)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT adi_tmc3241
#define EMUL_3241(n)  TMC5262_EMUL(n, EMUL_TMC3241)
DT_INST_FOREACH_STATUS_OKAY(EMUL_3241)
#undef DT_DRV_COMPAT
