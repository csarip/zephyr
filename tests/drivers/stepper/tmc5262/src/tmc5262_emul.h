/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * Test-only SPI emulator of the TMC5262, TMC2262 and TMC3241.
 *
 * It models the register file (reset values and register availability per variant), the 40 bit
 * datagram with the pipelined read, the SPI_STATUS byte, write-1-to-clear GSTAT and RAMP_STAT
 * events, IOIN (including the DRV_ENN pin level), the PLL lock handshake and a minimal ramp
 * generator (velocity, standstill, deceleration after a stop). Every register write is logged
 * so that tests can check sequences. Rules of the datasheet that the driver must obey are
 * counted as violations (access to a register the variant does not have, VMAX written in hold
 * mode, TZEROWAIT lowered while the stop target is stale, a write to reserved register bits where
 * the emulator knows them). Fault injection: silent or floating bus, SPI transfer errors, a delayed
 * coil resistance measurement.
 */

#ifndef TESTS_DRIVERS_STEPPER_TMC5262_EMUL_H_
#define TESTS_DRIVERS_STEPPER_TMC5262_EMUL_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/drivers/emul.h>

struct tmc5262_emul_wr {
	uint8_t reg;
	uint32_t val;
};

/** Number of logged writes since the last clear (the log holds the first 256) */
int tmc5262_emul_log_count(const struct emul *e);
void tmc5262_emul_log_clear(const struct emul *e);
const struct tmc5262_emul_wr *tmc5262_emul_log_get(const struct emul *e, int idx);
/** Number of logged writes to @p reg, optionally only those with value @p val (val_any = false) */
int tmc5262_emul_log_find(const struct emul *e, uint8_t reg, bool match_val, uint32_t val,
			  int start);

uint32_t tmc5262_emul_reg_get(const struct emul *e, uint8_t reg);
void tmc5262_emul_reg_set(const struct emul *e, uint8_t reg, uint32_t val);

/** Datasheet rule violations counted by the emulator */
uint32_t tmc5262_emul_invalid_access(const struct emul *e);
uint32_t tmc5262_emul_vmax_in_hold(const struct emul *e);
uint32_t tmc5262_emul_reverse_risk(const struct emul *e);
uint32_t tmc5262_emul_vmax_writes(const struct emul *e);
/** Writes with reserved bits set (UDC_CONF[7:4] on the TMC2262) */
uint32_t tmc5262_emul_reserved_writes(const struct emul *e);

/** Floating bus: SPI replies are all ones */
void tmc5262_emul_set_floating(const struct emul *e, bool floating);
/**
 * SPI transfer errors: after @p after more good transfers every transfer returns @p err
 * (negative errno). err = 0 removes the error.
 */
void tmc5262_emul_io_error(const struct emul *e, int err, int after);
/** Remember the time of the next SPI transfer (the first one after this call) */
void tmc5262_emul_mark_next_transfer(const struct emul *e);
/** Uptime in ms of the transfer marked by tmc5262_emul_mark_next_transfer(), -1 if none yet */
int64_t tmc5262_emul_marked_transfer_ms(const struct emul *e);
/**
 * Coil resistance model: R_COIL reads 0 until @p delay_ms after the last write to T_RCOIL_MEAS,
 * then @p value (R_COIL layout). The time of the first R_COIL read request after the write is
 * kept.
 */
void tmc5262_emul_rcoil_model(const struct emul *e, int delay_ms, uint32_t value);
/** Ms between the last T_RCOIL_MEAS write and the first R_COIL read request, -1 if none */
int64_t tmc5262_emul_rcoil_first_read_ms(const struct emul *e);

/** Sample the SLEEPN pin: low puts the emulated chip to sleep (registers are lost on wake) */
void tmc5262_emul_sync_pins(const struct emul *e);
/** Silent device: SPI replies are all zero */
void tmc5262_emul_set_silent(const struct emul *e, bool silent);
/** Reset the register bank as after a power-up or SLEEPN reset (GSTAT.reset flags set) */
void tmc5262_emul_reset(const struct emul *e);
/** Missing reference resistor (IOIN.ext_res_det = 0) */
void tmc5262_emul_set_no_resistor(const struct emul *e, bool no_resistor);
/** The PLL commit bit never clears (the PLL does not get stable) */
void tmc5262_emul_pll_commit_stuck(const struct emul *e, bool stuck);
/** Number of read requests (first datagram of a read) of a register */
uint32_t tmc5262_emul_read_requests(const struct emul *e, uint8_t reg);
/** Let DRV_STATUS.stst rise now instead of after its detection delay */
void tmc5262_emul_settle(const struct emul *e);
/** Let the next @p n PLL commits end with clk_1m0_tmo and clk_loss set */
void tmc5262_emul_pll_fail_next(const struct emul *e, int n);
int tmc5262_emul_pll_commits(const struct emul *e);

/** Driver fault: GSTAT.drv_err and the given DRV_STATUS bits; persistent faults never clear */
void tmc5262_emul_set_fault(const struct emul *e, uint32_t drv_status_bits, bool persistent);
/**
 * A stall: the StallGuard flag in DRV_STATUS and, on ramp generator variants,
 * RAMP_STAT.event_stop_sg. Like the chip the flag also reads 1 whenever StallGuard is not active
 * (TCOOLTHRS = 0 or TSTEP > TCOOLTHRS), and the event is only generated while it is active.
 */
void tmc5262_emul_set_stall(const struct emul *e, bool stall);
/** TSTEP while the motor runs (20 bit), it reads 0xFFFFF at standstill. The default is 100. */
void tmc5262_emul_set_tstep(const struct emul *e, uint32_t tstep);
/** The motor runs at @p velocity (not standstill, vzero = 0) */
void tmc5262_emul_start_motion(const struct emul *e, int32_t velocity);
/** The move is complete: XACTUAL = XTARGET, standstill, position reached and its event */
void tmc5262_emul_finish_move(const struct emul *e);
/** Number of SPI transfers a deceleration takes after a stop was requested */
void tmc5262_emul_set_decel_transfers(const struct emul *e, int n);
/** Set a latched RAMP_STAT bit (events 4..7, status bits) */
void tmc5262_emul_ramp_stat_set(const struct emul *e, uint32_t bits);
/** Set ENC_STATUS bits (TriCoder / encoder events); persistent bits survive a clear */
void tmc5262_emul_set_enc_status(const struct emul *e, uint32_t bits, bool persistent);
/** Number of SPI transfers handled */
uint32_t tmc5262_emul_transfers(const struct emul *e);
/** Last SPI_STATUS byte the emulator returned */
uint8_t tmc5262_emul_last_spi_status(const struct emul *e);

#endif /* TESTS_DRIVERS_STEPPER_TMC5262_EMUL_H_ */
