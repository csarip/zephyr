/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Cherrence Sarip <Cherrence.sarip@analog.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_STEPPER_TMC522X_H_
#define ZEPHYR_DRIVERS_STEPPER_TMC522X_H_

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/stepper.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name TMC522X SPI Communication Constants
 * @{
 */
#define TMC522X_WRITE_BIT    0x80U
#define TMC522X_ADDRESS_MASK 0x7FU
/** @} */

/**
 * @name TMC522X/TMC5222 Configuration Registers
 * @{
 */
#define TMC522X_REG_GCONF        0x00
#define TMC522X_REG_GSTAT        0x01
#define TMC522X_REG_NODECONF     0x03
#define TMC522X_REG_IOIN         0x04
#define TMC522X_REG_DRVCONF      0x05
#define TMC522X_REG_GLOBALSCALER 0x06
#define TMC522X_REG_DIAG_CONF    0x07
/** @} */

/**
 * @name TMC522X GCONF register bits
 * @{
 */

/**
 * ═══════════════════════════════════════════════════════════════════════════
 * SpreadCycle vs StealthChop2 - Comprehensive Comparison
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * The TMC522x provides TWO fundamentally different chopper modes for motor
 * current control. Selection is controlled by the EN_PWM_MODE bit in GCONF.
 *
 * ┌──────────────────────┬────────────────────────┬────────────────────────┐
 * │ Feature              │ SpreadCycle            │ StealthChop2           │
 * │                      │ (EN_PWM_MODE = 0)      │ (EN_PWM_MODE = 1)      │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ CONTROL METHOD       │ Current Chopping       │ Voltage PWM            │
 * │                      │ (current regulation)   │ (voltage regulation)   │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ PRIMARY CONFIG       │ CHOPCONF register:     │ PWMCONF register:      │
 * │ REGISTERS            │ • TOFF, TBL, CHM       │ • PWM_OFS, PWM_GRAD    │
 * │                      │ • HSTRT, HEND          │ • PWM_AUTOSCALE        │
 * │                      │                        │ • PWM_FREQ             │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ AUDIBLE NOISE        │ Audible chopping       │ Silent operation       │
 * │                      │ Variable frequency     │ Fixed PWM frequency    │
 * │                      │ 20-50 kHz typical      │ 6-15 kHz selectable    │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ VIBRATION            │ Some mechanical        │ Minimal vibration      │
 * │                      │ resonance possible     │ Smooth torque          │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ TORQUE               │ High torque            │ Lower peak torque      │
 * │                      │ Up to rated motor      │ ~70-90% of rated       │
 * │                      │ Full current available │ Limited by PWM control │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ EFFICIENCY           │ Very efficient         │ Good efficiency        │
 * │                      │ 90-95%                 │ 85-90%                 │
 * │                      │ Minimal switching loss │ Higher PWM losses      │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ CURRENT RIPPLE       │ Higher ripple          │ Lower ripple           │
 * │                      │ Hysteresis-based       │ PWM-averaged           │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ MOTOR HEATING        │ Lower heating          │ Slightly higher        │
 * │                      │ (better efficiency)    │ (PWM losses)           │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ OPERATION PRINCIPLE  │ Adaptive hysteresis    │ Voltage-mode PWM       │
 * │                      │ Current sensing        │ Back-EMF compensation  │
 * │                      │ Fast/slow decay        │ Automatic regulation   │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ STALLGUARD MODE      │ StallGuard2            │ StallGuard4            │
 * │                      │ (SGT threshold)        │ (SG4_THRS threshold)   │
 * │                      │ 10-bit resolution      │ 9-bit resolution       │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ COOLSTEP             │ ✓ Supported            │ ✓ Supported            │
 * │ (Current Control)    │ Works well             │ Works well             │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ SPEED RANGE          │ Excellent: 1-2000 RPS  │ Good: 0.1-500 RPS      │
 * │                      │ Best for high speed    │ Best for low speed     │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ STARTUP BEHAVIOR     │ Immediate torque       │ Gentle startup         │
 * │                      │ May have resonance     │ Smooth acceleration    │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ TUNING COMPLEXITY    │ Moderate               │ Easy (auto-tuning)     │
 * │                      │ Tune HSTRT, HEND, TOFF │ Defaults work well     │
 * │                      │ Manual optimization    │ PWM_AUTOSCALE does it  │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ EMI / ELECTRICAL     │ Higher EMI             │ Lower EMI              │
 * │ NOISE                │ Variable frequency     │ Fixed frequency        │
 * │                      │ Harder to filter       │ Easier to filter       │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ RESONANCE HANDLING   │ Can excite resonance   │ Minimal resonance      │
 * │                      │ Needs damping tuning   │ Inherently smooth      │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ POSITION ACCURACY    │ Excellent              │ Very good              │
 * │                      │ Immediate correction   │ Smooth regulation      │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ DYNAMIC RESPONSE     │ Very fast              │ Fast                   │
 * │                      │ Direct current control │ PWM-limited response   │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ TYPICAL APPLICATIONS │ • CNC machines         │ • 3D printers          │
 * │                      │ • Industrial robotics  │ • Office equipment     │
 * │                      │ • High-speed pick/place│ • Medical devices      │
 * │                      │ • Heavy loads          │ • Audio/video gear     │
 * │                      │ • Outdoor/harsh env.   │ • Laboratory equipment │
 * │                      │ • High torque needed   │ • Residential use      │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ POWER CONSUMPTION    │ Lower (more efficient) │ Slightly higher        │
 * │ (at same current)    │ Direct current control │ PWM overhead           │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ CABLE LENGTH         │ Tolerant of long       │ Better with short      │
 * │ TOLERANCE            │ cables (robust)        │ cables (PWM sensitive) │
 * ├──────────────────────┼────────────────────────┼────────────────────────┤
 * │ TEMPERATURE          │ Less sensitive         │ More sensitive         │
 * │ SENSITIVITY          │ Current-based control  │ Resistance-dependent   │
 * └──────────────────────┴────────────────────────┴────────────────────────┘
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * When to Use Each Mode
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * USE SPREADCYCLE WHEN:
 * ────────────────────
 *   ✓ Noise is acceptable (industrial, outdoor, enclosed equipment)
 *   ✓ Maximum torque is required
 *   ✓ High-speed operation (>500 RPM)
 *   ✓ Heavy loads or sudden load changes
 *   ✓ Long motor cable runs (>2 meters)
 *   ✓ Harsh electrical environment (high EMI tolerance)
 *   ✓ Maximum efficiency is critical (battery-powered)
 *   ✓ Operating in wide temperature ranges
 *   ✓ Traditional stepper motor applications
 *   ✓ Industrial automation
 *
 *   Examples:
 *   • CNC routers and mills
 *   • Industrial pick-and-place machines
 *   • Packaging equipment
 *   • Conveyor systems
 *   • Automated guided vehicles (AGVs)
 *   • Heavy-duty linear actuators
 *
 * USE STEALTHCHOP2 WHEN:
 * ─────────────────────
 *   ✓ Silent operation is required (office, home, medical)
 *   ✓ Low vibration is critical
 *   ✓ Low to medium speed operation (<500 RPM)
 *   ✓ Precision positioning at low speeds
 *   ✓ Human proximity (labs, medical, consumer)
 *   ✓ Audio/video equipment (no noise interference)
 *   ✓ Smooth motion is priority over maximum torque
 *   ✓ Short motor cable runs (<1 meter preferred)
 *   ✓ Controlled environment (stable temperature)
 *   ✓ Lower peak torque is acceptable
 *
 *   Examples:
 *   • 3D printers (especially during infill)
 *   • Camera sliders and gimbals
 *   • Laboratory equipment
 *   • Medical imaging devices (CT/MRI positioning)
 *   • Office printers and scanners
 *   • Telescope mounts
 *   • Automated microscope stages
 *   • Home automation (blinds, curtains)
 *
 * HYBRID MODE (Mode Switching):
 * ────────────────────────────
 *   Use TPWMTHRS to switch between modes based on velocity:
 *     • Low speeds: StealthChop2 (quiet, smooth)
 *     • High speeds: SpreadCycle (torque, efficiency)
 *
 *   Configuration:
 *     EN_PWM_MODE = 1 (enable StealthChop2 capability)
 *     TPWMTHRS = velocity_threshold
 *     When TSTEP > TPWMTHRS: StealthChop2 active
 *     When TSTEP ≤ TPWMTHRS: SpreadCycle active
 *
 *   Best for applications needing:
 *     • Quiet homing/positioning (low speed)
 *     • Fast travel moves (high speed, high torque)
 *     • Variable speed profiles
 *
 *   Examples:
 *   • 3D printers (StealthChop2 for infill, SpreadCycle for travel)
 *   • Camera systems (silent positioning, fast repositioning)
 *   • Multi-axis machines with different speed requirements
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * Performance Comparison by Application Type
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * PRECISION POSITIONING (microscopes, metrology):
 *   Winner: StealthChop2
 *   Reason: Minimal vibration, smooth microstepping, no resonance
 *
 * HIGH-SPEED TRANSPORT (CNC rapids, pick-and-place):
 *   Winner: SpreadCycle
 *   Reason: Maximum torque, excellent high-speed performance
 *
 * AUDIO/VIDEO APPLICATIONS (camera sliders, gimbals):
 *   Winner: StealthChop2
 *   Reason: Silent operation, no mechanical noise, smooth motion
 *
 * HEAVY LOADS (conveyors, large actuators):
 *   Winner: SpreadCycle
 *   Reason: Maximum torque output, better efficiency under load
 *
 * CONSUMER PRODUCTS (home automation, small appliances):
 *   Winner: StealthChop2
 *   Reason: Silent operation required in residential environment
 *
 * INDUSTRIAL AUTOMATION (factories, warehouses):
 *   Winner: SpreadCycle
 *   Reason: Robust, efficient, noise acceptable, maximum performance
 *
 * MEDICAL DEVICES (imaging, surgical robots):
 *   Winner: StealthChop2
 *   Reason: Silent, smooth, minimal vibration critical
 *
 * BATTERY-POWERED (mobile robots, portable equipment):
 *   Winner: SpreadCycle (light loads) or Hybrid
 *   Reason: Better efficiency extends battery life
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * Configuration Quick Reference
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * SpreadCycle Configuration:
 * ─────────────────────────
 *   GCONF:
 *     EN_PWM_MODE = 0                    // Select SpreadCycle
 *
 *   CHOPCONF:
 *     TOFF = 3-5                         // Off-time (enable driver)
 *     TBL = 2                            // Blanking time
 *     CHM = 0                            // SpreadCycle mode
 *     HSTRT = 4-5                        // Hysteresis start
 *     HEND = 1-3                         // Hysteresis end
 *
 *   COOLCONF:
 *     SGT = 0 to +10                     // StallGuard2 threshold (tune)
 *
 *   TPWMTHRS = 0                         // Disable StealthChop
 *   TCOOLTHRS = 0 or velocity_limit     // StallGuard active when needed
 *
 * StealthChop2 Configuration:
 * ──────────────────────────
 *   GCONF:
 *     EN_PWM_MODE = 1                    // Select StealthChop2
 *
 *   CHOPCONF:
 *     TOFF = 3-5                         // Still required to enable driver!
 *
 *   PWMCONF:
 *     PWM_AUTOSCALE = 1                  // Enable automatic regulation
 *     PWM_AUTOGRAD = 1                   // Enable gradient tuning
 *     PWM_OFS = 36                       // Offset (default)
 *     PWM_GRAD = 14                      // Gradient (default)
 *     PWM_FREQ = 0-1                     // PWM frequency
 *
 *   SG4_THRS = 100 (tune)                // StallGuard4 threshold
 *   TPWMTHRS = 0xFFFFF                   // StealthChop at all speeds
 *   TCOOLTHRS = 0 or velocity_limit     // StallGuard4 active when needed
 *
 * Hybrid Configuration (Recommended for 3D Printers):
 * ───────────────────────────────────────────────────
 *   GCONF:
 *     EN_PWM_MODE = 1                    // Enable StealthChop2 capability
 *
 *   PWMCONF: (as above for StealthChop2)
 *   CHOPCONF: (as above for SpreadCycle)
 *
 *   TPWMTHRS = 500                       // Switch at ~500 full steps/sec
 *                                        // Below 500: StealthChop2 (quiet)
 *                                        // Above 500: SpreadCycle (fast)
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * EN_PWM_MODE - Enable StealthChop2 PWM Mode (GCONF bit 0)
 *
 * Primary control bit for selecting between SpreadCycle and StealthChop2.
 *
 * Settings:
 *   0 = SpreadCycle mode (current chopping, audible, high torque)
 *   1 = StealthChop2 mode (voltage PWM, silent, smooth)
 *
 * Note: Can be changed during operation for velocity-dependent mode switching.
 *       Use TPWMTHRS to set automatic switching threshold.
 */
#define TMC522X_GCONF_EN_PWM_MODE   BIT(0) /* 0: SpreadCycle, 1: StealthChop2 */
#define TMC522X_GCONF_SD            BIT(7)
#define TMC522X_GCONF_DIRECT_MODE   BIT(6)
#define TMC522X_GCONF_STOP_ENABLE   BIT(4)
#define TMC522X_GCONF_DRV_ENN       BIT(9) /* 1: Driver disable, 0: Driver enable */
/** @} */

/**
 * @name TMC522X DRVCONF register fields
 * @{
 */
#define TMC522X_DRVCONF_FS_GAIN_MASK  GENMASK(3, 2)
#define TMC522X_DRVCONF_FS_SENSE_MASK GENMASK(1, 0)
/** @} */

/**
 * @name TMC522X GLOBALSCALER register fields
 * @{
 */
#define TMC522X_GLOBALSCALER_A_MASK GENMASK(7, 0)
#define TMC522X_GLOBALSCALER_B_MASK GENMASK(15, 8)
/** @} */

/**
 * @name TMC522X DIAG_CONF register fields
 * @{
 */
#define TMC522X_DIAG1_INVPP_MASK       BIT(31)
#define TMC522X_DIAG1_NOD_PP_MASK      BIT(30)
#define TMC522X_DIAG0_INVPP_MASK       BIT(29)
#define TMC522X_DIAG0_NOD_PP_MASK      BIT(28)
#define TMC522X_DIAG1_POS_REACH_MASK   BIT(27)
#define TMC522X_DIAG0_STALL_MASK       BIT(3)
/** @} */

/**
 * @name TMC522X IOIN register fields
 * @{
 */
#define TMC522X_IOIN_EXT_CLK_MASK GENMASK(24, 23)
/** @} */

/**
 * @name TMC522X Clock frequency constants
 * @{
 */
#define TMC522X_INTERNAL_CLOCK_HZ  125000000 /* 125 MHz */
#define TMC522X_QUIESCENT_CLOCK_HZ 609000    /* 609 kHz */
/** @} */

/**
 * @name TMC522X Microstep Look-Up Table Registers
 * @{
 */
#define TMC522X_REG_MSLUT(x)      (0x08 + (x))
#define TMC522X_REG_MSLUT_START   0x10
#define TMC522X_REG_MSLUT_SEL     0x11
/** @} */

/**
 * @name TMC522X Position Compare Registers
 * @{
 */
#define TMC522X_REG_X_COMPARE       0x12
#define TMC522X_REG_X_COMPAR_REPEAT 0x13
/** @} */

/**
 * @name TMC522X Velocity Dependent Configuration Registers
 * @{
 */

/**
 * IHOLD_IRUN (0x14) - Hold and run current settings
 */
#define TMC522X_REG_IHOLD_IRUN  0x14

/**
 * TPOWERDOWN (0x15) - Power-down delay
 *
 * Delay time after motor becomes standstill (STST) before power-down.
 * Time = TPOWERDOWN * 2^18 / f_CLK
 * Range: ~0 to 5 seconds (with f_CLK = 12.5 MHz)
 * 0 = no delay
 * Higher values increase delay in steps of 2^16 clock cycles
 * Default: 0, Range: 0-255
 */
#define TMC522X_REG_TPOWERDOWN  0x15

/**
 * TSTEP (0x16) - Actual step time (read-only)
 *
 * Actual time between two 1/256 microsteps.
 * Measured in units of 1/f_CLK based on step input frequency.
 * Value = 0xFFFFF (1048575) on overflow or standstill
 * Used for velocity-based mode switching comparisons
 * Default: 0, Range: 0-1048575 (read-only)
 */
#define TMC522X_REG_TSTEP       0x16

/**
 * TPWMTHRS (0x17) - StealthChop PWM threshold
 *
 * Upper velocity threshold for StealthChop2 PWM mode.
 * Condition: TSTEP > TPWMTHRS
 *   - StealthChop2 PWM enabled
 *   - DcStep disabled
 * Sets velocity below which StealthChop2 is active.
 * Default: 0, Range: 0-1048575
 */
#define TMC522X_REG_TPWMTHRS    0x17

/**
 * TCOOLTHRS (0x18) - CoolStep/StallGuard threshold
 *
 * Lower velocity threshold for CoolStep & StallGuard2.
 * Condition: TCOOLTHRS < TSTEP < THIGH
 *   - Enables StallGuard2 (if configured)
 *   - Enables CoolStep (if configured)
 *   - Disables StealthChop2 voltage PWM mode
 *
 * IMPORTANT: StallGuard detection control:
 *   TCOOLTHRS = 0xFFFFF: StallGuard DISABLED (never active)
 *   TCOOLTHRS = 0:       StallGuard ENABLED at all velocities
 *   TCOOLTHRS = value:   StallGuard active when |VACTUAL| > TCOOLTHRS
 *
 * Default: 0, Range: 0-1048575
 */
#define TMC522X_REG_TCOOLTHRS   0x18

/**
 * THIGH (0x19) - High-speed threshold
 *
 * Upper threshold for high-speed operation.
 * Condition: TSTEP ≤ THIGH
 *   - CoolStep disabled (motor runs with normal current scale)
 *   - StealthChop2 voltage PWM disabled
 *   - If vhighfs=1: Fast decay mode (chm=1), chopper sync disabled
 *   - If vhighfs=0: Full-step mode, stall detection switches to DcStep
 *
 * Default: 0, Range: 0-1048575
 */
#define TMC522X_REG_THIGH       0x19

/**
 * @name TMC522X Velocity Threshold Helper Values
 * @{
 */
/* StallGuard control values for TCOOLTHRS register */
#define TMC522X_STALLGUARD_ENABLED_ALL_SPEEDS  0         /* Active at all velocities */
#define TMC522X_STALLGUARD_DISABLED            0xFFFFF   /* Never active (disabled) */

/* Feature disable values (set to 0 or 0xFFFFF depending on register) */
#define TMC522X_STEALTHCHOP_DISABLED           0xFFFFF   /* TPWMTHRS: Never use StealthChop */
#define TMC522X_COOLSTEP_DISABLED              0xFFFFF   /* TCOOLTHRS: Never use CoolStep */
#define TMC522X_HIGHSPEED_DISABLED             0         /* THIGH: Never enter high-speed mode */

/* Feature enable at all speeds */
#define TMC522X_STEALTHCHOP_ENABLED_ALL_SPEEDS 0         /* TPWMTHRS: Always use StealthChop */
#define TMC522X_COOLSTEP_ENABLED_ALL_SPEEDS    0         /* TCOOLTHRS: Always use CoolStep */

/**
 * ═══════════════════════════════════════════════════════════════════════════
 * StallGuard2 vs StallGuard4 - Comprehensive Comparison
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * The TMC522x provides TWO different StallGuard algorithms optimized for
 * different chopper modes. The driver AUTOMATICALLY selects the appropriate
 * algorithm based on motor velocity and threshold settings.
 *
 * ┌────────────────────┬─────────────────────────┬─────────────────────────┐
 * │ Feature            │ StallGuard2             │ StallGuard4             │
 * ├────────────────────┼─────────────────────────┼─────────────────────────┤
 * │ Chopper Mode       │ SpreadCycle             │ StealthChop2            │
 * │                    │ (high torque, audible)  │ (quiet, low vibration)  │
 * ├────────────────────┼─────────────────────────┼─────────────────────────┤
 * │ Active Condition   │ TSTEP ≤ TPWMTHRS        │ TCOOLTHRS ≥ TSTEP >     │
 * │                    │ (typically TPWMTHRS=0)  │ TPWMTHRS                │
 * ├────────────────────┼─────────────────────────┼─────────────────────────┤
 * │ Result Register    │ DRV_STATUS bits 9:0     │ SG4_RESULT (0x40)       │
 * │                    │ SG_RESULT               │                         │
 * ├────────────────────┼─────────────────────────┼─────────────────────────┤
 * │ Result Range       │ 0-1023 (10-bit)         │ 0-510 (9-bit)           │
 * │                    │ Wider dynamic range     │ Higher resolution       │
 * ├────────────────────┼─────────────────────────┼─────────────────────────┤
 * │ Threshold Type     │ SGT (Signed Offset)     │ SG4_THRS (Comparison)   │
 * │                    │ COOLCONF bits 22:16     │ Register 0x3F           │
 * ├────────────────────┼─────────────────────────┼─────────────────────────┤
 * │ Threshold Range    │ -64 to +63 (signed)     │ 0 to 511 (unsigned)     │
 * ├────────────────────┼─────────────────────────┼─────────────────────────┤
 * │ Stall Detection    │ (SG_RESULT + SGT) = 0   │ SG4_RESULT < SG4_THRS   │
 * │ Logic              │ Offset-based            │ Comparison-based        │
 * ├────────────────────┼─────────────────────────┼─────────────────────────┤
 * │ Tuning Method      │ Adjust SGT offset so    │ Set SG4_THRS to value   │
 * │                    │ SG_RESULT reaches 0     │ just above minimum      │
 * │                    │ at max load             │ SG4_RESULT before stall │
 * ├────────────────────┼─────────────────────────┼─────────────────────────┤
 * │ Sensitivity        │ Lower SGT = more        │ Higher SG4_THRS = more  │
 * │ Adjustment         │ sensitive               │ sensitive               │
 * ├────────────────────┼─────────────────────────┼─────────────────────────┤
 * │ Measurement        │ Updates per full step   │ Updates per full step   │
 * │ Update Rate        │ (4 microsteps @ 1/4)    │ (or averaged if         │
 * │                    │                         │ SG4_FILT_EN=1)          │
 * ├────────────────────┼─────────────────────────┼─────────────────────────┤
 * │ Filter Option      │ SFILT (COOLCONF bit 24) │ SG4_FILT_EN             │
 * │                    │ 0=standard, 1=filtered  │ (PWMCONF bit 24)        │
 * │                    │ Averages 4 full steps   │ Averages 4 phases       │
 * ├────────────────────┼─────────────────────────┼─────────────────────────┤
 * │ Individual Phase   │ Not available           │ SG4_IND_0/1/2/3         │
 * │ Measurements       │                         │ (when SG4_FILT_EN=1)    │
 * ├────────────────────┼─────────────────────────┼─────────────────────────┤
 * │ Primary Use Cases  │ • High torque apps      │ • Quiet operation       │
 * │                    │ • Traditional steppers  │ • Office/home use       │
 * │                    │ • CoolStep optimization │ • Low vibration needed  │
 * │                    │ • Sensorless homing     │ • Precision positioning │
 * ├────────────────────┼─────────────────────────┼─────────────────────────┤
 * │ Advantages         │ • Wider dynamic range   │ • Silent operation      │
 * │                    │ • Works at higher loads │ • No motor vibration    │
 * │                    │ • Better at high speed  │ • Higher resolution     │
 * │                    │ • Established algorithm │ • Better precision      │
 * ├────────────────────┼─────────────────────────┼─────────────────────────┤
 * │ Limitations        │ • Audible noise         │ • Lower torque limit    │
 * │                    │ • Mechanical vibration  │ • Not for high loads    │
 * │                    │ • Electromagnetic noise │ • Velocity dependent    │
 * ├────────────────────┼─────────────────────────┼─────────────────────────┤
 * │ Velocity Range     │ Good: 1-1000 RPS        │ Good: 0.1-500 RPS       │
 * │                    │ Poor: <1 RPS (unstable) │ Poor: >500 RPS          │
 * ├────────────────────┼─────────────────────────┼─────────────────────────┤
 * │ CoolStep           │ ✓ Supported             │ ✓ Supported             │
 * │ Compatibility      │ (automatic selection)   │ (automatic selection)   │
 * ├────────────────────┼─────────────────────────┼─────────────────────────┤
 * │ Configuration      │ TPWMTHRS = 0            │ TPWMTHRS = 0xFFFFF      │
 * │ in Driver          │ (SpreadCycle always on) │ (StealthChop at all     │
 * │                    │                         │ speeds)                 │
 * └────────────────────┴─────────────────────────┴─────────────────────────┘
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * Automatic Algorithm Selection Logic
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * The driver AUTOMATICALLY switches between StallGuard2 and StallGuard4 based
 * on the current motor velocity and threshold settings:
 *
 * Decision Flow:
 *
 *   1. Check TSTEP (current step time - inverse of velocity)
 *   2. Compare with TPWMTHRS:
 *
 *      TSTEP ≤ TPWMTHRS ?
 *      │
 *      ├─ YES → SpreadCycle Mode → StallGuard2
 *      │        • Use SG_RESULT (DRV_STATUS bits 9:0)
 *      │        • Stall when: (SG_RESULT + SGT) = 0
 *      │        • Active when: TCOOLTHRS < TSTEP
 *      │
 *      └─ NO  → StealthChop2 Mode → StallGuard4
 *               • Use SG4_RESULT (register 0x40)
 *               • Stall when: SG4_RESULT < SG4_THRS
 *               • Active when: TCOOLTHRS ≥ TSTEP > TPWMTHRS
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * Configuration Examples
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Example 1: SpreadCycle with StallGuard2 (High Torque Application)
 * ────────────────────────────────────────────────────────────────────────
 *   enable_stealthchop = false;  // Devicetree setting
 *   TPWMTHRS = 0;                // Driver sets automatically
 *   TCOOLTHRS = 0;               // Enable StallGuard at all speeds
 *   SGT = 0 to +10;              // Start tuning from 0
 *
 *   Result: StallGuard2 always active, high torque, audible operation
 *
 * Example 2: StealthChop2 with StallGuard4 (Quiet Application)
 * ────────────────────────────────────────────────────────────────────────
 *   enable_stealthchop = true;   // Devicetree setting
 *   TPWMTHRS = 0xFFFFF;          // Driver sets automatically
 *   TCOOLTHRS = 0;               // Enable StallGuard at all speeds
 *   SG4_THRS = 100;              // Start tuning from measured min value
 *
 *   Result: StallGuard4 always active, quiet operation, lower torque
 *
 * Example 3: Hybrid Mode (Speed-Dependent)
 * ────────────────────────────────────────────────────────────────────────
 *   enable_stealthchop = true;
 *   TPWMTHRS = 500;              // StealthChop below 500 steps/sec
 *   TCOOLTHRS = 0;
 *
 *   Result: StallGuard4 at low speeds (quiet)
 *           StallGuard2 at high speeds (high torque)
 *           Automatic switching based on velocity
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * Which One Should You Use?
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Use StallGuard2 when:
 *   ✓ High torque is required
 *   ✓ Motor noise is acceptable
 *   ✓ Operating at higher speeds (>10 RPS)
 *   ✓ Sensorless homing with hard stops
 *   ✓ CoolStep current optimization is priority
 *   ✓ Working with larger motors
 *
 * Use StallGuard4 when:
 *   ✓ Quiet operation is required (office, home, medical)
 *   ✓ Low vibration is critical
 *   ✓ Precision positioning at low speeds
 *   ✓ Smaller motors with lower torque needs
 *   ✓ Soft obstacle detection
 *   ✓ Battery-powered applications (lower current)
 *
 * Use Hybrid Mode when:
 *   ✓ Need quiet operation at low speeds
 *   ✓ Need high torque at high speeds
 *   ✓ Variable speed application
 *   ✓ Want best of both algorithms
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * StallGuard Algorithm Selection (StallGuard2 vs StallGuard4)
 *
 * The TMC522x automatically selects which StallGuard algorithm to use based on
 * the current chopper mode (determined by velocity thresholds):
 *
 * StallGuard2 (SpreadCycle mode):
 *   Active when: TSTEP <= TPWMTHRS (or TPWMTHRS=0)
 *   Uses: SGT threshold (COOLCONF bits 22:16, signed offset)
 *   Range: SG_RESULT 0-1023 (10-bit)
 *   Tuning: Adjust SGT so SG_RESULT reaches 0 at stall point
 *
 * StallGuard4 (StealthChop2 mode):
 *   Active when: TCOOLTHRS >= TSTEP > TPWMTHRS
 *   Uses: SG4_THRS threshold (register 0x3F, unsigned comparison)
 *   Range: SG4_RESULT 0-510 (9-bit, higher resolution)
 *   Tuning: Set SG4_THRS based on minimum SG4_RESULT before stall
 *   Stall condition: SG4_RESULT < SG4_THRS
 *
 * CRITICAL: For StallGuard4 to work, TPWMTHRS must be > 0!
 *   If TPWMTHRS=0, then TSTEP>0 is always false, StealthChop never activates,
 *   and StallGuard4 is never used (only StallGuard2 works).
 *
 * Recommended settings:
 *   For StealthChop2/StallGuard4: TPWMTHRS = 0xFFFFF (StealthChop at all speeds)
 *   For SpreadCycle/StallGuard2:  TPWMTHRS = 0 (StealthChop disabled)
 */
/** @} */

/**
 * @name TMC522X IHOLD_IRUN register fields
 * @{
 */
#define TMC522X_IHOLD_MASK       GENMASK(4, 0)
#define TMC522X_IRUN_MASK        GENMASK(12, 8)
#define TMC522X_IHOLDDELAY_MASK  GENMASK(19, 16)
#define TMC522X_IRUNDELAY_MASK   GENMASK(27, 24)
/** @} */

/**
 * @name TMC522X Ramp Generator Registers
 * @{
 */
#define TMC522X_REG_RAMPMODE   0x1A
#define TMC522X_REG_XACTUAL    0x1B
#define TMC522X_REG_VACTUAL    0x1C
#define TMC522X_REG_AACTUAL    0x1D
#define TMC522X_REG_VSTART     0x1E
#define TMC522X_REG_A1         0x1F
#define TMC522X_REG_V1         0x20
#define TMC522X_REG_A2         0x21
#define TMC522X_REG_V2         0x22
#define TMC522X_REG_AMAX       0x23
#define TMC522X_REG_VMAX       0x24
#define TMC522X_REG_DMAX       0x25
#define TMC522X_REG_D2         0x26
#define TMC522X_REG_D1         0x27
#define TMC522X_REG_VSTOP      0x28
#define TMC522X_REG_TZEROWAIT  0x29
#define TMC522X_REG_TVMAX      0x2A
#define TMC522X_REG_XTARGET    0x2B
/** @} */

/**
 * @name TMC522X Ramp Generator Driver Feature Control Registers
 * @{
 */
#define TMC522X_REG_VDCMIN    0x2C
#define TMC522X_REG_SW_MODE   0x2D
#define TMC522X_REG_RAMP_STAT 0x2E
#define TMC522X_REG_XLATCH    0x2F
/** @} */

/**
 * @name TMC522X SW_MODE register fields (0x2D)
 *
 * Controls stop modes, reference switches, latching, and StallGuard stop.
 * @{
 */

/* Bits 0-1: Reference switch stop enables */
/**
 * STOP_L_ENABLE (Bit 0)
 * Enables automatic motor stop when left reference switch (REFL) becomes active.
 * Motor restarts automatically once the switch is released.
 */
#define TMC522X_SW_MODE_STOP_L_ENABLE     BIT(0)

/**
 * STOP_R_ENABLE (Bit 1)
 * Enables automatic motor stop when right reference switch (REFR) becomes active.
 * Motor restarts automatically once the switch is released.
 */
#define TMC522X_SW_MODE_STOP_R_ENABLE     BIT(1)

/* Bits 2-3: Reference switch polarity */
/**
 * POL_STOP_L (Bit 2)
 * Sets active polarity for left reference switch (REFL).
 * 0 = Non-inverted (high active: high level on REFL stops motor)
 * 1 = Inverted (low active: low level on REFL stops motor)
 */
#define TMC522X_SW_MODE_POL_STOP_L        BIT(2)

/**
 * POL_STOP_R (Bit 3)
 * Sets active polarity for right reference switch (REFR).
 * 0 = Non-inverted (high active: high level on REFR stops motor)
 * 1 = Inverted (low active: low level on REFR stops motor)
 */
#define TMC522X_SW_MODE_POL_STOP_R        BIT(3)

/**
 * SWAP_LR (Bit 4)
 * Swaps left/right reference switch inputs (REFL ↔ REFR).
 * Useful for motor direction reversal or wiring differences.
 * 0 = Default assignments
 * 1 = REFL and REFR swapped
 */
#define TMC522X_SW_MODE_SWAP_LR           BIT(4)

/* Bits 5-8: Position latching controls */
/**
 * LATCH_L_ACTIVE (Bit 5)
 * Latches XACTUAL → XLATCH on active-going edge of REFL.
 * Activates latch_l_active for spurious stop event detection via status_latch_l.
 * 0 = Disabled
 * 1 = Latching on rising edge of status_stop_l
 */
#define TMC522X_SW_MODE_LATCH_L_ACTIVE    BIT(5)

/**
 * LATCH_L_INACTIVE (Bit 6)
 * Latches XACTUAL → XLATCH on inactive-going edge of status_stop_l.
 * Active level defined by pol_stop_l.
 * 0 = Disabled
 * 1 = Latching on falling edge of status_stop_l
 */
#define TMC522X_SW_MODE_LATCH_L_INACTIVE  BIT(6)

/**
 * LATCH_R_ACTIVE (Bit 7)
 * Latches XACTUAL → XLATCH on active-going edge of status_stop_r.
 * Activates latch_r_active for spurious stop detection via status_latch_r.
 * 0 = Disabled
 * 1 = Latching on rising edge of status_stop_r
 */
#define TMC522X_SW_MODE_LATCH_R_ACTIVE    BIT(7)

/**
 * LATCH_R_INACTIVE (Bit 8)
 * Latches XACTUAL → XLATCH on inactive-going edge of status_stop_r.
 * Active level is defined by pol_stop_r.
 * 0 = Disabled
 * 1 = Latching on falling edge of status_stop_r
 */
#define TMC522X_SW_MODE_LATCH_R_INACTIVE  BIT(8)

/**
 * SG_STOP (Bit 10) ⭐ CRITICAL FOR STALLGUARD2
 *
 * Enables stopping the motor using StallGuard detection.
 *
 * Operation:
 *   - Disables motor release after a stop event
 *   - Uses TCOOLTHRS to determine velocity threshold
 *   - Ramp generator sets VACTUAL = 0 when StallGuard event occurs
 *
 * IMPORTANT Requirements:
 *   - Should NOT be enabled during motor spin-up
 *   - Enable only when motor velocity is above stable StallGuard threshold
 *   - Must program velocity threshold using TCOOLTHRS register
 *   - Works in conjunction with COOLCONF.SG_STOP (bit 10)
 *   - BOTH bits must be enabled for automatic StallGuard stop to work
 *
 * Decode:
 *   0 = Disabled (motor does not stop on StallGuard event)
 *   1 = Enabled (motor stops when StallGuard detects stall)
 */
#define TMC522X_SW_MODE_SG_STOP_ENABLE    BIT(10)

/**
 * EN_SOFTSTOP (Bit 11)
 *
 * Uses configured soft-stop mode to decelerate motor using ramp settings:
 *   DMAX, V1, D1, V2, D2, VSTOP, TZEROWAIT
 *
 * Stop behavior based on velocity sign:
 *   - REFL, VIRTUAL_STOP_L → negative velocities
 *   - REFR, VIRTUAL_STOP_R → positive velocities
 *
 * Hard stop uses TZEROWAIT before releasing driver.
 *
 * WARNING: Do not use soft stop with StealthChop at high velocities!
 *          Can cause driver reset.
 *
 * Decode:
 *   0 = Hard stop (forces velocity to 0 immediately)
 *   1 = Soft stop (uses ramp deceleration)
 */
#define TMC522X_SW_MODE_EN_SOFTSTOP       BIT(11)

/**
 * EN_VIRTUAL_STOP_L (Bit 12)
 *
 * Enables automatic motor stop during active left virtual stop condition.
 *
 * Decode:
 *   0 = Disabled
 *   1 = Enabled
 */
#define TMC522X_SW_MODE_EN_VIRTUAL_STOP_L BIT(12)

/**
 * EN_VIRTUAL_STOP_R (Bit 13)
 *
 * Enables automatic motor stop during active right virtual stop condition.
 *
 * Decode:
 *   0 = Disabled
 *   1 = Enabled
 */
#define TMC522X_SW_MODE_EN_VIRTUAL_STOP_R BIT(13)

/**
 * EN_REFL_HOMING (Bit 14)
 *
 * On REFL rising edge, X_TARGET is loaded with VIRTUAL_STOP_L.
 *
 * In sngc mode:
 *   - sgp_sts.qn and drv_en.gv set
 *   - Before device reaches IHOLD, movement triggered by writing
 *     VIRTUAL_STOP_L to X_TARGET
 *
 * Writing to X_TARGET via COM interface clears this bit and disables homing.
 * When used with homing interrupt, bit clears automatically on interrupt.
 *
 * Decode:
 *   0 = Disabled (REFL interrupts cleared; position continues)
 *   1 = Enabled (triggers homing event on rising edge of REFL)
 */
#define TMC522X_SW_MODE_EN_REFL_HOMING    BIT(14)

/**
 * EN_REFR_HOMING (Bit 15)
 *
 * On rising edge of REFR, X_TARGET is set to VIRTUAL_STOP_R.
 *
 * In sngc mode:
 *   - sgp_sts.qn and drv_en.gv will be set (from GCONF)
 *   - Before reaching hold current (IHOLD), movement triggers by writing
 *     VIRTUAL_STOP_R to X_TARGET
 *
 * Writing to X_TARGET via COM interface clears this bit and disables homing.
 * When used with homing interrupt, bit clears automatically on interrupt.
 *
 * Decode:
 *   0 = Disabled (REFR interrupts cleared; position movement continues)
 *   1 = Enabled (triggers homing event on rising edge of REFR)
 */
#define TMC522X_SW_MODE_EN_REFR_HOMING    BIT(15)

/**
 * EN_AUTO_FEATURE (Bit 16)
 *
 * Enables automatic movement between two virtual stop positions:
 *   - VIRTUAL_STOP_L (0x33)
 *   - VIRTUAL_STOP_R (0x34)
 *
 * Operation:
 *   - Movement uses configured ramp parameters
 *   - Device updates X_TARGET automatically:
 *     * Loads VIRTUAL_STOP_L when reaching VIRTUAL_STOP_R
 *     * Loads VIRTUAL_STOP_R when reaching VIRTUAL_STOP_L
 *   - Behavior continues automatically until disabled
 *
 * Procedure:
 *   1. Drive to either virtual stop position defined in VIRTUAL_STOP_x registers
 *   2. Once target is reached, enable this bit to start automatic shuttling
 *   3. Motion continues back-and-forth until disabled
 *
 * Decode:
 *   0 = Disabled (finishes last automatic position update then stops)
 *   1 = Enabled (continuous movement between VIRTUAL_STOP_L and VIRTUAL_STOP_R)
 */
#define TMC522X_SW_MODE_EN_AUTO_FEATURE   BIT(16)

/** @} */

/**
 * @name TMC522X RAMP_STAT register fields
 * @{
 */
#define TMC522X_EVENT_STOP_SG_MASK     BIT(6)
#define TMC522X_EVENT_POS_REACH_MASK   BIT(7)
/** @} */

/**
 * @name TMC522X Encoder Registers
 * @{
 */
#define TMC522X_REG_X_BEMF         0x30
#define TMC522X_REG_BEMF_CONF      0x31
#define TMC522X_REG_BEMF_DEVIATION 0x32
#define TMC522X_REG_VIRTUAL_STOPL  0x33
#define TMC522X_REG_VIRTUAL_STOPR  0x34
#define TMC522X_REG_X_BEMF_LATCH   0x35
/** @} */

/**
 * @name TMC522X Motor Driver Registers
 * @{
 */
#define TMC522X_REG_MSCNT      0x36
#define TMC522X_REG_MSCURACT   0x37
#define TMC522X_REG_CHOPCONF   0x38
#define TMC522X_REG_COOLCONF   0x39
#define TMC522X_REG_DCCTRL     0x3A
#define TMC522X_REG_DRV_STATUS 0x3B

/**
 * PWMCONF (0x3C) - StealthChop2 PWM Configuration
 *
 * Main configuration register for StealthChop2 silent operation mode.
 * Controls PWM amplitude, frequency, automatic tuning, and regulation behavior.
 *
 * Key fields:
 *   - PWM_OFS/PWM_GRAD: Base amplitude and gradient
 *   - PWM_FREQ: PWM switching frequency (0-3)
 *   - PWM_AUTOSCALE: Enable automatic current regulation
 *   - PWM_AUTOGRAD: Enable automatic gradient tuning
 *   - FREEWHEEL: Output behavior when current is zero
 *   - PWM_MEAS_SD_ENABLE: Measurement timing control
 *   - PWM_DIS_REG_STST: Standstill regulation control
 *   - SG4_FILT_EN: StallGuard4 filtering
 *
 * Default: Varies by field (see individual field documentation)
 */
#define TMC522X_REG_PWMCONF    0x3C

/**
 * PWM_SCALE (0x3D) - StealthChop2 PWM Scaling Status [Read-Only]
 *
 * Reports the actual PWM amplitude scaling determined by the driver.
 * Provides insight into automatic regulation behavior.
 *
 * Field: PWM_SCALE_SUM (bits 7:0)
 *   - Actual PWM scaling value from automatic regulation
 *   - Higher precision than PWM_GRAD/PWM_OFS
 *   - Range: 0-255
 *   - Low values indicate higher motor load
 *   - High values indicate lower motor load
 *
 * Interpretation:
 *   0-50:   Very high load, near maximum amplitude
 *   50-150: Normal operating load
 *   150-255: Light load, reduced amplitude
 *
 * Use for:
 *   - Monitoring StealthChop2 regulation
 *   - Verifying automatic scaling is working
 *   - Load estimation during operation
 *
 * Note: Read-only register, automatically updated by driver
 */
#define TMC522X_REG_PWM_SCALE  0x3D

/**
 * PWM_AUTO (0x3E) - StealthChop2 Automatic Tuning Results [Read-Only]
 *
 * Contains the automatically generated PWM parameters from internal tuning.
 * Useful for debugging, calibration, and understanding regulation behavior.
 *
 * Fields:
 *   PWM_OFS_AUTO (bits 7:0):
 *     - Automatically determined PWM offset value
 *     - Result of automatic tuning for PWM_OFS
 *     - Can be copied to PWM_OFS for manual mode
 *     - Range: 0-255
 *
 *   PWM_GRAD_AUTO (bits 23:16):
 *     - Automatically determined PWM gradient value
 *     - Result of automatic tuning for PWM_GRAD
 *     - Can be copied to PWM_GRAD for manual mode
 *     - Range: 0-255
 *
 *   PWM_SCALE_AUTO (bits 8/15:9 - varies by version):
 *     - Shows automatically generated StealthChop2 PWM value
 *     - Indicates load level (low values = higher load)
 *     - Range: Typically 0-255 or signed -255 to +255
 *     - Read-only indication of current regulation state
 *
 * Use cases:
 *   - Debugging automatic tuning
 *   - Determining optimal manual PWM_OFS/PWM_GRAD values
 *   - Verifying tuning has completed successfully
 *   - Load monitoring during operation
 *
 * Note: Values update continuously during operation with PWM_AUTOGRAD=1
 */
#define TMC522X_REG_PWM_AUTO   0x3E
#define TMC522X_REG_SG4_THRS   0x3F
#define TMC522X_REG_SG4_RESULT 0x40
#define TMC522X_REG_SG4_IND    0x41
#define TMC522X_REG_SG_PREWARN_CONF 0x42
#define TMC522X_REG_OTP_MODE   0x7D
/** @} */

/**
 * @name TMC522X GSTAT register fields
 * @{
 */
#define TMC522X_GSTAT_RESET   BIT(0)
#define TMC522X_GSTAT_DRV_ERR BIT(1)
/** @} */

/**
 * @name TMC522X CHOPCONF register fields - SpreadCycle & Chopper Configuration
 * @{
 */

/**
 * ═══════════════════════════════════════════════════════════════════════════
 * WHAT IS CHOPPER MODE? - Fundamental Concept
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * CHOPPER MODE is the method used to control current flowing through stepper
 * motor coils. Instead of applying constant voltage/current, the driver rapidly
 * switches (chops) the current on and off to regulate it to a target level.
 *
 * WHY CHOPPING IS NECESSARY:
 * ─────────────────────────
 *
 * Problem without chopping:
 *   If you apply constant voltage to a motor coil:
 *     1. Current would continuously increase (L di/dt = V - I×R)
 *     2. Current would exceed safe limits and damage the motor
 *     3. No way to control exact current level
 *     4. Motor would overheat and burn out
 *
 * Solution - Current Chopping:
 *   The driver turns current ON and OFF rapidly to maintain average current:
 *     ON phase:  Current increases (voltage applied to coil)
 *     OFF phase: Current decreases (voltage removed, current decays)
 *     Result:    Current oscillates around target value (current ripple)
 *
 * BASIC CHOPPING CYCLE:
 * ────────────────────
 *
 *   Current
 *     ↑
 *     │     /\        /\        /\        Target current (IRUN)
 *     │    /  \      /  \      /  \       ← Actual current oscillates around target
 *     │   /    \    /    \    /    \
 *     │  /      \  /      \  /      \
 *     │ /        \/        \/        \
 *     └─────────────────────────────────→ Time
 *       ↑ON↑OFF↑ON↑OFF↑ON↑OFF
 *
 *   Voltage to coil:
 *   ████░░░░████░░░░████░░░░  (switching pattern)
 *   ON   OFF ON  OFF ON  OFF
 *
 * HOW CHOPPING WORKS:
 * ──────────────────
 *
 * Step 1: ON PHASE (Fast Rise)
 *   • Driver turns on MOSFETs to apply voltage
 *   • Current flows through motor coil
 *   • Current increases: di/dt = (V_supply - V_backEMF) / L
 *   • Continues until current reaches upper threshold
 *
 * Step 2: OFF PHASE (Slow Decay)
 *   • Driver turns off high-side MOSFETs
 *   • Current must continue flowing (inductance)
 *   • Two decay mechanisms available:
 *     a) SLOW DECAY: Current recirculates through freewheeling diodes
 *        - Current decreases slowly
 *        - Lower switching losses
 *        - More common in stepper drivers
 *     b) FAST DECAY: Current forced to decrease quickly
 *        - Reverse voltage applied
 *        - Faster regulation
 *        - Higher switching losses
 *
 * Step 3: DETECTION (Current Sensing)
 *   • Sense resistors measure actual current
 *   • Comparator checks if current below lower threshold
 *   • When threshold reached, cycle repeats (back to Step 1)
 *
 * KEY PARAMETERS:
 * ──────────────
 *
 * TOFF (Off-Time):
 *   • How long current is allowed to decay during OFF phase
 *   • Longer TOFF = slower chopping frequency, more current ripple
 *   • Shorter TOFF = faster frequency, less ripple, more switching losses
 *
 * TBL (Blanking Time):
 *   • Time to ignore switching noise after MOSFET transitions
 *   • Prevents false current measurements during switching transients
 *   • Too short = noise causes errors, Too long = slower regulation
 *
 * HYSTERESIS (HSTRT/HEND):
 *   • Window between upper and lower current thresholds
 *   • Wider window = lower frequency, higher ripple, smoother torque
 *   • Narrower window = higher frequency, lower ripple, faster response
 *
 * CHM (Chopper Mode):
 *   • Selects the algorithm for chopper control
 *   • CHM=0: SpreadCycle (adaptive, intelligent)
 *   • CHM=1: Classic constant off-time (traditional, predictable)
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * CHOPPER MODE vs STEALTH MODE
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * CHOPPER MODES (SpreadCycle/Classic):
 * ────────────────────────────────────
 *   Principle: CURRENT REGULATION
 *   • Directly measure and control motor current
 *   • Switch based on current thresholds (comparator-based)
 *   • Variable switching frequency (adapts to conditions)
 *   • Audible noise from variable frequency
 *   • Maximum torque and efficiency
 *   • Configured via CHOPCONF register
 *
 *   Switching pattern example:
 *   ████░░░░░░████░░░░████░░░░░░████  (variable frequency)
 *   ON  OFF   ON  OFF  ON   OFF   ON   ← Timing varies with load/speed
 *
 * STEALTHCHOP2 MODE:
 * ─────────────────
 *   Principle: VOLTAGE REGULATION
 *   • Control motor via voltage PWM (like voltage-mode PWM)
 *   • Fixed PWM frequency (6-15 kHz selectable)
 *   • Back-EMF compensation for current control
 *   • Silent operation (fixed frequency above audible range)
 *   • Lower peak torque but very smooth
 *   • Configured via PWMCONF register
 *
 *   Switching pattern example:
 *   ████░░████░░████░░████░░████░░  (fixed frequency)
 *   ON  OFF ON OFF ON OFF ON OFF ON  ← Consistent timing
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * PHYSICAL ANALOGY
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Think of chopper mode like a thermostat controlling room temperature:
 *
 * WITHOUT CHOPPING (bad):
 *   • Heater always on at full power
 *   • Room gets too hot
 *   • No temperature control
 *   • Energy wasted
 *
 * WITH CHOPPING (good):
 *   • Heater cycles on/off
 *   • ON: Temperature rises toward target
 *   • OFF: Temperature falls toward target
 *   • Result: Temperature oscillates around target (acceptable range)
 *   • Average temperature = target temperature
 *
 * Similarly for motor current:
 *   • ON: Current rises toward target
 *   • OFF: Current falls toward target
 *   • Result: Current oscillates around target (current ripple)
 *   • Average current = target current (IRUN setting)
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * SpreadCycle & Classic Off-Time Chopper Configuration (CHOPCONF Register 0x38)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * The chopper controls motor current by switching between on and off phases.
 * SpreadCycle is an intelligent adaptive chopper that provides smooth torque
 * and efficient operation. Classic mode offers constant off-time operation.
 *
 * Key Concepts:
 *   • Chopper cycles control motor current through PWM-like switching
 *   • TOFF sets the off-time (slow decay duration)
 *   • TBL masks switching noise from comparator measurements
 *   • CHM selects SpreadCycle (adaptive) or classic constant off-time mode
 *   • HSTRT/HEND configure hysteresis window for SpreadCycle mode
 *
 * Note: These parameters primarily affect SpreadCycle mode.
 *       StealthChop2 uses different control (PWMCONF register).
 */

/**
 * TOFF - Slow-Decay Off-Time Setting - Bits 3:0
 *
 * Sets the slow-decay duration (the "off-time") for the chopper.
 * This indirectly limits the maximum chopper frequency.
 *
 * CRITICAL: TOFF is required to enable the motor driver!
 *   TOFF = 0 → All driver transistors shut off (chopper disabled)
 *              Motor enters freewheel mode (no torque)
 *              Driver is effectively OFF
 *
 *   TOFF = 1-15 → Chopper active, off-time calculated as:
 *                 Off-time = 24 + (32 × TOFF) clock cycles
 *
 * Timing examples (with f_CLK = 12.5 MHz, t_CLK = 80 ns):
 *   ┌──────┬────────────────┬──────────────┬──────────────────┐
 *   │ TOFF │ Clock Cycles   │ Time (µs)    │ Max Freq (kHz)   │
 *   ├──────┼────────────────┼──────────────┼──────────────────┤
 *   │  0   │ Driver OFF     │ -            │ -                │
 *   │  1   │ 24 + 32 = 56   │ ~4.5 µs      │ ~222 kHz         │
 *   │  2   │ 24 + 64 = 88   │ ~7.0 µs      │ ~143 kHz         │
 *   │  3   │ 24 + 96 = 120  │ ~9.6 µs      │ ~104 kHz         │
 *   │  4   │ 24 + 128 = 152 │ ~12.2 µs     │ ~82 kHz          │
 *   │  5   │ 24 + 160 = 184 │ ~14.7 µs     │ ~68 kHz          │
 *   │ ...  │ ...            │ ...          │ ...              │
 *   │ 15   │ 24 + 480 = 504 │ ~40.3 µs     │ ~25 kHz          │
 *   └──────┴────────────────┴──────────────┴──────────────────┘
 *
 * Selection guide:
 *   Low TOFF (1-3):
 *     • Higher chopper frequency (>100 kHz)
 *     • Faster current regulation
 *     • More switching losses
 *     • Better for high-speed operation
 *
 *   Medium TOFF (4-7):
 *     • Balanced frequency (50-80 kHz)
 *     • Good efficiency and performance
 *     • Recommended for most applications
 *
 *   High TOFF (8-15):
 *     • Lower chopper frequency (<50 kHz)
 *     • Reduced switching losses
 *     • May be audible
 *     • Better for high-current, low-speed applications
 *
 * StealthChop2 consideration:
 *   When using StealthChop2 (EN_PWM_MODE=1), TOFF is NOT part of StealthChop
 *   control logic, but it must still be set to a non-zero value (typically 3-5)
 *   to enable the motor driver outputs.
 *
 * Range: 0-15
 * Default: 0 (driver disabled - must be configured!)
 * Recommended: 3-5 for most applications
 */
#define TMC522X_CHOPCONF_TOFF_MASK          GENMASK(3, 0)

/**
 * HSTRT - Hysteresis Start Offset (SpreadCycle Mode) - Bits 6:4
 *
 * Sets the hysteresis start level for SpreadCycle mode (CHM=0).
 * This value is added to HEND to determine the upper point of the hysteresis window.
 *
 * Operation:
 *   Hysteresis start value = HEND + HSTRT + 1
 *
 *   This creates a hysteresis window between:
 *     Lower threshold: HEND
 *     Upper threshold: HEND + HSTRT + 1
 *
 * SpreadCycle behavior:
 *   • Current reaches upper threshold → Fast decay starts
 *   • Current reaches lower threshold → Slow decay/on phase starts
 *   • Wider window (higher HSTRT) → More current ripple, smoother torque
 *   • Narrower window (lower HSTRT) → Less ripple, faster regulation
 *
 * Tuning guide:
 *   HSTRT = 0-2:
 *     • Narrow hysteresis window
 *     • Fast current regulation
 *     • Lower current ripple
 *     • May cause instability with some motors
 *
 *   HSTRT = 3-5:
 *     • Medium hysteresis window
 *     • Balanced performance
 *     • Recommended for most applications
 *
 *   HSTRT = 6-7:
 *     • Wide hysteresis window
 *     • Slower regulation
 *     • Higher current ripple
 *     • Smoother torque, less resonance
 *
 * Constraint (for typical current amplitudes ≤ 240):
 *   HSTRT + HEND ≤ 16
 *
 * Range: 0-7
 * Default: 5
 *
 * Note: Only used when CHM=0 (SpreadCycle mode).
 *       For classic mode (CHM=1), these bits configure fast decay time.
 */
#define TMC522X_CHOPCONF_HSTRT_TFD210_MASK  GENMASK(6, 4)

/**
 * HEND - Hysteresis End Level (SpreadCycle Mode) - Bits 10:7
 *
 * Defines the hysteresis end value (lower threshold) for SpreadCycle mode (CHM=0).
 * This determines when the hysteresis decrement phase ends.
 *
 * Value interpretation (4-bit field with special encoding):
 *   ┌──────────┬─────────────┬────────────────────────────┐
 *   │ Register │ Actual      │ Description                │
 *   │ Value    │ HEND Value  │                            │
 *   ├──────────┼─────────────┼────────────────────────────┤
 *   │ 0        │ -3          │ Negative HEND              │
 *   │ 1        │ -2          │ Negative HEND              │
 *   │ 2        │ -1          │ Negative HEND              │
 *   │ 3        │  0          │ Zero hysteresis end        │
 *   │ 4        │ +1          │ Positive HEND              │
 *   │ 5        │ +2          │ Positive HEND              │
 *   │ 6        │ +3          │ Positive HEND              │
 *   │ ...      │ ...         │ ...                        │
 *   │ 15       │ +12         │ Positive HEND (maximum)    │
 *   └──────────┴─────────────┴────────────────────────────┘
 *
 * Conversion formula:
 *   If register_value <= 2:
 *     Actual HEND = register_value - 3  (gives -3, -2, -1)
 *   If register_value == 3:
 *     Actual HEND = 0
 *   If register_value >= 4:
 *     Actual HEND = register_value - 3  (gives +1, +2, ... +12)
 *
 * Effect on hysteresis window:
 *   Hysteresis window = (HEND + HSTRT + 1)
 *
 *   Negative HEND (0-2):
 *     • Smaller hysteresis window
 *     • Faster current regulation
 *     • Lower current ripple
 *     • May increase switching frequency
 *
 *   Zero HEND (3):
 *     • Minimal hysteresis
 *     • Very fast regulation
 *     • Lowest ripple
 *     • Default starting point
 *
 *   Positive HEND (4-15):
 *     • Larger hysteresis window
 *     • Smoother operation
 *     • Higher current ripple
 *     • Lower switching frequency
 *
 * Tuning guide:
 *   Start with HEND = 2-3 (actual value -1 to 0)
 *   Increase HEND if:
 *     • Motor shows resonance or vibration
 *     • Current regulation is too aggressive
 *     • Switching frequency is too high
 *
 *   Decrease HEND if:
 *     • Current ripple is too high
 *     • Need faster current response
 *     • Motor torque is uneven
 *
 * Constraint (for typical current amplitudes ≤ 240):
 *   HSTRT + HEND ≤ 16
 *
 * Range: 0-15 (register value)
 *        -3 to +12 (actual HEND value)
 * Default: 2 (actual HEND = -1)
 *
 * Note: Only used when CHM=0 (SpreadCycle mode).
 *       For classic mode (CHM=1), these bits configure sine wave offset.
 */
#define TMC522X_CHOPCONF_HEND_OFFSET_MASK   GENMASK(10, 7)

/**
 * ═══════════════════════════════════════════════════════════════════════════
 * CHM - Chopper Mode Selection - Bit 14
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Selects which chopper algorithm the driver uses for current control.
 * This fundamentally changes HOW the driver regulates motor current.
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │  CHM = 0: SpreadCycle (Adaptive Hysteresis Chopper)                     │
 * │  CHM = 1: Classic (Constant Off-Time Chopper)                           │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * SPREADCYCLE MODE (CHM = 0) - Recommended
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * HOW IT WORKS:
 * ────────────
 * SpreadCycle is an intelligent adaptive chopper that automatically adjusts
 * its behavior based on motor inductance, load, and speed. It uses a variable
 * chopper frequency with hysteresis control.
 *
 * Algorithm overview:
 *   1. Current ON: MOSFETs turn on, current rises
 *   2. Upper threshold reached: MOSFETs turn off, slow decay begins
 *   3. Slow decay for TOFF duration (fixed time)
 *   4. After TOFF: Switch to fast decay mode
 *   5. Fast decay until lower threshold reached
 *   6. Repeat from step 1
 *
 * Current waveform (SpreadCycle):
 *   Current
 *     ↑     Upper threshold (IRUN + HSTRT)
 *     │    /‾\      /‾\      /‾\
 *     │   /   \    /   \    /   \     ← Variable rise/fall times
 *     │  /     \  /     \  /     \      (adapts to motor)
 *     │ /       \/       \/       \
 *     │         Lower threshold (IRUN - HEND)
 *     └─────────────────────────────────→ Time
 *       │←ON→│←─────OFF─────→│
 *            │←TOFF→│←Fast→│
 *            (slow)  (decay)
 *
 * Key features:
 *   • ADAPTIVE: Frequency varies with motor inductance and load
 *   • TWO-PHASE DECAY: Slow decay (TOFF), then fast decay (until threshold)
 *   • HYSTERESIS CONTROL: Upper/lower thresholds set by HSTRT and HEND
 *   • INTELLIGENT: Reduces resonance by spreading chopping energy
 *   • LOAD-RESPONSIVE: Automatically adjusts to changing conditions
 *
 * Parameters used in SpreadCycle (CHM=0):
 *   ┌──────────┬─────────────────────────────────────────────────────┐
 *   │ TOFF     │ Slow decay duration (fixed time)                    │
 *   │ HSTRT    │ Hysteresis start (sets upper current threshold)     │
 *   │ HEND     │ Hysteresis end (sets lower current threshold)       │
 *   │ TBL      │ Blanking time (noise filtering)                     │
 *   └──────────┴─────────────────────────────────────────────────────┘
 *
 * Advantages of SpreadCycle:
 *   ✓ Quieter operation (reduces resonance)
 *   ✓ Smoother torque production
 *   ✓ Better EMI characteristics (spread spectrum effect)
 *   ✓ Self-adapts to motor and load
 *   ✓ Works with StallGuard2 for load detection
 *   ✓ Recommended by Trinamic for most applications
 *   ✓ Reduces mid-range resonance issues
 *
 * Disadvantages:
 *   ✗ Slightly more parameters to tune
 *   ✗ Variable frequency may be harder to predict
 *   ✗ Requires understanding of HSTRT/HEND interaction
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * CLASSIC MODE (CHM = 1) - Traditional
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * HOW IT WORKS:
 * ────────────
 * Classic mode is a traditional constant off-time chopper. It maintains a
 * fixed off-time (TOFF) and uses fast decay for current regulation.
 *
 * Algorithm overview:
 *   1. Current ON: MOSFETs turn on, current rises
 *   2. Upper threshold reached: MOSFETs turn off
 *   3. Fast decay for TOFF duration (fixed time)
 *   4. TOFF complete: Back to step 1
 *   5. Frequency = f(TOFF, motor inductance, load)
 *
 * Current waveform (Classic):
 *   Current
 *     ↑     Upper threshold (IRUN + settings)
 *     │    /|\     /|\     /|\
 *     │   / | \   / | \   / | \    ← Faster, steeper decay
 *     │  /  |  \ /  |  \ /  |  \     (fast decay mode)
 *     │ /   |   V   |   V   |   \
 *     │     │←─TOFF→│←─TOFF→│
 *     └─────────────────────────────────→ Time
 *       │ON│ OFF │ON│ OFF │ON│ OFF
 *           (fast)
 *
 * Key features:
 *   • CONSTANT OFF-TIME: TOFF sets fixed decay duration
 *   • FAST DECAY: Uses fast decay mode during off-time
 *   • SIMPLE: Direct relationship between TOFF and frequency
 *   • PREDICTABLE: Constant timing behavior
 *   • TRADITIONAL: Standard chopper algorithm
 *
 * Parameters used in Classic mode (CHM=1):
 *   ┌──────────┬─────────────────────────────────────────────────────┐
 *   │ TOFF     │ Fixed off-time (directly sets chopper frequency)    │
 *   │ TFD      │ Fast decay time (replaces HSTRT functionality)      │
 *   │ OFFSET   │ Sine wave offset (replaces HEND functionality)      │
 *   │ TBL      │ Blanking time (noise filtering)                     │
 *   └──────────┴─────────────────────────────────────────────────────┘
 *
 * Note: When CHM=1, the HSTRT and HEND bit fields are reused:
 *   • HSTRT bits → TFD (fast decay time setting)
 *   • HEND bits → OFFSET (sine wave offset)
 *
 * Advantages of Classic mode:
 *   ✓ Simple, predictable behavior
 *   ✓ Direct frequency control via TOFF
 *   ✓ Easier to understand and tune
 *   ✓ Good for well-characterized motors
 *   ✓ Legacy compatibility
 *
 * Disadvantages:
 *   ✗ May exhibit more resonance at certain speeds
 *   ✗ Less adaptive to load changes
 *   ✗ Not as smooth as SpreadCycle
 *   ✗ Higher EMI (fixed frequency)
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * SPREADCYCLE vs CLASSIC - Side-by-Side Comparison
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * ┌──────────────────┬──────────────────────┬──────────────────────────┐
 * │ Feature          │ SpreadCycle (CHM=0)  │ Classic (CHM=1)          │
 * ├──────────────────┼──────────────────────┼──────────────────────────┤
 * │ Algorithm        │ Adaptive hysteresis  │ Constant off-time        │
 * │ Frequency        │ Variable (adaptive)  │ More constant            │
 * │ Decay mode       │ Slow → Fast (hybrid) │ Fast decay only          │
 * │ Off-time         │ TOFF (slow decay)    │ TOFF (total off-time)    │
 * │ Thresholds       │ HSTRT/HEND           │ TFD/OFFSET               │
 * │ Resonance        │ Reduced (spread)     │ May be higher            │
 * │ Smoothness       │ Smoother torque      │ Standard torque          │
 * │ EMI              │ Better (spread)      │ Standard                 │
 * │ Tuning           │ Medium complexity    │ Simpler                  │
 * │ Adaptability     │ Self-adjusting       │ Fixed behavior           │
 * │ StallGuard2      │ Full support         │ Full support             │
 * │ Recommended      │ Most applications    │ Special cases            │
 * └──────────────────┴──────────────────────┴──────────────────────────┘
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * WHEN TO USE EACH MODE
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Use SpreadCycle (CHM=0) for:
 *   ✓ General-purpose stepper applications
 *   ✓ Motors with resonance issues
 *   ✓ Applications requiring quiet operation
 *   ✓ Variable load conditions
 *   ✓ When using StallGuard2 for sensorless homing
 *   ✓ 3D printers, CNC machines, robotics
 *   ✓ When you want "set and forget" operation
 *
 * Use Classic mode (CHM=1) for:
 *   ✓ Well-characterized motor systems
 *   ✓ When you need precise frequency control
 *   ✓ Legacy system compatibility
 *   ✓ Simpler tuning requirements
 *   ✓ Applications with constant load
 *   ✓ When SpreadCycle behavior is unwanted
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * PRACTICAL EXAMPLE
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Scenario: 3D printer Z-axis motor showing mid-range resonance
 *
 * Problem with Classic mode (CHM=1):
 *   • Motor vibrates at certain speeds (resonance)
 *   • Print quality suffers (layer artifacts)
 *   • Fixed frequency excites motor resonance
 *
 * Solution with SpreadCycle (CHM=0):
 *   • Variable frequency avoids resonance
 *   • Smoother operation across speed range
 *   • Better print quality
 *   • Self-adapts to different layer heights (variable load)
 *
 * Configuration:
 *   CHM = 0         // Enable SpreadCycle
 *   TOFF = 3        // Medium off-time
 *   HSTRT = 4       // Medium hysteresis start
 *   HEND = 2        // Small hysteresis end (actual = -1)
 *   TBL = 2         // 36 clock cycles blanking
 *
 * Result: Smooth, quiet Z-axis operation with no resonance issues
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Range: 0 or 1
 * Default: 0 (SpreadCycle)
 * Recommended: 0 (SpreadCycle) for most applications
 *
 * Note: The TMC522x driver currently defaults to CHM=0 (SpreadCycle mode)
 *       as this provides the best performance for general applications.
 */
#define TMC522X_CHOPCONF_CHM_MASK           BIT(14)

/**
 * TBL - Comparator Blanking Time - Bits 16:15
 *
 * Sets the duration for which the current measurement comparator ignores
 * switching noise after a MOSFET switching event.
 *
 * Purpose:
 *   When MOSFETs switch on or off, they create voltage/current transients
 *   (ringing) that can cause false comparator triggers. TBL "blanks" the
 *   comparator during this noise period to ensure accurate current measurement.
 *
 * Timing values:
 *   ┌───────┬─────────────┬────────────────┬─────────────────────┐
 *   │ Value │ Clock Cycles│ Time (µs)      │ Recommended Use     │
 *   │       │ (tCLK)      │ @ 12.5 MHz CLK │                     │
 *   ├───────┼─────────────┼────────────────┼─────────────────────┤
 *   │  0    │ 20          │ ~1.6 µs        │ Low-noise loads     │
 *   │  1    │ 28          │ ~2.2 µs        │ Standard (default)  │
 *   │  2    │ 36          │ ~2.9 µs        │ Most applications   │
 *   │  3    │ 54          │ ~4.3 µs        │ Capacitive loads    │
 *   └───────┴─────────────┴────────────────┴─────────────────────┘
 *
 * Selection criteria:
 *   TBL = 0 (20 tCLK):
 *     • Shortest blanking time
 *     • For very clean switching (low capacitance)
 *     • Faster current regulation possible
 *     • Risk of noise-induced false triggers
 *
 *   TBL = 1 (28 tCLK):
 *     • Short blanking time
 *     • Standard motor connections
 *     • Good for typical applications
 *     • Balanced speed vs. noise immunity
 *
 *   TBL = 2 (36 tCLK) - RECOMMENDED:
 *     • Medium blanking time
 *     • Best for most applications
 *     • Good noise immunity
 *     • Handles moderate capacitance well
 *     • Industry standard setting
 *
 *   TBL = 3 (54 tCLK):
 *     • Longest blanking time
 *     • For highly capacitive loads
 *     • Long motor cables
 *     • External filters on motor phases
 *     • Maximum noise immunity
 *     • Slightly slower current regulation
 *
 * When to increase TBL:
 *   • Using long motor cables (>2 meters)
 *   • External EMI filters on motor phases
 *   • Observing current regulation instability
 *   • High electrical noise environment
 *   • Motors with high winding capacitance
 *
 * When to decrease TBL:
 *   • Very short motor connections (<30 cm)
 *   • Need fastest possible current regulation
 *   • Low-noise, controlled environment
 *   • Using shielded cables and good grounding
 *
 * Range: 0-3
 * Default: 2 (36 tCLK, ~2.9 µs)
 * Recommended: 1 or 2 for most applications
 *              2 or 3 for capacitive loads or long cables
 */
#define TMC522X_CHOPCONF_TBL_MASK           GENMASK(16, 15)

/**
 * VHIGHFS - High-Velocity Fullstep Selection - Bit 18
 *
 * Enables automatic switch to fullstep mode at high velocities (when TSTEP ≤ THIGH).
 *
 * Settings:
 *   0 = Normal operation at high velocity
 *   1 = Switch to fullstep mode when TSTEP ≤ THIGH
 *
 * Note: Primarily used for very high-speed applications where microstep
 *       resolution is not needed and fullstep provides better performance.
 */
#define TMC522X_CHOPCONF_VHIGHFS_MASK       BIT(18)

/**
 * VHIGHCHM - High-Velocity Chopper Mode - Bit 19
 *
 * Forces chopper to constant off-time mode at high velocities (when TSTEP ≤ THIGH).
 *
 * Settings:
 *   0 = Keep current chopper mode at high velocity
 *   1 = Switch to CHM=1 (classic mode) when TSTEP ≤ THIGH
 *
 * Note: Can improve performance at very high speeds by using simpler chopper.
 */
#define TMC522X_CHOPCONF_VHIGHCHM_MASK      BIT(19)

/**
 * MRES - Microstep Resolution - Bits 27:24
 *
 * Sets the microstep resolution for the motor.
 * See TMC522X_MRES_* constants for values.
 *
 * Range: 0-8 (256 to 1 microsteps per full step)
 */
#define TMC522X_CHOPCONF_MRES_MASK          GENMASK(27, 24)

/**
 * DRV_EN_SW - Software Driver Enable - Bit 9
 *
 * Software control for enabling/disabling the driver.
 *
 * Settings:
 *   0 = Driver disabled (outputs in high-impedance)
 *   1 = Driver enabled (normal operation)
 */
#define TMC522X_CHOPCONF_DRV_EN_SW_MASK     BIT(9)
/** @} */

/**
 * @name TMC522X COOLCONF register fields
 * @{
 */
/**
 * ═══════════════════════════════════════════════════════════════════════════
 * CoolStep Load-Adaptive Current Control
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * CoolStep automatically adjusts motor current based on actual load, reducing
 * power consumption and heat while maintaining torque when needed.
 *
 * How CoolStep Works:
 * ───────────────────
 *   1. Monitors StallGuard load measurement (SG_RESULT or SG4_RESULT)
 *   2. Compares against lower threshold (SEMIN) and upper threshold (SEMAX)
 *   3. Adjusts CS_ACTUAL (current scale) dynamically:
 *      - Load HIGH (SG_RESULT < SEMIN × 32):     Increase current
 *      - Load LOW (SG_RESULT > threshold):       Decrease current
 *      - Load in range:                          Hold current
 *
 * Prerequisites:
 * ──────────────
 *   1. StallGuard must be tuned first (SGT for SG2, or SG4_THRS for SG4)
 *   2. TCOOLTHRS must be < 0xFFFFF (enables StallGuard/CoolStep)
 *   3. SEMIN must be > 0 (enables CoolStep)
 *   4. Motor velocity must be: TCOOLTHRS < TSTEP < THIGH
 *
 * Tuning Sequence:
 * ────────────────
 *   1. Tune StallGuard threshold (SGT or SG4_THRS) first
 *   2. Set SEMIN to define minimum acceptable load threshold
 *   3. Set SEMAX to define maximum load range
 *   4. Adjust SEUP for current increase speed
 *   5. Adjust SEDN for current decrease speed
 *   6. Set SEIMIN based on StealthChop requirements
 *   7. Define velocity range with TCOOLTHRS and THIGH
 *
 * Typical Application:
 * ────────────────────
 *   Use CoolStep during constant-velocity operation where load varies.
 *   Disable during acceleration/deceleration (set TCOOLTHRS = VMAX temporarily).
 *   Full current needed during dynamic phases (small power contribution).
 */

/**
 * SEMIN (Lower StallGuard Threshold for CoolStep) - Bits 3:0
 *
 * Sets the lower threshold for StallGuard load measurement.
 * When SG_RESULT falls below this threshold, CoolStep increases motor current.
 *
 * Threshold calculation:
 *   Lower threshold = SEMIN × 32
 *
 *   This scales SEMIN to align with the lower half of the 10-bit SG_RESULT
 *   range (0-1023), providing appropriate sensitivity for load detection.
 *
 * Operation:
 *   - SG_RESULT < (SEMIN × 32): Motor under HIGH load → Increase current
 *   - SG_RESULT ≥ (SEMIN × 32): Motor load acceptable → Current stable or decreasing
 *
 * Special value:
 *   SEMIN = 0: CoolStep DISABLED (motor runs at fixed IRUN current)
 *
 * Tuning guide:
 *   - Start with SEMIN = 5 (threshold at 160)
 *   - Monitor SG_RESULT at target load
 *   - Increase SEMIN if current increases too frequently
 *   - Decrease SEMIN if motor needs more torque headroom
 *   - Typical range: 2-10 for most applications
 *
 * Range: 0-15 (4-bit)
 * Default: 0 (CoolStep disabled)
 *
 * Note: Works with both StallGuard2 (SG_RESULT) and StallGuard4 (SG4_RESULT)
 */
#define TMC522X_COOLCONF_SEMIN_MASK   GENMASK(3, 0)

/**
 * SEMAX (Upper StallGuard Threshold for CoolStep) - Bits 7:6
 *
 * Sets the upper threshold offset for StallGuard load measurement.
 * When SG_RESULT exceeds upper threshold, CoolStep decreases motor current.
 *
 * Threshold calculation:
 *   Upper threshold = (SEMIN + SEMAX + 1) × 32
 *
 *   This creates a hysteresis window between lower and upper thresholds,
 *   preventing current oscillation and providing stable operation.
 *
 * Operation window:
 *   ┌──────────────────────────────────────────────┐
 *   │ SG_RESULT Range         │ CoolStep Action    │
 *   ├─────────────────────────┼────────────────────┤
 *   │ < SEMIN × 32            │ Increase current   │
 *   │ SEMIN×32 to (SEMIN+     │ Hold current       │
 *   │   SEMAX+1)×32           │ (hysteresis band)  │
 *   │ > (SEMIN+SEMAX+1) × 32  │ Decrease current   │
 *   └─────────────────────────┴────────────────────┘
 *
 * Tuning guide:
 *   - SEMAX = 0: Narrow window (32-step hysteresis)
 *   - SEMAX = 15: Wide window (512-step hysteresis)
 *   - Larger SEMAX = more stable, less responsive
 *   - Smaller SEMAX = more responsive, may oscillate
 *   - Typical range: 2-5 for most applications
 *
 * Example:
 *   SEMIN = 5, SEMAX = 2:
 *     Lower threshold = 5 × 32 = 160
 *     Upper threshold = (5+2+1) × 32 = 256
 *     Hysteresis window = 96 (256-160)
 *
 * Range: 0-15 (4-bit)
 * Default: 0
 */
#define TMC522X_COOLCONF_SEMAX_MASK   GENMASK(7, 4)

/**
 * SEUP (Current Increase Step Size) - Bits 10:8
 *
 * Defines the step size for current increases when load is HIGH.
 * Each time SG_RESULT < (SEMIN × 32), CS_ACTUAL is incremented by this step.
 *
 * Step size encoding:
 *   ┌──────┬───────────┬─────────────────────────┐
 *   │ SEUP │ Step Size │ Response Characteristics│
 *   ├──────┼───────────┼─────────────────────────┤
 *   │  0   │     1     │ Slowest, smoothest      │
 *   │  1   │     2     │ Moderate                │
 *   │  2   │     4     │ Fast                    │
 *   │  3   │     8     │ Fastest, most aggressive│
 *   └──────┴───────────┴─────────────────────────┘
 *
 * Selection criteria:
 *   - Fast load changes (sudden obstacles): SEUP = 2 or 3
 *   - Slow load changes (gradual resistance): SEUP = 0 or 1
 *   - Risk of stall if too slow: Use higher SEUP
 *   - Risk of oscillation if too fast: Use lower SEUP
 *
 * Response time consideration:
 *   With SFILT=1 (filtered), measurement rate is 4x slower.
 *   Compensate by using higher SEUP value.
 *
 * Range: 0-3 (encoded as 1, 2, 4, 8)
 * Default: 0 (step size = 1)
 *
 * Note: Current increase is faster than decrease because crossing lower
 *       threshold is more critical (risk of stall vs missed power savings)
 */
#define TMC522X_COOLCONF_SEUP_MASK    GENMASK(9, 8)

/**
 * SG_STOP (Stop Motor on StallGuard Event) - Bit 10
 *
 * Enables automatic motor stop when stall is detected.
 * WARNING: Shares bit position with SEUP - implementation must handle carefully!
 *
 * Operation:
 *   When enabled and stall detected:
 *     1. Ramp generator stops motion
 *     2. VACTUAL set to zero
 *     3. EVENT_STOP_SG flag set in RAMP_STAT
 *     4. STALLGUARD flag set in DRV_STATUS
 *
 * Prerequisites:
 *   - StallGuard must be enabled (TCOOLTHRS < 0xFFFFF)
 *   - Motor must be in valid StallGuard velocity range
 *
 * To restart after stall:
 *   - Clear EVENT_STOP_SG in RAMP_STAT (write 1 to clear)
 *   - Or modify motion parameters to prevent immediate restart
 *
 * Applications:
 *   - Sensorless homing (detect hard stop)
 *   - Collision detection
 *   - Overload protection
 *
 * Note: In SpreadCycle use SFILT=0 for fastest stall detection
 */
#define TMC522X_COOLCONF_SG_STOP_MASK BIT(10)

/**
 * SEDN (Current Decrease Speed) - Bits 14:13
 *
 * Defines how many StallGuard measurements showing LOW load are required
 * before decreasing current. Provides stability filtering.
 *
 * Measurement count encoding:
 *   ┌──────┬──────────────┬─────────────────────────┐
 *   │ SEDN │ Measurements │ Response Characteristics│
 *   ├──────┼──────────────┼─────────────────────────┤
 *   │  0   │      32      │ Slowest, most stable    │
 *   │  1   │       8      │ Moderate                │
 *   │  2   │       2      │ Fast                    │
 *   │  3   │       1      │ Fastest, most responsive│
 *   └──────┴──────────────┴─────────────────────────┘
 *
 * Selection criteria:
 *   - Stable loads: SEDN = 0 or 1 (avoid current hunting)
 *   - Variable loads: SEDN = 2 or 3 (quick power savings)
 *   - Motor oscillation observed: Increase SEDN (slower decrease)
 *   - Slow power savings: Decrease SEDN (faster decrease)
 *
 * Asymmetric response:
 *   Current INCREASES faster than DECREASES by design:
 *     - Crossing lower threshold = risk of stall (critical)
 *     - Crossing upper threshold = missed power savings (non-critical)
 *
 *   This ensures torque availability takes priority over power optimization.
 *
 * Range: 0-3 (encoded as 32, 8, 2, 1 measurements)
 * Default: 0 (32 measurements required)
 */
#define TMC522X_COOLCONF_SEDN_MASK    GENMASK(14, 13)

/**
 * SEIMIN (Minimum Current Limit) - Bit 15
 *
 * Sets the minimum allowed current for CoolStep operation as a fraction of IRUN.
 * Prevents CoolStep from reducing current below safe operating limits.
 *
 * Settings:
 *   ┌────────┬──────────────┬─────────────────────────────────┐
 *   │ SEIMIN │ Minimum      │ Requirements                    │
 *   ├────────┼──────────────┼─────────────────────────────────┤
 *   │   0    │ ½ of IRUN    │ Valid when IRUN ≥ 16            │
 *   │        │              │ Allows more current reduction   │
 *   │        │              │ Good for SpreadCycle            │
 *   ├────────┼──────────────┼─────────────────────────────────┤
 *   │   1    │ ¼ of IRUN    │ Required when IRUN ≥ 28 for     │
 *   │        │              │ StealthChop2 (PWM stability)    │
 *   │        │              │ Maximum power savings           │
 *   └────────┴──────────────┴─────────────────────────────────┘
 *
 * Selection guide:
 *   SpreadCycle mode:
 *     - Use SEIMIN = 0 (½ IRUN minimum)
 *     - Provides adequate torque margin
 *     - Better for high-load applications
 *
 *   StealthChop2 mode:
 *     - Use SEIMIN = 1 (¼ IRUN minimum) when IRUN ≥ 28
 *     - Required for PWM regulation stability
 *     - Maximum energy savings
 *     - Use SEIMIN = 0 when IRUN < 28
 *
 * Current scaling relationship:
 *   CS_ACTUAL ranges from 0-31 (representing 1/32 to 32/32 of IRUN)
 *
 *   SEIMIN = 0: CS_ACTUAL minimum = 16 (½ of IRUN, 50%)
 *   SEIMIN = 1: CS_ACTUAL minimum =  8 (¼ of IRUN, 25%)
 *
 * Range: 0 or 1
 * Default: 0 (½ IRUN minimum)
 */
#define TMC522X_COOLCONF_SEIMIN_MASK  BIT(15)

/* StallGuard2 configuration (bits 16-24) */
/**
 * SGT (StallGuard2 Threshold) - Bits 22:16
 *
 * Signed threshold adjustment for StallGuard2 load detection in SpreadCycle mode.
 * SGT compensates for resistive losses and adjusts the sensitivity of stall detection.
 *
 * How it works:
 *   - SGT is added as an offset to the raw StallGuard2 measurement
 *   - Higher SGT → Less sensitive (allows more load before SG_RESULT reaches 0)
 *   - Lower SGT  → More sensitive (reaches 0 at lower loads)
 *
 * Initial Tuning Procedure:
 *   1. Start with SGT = 0 (neutral setting)
 *   2. Run motor at normal operation velocity/current/voltage
 *   3. Monitor SG_RESULT while applying increasing mechanical load
 *   4. If motor stalls before SG_RESULT reaches 0: decrease SGT (more negative)
 *   5. If SG_RESULT reaches 0 before motor stalls: increase SGT (more positive)
 *   6. Enable SG_STOP and verify motor stops safely on stall
 *   7. Fine-tune: Optimal when SG_RESULT = 0-100 at maximum load before stall
 *   8. Verify across velocity range (80%-120% of target) and temperature extremes
 *
 * Automatic Tuning (Low Velocity Method):
 *   1. Run motor at very low velocity (<10 RPM, few full steps/sec)
 *   2. Enable SFILT (filtered mode)
 *   3. Increase SGT from 0 until SG_RESULT starts rising
 *   4. Reduce to highest value where SG_RESULT stays at 0
 *   5. This gives most sensitive setting for useful stall detection
 *
 * Application notes:
 *   - Use TCOOLTHRS to define velocity range where StallGuard2 is active
 *   - Use THIGH to define upper velocity limit
 *   - Temperature correction may be needed (coil resistance increases with temp)
 *   - Measurement accuracy: ± max(1, |SGT|)
 *
 * Value interpretation (7-bit signed, two's complement):
 *   0          = Neutral threshold (starting point for tuning)
 *   +1 to +63  = Less sensitive (increases measurement range)
 *   -1 to -64  = More sensitive (earlier stall detection)
 *
 * Range: -64 to +63
 */
#define TMC522X_COOLCONF_SGT_MASK     GENMASK(22, 16)

/**
 * SFILT (StallGuard2 Filter) - Bit 24
 *
 * Enables precision filtering for StallGuard2 measurements.
 *
 * Filter behavior:
 *   - When enabled: Averages 4 load measurements (one per electrical period)
 *   - Update rate: One filtered result per 4 full steps
 *   - Compensates for motor construction variations (e.g., magnet misalignment)
 *
 * When to use:
 *   SFILT = 1 (Filtered mode):
 *     - High-precision load measurement required
 *     - Using CoolStep current control
 *     - Motor has high detent torque or construction variations
 *     - Stable, noise-free measurement more important than fast response
 *
 *   SFILT = 0 (Standard mode):
 *     - Rapid response to load changes required
 *     - Sensorless homing applications
 *     - Hard obstacle detection
 *     - Fast stall detection needed
 *
 * Settings:
 *   0 = Standard mode (faster, unfiltered, one reading per full step)
 *   1 = Filtered mode (slower, more precise, averaged over 4 full steps)
 *
 * Recommended: Enable (1) for CoolStep and precision applications
 *              Disable (0) for fast stall detection and homing
 */
#define TMC522X_COOLCONF_SFILT_MASK   BIT(24)

/* Helper macros for field extraction and conversion */
/**
 * Extract SGT value from COOLCONF register and convert to signed value.
 * SGT is stored as 7-bit two's complement in bits 22:16.
 *
 * @param coolconf_val The raw COOLCONF register value
 * @return Signed SGT value (-64 to +63)
 */
#define TMC522X_GET_SGT(coolconf_val) \
	((int8_t)(((coolconf_val) & TMC522X_COOLCONF_SGT_MASK) >> 16))

/**
 * Prepare SGT value for writing to COOLCONF register.
 * Converts signed value to 7-bit two's complement format.
 *
 * @param sgt_val Signed SGT value (-64 to +63)
 * @return Masked and shifted value ready for COOLCONF register
 */
#define TMC522X_SET_SGT(sgt_val) \
	(((uint32_t)(sgt_val) << 16) & TMC522X_COOLCONF_SGT_MASK)
/** @} */

/**
 * @name TMC522X StallGuard4 Register Fields
 *
 * StallGuard4 is optimized for StealthChop2 operation and provides accurate
 * load measurement and stall detection. Unlike StallGuard2 (which uses SGT
 * offset), StallGuard4 uses a comparison threshold (SG4_THRS).
 *
 * Key differences from StallGuard2:
 *   - StallGuard2: Works with SpreadCycle, uses SGT offset adjustment
 *   - StallGuard4: Works with StealthChop2, uses SG4_THRS threshold comparison
 *
 * StallGuard4 is active when: TCOOLTHRS ≥ TSTEP > TPWMTHRS
 * @{
 */

/**
 * SG4_THRS (0x3F) - StallGuard4 Threshold
 *
 * Threshold value for StallGuard4 stall detection.
 * Stall is detected when SG4_RESULT < SG4_THRS.
 *
 * Tuning procedure:
 *   1. Run motor at normal operating velocity
 *   2. Apply slowly increasing mechanical load until motor stalls
 *   3. Monitor lowest SG4_RESULT value before stall occurs
 *   4. Use this value as starting point for SG4_THRS
 *   5. Configure TCOOLTHRS to match lower velocity limit
 *   6. Monitor DIAGx output for stall pulses
 *   7. Adjust SG4_THRS if motor stops before actual stall
 *   8. Verify setting works across velocity range (80%-120% of target)
 *      and temperature extremes
 *
 * Sensitivity:
 *   Higher SG4_THRS = More sensitive (detects stall sooner, less torque needed)
 *   Lower SG4_THRS  = Less sensitive (allows more load before stall signal)
 *
 * Range: 0-511, Default: 0
 * Bits: 8:0
 */
#define TMC522X_SG4_THRS_MASK GENMASK(8, 0)

/**
 * SG4_RESULT (0x40) - StallGuard4 Load Measurement Result
 *
 * Read-only StallGuard4 load measurement value.
 * Updated every full step (or averaged over 4 measurements if SG4_FILT_EN=1).
 *
 * Value interpretation:
 *   High values (e.g., 300-510) = Low mechanical load, motor spinning freely
 *   Medium values (e.g., 100-300) = Normal operating load
 *   Low values (e.g., 1-100)     = High load, approaching stall
 *   0                             = Maximum load (stalled or about to stall)
 *
 * Load relationship:
 *   Higher SG4_RESULT → Lower load → Smaller load angle
 *   Lower SG4_RESULT  → Higher load → Larger load angle (up to 90°)
 *
 * Stall condition: SG4_RESULT < SG4_THRS triggers DIAGx pulse
 *
 * Range: 0-510 (slightly higher resolution than StallGuard2)
 * Bits: 8:0
 *
 * Note: Only valid when StallGuard4 is active (TCOOLTHRS ≥ TSTEP > TPWMTHRS)
 */
#define TMC522X_SG4_RESULT_MASK GENMASK(8, 0)

/**
 * SG4_IND (0x41) - StallGuard4 Individual Phase Measurements
 *
 * Contains individual per-phase StallGuard4 measurements used to calculate
 * SG4_RESULT. Available when SG4_FILT_EN=1 (filtered mode).
 *
 * Bit layout:
 *   Bits 7:0   - SG4_IND_0: Phase A falling zero transition (cosine falling)
 *   Bits 15:8  - SG4_IND_1: Phase A rising zero transition (cosine rising)
 *   Bits 23:16 - SG4_IND_2: Phase B falling zero transition (sine falling)
 *   Bits 31:24 - SG4_IND_3: Phase B rising zero transition (sine rising)
 *
 * When SG4_FILT_EN=0 (unfiltered):
 *   Only SG4_IND_0 is valid (updated every full step)
 *   SG4_RESULT = SG4_IND_0
 *
 * When SG4_FILT_EN=1 (filtered):
 *   All four SG4_IND_x values available
 *   SG4_RESULT = mean(SG4_IND_0, SG4_IND_1, SG4_IND_2, SG4_IND_3)
 *   Each full step contributes 25% to overall result
 *   Better for soft obstacle detection and imprecise motors
 *
 * Range per field: 0-255
 * Interpretation: Same as SG4_RESULT (low=high load, high=low load)
 */
#define TMC522X_SG4_IND_0_MASK GENMASK(7, 0)    /* Phase A falling */
#define TMC522X_SG4_IND_1_MASK GENMASK(15, 8)   /* Phase A rising */
#define TMC522X_SG4_IND_2_MASK GENMASK(23, 16)  /* Phase B falling */
#define TMC522X_SG4_IND_3_MASK GENMASK(31, 24)  /* Phase B rising */

/**
 * SG_PREWARN_CONF (0x42) - StallGuard Pre-Warning Configuration
 *
 * Optional PT1 low-pass filter for StallGuard results with separate threshold.
 * Takes SG_RESULT as input and provides filtered output with dedicated flag.
 *
 * Fields (bit positions may vary - check datasheet):
 *   SG_PREWARN_FILTER:  Filter length (0/2/4/8/16/32/64 samples)
 *   SG_PREWARN_THRSH:   Threshold for filtered result
 *   SG_PREWARN_RESULT:  Filtered StallGuard value (read-only)
 *   SG_PREWARN_RESULT_VALID: Filter output valid flag
 *
 * The STALLGUARD_PREWARN flag is available in:
 *   - DRV_STATUS register (bit 23)
 *   - Can be routed to DIAG0/1 outputs via DIAG_CONF
 *
 * Use case: Early warning before actual stall occurs
 */
/* Note: Individual field masks to be added based on datasheet bit positions */

/* Helper macros for field extraction */
/**
 * Extract SG4_RESULT from register value
 */
#define TMC522X_GET_SG4_RESULT(sg4_result_val) \
	((sg4_result_val) & TMC522X_SG4_RESULT_MASK)

/**
 * Extract individual phase measurements from SG4_IND register
 */
#define TMC522X_GET_SG4_IND_0(sg4_ind_val) \
	((sg4_ind_val) & TMC522X_SG4_IND_0_MASK)
#define TMC522X_GET_SG4_IND_1(sg4_ind_val) \
	(((sg4_ind_val) & TMC522X_SG4_IND_1_MASK) >> 8)
#define TMC522X_GET_SG4_IND_2(sg4_ind_val) \
	(((sg4_ind_val) & TMC522X_SG4_IND_2_MASK) >> 16)
#define TMC522X_GET_SG4_IND_3(sg4_ind_val) \
	(((sg4_ind_val) & TMC522X_SG4_IND_3_MASK) >> 24)
/** @} */

/**
 * @name TMC522X PWMCONF register fields - StealthChop2 Configuration
 * @{
 */

/**
 * ═══════════════════════════════════════════════════════════════════════════
 * StealthChop2 PWM Configuration (PWMCONF Register 0x3C)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * StealthChop2 is a silent PWM-based chopper mode that eliminates audible motor
 * noise by using voltage-mode PWM instead of current chopping. It's ideal for
 * applications requiring quiet operation.
 *
 * Key Features:
 *   • Silent operation (no chopper noise)
 *   • Lower vibration and resonance
 *   • Automatic current regulation
 *   • Works with StallGuard4 for load detection
 *
 * Enabling StealthChop2:
 *   1. Set EN_PWM_MODE bit in GCONF register (bit 0)
 *   2. Configure TPWMTHRS velocity threshold
 *   3. Set PWM parameters in PWMCONF register
 *   4. Motor should be at standstill during initialization
 *
 * Note: StealthChop2 activates when: TSTEP > TPWMTHRS
 */

/**
 * PWM_OFS - PWM Offset (Base Amplitude) - Bits 7:0
 *
 * Defines the base PWM amplitude for StealthChop2 before regulation kicks in.
 * This is the starting point for automatic amplitude adjustment.
 *
 * Operation:
 *   - Defines user-defined amplitude at zero velocity
 *   - Automatic tuning (PWM_OFS_AUTO) can fill this value
 *   - Used as baseline for PWM_GRAD slope adjustment
 *
 * Special values:
 *   0 = Disables linear current-scaling based on current setting
 *       (not recommended for normal operation)
 *
 * Tuning:
 *   - Start with default (~0x1D = 29)
 *   - Higher values = more aggressive startup
 *   - Lower values = smoother but may lack torque
 *   - Use PWM_OFS_AUTO readback for optimal value
 *
 * Range: 0-255
 * Default: 0x1D (29)
 */
#define TMC522X_PWMCONF_PWM_OFS_MASK         GENMASK(7, 0)

/**
 * PWM_GRAD - PWM Gradient (Amplitude Slope) - Bits 15:8
 *
 * Defines the amplitude gradient (slope) used during motor motion.
 * Controls how PWM amplitude increases with velocity.
 *
 * Operation:
 *   - Defines velocity-dependent amplitude increase
 *   - Used for preloading automatic tuning (PWM_AUTOGRAD)
 *   - Shapes dynamic amplitude response during motion
 *   - PWM amplitude = PWM_OFS + (PWM_GRAD × velocity_factor)
 *
 * Tuning:
 *   - Start with 0 for automatic gradient determination
 *   - Increase if motor loses steps at higher velocities
 *   - Decrease if excessive current at higher speeds
 *   - Use PWM_GRAD_AUTO readback for optimal value
 *
 * With PWM_AUTOGRAD=1:
 *   - Value is automatically determined and updated
 *   - Manual setting serves as initial guess
 *
 * Range: 0-255
 * Default: 0 (automatic tuning determines value)
 */
#define TMC522X_PWMCONF_PWM_GRAD_MASK        GENMASK(15, 8)

/**
 * PWM_REG - Regulation Loop Strength - Bits 11:8 (overlaps with PWM_GRAD upper bits)
 *
 * Note: This field's exact position may vary by TMC chip version.
 * On some TMC522x variants, this is a separate 4-bit field.
 *
 * Controls how aggressively the PWM amplitude regulator responds to current errors.
 * Higher values = faster adaptation to load changes, but may cause noise or overshoot.
 *
 * Operation:
 *   - Defines feedback loop gain for current regulation
 *   - Higher values: Faster response, risk of oscillation
 *   - Lower values: Slower response, more stable
 *
 * Tuning:
 *   - Start with default (4)
 *   - Increase if current regulation is too slow
 *   - Decrease if you observe current oscillation or noise
 *   - Typical range: 1-8 for most applications
 *
 * Range: 1-15 (0 not recommended)
 * Default: 4
 *
 * Note: Consult specific TMC522x datasheet for exact bit position
 */
/* Commented out - exact bit position varies by chip version
 * #define TMC522X_PWMCONF_PWM_REG_MASK         GENMASK(11, 8)
 */

/**
 * PWM_LIM - PWM Amplitude Limiter - Bits top 4 of amplitude
 *
 * Limits the maximum PWM amplitude when switching from SpreadCycle to StealthChop2.
 * Ensures smooth mode transitions without current spikes.
 *
 * Operation:
 *   - Sets upper limit for PWM amplitude during transitions
 *   - Affects top 4 bits of 8-bit PWM amplitude value
 *   - Prevents excessive current when switching chopper modes
 *
 * Tuning:
 *   - Default (12) works for most applications
 *   - Increase (13-15) for higher torque during transitions
 *   - Decrease (8-11) if you see current spikes when switching modes
 *
 * Range: 0-15
 * Default: 12
 *
 * Note: Primarily relevant for applications that switch between
 *       SpreadCycle and StealthChop2 during operation
 */
/* Commented out - exact bit position varies by chip version
 * #define TMC522X_PWMCONF_PWM_LIM_MASK         GENMASK(?, ?)
 */

/**
 * PWM_FREQ - PWM Frequency Selection - Bits 17:16
 *
 * Selects the PWM switching frequency for StealthChop2.
 * Higher frequencies reduce audible noise but may affect current regulation.
 *
 * Frequency calculation: fPWM = fCLK / (2 × divisor)
 *
 * Settings:
 *   ┌──────┬──────────┬─────────────────────────────────┐
 *   │ Value│ Divisor  │ Frequency (fCLK = 12.5 MHz)     │
 *   ├──────┼──────────┼─────────────────────────────────┤
 *   │  0   │  1024    │ ~6.1 kHz  (lowest frequency)    │
 *   │  1   │   683    │ ~9.2 kHz                        │
 *   │  2   │   512    │ ~12.2 kHz                       │
 *   │  3   │   410    │ ~15.2 kHz (highest frequency)   │
 *   └──────┴──────────┴─────────────────────────────────┘
 *
 * Selection criteria:
 *   - Lower frequency (0): Better current regulation, more audible
 *   - Higher frequency (2-3): Quieter, faster current response
 *   - Recommended: 0 or 1 for most applications
 *
 * Trade-offs:
 *   Higher frequency → Less audible noise, faster adaptation
 *   Lower frequency  → Better regulation quality, more stable
 *
 * Range: 0-3
 * Default: 0 (lowest frequency, best regulation)
 */
#define TMC522X_PWMCONF_FREQ_MASK            GENMASK(17, 16)

/**
 * PWM_AUTOSCALE - Automatic Current Scaling - Bit 18
 *
 * Enables automatic PWM amplitude adjustment to maintain target current.
 * This is the primary enable for StealthChop2's automatic regulation.
 *
 * Settings:
 *   0 = Forward-controlled mode (manual amplitude control)
 *       - PWM amplitude set by PWM_OFS and PWM_GRAD only
 *       - No automatic adjustment
 *       - Not recommended for normal operation
 *
 *   1 = Automatic scaling based on current regulation (recommended)
 *       - Driver automatically adjusts PWM amplitude
 *       - Maintains target current set by IRUN
 *       - Compensates for load, voltage, and temperature changes
 *       - Required for proper StealthChop2 operation
 *
 * Operation with autoscaling:
 *   - Monitors actual motor current
 *   - Compares against target (IRUN setting)
 *   - Adjusts PWM amplitude to match
 *   - Result visible in PWM_SCALE_SUM (read-only)
 *
 * Range: 0-1
 * Default: 1 (automatic scaling enabled)
 *
 * Note: Must be enabled for proper StealthChop2 operation
 */
#define TMC522X_PWMCONF_AUTOSCALE_MASK       BIT(18)

/**
 * PWM_AUTOGRAD - Automatic Gradient Tuning - Bit 19
 *
 * Enables automatic tuning of PWM_GRAD (velocity-dependent amplitude slope).
 *
 * Settings:
 *   0 = Manual gradient control
 *       - PWM_GRAD value used as-is from register
 *       - User must tune PWM_GRAD manually
 *       - Gradient does not adapt to motor characteristics
 *
 *   1 = Automatic gradient tuning (recommended)
 *       - Driver automatically determines optimal PWM_GRAD
 *       - Adapts to motor inductance and velocity
 *       - Result visible in PWM_GRAD_AUTO (read-only)
 *       - Manual PWM_GRAD serves as initial guess
 *
 * Automatic tuning process:
 *   - Runs during first few motor movements
 *   - Analyzes motor response at different velocities
 *   - Determines optimal gradient for smooth operation
 *   - Continuously adapts during operation
 *
 * Range: 0-1
 * Default: 1 (automatic tuning enabled)
 *
 * Note: Recommended to keep enabled for optimal performance
 */
#define TMC522X_PWMCONF_AUTOGRAD_MASK        BIT(19)

/**
 * FREEWHEEL - Output Freewheeling Mode - Bits 21:20
 *
 * Controls the motor output behavior when current is zero.
 * Available only when StealthChop2 is active and motor is not in hold mode.
 *
 * Settings:
 *   ┌───────┬──────────────────┬─────────────────────────────────┐
 *   │ Value │ Mode             │ Description                     │
 *   ├───────┼──────────────────┼─────────────────────────────────┤
 *   │   0   │ Normal operation │ Normal chopper regulation       │
 *   │       │                  │ (default, standard mode)        │
 *   ├───────┼──────────────────┼─────────────────────────────────┤
 *   │   1   │ Freewheel (LS)   │ Short through low-side FETs     │
 *   │       │                  │ Motor coasts freely             │
 *   ├───────┼──────────────────┼─────────────────────────────────┤
 *   │   2   │ Freewheel (HS)   │ Short through high-side FETs    │
 *   │       │                  │ Motor coasts freely             │
 *   ├───────┼──────────────────┼─────────────────────────────────┤
 *   │   3   │ Coil shorted     │ Passive braking (short coil)    │
 *   │       │                  │ Creates damping effect          │
 *   └───────┴──────────────────┴─────────────────────────────────┘
 *
 * Availability:
 *   - Only effective when StealthChop2 is active (EN_PWM_MODE=1)
 *   - Only when IHOLD=0 (not in hold current mode)
 *   - Otherwise: Normal chopper operation is used
 *
 * Use cases:
 *   - Mode 1/2: Reduce power consumption during idle phases
 *   - Mode 3: Add passive damping to reduce oscillation
 *
 * Range: 0-3
 * Default: 0 (normal operation)
 */
#define TMC522X_PWMCONF_FREEWHEEL_MASK       GENMASK(21, 20)

/**
 * PWM_MEAS_SD_ENABLE - Current Measurement During Slow Decay - Bit 22
 *
 * Controls when motor current is measured in slow-decay mode.
 * Affects current regulation accuracy.
 *
 * Settings:
 *   0 = Measure current during on-phases only
 *       - Measures when high-side FETs are conducting
 *       - May result in lower measured current
 *       - Simple measurement strategy
 *
 *   1 = Also measure during slow-decay phases (recommended)
 *       - Includes measurements during slow-decay
 *       - More accurate current representation
 *       - Better regulation quality
 *       - Recommended for most applications
 *
 * Impact on regulation:
 *   - Enabled (1): More accurate, better regulation
 *   - Disabled (0): Faster but less accurate
 *
 * Range: 0-1
 * Default: 0 (but 1 is recommended for better accuracy)
 */
#define TMC522X_PWMCONF_PWM_MEAS_SD_ENABLE_MASK BIT(22)

/**
 * PWM_DIS_REG_STST - Disable Current Regulation During Standstill - Bit 23
 *
 * Controls whether current regulation remains active when motor is stopped.
 * Useful for very low standstill current and silent operation.
 *
 * Settings:
 *   0 = Regulation remains active at standstill (normal behavior)
 *       - Current is actively regulated to IHOLD
 *       - Stable, predictable standstill current
 *       - Small PWM activity may be audible
 *
 *   1 = Current regulation disabled at standstill
 *       - No active current regulation when stopped
 *       - Current may drift slowly (voltage/temperature dependent)
 *       - Completely silent standstill
 *       - Lower power consumption
 *
 * Use cases for disabling (1):
 *   - Applications needing completely silent standstill
 *   - Very low current required at standstill (< IHOLD)
 *   - Battery-powered applications (minimize quiescent current)
 *
 * Caution:
 *   - With regulation disabled, current is not precisely controlled
 *   - May increase or decrease based on conditions
 *   - Not suitable for applications requiring precise holding torque
 *
 * Range: 0-1
 * Default: 0 (regulation active at standstill)
 */
#define TMC522X_PWMCONF_PWM_DIS_REG_STST_MASK   BIT(23)

/**
 * SG4_FILT_EN (StallGuard4 Filter Enable) - Bit 24
 *
 * Controls StallGuard4 measurement filtering mode.
 *
 * Settings:
 *   0 = Unfiltered mode (default):
 *       - SG4_RESULT updates every full step
 *       - Only SG4_IND_0 contains valid measurement
 *       - Fastest reaction to load changes
 *       - Best for hard obstacle / sudden blockage detection
 *
 *   1 = Filtered mode:
 *       - Four individual phase measurements taken (SG4_IND_0 to SG4_IND_3)
 *       - SG4_RESULT = average of all four measurements
 *       - Each full step contributes only 25% to result
 *       - Smoother, more stable load measurement
 *       - Best for soft obstacle detection or imprecise motors
 *       - Reduces sensitivity to sudden load increase
 *
 * Note: Filtering provides better noise immunity but slower response
 */
#define TMC522X_PWMCONF_SG4_FILT_EN_MASK     BIT(24)
/** @} */

/**
 * @name TMC522X DRV_STATUS register fields
 * @{
 */
/**
 * SG_RESULT (StallGuard2 Result) - Bits 9:0
 *
 * StallGuard2 load measurement result (read-only) for SpreadCycle mode.
 * This 10-bit value provides real-time mechanical load feedback.
 *
 * IMPORTANT: This is StallGuard2 (for SpreadCycle mode).
 *            For StealthChop2 mode, use SG4_RESULT register (0x40) instead.
 *
 * Load Curve Behavior:
 *   The SG_RESULT value changes linearly with motor load:
 *
 *   ┌─────────────────────────────────────────┐
 *   │ Motor Load    │ SG_RESULT │ Load Angle  │
 *   ├───────────────┼───────────┼─────────────┤
 *   │ 0% (no load)  │ High      │ Small       │
 *   │               │ 500-1023  │ 0°-30°      │
 *   │               │     ↓     │     ↓       │
 *   │ Increasing... │ Decreases │ Increases   │
 *   │               │  linearly │  linearly   │
 *   │               │     ↓     │     ↓       │
 *   │ Max safe load │   0-100   │ ~60°-80°    │
 *   │ (tuned SGT)   │           │             │
 *   │               │     ↓     │     ↓       │
 *   │ Stall point   │     0     │ 90°         │
 *   │ Motor loses   │           │ Torque      │
 *   │ steps         │           │ collapses   │
 *   └───────────────┴───────────┴─────────────┘
 *
 * Value interpretation:
 *   High values (e.g., 500-1023) = Low mechanical load, motor running freely
 *   Medium values (e.g., 100-500) = Normal operating load
 *   Low values (e.g., 1-100)      = High load, approaching stall
 *   0                              = Maximum load (stalled or about to stall)
 *
 * Tuning workflow (with SGT adjustment):
 *   1. Run motor with expected maximum load (just before stall point)
 *   2. Read SG_RESULT value
 *   3. Adjust SGT threshold so SG_RESULT reads 0-100 at max load
 *   4. This maintains useful dynamic range above zero for load monitoring
 *   5. Verify SG_RESULT shows 100+ increase when load is removed
 *
 * Load relationship:
 *   Higher SG_RESULT → Lower load → Smaller load angle → Motor has torque margin
 *   Lower SG_RESULT  → Higher load → Larger load angle → Approaching stall
 *   SG_RESULT = 0    → Load angle ≈ 90° → Motor at stall threshold
 *
 * Update rate: One measurement per full step (4 microsteps at 1/4 stepping)
 *              With SFILT=1: Averaged over 4 full steps (one electrical period)
 *
 * Factors affecting SG_RESULT:
 *   - Motor construction and characteristics
 *   - Mechanical load on motor shaft
 *   - Motor velocity (set valid range with TCOOLTHRS and THIGH)
 *   - Supply voltage (tighter regulation = more accurate)
 *   - Motor current setting
 *   - Temperature (coil resistance increases → adjust SGT if needed)
 *   - Clock frequency (use crystal for highest precision)
 *
 * Operational limits:
 *   - Too slow (<1 RPS): Low back-EMF, unstable measurement, temperature dependent
 *   - Too fast: Back-EMF approaches supply voltage, poor response
 *   - Use TCOOLTHRS to define lower velocity threshold
 *   - Use THIGH to define upper velocity threshold
 *
 * Measurement accuracy: ± max(1, |SGT|)
 *
 * Range: 0-1023 (10-bit value)
 *
 * Active when: TCOOLTHRS < TSTEP < THIGH (SpreadCycle/CoolStep region)
 *
 * Applications:
 *   - Stall detection (SG_RESULT reaches 0)
 *   - Sensorless homing (detect hard stop)
 *   - CoolStep current optimization (load-adaptive current control)
 *   - Real-time load monitoring
 *
 * See also: SG4_RESULT (0x40) for StallGuard4 with StealthChop2
 */
#define TMC522X_DRV_STATUS_SG_RESULT_MASK     GENMASK(9, 0)
#define TMC522X_DRV_STATUS_S2VSA              BIT(12)
#define TMC522X_DRV_STATUS_S2VSB              BIT(13)
#define TMC522X_DRV_STATUS_STEALTH            BIT(14)
#define TMC522X_DRV_STATUS_FSACTIVE           BIT(15)

/**
 * CS_ACTUAL (Actual Motor Current Scale) - Bits 20:16 [Read-Only]
 *
 * Reports the actual current scaling value being applied to motor coils.
 * This is the result of CoolStep's dynamic current adjustment or fixed IRUN setting.
 *
 * Value range: 0-31 (5-bit)
 *
 * Current calculation:
 *   Actual motor current = (CS_ACTUAL / 32) × IRUN_setting
 *
 * Operating modes:
 *   ┌─────────────────┬────────────────────────────────────┐
 *   │ CoolStep Status │ CS_ACTUAL Behavior                 │
 *   ├─────────────────┼────────────────────────────────────┤
 *   │ Disabled        │ Fixed at IRUN value (0-31)         │
 *   │ (SEMIN = 0)     │ No dynamic adjustment              │
 *   ├─────────────────┼────────────────────────────────────┤
 *   │ Enabled         │ Dynamic adjustment based on load:  │
 *   │ (SEMIN > 0)     │ - Increases when load HIGH         │
 *   │                 │ - Decreases when load LOW          │
 *   │                 │ - Clamped by SEIMIN limit          │
 *   └─────────────────┴────────────────────────────────────┘
 *
 * CoolStep limits:
 *   SEIMIN = 0: CS_ACTUAL minimum = 16 (½ of IRUN, 50% current)
 *   SEIMIN = 1: CS_ACTUAL minimum =  8 (¼ of IRUN, 25% current)
 *   Maximum value: Always 31 (corresponds to IRUN setting)
 *
 * Interpreting CS_ACTUAL during CoolStep:
 *   CS_ACTUAL = 31:     Motor at maximum configured current (IRUN)
 *                       → Load is HIGH, may need more IRUN headroom
 *
 *   CS_ACTUAL = 16-30:  Motor using partial current
 *                       → Normal operation, load being managed
 *
 *   CS_ACTUAL = 8-15:   Motor at reduced current (only if SEIMIN=1)
 *                       → Low load, good power savings
 *
 *   CS_ACTUAL at minimum (16 or 8): Motor at lowest allowed current
 *                       → Very light load, maximum power savings
 *
 * Monitoring guidelines:
 *   - Read CS_ACTUAL to verify CoolStep is working
 *   - If always at 31: Increase IRUN or adjust SEMIN (thresholds too tight)
 *   - If always at minimum: Decrease IRUN or adjust SEMAX (thresholds too loose)
 *   - Should vary with load changes if CoolStep is tuned correctly
 *
 * Velocity dependency:
 *   CS_ACTUAL only changes when:
 *     - CoolStep is enabled (SEMIN > 0)
 *     - Motor velocity is in range: TCOOLTHRS < TSTEP < THIGH
 *     - Outside this range: CS_ACTUAL = IRUN (fixed)
 *
 * Power consumption estimation:
 *   Power savings = 1 - (CS_ACTUAL / 31)²
 *
 *   Examples:
 *     CS_ACTUAL = 31 → 0% savings (full current)
 *     CS_ACTUAL = 16 → 73% savings (½ current)
 *     CS_ACTUAL =  8 → 93% savings (¼ current)
 *
 * Use cases:
 *   - Verify CoolStep operation (should see value changing with load)
 *   - Tune CoolStep thresholds (observe response to load changes)
 *   - Calculate instantaneous power consumption
 *   - Debug current-related issues (unexpected values indicate misconfiguration)
 *   - Optimize IRUN setting (if CS_ACTUAL rarely reaches 31, can reduce IRUN)
 *
 * Read using: TMC522X_GET_CS_ACTUAL(drv_status) macro
 */
#define TMC522X_DRV_STATUS_CS_ACTUAL_MASK     GENMASK(20, 16)
#define TMC522X_DRV_STATUS_RES_DET            BIT(22)
#define TMC522X_DRV_STATUS_STALLGUARD_PREWARN BIT(23)
#define TMC522X_DRV_STATUS_STALLGUARD         BIT(24)
#define TMC522X_DRV_STATUS_OT                 BIT(25)
#define TMC522X_DRV_STATUS_OTPW               BIT(26)
#define TMC522X_DRV_STATUS_S2GA               BIT(27)
#define TMC522X_DRV_STATUS_S2GB               BIT(28)
#define TMC522X_DRV_STATUS_OLA                BIT(29)
#define TMC522X_DRV_STATUS_OLB                BIT(30)
#define TMC522X_DRV_STATUS_STST               BIT(31)

/* Legacy aliases for compatibility */
#define TMC522X_DRV_STATUS_STEALTHCHOP TMC522X_DRV_STATUS_STEALTH

/* Helper macros for field extraction */
/**
 * Extract SG_RESULT (StallGuard2 result) from DRV_STATUS register.
 *
 * @param drv_status_val The raw DRV_STATUS register value
 * @return StallGuard2 load measurement (0-1023)
 *         High value = low load, Low value = high load
 */
#define TMC522X_GET_SG_RESULT(drv_status_val) \
	((drv_status_val) & TMC522X_DRV_STATUS_SG_RESULT_MASK)

/**
 * Extract CS_ACTUAL (actual current scaling) from DRV_STATUS register.
 *
 * @param drv_status_val The raw DRV_STATUS register value
 * @return Actual current scale value (0-31)
 */
#define TMC522X_GET_CS_ACTUAL(drv_status_val) \
	(((drv_status_val) & TMC522X_DRV_STATUS_CS_ACTUAL_MASK) >> 16)
/** @} */

/**
 * @name TMC522X Microstep resolution values
 * @{
 */
#define TMC522X_MRES_256 0
#define TMC522X_MRES_128 1
#define TMC522X_MRES_64  2
#define TMC522X_MRES_32  3
#define TMC522X_MRES_16  4
#define TMC522X_MRES_8   5
#define TMC522X_MRES_4   6
#define TMC522X_MRES_2   7
#define TMC522X_MRES_1   8
/** @} */

/**
 * @brief TMC522X Ramp mode
 */
enum tmc522x_rampmode {
	TMC522X_RAMPMODE_POSITION = 0,
	TMC522X_RAMPMODE_POSITIVE_VELOCITY,
	TMC522X_RAMPMODE_NEGATIVE_VELOCITY,
	TMC522X_RAMPMODE_HOLD,
};

/**
 * @brief TMC522X PWM freewheel mode
 */
enum tmc522x_pwm_freewheel {
	TMC522X_FREEWHEEL_NORMAL = 0,
	TMC522X_FREEWHEEL_FREEWHEEL,
	TMC522X_FREEWHEEL_COILSHORT_LS,
	TMC522X_FREEWHEEL_COILSHORT_HS,
};

/**
 * @brief TMC522X PWM frequency
 */
enum tmc522x_pwm_freq {
	TMC522X_PWM_FREQ_2_1024 = 0,
	TMC522X_PWM_FREQ_2_683,
	TMC522X_PWM_FREQ_2_512,
	TMC522X_PWM_FREQ_2_410,
};
/** @} */

/**
 * @brief Bus union to support both SPI and I2C interfaces
 */
union tmc522x_bus {
#if CONFIG_TMC522X_BUS_SPI
	struct spi_dt_spec spi;
#endif
#if CONFIG_TMC522X_BUS_I2C
	struct i2c_dt_spec i2c;
#endif
};

/**
 * @brief Bus operation function pointer types
 */
typedef int (*tmc522x_bus_check_fn)(const union tmc522x_bus *bus);
typedef int (*tmc522x_bus_init_fn)(const union tmc522x_bus *bus);
typedef int (*tmc522x_reg_read_fn)(const union tmc522x_bus *bus, uint8_t reg_addr, uint32_t *reg_val);
typedef int (*tmc522x_reg_write_fn)(const union tmc522x_bus *bus, uint8_t reg_addr, uint32_t reg_val);

/**
 * @brief Bus I/O operations structure
 */
struct tmc522x_bus_io {
	tmc522x_bus_check_fn check;
	tmc522x_bus_init_fn init;
	tmc522x_reg_read_fn read;
	tmc522x_reg_write_fn write;
};

/**
 * @brief TMC522X device configuration (compile-time constant)
 */
struct tmc522x_config {
	/** Bus configuration (SPI or I2C) */
	union tmc522x_bus bus;
	/** Bus I/O operations */
	const struct tmc522x_bus_io *bus_io;
	/** Driver enable GPIO specification (DRVEN pin - required for stepper_enable()) */
	struct gpio_dt_spec drven_gpio;
	/** Sleep/wake GPIO specification (SLEEPN pin, active high - datasheet power-up step 1) */
	struct gpio_dt_spec sleepn_gpio;
	/** StallGuard interrupt GPIO specification (DIAG0 pin - optional) */
	struct gpio_dt_spec stallguard_gpio;
	/** Position reached interrupt GPIO specification (DIAG1 pin - optional) */
	struct gpio_dt_spec pos_reached_gpio;
	/** Clock frequency configuration */
	const uint32_t clock_frequency;
	/** Flag indicating if drven GPIO is present */
	bool has_drven;
	/** Flag indicating if sleepn GPIO is present */
	bool has_sleepn;
	/** Flag indicating if stallguard GPIO interrupt is present */
	bool has_stallguard_irq;
	/** Flag indicating if position reached GPIO interrupt is present */
	bool has_pos_reached_irq;
	/** Enable StealthChop2 mode (true=StealthChop2, false=SpreadCycle) */
	bool enable_stealthchop;
	/** Default microstep resolution */
	const uint16_t default_micro_step_res;
	/** Default ramp generator parameters from devicetree */
	const uint32_t default_vstart;
	const uint32_t default_a1;
	const uint32_t default_v1;
	const uint32_t default_a2;
	const uint32_t default_v2;
	const uint32_t default_amax;
	const uint32_t default_vmax;
	const uint32_t default_dmax;
	const uint32_t default_d2;
	const uint32_t default_d1;
	const uint32_t default_vstop;
	/** Default current settings from devicetree */
	const uint8_t default_ihold;
	const uint8_t default_irun;
	const uint8_t default_iholddelay;
	const uint8_t default_irundelay;
	const uint32_t default_tpowerdown;
	/** Default driver configuration from devicetree */
	const uint8_t default_fs_gain;
	const uint8_t default_fs_sense;
	const uint8_t default_global_scaler;
	/** Default chopper configuration from devicetree */
	const uint8_t default_toff;
	const uint8_t default_tbl;
	const uint8_t default_hstrt;
	const uint8_t default_hend;
	const bool default_chm;  /* false=SpreadCycle (CHM=0), true=Classic (CHM=1) */
	/** Default PWM configuration from devicetree */
	const uint8_t default_pwm_freewheel;
	const uint8_t default_pwm_freq;
	/** Default StallGuard configuration from devicetree */
	const int8_t default_sgt;
	const uint16_t default_sgt4;
	/** Default CoolStep configuration from devicetree */
	const uint8_t default_semin;
	const uint8_t default_semax;
	const uint8_t default_seup;
	const uint8_t default_sedn;
	const bool default_seimin;
	/** Motor physical parameters from devicetree */
	const uint32_t default_step_angle_millidegrees;
};

/**
 * @brief TMC522X runtime data
 *
 * Note: The actual implementation uses an extended structure with additional
 * private fields for motor control, chopper configuration, PWM settings, etc.
 * This structure serves as the base for device data following Zephyr patterns.
 */
struct tmc522x_data {
	/** Semaphore for register access protection */
	struct k_sem sem;

	/* Device pointer for interrupt context */
	const struct device *dev;

	/* Interrupt support */
	struct gpio_callback stallguard_cb;
	struct gpio_callback pos_reached_cb;
	struct k_work stallguard_work;
	struct k_work pos_reached_work;

	/* Clock configuration */
	uint32_t clock;
	bool enabled;

	/* Event tracking */
	bool stallguard_event_enabled;
	bool pos_reached_event_enabled;

	/* Current control */
	uint8_t current_run;
	uint8_t current_hold;

	/* Microstep and mode */
	uint8_t microstep_res;
	uint32_t step_angle_millidegrees;

	/* Chopper configuration */
	uint8_t toff;
	uint8_t tbl;
	uint8_t hstrt_tfd210;   /* SpreadCycle (CHM=0): HSTRT, Classic (CHM=1): TFD[2:0] */
	uint8_t hend_offset;    /* SpreadCycle (CHM=0): HEND, Classic (CHM=1): OFFSET */
	bool chm;               /* Chopper mode: false=SpreadCycle (CHM=0), true=Classic (CHM=1) */
	bool vhighchm;
	bool vhighfs;

	/* PWM configuration */
	bool pwm_dis_reg_stat;
	bool pwm_meas_sd_enable;
	enum tmc522x_pwm_freewheel pwm_freewheel;
	enum tmc522x_pwm_freq pwm_freq;

	/* StallGuard configuration */
	int8_t sgt;        /* StallGuard2 threshold: signed -64 to +63 */
	uint16_t sgt4;     /* StallGuard4 threshold: unsigned 0 to 511 */
	bool sg_stop;      /* Stop motor on stall detection */

	/* CoolStep configuration */
	uint8_t semin;     /* Minimum StallGuard value for smart current control (0-15) */
	uint8_t semax;     /* Maximum StallGuard value (0-15) */
	uint8_t seup;      /* Current increment step width (0-3: 1,2,4,8) */
	uint8_t sedn;      /* Current decrement speed (0-3: 32,8,2,1) */
	bool seimin;       /* Minimum current: false=1/2 of IRUN, true=1/4 of IRUN */

	/* Delay settings */
	uint8_t irundelay;
	uint8_t iholddelay;

	/* Ramp mode */
	enum tmc522x_rampmode rampmode;

	/* Event callback for stepper events */
	stepper_event_callback_t event_callback;
	void *event_callback_user_data;

	/* Cached configuration attributes */
	uint8_t fs_gain;
	uint8_t fs_sense;
	uint8_t global_scaler_a;
	uint8_t global_scaler_b;
	uint32_t tpowerdown;
	uint32_t tpwmthrs;
	uint32_t tcoolthrs;
	uint32_t thigh;
	uint32_t vstart;
	uint32_t a1;
	uint32_t v1;
	uint32_t a2;
	uint32_t v2;
	uint32_t amax;
	uint32_t vmax;
	uint32_t dmax;
	uint32_t d2;
	uint32_t d1;
	uint32_t vstop;
	int32_t xtarget;
};

/**
 * @brief Write a 32-bit register to the TMC522X via SPI
 *
 * @param dev      Pointer to the TMC522X device
 * @param reg_addr Register address (0x00-0x7F)
 * @param value    32-bit value to write
 * @return         0 on success, negative errno on failure
 */
int tmc522x_reg_write(const struct device *dev, uint8_t reg_addr, uint32_t value);

/**
 * @brief Read a 32-bit register from the TMC522X via SPI
 *
 * @param dev      Pointer to the TMC522X device
 * @param reg_addr Register address (0x00-0x7F)
 * @param value    Pointer to store the read value
 * @return         0 on success, negative errno on failure
 */
int tmc522x_reg_read(const struct device *dev, uint8_t reg_addr, uint32_t *value);

/**
 * @brief Burst-read N consecutive 32-bit registers from TMC522X
 *
 * For SPI, this performs individual reads for each register.
 *
 * @param dev        Pointer to the TMC522X device
 * @param start_addr Starting register address
 * @param buf        Buffer to receive N*4 bytes
 * @param num_regs   Number of 32-bit registers to read
 * @return           0 on success, negative errno on failure
 */
int tmc522x_burst_read(const struct device *dev, uint8_t start_addr, uint8_t *buf, size_t num_regs);

/**
 * @brief Set chopper mode (SpreadCycle or StealthChop2)
 *
 * Configures the EN_PWM_MODE bit in GCONF register to select between
 * SpreadCycle (traditional chopper) or StealthChop2 (quiet) mode.
 *
 * @param dev          Pointer to the TMC522X device
 * @param stealthchop2 true for StealthChop2, false for SpreadCycle
 * @return             0 on success, negative errno on failure
 */
int tmc522x_stepper_set_chopper_mode(const struct device *dev, bool stealthchop2);

/**
 * @brief Switch between SpreadCycle (CHM=0) and Classic (CHM=1) chopper modes
 *
 * Dynamically changes the CHM bit in CHOPCONF register to select chopper algorithm.
 *
 * This is DIFFERENT from tmc522x_stepper_set_chopper_mode():
 *   - tmc522x_stepper_set_chopper_mode(): SpreadCycle vs StealthChop2 (EN_PWM_MODE)
 *   - tmc522x_set_chm_mode(): SpreadCycle vs Classic (CHM bit)
 *
 * CHM=0 (SpreadCycle): Adaptive hysteresis chopper, variable frequency, recommended
 * CHM=1 (Classic):     Constant off-time chopper, predictable, legacy compatibility
 *
 * Safe to call during motor operation. See function implementation for detailed docs.
 *
 * @param dev          Pointer to the TMC522X device
 * @param classic_mode true for Classic mode (CHM=1), false for SpreadCycle (CHM=0)
 * @return             0 on success, negative errno on failure
 */
int tmc522x_set_chm_mode(const struct device *dev, bool classic_mode);

/**
 * @brief Enable the driver stage via register (clears drv_enn bit)
 *
 * Enables the power MOSFETs by clearing the drv_enn bit (bit 9) in GCONF register.
 * This allows the motor to be energized and produce torque.
 *
 * Note: Actual driver enable = DRV_EN && drv_enn && (TOFF != 0)
 * This function sets drv_enn=0 (enabled) while preserving TOFF value.
 *
 * @param dev Pointer to the TMC522X device
 * @return    0 on success, negative errno on failure
 */
int tmc522x_driver_enable(const struct device *dev);

/**
 * @brief Disable the driver stage via register (sets drv_enn bit)
 *
 * Disables the power MOSFETs by setting the drv_enn bit (bit 9) in GCONF register.
 * This puts the motor into freewheeling mode while preserving all register content
 * including TOFF configuration.
 *
 * Note: Actual driver enable = DRV_EN && drv_enn && (TOFF != 0)
 * This function sets drv_enn=1 (disabled) to switch off the driver stage.
 *
 * @param dev Pointer to the TMC522X device
 * @return    0 on success, negative errno on failure
 */
int tmc522x_driver_disable(const struct device *dev);

/**
 * @brief Enable driver via DRVEN pin (hardware emergency stop release)
 *
 * Drives the DRVEN pin high to enable the driver output stage. This is
 * an asynchronous hardware control that bypasses digital blocks.
 *
 * @param dev Pointer to the TMC522X device
 * @return    0 on success, negative errno on failure (or 0 if no GPIO configured)
 */
int tmc522x_drven_enable(const struct device *dev);

/**
 * @brief Disable driver via DRVEN pin (hardware emergency stop)
 *
 * Drives the DRVEN pin low to immediately tri-state the driver output stage.
 * This provides a hardware-based safety channel independent of MCU software.
 * Torque is immediately removed from motor. Register content is preserved.
 *
 * @param dev Pointer to the TMC522X device
 * @return    0 on success, negative errno on failure (or 0 if no GPIO configured)
 */
int tmc522x_drven_disable(const struct device *dev);

/**
 * @brief Set motor to run at constant velocity (velocity mode)
 *
 * Switches the motor to velocity mode and runs at the specified velocity.
 * Positive values rotate forward, negative values rotate backward.
 * Motor will continue running until stopped or switched to position mode.
 *
 * @param dev      Pointer to the TMC522X device
 * @param velocity Target velocity in microsteps/second (signed)
 *                 Positive: forward rotation (RAMPMODE=1)
 *                 Negative: backward rotation (RAMPMODE=2)
 *                 Zero: stop motor
 * @return         0 on success, negative errno on failure
 */
int tmc522x_set_velocity(const struct device *dev, int32_t velocity);

/**
 * @brief Get actual motor velocity
 *
 * Reads the VACTUAL register to get current motor velocity.
 *
 * @param dev      Pointer to the TMC522X device
 * @param velocity Pointer to store the current velocity (microsteps/second, signed)
 * @return         0 on success, negative errno on failure
 */
int tmc522x_get_actual_velocity(const struct device *dev, int32_t *velocity);

/**
 * @brief Get actual motor acceleration
 *
 * Reads the AACTUAL register to get current motor acceleration.
 *
 * @param dev    Pointer to the TMC522X device
 * @param accel  Pointer to store the current acceleration (microsteps/s², signed)
 * @return       0 on success, negative errno on failure
 */
int tmc522x_get_actual_acceleration(const struct device *dev, int32_t *accel);

/**
 * @brief Set StealthChop2 velocity threshold
 *
 * Configures the velocity above which StealthChop2 mode is disabled.
 * StealthChop2 provides silent operation but is limited to lower speeds.
 *
 * Formula: TPWMTHRS = (clock * step_angle_millideg) / (velocity * 1000 * microsteps)
 *
 * @param dev           Pointer to the TMC522X device
 * @param velocity_pps  Threshold velocity in pulses per second (microsteps/s)
 *                      0 = StealthChop2 disabled
 *                      >0 = StealthChop2 enabled below this velocity
 * @return              0 on success, negative errno on failure
 */
int tmc522x_set_stealthchop_threshold(const struct device *dev, uint32_t velocity_pps);

/**
 * @brief Set CoolStep/StallGuard velocity threshold
 *
 * Configures the velocity above which CoolStep and StallGuard are active.
 * StallGuard detection only works when TSTEP <= TCOOLTHRS (velocity high enough).
 *
 * Formula: TCOOLTHRS = (clock * step_angle_millideg) / (velocity * 1000 * microsteps)
 *
 * @param dev           Pointer to the TMC522X device
 * @param velocity_pps  Threshold velocity in pulses per second (microsteps/s)
 *                      0 = CoolStep/StallGuard disabled
 *                      >0 = Active above this velocity
 * @return              0 on success, negative errno on failure
 */
int tmc522x_set_coolstep_threshold(const struct device *dev, uint32_t velocity_pps);

/**
 * @brief Set fullstep velocity threshold
 *
 * Configures the velocity above which the motor switches to fullstep mode
 * for improved high-speed efficiency.
 *
 * Formula: THIGH = (clock * step_angle_millideg) / (velocity * 1000 * microsteps)
 *
 * @param dev           Pointer to the TMC522X device
 * @param velocity_pps  Threshold velocity in pulses per second (microsteps/s)
 *                      0 = Fullstep switching disabled
 *                      >0 = Switch to fullstep above this velocity
 * @return              0 on success, negative errno on failure
 */
int tmc522x_set_fullstep_threshold(const struct device *dev, uint32_t velocity_pps);

/**
 * @brief Read a register from the TMC522X device
 *
 * This function provides direct register access for testing and debugging.
 * It uses the appropriate bus (SPI or I2C) based on devicetree configuration.
 *
 * @param dev       Pointer to the TMC522X device
 * @param reg_addr  Register address to read from (0x00 - 0x7D)
 * @param reg_val   Pointer to store the 32-bit register value
 * @return          0 on success, negative errno on failure
 */
int tmc522x_read(const struct device *dev, uint8_t reg_addr, uint32_t *reg_val);

/**
 * @brief Write a register to the TMC522X device
 *
 * This function provides direct register access for testing and debugging.
 * It uses the appropriate bus (SPI or I2C) based on devicetree configuration.
 *
 * @param dev       Pointer to the TMC522X device
 * @param reg_addr  Register address to write to (0x00 - 0x7D)
 * @param reg_val   32-bit value to write to the register
 * @return          0 on success, negative errno on failure
 */
int tmc522x_write(const struct device *dev, uint8_t reg_addr, uint32_t reg_val);

/**
 * @brief Enable or disable StallGuard event notifications
 *
 * @param dev       Pointer to the TMC522X device
 * @param enable    true to enable, false to disable
 * @return          0 on success, negative errno on failure
 */
int tmc522x_enable_stallguard_event(const struct device *dev, bool enable);

/**
 * @brief Enable or disable StallGuard detection
 *
 * NOTE: TMC5221 does not have a dedicated "StallGuard enable" bit.
 * Instead, StallGuard is controlled by the TCOOLTHRS velocity threshold.
 * This function sets TCOOLTHRS to effectively enable/disable StallGuard:
 *   - enable=true:  TCOOLTHRS = 0 (active at all velocities)
 *   - enable=false: TCOOLTHRS = 0xFFFFF (never active)
 *
 * For velocity-dependent StallGuard, use tmc522x_set_coolstep_threshold().
 *
 * @param dev Pointer to the device structure
 * @param enable true to enable StallGuard detection, false to disable
 * @return 0 on success, negative errno on failure
 */
int tmc522x_enable_stallguard(const struct device *dev, bool enable);

/**
 * @brief Check and display StallGuard configuration status
 *
 * This function reads and displays all relevant registers to determine
 * if StallGuard is properly enabled and configured. Useful for debugging
 * stall detection issues.
 *
 * @param dev Pointer to the device structure
 * @return 0 on success, negative errno on failure
 */
int tmc522x_check_stallguard_status(const struct device *dev);

/**
 * @brief Clear StallGuard stall event flag
 *
 * Clears the EVENT_STOP_SG flag in the RAMP_STAT register.
 * This is a Write-1-to-Clear (W1C) operation that resets the stall event.
 * Should be called after handling a stall event to allow new movements.
 *
 * @param dev Pointer to the device structure
 * @return 0 on success, negative errno code on error
 */
int tmc522x_clear_stall(const struct device *dev);

/* Bus I/O operations - defined in tmc522x_spi.c and tmc522x_i2c.c */
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
extern const struct tmc522x_bus_io tmc522x_bus_io_spi;
#endif

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
extern const struct tmc522x_bus_io tmc522x_bus_io_i2c;
#endif

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_STEPPER_TMC522X_H_ */
