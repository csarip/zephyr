.. _stepper-adi-tmc5262:

ADI TMC5262, TMC2262 and TMC3241
################################

One driver supports three Analog Devices stepper motor ICs that share one SPI protocol, one PLL and
reset scheme and one register map. The device variant is chosen only by the devicetree compatible.

.. list-table::
   :header-rows: 1

   * - Compatible
     - Device
     - Step source
     - Features
   * - :dtcompatible:`adi,tmc5262`
     - Two-phase controller and driver, 65 V
     - Ramp generator or STEP/DIR
     - StealthChop+, StallGuard+, CoolStep+, uDcStep, TriCoder, coil measurement, reference
       switches, position compare
   * - :dtcompatible:`adi,tmc2262`
     - Two-phase driver, 65 V
     - STEP/DIR only
     - StealthChop+, StallGuard+, CoolStep+, uDcStep, TriCoder, coil measurement
   * - :dtcompatible:`adi,tmc3241`
     - Three-phase controller and driver, 65 V
     - Ramp generator or STEP/DIR
     - SpreadCycle with StallGuard2, CoolStep, reference switches, position compare

The full scale current ranges are 1.5, 3, 4.5 and 6 A peak (1.06, 2.12, 3.18 and 4.24 A RMS) on all
three devices. The ramp generator and every unit conversion use the fixed 16 MHz system clock that the
PLL provides, whatever the clock source.

Devicetree
**********

The chip node is the SPI device (SPI mode 3, at most 8 MHz) and the parent of up to three children:

* ``adi,tmc5262-stepper-driver``: :c:group:`stepper_hw_driver` API (enable, disable, microstep
  resolution, fault and stall events).
* ``adi,tmc5262-stepper-ctrl``: :c:group:`stepper_ctrl` API on the internal ramp generator
  (TMC5262 and TMC3241 with ``step-source = "ramp-generator"``).
* ``adi,tmc5262-sensor``: sensor API for the supply voltage and the chip temperature.

STEP/DIR operation, always used by the TMC2262, is delegated to the
``zephyr,gpio-step-dir-stepper-ctrl`` motion controller. Its ``stepper-driver`` property points to the
driver child, which supplies the step pulse width and the active edge setting of the chip.

.. code-block:: devicetree

   &spi1 {
           tmc5262: tmc5262@0 {
                   compatible = "adi,tmc5262";
                   reg = <0>;
                   spi-max-frequency = <DT_FREQ_M(4)>;
                   sleepn-gpios = <&gpiob 4 GPIO_ACTIVE_HIGH>;
                   drv-enn-gpios = <&gpiob 5 GPIO_ACTIVE_LOW>;
                   current-range-ma = <4240>;
                   irun = <127>;
                   ihold = <32>;

                   tmc5262_driver: stepper-driver {
                           compatible = "adi,tmc5262-stepper-driver";
                           micro-step-res = <256>;
                   };

                   tmc5262_ctrl: motion-controller {
                           compatible = "adi,tmc5262-stepper-ctrl";
                           vmax = <53687>;
                           amax = <440>;
                           dmax = <440>;
                   };

                   tmc5262_sensor: sensor {
                           compatible = "adi,tmc5262-sensor";
                   };
           };
   };

``drv-enn-gpios`` must be ``GPIO_ACTIVE_LOW`` and ``sleepn-gpios`` ``GPIO_ACTIVE_HIGH``: the build
checks this, and at run time the driver reads ``IOIN.DRV_ENN`` back to confirm that the power stage
really is off after initialization.

Optional property groups, all with the datasheet reset values as defaults and rejected at build time on
variants that do not have the feature:

* TMC5262 and TMC2262: StealthChop+ regulators, StallGuard+, CoolStep+, uDcStep, coil measurement
  (``cur-i`` ... ``rcoil-user-b``, see :dtcompatible:`adi,tmc5262`). uDcStep has ``udc-accel-thrs`` and
  ``udc-decel-thrs`` on the TMC5262 (decel must be greater than accel); the TMC2262 has only
  ``udc-decel-thrs`` (``UDC_CONF[7:4]`` is reserved and never written).
* TMC5262 and TMC3241: reference switches, virtual stops and position compare (``stop-l-enable`` ...
  ``x-compare-repeat``). They need ``step-source = "ramp-generator"``. The position compare event is
  routed to DO0 or DO1 with the DO source ``xcomp``.

Initialization and fault handling
*********************************

The driver resets the chip (SLEEPN pulse), checks it (``IOIN`` and ``GSTAT``), starts the PLL (up to
three attempts), clears the power-up flags and writes the configuration. The power stage stays off until
``stepper_enable()``. A missing reset flag after a hardware reset is only logged, not an error.

``stepper_enable()`` cycles the power stage (DRV_ENN and ``TOFF`` = 0) if a driver fault, short circuit
or overtemperature is latched, and fails with ``-EIO`` if the fault persists. A fault while running
disables the power stage and is reported with ``STEPPER_EVENT_FAULT_DETECTED``. An unexpected reset
(``GSTAT.reset``) is detected before the next motion command of the ramp generator: the cached
configuration is re-applied and the event is reported. In STEP/DIR operation the motion commands go to
the step-dir motion controller and never reach the driver, so a reset is detected by ``stepper_enable()``,
by the fault event callback or by ``tmc5262_reinit()`` instead.

Stall detection
***************

``STEPPER_EVENT_STALL_DETECTED`` needs StallGuard to be tuned and active. ``DRV_STATUS.stallguard``
reads 1 while StallGuard is not active (``SG_RESULT`` 0), so the driver only reports a stall if the
chip generates a valid stall output: ``TCOOLTHRS`` must be set (0 is off) and ``TSTEP`` must not exceed
it (``TCOOLTHRS`` >= ``TSTEP``). A run that starts below that velocity, for example during the
acceleration, is not reported until it enters the window. With ``COOLCONF.thigh_sg_off`` set,
StallGuard is also off above ``THIGH``. With StealthChop+ on a variant with StallGuard+, the low
velocity window ``TCOOLTHRS`` < ``TSTEP`` < ``TSGP_LOW_VEL_THRS`` is accepted as well. The stop-on-stall
event of the ramp generator (``RAMP_STAT.event_stop_sg``, enabled with ``tmc5262_set_stallguard_stop()``)
is generated by the chip with the same conditions and is always reported.

Tune the threshold (``tmc5262_set_stallguard2()``, ``stallguard2-threshold``) on the bench: raise
``SGT`` until ``SG_RESULT`` stays above the stall limit while the motor runs free, and drops to
the limit under load. Example (TMC5262-EVAL, 51200 microsteps/s, about 415 mA): ``SGT`` 13 and
``TCOOLTHRS`` 1000 gave ``SG_RESULT`` of about 350 with the motor running free, and the stop-on-stall
stopped the motor under load.

``STEPPER_CTRL_EVENT_STEPS_COMPLETED`` only follows a move commanded with ``stepper_ctrl_move_by()`` or
``stepper_ctrl_move_to()``. The ``position reached`` flag of the chip is also set after the
initialization, a stop or a new reference position, and is not reported as a completed move. A move
that finished before an event callback was registered is not reported either.

Stopping
********

A stop of the ramp generator raises ``TZEROWAIT`` to its maximum and writes ``XTARGET = XACTUAL``
in positioning mode. The target then lies at or behind the motor, so the motor decelerates with DMAX,
D1 and D2 and comes to rest a short distance past the point where the stop was requested. The long
zero-wait keeps it from reversing towards the stale target. At standstill the driver writes the final
``XTARGET = XACTUAL`` (the position where the motor stopped) and restores ``TZEROWAIT``: the motor is
held where it came to rest and does not drive back. ``VMAX`` is only written together with a motion command. ``STEPPER_CTRL_EVENT_STOPPED`` is
reported at the final standstill.

Callbacks run in the system work queue. A callback must not call ``stepper_disable()`` (it waits for the
stop): request ``stepper_ctrl_stop()`` and disable from a thread after the stopped event.

``stepper_ctrl_configure_ramp()`` describes a trapezoid (maximum speed, acceleration, deceleration). It
replaces a multi-phase profile of the devicetree (``v1``, ``v2``, ``a1``, ``a2``, ``d1``, ``d2``) by a
single phase profile (A1 = A2 = AMAX, D1 = D2 = DMAX, V1 = V2 = 0) and logs a warning if it does.
``VSTART``, ``VSTOP`` and ``TZEROWAIT`` are kept. A speed above the datasheet limit of
:math:`2^{23} - 512` ``VMAX`` units (about 7.9995 Mmicrosteps/s at 16 MHz) is rejected with ``-EINVAL``.

STEP/DIR operation
******************

The step-dir motion controller only reads the step pulse width and the active edge from the driver child;
it has no link back to the driver. ``stepper_disable()``, ``tmc5262_sleep()`` and the device power
management ``SUSPEND`` action therefore cannot stop it, and it keeps generating STEP pulses and counting
position. As for the other STEP/DIR drivers, the application stops the motion controller first:

.. code-block:: c

   stepper_ctrl_stop(step_dir_ctrl);
   stepper_disable(tmc2262_driver);

(``stepper_ctrl_stop()`` of the gpio step-dir controller returns immediately, the controller has no
deceleration.)

Power management
****************

``tmc5262_sleep()`` and ``tmc5262_wake()`` use the SLEEPN pin; the registers are lost in sleep mode and
re-applied on wake-up (the first SPI access follows SLEEPN high by the 2 ms wake-up time). The device
power management action ``SUSPEND`` uses them; without a SLEEPN pin it stops the motion and turns the
power stage off. In STEP/DIR operation stop the step-dir motion controller before ``tmc5262_sleep()``
or ``SUSPEND``.

Extended API
************

The chip-level configuration and diagnostics that the generic stepper API does not cover (current
range, thresholds, StallGuard, CoolStep, uDcStep, encoder and TriCoder events, decoded status, ADC
readback, direct register access) are in ``zephyr/drivers/stepper/stepper_tmc5262.h``. Functions of a
feature that no enabled variant has are not compiled in.

``tmc5262_measure_rcoil()`` has no completion flag to wait for: the chip measures the coil resistance
continuously and needs 50 to 100 ms of standstill. The function waits 100 ms and polls ``R_COIL`` up to
four more times at 50 ms intervals, then returns ``-ETIMEDOUT``.

Test suite
**********

``tests/drivers/stepper/tmc5262`` runs a ztest suite against a test SPI emulator of the three variants
on ``native_sim``:

.. code-block:: console

   west twister -T tests/drivers/stepper/tmc5262 -p native_sim

Build-time negative tests (clock range, SPI frequency, properties of other variants) are run with
``tests/drivers/stepper/tmc5262/negative/run.sh``.

API Reference
*************

.. doxygengroup:: stepper_tmc5262
