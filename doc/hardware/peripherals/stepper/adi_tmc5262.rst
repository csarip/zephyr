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
  (``cur-i`` ... ``rcoil-user-b``, see :dtcompatible:`adi,tmc5262`).
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
(``GSTAT.reset``) is detected before the next motion command: the cached configuration is re-applied
and the event is reported.

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

Power management
****************

``tmc5262_sleep()`` and ``tmc5262_wake()`` use the SLEEPN pin; the registers are lost in sleep mode and
re-applied on wake-up. The device power management action ``SUSPEND`` uses them; without a SLEEPN pin it
stops the motion and turns the power stage off.

Extended API
************

The chip-level configuration and diagnostics that the generic stepper API does not cover (current
range, thresholds, StallGuard, CoolStep, uDcStep, encoder and TriCoder events, decoded status, ADC
readback, direct register access) are in ``zephyr/drivers/stepper/stepper_tmc5262.h``. Functions of a
feature that no enabled variant has are not compiled in.

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
