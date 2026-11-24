.. zephyr:code-sample:: tmc5262
   :name: TMC5262 / TMC2262 / TMC3241 stepper
   :relevant-api: stepper_interface stepper_ctrl stepper_tmc5262

   Move a stepper motor with the ADI TMC5262, TMC2262 or TMC3241 and report the driver status.

Description
***********

This sample initializes one of the ADI TMC5262 (two-phase controller and driver), TMC2262
(two-phase driver, STEP/DIR only) or TMC3241 (three-phase controller and driver) stepper motor
ICs, which share the unified ``adi,tmc5262`` driver. It then:

#. registers event callbacks and sets the microstep interval,
#. enables the power stage,
#. moves :kconfig:option:`CONFIG_SAMPLE_MOVE_MICROSTEPS` microsteps. On the TMC5262 and TMC3241
   this is a positioning move on the internal ramp generator. On the TMC2262 the same number of
   microsteps is generated as STEP/DIR pulses by the ``zephyr,gpio-step-dir-stepper-ctrl``
   motion controller, which takes the step pulse timing from the TMC2262 stepper driver node,
#. prints the decoded ``DRV_STATUS``, the supply voltage and the chip temperature, and
#. stops (deceleration, wait for standstill) and disables the power stage.

The device is selected by the devicetree overlay. All overlays define three aliases:
``stepper-chip`` (the SPI device), ``stepper-driver`` and ``stepper-ctrl``.

The chip also has an optional sensor child node (compatible adi,tmc5262-sensor) that exposes the supply
voltage and the chip temperature through the sensor API; see :ref:`stepper-adi-tmc5262`.

Requirements
************

* A TMC5262, TMC2262 or TMC3241 evaluation board connected to the SPI bus (SPI mode 3, up to
  8 MHz), plus SLEEPN, DRV_ENN and (TMC2262) STEP/DIR GPIOs. The overlays use the SPI1 and GPIOB
  pins of the Nucleo-G071RB; adapt ``boards/nucleo_g071rb.overlay`` and the device overlay to
  your wiring.
* A motor supply (VS 4.5 V to 65 V) and a stepper motor. Select ``current-range-ma``, ``irun``
  and ``ihold`` in the overlay to match the motor before applying power.
* The TMC5262-EVKIT user guide and the device datasheets describe the connector pinout.

Building and Running
********************

Select the overlay of the device with ``EXTRA_DTC_OVERLAY_FILE``.

TMC5262:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/stepper/tmc5262
   :board: nucleo_g071rb
   :goals: build flash
   :gen-args: -DEXTRA_DTC_OVERLAY_FILE=tmc5262.overlay

TMC2262:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/stepper/tmc5262
   :board: nucleo_g071rb
   :goals: build flash
   :gen-args: -DEXTRA_DTC_OVERLAY_FILE=tmc2262.overlay

TMC3241:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/stepper/tmc5262
   :board: nucleo_g071rb
   :goals: build flash
   :gen-args: -DEXTRA_DTC_OVERLAY_FILE=tmc3241.overlay

Sample Output
=============

.. code-block:: console

   <inf> stepper_tmc5262: TMC5262 sample, CONFIG_SAMPLE_MOVE_MICROSTEPS=3200
   <inf> stepper_tmc5262: Actual position: 3200
   <inf> stepper_tmc5262: DRV_STATUS: standstill=1 otpw=0 ot=0 stall=0 cs_actual=31 sg_result=0
   <inf> stepper_tmc5262: Supply 24000 mV, chip temperature 31.500 degC
   <inf> stepper_tmc5262: Stopped and disabled

References
**********

* TMC5262 datasheet 19-102041, TMC2262 datasheet 19-102038, TMC3241 datasheet 19-100246.
