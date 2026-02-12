.. zephyr:code-sample:: tmc522x
   :name: TMC522x stepper
   :relevant-api: stepper_interface

   Test TMC522x stepper motor position and velocity modes.

Description
***********

This sample application tests the TMC522x stepper motor driver in position mode and velocity mode
across both SpreadCycle and StealthChop chopper configurations.

References
**********

.. note::
   TMC522x datasheet not yet publicly available.

Wiring
******

This sample supports both SPI (TMC5221) and I2C (TMC5222) variants. The board's devicetree must
define a ``stepper0`` alias for the stepper motor node.

Building and Running
********************

This sample requires a TMC522x stepper driver. It should work with any platform featuring a SPI
or I2C peripheral interface. It does not work on QEMU.

Build for SPI (TMC5221)
=======================

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/stepper/tmc522x
   :board: nucleo_f413zh
   :gen-args: -DDTC_OVERLAY_FILE=boards/nucleo_f413zh_spi.overlay
   :goals: build flash

Build for I2C (TMC5222)
=======================

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/stepper/tmc522x
   :board: nucleo_f413zh
   :gen-args: -DDTC_OVERLAY_FILE=boards/nucleo_f413zh_i2c.overlay
   :goals: build flash

Sample Output
=============

.. code-block:: console

   [00:00:00.000,000] <inf> stepper_tmc522x: TMC522x stepper sample
   [00:00:00.100,000] <inf> stepper_tmc522x: Test 1: SpreadCycle + Position
   [00:00:03.200,000] <inf> stepper_tmc522x: Test 2: SpreadCycle + Velocity
   [00:00:07.300,000] <inf> stepper_tmc522x: Test 3: StealthChop + Position
   [00:00:11.400,000] <inf> stepper_tmc522x: Test 4: StealthChop + Velocity
   [00:00:15.500,000] <inf> stepper_tmc522x: Test 5: Reverse Velocity
   [00:00:18.600,000] <inf> stepper_tmc522x: All tests complete


