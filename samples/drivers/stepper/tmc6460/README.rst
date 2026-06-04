.. zephyr:code-sample:: tmc6460
   :name: TMC6460 Sample
   :relevant-api: stepper

   Verify SPI communication with a Trinamic TMC6460 motor controller.

Overview
********

This sample reads and writes registers on the TMC6460 to confirm SPI
communication is functional.

Requirements
************

- A board with an available SPI bus.
- A TMC6460 evaluation board connected to the SPI bus with appropriate
  chip-select wiring.

Building and Running
********************

.. code-block:: console

   west build -b nucleo_h563zi samples/drivers/stepper/tmc6460
   west flash

Replace ``nucleo_h563zi`` with your target board. If your board does not
already have an overlay under ``boards/``, create one that places the
``tmc6460`` node on the correct SPI bus.

Sample Output
*************

.. code-block:: console

   [00:00:00.010,000] <inf> tmc6460_sample: TMC6460 device ready: tmc6460@0
   [00:00:00.011,000] <inf> tmc6460_sample: CHIP.ID = 0x00006460
   [00:00:00.012,000] <inf> tmc6460_sample: IO_CONFIG = 0x00000000
   [00:00:00.013,000] <inf> tmc6460_sample: Wrote IO_CONFIG = 0x00000004
   [00:00:00.014,000] <inf> tmc6460_sample: IO_CONFIG readback = 0x00000004
   [00:00:00.015,000] <inf> tmc6460_sample: TMC6460 sample complete
