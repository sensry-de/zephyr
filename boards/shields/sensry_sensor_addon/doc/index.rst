.. SPDX-FileCopyrightText: Copyright (c) 2026 Sensry GmbH
.. SPDX-License-Identifier: Apache-2.0

.. _sensry_sensor_addon:

Sensry Kallisto Sensor Add-On
#############################

Overview
********

The Sensry Kallisto Sensor Add-On is designed for nRF54 Series DKs, using
the Nordic Expansion Board header and native Zephyr drivers for plug-and-play
sensor integration.

It combines a BMI270 IMU, LIS2MDL magnetometer, BME680 environmental sensor,
light sensor, PDM microphone, real-time clock and GPIO expander.

.. list-table::
   :widths: 50 50
   :align: center

   * - .. figure:: sensry_sensor_addon_top.webp
          :alt: Sensry Kallisto Sensor Add-On top view
          :width: 280px
          :align: center

          Top view

     - .. figure:: sensry_sensor_addon_bottom.webp
          :alt: Sensry Kallisto Sensor Add-On bottom view
          :width: 280px
          :align: center

          Bottom view

Supported devices
*****************

.. list-table:: Devices, Zephyr drivers and devicetree node labels
   :header-rows: 1
   :widths: 40 35 25

   * - Device
     - Zephyr driver
     - Node label
   * - BMI270 accelerometer and gyroscope
     - :dtcompatible:`bosch,bmi270`
     - ``imu``
   * - LIS2MDL magnetometer
     - :dtcompatible:`st,lis2mdl`
     - ``magnetometer``
   * - BME680 temperature, pressure, humidity and gas resistance
     - :dtcompatible:`bosch,bme680`
     - ``environmental_sensor``
   * - OPT3002 light sensor
     - :dtcompatible:`ti,opt3001`
     - ``light_sensor``
   * - PDM microphone
     - :dtcompatible:`nordic,nrf-pdm`
     - ``microphone``
   * - PCF85063A real-time clock
     - :dtcompatible:`nxp,pcf85063a`
     - ``external_rtc``
   * - PCA9554-compatible GPIO expander
     - :dtcompatible:`nxp,pca9554`
     - ``i2s_gpio_controller``

Use the node label with ``DT_NODELABEL()`` to access a device:

.. code-block:: c

   const struct device *const bme680 =
       DEVICE_DT_GET(DT_NODELABEL(environmental_sensor));


Setup
*****

For the :zephyr:board:`nrf54l15dk`, use target
``nrf54l15dk/nrf54l15/cpuapp``.

The :zephyr:board:`nrf54lm20dk` can also be used with a board-specific
overlay that maps the sensor interfaces and chip-select signals to its host pins.

Each supported host has a board-specific overlay in the shield's ``boards/``
directory. It maps ``sensry_i2c``, ``sensry_spi`` and ``sensry_pdm`` to host
controllers, configures their pins, then includes ``sensry_sensor_addon.dtsi``
with the shared shield devices. No changes to the host board sources are needed.

The pin assignments below apply to the nRF54L15 DK:

======================  ======
Signal                  DK pin
======================  ======
I2C SDA                 P1.12
I2C SCL                 P1.11
SPI SCK                 P2.06
SPI MOSI                P2.08
SPI MISO                P2.09
BMI270 CS               P1.10
LIS2MDL CS              P1.14
TPM CS (held high)      P1.06
PDM clock               P1.04
PDM data                P1.05
======================  ======

The console uses the DK's VCOM0 interface.

Build and run
*************

Board support and examples
==========================

Board support and ready-to-use application examples are available on the
`Sensry Sensor Add-On website <https://sensor-addon.sensry.io/>`_.

Sensor shell example
====================

From the Zephyr repository, build and flash the sensor shell sample:

.. code-block:: shell

   west build -b nrf54l15dk/nrf54l15/cpuapp --shield sensry_sensor_addon \
     -d build-sensor-shell samples/sensor/sensor_shell -- \
     -DCONFIG_I2C_RTIO=y

   west flash -d build-sensor-shell

``CONFIG_I2C_RTIO`` enables the I2C transfers used by the BME680 asynchronous
sensor API. No sample-specific devicetree overlay is needed.

Open the VCOM0 serial port at 115200 baud. List the sensors and available commands:

.. code-block:: console

   uart:~$ sensor info
   uart:~$ sensor --help

Use the device names printed by ``sensor info``. These are runtime device names,
which may differ from the devicetree node labels listed above. Press Tab to
complete device and channel names.

Read the BME680 environmental sensor, light sensor and magnetometer:

.. code-block:: console

   uart:~$ sensor get environmental_sensor ambient_temp press humidity gas_resistance
   uart:~$ sensor get light_sensor light
   uart:~$ sensor get magnetometer magn_xyz

The microphone, RTC and GPIO expander use separate APIs and are not accessed
through the ``sensor`` commands.

References
**********

* `Sensry <https://sensor-addon.sensry.io/>`_
