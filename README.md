# ESP32 DHT Sensor Module

An ESP32 module for reading DHT11 and DHT22 sensors

Other examples found online used polling and bit banging to interface with the sensor. Both of these methods antithetical to interrupt driven and mult-tasking operating systems like FreeRTOS.

This implementation uses the GPIO edge interrupts and an xQueue to free the task switching system to run normally during sensor reads.

## Installation

Copy `dht_sensor.c` into your project. Add a header for `DHT_Init() and DHT_Sample()`

## API

## void DHT_init(int gpio)

Call once to install the gpio isr service and initialize the gpio to the hold state.

## bool DHT_sample(int gpio, double *p_humdity, double *p_temperature_c)

Call to read the sensor.

Returns true if successfull, false if either the read failed (checksum) or timed out.
