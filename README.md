# Teleoperation Sensing and Communication

Embedded sensing and wireless communication components developed for a team teleoperation project using nRF52833 microcontrollers, IMUs, and IEEE 802.15.4.

## Overview

This repository contains my contributions to a larger teleoperation system developed as a final project for a microcontroller systems course at Northwestern University.

My work focused on the sensing and communication components of the system. Multiple orientation sensors were integrated with an nRF52833 microcontroller to collect wrist, forearm, and upper-arm orientation measurements. The sensor data was then packaged and transmitted wirelessly using IEEE 802.15.4.

## Implementation

The sensing system used multiple IMUs connected through I2C and UART interfaces. Embedded software collected orientation measurements from the sensors and prepared the measurements for wireless transmission.

The communication system consisted of two nRF52833 applications:

- `radio_154_send`: collects sensor measurements and transmits orientation data over IEEE 802.15.4.
- `radio_154_recv`: receives the transmitted sensor data for use by the larger teleoperation system.

## Repository Structure

```text
Teleoperation-Sensing-Communication/
├── radio_154_send/
│   ├── main.c
│   ├── bno055.c
│   ├── bno055.h
│   ├── bno085.c
│   ├── bno085.h
│   ├── fsr.c
│   ├── fsr.h
│   └── Makefile
│
└── radio_154_recv/
    ├── main.c
    └── Makefile
