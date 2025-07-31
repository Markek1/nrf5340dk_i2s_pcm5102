# Zephyr I2S Example: nRF5340 + PCM5102

This project demonstrates how to use the I2S peripheral on a Nordic nRF5340 DK to send audio data to a PCM5102A stereo DAC using the Zephyr RTOS.

The application generates a continuous 440 Hz sine wave tone to verify that the I2S communication is working correctly.

---
## Hardware Requirements

* Nordic nRF5340 Development Kit (`nrf5340dk_nrf5340_cpuapp`)
* [PCM5102A Stereo DAC Module (GY-PCM5102)](https://allegro.cz/produkt/rozhrani-i2s-pcm5102a-dac-dekoder-gy-pcm5102-dac-pcm-i2s-audio-rpi-41e07445-1044-4a41-8f53-18d6d8dfcc0f?offerId=16459852563)
* Jumper wires
* Headphones or a speaker with a 3.5mm jack to test the audio output.

---
## Wiring 🔌

| nRF5340 DK Pin | PCM5102A Pin | Description          |
| :------------- | :----------- | :------------------- |
| **`P1.08`** | `BCK` / `BCKL` | Bit Clock            |
| **`P1.06`** | `LCK` / `LRCK` | Left/Right Clock     |
| **`P1.07`** | `DIN`          | Data In              |
| **`3.3V`** | `VIN` / `VCC`  | 3.3V Power           |
| **`GND`** | `GND`          | Ground               |
