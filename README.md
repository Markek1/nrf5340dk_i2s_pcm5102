# Zephyr I²S: nRF5340 → PCM5102A

nRF5340 DK drives a PCM5102A DAC via Zephyr’s I²S.

**Design**
- High-priority **feeder thread** always feeds I²S using `i2s_buf_write()`. If no app audio is queued, it writes **silence** (no underruns).
- **`main()`** simulates a busy game loop: hogs CPU and occasionally enqueues audio blocks.
- Default output: **1 s tone / 1 s silence** with short fades to avoid clicks.

---

## Hardware

- nRF5340 DK (`nrf5340dk_nrf5340_cpuapp`)
- PCM5102A (GY-PCM5102)
- Jumper wires + headphones/speaker (3.5 mm)

## Wiring

| nRF5340 DK | PCM5102A | Description |
|---|---|---|
| **P1.08** | BCK/BCKL | Bit clock |
| **P1.06** | LCK/LRCK | LR clock |
| **P1.07** | DIN | Serial data |
| **3V3**   | VIN/VCC | 3.3 V |
| **GND**   | GND     | Ground |

---

## Runtime Behavior

- Format: **11025 Hz**, **16-bit**, **stereo interleaved**.
- Block size: **256 frames** (~23.2 ms per block).
- Feeder (prio **0**) preempts main (prio **4**) and falls back to zeros if the queue is empty.
- Main produces one tone block per period during “on” seconds; produces nothing during “off”.

---

## Tuning

- **Lower latency:** `SAMPLES_PER_BLOCK=128`, `QUEUE_DEPTH=3–4`.
- **More spike tolerance:** larger blocks and/or deeper queue.
- **Hard edges:** remove the fade-in/out calls.
- Planning real assets later? Consider **48 kHz** and adjust the clock + `SAMPLE_RATE`.

---

## Quick Troubleshooting

- **Only first blip then silence:** ensure feeder priority is higher (numerically lower) than main.
- **No sound:** recheck wiring and confirm the I²S log reports ~**11025 Hz**.
- **Logs stall (deferred mode):** raise the log thread priority.
