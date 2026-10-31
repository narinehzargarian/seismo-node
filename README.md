# seismo-node

Earthquake-detecting sensor node for embedded Linux, written in C++20.

It reads a LIS3DH accelerometer over I2C at 100 Hz, detects shaking with an STA/LTA trigger, measures the dominant frequency with an FFT, streams the waveform over UDP, and sends alerts over TCP. It recovers on its own from sensor and network failures.

**Hardware status:** tested against a simulated chip that exposes the LIS3DH's registers, with injected faults. The I2C layer compiles for Linux and ARM in CI but has not been run on a physical sensor yet. The target is a Raspberry Pi with a LIS3DH.

## Architecture

```
 LIS3DH ── I2C ──> acquisition thread ──> lock-free ring buffer ──> processing thread ──> UDP: waveform
 (or simulated)    100 Hz, fault recovery                           filter, detect, FFT
                                                                          │
                                                                    bounded queue
                                                                          │
                                                                    alert thread ─────> TCP: alerts
```

| Part | What it does |
|---|---|
| `lis3dh` | Register-level driver written from the datasheet, on top of a `RegisterBus` interface |
| `linux_i2c` | Real bus using `/dev/i2c-*` and the `I2C_RDWR` ioctl |
| `sim_sensor` | Simulated chip with the same registers: quakes, knocks, bus failures, stuck output |
| `ring_buffer` | Lock-free single-producer, single-consumer queue |
| `dsp` | High-pass filter, STA/LTA trigger, Hann window and radix-2 FFT |
| `protocol` | Big-endian binary packets with sequence numbers and CRC-32, plus a loss tracker |
| `net` | POSIX sockets: UDP for data, TCP for alerts, with reconnect |
| `node` | The three threads, watchdog logic, and clean shutdown |

## Design decisions

**Three threads.** Acquisition has a deadline every 10 ms, so it never touches the network, and alerts can block on TCP without holding up signal processing.

**Lock-free ring buffer between acquisition and processing.** The timed thread must never wait on a lock held by another thread. If the buffer fills, it counts an overrun and keeps going.

**Absolute wake-up times.** Sleeping until the next deadline, rather than for a fixed duration, stops small delays from adding up into drift. Missed deadlines are counted.

**UDP for waveform, TCP for alerts.** Old waveform data isn't worth retrying, so it is sent once, and the receiver measures any loss from sequence numbers. Alerts must arrive, so they use TCP and are retried across reconnects. The queue is bounded, so a long outage can't use up memory.

**Explicit packet format with a CRC.** Fields are written byte by byte in a fixed order instead of copying structs, so packets are portable across compilers and CPUs. The CRC rejects corrupted packets before they reach any processing.

**The register interface is the seam for testing.** The driver doesn't know whether it is talking to a real chip or the simulated one, so the same code runs in both cases.

**Fault detection.** Five read errors in a row is a bus fault. 200 identical samples is a stuck sensor, because a real accelerometer always has noise. Either one triggers re-initialization with exponential backoff, up to 1.6 s.

**Filter reset after gaps.** When samples are missed, the filter state is stale, so it resets. The current packet is also sent early, because a packet's samples must be evenly spaced.

## A bug the tests missed

An end-to-end demo showed a quake being detected 3.2 s late, on the strong S wave instead of the weak P wave, but only when a door slam had happened earlier. Every unit test passed.

The slam's first large sample entered the 30 s background average before the trigger fired. That single sample doubled the baseline and left the detector half as sensitive for the next 30 s.

The fix is to let a sample join the background only after it has left the short window, and never during an event. Regression tests cover it at both the unit and system level.

## Running it

Requires CMake, a C++20 compiler, and GoogleTest. Python 3 is used for the system tests.

```sh
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
python3 tools/system_test.py build
```

Run a simulated scenario at 20x speed, using two terminals:

```sh
./build/seismo-recv --csv waveform.csv
./build/seismo-node --sim --speed 20 --duration 120 --bus-fault-at 10 --slam-at 45 --quake-at 60
```

Receiver output from that run:

```json
{"type":"sensor_fault_bus","t_s":10.04}
{"type":"sensor_recovered","t_s":14.74}
{"type":"event_start","t_s":45.01,"ratio":32.6}
{"type":"transient","t_s":45.01,"duration_s":1.18,"peak_mg":204}
{"type":"event_start","t_s":60.34,"ratio":4.0}
{"type":"event_end","t_s":60.34,"duration_s":18.17,"peak_mg":105,"dominant_hz":3.03}
{"type":"recv_stats","packets":461,"samples":11524,"lost":0,"duplicates":0,"reordered":0,"corrupt":0}
```

The node survives the bus failure, classifies the door slam as a transient because it lasts under 2 s, and catches the quake 0.34 s after it starts, on the P wave. The measured 3.03 Hz matches the simulated 3 Hz S wave.

On a Raspberry Pi with a LIS3DH, the sensor flag would be `--i2c /dev/i2c-1` instead of `--sim`.

## Testing

- **Unit tests (GoogleTest):** ring buffer including two real threads, driver against a fake bus, filter, trigger, FFT, every single-bit corruption of a packet, truncated packets, and sequence wrap-around.
- **In-process node tests:** the full pipeline with the simulated chip and a fake network.
- **System tests (`tools/system_test.py`):** the real executables as separate processes over real sockets.
- **CI (GitHub Actions):** all of the above with plain, AddressSanitizer + UBSan, and ThreadSanitizer builds, plus an aarch64 cross-compile.

## Known limits

- Nothing is detected during the first 31 s, while the two averages fill. A knock during that time can also skew the baseline until it ages out.
- Sequence numbers reveal a lost packet only when a later one arrives, so a lost final packet isn't counted.
- The simulated quake is a synthetic P and S wave, not recorded seismic data, and the thresholds have not been tuned on a real site.
- The LIS3DH's noise floor limits it to strong, nearby shaking. This is a systems project, not a replacement for a real seismometer.
- Timing runs in user space on a standard Linux kernel, so it is not hard real-time. A `PREEMPT_RT` kernel and the chip's FIFO would be the next steps.
