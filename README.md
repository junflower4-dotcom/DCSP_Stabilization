# Gimbal Stabilization Loop

This repository contains C-based control code for a gimbal stabilization loop experiment utilizing NI-DAQmx.  
The program handles real-time data acquisition, sensor signal processing, applies control algorithms (PD/PI) along with a Low-Pass Filter (LPF), and logs the experimental data.

## 🎬 Experimental Video
Click the image below to watch the gimbal stabilization loop demonstration on YouTube Shorts.

### Demo Video
| Stabilization Test |
| :---: |
| <a href="https://youtube.com/shorts/oQwa-0CZZsY"><img src="https://img.youtube.com/vi/oQwa-0CZZsY/0.jpg" width="300"></a> |
| Stabilization Loop Demo |

---

## File Structure

- `main.c`: Program initialization, control loop execution, data logging, and termination sequence.
- `mygimbal_runexp.c`: DAQ I/O operations, sensor processing, control algorithms, and data saving.
- `mygimbal_runexp.h`: Shared constants and function declarations.
- `mygimbal_stabilization.c`: Stabilization experiment input logic and experimental information.
- `mygimbal_stabilization.h`: Configurations for sampling frequency, trial duration, and output file.

## Key Specifications & Settings

- **Sampling Frequency:** 200 Hz
- **Gyro LPF Cutoff Frequency:** 10 Hz
- **Designation Controller Type:** PD Controller
- **Stabilization Controller Type:** PI Controller
- **Emergency Stop Key:** `f`
- **Output File:** `Stabilization_Loop.txt`

## DAQ Channel Configuration

- `Dev4/ai0`: Disturbance input
- `Dev4/ai2`: Gyro voltage
- `Dev4/ai3`: Potentiometer voltage
- `Dev4/ao0`: Switch command
- `Dev4/ao1`: Motor command

## System Requirements & Environment

This project is configured and tested under the following environment:

- **OS:** Windows
- **IDE:** Visual Studio
- **Driver:** NI-DAQmx driver with C development support
- **Device Name:** NI DAQ device configured as `Dev4` in NI MAX

## Pre-execution Checklists & Notes

- **Channel Verification:** Always verify that the physical wiring and hardware channel settings match the DAQ configurations specified in the code before running the experiment.
- **Git Target:** The generated experimental text files (`*.txt`) are untracked and automatically ignored by `.gitignore` by default.
