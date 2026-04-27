# HX711 Gripper Force Sensing — BlueROV2

This system reads force data from an HX711 load-cell amplifier mounted on the
BlueROV2 gripper, streams it over UDP through the tether, and publishes it as
a ROS2 topic on your laptop.

---

## System Overview

```
BlueROV2 Raspberry Pi                       Laptop
─────────────────────────────────           ──────────────────────────────
HX711 (GPIO 4/5)                            
    │                                       
    ▼                                       
hx711_udp_publisher_14-48.py   ──UDP──▶    gripper_force_bridge.py
(runs as hx711.service)         9870         │
                                             ▼
                                        /gripper/force
                                    (geometry_msgs/WrenchStamped)
```

| File | Runs on | Purpose |
|------|---------|---------|
| `hx711_udp_publisher_14-48.py` | BlueROV2 Pi | Reads HX711 sensor, sends UDP packets |
| `hx711_14-48.service` | BlueROV2 Pi | systemd service to auto-start the publisher |
| `gripper_force_bridge.py` | Your laptop | Receives UDP, publishes ROS2 topic |

---

## Hardware Setup

- **HX711 SCK** → GPIO **4** (SERIAL 3, pin 2 / TX)
- **HX711 DT** → GPIO **5** (SERIAL 3, pin 3 / RX)
- Tether IP: Pi = `192.168.2.2`, Laptop = `192.168.2.1`
- UDP Port: **9870**

---

## Deploying to the BlueROV2 Pi

### 1. Copy files to the Pi

```bash
scp hx711_udp_publisher_14-48.py pi@192.168.2.2:/tmp/
scp hx711_14-48.service          pi@192.168.2.2:/tmp/
```

### 2. Install on the Pi

```bash
ssh pi@192.168.2.2

sudo mv /tmp/hx711_udp_publisher_14-48.py /home/pi/
sudo mv /tmp/hx711_14-48.service /etc/systemd/system/

sudo pip3 install RPi.GPIO
```

### 3. Enable and start the service

```bash
sudo systemctl daemon-reload
sudo systemctl enable hx711_14-48.service
sudo systemctl start  hx711_14-48.service
```

### 4. Check it is running

```bash
sudo systemctl status hx711_14-48.service
# or follow the live log:
journalctl -u hx711_14-48.service -f
```

## Running the ROS2 Bridge (Laptop)

Make sure your ROS2 environment is sourced, then:

```bash
ros2 run gripper_force_bridge.py or your launch file
```

Force data will appear on `/gripper/force`:

```bash
ros2 topic echo /gripper/force
```

---

### Layer 2 — Laptop side (`gripper_force_bridge.py`)

A 3-sample rolling median on the raw counts kills any remaining
single-sample electrical spikes. At 20 Hz this adds ~100 ms latency,
which is negligible for gripper control. Set `median_window:=1` to disable.

---

## Configuration Reference

### Pi publisher (`hx711_udp_publisher_14-48.py`)

| Constant | Default | Description |
|----------|---------|-------------|
| `LAPTOP_IP` | `192.168.2.1` | UDP destination |
| `UDP_PORT` | `9870` | Must match bridge |
| `RATE_HZ` | `20` | Publishing rate |
| `PIN_SCK` | `4` | GPIO clock pin |
| `PIN_DT` | `5` | GPIO data pin |
| `JUMP_THRESHOLD` | `100,000` | Max count change per sample |
| `WARMUP_DISCARD` | `10` | Samples thrown away at startup |
| `WARMUP_COLLECT` | `10` | Samples used to establish baseline |
| `RECOVERY_SAMPLES` | `10` | Rejected samples needed to trigger auto-recovery |
| `RECOVERY_SPREAD` | `50,000` | Agreement tolerance for auto-recovery |

### Laptop bridge (`gripper_force_bridge.py`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `offset` | `0.0` | Raw count at zero load |
| `slope` | `1.0` | Raw counts per Newton |
| `frame_id` | `gripper` | ROS2 frame ID in message header |
| `median_window` | `3` | Rolling median window size (set to 1 to disable) |

---

