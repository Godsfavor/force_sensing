# HX711 Gripper Force Sensing — BlueROV2

This system reads force data from an HX711 load-cell amplifier mounted on the
BlueROV2 gripper, streams it over UDP through the tether, and publishes it as
a ROS2 topic on your laptop.

---



## Hardware Setup

- **HX711 SCK** → GPIO **4** (SERIAL 3, pin 2 / TX)
- **HX711 DT** → GPIO **5** (SERIAL 3, pin 3 / RX)
- Tether IP: Pi = `192.168.2.2`, Laptop = `192.168.2.1`
- UDP Port: **9870**

---

## Deploying to the BlueROV2 Pi

### 1. Copy files to the Pi


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
sudo systemctl enable


### 4. Check it is running

```bash
sudo systemctl status

## Running the ROS2 Bridge (Laptop)

Make sure your ROS2 environment is sourced, then:

```bash
ros2 run gripper_force_bridge.py or your launch file
```

Force data will appear on `/force`:

```bash
ros2 topic echo /gripper/force
```

---

### Layer 2 — Laptop side (`gripper_force_bridge.py`)

A 3-sample rolling on the raw counts kills any remaining
single-sample electrical spikes. At 20 Hz this adds ~100 ms latency,
which is negligible for gripper control. 

---


