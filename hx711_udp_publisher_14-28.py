#!/usr/bin/env python3
"""
HX711 -> UDP publisher with robust warmup and outlier filter.

Runs on the BlueROV2 Raspberry Pi (as hx711.service).
Reads the HX711 load-cell amplifier on SERIAL 3 of the Navigator
(GPIO 4 = SCK, GPIO 5 = DT) and sends each reading as a UDP
packet to the laptop.

Warmup logic:
  - Discards the first WARMUP_DISCARD samples entirely
  - Then collects the next WARMUP_COLLECT samples and uses their median
    as the initial last_good value
  - No publishing happens during warmup
  - This prevents a single bad first sample from poisoning last_good

Outlier filter (active after warmup):
  - Specific known-garbage bit patterns (-1)
  - Any reading that jumps more than JUMP_THRESHOLD from last_good
  - When rejected, the last good value is re-sent to keep rate steady

Deploy:
    sudo cp hx711_udp_publisher.py /home/pi/
    sudo systemctl restart hx711.service
"""

import json
import socket
import statistics
import sys
import time

try:
    import RPi.GPIO as GPIO
except ImportError:
    print("RPi.GPIO not installed. Run: sudo pip3 install RPi.GPIO")
    sys.exit(1)


# ---------- CONFIG ----------
LAPTOP_IP = "192.168.2.1"     # standard BlueROV tether laptop IP
UDP_PORT  = 9870              # must match force_bridge_node.py
RATE_HZ   = 20                # readings per second

PIN_SCK = 4                   # SERIAL 3 pin 2 (TX)
PIN_DT  = 5                   # SERIAL 3 pin 3 (RX)

# Outlier filter
JUMP_THRESHOLD   = 100_000    # max legitimate change between consecutive samples
KNOWN_GARBAGE    = {-1}       # specific corrupt bit patterns observed

# Robust warmup
WARMUP_DISCARD   = 10         # throw away first N samples (ignore transients)
WARMUP_COLLECT   = 10         # then collect N samples, use median as baseline
WARMUP_SPREAD    = 50_000     # samples must agree within this for warmup to succeed
# ----------------------------


def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(PIN_SCK, GPIO.OUT)
    GPIO.setup(PIN_DT, GPIO.IN)
    GPIO.output(PIN_SCK, False)
    time.sleep(0.1)


def wait_ready(timeout_s=0.5):
    start = time.time()
    while GPIO.input(PIN_DT) == GPIO.HIGH:
        if time.time() - start > timeout_s:
            return False
        time.sleep(0.001)
    return True


def read_raw():
    """Read one 24-bit sample, channel A, gain 128."""
    if not wait_ready():
        return None
    value = 0
    for _ in range(24):
        GPIO.output(PIN_SCK, True)
        value = (value << 1) | GPIO.input(PIN_DT)
        GPIO.output(PIN_SCK, False)
    GPIO.output(PIN_SCK, True)
    GPIO.output(PIN_SCK, False)
    if value & 0x800000:
        value -= 1 << 24
    return value


def robust_warmup(period):
    """Run warmup sequence. Returns initial last_good value, or None if failed."""
    print(f"Warmup: discarding first {WARMUP_DISCARD} samples...")
    discarded = 0
    while discarded < WARMUP_DISCARD:
        t0 = time.time()
        read_raw()  # read and throw away
        discarded += 1
        elapsed = time.time() - t0
        if elapsed < period:
            time.sleep(period - elapsed)

    print(f"Warmup: collecting {WARMUP_COLLECT} stable samples...")
    samples = []
    attempts = 0
    max_attempts = WARMUP_COLLECT * 3  # give up after 3x tries
    while len(samples) < WARMUP_COLLECT and attempts < max_attempts:
        t0 = time.time()
        raw = read_raw()
        attempts += 1
        if raw is not None and raw not in KNOWN_GARBAGE:
            samples.append(raw)
        elapsed = time.time() - t0
        if elapsed < period:
            time.sleep(period - elapsed)

    if len(samples) < WARMUP_COLLECT:
        print(f"Warmup failed: only got {len(samples)} valid samples.")
        return None

    spread = max(samples) - min(samples)
    if spread > WARMUP_SPREAD:
        print(f"Warmup failed: samples too noisy (spread={spread}, "
              f"max allowed={WARMUP_SPREAD}). Samples: {samples}")
        return None

    baseline = int(statistics.median(samples))
    print(f"Warmup OK: baseline={baseline} (spread={spread})")
    return baseline


def is_outlier(raw, last_good):
    """Decide whether to reject this reading (post-warmup only)."""
    if raw is None:
        return True
    if raw in KNOWN_GARBAGE:
        return True
    if abs(raw - last_good) > JUMP_THRESHOLD:
        return True
    return False


def main():
    setup_gpio()
    sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / RATE_HZ

    print(f"Sending HX711 readings to {LAPTOP_IP}:{UDP_PORT} at {RATE_HZ} Hz")
    print(f"Filter: reject jumps > {JUMP_THRESHOLD} or values in {KNOWN_GARBAGE}")

    # Run warmup. If it fails, retry forever until it succeeds.
    last_good = None
    while last_good is None:
        last_good = robust_warmup(period)
        if last_good is None:
            print("Retrying warmup in 2 seconds...")
            time.sleep(2.0)

    print("Publishing started. Press CTRL+C to stop.\n")

    samples_seen   = 0
    rejected_count = 0

    try:
        while True:
            t0  = time.time()
            raw = read_raw()

            if is_outlier(raw, last_good):
                rejected_count += 1
                out_raw = last_good
                valid   = True
                if rejected_count % 20 == 1:
                    print(f"Rejected raw={raw} (total rejects: {rejected_count})")
            else:
                out_raw   = raw
                last_good = raw
                valid     = True

            samples_seen += 1
            pkt = {
                "stamp": time.time(),
                "raw":   int(out_raw),
                "valid": valid,
            }
            sock.sendto(json.dumps(pkt).encode("utf-8"),
                        (LAPTOP_IP, UDP_PORT))

            if samples_seen % 20 == 0:
                print(f"raw = {out_raw:>10d}  (rejects: {rejected_count})")

            elapsed = time.time() - t0
            if elapsed < period:
                time.sleep(period - elapsed)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        GPIO.cleanup()
        sock.close()


if __name__ == "__main__":
    main()
