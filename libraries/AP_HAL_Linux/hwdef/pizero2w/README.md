# Low Latency ArduPilot Setup on Raspberry Pi Zero 2 W

This guide documents the configuration required to achieve sub-3ms latency (typically 1.7ms - 2.5ms) when running ArduPilot on a Raspberry Pi Zero 2 W, communicating via UART to a secondary flight controller (e.g., Betaflight).

- **Hardware:** Raspberry Pi Zero 2 W
- **OS:** Raspberry Pi OS (Debian 12 / Bookworm)
- **Goal:** Enable Real-Time scheduling and Hardware UART (PL011) to eliminate latency caused by the Mini-UART and standard Linux scheduler.

## 1. Enable Hardware UART (PL011)

The Pi Zero 2 W defaults to using the "Mini UART" for the GPIO pins, which is tied to the VPU clock and causes high latency (~20ms). We must disable Bluetooth to repurpose the high-performance hardware UART (PL011) for the flight controller.

1. Edit the config file:
   ```bash
   sudo nano /boot/firmware/config.txt
   ```
   *(Note: On older OS versions, this is `/boot/config.txt`)*

2. Add the following lines to the end of the file:
   ```
   # Disable Bluetooth to free up PL011 Hardware UART
   dtoverlay=disable-bt

   # Force UART to be enabled (essential since we are removing the console)
   enable_uart=1
   ```

3. Save and exit (`Ctrl+O`, `Enter`, `Ctrl+X`).

## 2. Kernel Optimization (CPU Isolation)

We isolate CPU cores 2-3 for ArduPilot and remove the serial console to prevent boot logs from interfering with the flight controller connection.

1. Edit the kernel command line:
   ```bash
   sudo nano /boot/firmware/cmdline.txt
   ```

2. **Remove** any reference to the serial console, such as `console=serial0,115200` or `console=ttyAMA0,115200`. (Keep `console=tty1`).

3. **Append** the following flags to the end of the line (do not create a new line, keep it all on one line):
   ```
   isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3 irqaffinity=0-1
   ```

4. Save and exit.

## 3. Disable Real-Time Throttling

By default, Linux prevents real-time processes from using 100% of the CPU (reserving 5% for system tasks). Since cores 2-3 are isolated, we want ArduPilot to have 100% access without being throttled.

1. Edit sysctl.conf:
   ```bash
   sudo nano /etc/sysctl.conf
   ```

2. Add the following line to the end of the file:
   ```
   kernel.sched_rt_runtime_us = -1
   ```

3. Save and exit.

## 4. Systemd Service Configuration

Create a service file that pins ArduPilot to isolated cores 2-3 and sets the CPU governor to performance.

**Note:** We do NOT set `CPUSchedulingPolicy=fifo` in systemd because ArduPilot's internal scheduler handles real-time thread priorities. Setting it in systemd would affect all threads including library threads (like pigpio) that need normal scheduling.

**Network Update:** We use `tcp:0.0.0.0:5770` instead of a specific IP. This puts ArduPilot in **Server Mode**, waiting for a connection from *any* Ground Control Station IP, avoiding hardcoding issues.

1. Create/Edit the service file:
   ```bash
   sudo nano /etc/systemd/system/ardupilot.service
   ```

2. Paste the following configuration:
   ```ini
   [Unit]
   Description=Autopilot service
   After=systemd-modules-load.service

   [Service]
   Type=simple
   User=root

   # --- Core Isolation ---
   # Pin to cores 2-3 (matches isolcpus in cmdline.txt)
   CPUAffinity=2 3

   # --- Process Priority ---
   # Nice is less important when using FIFO, but good practice
   Nice=-20
   LimitNICE=-20

   # --- IO Priority ---
   IOSchedulingClass=realtime
   IOSchedulingPriority=0

   # --- CPU Governor & Command ---
   # Force CPU to performance mode to prevent clock-down latency
   ExecStartPre=/bin/bash -c "echo performance | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor"

   # ARDUPILOT COMMAND
   # --serial0 tcp:0.0.0.0:5770 -> Listens for TCP connections on Port 5770 (Server Mode)
   # --serial1 /dev/serial0     -> Connects to Betaflight via Hardware UART
   ExecStart=/home/ardupilot/arducopter --serial0 tcp:0.0.0.0:5770 --serial1 /dev/serial0

   Restart=on-failure

   [Install]
   WantedBy=multi-user.target
   ```

3. Reload the systemd daemon:
   ```bash
   sudo systemctl daemon-reload
   ```

## 5. Finalizing

Reboot the system to apply the kernel and UART changes.

```bash
sudo reboot
```

## 6. Verification

After rebooting, verify the configuration.

### Check CPU Isolation
```bash
cat /sys/devices/system/cpu/isolated
```
**Expected:** `2-3`

### Check IRQ Affinity
```bash
for i in /proc/irq/*/smp_affinity_list; do echo "$i: $(cat $i)"; done | head -5
```
**Expected:** All IRQs on `0-1` only

### Check ArduPilot Threads
```bash
PID=$(pgrep arducopter)
ps -L -p $PID -o tid,psr,cls,rtprio,comm
```

**Expected Output:**
- ArduPilot threads (`ap-timer`, `ap-uart`, `ap-rcin`, `ap-io`, etc.) should show:
  - **PSR:** `2` or `3` (running on isolated cores)
  - **CLS:** `FF` (FIFO / Real-Time)
  - **RTPRIO:** `10-15` (ArduPilot sets these internally)
- Library threads (e.g., pigpio) may show `TS` (normal scheduling) - this is correct

If ArduPilot threads show `TS` instead of `FF`, the real-time configuration failed.
