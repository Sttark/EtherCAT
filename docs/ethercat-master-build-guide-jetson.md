# IgH EtherCAT Master Build Guide (Jetson)

This is the exact build/install workflow to get the IgH EtherCAT master working on this machine with:

- kernel: `5.15.148-rt-tegra`
- source: `/home/sttark/Desktop/github/igh-ethercat-master`
- EtherCAT NIC driver: `igb` (`ec_igb`) for Intel i210
- EoE disabled (`--enable-eoe=no`)

## 1) Preconditions

Run these checks first:

```bash
uname -r
ls -ld /home/sttark/Desktop/github/igh-ethercat-master
ls -ld "/lib/modules/$(uname -r)/build"
```

Expected:

- `uname -r` prints `5.15.148-rt-tegra`
- the IgH source directory exists
- `/lib/modules/$(uname -r)/build` exists and points to valid kernel headers

## 2) Configure + Build

```bash
cd /home/sttark/Desktop/github/igh-ethercat-master
make clean
./bootstrap
./configure --sysconfdir=/etc --with-linux-dir="/lib/modules/$(uname -r)/build" --enable-generic --enable-igb --enable-eoe=no
make -j"$(nproc)" all modules
```

Notes:

- `--enable-igb` is required for Intel i210 (`ec_igb`).
- `--enable-generic` keeps `ec_generic` available as a fallback.
- `--enable-eoe=no` disables EoE support in the master.

## 3) Install Modules + Tools

```bash
cd /home/sttark/Desktop/github/igh-ethercat-master
sudo make modules_install install
sudo depmod -a "$(uname -r)"
```

## 4) Clean Up Stale Duplicate Modules (one-time sanity check)

The active modules must resolve to:

- `/lib/modules/$(uname -r)/ethercat/master/ec_master.ko*`
- `/lib/modules/$(uname -r)/ethercat/devices/igb/ec_igb.ko*`

Check:

```bash
modinfo ec_master | awk '/filename|srcversion|vermagic/'
modinfo ec_igb | awk '/filename|srcversion|vermagic/'
```

If `modinfo` points to top-level stale copies like:

- `/lib/modules/.../ethercat/ec_master.ko*`
- `/lib/modules/.../ethercat/ec_igb.ko*`

move them out of the way:

```bash
sudo mkdir -p "/lib/modules/$(uname -r)/ethercat/stale_backup"
sudo mv "/lib/modules/$(uname -r)/ethercat/ec_master.ko" "/lib/modules/$(uname -r)/ethercat/stale_backup/" 2>/dev/null || true
sudo mv "/lib/modules/$(uname -r)/ethercat/ec_master.ko.xz" "/lib/modules/$(uname -r)/ethercat/stale_backup/" 2>/dev/null || true
sudo mv "/lib/modules/$(uname -r)/ethercat/ec_igb.ko" "/lib/modules/$(uname -r)/ethercat/stale_backup/" 2>/dev/null || true
sudo mv "/lib/modules/$(uname -r)/ethercat/ec_igb.ko.xz" "/lib/modules/$(uname -r)/ethercat/stale_backup/" 2>/dev/null || true
sudo depmod -a "$(uname -r)"
```

## 5) Persistent Runtime Configuration

### 5.1 `/etc/ethercat.conf`

Set:

```ini
MASTER0_DEVICE="ff:ff:ff:ff:ff:ff"
DEVICE_MODULES="igb"
UPDOWN_INTERFACES="<i210_interface_name>"
```

Find the i210 interface name first:

```bash
ip -br link
```

Then replace `<i210_interface_name>` with the real interface (for example `eth1` or `enP...`).

Why:

- wildcard master device avoids MAC churn from `ec_igb`
- `igb` ensures `ec_igb` is used for the i210
- explicit interface allows `ethercatctl` to bring link up/down correctly

### 5.2 Keep NetworkManager off EtherCAT NIC

Create:

`/etc/NetworkManager/conf.d/99-ethercat-i210-unmanaged.conf`

```ini
[keyfile]
unmanaged-devices=interface-name:<i210_interface_name>
```

Then:

```bash
sudo systemctl restart NetworkManager
```

### 5.3 Pin EtherCAT OP kernel thread to isolated CPU

Create:

`/etc/modprobe.d/ec_master.conf`

```conf
options ec_master run_on_cpu=2
```

Do not put `run_on_cpu=2` in `MODPROBE_FLAGS` in `/etc/ethercat.conf`.

## 6) Restart EtherCAT Stack

```bash
sudo systemctl restart ethercat
sudo systemctl status ethercat --no-pager -l
```

### 6.1 Required on this kernel: force-claim i210 from built-in `igb`

On this Jetson RT kernel, native `igb` is built into the kernel (`CONFIG_IGB=y`), so `ethercatctl` cannot unload it like a normal module. You must explicitly unbind the i210 PCI device from native `igb` and bind it to `ec_igb`.

Use the script in this docs folder:

- `/home/sttark/Desktop/github/EtherCAT/docs/jetson-claim-i210-ec_igb.sh`

Run once manually:

```bash
sudo bash /home/sttark/Desktop/github/EtherCAT/docs/jetson-claim-i210-ec_igb.sh
sudo systemctl restart ethercat
```

Install it as a persistent service:

```bash
sudo install -m 0755 /home/sttark/Desktop/github/EtherCAT/docs/jetson-claim-i210-ec_igb.sh /usr/local/sbin/jetson-claim-i210-ec_igb.sh
sudo install -m 0644 /home/sttark/Desktop/github/EtherCAT/docs/jetson-claim-i210-ec_igb.service /etc/systemd/system/jetson-claim-i210-ec_igb.service
sudo systemctl daemon-reload
sudo systemctl enable --now jetson-claim-i210-ec_igb.service
```

If you update the script or unit later, reinstall and restart:

```bash
sudo install -m 0755 /home/sttark/Desktop/github/EtherCAT/docs/jetson-claim-i210-ec_igb.sh /usr/local/sbin/jetson-claim-i210-ec_igb.sh
sudo install -m 0644 /home/sttark/Desktop/github/EtherCAT/docs/jetson-claim-i210-ec_igb.service /etc/systemd/system/jetson-claim-i210-ec_igb.service
sudo systemctl daemon-reload
sudo systemctl restart jetson-claim-i210-ec_igb.service
sudo systemctl restart ethercat
```

## 7) Verify Working State

```bash
lsmod | awk 'NR==1 || $1=="ec_master" || $1=="ec_igb" || $1=="igb"'
sudo ethercat master
sudo ethercat slaves
```

Expected:

- `ec_master` and `ec_igb` loaded
- `ethercat master` works (no ioctl error)
- slaves enumerate when EtherCAT network is present

Observed working output on this machine:

```text
0  0:0  PREOP  +  EtherCAT Adapter(Rev2.0)
1  0:1  PREOP  +  Kossi KSD N2 EtherCAT Servo Drives
2  0:2  PREOP  +  MicroFlex e190 Build 5904.6 (CoE)
3  5:0  PREOP  +  MicroFlex e190 Build 5904.6 (CoE)
4  5:1  PREOP  +  MicroFlex e190 Build 5904.6 (CoE)
5  5:2  PREOP  +  MicroFlex e190 Build 5904.6 (CoE)
6  1:0  PREOP  +  Kossi KSD N2 EtherCAT Servo Drives
7  1:0  PREOP  +  Kossi KSD N2 EtherCAT Servo Drives
8  1:0  PREOP  +  Kossi KSD N2 EtherCAT Servo Drives
```

Optional deep checks:

```bash
cat /sys/module/ec_master/parameters/run_on_cpu
nmcli device status | awk '$1=="<i210_interface_name>"'
sudo dmesg | awk '/EtherCAT|ec_igb|Unknown symbol|disagrees|invalid module format|ioctl/' | tail -n 120
```

## 8) Known Gotchas

1. `make install` overwrites `/etc/ethercat.conf` with defaults.  
   Re-apply runtime config after each install.

2. If `sudo ethercat master` shows ioctl errors (`Inappropriate ioctl for device`), check `/dev/EtherCAT0` major:

```bash
ls -l /dev/EtherCAT0
awk 'tolower($2)=="ethercat"{print}' /proc/devices
```

If mismatched:

```bash
EC_MAJOR=$(awk 'tolower($2)=="ethercat"{print $1}' /proc/devices)
sudo rm -f /dev/EtherCAT0
sudo mknod /dev/EtherCAT0 c "$EC_MAJOR" 0
sudo chmod 666 /dev/EtherCAT0
```

3. If service restart fails because modules are in use, stop client processes first (for example your Python app), then restart:

```bash
sudo systemctl restart ethercat
```

## 9) Minimal Rebuild Shortcut

Use this when source changes are already in place:

```bash
cd /home/sttark/Desktop/github/igh-ethercat-master
make -j"$(nproc)" all modules
sudo make modules_install install
sudo depmod -a "$(uname -r)"
sudo systemctl restart ethercat
```

