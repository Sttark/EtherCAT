# IgH EtherCAT Master Build Guide (Raspberry Pi 5)

This is the exact build/install workflow to get the IgH EtherCAT master working on this machine with:

- kernel: `6.12.67-v8-16k-rt`
- source: `/usr/src/igh-ethercat-master`
- EtherCAT NIC driver: `r8169` (`ec_r8169`)
- EoE disabled (`--disable-eoe`)

## 1) Preconditions

Run these checks first:

```bash
uname -r
ls -ld /usr/src/igh-ethercat-master
ls -ld /usr/src/raspberrypi-linux-rpi-6.12.y
```

Expected:

- `uname -r` prints `6.12.67-v8-16k-rt`
- both source directories exist

## 2) Configure + Build

```bash
cd /usr/src/igh-ethercat-master
sudo make clean
sudo ./configure --sysconfdir=/etc --enable-r8169 --with-r8169-kernel=6.12 --disable-eoe
sudo make all modules
```

Notes:

- `--with-r8169-kernel=6.12` is required for this tree (do not pass full `uname -r` here).
- `--disable-eoe` avoids EoE mailbox traffic in the master.

## 3) Install Modules + Tools

```bash
cd /usr/src/igh-ethercat-master
sudo make modules_install install
sudo depmod -a "$(uname -r)"
```

## 4) Clean Up Stale Duplicate Modules (one-time sanity check)

The active modules must resolve to:

- `/lib/modules/$(uname -r)/ethercat/master/ec_master.ko.xz`
- `/lib/modules/$(uname -r)/ethercat/devices/r8169/ec_r8169.ko.xz`

Check:

```bash
modinfo ec_master | awk '/filename|srcversion|vermagic/'
modinfo ec_r8169 | awk '/filename|srcversion|vermagic/'
```

If `modinfo` points to top-level stale copies like:

- `/lib/modules/.../ethercat/ec_master.ko.xz`
- `/lib/modules/.../ethercat/ec_r8169.ko.xz`

move them out of the way:

```bash
sudo mkdir -p "/lib/modules/$(uname -r)/ethercat/stale_backup"
sudo mv "/lib/modules/$(uname -r)/ethercat/ec_master.ko.xz" "/lib/modules/$(uname -r)/ethercat/stale_backup/" 2>/dev/null || true
sudo mv "/lib/modules/$(uname -r)/ethercat/ec_r8169.ko.xz" "/lib/modules/$(uname -r)/ethercat/stale_backup/" 2>/dev/null || true
sudo depmod -a "$(uname -r)"
```

## 5) Persistent Runtime Configuration

### 5.1 `/etc/ethercat.conf`

Set:

```ini
MASTER0_DEVICE="ff:ff:ff:ff:ff:ff"
DEVICE_MODULES="r8169"
UPDOWN_INTERFACES="eth1"
```

Why:

- wildcard master device avoids MAC churn from `ec_r8169`
- `r8169` ensures `ec_r8169` is used
- `eth1` is brought up/down by `ethercatctl`

### 5.2 Keep NetworkManager off EtherCAT NIC

Create:

`/etc/NetworkManager/conf.d/99-ethercat-eth1-unmanaged.conf`

```ini
[keyfile]
unmanaged-devices=interface-name:eth1
```

Then:

```bash
sudo systemctl restart NetworkManager
```

### 5.3 Pin EtherCAT OP kernel thread to CPU 2

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

## 7) Verify Working State

```bash
lsmod | awk 'NR==1 || $1 ~ /^ec_/ || $1=="realtek"'
sudo ethercat master
sudo ethercat slaves
```

Expected:

- `ec_master` and `ec_r8169` loaded
- `ethercat master` works (no ioctl error)
- slaves enumerate

Optional deep checks:

```bash
cat /sys/module/ec_master/parameters/run_on_cpu
nmcli device status | awk '$1=="eth1"'
sudo dmesg | awk '/EtherCAT|ec_r8169|Unknown symbol|disagrees|invalid module format|ioctl/' | tail -n 120
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
cd /usr/src/igh-ethercat-master
sudo make all modules
sudo make modules_install install
sudo depmod -a "$(uname -r)"
sudo systemctl restart ethercat
```

