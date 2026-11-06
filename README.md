<!-- SPDX-License-Identifier: GPL-2.0-only -->

# rtlmq

`rtlmq` is a Linux Ethernet driver for Realtek RTL8125B 2.5 GbE network adapters. It has been tested on real RTL8125B hardware.

## Supported hardware

- **Tested:** RTL8125B (`10ec:8125`, XID `0x641`, configuration method 4 or 5).
- **Not yet tested:** RTL8126A. Experimental code is included, but compatibility is not guaranteed.
- **Not supported:** RTL8127 and other RTL8125 revisions.

Not every adapter labeled “RTL8125” is compatible. Check your adapter with `lspci -nnk` if you're unsure.

## Features

- 2.5 Gb Ethernet with multiple receive queues
- Hardware checksum, VLAN, and TCP segmentation offloads
- XDP and AF_XDP zero-copy support
- Jumbo frames, Wake-on-LAN, and Energy Efficient Ethernet
- PTPv2 hardware timestamping over UDP/IPv4
- Configuration and statistics through `ethtool`

The defaults are intended for normal use. Advanced tuning is optional.

## Build and install

You need:

- A compiler, `make`, and the build files for your running Linux kernel
- Realtek firmware `rtl_nic/rtl8125b-2.fw` (provided by your distribution's firmware package)

From the source directory, build and install the module:

```sh
make production
sudo install -D -m 0644 rtlmq.ko /lib/modules/$(uname -r)/updates/rtlmq.ko
sudo depmod -a
sudo modprobe rtlmq
```

If your system requires signed kernel modules (for example, with Secure Boot enabled), sign `rtlmq.ko` before installing it.

**Important:** Loading `rtlmq` does not automatically replace another driver. If the adapter is already using `r8169`, you must switch drivers separately. Do this from the computer itself or through another network connection, because switching can temporarily disconnect you.

## Check that it's working

Find your network interface:

```sh
ip -br link
```

Replace `enp3s0` below with your interface name:

```sh
ethtool -i enp3s0    # Check the active driver
ethtool enp3s0       # Check the link and speed
ethtool -S enp3s0    # View statistics
```

The active driver should be `rtlmq`. If it still says `r8169`, the adapter has not switched to `rtlmq`.

## Known limitations

- Hardware support is currently limited to the tested RTL8125B revision.
- Some advanced offloads work only with standard 1500-byte packets.
- The maximum jumbo-frame size, some power-management behavior, and certain performance settings have not been fully tested.

## Report a problem

When reporting an issue, include what happened and the output of:

```sh
uname -r
modinfo rtlmq
lspci -nnk
ethtool -i enp3s0  # Replace with your interface
```

Please also include relevant kernel log messages (`dmesg`) and steps to reproduce the issue.

## License

GPL-2.0-only. Firmware has its own license.
