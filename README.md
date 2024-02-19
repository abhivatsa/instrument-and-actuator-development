# EtherCAT Installation Guide

This guide provides step-by-step instructions to install EtherCAT on your system. Follow these instructions to ensure a smooth installation process.

## Prerequisites

Before installing EtherCAT, make sure you have the following prerequisites installed on your system:

- `autoconf`
- `libtool`
- `build-essential`
- `pkg-config`
- `cmake`
- `libeigen3-dev`

You can install these prerequisites by running the following commands:

```bash
sudo apt-get install autoconf -y
sudo apt-get install libtool -y
sudo apt-get install build-essential -y
sudo apt-get install pkg-config -y
sudo apt  install cmake -y
sudo apt install libeigen3-dev -y
```
```bash
./bootstrap # to create the configure script, if downloaded from the repo

./configure --enable-sii-assign --enable-hrtimer --enable-cycles --disable-eoe --disable-8139too --sysconfdir=/etc
make all modules
```

... and as root:

```bash
make modules_install install
depmod
```
... and then customizing the appropriate configuration file:

```
nano /etc/sysconfig/ethercat # For init.d based distro
```
Make sure, that the 'udev' package is installed, to automatically create the
EtherCAT character devices. The character devices will be created with mode
0660 and group root by default. If you want to give normal users reading
access, create a udev rule like this:
```
echo KERNEL==\"EtherCAT[0-9]*\", MODE=\"0664\" > /etc/udev/rules.d/99-EtherCAT.rules
```
Now you can start the EtherCAT master:

```
/etc/init.d/ethercat start # For init.d based distro
```
Have a look at the examples/ subdirectory for some application examples.

