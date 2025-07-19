# 🚀 Raspberry Pi 4 Device Driver Sandbox

[![Linux Kernel](https://img.shields.io/badge/Linux-6.12-blue.svg)](https://www.kernel.org)
[![Target: Raspberry Pi 4](https://img.shields.io/badge/Target-RPi4-yellowgreen.svg)](https://www.raspberrypi.com)
[![Language: C](https://img.shields.io/badge/Language-C-informational.svg)](https://en.wikipedia.org/wiki/C_%28programming_language%29)
[![License: MIT](https://img.shields.io/badge/License-MIT-lightgrey.svg)](LICENSE)

---

## 🎯 About

This repository is my **Linux sandbox** for building, testing, and iterating on Linux kernel **device drivers** targeted at the **Raspberry Pi 4** platform. Whether you're writing GPIO togglers, SPI controllers, or full-fledged SoC drivers, this space lets you experiment safely without affecting a production system.

## ⚙️ Features

* **Automated Build**: Scripts to cross-compile against kernel 6.12 for Raspberry Pi 4.
* **Test Suite**: Basic load/unload and functionality tests under `/tests/`.
* **Device Tree Overlays**: Examples to bind custom drivers to GPIO, SPI, I²C, and more.

## 🧰 Prerequisites

* A running **Raspberry Pi 4** with kernel 6.12.
* **GNU toolchain** (gcc-13 or compatible) with cross-compilation support.
* **bc**, **make**, **git**, **device-tree-compiler** (`dtc`).

## 🚀 Getting Started

1. **Clone** the repo:

   ```bash
   git clone https://github.com/EduardoGMohedano/Linux-sandbox
   cd Linux-sandbox
   ```

2. **Configure** your cross-compiler in `scripts/env.sh`:

   ```bash
   export ARCH=arm64
   export CROSS_COMPILE=aarch64-linux-gnu-
   source scripts/env.sh
   ```

3. **Build** the kernel modules:

   ```bash
   ./scripts/build.sh
   ```

4. **Load** your module on the Pi:

   ```bash
   scp out/<module>.ko pi@raspberrypi:/home/pi/
   ssh pi@raspberrypi "sudo insmod <module>.ko"
   ```

5. **Run** tests:

   ```bash
   ./tests/run_tests.sh
   ```

## 📂 Repository Structure

```
├── docs/               # Architectural diagrams & notes
├── drivers/            # Your custom driver implementations
├── scripts/            # Build, deploy, and test helpers
├── tests/              # Basic shell-based test scripts
├── overlays/           # Device Tree Overlays examples
├── out/                # Compiled modules and artifacts
├── .gitignore
├── LICENSE
└── README.md           # ← You are here
```

## 📜 License

This project is licensed under the **MIT License**. See [LICENSE](LICENSE) for details.

---

*Happy hacking on the Pi 4!*
