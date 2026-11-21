# Setup and Run Guide

---

## Option 1: Build from Source

Step 1: Download full environment (7GB)
https://drive.google.com/file/d/1nJowpYsaRMkd-KCEaE6X97rAPc1WYu8L/view?usp=sharing

Step 2: Extract
cd ~
tar xzf zephyr_full_setup.tar.gz

Step 3: Build
source ~/zephyr_microros_venv/bin/activate
source ~/zephyrproject/zephyr/zephyr-env.sh
cd ~/zephyr_project
west build -b nucleo_h755zi_q/stm32h755xx/m7

Step 4: Flash
west flash

---

## Option 2: Flash Pre-built Binary

Step 1: Clone
git clone https://github.com/ephemeris-void/apsa.git
cd apsa

Step 2: Flash
west flash --hex-file prebuilt/zephyr.hex
