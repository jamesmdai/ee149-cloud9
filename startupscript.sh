#!/bin/sh -e

sleep 10
cd ~/ee149-cloud9
git fetch
git pull
cd lora
python3 radio_rfm69.py
