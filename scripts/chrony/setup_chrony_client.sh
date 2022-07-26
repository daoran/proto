#!/bin/bash
set -e
sudo apt-get remove ntp ntpdate -qq
sudo apt-get install chrony -qq
sudo cp configs/chrony_client.conf /etc/chrony/chrony.conf
sudo systemctl restart chrony.service
sleep 2
sudo systemctl --no-pager status chrony.service
chronyc sources
