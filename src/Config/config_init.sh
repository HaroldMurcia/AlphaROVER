#!/bin/bash
# -*- ENCODING: UTF-8 -*-

echo "
==============================================================================
Title           :config_init.sh
Description     :This script configures the alphaROVER
Author          :Harold F MURCIA - www.haroldmurcia.com
Date            :16/09/2020
Version         :0.1
Usage           :./config_init
Notes           :Executes just at first time
==============================================================================
"

today=$(date +%Y%m%d)
div=======================================

echo "
      _       _         _____  _____  _____  _____  _____
 ___ | | ___ | |_  ___ | __  ||     ||  |  ||   __|| __  |
| .'|| || . ||   || .'||    -||  |  ||  |  ||   __||    -|
|__,||_||  _||_|_||__,||__|__||_____| \___/ |_____||__|__|
        |_|
"
# Paths
path_alpha_config=$(pwd)

# CONFIGs
sudo chmod -R 777 $path_alpha_config$"/alphaROVER.sh"
sudo cp $path_alpha_config$"/99-alphaBot.rules" "/etc/udev/rules.d/99-alphaBot.rules"
sudo udevadm trigger
echo "source $path_alpha_config"/alphaROVER.sh"" >> ~/.bashrc
cd $path_alpha_config
source ~/.bashrc
