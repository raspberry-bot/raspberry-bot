#!/usr/bin/env bash

sudo mkdir -p $RASPBERRYBOT_ROOT
sudo mkdir -p $RASPBERRYBOT_ROOT/logs
sudo mkdir -p $SUPERVISOR_ROOT/conf
sudo mkdir -p $SUPERVISOR_ROOT/logs

sudo touch $RASPBERRYBOT_ROOT/logs/events.log

sudo -E bash -c "rm $RASPBERRYBOT_ROOT/api"
sudo -E bash -c "$RASPBERRYBOT_ROOT/web"
sudo ln -s $RASPBERRYBOT_ROOT/src/api $RASPBERRYBOT_ROOT
sudo ln -s $RASPBERRYBOT_ROOT/src/web $RASPBERRYBOT_ROOT

sudo cp $RASPBERRYBOT_ROOT/src/configs/bot-config.json $API_SERVER_ROOT/bot-config.json

# sudo cp $RASPBERRYBOT_ROOT/src/configs/firmware-config.txt /boot/firmware/config.txt
sudo cp $RASPBERRYBOT_ROOT/src/configs/netdata.conf /etc/netdata/netdata.conf

sudo -E bash -c "rm $SUPERVISOR_ROOT/conf/supervisord.conf"
sudo rm /etc/init.d/raspberrybot-api.sh
sudo -E bash -c "ln -s $RASPBERRYBOT_ROOT/src/configs/supervisord/supervisord.conf $SUPERVISOR_ROOT/conf"
sudo -E bash -c "ln -s $RASPBERRYBOT_ROOT/src/configs/init_scripts/raspberrybot-api.sh /etc/init.d/"

sudo rm /etc/nginx/nginx.conf
sudo -E bash -c "ruby $RASPBERRYBOT_ROOT/src/configs/nginx/nginx.rb"

# sudo -E bash -c "/etc/init.d/videocard"
# sudo -E bash -c "ln -s $RASPBERRYBOT_ROOT/src/configs/init_scripts/rc3.d-videocard /etc/init.d/videocard"

sudo -E bash -c "/etc/init.d/supervisord"
sudo -E bash -c "ln -s $RASPBERRYBOT_ROOT/src/configs/init_scripts/rc3.d-supervisord /etc/init.d/supervisord"