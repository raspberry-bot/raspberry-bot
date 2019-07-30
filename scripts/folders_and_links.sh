#!/usr/bin/env bash

sudo mkdir -p $GREENBOTS_ROOT
sudo mkdir -p $GREENBOTS_ROOT/logs
sudo mkdir -p $SUPERVISOR_ROOT/conf
sudo mkdir -p $SUPERVISOR_ROOT/logs

sudo touch $GREENBOTS_ROOT/logs/events.log

sudo -E bash -c "rm $GREENBOTS_ROOT/api"
sudo -E bash -c "$GREENBOTS_ROOT/web"
sudo ln -s $GREENBOTS_ROOT/src/api $GREENBOTS_ROOT
sudo ln -s $GREENBOTS_ROOT/src/web $GREENBOTS_ROOT

sudo cp $GREENBOTS_ROOT/src/configs/bot-config.json $API_SERVER_ROOT/bot-config.json

sudo cp $GREENBOTS_ROOT/src/configs/firmware-config.txt /boot/firmware/config.txt
sudo cp $GREENBOTS_ROOT/src/configs/netdata.conf /etc/netdata/netdata.conf

sudo -E bash -c "rm $SUPERVISOR_ROOT/conf/supervisord.conf"
sudo rm /etc/init.d/greenbots-api.sh
sudo -E bash -c "ln -s $GREENBOTS_ROOT/src/configs/supervisord/supervisord.conf $SUPERVISOR_ROOT/conf"
sudo -E bash -c "ln -s $GREENBOTS_ROOT/src/configs/init_scripts/greenbots-api.sh /etc/init.d/"

sudo rm /etc/nginx/nginx.conf
sudo -E bash -c "ruby $GREENBOTS_ROOT/src/configs/nginx/nginx.rb"

sudo -E bash -c "/etc/init.d/videocard"
sudo -E bash -c "ln -s $GREENBOTS_ROOT/src/configs/init_scripts/rc3.d-videocard /etc/init.d/videocard"

sudo -E bash -c "/etc/init.d/supervisord"
sudo -E bash -c "ln -s $GREENBOTS_ROOT/src/configs/init_scripts/rc3.d-supervisord /etc/init.d/supervisord"