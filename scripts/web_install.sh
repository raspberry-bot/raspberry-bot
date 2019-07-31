#!/bin/bash

export DEBIAN_FRONTEND=noninteractive

sudo apt update
sudo apt -y upgrade

sudo apt -y install nginx python3-pip wireless-tools

sudo pip3 install tornado supervisor wifi

sudo bash -c "cat > /etc/nginx/nginx.conf" << EOL
user www-data;
worker_processes auto;
pid /run/nginx.pid;

include /etc/nginx/modules-enabled/*.conf;

error_log /var/log/nginx/error.log;

events {
    worker_connections 1024;
    use epoll;
}

http {
    root    /opt/raspberrybot/interfaces/web/;
    index   index.html;

    upstream frontends {
        server 127.0.0.1:8001;
        server 127.0.0.1:8002;
        server 127.0.0.1:8003;
        server 127.0.0.1:8004;
    }
    upstream operate {
        server 127.0.0.1:8101;
    }

    access_log /var/log/nginx/access.log;

    include /etc/nginx/mime.types;
    default_type application/octet-stream;

    types_hash_max_size 2048;
    server_tokens off;
    proxy_http_version 1.1;
    proxy_read_timeout 200;
    
    sendfile on;

    tcp_nopush on;
    tcp_nodelay on;

    gzip on;
    gzip_min_length 1000;
    gzip_proxied any;
    gzip_types text/plain text/html text/css text/xml
               application/x-javascript application/xml
               application/atom+xml text/javascript;

  	ssl_protocols TLSv1 TLSv1.1 TLSv1.2; # Dropping SSLv3, ref: POODLE
	ssl_prefer_server_ciphers on;

    # Only retry if there was a communication error, not a timeout
    # on the Tornado server (to avoid propagating "queries of death"
    # to all frontends)
    proxy_next_upstream error;

    server {
        listen 80;
        server_name raspberrybot.local;

        # Allow file uploads
        client_max_body_size 50M;

        location ^~ /assets/ {
            root /opt/raspberrybot/interfaces/web/;
            if (\$query_string) {
                expires max;
            }
        }
        location = /favicon.ico {
            rewrite (.*) /assets/favicon.ico;
        }
        location = /robots.txt {
            rewrite (.*) /assets/robots.txt;
        }

        location /api/operate {
            proxy_pass_header Server;
            proxy_set_header Host \$http_host;
            proxy_redirect off;
            proxy_set_header X-Real-IP \$remote_addr;
            proxy_set_header X-Scheme \$scheme;
            proxy_pass http://operate;
        }

        location /api {
            proxy_pass_header Server;
            proxy_set_header Host \$http_host;
            proxy_redirect off;
            proxy_set_header X-Real-IP \$remote_addr;
            proxy_set_header X-Scheme \$scheme;
            proxy_pass http://frontends;
        }
    }
}
EOL

sudo mkdir -p /var/log/supervisor
sudo mkdir -p /opt/raspberrybot/config
sudo mkdir -p /opt/raspberrybot/logs

echo "{}" | sudo tee -a /opt/raspberrybot/config/config.json
sudo touch /opt/raspberrybot/logs/events.log
sudo chown -R www-data /opt/raspberrybot/config
sudo chown -R www-data /opt/raspberrybot/logs
sudo usermod -a -G adm www-data

sudo bash -c "cat > /etc/supervisord.conf" << EOL
[supervisord]
logfile=/var/log/supervisord.log
logfile_maxbytes=50MB
logfile_backups=3
loglevel=info
pidfile=/var/run/supervisord.pid
nodaemon=false

[supervisorctl]
serverurl = unix://supervisord.sock

[unix_http_server]
file = supervisord.sock

[rpcinterface:supervisor]
supervisor.rpcinterface_factory = supervisor.rpcinterface:make_main_rpcinterface

[group:tornado_cluster]
programs = tornado_web_cluster

[group:operate_cluster]
programs = tornado_operate_web_cluster

[program:tornado_web_cluster]
numprocs = 4
numprocs_start = 1
command = python3 /opt/raspberrybot/interfaces/web/server.py --port=80%(process_num)02d
process_name = %(program_name)s%(process_num)d
redirect_stderr = true
stderr_logfile = /var/log/supervisor/tornado-stderr.log
stdout_logfile = /var/log/supervisor/tornado-stdout.log 
autostart = true
autorestart = true

[program:tornado_operate_web_cluster]
numprocs = 1
numprocs_start = 1
command = python3 /opt/raspberrybot/interfaces/web/operate.py --port=81%(process_num)02d
process_name = %(program_name)s%(process_num)d
redirect_stderr = true
stderr_logfile = /var/log/supervisor/tornado-operaate-stderr.log
stdout_logfile = /var/log/supervisor/tornado-operaate-stdout.log 
autostart = true
autorestart = true
EOL

sudo supervisord -c /etc/supervisord.conf
sudo service nginx restart

sudo snap refresh core --edge
sudo snap install avahi-client
sudo snap install avahi

sudo ifconfig wlan0 up


# Enabling Camera
echo "start_x=1" >> /boot/firmware/config.txt
echo "gpu_mem=128" >> /boot/firmware/config.txt
sudo modprobe bcm2835-v4l2
v4l2-ctl -D














# 
sudo mkdir raspberrypi
sudo chown -R ubuntu:ubuntu raspberrypi
git clone https://github.com/raspberrypi/userland.git --depth 1
cd raspberrypi
./buildme




echo "OpenCV installation"

# OpenCV Installation
sudo mkdir /opt/opencv
sudo mkdir /opt/opencv_contrib
sudo chown -R ubuntu:ubuntu /opt/opencv
sudo chown -R ubuntu:ubuntu /opt/opencv_contrib
cwd=$(pwd)

sudo apt-get install build-essential cmake pkg-config
sudo apt-get install libjpeg8-dev libtiff4-dev libjasper-dev libpng12-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get install libatlas-base-dev gfortran python3-dev 

git clone https://github.com/Itseez/opencv.git --depth 1 --branch 3.4.0
cd opencv
mkdir build
git checkout 3.4.0

cd $cwd
git clone https://github.com/Itseez/opencv_contrib.git --depth 1 --branch 3.4.0
cd opencv_contrib
git checkout 3.4.0

cd ../opencv/build
cmake

make -j4
sudo make install
sudo ldconfig


