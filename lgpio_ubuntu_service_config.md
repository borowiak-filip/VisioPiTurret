## lgpio Ubuntu Server 22.04 Setup
- Build from source as on https://abyz.me.uk/rpi/pigpio/download.html
- Add a service to `sudo nano /lib/systemd/system/pigpiod.service`
- Service:
  Example
  ```ini
  [Unit]
  Description=Daemon required to control GPIO pins via pigpio
  [Service]
  ExecStart=/usr/local/bin/pigpiod
  ExecStop=/bin/systemctl kill -s SIGKILL pigpiod
  Type=forking
  [Install]
  WantedBy=multi-user.target
  ```

reboot and run `sudo systemctl status pigpiod` to verify

to activate the service run `sudo systemctl start pigpiod`
to stop the service run `sudo systemctl stop pigpiod`
to kill the service run `sudo systemctl kill -s SIGKILL pigpiod` (this is what the service should execute itself when stopping per ExecStop)
