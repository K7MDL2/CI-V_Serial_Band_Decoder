
#!/bin/bash

# This sets the pin for a DHT11 temp humidity sensor
# This script updates /boot/firmware/config/txt to let the OS talk to the sensor
# after reboot you can use this command to see if the device is connected and working OK.
#   cat /sys/bus/iio/devices/iio:device0/in_temp_input 
#
DHT11_PIN="15"

echo "Do not run this script as sudo!"

#assuming the path this sxript resides in also has the install target files
# Get current workign path/folder
currUser=$USER
currPath=$PWD          # to assign current path to a variable
#currPath=${result:-/}        # to correct for the case where PWD is / (root)
printf 'Current user is %s\n' "$currUser"
printf 'Install source folder is %s\n' "$currPath"
printf 'Log Files will be located in %s\n' "/home/$currUser"
SERVICE_PATH=/home/$currUser/.config/systemd/user
mkdir -p $SERVICE_PATH
printf 'systemd config file is located in %s\n' "$SERVICE_PATH"

ifconfig > "/home/$currUser/if_dump"

sudo apt update
sudo apt upgrade -y
printf 'Python version is %s'
python --version
sudo apt install tcpdump
sudo apt install python3-numpy

#  search model info for string for '5' in 'Raspberry Pi 5 ....'
CPUMODEL=$(awk '{print $3}' /proc/device-tree/model)
echo "CPU Model is " $CPUMODEL
if [ $CPUMODEL = 5 ]; then
    sudo apt install python3-rpi-lgpio
else  # if not a Pi 5 then
    sudo apt install python3-rpi.gpio
fi

echo "Updating /boot/firmware/config.txt for DHT11 temp sensor GPIO pin"
CONFIG="/boot/firmware/config.txt"
sudo sed -i "/dtoverlay=dht/d" $CONFIG
echo "dtoverlay=dht11,gpiopin=$DHT11_PIN" | sudo tee --append $CONFIG
echo "DHT11 GPIO pin = $DHT11_PIN"

SERVICE="Decoder.service"

echo "Stopping Decoder service if already installed"
systemctl --user stop $SERVICE
systemctl --user disable $SERVICE

echo "Removing old service file"
rm $SERVICE_PATH/$SERVICE

echo "Resetting systemd"
systemctl --user daemon-reload
systemctl --user reset-failed

echo "Copying files and configuring the new systemd service ..."
sudo cp $PWD/CI-V_Serial.py /usr/local/bin
sudo chmod +x /usr/local/bin/CI-V_Serial.py

sudo cp $PWD/view_log /usr/local/bin
sudo chmod +x /usr/local/bin/view_log

sudo cp $PWD/chk_dht11 /usr/local/bin
sudo chmod +x /usr/local/bin/chk_dht11

sudo cp $PWD/stop_Decoder /usr/local/bin
sudo chmod +x /usr/local/bin/stop_Decoder

sudo cp $PWD/start_Decoder /usr/local/bin
sudo chmod +x /usr/local/bin/start_Decoder

cp $PWD/Decoder.config /home/$USER

sudo loginctl enable-linger $USER
echo "Installing new Decoder service"
cp $currPath/$SERVICE $SERVICE_PATH
chmod 664 $SERVICE_PATH/$SERVICE
sed -i "/StandardOutput/d" $SERVICE_PATH/$SERVICE
echo "StandardOutput=append:/home/$currUser/Decoder.log" | sudo tee --append $SERVICE_PATH/$SERVICE
sed -i "/StandardError/d" $SERVICE_PATH/$SERVICE
echo "StandardError=file:/home/$currUser/Decoder.err" | sudo tee --append $SERVICE_PATH/$SERVICE

echo "Restarting systemctl daemon ..."
systemctl --user enable Decoder.service
systemctl --user daemon-reload
systemctl --user reset-failed

# pause here and ask for reboot
while true; do
read -p "Reboot to finish.  Do you want to proceed? y/n " yn
case $yn in
	[yY] ) echo ok, we will proceed;
		break;;
	[nN] ) echo exiting...;
		exit;;
	* ) echo invalid response;;
esac
done

sudo reboot

