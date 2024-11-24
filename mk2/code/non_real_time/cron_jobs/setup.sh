#TODO - separate package installation into main setup script

apt install python3-gpiozero -y
sudo apt-get -y install python3-rpi.gpio
sudo apt update
sudo apt full-upgrade
#pip install board --break-system-packages
#sudo pip3 install adafruit-circuitpython-neopixel-spi --break-system-packages

mkdir -p /scripts/
mkdir -p /cron-out/

for cronjob_dir in *; 
    do 
        if [ $cronjob_dir = "setup.sh" ]; then
            continue 
        fi
        for file in $cronjob_dir/*; 
            do
                file=$(basename "${file}")
                if [ $file = "cronjob" ]; then
                    crontab -l > temp_cron
                    crontask=$(cat $cronjob_dir/$file)
                    escaped_crontask=$(printf '%s' "$crontask" | sed 's/[][\.*^$()|+?{}\\]/\\&/g')
                    if grep -qF -- "$escaped_crontask" temp_cron; then
                        rm temp_cron
                        continue
                    else    
                        echo "${crontask}" >> temp_cron
                        crontab temp_cron
                    fi
                    rm temp_cron
                else
                    cp $cronjob_dir/$file /scripts/    
                fi
            done;
    done;
