#!/bin/bash
### CRONJOB SETUP ###
echo "Beginning cronjob setup"
#copy relative path to setup.sh script into a file for use by cronjob scripts
SETUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "export HEX_NON_REALTIME=\"$SETUP_DIR\"" | sudo tee /etc/profile.d/hexapod_env.sh > /dev/null
sudo chmod +x /etc/profile.d/hexapod_env.sh

cd cron_jobs 

mkdir -p /scripts/
mkdir -p /cron-out/

touch temp_cron
crontab -l 2>/dev/null > temp_cron 

for cronjob_dir in *; do 
        [ -d "$cronjob_dir" ] || continue #skip if not a directory
        for file in $cronjob_dir/*; do
                file=$(basename "${file}")
                if [ $file = "cronjob" ]; then
                    # only add cronjob if it does not already exist in the jobs list
                    crontask=$(cat $cronjob_dir/$file)
                    escaped_crontask=$(printf '%s' "$crontask" | sed 's/[][\.*^$()|+?{}\\]/\\&/g')
                    if grep -qF -- "$escaped_crontask" temp_cron; then
                        continue
                    else    
                        echo "${crontask}" >> temp_cron
                    fi
                else
                    cp $cronjob_dir/$file /scripts/ 
                    chmod +x "/scripts/$file"   
                fi
            done;
    done;

crontab temp_cron
rm temp_cron

cd ..
echo "Completed cronjob setup" 
