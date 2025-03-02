### CRONJOB SETUP ###
echo "Beginning cronjob setup"
cd cron_jobs 

mkdir -p /scripts/
mkdir -p /cron-out/

for cronjob_dir in *; 
    do 
        for file in $cronjob_dir/*; 
            do
                file=$(basename "${file}")
                if [ $file = "cronjob" ]; then
                    crontab -l > temp_cron
                    # only add cronjob if it does not already exist in the jobs list
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

cd ..
echo "Completed cronjob setup" 
