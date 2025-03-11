#!/bin/bash

##########################################################################################
# This entrypoint can be specified to just have the container running indefinitely in idle
##########################################################################################

# Find the mutter-Xwaylandauth file and symlink it to the correct location
AUTH_FILE=$(find /run/user/1002/ -name 'mutter-Xwaylandauth*' | head -n 1)

if [ -n "$AUTH_FILE" ]; then
  ln -sf "$AUTH_FILE" /root/.Xauthority
else
  echo "Xwayland auth file not found!"
  exit 1  # Exit if the file is not found
fi

# Keep the container running indefinitely in idle
while :
do
    sleep 1
done
