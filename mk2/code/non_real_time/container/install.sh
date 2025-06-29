#!/bin/bash
export XAUTH_FILE=$(ls /run/user/1002/.mutter-Xwaylandauth.* | head -n 1)
./build.sh
docker compose -f docker-compose-prod.yml up -d