#!/bin/bash
export XAUTH_FILE=$(ls /run/user/1002/.mutter-Xwaylandauth.* | head -n 1)
docker compose -f docker-compose-dev.yml up -d
