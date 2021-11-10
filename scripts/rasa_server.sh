#!/bin/bash

BOT_DIR="/home/ouss/Documents/cogrob/cogrob_ws/src/rasa_bot"

cd $BOT_DIR

rasa run -m models --endpoints endpoints.yml --port 5002 --credentials credentials.yml --enable-api
