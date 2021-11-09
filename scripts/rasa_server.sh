#!/bin/bash

BOT_DIR="/home/earendil/Desktop/chatbot_contest"

cd $BOT_DIR

rasa run -m models --endpoints endpoints.yml --port 5002 --credentials credentials.yml --enable-api
