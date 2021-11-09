#!/usr/bin/env python
from rasa_ros.srv import Dialogue, DialogueResponse

import rospy
import requests
from API import API


def handle_service(req):
    input_text = req.input_text   

    # Get answer        
    get_answer_url = 'http://localhost:5002/webhooks/rest/webhook'
    message = {
        "sender": 'bot',
        "message": input_text
    }

    r = requests.post(get_answer_url, json=message)
    print(r.json())
    response = DialogueResponse()
    response.answer = ""
    for line in r.json():
        text = line['text']
        if text.startswith("__add__"):
            texts = text.split(",")
            response.answer = "Adding: " + API.add_item(texts[2], texts[1])
        elif text.startswith("__remove__"):
            texts = text.split(",")
            response.answer = "Removing: " + API.remove_item(texts[2], texts[1])
        elif text.startswith("__show__"):
            response.answer = "Your shopping list: " + API.show_list()
        else:
            response.answer += text + ' ' if 'text' in line else ''

    return response

def main():

    # Server Initialization
    rospy.init_node('dialogue_service')

    s = rospy.Service('dialogue_server',
                        Dialogue, handle_service)

    rospy.logdebug('Dialogue server READY.')
    rospy.spin()


if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException as e:
        pass
