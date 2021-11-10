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
    response = DialogueResponse()
    response.answer = ""
    try:
        for line in r.json():
            text = line['text']
            if text.startswith("__add__"):
                _, quantity, item = text.split(",")
                new_quantity = API.add_item(item, quantity)['quantity']
                response.answer = f"Adding {quantity} {item}, you have now {new_quantity} {item} in your shopping list." 
            elif text.startswith("__remove__"):
                _, quantity, item = text.split(",")
                try:
                    new_quantity = API.remove_item(item, quantity)
                    new_quantity = new_quantity.get('quantity') if new_quantity is not None else 0
                    response.answer = f"Removing {quantity} {item}, you have now {new_quantity} {item} in your shopping list." 
                except ValueError:
                    response.answer = f"There's no item {item} in your shopping list."
            elif text.startswith("__show__"):
                response.answer = "\n" + str(API.show_list())
            else:
                response.answer += text + ' ' if 'text' in line else ''
    except Exception as e:
        print(e)
        response.answer = "Operation failed."

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
