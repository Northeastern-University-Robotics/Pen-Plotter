#! /usr/bin/env python

import os
import rospy
import flask
import rospy
import threading
from frontend_msgs.msg import Joystickmsgs   #Custom Message
import simplejson 
import json


threading.Thread(target=lambda: rospy.init_node('json_rosmsg_bridge', disable_signals=True)).start()  
pub = rospy.Publisher('/joystick_inputs', Joystickmsgs, queue_size=10)


app = flask.Flask(__name__)

@app.route('/northeastern-robotics.com', methods=['GET'])  #server URL?
def get_json():
    json_data = flask.request.get_json   #get json data and convert it to dictionary 
  
    msg = Joystickmsgs
    msg.left_angle = json_data['left_joystick']['leftAngle']
    msg.left_strength = json_data['left_joystick']['leftStrength']
    msg.right_angle = json_data['right_joystick']['rightAngle']
    msg.right_strength = json_data['right_joystick']['rightStrength']
    msg.x_coord = json_data['x_coord']
    msg.y_coord = json_data['y_coord']
    pub.publish(msg)
    

if __name__ == '__main__':
    app.run(host='69.164.212.94', port=1500)
