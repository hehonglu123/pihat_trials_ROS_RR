#TODO ALL: import ROS/RR libraries ROS message types

#import sense hat module
from sense_hat import SenseHat
import time


#TODO RR: create a python object, containing parameters/functions defined in service definition
sense=SenseHat()
pix_rgb=[255,255,255]
for i in range(8):
    for j in range(8):
        #light up each pixel individually
        sense.set_pixel(i,j,pix_rgb)
        time.sleep(0.2)
        sense.clear()

#TODO ROS: initialize rosnode, subscriber
#TODO RR: register service type from file, initialize object defined above, register service




#TODO ALL: hold script running