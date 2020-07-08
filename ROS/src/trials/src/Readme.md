# ROS Survey
The structure of ROS is a little different from Robot Raconteur. It has the Publisher-Subscriber relationship between different nodes.

# Setup
Unlike RobotRaconteur, ROS requires a special workspace to build the content. Therefore, type in following commands to build your workspace:
```
cd ~/pihat_trials/ROS
catkin_make
```
It should finish without errors, and generating `/build` and `/devel` folders under the same directory. However, to make sure your code knows what you've built, we need to source it:
```
$ echo 'source ~/pihat_trials/ROS/devel/setup.bash' >> ~/.bashrc 
```
Finish above steps on both pi and laptop. If errors like something not found or not built, try `source ~/pihat_trials/ROS/devel/setup.bash`.

# Message Types
Similar to RobotRaconteur service definition, for ROS there're [message types](http://wiki.ros.org/Messages) and [service types](http://wiki.ros.org/Services). In this example we only uses publisher/subscriber, so just messages types.
When building ROS, many message and service types are built together, which you can look up online: http://wiki.ros.org/common_msgs. But we have our own one under `pihat_trials/ROS/src/trials/msg/dresult.msg`
```
string[] names
float32[] centroids
```
This bascially shows the message contains a list of string `names` and a list of float32 `centroids`, with unset length. Most of the time the common messages meet our need. The messages are built through `catkin_make`, and if there's error saying message not built, try `$ source ~/pihat_trials/ROS/devel/setup.bash` again.

# ROS Publisher
On Pi side, under `~/pihat_trials/ROS/src/trials/src/` there is a python script called `cam_pub.py`. At the very top, we include ROS library and message types:
``` 
import rospy
from sensor_msgs.msg import Image
import ros_numpy
```
The `Webcam_impl()` class is a webcam class, which contains camera metadata and a `CaptureFrame()` function. Then take a look at `main`:
```
pub = rospy.Publisher('image_raw', Image, queue_size=0) 
rospy.init_node('picam', anonymous=True) 
```
Here ROS node is initialized with a publisher, published to topic `image_raw` of type `Image` ([sensor_msgs/Image.msg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html))
```
while not rospy.is_shutdown(): 
```
This while loop holds the sciprt from exiting until ROS is shutdown, and inside the loop:
```
frame=picam.CaptureFrame() 
pub.publish(ros_numpy.msgify(Image,frame,encoding="bgr8")) 
```
The image is captured and convert to ROS Image type, and finally published to the topic by the publisher.
To run this script, open a new terminal and run `roscore`. After that, you can run this script by `python cam_pub.py`.
For every ROS communication, there needs to be one and only one roscore running. To check if the images are successfully published or not, open up a new terminal and type in `rostopic echo image_raw`.
This way the terminal shall display the image data.

# ROS Subscriber
On your computer side, under `~/pihat_trials/ROS/src/trials/src/` there is a python script called `cam_sub.py`. Similarly, we include ROS library and message types at the top.
Unlike a publisher, a subscriber subscribe to the topic, and trigger the `callback()` function. Inside main, 
```
rospy.init_node('stream_node', anonymous=True)
sub = rospy.Subscriber("image_raw",Image,callback)
rospy.spin()
```
ROS node is intialized, and a subscriber `sub` is set up to subscribe to ROS topic `image_raw`, with `Image` type, triggering `callback()` function. `rospy.spin()` keeps this script running until user shutdown. Now let's take a look at the `callback()` function.
`def callback(data)` means this function takes in an argument of `data`, which should be the message type specified in the subscriber setup (`Image`). 
```
cv_image=ros_numpy.numpify(data)
```
This line basically converts the `Image` type data into an openCV image object, so that it could be displayed out on screen.

# ROS Communication Setup
In order to run this script on your laptop, it's necessary to set up ROS communication. First, open up a few terminals and make your laptop a ROS master:
```
$ export ROS_MASTER_URI=http://<computer's IP>:11311
$ export ROS_IP=<computer's IP>
```
Do this for every single terminal on the laptop. Then start a roscore in one of those terminals

On pi side, open up a few terminals and make your laptop a ROS master:
```
$ export ROS_MASTER_URI=http://<computer's IP>:11311
$ export ROS_IP=<pi's IP>
```
Do this for every single terminal on the pi.

After that, the ROS communication between the pi and the laptop is setup. Now we can try out the webcam streaming. On pi side, run `$ python webcam_pub.py`, and on laptop side, run `$ python webcam_sub.py`. You should be able to see a pop window containing the live images from the camera.


# Task:
## 1

Given the basic examples commanding the sensor hat, write a simple ROS script subscribing to a [Uint16MultiArray.msg](http://docs.ros.org/melodic/api/std_msgs/html/msg/UInt16MultiArray.html), with the ability to control each LED color, (first 2 element for location, last 3 for RGB).
On RaspberryPi, copy the `hat.py` under `pihat_trials/Examples` folder to `pihat_trials/ROS/src/trials/src/`, and modify this script to a subscriber according to the hints.

Test it out with a publisher after finished!
## 2
Given the basic examples taking in images and output images with bounding boxes and results, write an ROS node that subscribes webcam images from webcam publisher and publish detection results (message type mentioned in first section) and image. 
On computer, copy the `detection.py` under `pihat_trials/Examples` folder to `pihat_trials/ROS/src/trials/src/`, and modify this script according to the hints. Try print out detection result to the terminal.

## 3
Test it out with a subscriber showing the image with bounding box, and publish the result to the subscriber from task 1, lighting up LED according to the duckie's location in the image frame.
