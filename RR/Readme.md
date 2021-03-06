# Robot Raconteur Survey
Robot Raconteur is an object oriented Service-Client middleware. An RR service generally runs with a hardware (e.g. sensors,actuators) attached to a robot/computer to have direct communication between them. An RR client can receive messages that are sent from services and can call object functions in the service to command the robot. In this survey, we’ll first demonstrate how RR works with virtual duckiebot keyboard control.
In this example, we'll go through how RR webcam streaming works.
# Service Definition:
Each RR services have a service definition file, which includes properties, functions and others that are exposed to clients. In other word, the definition inside the `.robdef` file are the ones the client has access to. 
```
service experimental.createwebcam2

stdver 0.9

struct WebcamImage
    field int32 width
    field int32 height
    field int32 step
    field uint8[] data
end

object Webcam
    property WebcamImage image
end
```
First there's a struct for `WebcamImage`, consisting of `width, height, step and data`. The first three elements are the metadata of an image, and the `data` contains the actual pixel information as an 1-D array.
The object that being exposed to the network has the `object` keyword, and in this case it's the `Webcam`. The object in the example only has one member, a property of `WebcamImage` with name `image`.

# RR Service:
On the pi side, inside `pihat_trials/RR`, there's `webcam_service.py` python script for RR webcam service. This example continuously capturing images from a webcam.
At very top, the RobotRaconteur library is imported:
```
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
```
The class `Webcam_impl(object)` is the python object that will be used for RR, but not all of its members are members in service definition. The only member in the service definition, `image`, is defined upon initializaiton:
```
self.image=RRN.NewStructure("experimental.createwebcam2.WebcamImage")
```
Other parts in the class are mostly for continuously streaming of the webcam. Inside `CaptureFrame()` function, the image data is read inside the while loop and formed into the struct defined in service definition:
```
ret, frame=self._capture.read()
if not ret:
    raise Exception("Could not read from webcam")
self.image.width=frame.shape[1]
self.image.height=frame.shape[0]
self.image.step=frame.shape[1]*3
self.image.data=frame.reshape(frame.size, order='C')
```
Now take a look at the main function, the node name and port number is specified in the following line:
```
with RR.ServerNodeSetup("Webcam_Service",2355) as node_setup:
```
In this case, the node name is "Webcam_Service" and the port is 2355.
The service definition file is registered right after that:
```
RRN.RegisterServiceTypeFromFile("experimental.createwebcam2")
```
Then the object is created and the service is registered:
```
c1=Webcam_impl()
RRN.RegisterService("Webcam","experimental.createwebcam2.Webcam",c1)
```
The service is registered with name "Webcam", type of "experimental.createwebcam2.Webcam", actual object of c1

The `input()` function at last holds the script from exiting. To run this script, simply do `$ python webcam_service.py`.

# RR Client:
On your laptop side, there's script `streaming_client.py` under `pihat_trials/RR`. The RR client library is imported at the top:
```
from RobotRaconteur.Client import *
```
The function `WebcamImageToMat(image)` converts RR image property to openCV object. Inside main function, the `url` is the string containing the IP address of the service, the port the service is on and the service name. The default url in script is
```
url='rr+tcp://raspberrypi:2355/?service=Webcam'
```
But it's possible to use command line options to provide the url:
```
$ python streaming.py rr+tcp://raspberrypi:2355/?service=Webcam
```

,where `raspberrypi` is the hostname of the RaspberryPi.
```
cam=RRN.ConnectService(url)
```
`cam` is the object returned by `ConnectService`, and it should contain the member defined in the service definition, which is the `image` property. And the image is accessed and converted to openCV object by
```
WebcamImageToMat(cam.image)
```
To run this script, simply do `$ python streaming.py`.

# Task:
## 1
Given the basic examples commanding the sensor hat, write a simple RR service, with the ability to control each LED color.
On RaspberryPi, copy the `hat.py` under `pihat_trials/Examples` to `pihat_trials/RR` folder, and modify it to a service according to the hints.

Test it out with a client.
## 2
Given the basic examples taking in images and output images with bounding boxes and results, write an RR service that takes in webcam images from webcam service and output detection results including the image. 
You're provided with the service definition `experimental.detection.robdef`, take a look and on computer, copy `detection.py` under `pihat_trials/Examples` to `pihat_trials/RR` folder, modify it to a detection service according to the hints. Try print out the test result to terminal.

## 3
Create a final client, connect that client to the service from task 1 and task 2, lighting up LED according to the duckie's location in the image frame.
