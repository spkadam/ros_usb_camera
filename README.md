# Using USB cameras in ROS
## Camera capture

ROS has quite a few camera capture libraries. I used http://wiki.ros.org/usb_cam, but you could also use http://wiki.ros.org/uvc_camera, which has a better tutorial on the ROS wiki. Install usb_cam using `sudo apt-get install ros-ikinetic-usb-cam` and install libuvc_camera using `sudo apt-get install ros-kinetic-uvc-camera`. The process probably involves using roslaunch on some sort of .launch file to initialize the camera. Here is an example launch file for uvc_camera: https://github.com/spkadam/ros_usb_camera/blob/master/launch/camera.launch. You can run it using by navigating to the directory containing the launch file and running `roslaunch camera.launch device:=/dev/video1 `, where the 1 is variable. Use `ls /dev/video*` to list all connected video devices. If you run into an issue about setting the format, try seeing this thread: http://answers.ros.org/question/38592/uvc_camera-initialization-problem/. If you want to view the output, run `rostopic list` and look for “camera name”/image_raw. This is the raw output of the camera, and you can display it using the command `rosrun image_view image_view image:=/“camera name”/image_raw`.

## Camera calibration

Follow this tutorial: http://wiki.ros.org/camera_calibration. You don’t need to compile camera_calibration; you already have it. There are links to the pages for mono-camera and stereo camera setups. If you use their grid printer out, it is possible to use a smaller calibration grid than they recommend as long as you change the --size calibration flag in the command to match the side length in meters of your grid’s squares.

## Image Processing

The image processing node will also apply the camera calibration to straighten the image. Run ROS_NAMESPACE="camera name" `rosrun image_proc image_proc`. The wiki article is here: http://wiki.ros.org/image_proc?distro=kinetic. If you run `rostopic list` you will see that image_proc also generates a bunch of other nodes, link image_mono, image_rect, and image_rect_color. rect stands for rectified. image_mono is an uncalibrated black and white output, image_rect is a calibrated black and white output, and image_rect_color is a calibrated color output.

## Using the launch files
The `camera.launch` file is used to link the USB camera to ROS; make sure to change the parameters according to your camera and to specify the device argument while launching.
The `follow_line_with_pid.launch` launches the pid_control.py and the follow_line_step_pid.py file. The objective here is to detect the centroid of the green sticky note and give a turtlebot control commands to track the same.

## Results
### original image:
![Original Image](https://github.com/spkadam/ros_usb_camera/blob/master/aa1.png)
### result:
![Result](https://github.com/spkadam/ros_usb_camera/blob/master/final_result.png)

