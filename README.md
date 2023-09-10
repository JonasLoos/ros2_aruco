# ros2_aruco

ROS2 Wrapper for OpenCV Aruco Marker Tracking

This package depends on a recent version of OpenCV python bindings:

```
pip install "opencv-contrib-python>=4.7"
```


## ROS2 API for the ros2_aruco Node

This node locates Aruco AR markers in images and publishes their ids and poses.

```
ros2 run ros2_aruco aruco_node
```

Subscriptions:
* `/camera/image_raw` (`sensor_msgs.msg.Image`)
* `/camera/camera_info` (`sensor_msgs.msg.CameraInfo`)

Published Topics:
* `/aruco_poses` (`geometry_msgs.msg.PoseArray`) - Poses of all detected markers (suitable for rviz visualization)
* `/aruco_markers` (`ros2_aruco_interfaces.msg.ArucoMarkers`) - Provides an array of all poses along with the corresponding marker ids
* `/aruco_image` (`sensor_msgs.msg.Image`) - analyzed image where the detected markers are marked

Parameters:
* `marker_size` - size of the markers in meters (default .0625)
* `aruco_dictionary_id` - dictionary that was used to generate markers (default `DICT_5X5_250`)
* `image_topic` - image topic to subscribe to (default `/camera/image_raw`)
* `camera_info_topic` - Camera info topic to subscribe to (default `/camera/camera_info`)
* `camera_frame` - Camera optical frame to use (default to the frame id provided by the camera info message.)
* `aruco_parameters` - comma separated list of aruco parameters to set (e.g. `maxBorderBits=2`), see [OpenCV DetectorParameters Documentation](https://docs.opencv.org/4.x/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html) for more information.

Examples:
```bash
ros2 run ros2_aruco aruco_node
```
```bash
ros2 run ros2_aruco aruco_node --ros-args -p marker_size:=0.15 -p aruco_dictionary_id:=DICT_ALVAR_16 -p image_topic:=/camera/camera -p camera_info_topic:=/camera/camera_info -p aruco_parameters:="markerBorderBits=2"
```


## Generating Marker Images

```
ros2 run ros2_aruco aruco_generate_marker
```

Pass the `-h` flag for usage information: 

```
usage: aruco_generate_marker [-h] [--id ID] [--size SIZE] [--dictionary]

Generate a .png image of a specified maker.

optional arguments:
  -h, --help     show this help message and exit
  --id ID        Marker id to generate (default: 1)
  --size SIZE    Side length in pixels (default: 200)
  --dictionary   Dictionary to use. Valid options include: DICT_4X4_100,
                 DICT_4X4_1000, DICT_4X4_250, DICT_4X4_50, DICT_5X5_100,
                 DICT_5X5_1000, DICT_5X5_250, DICT_5X5_50, DICT_6X6_100,
                 DICT_6X6_1000, DICT_6X6_250, DICT_6X6_50, DICT_7X7_100,
                 DICT_7X7_1000, DICT_7X7_250, DICT_7X7_50, DICT_ARUCO_ORIGINAL,
                 (default: DICT_5X5_250)
```
