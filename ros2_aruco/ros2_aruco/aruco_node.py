"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_markers (ros2_aruco_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding
       marker ids.

    /aruco_image (sensor_msgs.msg.Image)
       Image with detected markers drawn on it.

Parameters:
    marker_size - size of the markers in meters (default .0625)
    aruco_dictionary_id - dictionary that was used to generate markers
                          (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /camera/image_raw)
    camera_info_topic - camera info topic to subscribe to
                         (default /camera/camera_info)
    camera_frame - frame id of the camera (default from camera_info_topic)
    aruco_parameters - comma separated list of aruco parameters to set (e.g. `maxBorderBits=2`)

Authors:
Nathan Sprague (until 2020-10-26)
Jonas Loos (2023)
"""

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
from ros2_aruco import transformations

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers



class ArucoNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('aruco_node')

        # Declare and read parameters
        self.declare_parameter("marker_size", .0625)
        self.declare_parameter("aruco_dictionary_id", "DICT_5X5_250")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_frame", None)
        self.declare_parameter("aruco_parameters", None)

        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        dictionary_id_name = self.get_parameter(
            "aruco_dictionary_id").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        aruco_parameters_str = self.get_parameter("aruco_parameters").get_parameter_value().string_value

        self.get_logger().info(f'Scanning topics `{image_topic}` and `{info_topic}` for `{dictionary_id_name}` aruco tags with a marker size of `{self.marker_size}`.')

        # Set up subscriptions
        self.info_sub = self.create_subscription(CameraInfo,
                                                 info_topic,
                                                 self.info_callback,
                                                 qos_profile_sensor_data)

        self.create_subscription(Image, image_topic,
                                 self.image_callback, qos_profile_sensor_data)

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)
        self.image_pub = self.create_publisher(Image, 'aruco_image', 10)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        # setup aruco detector and cv-ros image converter bridge
        params = cv2.aruco.DetectorParameters()
        if aruco_parameters_str != "None":
            # parse aruco parameters
            for x in aruco_parameters_str.split(','):
                # separate into key and value
                assert x.count('=') == 1, f'invalid aruco parameter: {x}'
                key_str, value_str = x.split('=')
                key = key_str.strip()
                # convert value to correct type
                value_type = type(getattr(params, key))
                value = value_type(value_str.strip())
                # set parameter
                params.__setattr__(key, value)
        self.aruco_detector = cv2.aruco.ArucoDetector(self.get_aruco_dict(dictionary_id_name), params)
        self.bridge = CvBridge()

        # init logging msg
        self.log_msg_old = ''

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        if not self.camera_frame:
            self.camera_frame = info_msg.header.frame_id
            if not self.camera_frame:
                self.get_logger().warn(f'`frame_id` from `{self.info_topic}` is empty. You can set it manually using the `camera_frame` argument.')

        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):

        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received! (Continuing with default params)")
            self.intrinsic_mat = np.array([
                [640,   0, 320],  # these numbers assume a 640x480 px image
                [  0, 480, 240],
                [  0,   0,   1],
            ])
            self.distortion = np.zeros((5,1))  # assuming a perfect lens

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='mono8')
        aruco_image = cv2.cvtColor(cv_image,cv2.COLOR_GRAY2RGB)
        markers = ArucoMarkers()
        pose_array = PoseArray()
        markers.header.frame_id = self.camera_frame
        pose_array.header.frame_id = self.camera_frame

        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        corners, marker_ids, rejected = self.aruco_detector.detectMarkers(cv_image)
        if marker_ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.intrinsic_mat, self.distortion)
            for i, marker_id in enumerate(marker_ids):
                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])

                # get corner positions
                (topLeft, topRight, bottomRight, bottomLeft) = corners[i].reshape((4, 2))
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                # draw lines around the marker and display the marker id
                cv2.line(aruco_image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(aruco_image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(aruco_image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(aruco_image, bottomLeft, topLeft, (0, 255, 0), 2)                    
                cv2.putText(aruco_image, str(marker_id),(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)

            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)
        
        # log found markers (only if things change)
        log_msg = f'found markers: {None if marker_ids is None else [j for i in marker_ids for j in i]}'
        if log_msg != self.log_msg_old:
            self.get_logger().info(log_msg)
            self.log_msg_old = log_msg

        # publish aruco image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(aruco_image))

    def get_aruco_dict(self, dictionary_id_name):
        '''Get an aruco dictionary from a dictionary id.'''

        if dictionary_id_name == "DICT_ALVAR_16":
            # markers 0-15 of https://wiki.ros.org/ar_track_alvar
            alvar16_bytelists = [
                cv2.aruco.Dictionary_getByteListFromBits(
                    np.array([(m >> j) & 1 for j in range(24, -1, -1)], dtype=np.uint8).reshape((5,5)))
                for m in [
                    0b1101111011101011111111111,
                    0b1101111011101010011011101,
                    0b1101111011101011011010110,
                    0b1101111011101010111110100,
                    0b1101111011101010111001110,
                    0b1101111011101011011101100,
                    0b1101111011101010011100111,
                    0b1101111011101011111000101,
                    0b1101111011101010010111110,
                    0b1101111011101011110011100,
                    0b1101111011101010110010111,
                    0b1101111011101011010110101,
                    0b1101111011101011010001111,
                    0b1101111011101010110101101,
                    0b1101111011101011110100110,
                    0b1101111011101010010000100
            ]]

            return cv2.aruco.Dictionary(np.concatenate(alvar16_bytelists), 5, 2)

        # default -> use predefined dictionary (e.g. DICT_5X5_250)
        # Make sure we have a valid dictionary id
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error("Bad aruco_dictionary_id: {}".format(dictionary_id_name))
            custom_options = ['DICT_ALVAR_16']
            options = "\n".join(["    " + s for s in dir(cv2.aruco) + custom_options if s.startswith("DICT")])
            self.get_logger().error("Valid options:\n{}".format(options))
        return cv2.aruco.getPredefinedDictionary(dictionary_id)


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
