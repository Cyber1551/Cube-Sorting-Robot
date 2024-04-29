#!/usr/bin/env python3
import rospy, time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
import cv2, math, image_geometry, tf, math
import numpy as np
from std_msgs.msg import Int32, Float32MultiArray

class CubeDetector:
    def __init__(self):
        rospy.init_node('cube_detector', anonymous=False, log_level=rospy.DEBUG)

        self.camera_info       = rospy.Subscriber('/world/cam_frontal/camera_info', CameraInfo, self.camera_info_cb)
        self.target_success    = rospy.Subscriber('/target_success', Bool, self.target_success_cb)
        self.image_sub         = rospy.Subscriber('/world/cam_frontal/image_raw', Image, self.image_cb)
        self.color_count_pub   = rospy.Publisher('/color_count', Int32, queue_size=10)
        self.cube_location_pub = rospy.Publisher('/cube_location', Float32MultiArray, queue_size=10)
        self.image_pub         = rospy.Publisher('/detected_image', Image, queue_size=10)
        self.target_point_pub  = rospy.Publisher('/target_point', Float32MultiArray, queue_size=10)
        self.go_to_home        = rospy.Publisher('/go_to_home', Bool, queue_size=10)
        self.bridge            = CvBridge()
        self.img_proc          = image_geometry.PinholeCameraModel()
        self.camera_info       = CameraInfo()
        self.listener          = tf.TransformListener()
        self.cubes_index       = 0
        self.points_index      = 0
        self.target_success    = True
        self.update_coordinates= True
        self.init_time         = time.time()
        self.current_time      = 0
        self.found_colors      = {'red': False, 'green': False, 'blue': False, 
                                'yellow': False, 'purple': False, 'orange': False, }
        self.picking_points    = {'red': [], 'green': [], 'blue': [], 
                                'yellow': [], 'purple': [], 'orange': [], }
        
        ## camera properties 
        self.camera_params     = \
        {
            'Cx': 320.5,
            'Cy': 240.5,
            'focalLength': 277.191356
        }
    
    def camera_info_cb(self, info):
        self.cam_info = info
        self.img_proc.fromCameraInfo(self.cam_info)

    def target_success_cb(self, msg):
        if self.current_time - self.init_time > 10:
            self.target_success = msg.data
            rospy.loginfo(f'{self.target_success}')

    def image_cb(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        # Apply color detection for each color
        image_with_boxes = self.detect_colors(cv_image)
        # Publish color count
        if self.update_coordinates:
            rospy.loginfo_throttle(5, f'self.picking_points==> {self.picking_points}')
        segment_circle = 0
        available_color_list = []
        for color, state in self.found_colors.items():
            if state == True:
                segment_circle += 1
                available_color_list.append(color)
        dropping_points = self.generate_points(self.picking_points, segment_circle, 0.5, available_color_list)
        if self.update_coordinates:
            rospy.loginfo_throttle(2, f'color Info==> {segment_circle, available_color_list}')
            rospy.loginfo_throttle(2, f'dropping_points==> {dropping_points}')
        if available_color_list:
            self.schedule_robot_movement(self.picking_points, dropping_points, available_color_list)
        # Publish image with bounding boxes
        image_msg = self.bridge.cv2_to_imgmsg(image_with_boxes, encoding="bgr8")
        self.image_pub.publish(image_msg)

    def schedule_robot_movement(self, picking_points, dropping_points, available_color_list):
        self.current_time = time.time()
        self.update_coordinates = False
        if self.target_success:
            try:
                target_color = available_color_list[self.cubes_index]
            except:
                for i in range(3):
                    self.go_to_home.publish(True)
                    time.sleep(0.3)
                    return 0
            num_points = len(picking_points[target_color])
            location = Float32MultiArray()
            location.data = picking_points[target_color][self.points_index] + dropping_points[target_color][self.points_index] 
            rospy.logdebug(f'Sending picking location=> {location}')
            self.target_point_pub.publish(location)
            self.points_index += 1
            if self.points_index >= num_points:
                self.points_index = 0
                self.cubes_index += 1 
            self.target_success = False
            self.init_time = time.time()

    def generate_points(self, center_points, segment_circle, min_distance, available_color_list):
        points = {color: [] for color in available_color_list}
        segment_angle = 360 / segment_circle
        for i in range(segment_circle):
            # Calculate the angle range for the current segment
            start_angle = (i * segment_angle) + 10
            end_angle = ((i + 1) * segment_angle) - 10

            # Generate points evenly spaced within the segment
            num_points = len(center_points[available_color_list[i]])
            # num_points = int((segment_angle / 360) * (2 * math.pi * 1) / min_distance) + 1
            divider = num_points - 1 if num_points > 1 else 1
            angle_step = (end_angle - start_angle) / divider

            for j in range(num_points):
                angle = math.radians(start_angle + j * angle_step)
                x = 0.7 * math.cos(angle)
                y = 0.7 * math.sin(angle)
                z = 0.21
                points[available_color_list[i]].append([x, y,z])

        return points
    
    def detect_colors(self, image):
        # Define color ranges for each cube color
        color_ranges = {
            'red': ((0, 100, 100), (10, 255, 255)),
            'green': ((40, 100, 100), (80, 255, 255)),
            'blue': ((100, 100, 100), (140, 255, 255)),
            'yellow': ((20, 100, 100), (30, 255, 255)),
            'purple': ((130, 100, 100), (160, 255, 255)),
            'orange': ((10, 100, 100), (20, 255, 255))
        }
        # Before entering another loop, empty all lists in centers_init
        if self.update_coordinates:
            for color in self.picking_points:
                self.picking_points[color] = []
        image_with_boxes = image.copy()
        for color_name, (lower, upper) in color_ranges.items():
            # Convert color range to numpy array
            lower = np.array(lower, dtype=np.uint8)
            upper = np.array(upper, dtype=np.uint8)
            # Threshold the image to get binary mask
            mask = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), lower, upper)
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # Find center of each contour and draw bounding box
            # Get the center of the image
            cX_image = self.cam_info.width // 2
            cY_image = self.cam_info.height // 2
            for contour in contours:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX_contour = int(M["m10"] / M["m00"])
                    cY_contour = int(M["m01"] / M["m00"])
                    # Calculate offset from the center of the image
                    offset_x = cX_contour - cX_image
                    offset_y = cY_contour - cY_image
                    output_base = self.transform_pose(offset_x, offset_y)
                    if self.update_coordinates:
                        self.picking_points[color_name].append([output_base.point.x, output_base.point.y, output_base.point.z])
                        self.found_colors[color_name] = True
                    # Draw bounding box around the cube
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(image_with_boxes, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    
                else:
                    if self.update_coordinates:
                        self.found_colors[color_name] = False
            cv2.imshow('Cube Sorting', image_with_boxes)
            if cv2.waitKey(1) == ord('q'):
                break
        return image_with_boxes

    def transform_pose(self, offset_x, offset_y):
        y = -offset_x * (0.1/18.3)
        x = -offset_y * (0.1/18.5)
        try:
            (trans,rot) = self.listener.lookupTransform('/base_link', '/camera_frame', rospy.Time(0))
            pointstamp = PointStamped()
            pointstamp.header.frame_id = 'camera_frame'
            pointstamp.header.stamp = rospy.Time(0)
            pointstamp.point.x = 3.0
            pointstamp.point.y = y
            pointstamp.point.z = x
            # Tran: 0.837251 -0.026742 0.721466
            gripper_frame_pose = self.listener.transformPoint('gripper_touch_pnt', pointstamp)
            pointstamp = PointStamped()
            pointstamp.header.frame_id = 'gripper_touch_pnt'
            pointstamp.header.stamp = rospy.Time(0)
            pointstamp.point = gripper_frame_pose.point
            output_gripper = pointstamp
            base_frame_pose = self.listener.transformPoint('base_link', output_gripper)
            pointstamp = PointStamped()
            pointstamp.header.frame_id = 'base_link'
            pointstamp.header.stamp = rospy.Time(0)
            pointstamp.point = base_frame_pose.point
            output_base = pointstamp
            output_base.point.z = 0.2
            # rospy.loginfo(f'BASE==> {output_base.point.x:.2f}, {output_base.point.y:.2f} {output_base.point.z:.2f}')
            return output_base
        except Exception as e:
            rospy.logerr(f'error {e}')
    
if __name__ == '__main__':
    cube_detector = CubeDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")