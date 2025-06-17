import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from mecharm_interfaces.srv import CapturePose
from pose_estimation_pkg.libs.main import MainApp
from pose_estimation_pkg.server import Hyco_Integrator
import os
import time
import cv2
import asyncio
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

class CamPose(Node):
    def __init__(self):
        super().__init__('camera_pose_srv')

        self.reentrant_callback = ReentrantCallbackGroup()
        self.mutual_callback = MutuallyExclusiveCallbackGroup()

        self.color_image = None
        self.depth_image = None
        self.intrinsics = None
        self.image_dir = "/root/images/"
        os.makedirs(self.image_dir, exist_ok=True)
        self.classes = {0: "white_connector", 1: "blue_connector"}
        self.bridge = CvBridge()
        self.display = False
        self.crop_size = (640, 640)
        self.pose_3d = MainApp(object_name="white_connector", display=self.display)
        self.hyco = Hyco_Integrator(option="ws", image_dir=self.image_dir)

        # Async event loop in background thread
        self.async_loop = asyncio.new_event_loop()
        threading.Thread(target=self._start_loop, daemon=True).start()

        self.declare_params()
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 1)

        self.capture_pose_service = self.create_service(
            CapturePose, 'capture_pose', self.capture_pose_callback, callback_group=self.mutual_callback)

    def _start_loop(self):
        asyncio.set_event_loop(self.async_loop)
        self.async_loop.run_forever()

    def declare_params(self):
        self.declare_parameter('save_image', False)
        self.declare_parameter('class_id', 0)
        self.declare_parameter('6dof', False)
        self.declare_parameter("white_neg_x_pick_offset",0.30)
        self.declare_parameter("white_pos_x_pick_offset",0.10)
        self.declare_parameter("blue_neg_x_pick_offset",0.20)
        self.declare_parameter("blue_pos_x_pick_offset",0.0)
        self.declare_parameter("crop_inference", True)


    def capture_pose_callback(self, request, response):
        pose_est_start_time = time.time()
        self.get_logger().info("Starting capture pose callback...")

        self.color_image = None
        self.depth_image = None

        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_rect_raw', self.image_callback, 1, callback_group=self.reentrant_callback)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.depth_callback, 1, callback_group=self.reentrant_callback)

        timeout = 3.0
        start_time = time.time()

        while self.color_image is None or self.depth_image is None:
            if time.time() - start_time > timeout:
                self.get_logger().warn("Timed out waiting for images.")
                response.pose.has_detection = False
                self.cleanup_subscriptions()
                return response
        
        
        time_for_capturing_img = time.time()-start_time
        self.get_logger().info(f"time taken for capturing img:{time_for_capturing_img}")
        
        crop_inference = self.get_parameter('crop_inference').get_parameter_value().bool_value
        self.write_image(crop_inference)

        save_image = self.get_parameter('save_image').get_parameter_value().bool_value
        dof_6d = self.get_parameter('6dof').get_parameter_value().bool_value
        class_id = self.get_parameter('class_id').get_parameter_value().integer_value
        object_name = self.classes[class_id]
        white_neg_x_pick_offset = self.get_parameter('white_neg_x_pick_offset').get_parameter_value().double_value
        white_pos_x_pick_offset = self.get_parameter('white_pos_x_pick_offset').get_parameter_value().double_value

        blue_neg_x_pick_offset = self.get_parameter('blue_neg_x_pick_offset').get_parameter_value().double_value
        blue_pos_x_pick_offset = self.get_parameter('blue_pos_x_pick_offset').get_parameter_value().double_value

        inference_start_time = time.time()
        future = asyncio.run_coroutine_threadsafe(self.hyco.get_bboxes(), self.async_loop)
        try:
            bboxes = future.result(timeout=5.0)
        except Exception as e:
            self.get_logger().error(f"Async call failed: {e}")
            response.pose.has_detection = False
            self.cleanup_subscriptions()
            return response

        time_for_inference = time.time()-inference_start_time
        self.get_logger().info(f"time taken for inference:{time_for_inference}")
        if crop_inference:
            bboxes = self.shift_bbox(bboxes)
        self.get_logger().info(f"BBoxes: {bboxes}")        

        pose_estimation_time = time.time()
        camera_pose, rgb_img = self.pose_3d.cam_infer(
            self.color_image, self.depth_image, bboxes, object_name, dof_6d, save_image,
            white_neg_x_pick_offset= white_neg_x_pick_offset,white_pos_x_pick_offset= white_pos_x_pick_offset,
            blue_neg_x_pick_offset= blue_neg_x_pick_offset,blue_pos_x_pick_offset= blue_pos_x_pick_offset)
        
        time_for_pose_estimation = time.time()-pose_estimation_time
        self.get_logger().info(f"time taken for pose_estimation:{time_for_pose_estimation}")


        if camera_pose is not None and (not dof_6d):
            response.pose.pose.x, response.pose.pose.y, response.pose.pose.z = camera_pose[:3]
            response.pose.has_detection = self.pose_3d.has_detected
        elif camera_pose is not None and dof_6d:
            response.pose.pose.x, response.pose.pose.y, response.pose.pose.z = camera_pose[:3]
            response.pose.pose.rx, response.pose.pose.ry, response.pose.pose.rz = camera_pose[3:]
            response.pose.has_detection = self.pose_3d.has_detected
        else:
            response.pose.has_detection = False

        self.cleanup_subscriptions()
        total_time = time.time() - pose_est_start_time
        self.get_logger().info(f"Total Time taken for pose estimation: {total_time}")
        return response

    def cleanup_subscriptions(self):
        if hasattr(self, 'image_sub'):
            self.destroy_subscription(self.image_sub)
        if hasattr(self, 'depth_sub'):
            self.destroy_subscription(self.depth_sub)

    def camera_info_callback(self, msg):
        self.intrinsics = {
            'fx': msg.k[0],
            'fy': msg.k[4],
            'ppx': msg.k[2],
            'ppy': msg.k[5]
        }

    def image_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

    def write_image(self, crop_inference):
        image = self.color_image.copy()
        if crop_inference:
            image = self.crop_image(image)
        image_path = os.path.join(self.image_dir, "1.jpg")
        cv2.imwrite(image_path, image)
        self.get_logger().info(f"Image written to {image_path}")


    def crop_image(self, image):
        H,W, _ = image.shape
        self.x_shift = int((W - self.crop_size[0]) / 2) # 320
        self.y_shift = int(H - self.crop_size[0]) # 80
        image = image[ : , self.x_shift: -self.x_shift]
        image = image[self.y_shift : , : ]
        print(image.shape)
        return image


    def shift_bbox(self, bboxes):
        for bbox in bboxes:
            bbox[2] += self.x_shift
            bbox[4] += self.x_shift
            bbox[3] += self.y_shift
            bbox[5] += self.y_shift
        return bboxes


def main(args=None):
    rclpy.init(args=args)
    node = CamPose()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
