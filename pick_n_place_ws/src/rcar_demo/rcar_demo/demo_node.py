import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from mecharm_interfaces.srv import CapturePose
from rclpy.action import ActionClient
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from mecharm_interfaces.srv import ArmMoveToPick,ArmLinearControl,ArmGripperControl,SearchObject,TransformPose
from mecharm_interfaces.srv import GoToGoal,SetInitialPose
from geometry_msgs.msg import PoseStamped
import time
from datetime import datetime

class TaskNode(Node):
    def __init__(self):
        super().__init__('rcar_demo_node')
        self.YELLOW = "\033[33m"
        self.RESET = "\033[0m"
        self.capture_pose_client = self.create_client(CapturePose, 'capture_pose')
        while not self.capture_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the capture_pose service...')
        self.get_logger().info('capture_pose service is available.')
        self.arm_set_pose_client = self.create_client(ArmMoveToPick, 'set_pose') 
        while not self.arm_set_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the set_pose service...')
        self.get_logger().info('set_pose service is available.')
        self.arm_get_pose_client = self.create_client(TransformPose, 'get_pose') 
        while not self.arm_get_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the get_pose service...')
        self.get_logger().info('get_pose service is available.')
        self.arm_set_pick_pose_client = self.create_client(ArmMoveToPick, 'set_pick_pose')
        # Wait until the service is available
        while not self.arm_set_pick_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the set_pose service...')
        self.arm_linear_control_client = self.create_client(ArmLinearControl, 'arm_linear_control')
        # Wait until the service is available
        while not self.arm_linear_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the arm_linear_control service...')
        self.gripper_client = self.create_client(ArmGripperControl, 'arm_gripper_control')
       # Wait until the service is available
        while not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the arm_gripper_control service...')
        self.arm_search_object_client = self.create_client(SearchObject, 'arm_search_object')
        # Wait until the service is available
        while not self.arm_search_object_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the set_pose service...')
        self.declare_parameter('start_demo', False)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.triggered = False
        self.goal=0.0
        self.pose = None
        self.transformed_pose = None
        self.initial_pose_not_set=True
        self.initial_delay=True
        self.gripper_state=False
        self.pick_completed=False
        self.drop_completed=False
        self.home_reached=True
        self.pose_estimated=False
        self.arm_home_pose_set=False



    def timer_callback(self):        
        start_demo = self.get_parameter('start_demo').get_parameter_value().bool_value
        if start_demo and not self.triggered:
            self.get_logger().info('Trigger received! Executing tasks sequentially...')
            self.execute_tasks()
            self.triggered = True
            self.get_logger().info('Tasks initiated.')
        elif not start_demo:
            self.triggered = False


    def execute_tasks(self,):  
        self.get_logger().info('Going to home pose...')
        start_time = time.time()
        if(not self.arm_home_pose_set):
            self.arm_home_pose_set=True
            self.get_logger().info('Going to home pose...')
            arm_move_request = ArmMoveToPick.Request()
            arm_move_request.base_pose.x = 9.49
            arm_move_request.base_pose.y =  106.08
            arm_move_request.base_pose.z =  -132.09
            arm_move_request.base_pose.rx =  24.78
            arm_move_request.base_pose.ry = -10.98
            arm_move_request.base_pose.rz = 44.91
            arm_move_future = self.arm_set_pose_client.call_async(arm_move_request)
            arm_move_future.add_done_callback(self.open_gripper)
        else:
            self.capture_pose()
        time_diff = time.time() - start_time
        self.get_logger().info(f"Go to home pose, time: {time_diff}")
        

    def open_gripper(self,future):
        start_time = time.time()
        self.get_logger().info("activating gripper")
        gripper_request=ArmGripperControl.Request()
        gripper_request.gripper_state=True
        gripper_future=self.gripper_client.call_async(gripper_request)
        self.drop_completed=True
        gripper_future.add_done_callback(self.capture_pose)       
        time_diff = time.time() - start_time
        self.get_logger().info(f"Open Gripper, time: {time_diff}") 

    
    def init_pose_cb(self, future):
        self.get_logger().info('Task 1 Completed: Inital Pose Set..')
        self.get_logger().info("Task 2: Activating gripper...")
        gripper_request=ArmGripperControl.Request()
        gripper_request.gripper_state=True
        gripper_future=self.gripper_client.call_async(gripper_request)
        gripper_future.add_done_callback(self.start_nav_cb)
    

    def start_nav_cb(self,future):
        self.get_logger().info("Task 2 Completed: Gripper Activated.")
        if(self.initial_pose_not_set):
            time.sleep(2)
        self.navigate_to_goal(future,pose_x=1.0,pose_y=0.0,pose_yaw=0.0)


    def nav2_callback(self, future):
        try:
            response = future.result()
            if response.success:  # Assuming the service response has a `success` flag
                self.get_logger().info('Navigation successful! Proceeding to next task.')
                self.capture_pose()
            else:
                self.get_logger().error('Navigation failed!')
        except Exception as e:
            self.get_logger().error(f"Failed to reach goal: {e}")


    def capture_pose(self,future):
        start_time = time.time()
        self.get_logger().info('Task 1: Calling CapturePose service...')
        request = CapturePose.Request()
        capture_future = self.capture_pose_client.call_async(request)
        capture_future.add_done_callback(self.capture_pose_response)
        time_diff = time.time() - start_time
        self.get_logger().info(f"Capture pose Service, time: {time_diff}")


    def capture_pose_response(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Task 1: CapturePose service call failed: {e}")
            return
        self.get_logger().info(f"has detcetion {response.pose.has_detection}")
        if response.pose.has_detection:
            self.pose_estimated=True
            self.pose = response.pose.pose
            self.get_logger().info(f"Task 1: Detected Pose: x={self.pose.x}, y={self.pose.y}, z={self.pose.z}, has_detection={response.pose.has_detection}")
        else:
            self.get_logger().info("Task 1: No object detected.")
            self.set_parameters([Parameter('start_demo', Parameter.Type.BOOL, False)])
            self.triggered = False
            return
        self.get_logger().info('Task : Transform Camera Pose to Base Pose.')
        convert_coords_request = TransformPose.Request()
        convert_coords_request.base_pose.x = self.pose.x
        convert_coords_request.base_pose.y = self.pose.y
        convert_coords_request.base_pose.z = self.pose.z
        convert_coords_request.base_pose.rx = self.pose.rx
        convert_coords_request.base_pose.ry = self.pose.ry
        convert_coords_request.base_pose.rz = self.pose.rz
        start_time = time.time()
        convert_coords_future = self.arm_get_pose_client.call_async(convert_coords_request)
        convert_coords_future.add_done_callback(self.move_arm)
        time_diff = time.time() - start_time
        self.get_logger().info(f"Transform pose to base pose, time: {time_diff}")

    

    def move_arm(self, future):
        start_time = time.time()
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Task 1: TransformPose service call failed: {e}")
            return

        self.transformed_pose = response.transformed_pose
        arm_move_request = ArmMoveToPick.Request()
        arm_move_request.base_pose.x = self.transformed_pose[0] - 30.0
        arm_move_request.base_pose.y = self.transformed_pose[1]
        arm_move_request.base_pose.z = self.transformed_pose[2]
        arm_move_request.base_pose.rx = self.transformed_pose[3]
        arm_move_request.base_pose.ry = self.transformed_pose[4]
        arm_move_request.base_pose.rz = self.transformed_pose[5]
        arm_move_future = self.arm_set_pick_pose_client.call_async(arm_move_request)
        arm_move_future.add_done_callback(self.move_arm_in_x)
        time_diff = time.time() - start_time
        self.get_logger().info(f"Move to pick location, time: {time_diff}")
    
    def move_arm_in_x(self,future):
        self.get_logger().info("moving in x to pick object")
        arm_move_x_request=ArmLinearControl.Request()
        arm_move_x_request.axis = 0
        arm_move_x_request.move_length = 30.0
        arm_move_x_future = self.arm_linear_control_client.call_async(arm_move_x_request)
        arm_move_x_future.add_done_callback(self.move_x_cb)

    def move_x_cb(self,future):
        start_time = time.time()
        self.get_logger().info("activating gripper")
        gripper_request=ArmGripperControl.Request()
        gripper_request.gripper_state=False
        gripper_future=self.gripper_client.call_async(gripper_request)
        gripper_future.add_done_callback(self.move_post_pick)
        time_diff = time.time() - start_time
        self.get_logger().info(f"Close the gripper, time: {time_diff}")
        

    def move_post_pick(self,future):
        start_time = time.time()
        self.get_logger().info('Setting post pick pose...')
        arm_move_request = ArmMoveToPick.Request()
        arm_move_request.base_pose.x = 9.49
        arm_move_request.base_pose.y =  106.08
        arm_move_request.base_pose.z =  -132.09
        arm_move_request.base_pose.rx =  24.78
        arm_move_request.base_pose.ry = -10.98
        arm_move_request.base_pose.rz = 44.91
        arm_move_future = self.arm_set_pose_client.call_async(arm_move_request)
        self.pick_completed=True
        arm_move_future.add_done_callback(self.move_drop_pose)
        time_diff = time.time() - start_time
        self.get_logger().info(f"Move to home pose after Picking, time: {time_diff}")
        

    def navigate_to_goal(self, future ,pose_x,pose_y,pose_yaw):
        self.get_logger().info(f"Sending navigation goal to ")
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = pose_x
        pose.pose.position.y = pose_y
        pose.pose.orientation.w = 1.0  # Facing forward
        goal_msg.pose = pose
        self._navigate_action_client.wait_for_server()
        self.get_logger().info("Navigation action server available. Sending goal...")
        self._send_goal_future = self._navigate_action_client.send_goal_async(
            goal_msg, feedback_callback=self.navigate_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.navigate_goal_response_callback)

    def navigate_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received navigation feedback: {feedback}")

    def navigate_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Navigation goal rejected.")
            return
        self.get_logger().info("Navigation goal accepted.")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.navigate_get_result_callback)

    def navigate_get_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info("Navigation succeeded.")
            if(self.pick_completed):
                self.move_drop_pose()
                self.pick_completed=False
            elif(not self.pose_estimated):
                self.capture_pose()
                self.pose_estimated=True
            elif(self.drop_completed):
                self.pick_completed=False
                self.drop_completed=False
                self.pose_estimated=True
            else:
                return 
        except Exception as e:
            self.get_logger().error(f"Navigation action failed: {e}")
            return


    def pick_complete_cb(self):
         self.get_logger().error(f"Navigating to Drop Location")


    def move_drop_pose(self,future):
        start_time = time.time()
        self.get_logger().info('dropping object')
        arm_move_request = ArmMoveToPick.Request()
        arm_move_request.base_pose.x = 99.75
        arm_move_request.base_pose.y =  0.0
        arm_move_request.base_pose.z =  -93.16
        arm_move_request.base_pose.rx =  87.71
        arm_move_request.base_pose.ry = -10.98
        arm_move_request.base_pose.rz = 44.91
        arm_move_future = self.arm_set_pose_client.call_async(arm_move_request)
        self.pick_completed=True
        arm_move_future.add_done_callback(self.drop_object)
        time_diff = time.time() - start_time
        self.get_logger().info(f"Move to Drop Pose, time: {time_diff}")
        

    def go_to_home(self):
        self.navigate_to_goal(future=None,pose_x=0.0,pose_y=0.0,pose_yaw=0.0)

    def drop_object(self,future):
        start_time = time.time()
        self.get_logger().info("activating gripper")
        gripper_request=ArmGripperControl.Request()
        gripper_request.gripper_state=True
        gripper_future=self.gripper_client.call_async(gripper_request)
        self.drop_completed=True
        gripper_future.add_done_callback(self.move_to_home)
        self.set_parameters([Parameter('start_demo', Parameter.Type.BOOL, False)])
        self.triggered = False
        time_diff = time.time() - start_time
        self.get_logger().info(f"Drop Object, time: {time_diff}")


    def move_to_home(self,future):
        start_time = time.time()
        self.get_logger().info('Going to home pose...')
        arm_move_request = ArmMoveToPick.Request()
        arm_move_request.base_pose.x = 9.49
        arm_move_request.base_pose.y =  106.08
        arm_move_request.base_pose.z =  -132.09
        arm_move_request.base_pose.rx =  24.78
        arm_move_request.base_pose.ry = -10.98
        arm_move_request.base_pose.rz = 44.91
        arm_move_future = self.arm_set_pose_client.call_async(arm_move_request)
        self.set_parameters([Parameter('start_demo', Parameter.Type.BOOL, False)])
        self.triggered = False
        time_diff = time.time() - start_time
        self.get_logger().info(f"Back to Home pose, time: {time_diff}")



def main(args=None):
    rclpy.init(args=args)
    node = TaskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
