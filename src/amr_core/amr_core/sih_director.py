import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
import zenoh, math, time, threading, os
from rclpy.serialization import serialize_message
from ament_index_python.packages import get_package_share_directory

from amr_core.local_planner import LocalPlanner
from amr_core.navigator import DStarLitePlanner
# ADDED: Imported AMRTelemetry and Pose2D for dashboard updates
from fleet_interfaces.msg import LocalTrajectory, AMRTelemetry, Pose2D
from geometry_msgs.msg import Point32

class PrototypeDirector(Node):
    def __init__(self):
        super().__init__('sih_director_node')
        
        self.r1_pose = {'x': None, 'y': None, 'yaw': 0.0}
        self.r2_pose = {'x': None, 'y': None, 'yaw': 0.0}
        
        # Native ROS 2 Pub/Sub (Will automatically connect to Gazebo Bridge)
        self.sub_r1 = self.create_subscription(Odometry, '/robot_1/odom', self.odom_r1, qos_profile_sensor_data)
        self.sub_r2 = self.create_subscription(Odometry, '/robot_2/odom', self.odom_r2, qos_profile_sensor_data)
        
        self.pub_r1 = self.create_publisher(Twist, '/robot_1/cmd_vel', 10)
        self.pub_r2 = self.create_publisher(Twist, '/robot_2/cmd_vel', 10)
        
        # Zenoh setup for Dashboard UI communication
        z_conf = zenoh.Config()
        z_conf.insert_json5("mode", '"client"') # Prevents port crashes
        self.z_session = zenoh.open(z_conf)
        
        # ADDED: Telemetry publisher for React UI
        self.telem_pub = self.z_session.declare_publisher('fleet_status')
        self.traj_pub = self.z_session.declare_publisher('fleet_trajectories')
        self.trigger_sub = self.z_session.declare_subscriber('fleet/sih_override', self.start_sequence)
        
        self.get_logger().info("SIH Director Node Online. Waiting for React UI Trigger...")

    def odom_r1(self, msg):
        self.r1_pose['x'] = msg.pose.pose.position.x
        self.r1_pose['y'] = msg.pose.pose.position.y
        self.r1_pose['yaw'] = self.extract_yaw(msg)

    def odom_r2(self, msg):
        self.r2_pose['x'] = msg.pose.pose.position.x
        self.r2_pose['y'] = msg.pose.pose.position.y
        self.r2_pose['yaw'] = self.extract_yaw(msg)

    def extract_yaw(self, msg):
        q = msg.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # ADDED: Function to broadcast live telemetry to the UI
    def publish_telemetry(self, robot_id, pose, task_id):
        if pose['x'] is None: 
            return
        msg = AMRTelemetry()
        msg.robot_id = robot_id
        msg.pose = Pose2D(x=float(pose['x']), y=float(pose['y']), theta=float(pose['yaw']))
        msg.battery_percent = 100.0
        msg.system_status = 1
        msg.is_busy = True
        msg.current_task_id = task_id
        self.telem_pub.put(serialize_message(msg))

    def broadcast_trajectory(self, robot_id, path):
        traj_msg = LocalTrajectory()
        traj_msg.robot_id = robot_id
        traj_msg.future_waypoints = [Point32(x=float(x), y=float(y), z=0.0) for x, y in path]
        self.traj_pub.put(serialize_message(traj_msg))

    def get_dstarlite_path(self, planner, start_x, start_y, target_x, target_y):
        start_col, start_row = planner.world_to_grid(start_x, start_y)
        target_col, target_row = planner.world_to_grid(target_x, target_y)
        grid_path = planner.compute_path((start_col, start_row), (target_col, target_row))
        if not grid_path: return []
        return [planner.grid_to_world(col, row) for col, row in grid_path]

    def generate_full_route(self, planner, spawn_x, spawn_y, pick_x, pick_y, drop_x, drop_y):
        leg1 = self.get_dstarlite_path(planner, spawn_x, spawn_y, pick_x, pick_y)
        leg2 = self.get_dstarlite_path(planner, pick_x, pick_y, drop_x, drop_y)
        if leg1 and leg2:
            return leg1 + leg2[1:]
        return leg1 + leg2

    def trigger_dynamic_obstacle(self, planner, obstacle_x, obstacle_y, current_x, current_y, padding=2):
        obs_col, obs_row = planner.world_to_grid(obstacle_x, obstacle_y)
        start_col, start_row = planner.world_to_grid(current_x, current_y)
        changed_cells = []
        for r in range(obs_row - padding, obs_row + padding + 1):
            for c in range(obs_col - padding, obs_col + padding + 1):
                if planner._in_bounds(c, r):
                    changed_cells.append((c, r, 1))
        repaired_grid = planner.update_obstacles(changed_cells, start=(start_col, start_row))
        if not repaired_grid: return []
        return [planner.grid_to_world(col, row) for col, row in repaired_grid]

    def vector_to_twist(self, vx, vy, current_yaw):
        if abs(vx) < 0.01 and abs(vy) < 0.01: return 0.0, 0.0
        target_yaw = math.atan2(vy, vx)
        yaw_err = target_yaw - current_yaw
        while yaw_err > math.pi: yaw_err -= 2.0 * math.pi
        while yaw_err < -math.pi: yaw_err += 2.0 * math.pi
        v_mag = math.hypot(vx, vy)
        return min(v_mag * max(0.0, math.cos(yaw_err)), 0.5), max(-1.0, min(1.0, 2.0 * yaw_err))

    def start_sequence(self, sample):
        self.get_logger().info("OVERRIDE TRIGGERED. Launching D* Lite thread...")
        threading.Thread(target=self.control_loop, daemon=True).start()

    def control_loop(self):
        while rclpy.ok() and (self.r1_pose['x'] is None or self.r2_pose['x'] is None):
            time.sleep(0.1)
            
        self.get_logger().info("Odometry Locked. Planning routes...")
        share_dir = get_package_share_directory('amr_core')
        costmap_path = os.path.join(share_dir, 'config', 'costmap.json')
        
        dstar_r1 = DStarLitePlanner.from_costmap(costmap_path, robot_radius=0.95)
        dstar_r2 = DStarLitePlanner.from_costmap(costmap_path, robot_radius=0.95)

        # TARGET WAYPOINTS
        R1_PICKUP, R1_DROP = (-11.5, 13.0), (-9.0, -16.0)
        R2_PICKUP, R2_DROP = (-11.5, 13.0), (-2.0, -21.0)
        
        # We simplify to direct point-to-point for the prototype demonstration
        path_r1 = self.generate_full_route(dstar_r1, self.r1_pose['x'], self.r1_pose['y'], R1_PICKUP[0], R1_PICKUP[1], R1_DROP[0], R1_DROP[1])
        path_r2 = self.generate_full_route(dstar_r2, self.r2_pose['x'], self.r2_pose['y'], R2_PICKUP[0], R2_PICKUP[1], R2_DROP[0], R2_DROP[1])
        
        planner_r1 = LocalPlanner(v_max=0.4, lookahead=0.5)
        planner_r2 = LocalPlanner(v_max=0.4, lookahead=0.5)
        planner_r1.on_path(path_r1)
        planner_r2.on_path(path_r2)
        
        self.broadcast_trajectory("robot_1", path_r1)
        self.broadcast_trajectory("robot_2", path_r2)
        
        detour_triggered = False
        
        while rclpy.ok() and (planner_r1.is_active or planner_r2.is_active):
            planner_r1.on_odometry(self.r1_pose['x'], self.r1_pose['y'], self.r1_pose['yaw'])
            planner_r2.on_odometry(self.r2_pose['x'], self.r2_pose['y'], self.r2_pose['yaw'])
            
            # ADDED: Push live coordinates to the dashboard
            self.publish_telemetry("robot_1", self.r1_pose, "DEMO_R1")
            self.publish_telemetry("robot_2", self.r2_pose, "DEMO_R2")

            dist = math.hypot(self.r1_pose['x'] - self.r2_pose['x'], self.r1_pose['y'] - self.r2_pose['y'])
            if dist < 2.5 and not detour_triggered:
                self.get_logger().info("CONFLICT DETECTED! Executing D* Lite local repair...")
                path_r2_detour = self.trigger_dynamic_obstacle(
                    dstar_r2, self.r1_pose['x'], self.r1_pose['y'], 
                    self.r2_pose['x'], self.r2_pose['y'], padding=2
                )
                if path_r2_detour:
                    planner_r2.on_path(path_r2_detour)
                    self.broadcast_trajectory("robot_2", path_r2_detour)
                detour_triggered = True

            vx1, vy1 = planner_r1.step() if planner_r1.is_active else (0.0, 0.0)
            vx2, vy2 = planner_r2.step() if planner_r2.is_active else (0.0, 0.0)
            
            v1, w1 = self.vector_to_twist(vx1, vy1, self.r1_pose['yaw'])
            v2, w2 = self.vector_to_twist(vx2, vy2, self.r2_pose['yaw'])
            
            t1, t2 = Twist(), Twist()
            t1.linear.x, t1.angular.z = float(v1), float(w1)
            t2.linear.x, t2.angular.z = float(v2), float(w2)
            
            self.pub_r1.publish(t1)
            self.pub_r2.publish(t2)
            time.sleep(0.05)

        stop = Twist()
        self.pub_r1.publish(stop)
        self.pub_r2.publish(stop)
        
        # Final idle telemetry push
        self.publish_telemetry("robot_1", self.r1_pose, "IDLE")
        self.publish_telemetry("robot_2", self.r2_pose, "IDLE")
        self.get_logger().info("Sequence complete.")

def main(args=None):
    rclpy.init(args=args)
    node = PrototypeDirector()
    rclpy.spin(node)
    node.z_session.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()