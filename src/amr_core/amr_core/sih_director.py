import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
import zenoh, math, time, threading
from rclpy.serialization import serialize_message

from amr_core.local_planner import LocalPlanner
from fleet_interfaces.msg import LocalTrajectory, AMRTelemetry, Pose2D
from geometry_msgs.msg import Point32

# Embedded path_2.json routes
FLEET_ROUTES = {
    "robot_1": {
        "leg_1" : [[-12.21, -0.54], [-11.62, 0.01], [-11.34, 0.62],[-10.55, 1.14], [-9.32, 1.8],[-8.71, 2.33], [-8.18, 2.33], [-7.79, 2.59],[-6.79, 2.59], [-6.23, 2.59], [-5.66, 2.59],[-5.02, 2.59], [-4.59, 2.59], [-4.11, 2.99],[-3.69, 2.99], [-3.14, 2.99], [-2.81, 2.99], [-2.18, 2.99],[-1.95, 2.99], [-1.56, 2.99], [-1.09, 2.99], [-0.46, 2.99],[0.14, 2.99], [0.803, 2.99], [1.13, 2.99], [1.22, 2.99],[1.44, 2.99], [1.95, 2.99], [2.42, 2.99], [2.94, 2.99],[3.55, 3.14], [4.29, 3.14], [4.93, 3.14], [5.32, 3.26],[5.91, 3.50], [6.24, 3.90], [7.52, 4.92], [8.33, 5.55],[8.86, 6.32], [9.80, 7.3], [10.3, 8.09], [10.5, 8.52], [10.5, 9.0]],
        "leg_2" : [[10.5, 9], [10.5, 8.9], [10.05, 8.13], [9.51, 7.18],[8.88, 6.17], [8.58, 5.23], [8.16, 5.19],[7.66, 4.54], [7.25, 4.13], [6.48, 3.71],[6.04, 3.33], [5.52, 3.24], [4.90, 3.16],[4.49, 3.16], [3.67, 3.16], [2.43, 3.16], [1.60, 2.82],[0.35, 2.41], [-0.5, 2.06], [-1.40, 1.43],[-2.14, 1.43], [-2.93, 0.92], [-3.67, 0.46],[-4.59, -0.18], [-5.48, -0.72], [-6.15, -1.0],[-7.32, -1.0], [-8.0, -1.0], [-9.0, -1.0]]
        },
    "robot_2": {
        "leg_1_initial" : [[-13, -6], [-12.50, -5.57], [-11.5, -4.90],[-10.5, -4.6], [-9.9, -4.03], [-9.6, -3.3],[-8.7, -2.19], [-7.83, -1.62], [-6.99, -1.63],[-6.24, -1.01], [-5.54, -0.40], [-5.07, 0.640],[-4.19, 1.51], [-3.48, 2.11], [-2.43, 2.86],[-2.063, 3.08], [-1.04, 3.02], [0.02, 2.92],[1.13, 3.27], [2.33, 3.23], [3.64, 3.19],[4.66, 3.16], [5.2, 3.14], [6.38, 3.10],[7.66, 3.195], [8.14, 3.80], [9.42, 4.44],[10.01, 4.89], [10.50, 4.86], [10.5, 5.5]],
        "leg_1_final" : [[-13, -6], [-12.50, -5.57], [-11.5, -4.90],[-10.5, -4.6], [-9.9, -4.03], [-9.6, -3.3],[-8.7, -2.29], [-7.83, -1.62], [-6.99, -1.63],[-6.24, -1.01], [-5.54, -0.40], [-5.07, 0.640],[-4.19, 1.51], [-3.48, 2.11], [-2.43, 2.86],[-2.063, 3.08], [-1.04, 3.02], [0.02, 2.92],[1.13, 3.27],[3.3, 3.2], [3.5, 2.3], [4.0, 1.18],[4.00, 0.3], [4.00, -0.9],[4.00, -1.84], [4.13, -2.8],[4.43, -3.2], [5.13, -3.33], [6.43, -3.70],[8.62, -3.2], [9.16, -2.5], [9.20, -1.34],[9.6, 0.55], [10.3, 2.82], [10.4, 3.97],[10.5, 4.5], [10.5, 5.5]],
        "leg_2" : [[10.5, 5.5],[10.0, 6.0],[10.0, 6.3],[10.0, 6.9],[10.0, 7.5],[10.0, 7.9],[9.5, 8.3],[9.0, 8.7],[8.5, 9.3],[8.0, 9.5],[7.5, 9.5],[7.0, 9.5],[6.8, 9.5],[6.2, 9.5],[5.5, 9.5],[5.0, 9.5],[4.5, 9.5],[4.0, 9.5],[3.5, 9.5],[3.0, 9.5],[2.1, 9.5],[1.7, 9.5],[1.2, 9.5],[0.5, 9.5],[0, 9.5],[-0.5, 9.5],[-1.0, 9.5],[-1.5, 9.5],[-2.0, 9.5],[-2.5, 9.5],[-3.0, 9.5],[-3.5, 9.5],[-4.0, 9.5],[-4.5, 9.5],[-5.0, 9.5],[-5.5, 9.5],[-7.0, 9.5],[-7.5, 9.5],[-8.0, 9.5],[-9.0, 9.5],[-10.0, 9.5],[-10.5, 9.5],[-11.0, 9.5],[-11.5, 9.5]]
        },
    "robot_3": {
        "leg_1" : [[-12.9, -10.6],[-12.4, -10.04],[-11.8,-9.6],[-11.3,-9.16],[-10.8,-8.4],[-10.5, -8.0],[-10.5, -7.5],[-10.5, -7.0],[-10.5, -6.5],[-10.5, -6.0],[-10.5, -5.5],[-10.5, -5.0],[-10.5, -4.5],[-10.5, -4.0],[-10.5, -3.5],[-10.5, -3.0],[-10.5, -2.5],[-10.5, -2.0],[-10.5, -1.5],[-10.5, -1.0],[-10.5, -0.5],[-10.5, -0.0],[-10.0, 0.5],[-9.5, 1.0],[-9.0, 1.5],[-9.5, 1.0], [-9.0, 1.5],[-8.5, 2.0],[-8.0, 2.5],[-7.5, 3.0],[-7.5, 3.6],[-7.5, 4.0],[-7.5, 5.6],[-8.5, 6.4],[-9.0, 6.5],[-9.5, 7.0],[-10.0, 7.5  ],[-10.5, 8.0],[-11.0, 8.5],[-11.5, 9.0],[-11.5, 9.5],[-11.5, 10.0],[-11.5, 10.5],[-11.5, 11.0],[-11.5, 11.5],[-11.5, 12.0],[-11.5, 12.5],[-11.5, 13.0]],
        "leg_2" : [[-11.04, 12.41], [-9.9, 11.82], [-8.25, 10.02],[-6.68, 8.7], [-5.6, 7.7], [-4.23, 6.9],[-3.64, 7.0], [-2.55, 7.0], [-2.0, 7.0],[-1.5, 7.0], [-1.0, 7.0], [-0.5, 7.0], [0, 7.0],[0.5, 7.0], [1.0, 7.0], [1.5, 7.0], [2.0, 7.0],[2.5, 7.0], [3.0, 7.0], [3.5, 7.0], [4.0, 7.0],[4.5, 7.0], [5.0, 7.0], [5.5, 7.0], [6.0, 6.3],[6.5, 5.5], [7.2, 5.1], [8.0, 4.2],[8.5, 3.5], [9.1, 2.0],[9.6, 1.0], [10.04, -0.05],[10.2, -1.5], [11.2, -2.5], [11.5, -3.0],[11.5, -4.00], [11.5, -5.0]]
        }
}

class PrototypeDirector(Node):
    def __init__(self):
        super().__init__('sih_director_node')
        
        self.poses = {
            'robot_1': {'x': None, 'y': None, 'yaw': 0.0},
            'robot_2': {'x': None, 'y': None, 'yaw': 0.0},
            'robot_3': {'x': None, 'y': None, 'yaw': 0.0}
        }
        
        # State machine now includes 'started' to wait for individual UI clicks
        self.mission = {
            'robot_1': {'leg': 'TO_PICKUP', 'started': False, 'active_path': FLEET_ROUTES['robot_1']['leg_1'], 'next_path': FLEET_ROUTES['robot_1']['leg_2'], 'charge_path': [[-9.0, -1.0], [-13.0, -1.0]], 'task_id': 'DEMO_R1', 'done': False},
            'robot_2': {'leg': 'TO_PICKUP', 'started': False, 'active_path': FLEET_ROUTES['robot_2']['leg_1_initial'], 'next_path': FLEET_ROUTES['robot_2']['leg_2'], 'task_id': 'DEMO_R2', 'done': False, 'dodged': False},
            'robot_3': {'leg': 'TO_PICKUP', 'started': False, 'active_path': FLEET_ROUTES['robot_3']['leg_1'], 'next_path': FLEET_ROUTES['robot_3']['leg_2'], 'task_id': 'DEMO_R3', 'done': False}
        }

        self.local_drivers = {
            'robot_1': LocalPlanner(v_max=1.5, k_v=1.5, k_omega=3.0, d_tolerance=0.4),
            'robot_2': LocalPlanner(v_max=1.5, k_v=1.5, k_omega=3.0, d_tolerance=0.4),
            'robot_3': LocalPlanner(v_max=1.5, k_v=1.5, k_omega=3.0, d_tolerance=0.4)
        }

        self.batteries = {
            'robot_1': 99.5,
            'robot_2': 98.0,
            'robot_3': 100.0
        }

        # Initialize local planners with their starting paths
        for r_id, m in self.mission.items():
            self.local_drivers[r_id].on_path(m['active_path'])

        # Native ROS 2 Subscriptions and Publishers
        self.sub_r1 = self.create_subscription(Odometry, '/robot_1/odom', lambda msg: self.odom_cb('robot_1', msg), qos_profile_sensor_data)
        self.sub_r2 = self.create_subscription(Odometry, '/robot_2/odom', lambda msg: self.odom_cb('robot_2', msg), qos_profile_sensor_data)
        self.sub_r3 = self.create_subscription(Odometry, '/robot_3/odom', lambda msg: self.odom_cb('robot_3', msg), qos_profile_sensor_data)

        self.cmd_publishers = {
            'robot_1': self.create_publisher(Twist, '/robot_1/cmd_vel', 10),
            'robot_2': self.create_publisher(Twist, '/robot_2/cmd_vel', 10),
            'robot_3': self.create_publisher(Twist, '/robot_3/cmd_vel', 10)
        }

        z_conf = zenoh.Config()
        z_conf.insert_json5("mode", '"client"')
        self.z_session = zenoh.open(z_conf)

        self.telem_pub = self.z_session.declare_publisher('fleet_status')
        self.traj_pub = self.z_session.declare_publisher('fleet_trajectories')
        
        # Listen to both potential task assignment topics for maximum compatibility
        self.trigger_sub = self.z_session.declare_subscriber('fleet/sih_override', self.task_assignment_cb)
        self.task_sub = self.z_session.declare_subscriber('fleet/task_assign', self.task_assignment_cb)

        self.get_logger().info("SIH Responsive Director Online. Awaiting specific task assignments...")
        
        # Start control loop instantly in the background
        threading.Thread(target=self.control_loop, daemon=True).start()

    def odom_cb(self, robot_id, msg):
        self.poses[robot_id]['x'] = msg.pose.pose.position.x
        self.poses[robot_id]['y'] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.poses[robot_id]['yaw'] = math.atan2(siny_cosp, cosy_cosp)

    def publish_telemetry(self, robot_id, task_id, is_driving):
        pose = self.poses[robot_id]
        if pose['x'] is None: return
        msg = AMRTelemetry()
        msg.robot_id = robot_id
        msg.pose = Pose2D(x=float(pose['x']), y=float(pose['y']), theta=float(pose['yaw']))
        msg.battery_percent = float(self.batteries[robot_id])
        msg.system_status = 1 if is_driving else 0
        msg.is_busy = is_driving
        msg.current_task_id = task_id
        self.telem_pub.put(serialize_message(msg))

    def broadcast_trajectory(self, robot_id, path, remaining_paths=None):
        full_path = list(path)
        if remaining_paths:
            for p in remaining_paths:
                full_path.extend(p)
            
        traj_msg = LocalTrajectory()
        traj_msg.robot_id = robot_id
        traj_msg.future_waypoints = [Point32(x=float(x), y=float(y), z=0.0) for x, y in full_path]
        self.traj_pub.put(serialize_message(traj_msg))

    def get_sliced_dodge_path(self, current_x, current_y, full_dodge_path):
        min_dist = float('inf')
        start_idx = 0
        for i, wp in enumerate(full_dodge_path):
            d = math.hypot(current_x - wp[0], current_y - wp[1])
            if d < min_dist:
                min_dist = d
                start_idx = i
        return full_dodge_path[start_idx:]

    def task_assignment_cb(self, sample):
        try:
            payload = sample.payload.decode('utf-8')
        except:
            payload = str(sample.payload)
            
        self.get_logger().info(f"UI Event Received: {payload}")
        
        # Activate robots individually if their ID is found in the payload
        assigned = False
        for r_id in ['robot_1', 'robot_2', 'robot_3']:
            if r_id in payload or self.mission[r_id]['task_id'] in payload:
                if not self.mission[r_id]['started']:
                    self.mission[r_id]['started'] = True
                    self.get_logger().info(f"[{r_id}] Task Assigned! Activating hardcoded route.")
                    
                    rem_paths = [self.mission[r_id]['next_path']]
                    if r_id == 'robot_1':
                        rem_paths.append(self.mission[r_id]['charge_path'])
                        
                    self.broadcast_trajectory(r_id, self.mission[r_id]['active_path'], remaining_paths=rem_paths)
                    assigned = True
                
        # Fallback: if the UI sends a generic start without specifying a robot, activate all unstarted
        if not assigned:
            self.get_logger().info("Generic start detected. Activating all idle robots.")
            for r_id, m in self.mission.items():
                if not m['started']:
                    m['started'] = True
                    rem_paths = [m['next_path']]
                    if r_id == 'robot_1':
                        rem_paths.append(m['charge_path'])
                    self.broadcast_trajectory(r_id, m['active_path'], remaining_paths=rem_paths)

    def control_loop(self):
        while rclpy.ok() and any(self.poses[r]['x'] is None for r in ['robot_1', 'robot_2', 'robot_3']):
            time.sleep(0.1)
            
        self.get_logger().info("All 3 AMRs localized. Waiting for individual task assignments...")

        tick = 0

        while rclpy.ok():
            tick += 1
            for r_id in ['robot_1', 'robot_2', 'robot_3']:
                m = self.mission[r_id]
                
                # Always update odometry so telemetry is accurate even while idle
                self.local_drivers[r_id].on_odometry(self.poses[r_id]['x'], self.poses[r_id]['y'], theta=self.poses[r_id]['yaw'])
                
                is_driving = m['started'] and not m['done']
                if tick % 10 == 0:
                    self.publish_telemetry(r_id, m['task_id'] if is_driving else "IDLE", is_driving)

                if is_driving:
                    # Drain battery while moving
                    self.batteries[r_id] -= 0.0001
                    if self.batteries[r_id] < 15.0: self.batteries[r_id] = 15.0

                    # SCENE 1: Head-On Deadlock
                    if r_id == 'robot_2' and self.mission['robot_1']['started'] and not self.mission['robot_1']['done']:
                        d_r1_r2 = math.hypot(self.poses['robot_2']['x'] - self.poses['robot_1']['x'], 
                                             self.poses['robot_2']['y'] - self.poses['robot_1']['y'])
                        
                        if d_r1_r2 < 4.6 and not m['dodged'] and m['leg'] == 'TO_PICKUP':
                            self.get_logger().info("[SCENE 1] Head-on detected! Robot 2 swapping to detour array.")
                            sliced_path = self.get_sliced_dodge_path(self.poses['robot_2']['x'], self.poses['robot_2']['y'], FLEET_ROUTES['robot_2']['leg_1_final'])
                            self.local_drivers['robot_2'].on_path(sliced_path)
                            m['active_path'] = sliced_path
                            self.broadcast_trajectory('robot_2', sliced_path, remaining_paths=[m['next_path']])
                            m['dodged'] = True

                    driver = self.local_drivers[r_id]
                    
                    if not driver.is_active:
                        if m['leg'] == 'TO_PICKUP':
                            self.get_logger().info(f"[{r_id}] Reached PICKUP. Transitioning to DROP...")
                            m['leg'] = 'TO_DROP'
                            m['active_path'] = m['next_path']
                            driver.on_path(m['active_path'])
                            
                            rem_paths = [m['charge_path']] if r_id == 'robot_1' else None
                            self.broadcast_trajectory(r_id, m['active_path'], remaining_paths=rem_paths)
                            
                        elif m['leg'] == 'TO_DROP' and r_id == 'robot_1':
                            self.get_logger().info(f"[{r_id}] Reached DROP. Transitioning to CHARGE...")
                            m['leg'] = 'TO_CHARGE'
                            m['active_path'] = m['charge_path']
                            driver.on_path(m['active_path'])
                            self.broadcast_trajectory(r_id, m['active_path'])
                            
                        else:
                            self.get_logger().info(f"[{r_id}] Reached Final Goal. Task Completed.")
                            m['done'] = True
                            
                            # CRITICAL FIX: Tell the Dashboard the task is complete
                            self.z_session.put("fleet/task_complete", f"{r_id} {m['task_id']}")
                            
                            self.cmd_publishers[r_id].publish(Twist())
                            continue

                    # Execute motion
                    v, w = driver.step() if driver.is_active else (0.0, 0.0)
                    t = Twist()
                    t.linear.x, t.angular.z = float(v), float(w)
                    self.cmd_publishers[r_id].publish(t)
                else:
                    # Recharge battery if idle and parked near a dock/spawn (e.g. x < -12)
                    if self.poses[r_id]['x'] is not None and self.poses[r_id]['x'] < -12.0:
                        self.batteries[r_id] += 0.05
                        if self.batteries[r_id] > 100.0: self.batteries[r_id] = 100.0
                        
                    # Keep idle robots securely stopped
                    self.cmd_publishers[r_id].publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = PrototypeDirector()
    rclpy.spin(node)
    node.z_session.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()