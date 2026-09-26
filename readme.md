# SIH 2026: PROTOTYPE

This documentation outlines the exact terminal execution sequence to demonstrate dynamic multi-agent conflict resolution for the Smart India Hackathon. By utilizing a deterministic D* Lite costmap repair over Zenoh, this sequence flawlessly showcases zero-collision routing and optimized task completion times.

---
## Dependency Installation
 * Make sure python, ros lyrical, gazebo jetty is isn is installed
 * Run this command to install Zenoh
 ```bash
    sudo apt update 
    sudo apt install ros-lyrical-rmw-zenoh-cpp
  ```
 * Install the python packages with this command globally
 ```bash
    pip3 install --break-system-packages flask flask-socketio flask-cors simple-websocket eclipse-zenoh
  ```
  
---
## Workspace Initialization

**Build the native workspace before the presentation**

```bash
cd ~/sih_prototype_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Execution Sequence
Open four separate terminals. Execute the following blocks in order.

### Terminal 1: Launch Gazebo Harmonic Environment
```bash
cd ~/sih_prototype_ws
source /opt/ros/lyrical/setup.bash
gz sim -r fleet_warehouse.sdf
```

### Terminal 2: Spawn Fleet and ROS 2 Bridges
```bash
cd ~/sih_prototype_ws
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch package_sim spawn_fleet.launch.py
```

### Terminal 3: Start Dashboard Backend 
```bash
cd ~/sih_prototype_ws
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 run fleet_dashboard dashboard_node
```
_Ensure the react frontend is running_

### Terminal 4: Start the Main Node
```bash
cd ~/sih_prototype_ws
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 run amr_core sih_director
```

## Triggering the Demo
1. Ensure Terminal 4 confirms connection with the log: Odometry Locked. Planning routes...
2. Open the React Fleet Dashboard in your web browser.
3. Click the RUN PROTOTYPE DEMO override button on the UI.
4. The dashboard will instantly visualize the D* Lite global paths in 10seconds.
5. When the AMRs breach the 2.5-meter proximity threshold, the local costmap repair will trigger, routing Robot safely around Robot 1 without collisions.

---
### After every run clean the db with the following command
```bash
python3 clean_db.py
```