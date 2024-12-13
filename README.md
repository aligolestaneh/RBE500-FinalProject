# RBE500-FinalProject

# Setup 
1. First Setup the Github , if you are using the git on your system for the first time, follow these links [1. Generate SSH Key for here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent), [2. Add to your github account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)
2. Create ROS2 workspace
    You may ignore this step if you have already a workspace available and jump to `setup step 2` with your own workspace directory
    ```bash
    $ mkdir -p ~/colcon_ws/src
    $ cd ~/colcon_ws/src
    ```
3. Setup Open Manipulator X ROS2 packages:
    ```bash
    $ git clone https://github.com/hylander2126/OpenManipulatorX_ROS2.git
    ```
4. Clone git repo in your ROS workspace (currently Private with team member access only)
    ```bash
    $ git clone git@github.com:aligolestaneh/RBE500-FinalProject.git
    $ cd ~/colcon_ws
    $ rosdep install --from-paths src --ignore-src -r -y
    $ colcon build --symlink-install
    ```
# Code Overview
## Packages
### rbe500_final_project_msgs
It has custom srv and msgs  definations required for the `rbe500_final_project` pkg. Currently it has a custom service defination to get the `joint_angles` from Inverse kinematics node.

### rbe500_final_project
A ROS2 package having all required nodes for the final project.

#### Core Libraries    
* `manipulator_core_lib`:
A C++ library independent of ROS2 which takes care of Forward and Inverse kinematics of the Open manipulator X. The DH parameters is hardcoded here but you can setup the link_lengths independently.

* `manipulator_fk_ros_lib`: 
A ROS2 wrapper library which utlizes the `manipulator_core_lib` to implement forward kinematics. 
    * **Subscribed Topics**: 
        * `/joint_state` ([sensor_msgs/msg/JointState](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html)) : **joint angles** of the mainpulator in radians.
    * **Published Topics**: 
        * `/end_effector_pose` ([geometry_msgs/msg/PoseStamped](https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html)) : Publishes pose of the end_effector w.r.t *link_0* frame. Used Forward kinematics as per current DH params of the mainpulator to get the pose data.

* `manipulator_ik_ros_lib`: 
A ROS2 wrapper library which utlizes the `manipulator_core_lib` to implement inverse kinematics.
    * **Services**: 
        * `/get_joint_angles_ik` (*rbe500_final_project_msgs/srv/GetJointAngles*) : It takes  *end_effector pose* as request and respond calculated *joint_angles* .
    

* `manipulator_follow_actions_lib`: 
A ROS2 wrapper library which reads the action sequence from the **manipulator_actions.yaml** and move the manipulator as per the given sequence. It does it in three major steps:
    1. If action type is *go_to_position* then:
        * Calculate the *joint_angles* for the given pose using **manipulator_ik_ros_lib** service: `/get_joint_angles_ik`. 
        * Moves manipualtor to the  calculated joint_angles using *OpenManipulatorX* service `/goal_joint_space_path`.
    2. If action type is *open_gripper* or *close_gripper* then:
        * Calls the *OpenManipulatorX* service `/goal_tool_control`  to open and close gripper.
    * **Published Topics**: 
        * `/goal_marker` ([visualization_msgs/msg/Marker](https://docs.ros2.org/latest/api/visualization_msgs/msg/Marker.html)) : Publishes the current `go_to_position` pose as a sphere marker.

* `manipulator_vlk_lib`:
A ROS2 wrapper library that provides services for velocity-level kinematics of the Open Manipulator X. It computes joint velocities for a given end-effector twist and vice versa. It subscribes to the `/joint_state` topic and updates the end-effector pose and DH parameters in the subscirber callback. Whenever, a service request is made, it calculates the joint velocities or the end effector twist based on the updated DH parameter and Jacobian.
    * **Services**:
        * `/get_end_effector_twist` (*rbe500_final_project_msgs/srv/GetEndEffectorTwist*): It takes target joint velocities and returns the corresponding end-effector twist. 
        * `/get_joint_velocities` (*rbe500_final_project_msgs/srv/GetJointVelocities*): It takes end-effector twist and responds the corresponding joint velocities.
* `manipulator_jp_updater_lib`:
A ROS2 wrapper library that updates joint angles based on the published joint velocity from the `manipulator_move_ee_node`. This library subscribes to reference joint velocities and calculates target joint angles, calling the OpenManipulatorX joint space service to move the manipulator to the target joint angles. It ensures that the updated joint positions are clamped based on joint position limits to avoid conflicts with the OpenManipulatorX. The update of the joint position is depend on the `/target_joint_velocities` publish rate.
    * **Subscribed Topics**:
        * `/target_joint_velocities` ([rbe500_final_project_msgs/msg/JointVelocity]): Receives joint velocities in radians/second.
        * `/joint_states` ([sensor_msgs/msg/JointState](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html)): Receives current joint angles in radians.
    * **Published Topics**:
        * `/target_joint_position` ([rbe500_final_project_msgs/msg/JointVelocity]): Publishes the calculated target joint velocities.
    * **Services**:
        * `/goal_joint_space_path` (*open_manipulator_msgs/srv/SetJointPosition*): Service to move the manipulator to the specified joint angles.
* `manipulator_move_ee_lib`:
A ROS2 library provides functionality to compute the required joint velocities based on the desired end effector twist and to publish these velocities for smooth and precise movement of the manipulator. It moves the manipulator intially at the `home` cofiguration and then publish the joint velocities by calling the `get_joint_velocities` service at a frequency defined in the `manipulator_vlk_params.yaml` param file.
    * **Published Topics**:
        * `/target_joint_velocities` ([rbe500_final_project_msgs/msg/JointVelocity]): Publishes the calculated target joint velocities. It uses the services provided from the `manipulator_vlk_lib` .
    * **Services**:
        * `/get_joint_velocities` (*open_manipulator_msgs/srv/SetJointPosition*): Service to get the manipulator joint velocity from the `manipulator_vlk_node` for the specifies end effector twist.
* `manipulator_pd_lib`:
A PD controller library provides functionality to calculate the control effort required to move a joint to a desired position using PD control principles. Users can set the proportional (Kp) and derivative (Kd) gains to tune the controller's performance.

* `manipulator_pd_ros_lib`:
A ROS2 wrapper library that implements a PD controller for the OpenX Manipulator. This library updates joint angles based on the current joint position and desired target joint angle, using a PD control strategy to compute the necessary current to move the manipulator. The output is then published to the `/set_current` topic to control the manipulator's movement.
    * **Subscribed Topics**:
        * `/set_current`([dynamixel_sdk_custom_interfaces/msg/SetCurrent]): Subscribes to this topic to receive the current settings for the manipulator's joint.
    * **Published Topics**:
        * `/pd_output_viz`([geometry_msgs/msg/PointStamped]): Publishes PD controller output data for visualization and debugging purposes.
    * **Services**:
        * `/get_position`(*dynamixel_sdk_custom_interfaces/srv/GetPosition*): Service client to retrieve the current joint angle of the manipulator.
        * `/get_current`(*dynamixel_sdk_custom_interfaces/srv/GetCurrent*): Service client to retrieve the current ampere being used by the manipulator's joint.


## Parameters
* `manipulator_core_pramas.yaml`: It has all the requried parameters for setting up *manipulator_core_lib* .
    1. `ik_max_iteration` : 
        * Type: Integer
        * Decription: Max interation to do for Newton Raphson Loop

    2. `ik_tolerance`:
        * Type: Double
        * Decription: Epsilon value to check convergance of Newton raphson method

    3. `use_newton_raphson_ik`:
        * Type: Bool
        * Decription: Set `TRUE` to use Netwon Raphson method ofr INverse kinematics.
    
    4. `link_lengths`:
        * Type: List of Paramters
        * Decription: List of link_length parameters, each having its name and Double type value

    5. `manipulator_links`:
        * Type: List of String
        * Decription: Link names to be used for setting the DH params. These names act as the parameter names of link_lengths vector. For ex: If LinkA has been mentioned , it will only read the `LinkA` parameter from the `link_lengths` paramter list.

* `manipulator_actions.yaml`: It contains poses and action sequence to move the manipualtor to various positions. 
    * **poses**: It has list of end effector pose (position and oreintation) w.r.t to *link_0*.
    
        User can addd new poses as per this template:
        ```yaml
        poses:
            <pose-name>:
                position:
                    x: <double value>
                    y: <double value>
                    z: <double value>
                orientation:
                    x: <double value>
                    y: <double value>
                    z: <double value>
                    w: <double value>
        ```
        The best way is to directly copy paste the data from `/end_effector_pose` topic. For example:
        ```yaml
        poses:
            show_pick_obj:
                position:
                    x: 0.006
                    y: 0.108
                    z: 0.266
                orientation:
                    x: 0.549
                    y: 0.445
                    z: -0.524
                    w: -0.476    
        ```
    * **actions**: It has a list of actions to be executed using **manipulator_follow_actions_lib** sequentially. Each action has following parameters:
        - **type(*string*)** : Type of the action will be single string from the following sets:
    
            * *go_to_position*: It moves the manipulator to the pose.
            * *open_gripper*: It completely opens the gripper.
            * *close_gripper*: It completely clsoes the gripper.
        - **position(*string*)**: Pose name corresponding to a pose specified in *poses* parameter.   It gets used only if your action type is *go_to_position*.
        - **wait(integer)**: Wait time in seconds after an action has been completed. If you don't mention it. The default value get set to 2 seconds automatically in the code.
        
        User can addd new *go_to_position* action as per this template:
        ```yaml
        actions:
            - type: go_to_position
            position: home_position
            wait: 1

            OR
        actions:
            - type: go_to_position
            position: home_position  
        ```
        User can addd new *open_gripper* action as per this template:
        ```yaml
        actions:
            - type: open_gripper
            wait: 1

            OR
        actions:
            - type: open_gripper
        ```
        User can addd new *close_gripper* action as per this template:
        ```yaml
        actions:
            - type: close_gripper
            wait: 1

            OR
        actions:
            - type: close_gripper
        ```
* `manipulator_vlk_params.yaml`: It contains parameters required for moving the end effector using velocity kinematics. The parameters are as follows:
    1. `joint_position_limits`: 
        * Type: List of Doubles
        * Description: The max absolute limits for the joint positions in radians. This ensures that the manipulator does not exceed its physical limits.
    
    2. `end_effectot_twist_linear`: 
        * Type: List of Doubles
        * Description: Linear twist for the end effector, represented as a vector [vx, vy, vz]. This defines the desired linear velocity of the end effector.
    
    3. `update_frequency`: 
        * Type: Double
        * Description: Frequency at which the manipulator updates its position, in Hertz (Hz). This determines how often the control commands are sent to the manipulator.
    
    4. `move_duration`: 
        * Type: Double
        * Description: Duration for which the manipulator should move, in seconds. This specifies how long the manipulator will execute the movement commands.
    
    5. `home_joint_pos`: 
        * Type: List of Doubles
        * Description: The home position for the joints of the manipulator, specified in radians. This is the position to which the manipulator will return when homing.
* `manipulator_pd.yaml`: It contains the parameters for the PD controller and setpoint:
    1. `current_limits`: 
        * Type: List of Integers
        * Description: The limits for the current in degrees, specifying the minimum and maximum allowable values for the joint's current.
    
    2. `joint_id`: 
        * Type: Integer
        * Description: The identifier for the joint being controlled. This is used to specify which joint the parameters apply to.
    
    3. `target_joint_angle`: 
        * Type: Integer
        * Description: The desired target angle for the joint in tenths of degrees. This is the angle the PD controller aims to achieve.
    
    4. `kp`: 
        * Type: Double
        * Description: The proportional gain for the PD controller. This value influences how aggressively the controller responds to the error between the current and target joint angles.
    
    5. `kd`: 
        * Type: Double
        * Description: The derivative gain for the PD controller. This value affects the damping of the controller response, helping to reduce overshoot and oscillations.


# Running Nodes
## Forward Kinematics:
1. Launch the OpenX Manipulator control
    ```bash
    $ source ~/colcon_ws/install/setup.bash
    $ ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
    ```
2. Launch Forward Kinematics Node:
    ```bash
    $ source ~/colcon_ws/install/setup.bash
    $ ros2 launch rbe500_final_project manipulator_fk_ros.launch.py
    ```
3. Check End effector pose:
    ```bash
    $ ros2 topic echo /end_effector_pose
    ```
## Inverse Kinematics:
1. Launch the OpenX Manipulator control
    ```bash
    $ source ~/colcon_ws/install/setup.bash
    $ ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
    ```
2. Launch Inverse Kinematics Node:
    ```bash
    $ source ~/colcon_ws/install/setup.bash
    $ ros2 launch rbe500_final_project manipulator_ik_ros.launch.py
    ```
3. Get Joint Angles:
    ```bash
    $ source ~/colcon_ws/install/setup.bash
    $ ros2 service call /get_joint_angles_ik rbe500_final_project_msgs/srv/GetJointAngles "{end_effector_pose: {position: {x: 0.1, y: 0.2, z: 0.3}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
    ```
## Pick and Place Action sequence:
1. Launch the OpenX Manipulator control
    ```bash
    $ source ~/colcon_ws/install/setup.bash
    $ ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
    ```
2. Launch Inverse Kinematics Node:
    ```bash
    $ source ~/colcon_ws/install/setup.bash
    $ ros2 launch rbe500_final_project manipulator_ik_ros.launch.py
    ```
3. Execute Pick and Place action sequence
    ```bash
    $ source ~/colcon_ws/install/setup.bash
    $ ros2 run rbe500_final_project manipulator_follow_actions_node
    ```
## Moving End-Effector with Twist:
1. Launch the OpenX Manipulator control
    ```bash
    $ source ~/colcon_ws/install/setup.bash
    $ ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
    ```
2. Launch Velocity Kinematics Node:
    ```bash
    $ source ~/colcon_ws/install/setup.bash
    $ ros2 launch rbe500_final_project manipulator_vlk_ros.launch.py
    ```
3. Execute Move End effector Node
    ```bash
    $ source ~/colcon_ws/install/setup.bash
    $ ros2 launch rbe500_final_project manipulator_move_ee.launch.py
    ```

## Moving Joint4 with PD controller:
1. Run the Dynamixel Read Write Current node:
    ```bash
    $ source ~/colcon_ws/install/setup.bash
    $ ros2 run dynamixel_sdk_examples current_read_write_node
    ```
2. Run the Dynamixel Read Write Position node:
    ```bash
    $ source ~/colcon_ws/install/setup.bash
    $ ros2 run dynamixel_sdk_examples read_write_node
    ```
3. Execute PD controller launch
    ```bash
    $ source ~/colcon_ws/install/setup.bash
    $ ros2 launch rbe500_final_project manipulator_pd_ros.launch.py
    ```

# Writing Test Cases 
## CMakelist Edits
All test cases has been written up in the test folder. You may add your test cases aswell by creating your own test `cpp` file or editing the existing ones. Please don't edit `main.cpp`.  Make sure you add it into `CMakelist.txt` in order to build your unit test code.
```cmake
set(TESTFILES
  test/main.cpp
  test/simple_test.cpp
  test/<your_test_file_name>_test.cpp) #<<------ add here

```
## Compiling and Test case results

In order to view the test cases results and you need to build the package first:
```bash
$ cd <your-workspace> 
$ colcon build --symlink-install --packages-select rbe500_final_project
```
Now you can build the test cases:
1. Compiling without test cases result verbose:
    ```bash
    $ colcon test --packages-select rbe500_final_project
    ```
2. Compiling the test cases along with result verbose:
    ```bash
    $ colcon test --packages-select rbe500_final_project --event-handler=console_direct+
    ```

