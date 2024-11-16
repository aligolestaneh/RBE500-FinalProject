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

