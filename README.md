# RBE500-FinalProject

# Setup 
1. First Setup the Github , if you are using the git on your system for the first time, follow these links [1. Generate SSH Key for here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent), [2. Add to your github account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)
2. Create ROS2 workspace
    You may ignore this step if you have already a workspace available and jump to `setup step 2` with your own workspace directory
    ```
    $ mkdir -p ~/colcon_ws/src
    $ cd ~/colcon_ws/src
    ``
3. Setup Open Manipulator X ROS2 packages:
    ```
    git clone https://github.com/hylander2126/OpenManipulatorX_ROS2.git
    ```
4. Clone git repo in your ROS workspace
    ```
    $ git clone git@github.com:aligolestaneh/RBE500-FinalProject.git
    $ cd ~/colcon_ws
    $ rosdep install --from-paths src --ignore-src -r -y
    $ colcon build --symlink-install
    ```
# Code Overview
## Packages
* ### rbe500_final_project_msgs
    It has custom srv and msgs  definations required for the `rbe500_final_project` pkg. Currently it has a custom service defination to get the `joint_angles` from Inverse kinematics node.

* ### rbe500_final_project
    A ROS2 package having all required nodes for the final project.

    #### Core Libraries
    
    * `manipulator_core_lib`:
    A C++ library independent of ROS2 which takes care of Forward and Inverse kinematics of the Open manipulator X. The DH parameters is hardcoded here but you can setup the link_lengths independently.

    * `manipulator_fk_ros_lib`: 
    A ROS2 wrapper library which utlizes the `manipulator_core_lib` to implement forward kinematics.

    * `manipulator_ik_ros_lib`: 
    A ROS2 wrapper library which utlizes the `manipulator_core_lib` to implement inverse kinematics.

    * `manipulator_follow_actions_lib`: 
    A ROS2 wrapper library which reads the action sequence from the `manipulator_actions.yaml` and utlizes the inverse kinematics `/get_joint_angles_ik` to get the `joint_angles`. It further use the `OpenManipulatorX` services: `/goal_joint_space_path` to achieve the  calculated joint_angles and `/goal_tool_control` service to open and close gripper.

    #### Parameters
    * `manipulator_core_pramas.yaml`:
        1. `ik_max_iteration` : 
            * Type: Integer
            * Decription: Max interation to do for Newton Raphson Loop

        2. `ik_tolerance`:
            * Type: Double
            * Decription: Epsilon value to check convergance of Newton raphson method

        3. `use_newton_raphson_ik`:
            * Type: Bool
            * Decription: Set `TRUE` to use Netwon Raphson method ofr INverse kinematics.

        4. `manipulator_links`:
            * Type: List of String
            * Decription: Link names to be used for setting the DH params. These names act as the parameter names of link_lengths vector. For ex: If LinkA has been mentioned , it will only read the link length parameter from the `link_lengths` paramter.
        
        5. `link_lengths`:
            * Type: List of Paramters
            * Decription: List of link_length parameters, each having its name and Double type value

# Running Nodes
## Forward Kinematics:
1. Launch the OpenX Manipulator control
```
source ~/colcon_ws/install/setup.bash
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
```
2. Launch Forward Kinematics Node:
```
source ~/colcon_ws/install/setup.bash
ros2 launch rbe500_final_project manipulator_fk_ros.launch.py
```
3. Check End effector pose:
```
ros2 topic echo /end_effector_pose
```
## Inverse Kinematics:
1. Launch the OpenX Manipulator control
```
source ~/colcon_ws/install/setup.bash
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
```
2. Launch Inverse Kinematics Node:
```
source ~/colcon_ws/install/setup.bash
ros2 launch rbe500_final_project manipulator_ik_ros.launch.py
```
3. Get Joint Angles:
```
source ~/colcon_ws/install/setup.bash
$ ros2 service call /get_joint_angles_ik rbe500_final_project_msgs/srv/GetJointAngles "{end_effector_pose: {position: {x: 0.1, y: 0.2, z: 0.3}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```
## Pick and Place Action sequence:
1. Launch the OpenX Manipulator control
```
source ~/colcon_ws/install/setup.bash
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
```
2. Launch Inverse Kinematics Node:
```
source ~/colcon_ws/install/setup.bash
ros2 launch rbe500_final_project manipulator_ik_ros.launch.py
```
3. Execute Pick and Place action sequence
```
source ~/colcon_ws/install/setup.bash
$ ros2 run rbe500_final_project manipulator_follow_actions
```


# Writing Test Cases 
## CMakelist Edits
All test cases has been written up in the test folder. You may add your test cases aswell by creating your own test `cpp` file or editing the existing ones. Please don't edit `main.cpp`. Follow these steps if you create your own test files, then make sure you add it into `CMakelist.txt`
```
set(TESTFILES
  test/main.cpp
  test/simple_test.cpp
  test/<your_test_file_name>_test.cpp) #<<------ add here

```
## Compiling and Test case results

In order to view the test cases results and you need to build the package first:
```
$ cd <your-workspace> 
$ colcon build --symlink-install --packages-select rbe500_final_project
```
Now you can build the test cases:
1. Compiling without test cases results:
    ```
    $ colcon test --packages-select rbe500_final_project
    ```
2. Compiling the test cases along with result verbose:
    ```
    colcon test --packages-select rbe500_final_project --event-handler=console_direct+
    ```

