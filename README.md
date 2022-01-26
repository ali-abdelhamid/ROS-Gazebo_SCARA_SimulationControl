# ROS-Gazebo_SCARA_SimulationControl
ROS Package for simulating a Selective Compliance Articulated Robotic Arm (SCARA) with built in ROS topics for commanding End Effector position and monitioring joint angles


![jhkj](https://user-images.githubusercontent.com/44533559/151183809-f2cab475-f284-4656-9820-c8d01b9794f7.png)



**Introduction**
	Pairing Gazebo’s powerful simulation engine with the robust ROS communication protocol offers endless possibilities in terms of robot design and control by allowing for a modular interaction infrastructure with the simulated robot. And while we are just scratching the surface, in this report, we aim to outline and describe our first attempt of simulating a given SCARA manipulator: from handwritten derivations to executed code.


**Contents**
  1. Assumptions
  2. Instructions for running ROS package
  3. Launching the Robot in Gazebo
    1. Build URDF file for Robot (using existing “world” file from Gazebo worlds)
    2. Create Launch file
    3. Publisher/Subscriber Node Pseudo Code:
      1. Read Current Joint Values
      2. Derive Forward Kinematics (Denavit-Hartenberg Approach)
      3. Calculate Quaternions (From Euler Angles)
  4. Service Server-Server Client Node Pseudo Code
  5. Appendix
    
    

**Assumptions**
* Python libraries such as Numpy and Math are already installed and added as package dependencies
* By “pose” the assignment refers to the geometry_msgs “Pose” message which is part position and part orientation (in quaternions)
* Assigned arbitrary values to physical constants: link size/mass/inertia and gravity. These are all assigned at the start of the script and can be adjusted, all the math is done with variables
* Only requested code to append to the report is the robot definition file. The full scripts for the publisher/subscriber and server/client nodes are included in the package. In this report, we focus on describing the theory behind the script implementation and on proving that we understand what the nodes should be doing and how we can use different ROS commands in the terminal to test our node performance



**Instructions for Running ROS Package:**
  1. Extract zip folder submission and paste the extracted “rbe500_project” folder in your catkin_ws
  2. Open Terminal
  3. Run "roscore" to run Master Node (preset)
  4. Open New Terminal
  5. Run "cd catkin_ws" to access workspace (preset)
  6. Run "catkin_make" to build our rbe500_project package
  7. Run "soruce devel/setup.bash" to update changes
  8. Run "roslaunch rbe500_project spawn.launch" to run launch file
  9. Open New Terminal
  10. Run “rosrun rbe500_project robot_FK.py” to run the FK node
  11. Open New Terminal
  12. Run “rostopic echo -n 1 Pose1” to check what is being published to the Pose1 topic
  13. To confirm that the node is working “live”
  14. Use “rostopic echo Pose1” for a real time print of what is being published to the Pose1 topic 



**Launching the Robot in Gazebo**
  1. Create a new package and make the necessary adjustments in the CmakeLists and the package files in order to allow for tools like rospy, message types, and message generation that are required for using existing message types and for allowing cross-platform script execution
  2. Create a URDF file to describe the robot. We followed the Gazebo tutorials as instructed and some of the most important lessons we learned were:
      * The necessity of working with a .xacro file in place of a .urdf file especially when there is a growing number of variables to be used. Xacro format allows for the assigning of variables which makes for a cleaner script
      * The requirement of a “world” link before the first joint in order to assign the world frame as the “parent” of the first joint. This allows a non-fixed joint to move about a fixed world axis without leaving the origin
      * The visual and collision pills in a URDF file may not necessarily have the same geometries: this is so that very complicated shapes can be considered in collision calculations but does not have to be fully displayed in the simulation (the more complicated the shapes, the more computing power required to display)
      * We can use the same materials.xacro file as used in the Gazebo tutorials (as with the world file) as long as it is located in the correct folder (urdf) and is called properly in the launch file
      * Transmission pills can be used to assign joints for both reading their current state via Joint_State_Publisher and assigning effort values to the joint via Joint_State_controller. For this part of the assignment, we are only concerned with the joint states publisher.
  3. Before creating a launch file, we create a launch folder to keep such files. In addition to the common setting of gazebo parameters (which is also not required to run Gazebo) the launch file is fairly straightforward:
      * Loads the URDF (.xacro) file into the ROS Parameter server (has to find the appropriate .xacro file in the urdf folder in the said package
      * Launches the robot_state_publisher node which publishes joint state values that we can then subscribe to within our FK publisher/subscriber
      * Assigns the .yaml configuration file where joint state publisher and controller nodes can be set up. The controller in our configuration file was only used to test the robot and is not intended as a submission attempt for part 2 of the project (PID control).



**Publisher/Subscriber Pseudo Code**
	The subscriber section of the script follows the same format of the ROS tutorials where the subscriber is declared in the section defining the node that will be running the .py file. However, the publisher is declared outside of the node definition but the publishing command itself is implemented within the callback function. This way, the output from the callback function is published to the specified topic every single time the callback function is triggered. What triggers this callback function is a new “message” published by Gazebo joint_state_publisher to the “/simple_model/joint_states” topic as defined in the .launch and .yaml files. Every time a new set of joint states is published by Gazebo to the topic, our node that is subscribed to the same topic takes the message in the topic and feeds it to the callback function which completes the necessary calculations and then publishes a Pose message to a separate topic. The rest of the code is very simple:
  1. Construct robot in home position and derive DH parameters:
  2. Derive A1, A2, and A3 matrices and then multiply them to get final homogeneous transformation matrix (shown in robot_FK.py script). Note that these matrices are populated with variables in place of the joint values (θ)
  3. In order to provide the values for the joints, we go back to our subscriber and dissect the message to get the values that we want. Note that joint_state_publisher has different fields, we only want the fields that correspond to our 3 joint values so we extract them individually
  4. With the live joint values, we are now ready to calculate our numerical homogeneous transformation matrix
  5. Seeing as the requirement is to publish a Pose message, we also included lines that first use the rotation matrix (top left 3x3 corner of HT) to convert to Euler angles which we can then use to calculate the corresponding angles in quaternions 
  6. Now that we have our position vector from the HT and the quaternion angles: we have all we need to build the Pose message before attempting to publish it. Note in the script how the variable carrying the Pose message is first declared as a Pose type and then populated. 
  7. The final line in the callback function is the publish command to send the calculated Pose message to the topic declared in the “rospy.Publisher...” line
