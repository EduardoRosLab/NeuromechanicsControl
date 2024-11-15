# NeuromechanicsControl

Cerebello-muscular torque control of a Baxter robot with variable cocontraction to adjust the robot motor behavior.
The repository includes the EDLUT simulator, the ROS packages to perform a closed-loop cerebello-muscular torque control and the configuration files needed.


###### Work published in:
Abadía, I., Bruel, A., Courtine, G., Ijspeert, A., Ros, E., & Luque, N. R. A neuromechanics solution for adjustable robot compliance and accuracy. Science Robotics
****


## Requirements
This project was developed and tested using ROS Kinetic running in Ubuntu 16.
Due to computational constraints the project differentiates between a controller PC running the cerebellar SNN and all control nodes, and a monitor PC running the monitoring tools.
The controller PC had the following characteristics:
* RAM 24GB
* Intel® Core™ i7-5820K CPU @ 3.30GHz × 12
* GeForce RTX 2080/PCIe/SSE2

The project assumes Baxter SDK is installed at ~/ros_ws


## Installation
In the controller PC:

* Install EDLUT simulator (this step requires an NVIDIA GPU with CUDA support and CUDA installation). Open a terminal and go to EDLUT folder:

```
$ cd ~/NeuromechanicsControl/EDLUT
$ chmod u+x configure
$ ./configure
$ make
$ sudo bash
$ make install
```

* Compile the control ROS package (this step requires the installation of ROS and proper Baxter configuration)

```
$ cd ros_ws
$ ./baxter.sh
$ cd ~/NeuromechanicsControl/ROS-packages/control
$ catkin_make
```

* Copy the content from folder NeuromechanicsControl/neuron_models to ~/.ros/data .


In the monitor PC:

* Compile the monitoring ROS package (this step requires the installation of ROS and proper Baxter configuration)

```
$ cd ros_ws
$ ./baxter.sh
$ cd ~/NeuromechanicsControl/ROS-packages/monitoring
$ catkin_make
```

* Edit the file monitoring/devel/env.sh to include ROS IP and DISPLAY variables. An example of the configuration is provided at /ROS-packages/monitoring/src/neuromechanics_control/env-loader/env.sh

Configure an ssh connection between the controller and monitoring PCs. The configuration used in this project assumes the following IPs: Baxter 192.168.2.1; Control PC 192.168.2.2; Monitor PC 192.168.2.3

## Execution
* In the Controller PC:
```
$ cd ros_ws
$ ./baxter.sh
$ rostopic pub /robot/joint_state_publish_rate std_msgs/UInt16 500
$ cd ~/NeuromechanicsControl/ROS-packages/control
$ source devel/setup.bash
$ roslaunch neuromechanics_control experiment.launch
```
* Substitute "experiment.launch" with the launch file of the corresponding experiment. In NeuromechanicsControl/ROS-packages/control/src/neuromechanics_control/launch you can find the launch files of the different experiments. The project includes three families of motor tasks (eight-like trajectory in the horizontal plane, circular trajectory in the vertical plane, T-shaped trajectory in the horizontal plane) performed at different speeds. The  following control modes are included:
	* Cerebello-muscular torque control
	* PD-muscular torque control
  * Direct PD torque control
  * Factory-default position control

* The cocontraction value can be changed within the launch file: Go to the parameters of the node "cocontraction_profile_publisher" and modify the value of param "cocontraction":
```
  <param name="cocontraction" value="0.0" type="double" />
```

* Active modulation of the cocontraction level is also provided in "active_cocontraction_t_trajectory_10s.launch". The control mode can change between:
  * Compliance priority mode
  * Robustness priority mode
  * Frontal robustness & Lateral compliance
  * Lateral robustness & Frontal compliance

To change the control mode, adjust the weighted trade-off between compliance and robustness by changing the value of:
```
  <rosparam param="w_compliance_robustness">
```

* To use the monitoring tools run the rqt_reconfigure ROS plugin in the Monitoring PC:

```
$ cd ros_ws
$ ./baxter.sh
$ $ rosrun rqt_reconfigure rqt_reconfigure
```
A new window will be displayed allowing the user to choose among different monitoring tools.

* When the experiment finishes the cerebellar synaptic weight distribution acquired as a result of the motor adaptation process is automatically saved at ~/.ros/data with file name "output_weightXXX.dat"
You can use that generated file as the input synaptic weight file for EDLUT simulator at the beginning of another experiment so that the experiment starts with the already acquired cerebellar synaptic weight distribution. To do so, within the launch file go to the parameters of the node "simulator_node" and modify the value of param "weight file" so that it points to the desired synaptic weight distribution file:
```
    <param name="weight_file" value="output_weightXXX.dat" type="str" />
```
* The experiment data is saved in a rosbag file at /NeuromechanicsControl/ROS-packages/control/src/neuromechanics_control/rosbags
