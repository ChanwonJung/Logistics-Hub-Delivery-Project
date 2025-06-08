# Coupang Logistics Hub Delivery Project

** Among the above folders, the "최종" folder is the final project source code. **.

## 1. Project Overview
- The demand for cooperative robots is increasing due to the risk of injury and the burden of labor costs during logistics work.
- It aims to reduce the risk and burden that humans can feel by moving heavy products using manipulators.
- In other words, the Doosan manipulator M0609 model is used to build a logistics automation system to enable hourly work.
- Through this, it is possible to improve the efficiency of corporate productivity, prevent industrial safety accidents, and reduce errors.



## 2. Utilization equipment and development environment
- Doosan Manipulator M0609
- ROS2 humble(ubuntu 22.04)
- Lego blocks
- OnRobot2 Gripper



## 3. Project progress
![image](https://github.com/user-attachments/assets/72acfe90-bd1c-4ed1-a3f0-fef858b7fb4a)
The size of the product was classified according to the width of the gripper when the object was held using the weblogic within the gripper manufacturer onrobot.

![image](https://github.com/user-attachments/assets/7764861f-1d61-4cb6-a121-78ae0638296f)


<Algebraic flow chart>: Store blocks from factory to hub -> Load blocks in the hub area -> Ship blocks when an order is placed from the consumer


a. Block warehousing: When the block approaches with a conveyor belt, the manipulator descends to the z-axis and grabs the object. Then, the block is loaded into the hub through compliance control and external force with task_compliance_ctrl(), set_desired_force().


b. Block loading: When loading blocks, the external force is set to be recognized so that the Lego can fit into the hole well. After that, when the block is stabilized, it is considered a success and the manipulator moves to the initial coordinates. It is set to be able to stack up to 3 floors at a time for each block size, and if the blocks to be loaded are stored even though they are all stacked within the allowable hub area for each size, it is set to load up to 4 blocks regardless of the size of the overflow area.


![image](https://github.com/user-attachments/assets/020dd59c-7ca9-48b7-9e8d-478843161db8)
It can be seen that an external force is instantaneously generated in the z-axis when the block is loaded.

![image](https://github.com/user-attachments/assets/4bd6861c-61c8-473a-b3e4-da4e09364a2f)

<img width="669" alt="image" src="https://github.com/user-attachments/assets/57522522-255f-48f8-bab9-ebbec209afdd" />


c. Block shipment: When an order comes from a consumer and the product in the hub needs to be shipped, the gripper grabs the product, twists it in the Ry axis, and goes up the z axis. After that, the product is placed as the shipping coordinates and then moved back to the initial coordinates.


d. Defective product handling: If a product is recognized as a defective product when picked up with a gripper from a conveyor belt, it is stored separately in its own area that is treated as a defective product. After that, the height of the defective products piled in the defective product area was measured with get_current_posx()[2], and the move_periodic() function was used so that the defective products could be accumulated a little flat.



## 4. Errors experienced during the project
a. We tried to use the PyModBus Protocol, which communicates the width of the gripper in real time when an onrobot gripper, not a weblogic, picked up the object, but this was not possible due to the nature of the I/O controller.

https://github.com/ABC-iRobotics/onrobot-ros2/tree/main/onrobot_rg_control/onrobot_rg_control

b. The event of the hardware itself was rejected by using the set_desired_force() external command with the move().



## 5. Demo

https://drive.google.com/file/d/1xDryZHY2w0bPtmrQmrJiEwY50bOkdlcB/view?usp=drive_link
