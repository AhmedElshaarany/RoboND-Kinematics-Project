## Project: Kinematics Pick & Place

**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/Joint_Angles_and_Axes.png
[image5]: ./misc_images/Kuka_links.png
[image6]: ./misc_images/dh-transform-matrix.png
[image7]: ./misc_images/euler.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The forward_kinematics demo was used to derive the DH parameters. Below is a screenshot of running the demo.

![alt text][image1]

The joint angles and links are shown in the images below.

![alt text][image4]
![alt text][image5]


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Below is the Modified DH parameters table for the Kuka arm by using the kr210.urdf.xacro.

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  - pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

The individual transformation matrix is used to calculate theoverall transformation matrix for forward kinematics.

Below is the general form used as a function to generate a mutable matrix.

![alt text][image6]

The individual transformation matrices about each joint and the generalized homogeneous transform between base\_link and gripper\_link using the gripper pose are shown in the IK_server.py script in lines (49-68). Below are the simplified individual transformation matrices.

```python
T0_1 = Matrix([[cos(q1), -sin(q1), 0, 0], [sin(q1), cos(q1), 0, 0], [0, 0, 1, 0.750000000000000], [0, 0, 0, 1]])

T1_2 = Matrix([[cos(q2 - 0.5*pi), -sin(q2 - 0.5*pi), 0, 0.350000000000000], [0, 0, 1, 0], [-sin(q2 - 0.5*pi), -cos(q2 - 0.5*pi), 0, 0], [0, 0, 0, 1]])

T2_3 = Matrix([[cos(q3), -sin(q3), 0, 1.25000000000000], [sin(q3), cos(q3), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

T3_4 = Matrix([[cos(q4), -sin(q4), 0, -0.0540000000000000], [0, 0, 1, 1.50000000000000], [-sin(q4), -cos(q4), 0, 0], [0, 0, 0, 1]])

T4_5 = Matrix([[cos(q5), -sin(q5), 0, 0], [0, 0, -1, 0], [sin(q5), cos(q5), 0, 0], [0, 0, 0, 1]])

T5_6 = Matrix([[cos(q6), -sin(q6), 0, 0], [0, 0, 1, 0], [-sin(q6), -cos(q6), 0, 0], [0, 0, 0, 1]])

T6_EE = Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.303000000000000], [0, 0, 0, 1]])

```


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

As shown in the lessons, the last three joints in our robot are revolute. In addition, the joint axes of those last three joints intersect at a single point. Therefore,  we have a spherical wrist with joint_5 being the common intersection point and hence the wrist center.

This enables us to kinematically decouple the Inverse Kinematics problem into inverse position and inverse orientation. We will discuss below how each problem is addressed.

##### Inverse Position Problem

For the inverse position problem, Theta 2, and Theta 3 was solved by using the example from the lessons. The image below shows how they are calculated.

![alt text][image2]

##### Inverse Orientation Problem

This problem involves calculating theta 4, 5, and 6 using the now known theta 1, 2, and 3. These Euler angles can be calculated using the rotation matrix of joints 3 to 6. Below is the equation used to calculated this rotation matirx.

![alt text][image7]

The math for deriving the theta angles for the each joint is shown in the IK_server.py script in lines (121-146).


##### Multiple solutions for theta 4, 5, and 6
For the multiple solutions of theta 4, 5, and 6, the angles that reduce the wrist rotation are selected as shown in IK_server.py lines (159-170).

### Project Implementation

Running the first test case of IK_debug, below are the error values

```python
Total run time to calculate joint angles from pose is 0.5016 seconds

Wrist error for x position is: 0.00000503
Wrist error for y position is: 0.00000512
Wrist error for z position is: 0.00000585
Overall wrist offset is: 0.00000926 units

Theta 1 error is: 0.00136747
Theta 2 error is: 0.00325738
Theta 3 error is: 0.00339563
Theta 4 error is: 6.53212647
Theta 5 error is: 0.39551490
Theta 6 error is: 6.86340402

**These theta errors may not be a correct representation of your code, due to the fact            
that the arm can have muliple positions. It is best to add your forward kinmeatics to            
confirm whether your code is working or not**
 

End effector error for x position is: 0.04253562
End effector error for y position is: 0.03889778
End effector error for z position is: 0.12694560
Overall end effector offset is: 0.13941844 units 

```
The error values are relatively low, but the run time can further be improved by maybe creating a class and calling the class functions whenever needed which would save time.

Below is a screen shot of the Kuka arm on its way to place the object in the bin.

![alt text][image3]


