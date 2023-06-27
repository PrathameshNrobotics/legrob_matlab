# legrob_matlab
matlab files for the biped locomotion

Line 10 - Line 35:
This for-loop starts up the motion and gets the right leg in swing phase. This loop has to be separate since starting leg will execute half motion compared to the rest of the walking motion. 

x-axis: motion in forward direction

y-axis: motion in verticle direction. You will have to modify the axis according to the simulation

The foot in the swing phase follows a semi-circular trajectory and the leg in stance phase adjusts according to the CoM shift. 

Line 14 - Line 16: denotes the intermediate equations for inverse kinematics.
theta1, p, and theta2 are the required inverse kinematic equations. 

theta1: roll angle (revolute joint in the forward direction) of the swing leg. In this case, the right leg

theta2: roll angle (revolute joint in the forward direction) of the stance leg. In this case, left leg

p: prismatic joint movement of the swing leg. In this case, the right leg.


Line 19- Line 23: denote the 2 cases for theta2 depending on theta1


Line 25 - Line 34: is just the plotting function for MATLAB. You do not need to implement it in pybullet. You just need to send the theta1, theta2, and p values to the respective joints in pybullet.


Line 40 - Line 97:
This block has 2 for-loops nested in another for-loop. This block gets the robot in walking motion after the first leg (right leg here) has been set. Now since we started with the right leg initially, this nested for-loop block will start with the left leg, followed by the right leg, and the sequence continues. 
Variable iter: denotes the number of iterations the walking pattern will execute for. It is currently set to 4*lim. Changing the ‘4’ to any number of your choice will increase the number of gait cycles executed, in short, will help the simulation run for a long. You can even set it as an infinite while loop. Just make sure ou adjust the parameter accordingly. 


Line 45 - Line 68:
This block denotes the left leg movement and is framed exactly the same way as the block (Line 10 - Line 35)


Line 72 - Line 95:
This block denotes the right leg movement and is framed exactly the same way as the block (Line 10 - Line 35)

