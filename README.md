# dm_arm_control
Hardware control package for Diymore 6dof arm including driver and Inverse kinematics service. 
Arm available here:
https://www.amazon.com/diymore-Aluminium-Mechanical-Robotic-Arduino/dp/B01LW0LUPT/ref=sr_1_1?keywords=diymore+arm&qid=1639005644&sr=8-1

Arm controller takes input in Pose msg format and returns success or failure. 

note: wrote this mainly to get more experience writing Inverse Kinematics. Plan on using for some light tasks. For heavier applications I'd recommend a different arm, or replacing some of the servos with stronger ones as arm cannot support much weight. 
