# Cassie Controller

The objective of this repository is to contain controllers for the Cassie robot written both in Python and in C++. First iteration will be in Python to speed up development, then the code should be ported to C++. I don't know yet exactly which structure of controller I should use, but I am constantly reading papers about various controllers implemented in the robot and I hope to create some controller that is inspired by these while also applying my previous knowledge. Previously, I designed a controller for the quadruped Solo12 consisting of a MPC as a centroidal controller, a task-based inverse dynamics controller as a whole-body controller and a stabilizer to provide correction wrenches to the robot, and I would like to use some of these techniques in a controller for Cassie as well.

The main reason for all of this is to exercise my robotics / control theory / simulation knowledge and C++ programming capabilities.

Many thanks to https://github.com/bulletphysics/pybullet_robots/ from where I obtained the Cassie model. 