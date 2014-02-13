This repo is to contain the code for development experiments that we do for
Krang. The following experiments have taken place so far:

1) Kinematics: Testing the Kinect output against the forward kinematics
2) Balancing: Getting the center of mass model and the balancing 
3) Current control: Implementing current control on the arm towards more smooth
   compliant behavior and safer/faster two hand manipulation.
4) Grippers: Opening and closing grippers fully, ability to squeeze until 
	 a current limit
5) Manipulation: The two hands move (independently) to take objects from
   a desk, from each other and place them. Vision is incorporated.
6) TwoHands: The two hands manipulate objects at the same time which induces
   closure problems and must be handled with task constraints or compliance.
7) Teleop: The development code for using liberty and having a full interface
   with the Logitech joystick. Once done, we will move to demos.
8) Navigation: Development of autonomous navigation using encoder values and
   vision data.
