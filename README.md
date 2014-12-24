Kondo_Kinect
============

For documentation of the project visit: http://amente.github.io/projects/KondoKinect/index.html

The user software is implemented in C#, using Microsoft .NET Framework 4.5. For the
user interface, we used the Windows Presentation Framework (WPF). The
text-box and slider use dynamic data binding with GUI control. The source code can be broken
down to three major parts: CommandSender.cs, KinectController.cs, and RobotController.cs.

The CommandSender.cs is a series of classes dealing with command sending functions in
our system. It parses commands and communicate with the robot through serial ports. Basically
it develops the protocols for the system.

KinectController.cs has couple major functionalities like acquiring frame data from the
Kinect, defining the joints and bones for the body, and drawing the bones on the camera feed.
Basically it contains all the functions that involves Kinect sensor.

RobotController.cs is the implementation of the mapping logic for arm movements.  It
relates the four servo motors on the robot to the corresponding joints on the body that is
currently tracked by Kinect. After receiving the location of each joint from the sensor, it
translates the 2D vectors into degrees and sets the relating servo angles into the translated
angles.
