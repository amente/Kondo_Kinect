using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media.Media3D;


namespace Kondo_Kinect
{
    /// <summary>
    /// This class allows us to send commands to the robot based 
    /// on the joint positions of the person standing infront of the Kinect sensor
    /// </summary>
    class RobotController
    {
        private KinectController kinectController;
        private CommandSender commandSender;

        private bool isTracking = false;

        private List<RobotJoint> robotJoints;

        /// <summary>
        /// Describes a robot joint defined by three human joints
        /// </summary>
        private class RobotJoint
        {
            private JointType j1, j2, j3;
            private string name;
            private int channel;

            private int jointMin = 0;
            private int jointMax = 90;

            private int totalDegrees = 90;


            public RobotJoint(string name,JointType j1,JointType j2,JointType j3,int channel){
                this.j1 = j1;
                this.j2 = j2;
                this.j3 = j3;
                this.name = name;
                this.channel = channel;
            }

            public int TotalDegrees
            {
                get
                {
                    return totalDegrees;
                }
                set
                {
                    this.totalDegrees = value;
                }
            }

            public int JointMin
            {
                get
                {
                    return jointMin;
                }
                set
                {
                    this.jointMin = value;
                }
            }

            public int JointMax
            {
                get
                {
                    return jointMax;
                }
                set
                {
                    this.jointMax = value;
                }
            }

            public string Name
            {
                get
                {
                    return name;
                }
            }

            public JointType J3
            {
                get
                {
                    return j1; 
                }
            }

            public JointType J1
            {
                get
                {
                    return j2;
                }
            }
            public JointType J2
            {
                get
                {
                    return j3;
                }
            }
            public int Channel
            {
                get
                {
                    return channel;
                }
            }
        }

        /// <summary>
        /// Constructor for the singleton class
        /// </summary>
        public RobotController()
        {
            this.kinectController = KinectController.Instance;
            this.commandSender = CommandSender.Instance;
            this.isTracking = false;
            this.commandSender.SerialPortStatusChanged += serialPortStatusChanged;
            defineJoints();
        }

        private void serialPortStatusChanged(bool isOpen)
        {
            if(isOpen){
                start();
            }else{
                stop();
            }
        }

        private static RobotController robotController;

        // Used for sysnchronizing singleton instance during instantiation
        private static object syncRoot = new Object();

        /// <summary>
        /// The singleton instance
        /// </summary>
        public static RobotController Instance
        {
            get
            {
                // use a lock for additional thread safety
                lock (syncRoot)
                {
                    if (robotController == null)
                    {
                        robotController = new RobotController();
                    }
                }
                return robotController;
            }
        }

        /// <summary>
        /// Creates instances for joints on the robot that are mapped to the body
        /// </summary>
        private void defineJoints()
        {
            // A robot joint describes a controlling point for one servo on the robot
            this.robotJoints = new List<RobotJoint>();
            
            // Right Arm
            RobotJoint rightArmJoint = new RobotJoint("Right Arm",JointType.ShoulderRight, JointType.ElbowRight,JointType.WristRight,1);
            rightArmJoint.JointMin = -25;
            rightArmJoint.JointMax = 60;
            rightArmJoint.TotalDegrees = 180;
            this.robotJoints.Add(rightArmJoint);
           

            // Right Sholder
            RobotJoint rightShoulderJoint = new RobotJoint("Right Sholder", JointType.SpineShoulder, JointType.ShoulderRight, JointType.ElbowRight, 2);
            rightShoulderJoint.JointMin = -25;
            rightShoulderJoint.JointMax = 25;
            rightShoulderJoint.TotalDegrees = 180;
            this.robotJoints.Add(rightShoulderJoint);
            
           
            // Left Arm
            RobotJoint leftArmJoint = new RobotJoint("Left Arm", JointType.ShoulderLeft, JointType.ElbowLeft, JointType.WristLeft, 16);
            leftArmJoint.JointMin = -60;
            leftArmJoint.JointMax = 25;
            leftArmJoint.TotalDegrees = 180;
            this.robotJoints.Add(leftArmJoint);

            // Left Sholder
            RobotJoint leftShoudlerJoint = new RobotJoint("Left Sholder", JointType.SpineShoulder, JointType.ShoulderLeft, JointType.ElbowLeft, 15);
            leftShoudlerJoint.JointMin = -25;
            leftShoudlerJoint.JointMax = 25;
            leftShoudlerJoint.TotalDegrees = 180;
            this.robotJoints.Add(leftShoudlerJoint);           
            
        }



        /// <summary>
        /// This method starts the controller
        /// </summary>
        public void start()
        {
            if (!isTracking)
            {
                this.kinectController.MainBodyAvailable += BodyAvailable;
                this.isTracking = true;
            }

        }

        /// <summary>
        /// This method stops the controller
        /// </summary>
        public void stop()
        {
            if (isTracking)
            {
                this.kinectController.MainBodyAvailable -= BodyAvailable;
                this.isTracking = false;
            }
        }

        /// <summary>
        /// This is called when a body data is available from the kinect interface
        /// </summary>
        /// <param name="body"></param>
        private void BodyAvailable(Body body)
        {
            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

            // convert the joint points to depth (display) space
            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

            foreach (JointType jointType in joints.Keys)
            {
                // sometimes the depth(Z) of an inferred joint may show as negative
                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                CameraSpacePoint position = joints[jointType].Position;
                if (position.Z < 0)
                {
                    position.Z = 0.1f;
                }

                DepthSpacePoint depthSpacePoint = kinectController.CoordinateMapper.MapCameraPointToDepthSpace(position);
                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
            }

            //If both hands are closed, then calculate angles and send commands to the robot
            if (body.HandLeftState == HandState.Closed && body.HandRightState == HandState.Closed)
            {

                foreach (RobotJoint robotJoint in robotJoints)
                {
                        int angle = Convert.ToInt16(calculateAngle(jointPoints[robotJoint.J1], jointPoints[robotJoint.J2], jointPoints[robotJoint.J3]));
                        setServoAngle(robotJoint, angle);                          
                }
            }
        }

        private void setServoAngle(RobotJoint joint,int angle){
            Console.Write(joint.Name + "  :");
            Console.Write(" Raw: " + angle);
            int channel = joint.Channel;
            if (angle < joint.JointMin)
            {
                angle = joint.JointMin;
            }
            else if (angle > joint.JointMax)
            {
                angle = joint.JointMax;
            }

            angle =(int)(((angle-joint.JointMin) * 1.0 / (joint.JointMax - joint.JointMin)) * joint.TotalDegrees);
                        
            Console.WriteLine(" Adjusted: "+angle);

            setServoAngle(joint.Channel, angle);
        }

        public void setServoAngle(int channel, int angle)
        {
            Command cmd = new Command(1, new byte[] { 0xFE, 0x03, 0x01, (byte)channel, (byte)angle }, false);
            CommandSender.Send(cmd);
        }

        /// <summary>
        /// This method calculates angle between three points
        /// </summary>
        /// <param name="j1"></param>
        /// <param name="j2"></param>
        /// <param name="j3"></param>
        /// <returns></returns>
        private double calculateAngle(Point j1, Point j2, Point j3)
        {
            Vector a1 = new Vector(j1.X,j1.Y);
            Vector a2 = new Vector(j2.X, j2.Y);
            Vector a3 = new Vector(j3.X, j3.Y);

            
            Vector b1 = a2 - a1;
            Vector b2 = a2 - a3;                          

            return Vector.AngleBetween(b1,b2); 
        }


    }
}
