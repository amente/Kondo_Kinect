using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Drawing;

namespace Kondo_Kinect
{
    /// <summary>
    /// A delegate for method called when a body is available from the sensor 
    /// </summary>
    /// <param name="body"></param>
    public delegate void mainBodyAvailableHandler(Body body);
    
    /// <summary>
    // A delegate for method called when the status of the kinect sensor changes i.e When Turned ON or OFF
    /// </summary>
    /// <param name="isAvailable"></param>
    public delegate void sensorStatusChanged(Boolean isAvailable);

    /// <summary>
    /// A class for interacting with the kinect sensor
    /// </summary>
    class KinectController
    {
        /// <summary>
        /// Called when a body is being tracked infront of the sensor
        /// </summary>
        public event mainBodyAvailableHandler MainBodyAvailable;

        /// <summary>
        /// Called when the status of the sensor changes
        /// </summary>
        public event sensorStatusChanged SensorStatusChanged;

        /// <summary>
        /// Active kinect sensor, only one sensor is supported
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;


        /// <summary>
        /// Array for the bodies tracked by the sensor
        /// </summary>
        private Body[] bodies = null;


        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;


        /// <summary>
        /// The body of main person. (Used for controlling the Robot) Kinect has the ability to track up to 6 people
        /// We support only one person to controll the robot
        /// </summary>
        private Body mainBody = null;

       
        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// A flag for the sensor status
        /// </summary>
        private bool sensorAvailable;

        private static object syncRoot = new Object();

        private static volatile KinectController kinectController;

        public static KinectController Instance
        {
            get
            {
                // use a lock for additional thread safety
                lock (syncRoot)
                {
                    if (kinectController == null)
                    {
                        kinectController = new KinectController();
                    }
                }
                return kinectController;
            }
        }

        public KinectController()
        {
            // The Kinect API allows to use multiple sensors, we use just the default one
            this.kinectSensor = KinectSensor.GetDefault();

            // Get the coordinate mapper for the Kinect Sensor
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // Describes the display extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // Gets the size of the display space
            this.displayHeight = frameDescription.Height;
            this.displayWidth = frameDescription.Width;

            // Opens the reader instance for body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;

            defineBones();
          
            // Sent the available changed event notifier for the sensor
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            //Open the sensor
            this.kinectSensor.Open();

            //Set the status of the sensor
            this.sensorAvailable = this.kinectSensor.IsAvailable;

        }

        private void defineBones()
        {
            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            if (SensorStatusChanged != null)
            {
                this.sensorAvailable = this.kinectSensor.IsAvailable;
                SensorStatusChanged(this.sensorAvailable);
            }            
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // "The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used." 
                    bodyFrame.GetAndRefreshBodyData(this.bodies);                

                    dataReceived = true;
                }
                 
            }

            if (dataReceived)
            {
                foreach (Body body in this.bodies)
                {
                    if (body.IsTracked)
                    {
                        // We pick the first body in the array to be the main body
                        mainBody = body;
                        // Data has been recieved and main body detected notify listeners 
                        OnMainBodyAvailable(mainBody);                        
                        break;
                    }
                }
              
            }
        }

        public List<Tuple<JointType, JointType>> Bones
        {
            get
            {
                return this.bones;
            }
        }

        /// <summary>
        /// Called when a main body is available
        /// </summary>
        /// <param name="body"></param>
        private void OnMainBodyAvailable(Body body)
        {
            if(MainBodyAvailable != null){
                MainBodyAvailable(body);
            }
        }

        /// <summary>
        /// Indicates the status of the sensor
        /// </summary>
        public bool SensorIsAvailable
        {
            get{
                return sensorAvailable;
            }
        }

        /// <summary>
        /// Returns the coordinate mapper for the Kinect Sensor
        /// </summary>
        public CoordinateMapper CoordinateMapper
        {
            get
            {
                return coordinateMapper;
            }
        }

        /// <summary>
        /// Returns the display height for the camera window
        /// </summary>
        public int DisplayHeight
        {
            get
            {
                return displayHeight;
            }
        }

        /// <summary>
        /// Returns the display width for the cameram window
        /// </summary>
        public int DisplayWidth
        {
            get
            {
                return displayWidth;
            }
        }

        /// <summary>
        /// Closes the controller
        /// </summary>
        public void Close(){
            this.kinectSensor.Close();
        }
    }
}
