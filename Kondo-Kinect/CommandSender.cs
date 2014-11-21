using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Timers;

namespace Kondo_Kinect
{
   // public delegate void cmdResponseAvailableHandler(CommandResponse response);
    public delegate void serialPortStatusChangedHandler(bool isOpen);


    class CommandResponse
    {
        public CommandResponse()
        {

        }

        public static CommandResponse TimeoutResponse { get; set; }
    }


    
    class Command
    {

        int responseLength;
        byte[] bytes;
        bool expectsAnswer;


        public Command(int responseLength, byte[] bytes,bool expectsAnswer)
        {
            this.responseLength = responseLength;
            this.bytes = bytes;
            this.expectsAnswer = false;
        }


        public byte[] Bytes
        {
            get
            {
                return bytes;
            }
        }

        public int ResponseLength
        {
            get
            {
                return responseLength;
            }
        }

        public CommandResponse parse(StringBuilder responseString)
        {
            return new CommandResponse();
        }

       

        public bool ExpectsAnswer { get; set; }
    }


    class CommandSender
    {
        private static volatile CommandSender sender;
        private static object syncRoot = new Object();

        // Timer used for properly timing out ACK waits 
        private static Timer timer;
        private static int ACK_TIMEOUT = 3000;

        private static SerialPort serialPort;

        //public  event cmdResponseAvailableHandler CmdResponseAvailable;
        public event serialPortStatusChangedHandler SerialPortStatusChanged;

       
        private CommandSender() {
            serialPort = new SerialPort();          
            timer = new Timer(ACK_TIMEOUT);
            timer.AutoReset = false;
        }

        public static CommandSender Instance
        {
            get
            {
                // use a lock for additional thread safety
                lock (syncRoot)
                {
                    if (sender == null)
                    {
                        sender = new CommandSender();                       
                    }
                }
                return sender;
            }
        }

        private void OnSerialPortStatusChanged(bool isOpen)
        {
            if (SerialPortStatusChanged != null)
            {
                SerialPortStatusChanged(isOpen);
            }
        }

        public void closeSerialPort()
        {
            if (serialPort.IsOpen)
            {
                serialPort.Close();
            }
            OnSerialPortStatusChanged(false);
        }

        public bool openSerialPort(String name, int baud)
        {
            if (!serialPort.IsOpen)
            {
                serialPort.PortName = name;
                serialPort.BaudRate = baud;
                try
                {
                    serialPort.Open();
                    OnSerialPortStatusChanged(true);
                    return true;
                }
                catch (Exception e)
                {
                    Console.WriteLine(e.Message);
                }
            }

            return false;
        }

       /* private static void OnCmdResponseAvailable(CommandResponse response)
        {
            if (CmdResponseAvailable != null)
            {
                CmdResponseAvailable(response);
            }
        }*/

        public static void Send(Command cmd)
        {
            if (!serialPort.IsOpen) { return; }
            
            bool gotAnswer = false;
            StringBuilder responseString = new StringBuilder();
            SerialDataReceivedEventHandler handler = delegate(object sender, SerialDataReceivedEventArgs e)
            {
                if (!serialPort.IsOpen) { return; }

                byte[] smallBuffer = new byte[serialPort.BytesToRead];
                int recvBytes = serialPort.Read(smallBuffer, 0, smallBuffer.Length);
                responseString.Append(BitConverter.ToString(smallBuffer).Replace("-", string.Empty));
                if (responseString.Length >= cmd.ResponseLength)
                {
                    CommandResponse response = cmd.parse(responseString);
                    if(response!=null){
                        gotAnswer = true;
                        //OnCmdResponseAvailable(response);
                    }
                }
                
            };

            ElapsedEventHandler ackTimoutHandler  = delegate(Object source,ElapsedEventArgs e){
                gotAnswer = true;
                //OnCmdResponseAvailable(CommandResponse.TimeoutResponse);
            };

          
            timer.Elapsed += ackTimoutHandler;
            serialPort.DataReceived += handler;           
            byte[] cmdBytes = cmd.Bytes;              
            serialPort.Write(cmdBytes, 0, cmdBytes.Length);
            Console.WriteLine("Sent: " + BitConverter.ToString(cmdBytes));
            timer.Start();
            if (cmd.ExpectsAnswer)
            {
                while (!gotAnswer) ;
            }
            serialPort.DataReceived -= handler;
            timer.Elapsed -= ackTimoutHandler;
            
        }

        public bool serialPortIsOPen()
        {
            return serialPort.IsOpen;
        }      

    }
}
