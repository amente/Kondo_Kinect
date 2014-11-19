using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Kondo_Kinect
{
    public delegate void cmdResponseAvailableHandler(byte[] response);
    public delegate void serialPortStatusChangedHandler(bool isOpen);

    class Command
    {

        public static byte[] moveServo(byte channelNumber, int value)
        {
            byte[] cmd = new byte[] { 0xFE, 0x00, 0x00, 0x01, 0, 0, 0 };
            //byte sum = 0;
            cmd[2] = channelNumber;
            cmd[5] = (byte)(value + 0x4000);
            cmd[4] = (byte)((value + 0x4000) >> 8);
            for (int i = 0; i < cmd.Length - 1; i++)
            {
                cmd[6] += cmd[i];
            }
            return cmd;
        }
    }


    class CommandSender
    {
        private static volatile CommandSender sender;
        private static object syncRoot = new Object();

        private SerialPort serialPort;

        public event cmdResponseAvailableHandler CmdResponseAvailable;
        public event serialPortStatusChangedHandler SerialPortStatusChanged;

       
        private CommandSender() {
            serialPort = new SerialPort();
            serialPort.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);
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


        private void DataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
        {
            if (!serialPort.IsOpen)
            {
                return;
            }            
            byte[] smallBuffer = new byte[serialPort.BytesToRead];
            int recvBytes = serialPort.Read(smallBuffer, 0, smallBuffer.Length);
            OnCmdResponseAvaialbe(smallBuffer);
            Console.WriteLine(BitConverter.ToString(smallBuffer).Replace("-", string.Empty));
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

        private void OnCmdResponseAvaialbe(byte[] response)
        {
            if (CmdResponseAvailable != null)
            {
                CmdResponseAvailable(response);
            }
        }

        public void Send(byte[] cmd)
        {
            bool gotAck = false;
            cmdResponseAvailableHandler handler = delegate(byte[] r)
            {
                if (r[0] == 0x0D)
                {
                    gotAck = true;
                }
            };
            CmdResponseAvailable += handler;
            serialPort.Write(new byte[] { 0x0D }, 0, 1);
            while (!gotAck) ;
            serialPort.Write(cmd, 0, cmd.Length);
            CmdResponseAvailable -= handler;
        }

        public bool serialPortIsOPen()
        {
            return serialPort.IsOpen;
        }      

    }
}
