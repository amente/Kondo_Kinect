using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace Kondo_Kinect
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private KinectControlWindow kinectControlWindow;
        private ManualControlWindow manualControlWindow;

        private CommandSender commandSender;
        private int BaudRate = 9600;

        static string[] comPorts;

        public MainWindow()
        {
            InitializeComponent();
            refreshComPorts();
            this.commandSender = CommandSender.Instance;        
        }

        private void btnOpenKinectControl_Click(object sender, RoutedEventArgs e)
        {
            if (kinectControlWindow == null)
            {
                kinectControlWindow = new KinectControlWindow();                
            }
            kinectControlWindow.Show();
        }

        private void btnOpenManualControl_Click(object sender, RoutedEventArgs e)
        {
            if (manualControlWindow == null)
            {
                manualControlWindow = new ManualControlWindow();
                manualControlWindow.Show();
            }
            manualControlWindow.Show();
        }


        private void refreshComPorts()
        {
            comPorts = SerialPort.GetPortNames();
            if (comPorts.Length != 0)
            {
                this.cmbSerialPort.Items.Clear();
                this.cmbSerialPort.Items.Add(comPorts);
            }
            else
            {
                this.cmbSerialPort.Items.Add("<N/A>");
            }
            this.cmbSerialPort.SelectedIndex = 0;
        }

        private void cmbSerialPort_ContextMenuOpening(object sender, ContextMenuEventArgs e)
        {
            refreshComPorts();
        }

        private void btnConnect_Click(object sender, RoutedEventArgs e)
        {
            if (!commandSender.serialPortIsOPen())
            {
                
                String PortName = this.cmbSerialPort.SelectedItem.ToString();
                
                if (!commandSender.openSerialPort(PortName, BaudRate))
                {
                   MessageBox.Show("Unable to open port: " + PortName + "\n");
                }                     

            }
            else
            {
                commandSender.closeSerialPort();               
            }
        }


        private void serialPortStatusChanged(bool isOpen)
        {
            if (isOpen)
            {
                btnConnect.Content = "Disconnect";
                cmbSerialPort.IsEnabled = false;
                btnOpenKinectControl.IsEnabled = true;
                btnOpenManualControl.IsEnabled = true;
            }
            else
            {
                btnConnect.Content = "Connect";
                cmbSerialPort.IsEnabled = true;
                btnOpenKinectControl.IsEnabled = false;
                btnOpenManualControl.IsEnabled = false;
            }

        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            this.commandSender.SerialPortStatusChanged += serialPortStatusChanged;
        }


    }
}
