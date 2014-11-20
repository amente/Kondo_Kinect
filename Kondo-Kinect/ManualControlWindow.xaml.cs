using System;
using System.Collections.Generic;
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

    public class MyConvertor : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            return Math.Round((double)value);
        }

        public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            return System.Convert.ToDouble((String)value);
        }

    }
    /// <summary>
    /// Interaction logic for ManualControlWindow.xaml
    /// </summary>
    public partial class ManualControlWindow : Window
    {
        public ManualControlWindow()
        {
            InitializeComponent();
        }

        private void sliderValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            Slider slider = (Slider)(sender);
            int channelNumber = Convert.ToInt16(slider.Name.Substring(8));
            Console.WriteLine(channelNumber);
            int angle = Convert.ToInt16(slider.Value);
            RobotController.Instance.setServoAngle(channelNumber, angle);
        }
    }
}
