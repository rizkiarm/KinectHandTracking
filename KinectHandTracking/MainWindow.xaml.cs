using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace KinectHandTracking
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Members

        KinectSensor _sensor;
        MultiSourceFrameReader _reader;
        IList<Body> _bodies;

        #endregion

        #region Constructor

        public MainWindow()
        {
            InitializeComponent();
        }

        #endregion

        #region Event handlers

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            _sensor = KinectSensor.GetDefault();

            if (_sensor != null)
            {
                _sensor.Open();

                _reader = _sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Infrared | FrameSourceTypes.Body);
                _reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;
            }
        }

        private void Window_Closed(object sender, EventArgs e)
        {
            if (_reader != null)
            {
                _reader.Dispose();
            }

            if (_sensor != null)
            {
                _sensor.Close();
            }
        }

        public long lastTimeRendered;
        public const int CAMERA_FRAMERATE = 30;
        public Dictionary<ulong, Color> bodyColor = new Dictionary<ulong, Color>();

        void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            var reference = e.FrameReference.AcquireFrame();

            // Color
            using (var frame = reference.ColorFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    if (showColorCamera.IsChecked.Value)
                    {
                        long milliseconds = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
                        if(milliseconds - lastTimeRendered > 1000/CAMERA_FRAMERATE)
                        {
                            camera.Source = frame.ToBitmap();
                            lastTimeRendered = milliseconds;
                        }
                    }
                    else camera.Source = null;
                }
            }

            // Body
            using (var frame = reference.BodyFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    canvas.Children.Clear();

                    _bodies = new Body[frame.BodyFrameSource.BodyCount];

                    frame.GetAndRefreshBodyData(_bodies);

                    IEnumerable<Body> trackedBodies = _bodies.Where(b => b.IsTracked);

                    // Compute the closest body
                    Body nearestBody = null;
                    foreach (var body in trackedBodies)
                    {
                        if (nearestBody == null || body.Joints[JointType.Head].Position.Z < nearestBody.Joints[JointType.Head].Position.Z)
                        {
                            nearestBody = body;
                        }
                    }

                    foreach (var body in trackedBodies)
                    {
                        // Find the joints
                        Joint handRight = body.Joints[JointType.HandRight];
                        Joint thumbRight = body.Joints[JointType.ThumbRight];

                        Joint handLeft = body.Joints[JointType.HandLeft];
                        Joint thumbLeft = body.Joints[JointType.ThumbLeft];

                        // Decide active hand
                        bool isHandrightActive = handRight.Position.Y > handLeft.Position.Y;
                        bool isHandrightAction = body.HandRightState == HandState.Closed || body.HandRightState == HandState.Lasso;
                        bool isHandleftAction = body.HandLeftState == HandState.Closed || body.HandLeftState == HandState.Lasso;
                        bool isBothHandAction = isHandrightAction && isHandleftAction;

                        // Draw hands and thumbs
                        Color handRightColor = isHandrightActive || isBothHandAction ? (isHandrightAction ? Colors.Aqua : Colors.LightGreen) : (isHandrightAction ? Colors.Red : Colors.LightPink);
                        Color handLeftColor = !isHandrightActive || isBothHandAction ? (isHandleftAction ? Colors.Aqua : Colors.LightGreen) : (isHandleftAction ? Colors.Red : Colors.LightPink);

                        canvas.DrawHand(handRight, _sensor.CoordinateMapper, handRightColor);
                        canvas.DrawHand(handLeft, _sensor.CoordinateMapper, handLeftColor);
                        canvas.DrawThumb(thumbRight, _sensor.CoordinateMapper, handRightColor);
                        canvas.DrawThumb(thumbLeft, _sensor.CoordinateMapper, handLeftColor);

                        // Draw skeleton
                        if (!bodyColor.ContainsKey(body.TrackingId)) bodyColor[body.TrackingId] = Extensions.getRandomColor();
                        if (showSkeleton.IsChecked.Value) canvas.DrawSkeleton(body, _sensor.CoordinateMapper, bodyColor[body.TrackingId]);

                        // Draw body index

                        // Affect cursor
                        if (nearestBody == body && enableControl.IsChecked.Value) body.AffectOutsideWorld(_sensor.CoordinateMapper);

                        // Find the hand states
                        string rightHandState = "-";
                        string leftHandState = "-";

                        switch (body.HandRightState)
                        {
                            case HandState.Open:
                                rightHandState = "Open";
                                break;
                            case HandState.Closed:
                                rightHandState = "Closed";
                                break;
                            case HandState.Lasso:
                                rightHandState = "Lasso";
                                break;
                            case HandState.Unknown:
                                rightHandState = "Unknown";
                                break;
                            case HandState.NotTracked:
                                rightHandState = "Not tracked";
                                break;
                            default:
                                break;
                        }

                        switch (body.HandLeftState)
                        {
                            case HandState.Open:
                                leftHandState = "Open";
                                break;
                            case HandState.Closed:
                                leftHandState = "Closed";
                                break;
                            case HandState.Lasso:
                                leftHandState = "Lasso";
                                break;
                            case HandState.Unknown:
                                leftHandState = "Unknown";
                                break;
                            case HandState.NotTracked:
                                leftHandState = "Not tracked";
                                break;
                            default:
                                break;
                        }

                        tblRightHandState.Text = rightHandState;
                        tblLeftHandState.Text = leftHandState;

                        Point rightHandPosition = handRight.Scale(_sensor.CoordinateMapper);
                        Point leftHandPosition = handLeft.Scale(_sensor.CoordinateMapper);

                        tblRightHandPosition.Text = Math.Floor(rightHandPosition.X).ToString() + ", " + Math.Floor(rightHandPosition.Y).ToString();
                        tblLeftHandPosition.Text = Math.Floor(leftHandPosition.X).ToString() +", "+ Math.Floor(leftHandPosition.Y).ToString();

                        tblHandDistance.Text = Math.Floor((rightHandPosition.X - leftHandPosition.X)).ToString() + ", " + Math.Floor((rightHandPosition.Y - leftHandPosition.Y)).ToString();
                        tblHandBothStatus.Text = body.HandLeftState == HandState.Closed && body.HandRightState == HandState.Closed ? "Wheel" : "None";
                    }
                }
            }
        }

        #endregion
    }
}
