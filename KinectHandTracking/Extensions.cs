using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

using System.Windows.Forms;

namespace KinectHandTracking
{
    public static class Extensions
    {
        #region Camera

        public static ImageSource ToBitmap(this ColorFrame frame)
        {
            int width = frame.FrameDescription.Width;
            int height = frame.FrameDescription.Height;
            PixelFormat format = PixelFormats.Bgr32;

            byte[] pixels = new byte[width * height * ((format.BitsPerPixel + 7) / 8)];

            if (frame.RawColorImageFormat == ColorImageFormat.Bgra)
            {
                frame.CopyRawFrameDataToArray(pixels);
            }
            else
            {
                frame.CopyConvertedFrameDataToArray(pixels, ColorImageFormat.Bgra);
            }

            int stride = width * format.BitsPerPixel / 8;

            return BitmapSource.Create(width, height, 96, 96, format, null, pixels, stride);
        }

        public static ImageSource ToBitmap(this DepthFrame frame)
        {
            int width = frame.FrameDescription.Width;
            int height = frame.FrameDescription.Height;
            PixelFormat format = PixelFormats.Bgr32;

            ushort minDepth = frame.DepthMinReliableDistance;
            ushort maxDepth = frame.DepthMaxReliableDistance;

            ushort[] pixelData = new ushort[width * height];
            byte[] pixels = new byte[width * height * (format.BitsPerPixel + 7) / 8];

            frame.CopyFrameDataToArray(pixelData);

            int colorIndex = 0;
            for (int depthIndex = 0; depthIndex < pixelData.Length; ++depthIndex)
            {
                ushort depth = pixelData[depthIndex];

                byte intensity = (byte)(depth >= minDepth && depth <= maxDepth ? depth : 0);

                pixels[colorIndex++] = intensity; // Blue
                pixels[colorIndex++] = intensity; // Green
                pixels[colorIndex++] = intensity; // Red

                ++colorIndex;
            }

            int stride = width * format.BitsPerPixel / 8;

            return BitmapSource.Create(width, height, 96, 96, format, null, pixels, stride);
        }

        public static ImageSource ToBitmap(this InfraredFrame frame)
        {
            int width = frame.FrameDescription.Width;
            int height = frame.FrameDescription.Height;
            PixelFormat format = PixelFormats.Bgr32;

            ushort[] frameData = new ushort[width * height];
            byte[] pixels = new byte[width * height * (format.BitsPerPixel + 7) / 8];

            frame.CopyFrameDataToArray(frameData);

            int colorIndex = 0;
            for (int infraredIndex = 0; infraredIndex < frameData.Length; infraredIndex++)
            {
                ushort ir = frameData[infraredIndex];

                byte intensity = (byte)(ir >> 7);

                pixels[colorIndex++] = (byte)(intensity / 1); // Blue
                pixels[colorIndex++] = (byte)(intensity / 1); // Green   
                pixels[colorIndex++] = (byte)(intensity / 0.4); // Red

                colorIndex++;
            }

            int stride = width * format.BitsPerPixel / 8;

            return BitmapSource.Create(width, height, 96, 96, format, null, pixels, stride);
        }

        #endregion

        #region Body

        public static Point Scale(this Joint joint, CoordinateMapper mapper)
        {
            Point point = new Point();

            ColorSpacePoint colorPoint = mapper.MapCameraPointToColorSpace(joint.Position);
            point.X = float.IsInfinity(colorPoint.X) ? 0.0 : colorPoint.X;
            point.Y = float.IsInfinity(colorPoint.Y) ? 0.0 : colorPoint.Y;

            return point;
        }

        #endregion

        #region Drawing

        public static void DrawSkeleton(this Canvas canvas, Body body, CoordinateMapper mapper)
        {
            if (body == null) return;

            foreach (Joint joint in body.Joints.Values)
            {
                canvas.DrawPoint(joint, mapper);
            }

            canvas.DrawLine(body.Joints[JointType.Head], body.Joints[JointType.Neck], mapper);
            canvas.DrawLine(body.Joints[JointType.Neck], body.Joints[JointType.SpineShoulder], mapper);
            canvas.DrawLine(body.Joints[JointType.SpineShoulder], body.Joints[JointType.ShoulderLeft], mapper);
            canvas.DrawLine(body.Joints[JointType.SpineShoulder], body.Joints[JointType.ShoulderRight], mapper);
            canvas.DrawLine(body.Joints[JointType.SpineShoulder], body.Joints[JointType.SpineMid], mapper);
            canvas.DrawLine(body.Joints[JointType.ShoulderLeft], body.Joints[JointType.ElbowLeft], mapper);
            canvas.DrawLine(body.Joints[JointType.ShoulderRight], body.Joints[JointType.ElbowRight], mapper);
            canvas.DrawLine(body.Joints[JointType.ElbowLeft], body.Joints[JointType.WristLeft], mapper);
            canvas.DrawLine(body.Joints[JointType.ElbowRight], body.Joints[JointType.WristRight], mapper);
            canvas.DrawLine(body.Joints[JointType.WristLeft], body.Joints[JointType.HandLeft], mapper);
            canvas.DrawLine(body.Joints[JointType.WristRight], body.Joints[JointType.HandRight], mapper);
            canvas.DrawLine(body.Joints[JointType.HandLeft], body.Joints[JointType.HandTipLeft], mapper);
            canvas.DrawLine(body.Joints[JointType.HandRight], body.Joints[JointType.HandTipRight], mapper);
            canvas.DrawLine(body.Joints[JointType.HandTipLeft], body.Joints[JointType.ThumbLeft], mapper);
            canvas.DrawLine(body.Joints[JointType.HandTipRight], body.Joints[JointType.ThumbRight], mapper);
            canvas.DrawLine(body.Joints[JointType.SpineMid], body.Joints[JointType.SpineBase], mapper);
            canvas.DrawLine(body.Joints[JointType.SpineBase], body.Joints[JointType.HipLeft], mapper);
            canvas.DrawLine(body.Joints[JointType.SpineBase], body.Joints[JointType.HipRight], mapper);
            canvas.DrawLine(body.Joints[JointType.HipLeft], body.Joints[JointType.KneeLeft], mapper);
            canvas.DrawLine(body.Joints[JointType.HipRight], body.Joints[JointType.KneeRight], mapper);
            canvas.DrawLine(body.Joints[JointType.KneeLeft], body.Joints[JointType.AnkleLeft], mapper);
            canvas.DrawLine(body.Joints[JointType.KneeRight], body.Joints[JointType.AnkleRight], mapper);
            canvas.DrawLine(body.Joints[JointType.AnkleLeft], body.Joints[JointType.FootLeft], mapper);
            canvas.DrawLine(body.Joints[JointType.AnkleRight], body.Joints[JointType.FootRight], mapper);
        }

        public static void DrawPoint(this Canvas canvas, Joint joint, CoordinateMapper mapper)
        {
            if (joint.TrackingState == TrackingState.NotTracked) return;

            Point point = joint.Scale(mapper);

            Ellipse ellipse = new Ellipse
            {
                Width = 20,
                Height = 20,
                Fill = new SolidColorBrush(Colors.LightBlue)
            };

            Canvas.SetLeft(ellipse, point.X - ellipse.Width / 2);
            Canvas.SetTop(ellipse, point.Y - ellipse.Height / 2);

            canvas.Children.Add(ellipse);
        }

        public static void DrawHand(this Canvas canvas, Joint hand, CoordinateMapper mapper, Color color)
        {
            if (hand.TrackingState == TrackingState.NotTracked) return;

            Point point = hand.Scale(mapper);

            Ellipse ellipse = new Ellipse
            {
                Width = 100 * 1/hand.Position.Z,
                Height = 100 * 1/hand.Position.Z,
                Stroke = new SolidColorBrush(color),
                StrokeThickness = 4
            };

            Canvas.SetLeft(ellipse, point.X - ellipse.Width / 2);
            Canvas.SetTop(ellipse, point.Y - ellipse.Height / 2);

            canvas.Children.Add(ellipse);
        }

        public static void DrawThumb(this Canvas canvas, Joint thumb, CoordinateMapper mapper, Color color)
        {
            if (thumb.TrackingState == TrackingState.NotTracked) return;

            Point point = thumb.Scale(mapper);

            Ellipse ellipse = new Ellipse
            {
                Width = 40 * 1 / thumb.Position.Z,
                Height = 40 * 1 / thumb.Position.Z,
                Fill = new SolidColorBrush(color),
                Opacity = 0.7
            };

            Canvas.SetLeft(ellipse, point.X - ellipse.Width / 2);
            Canvas.SetTop(ellipse, point.Y - ellipse.Height / 2);

            canvas.Children.Add(ellipse);
        }

        public static void DrawLine(this Canvas canvas, Joint first, Joint second, CoordinateMapper mapper)
        {
            if (first.TrackingState == TrackingState.NotTracked || second.TrackingState == TrackingState.NotTracked) return;

            Point firstPoint = first.Scale(mapper);
            Point secondPoint = second.Scale(mapper);

            Line line = new Line
            {
                X1 = firstPoint.X,
                Y1 = firstPoint.Y,
                X2 = secondPoint.X,
                Y2 = secondPoint.Y,
                StrokeThickness = 8,
                Stroke = new SolidColorBrush(Colors.LightBlue)
            };

            canvas.Children.Add(line);
        }

        #endregion

        #region Outside World

        // Last state:
        // 0 = Nothing
        // 1 = Left down
        // 2 = Right down
        public static int lastState = 0;

        public static void AffectOutsideWorld(this Body body, CoordinateMapper mapper)
        {
            Joint handRight = body.Joints[JointType.HandRight];
            Joint handLeft = body.Joints[JointType.HandLeft];

            bool isHandrightActive = handRight.Position.Y > handLeft.Position.Y;

            Joint hand = isHandrightActive ? handRight : handLeft;
            HandState handState = isHandrightActive ? body.HandRightState : body.HandLeftState;

            if (hand.TrackingState == TrackingState.NotTracked) return;

            // Forward position
            Point point = hand.Scale(mapper).NormalizeToScreenResolution(2.5f);
            System.Drawing.Point dp = new System.Drawing.Point((int)Math.Floor(point.X), (int)Math.Floor(point.Y));
            System.Windows.Forms.Cursor.Position = dp;

            // Special state
            if(body.HandRightState == HandState.Closed && body.HandLeftState == HandState.Closed)
            {
                Point distance = new Point();
                distance.X = handRight.Scale(mapper).X - handLeft.Scale(mapper).X;
                distance.Y = handRight.Scale(mapper).Y - handLeft.Scale(mapper).Y;

                VirtualMouse.WheelTo((int) distance.Y);
            }
            else
            {
                // Common state
                switch (handState)
                {
                    case HandState.Open:
                        if (lastState == 1) VirtualMouse.LeftUp();
                        if (lastState == 2) VirtualMouse.RightUp();
                        lastState = 0;
                        break;
                    case HandState.Closed:
                        // Send left down
                        VirtualMouse.LeftDown();
                        lastState = 1;
                        break;
                    case HandState.Lasso:
                        // Send right click
                        if (lastState != 2) VirtualMouse.RightClick();
                        lastState = 2;
                        break;
                    case HandState.Unknown:
                        break;
                    case HandState.NotTracked:
                        break;
                    default:
                        break;
                }
            }

        }

        public static Point NormalizeToScreenResolution(this Point point, float multiplier)
        {
            Point normalizedPoint = new Point();

            float windowWidth = System.Windows.Forms.Screen.PrimaryScreen.Bounds.Width;
            float windowHeight = System.Windows.Forms.Screen.PrimaryScreen.Bounds.Height;

            // TODO: Find a way to get kinect width and height capture data
            float cameraWidth = 1920.0f;
            float cameraHeight = 1080.0f;

            normalizedPoint.X = (int)Remap(cameraWidth/2 + ((float)point.X-cameraWidth/2)*multiplier, 0.0f, cameraWidth, 0.0f, windowWidth);
            normalizedPoint.Y = (int)Remap(cameraHeight/2 + ((float)point.Y - cameraHeight / 2) * multiplier, 0.0f, cameraHeight, 0.0f, windowHeight);

            return normalizedPoint;
        }


        public static float Remap(float value, float from1, float to1, float from2, float to2)
        {
            return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
        }

        #endregion
    }
}
