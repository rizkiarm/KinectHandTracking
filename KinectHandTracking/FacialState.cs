using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using Microsoft.Kinect.Face;

namespace KinectHandTracking
{
    public class FacialState
    {
        public FaceFrameSource Source;
        public FaceFrameReader Reader;

        public FacialState(KinectSensor sensor, ulong trackingId, EventHandler<TrackingIdLostEventArgs> handler)
        {
            this.Source = new FaceFrameSource(sensor, trackingId, FaceFrameFeatures.BoundingBoxInColorSpace |
                                                  FaceFrameFeatures.FaceEngagement |
                                                  FaceFrameFeatures.Glasses |
                                                  FaceFrameFeatures.Happy |
                                                  FaceFrameFeatures.LeftEyeClosed |
                                                  FaceFrameFeatures.MouthOpen |
                                                  FaceFrameFeatures.PointsInColorSpace |
                                                  FaceFrameFeatures.RightEyeClosed);

            this.Source.TrackingIdLost += handler;

            this.Reader = this.Source.OpenReader();
        }
    }
}
