using System;
using System.Collections.Generic;
using System.Windows.Forms;
using System.Drawing;
using DirectX.Capture;
using Microsoft.DirectX;
using Microsoft.DirectX.Direct3D;
using sentience;

namespace WindowsApplication1
{    
    static class Program
    {        
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.Run(new frmMain());
        }
    }


    public class globals
    {
        public cameraSettings CamSettingsLeft = new cameraSettings();

        public String calibration_filename;
        public String calibration_filename_left;

        //the image is converted into a standard size for processing
        public int standard_width = 320/1;
        public int standard_height = 240/1;

        //bitmaps for left and right images
        public Image image_left;
        public Image image_right;
        public Byte[] left_bmp=null;
        public Byte[] right_bmp=null;
        public bool camerasRunning;

        public bool initStereo;
        public bool gvBusy;
        public bool gvCameraSelected;
        public int startupTicks;

        public struct ActiveLeft
        {
            public DirectX.Capture.Filter Camera;
            public Capture CaptureInfo;
            public int Counter;
            public int CounterFrames;
            public String PathVideo;
        }

        public ActiveLeft CaptureInformationLeft;
        public Filters WDM_filters = new Filters();

        public void PrepareCamLeft(String PathVideo)
        {
            //String[] s;

            //s = PathVideo.Split(".")
            ConfParamCamLeft();
            //CaptureInformation.CaptureInfo.Filename = s(0) + CStr(CaptureInformation.Counter) + ".avi"
            CaptureInformationLeft.Counter ++;
            CaptureInformationLeft.CaptureInfo.RenderPreview();
        }

        public void selectCamera(bool leftImage, int cameraindex)
        {
                CaptureInformationLeft.Camera = WDM_filters.VideoInputDevices[cameraindex];
                CaptureInformationLeft.CaptureInfo = new Capture(CaptureInformationLeft.Camera, null);
                CamSettingsLeft.cameraName = WDM_filters.VideoInputDevices[cameraindex].Name;
                CamSettingsLeft.Save();
        }


        public int getCameraIndex(String cameraName)
        {
            //return the index of the given camera
            int i;
            DirectX.Capture.Filter f;
            bool found = false;

            i = 0;
            while ((!found) && (i < WDM_filters.VideoInputDevices.Count))
            {
                f = WDM_filters.VideoInputDevices[i];
                if (f.Name == cameraName)
                    found = true;
                    else
                    i++;            
            }
            if (found)
                return(i);
                else
                return(-1);        
        }


        public void ConfParamCamLeft()
        {
            String[] s;
            Size size;
            float Rate;

            CaptureInformationLeft.CaptureInfo.Stop();

            // Change the compressor
            //CaptureInformation.CaptureInfo.VideoCompressor = WDM_filters.VideoCompressors(CaptureInformation.ConfWindow.cmbCompress.Items.IndexOf(CaptureInformation.ConfWindow.cmbCompress.Text))

            // Change the image size
            //s = CaptureInformationLeft.ConfWindow.cmbTam.Text.Split("x")
            if (CamSettingsLeft.resolution == "") CamSettingsLeft.resolution = "320x240";
            s = CamSettingsLeft.resolution.Split('x');
            size = new Size(Convert.ToInt32(s[0]), Convert.ToInt32(s[1]));
            CaptureInformationLeft.CaptureInfo.FrameSize = size;

            // Change the number of frames per second
            //s = CaptureInformationLeft.ConfWindow.cmbFPS.Text.Split(" ")
            if (CamSettingsLeft.frameRate == "") CamSettingsLeft.frameRate = "30";
            s = CamSettingsLeft.frameRate.Split(' ');
            Rate = Convert.ToSingle(s[0]);
            CaptureInformationLeft.CaptureInfo.FrameRate = Rate;
        }


    }
}