/* 
    MonoSLAM:  A vision based SLAM program
    Based upon SceneLib, by Andrew Davison ( http://www.doc.ic.ac.uk/~ajd )
    Copyright (C) 2006  Bob Mottram

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA 
*/

using System;
using System.IO;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Drawing.Imaging;
using System.Text;
using System.Windows.Forms;
using DirectX.Capture;
using Microsoft.DirectX;
using Microsoft.DirectX.Direct3D;
using sentience;
using monoSLAM;
using SceneLibrary;

namespace WindowsApplication1
{

    public partial class frmMain : common
    {
        monoSLAM_test test = new monoSLAM_test();
        classimage_mono raw_image, output_image=null;
        classimage raw_image_colour = null;
        int selected_display = MonoSLAM.DISPLAY_FEATURES;
        bool simulation_mode = true;
        bool reset = false;

        //whether DirectX 3D stuff has been initialised
        bool initialised_3D = false;

        globals global_variables;
        bool single_step = false;  //step through one frame at a time

        String video_path = System.AppDomain.CurrentDomain.BaseDirectory + "\\testimages\\";
        int video_frame_number=1;

        bool left_camera_running = false;
        bool left_imageloaded=false;
        String left_image_filename="";
        String output_filename1="";
        String output_filename2="";
        int output_file_index = 0;
        bool recordImages = false;

        //bool initialised = false;
        //int viewIndex = 1;

        int[] captureState = new int[2];
        Bitmap leftbmp;
        Bitmap background_bitmap = null;

        //const String PATH_STEREO_IMAGES = "C:\\Develop\\SluggishVision\\Stereo\\panorama";
        //int currentStereoPairIndex = 0;

        RotateFlipType transformLeft = 0;

        //output image
        Byte[] disp_bmp_data;
        bool outputInitialised = false;

        //types of stereo algorithm
        const int ALGORITHM_DISPARITY = 0;
        const int ALGORITHM_CAMERA_POSE = 1;
        const int ALGORITHM_POINTS = 2;
        const int ALGORITHM_OVERHEAD_MAP = 3;
        const int ALGORITHM_MODEL = 4;
        const int ALGORITHM_OLD = 5;

        DateTime stopWatchTime;


#region "Camera stuff"

        public void updateVisionLeft(Image input_img, bool leftImage)
        {
            try
            {
                if (captureState[0] == 1)
                {
                    if (global_variables.left_bmp == null)
                        global_variables.left_bmp = new Byte[input_img.Width * input_img.Height * 3];

                    updatebitmap((Bitmap)(input_img.Clone()), global_variables.left_bmp);

                    captureState[0] = 2;
                }
                left_camera_running = true;
            }
            catch
            {
            }
        }


        private void transformImage(PictureBox Frame, RotateFlipType transformType)
        {
            if (transformType > 0)
                Frame.Image.RotateFlip(transformType);
        }


        #region "Capture Event functions"


        public void RefreshLeftImage(PictureBox Frame) //object sender, FrameGrabArgs f)
        {
            try
            {
                //picLeftImage.Image = null;
                //picLeftImage.Image = Frame.Image;

                //transform the image appropriately
                transformImage(Frame, transformLeft);

                global_variables.CaptureInformationLeft.CounterFrames ++;
                updateVisionLeft(Frame.Image, true);

                //picLeftImage.Refresh();

            }
            catch //(Exception ex)
            {

            }
        }


        #endregion


        private void initCameraLeft2()
        {
            //leftbmp = New Bitmap(CaptureInformationLeft.ConfWindow.Width, CaptureInformationLeft.ConfWindow.Height, System.Drawing.Imaging.PixelFormat.Format24bppRgb)
            leftbmp = new Bitmap(320, 240, PixelFormat.Format24bppRgb);
            picLeftImage.Image = leftbmp;
            picLeftImage.Visible = true;

            global_variables.CaptureInformationLeft.CaptureInfo.PreviewWindow = this.picLeftImage;

            global_variables.ConfParamCamLeft();
            global_variables.PrepareCamLeft(global_variables.CaptureInformationLeft.PathVideo);

            //Define RefreshImage as event handler of FrameCaptureComplete
            global_variables.CaptureInformationLeft.CaptureInfo.FrameCaptureComplete += new Capture.FrameCapHandler(this.RefreshLeftImage);
            //AddHandler(global_variables.CaptureInformationLeft.CaptureInfo.FrameCaptureComplete, *RefreshLeftImage);

            global_variables.CaptureInformationLeft.CaptureInfo.CaptureFrame();

            global_variables.CaptureInformationLeft.Counter = 1;
            global_variables.CaptureInformationLeft.CounterFrames = 1;

            this.Show();
        }


        public void initCameraLeft()
        {
            //Call to AddCam to select an available camera
            AddCam AddCamera = new AddCam(global_variables);
            AddCamera.LeftImage = true;
            AddCamera.ShowDialog(this);

            if (global_variables.gvCameraSelected)
            {
                initCameraLeft2();
            }
        }


#endregion


        public frmMain()
        {
            global_variables = new globals();

            InitializeComponent();

            test.init();
        }

        private void exitToolStripMenuItem_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        private void grabCalibrationImages()
        {
            //take a snapshot of the left and right images and use these for calibration purposes
            String path;

            if (left_camera_running)
            {
                path = System.AppDomain.CurrentDomain.BaseDirectory + "\\";
                global_variables.calibration_filename_left = path + "calibration_left.bmp";

                if (File.Exists(global_variables.calibration_filename_left)) File.Delete(global_variables.calibration_filename_left);
                updatebitmap(global_variables.left_bmp,(Bitmap)global_variables.image_left);
                global_variables.image_left.Save(global_variables.calibration_filename_left);
                
                MessageBox.Show("Calibration images stored", "Sentience");
            }
            else
            {
                MessageBox.Show("The cameras must be running in order to capture calibration images", "sentience");
            }

        }

        private void frmMain_Load(object sender, EventArgs e)
        {
            //initialised = true;
            captureState[0] = 0;
            captureState[1] = 0;

            String path = System.AppDomain.CurrentDomain.BaseDirectory + "\\";
            global_variables.calibration_filename = path + "calibration.dat";
            global_variables.calibration_filename_left = path + "calibration_left.bmp";
            //mono_SLAMinitCalibration(global_variables.standard_width, global_variables.standard_height);
            //mono_SLAMloadCalibration(global_variables.calibration_filename);
            //mono_SLAMupdateCalibration(global_variables.standard_width, global_variables.standard_height);
        }



        private void beginStopWatch()
        {
            stopWatchTime = DateTime.Now;
        }

        private long endStopWatch()
        {
            DateTime currentTime = DateTime.Now;
            TimeSpan timeDiff;
            long milliseconds;

            timeDiff = currentTime.Subtract(stopWatchTime);
            milliseconds = (timeDiff.Minutes * 60000) + (timeDiff.Seconds * 1000) + timeDiff.Milliseconds;
            return (milliseconds);
        }


        private void openImage(bool leftImage)
        {
            //open an image
            Stream myStream;
            String filename;
            //int index;
            //long processingTime;
            PictureBox destImage;
            FileStream fs;

            openFileDialog1.Title = "Open Camera Image";
            destImage = picLeftImage;
            openFileDialog1.InitialDirectory = System.AppDomain.CurrentDomain.BaseDirectory;
            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                myStream = openFileDialog1.OpenFile();
                if (myStream != null)
                {
                    // Insert code to read the stream here.
                    filename = openFileDialog1.FileName;
                    fs = new FileStream(filename, FileMode.Open, FileAccess.Read);
                    destImage.Image = System.Drawing.Image.FromStream(fs);
                    fs.Close();
                    destImage.Visible = true;
                    destImage.Refresh();
                    left_imageloaded = true;
                    left_image_filename = filename;

                    myStream.Close();
                }
            }
        }

        private bool loadVideoFrame(String path, int index)
        {
            String filename;
            FileStream fs;
            bool success=false;

            filename = path + "rawoutput";
            if (index < 10)
            {
                filename += "000";
            }
            else
            {
                if (index < 100)
                {
                    filename += "00";
                }
                else
                {
                    if (index < 1000)
                    {
                        filename += "0";
                    }
                }
            }
            filename += index + ".jpg";

            if (File.Exists(filename))
            {
                fs = new FileStream(filename, FileMode.Open, FileAccess.Read);
                picLeftImage.Image = Image.FromStream(fs);
                fs.Close();
                picLeftImage.Visible = true;
                picLeftImage.Refresh();
                left_image_filename = filename;
                left_imageloaded = true;
                success=true;
            }
            return(success);
        }
 

        /// <summary>
        /// Show an image within the given picture box control
        /// </summary>
        /// <param name="pic">picture box control in which to draw the image</param>
        /// <param name="imageIndex">An index number corresponding to the type of image to be displayed</param>
        public void showImage(PictureBox pic, Byte[] imagedata, int image_width, int image_height)
        {
            SolidBrush brush;
            Rectangle rect;
            Byte r, g, b;
            int x, y;
            long n;
            Graphics gr;

            gr = Graphics.FromImage(pic.Image);
            rect = new Rectangle();

            n = 0;
            for (y = 0;y < image_height; y++)
            {
                for (x = 0; x < image_width; x++)
                {
                    b = imagedata[n];
                    n++;
                    g = imagedata[n];
                    n++;
                    r = imagedata[n];
                    n++;

                    brush = new SolidBrush(Color.FromArgb(r, g, b));
                    rect.X = (pic.Image.Width * x) / image_width;
                    rect.Y = (pic.Image.Height * y) / image_height;
                    rect.Width = pic.Image.Width / image_width * 2;
                    rect.Height = pic.Image.Height / image_height * 2;
                    gr.FillRectangle(brush, rect);
                }
            }
            pic.Refresh();
        }


        private void captureCameraImages()
        {
            //capture images from the two cameras
            //note the use of a state indicator to ensure that captures are only initiated at the appropriate time
            if (left_camera_running)
            {
                try
                {
                    if ((captureState[0] < 2) && (!global_variables.CaptureInformationLeft.CaptureInfo.Capturing))
                    {
                        captureState[0] = 1;
                        global_variables.CaptureInformationLeft.CaptureInfo.CaptureFrame();
                    }
                }
                catch
                {
                }
            }
        }

        private void recordDepthImages()
        {
            //record images for demo purposes
            if (recordImages)
            {
                output_file_index = output_file_index + 1;
                if (output_file_index > 998) output_file_index = 0;
                output_filename1 = System.AppDomain.CurrentDomain.BaseDirectory + "\\output1_" + output_file_index + ".bmp";
                output_filename2 = System.AppDomain.CurrentDomain.BaseDirectory + "\\output2_" + output_file_index + ".bmp";
                picLeftImage.Refresh();
                picLeftImage.Image.Save(output_filename1);
                picOutput1.Image.Save(output_filename2);
            }
        }


        private void updateProcessingTime(long processingTime)
        {
            float fps;

            if (processingTime > 0)
            {
                fps = (float)((long)(10000 / processingTime) / 10);
                if (fps < 30) txtfps.Text = Convert.ToString(fps);
            }
        }

        private void timUpdate_Tick(object sender, EventArgs e)
        {
            bool image_loaded;
            int wdth;
            int hght;

            if ((left_imageloaded) || (left_camera_running))
            {
                //get images from the two cameras
                captureCameraImages();

                //record depth images if necessary
                //recordDepthImages();

                if ((!test.frame_rate_warning) || (!test.enable_mapping))
                {
                    if (!test.enable_mapping)
                        cmdBeginTracking.Visible = true;
                    else
                        cmdBeginTracking.Visible = false;

                    lblTracking.Visible = !cmdBeginTracking.Visible;
                    lblFrameRateWarning.Visible = false;
                }
                else
                {
                    lblTracking.Visible = false;
                    lblFrameRateWarning.Visible = true;
                }


                if (simulation_mode)
                {
                    //update from loaded images
                    image_loaded = loadVideoFrame(video_path, video_frame_number);
                    if (image_loaded)
                    {
                        wdth = picLeftImage.Image.Width;
                        hght = picLeftImage.Image.Height;

                        if (global_variables.left_bmp == null)
                        {
                            global_variables.left_bmp = new Byte[wdth * hght * 3];
                            raw_image = new classimage_mono();
                            raw_image.createImage(wdth, hght);
                            raw_image_colour = new classimage();
                            raw_image_colour.createImage(wdth, hght);
                        }

                        updatebitmap((Bitmap)picLeftImage.Image, global_variables.left_bmp);

                        raw_image_colour.updateFromBitmap(global_variables.left_bmp, 1, wdth, hght);
                        raw_image.copyImage(raw_image_colour);

                        if (!outputInitialised)
                        {
                            picOutput1.Image = new Bitmap(global_variables.standard_width, global_variables.standard_height, PixelFormat.Format24bppRgb);
                            background_bitmap = new Bitmap(global_variables.standard_width, global_variables.standard_height, PixelFormat.Format24bppRgb);
                            disp_bmp_data = new Byte[global_variables.standard_width * global_variables.standard_height * 3];
                            output_image = new classimage_mono();
                            output_image.createImage(global_variables.standard_width,global_variables.standard_height);
                            outputInitialised = true;
                        }
                        output_image.updateFromImage(raw_image);

                        txtCaptureTime.Text = Convert.ToString(endStopWatch());
                        beginStopWatch();

                        test.GoOneStep(raw_image, raw_image_colour, output_image);

                        txtProcessingTime.Text = Convert.ToString(endStopWatch());
                        txtfps.Text = Convert.ToString(test.frames_per_second);
                        txtSpeed.Text = Convert.ToString(test.speed);

                        beginStopWatch();

                        test.ShowFeatures(output_image, selected_display);
                        output_image.saveToBitmap(disp_bmp_data, global_variables.standard_width, global_variables.standard_height);

                        // either show the output image in the picturebox, or transfer it
                        // to a separate background bitmap for use during 3D rendering
                        if ((selected_display != MonoSLAM.DISPLAY_AUGMENTED_REALITY) ||
                            ((selected_display == MonoSLAM.DISPLAY_AUGMENTED_REALITY) && (!(initialised_3D && test.enable_mapping))))
                        {
                            updatebitmap(disp_bmp_data, (Bitmap)picOutput1.Image);
                            picOutput1.Refresh();
                        }
                        else
                        {
                            updatebitmap(disp_bmp_data, background_bitmap);
                        }

                        // render 3D objects if augmented reality is enabled
                        render3D(d3dDevice);


                        if (!single_step) video_frame_number++;
                    }
                    else
                    {
                        video_frame_number = 1;
                        test.init();
                    }
                }
                else
                {
                    //update from cameras
                    if (captureState[0] == 2)
                    {
                        captureState[0] = 0;
                        captureState[1] = 0;

                        wdth = picLeftImage.Image.Width;
                        hght = picLeftImage.Image.Height;

                        if (raw_image == null)
                        {
                            // initialise raw image object
                            raw_image = new classimage_mono();
                            raw_image.createImage(wdth, hght);
                            raw_image_colour = new classimage();
                            raw_image_colour.createImage(wdth, hght);
                        }
                        
                        // shove the bitmap from the camera into an image object
                        raw_image_colour.updateFromBitmap(global_variables.left_bmp, 1, wdth, hght);
                        raw_image.copyImage(raw_image_colour);

                        if (!outputInitialised)
                        {
                            // initialise image bitmaps
                            picOutput1.Image = new Bitmap(global_variables.standard_width, global_variables.standard_height, PixelFormat.Format24bppRgb);
                            background_bitmap = new Bitmap(global_variables.standard_width, global_variables.standard_height, PixelFormat.Format24bppRgb);
                            disp_bmp_data = new Byte[global_variables.standard_width * global_variables.standard_height * 3];
                            output_image = new classimage_mono();
                            output_image.createImage(global_variables.standard_width, global_variables.standard_height);
                            outputInitialised = true;
                        }
                        output_image.updateFromImage(raw_image);

                        txtCaptureTime.Text = Convert.ToString(endStopWatch());
                        beginStopWatch();

                        // one processing cycle update
                        test.GoOneStep(raw_image, raw_image_colour, output_image);

                        //show info
                        txtProcessingTime.Text = Convert.ToString(endStopWatch());
                        txtfps.Text = Convert.ToString(test.frames_per_second);
                        txtSpeed.Text = Convert.ToString(test.speed);

                        beginStopWatch();

                        // show crosshair features or overhead map
                        test.ShowFeatures(output_image, selected_display);
                        output_image.saveToBitmap(disp_bmp_data, global_variables.standard_width, global_variables.standard_height);

                        // either show the output image in the picturebox, or transfer it
                        // to a separate background bitmap for use during 3D rendering
                        if ((selected_display != MonoSLAM.DISPLAY_AUGMENTED_REALITY) ||
                            ((selected_display == MonoSLAM.DISPLAY_AUGMENTED_REALITY) && (!(initialised_3D && test.enable_mapping))))
                        {
                            updatebitmap(disp_bmp_data, (Bitmap)picOutput1.Image);
                            picOutput1.Refresh();
                        }
                        else
                        {
                            updatebitmap(disp_bmp_data, background_bitmap);
                        }

                        // render 3D objects if augmented reality is enabled
                        render3D(d3dDevice);

                        if (reset)
                        {
                            test.init();
                            reset = false;
                        }
                    }
                }

            }

        }


        private void openLeftImageToolStripMenuItem_Click(object sender, EventArgs e)
        {
        }


        private void recordImagesToolStripMenuItem_Click(object sender, EventArgs e)
        {
            MessageBox.Show("This feature is not yet available");

            //recordImagesToolStripMenuItem.Checked = !recordImagesToolStripMenuItem.Checked;
            //output_file_index = 0;
            //recordImages = recordImagesToolStripMenuItem.Checked;
        }


        private void startSentienceToolStripMenuItem_Click(object sender, EventArgs e)
        {
            simulation_mode = false;
            initCameraLeft();
            if (global_variables.gvCameraSelected)
            {
                startSentienceToolStripMenuItem.Enabled = false;
                runSimulationToolStripMenuItem.Enabled = false;

                left_imageloaded = true;
                single_step = false;
                calibrationToolStripMenuItem.Enabled = true;
                updateCameraSettings();
                test.simulation_mode = false;
            }
        }

        private void calibrationToolStripMenuItem_Click(object sender, EventArgs e)
        {
            picLeftImage.Visible = false;
            panelCalibration.Visible = true;
            test.calibrating = true;
        }

        private void singleStepToolStripMenuItem_Click(object sender, EventArgs e)
        {
            left_imageloaded = true;
            single_step = true;
            video_frame_number++;
        }

        private void trackedFeaturesToolStripMenuItem_Click(object sender, EventArgs e)
        {
            trackedFeaturesToolStripMenuItem.Checked = true;
            overheadMToolStripMenuItem.Checked = false;
            searchEllipsesToolStripMenuItem.Checked = false;
            particleProbabilitiesToolStripMenuItem.Checked = false;
            augmentedRealityToolStripMenuItem.Checked = false;
            selected_display = MonoSLAM.DISPLAY_FEATURES;
        }

        private void overheadMToolStripMenuItem_Click(object sender, EventArgs e)
        {
            trackedFeaturesToolStripMenuItem.Checked = false;
            overheadMToolStripMenuItem.Checked = true;
            searchEllipsesToolStripMenuItem.Checked = false;
            particleProbabilitiesToolStripMenuItem.Checked = false;
            augmentedRealityToolStripMenuItem.Checked = false;
            selected_display = MonoSLAM.DISPLAY_MAP;
        }

        private void searchEllipsesToolStripMenuItem_Click(object sender, EventArgs e)
        {
            trackedFeaturesToolStripMenuItem.Checked = false;
            overheadMToolStripMenuItem.Checked = false;
            searchEllipsesToolStripMenuItem.Checked = true;
            particleProbabilitiesToolStripMenuItem.Checked = false;
            augmentedRealityToolStripMenuItem.Checked = false;
            selected_display = MonoSLAM.DISPLAY_ELLIPSES;
        }

        private void particleProbabilitiesToolStripMenuItem_Click(object sender, EventArgs e)
        {
            simulation_mode = true;
            trackedFeaturesToolStripMenuItem.Checked = false;
            overheadMToolStripMenuItem.Checked = false;
            searchEllipsesToolStripMenuItem.Checked = false;
            particleProbabilitiesToolStripMenuItem.Checked = true;
            augmentedRealityToolStripMenuItem.Checked = false;
            selected_display = MonoSLAM.DISPLAY_PROBABILITIES;            
        }

        private void runSimulationToolStripMenuItem_Click(object sender, EventArgs e)
        {
            left_imageloaded = true;
            single_step = false;
            test.simulation_mode = true;
        }

        private void cmdDistortionUp_Click(object sender, EventArgs e)
        {
            txtDistortion.Text = Convert.ToString((Convert.ToInt32(txtDistortion.Text) + 1));
            test.lens_distortion = Convert.ToSingle(txtDistortion.Text);
        }

        private void cmdDistortionDown_Click(object sender, EventArgs e)
        {
            txtDistortion.Text = Convert.ToString((Convert.ToInt32(txtDistortion.Text) - 1));
            test.lens_distortion = Convert.ToSingle(txtDistortion.Text);
        }

        private void cmdCentreDistortionXUp_Click(object sender, EventArgs e)
        {
            txtCentreOfDistortionX.Text = Convert.ToString((Convert.ToInt32(txtCentreOfDistortionX.Text) + 1));
            test.centre_of__distortion_x = Convert.ToSingle(txtCentreOfDistortionX.Text);
        }

        private void cmdCentreDistortionXDown_Click(object sender, EventArgs e)
        {
            txtCentreOfDistortionX.Text = Convert.ToString((Convert.ToInt32(txtCentreOfDistortionX.Text) - 1));
            test.centre_of__distortion_x = Convert.ToSingle(txtCentreOfDistortionX.Text);            
        }

        private void cmdCentreDistortionYup_Click(object sender, EventArgs e)
        {
            txtCentreOfDistortionY.Text = Convert.ToString((Convert.ToInt32(txtCentreOfDistortionY.Text) + 1));
            test.centre_of__distortion_y = Convert.ToSingle(txtCentreOfDistortionY.Text);
        }

        private void cmdCentreDistortionYdown_Click(object sender, EventArgs e)
        {
            txtCentreOfDistortionY.Text = Convert.ToString((Convert.ToInt32(txtCentreOfDistortionY.Text) - 1));
            test.centre_of__distortion_y = Convert.ToSingle(txtCentreOfDistortionY.Text);
        }

        private void updateCalibration()
        {

        }

        private void cmdCalibrationDone_Click(object sender, EventArgs e)
        {
            picLeftImage.Visible = true;
            panelCalibration.Visible = false;
            reset = true;
            initialised_3D = false;
            saveCameraSettings();
            test.calibrating = false;
        }

        private void txtFOV_KeyPress(object sender, KeyPressEventArgs e)
        {
            if (e.KeyChar == 13)
            {
                test.field_of_vision = Convert.ToSingle(txtFOV.Text);
                reset = true;
            }
        }

        private void txtDistortion_KeyPress(object sender, KeyPressEventArgs e)
        {
            if (e.KeyChar == 13)
            {
                test.lens_distortion = Convert.ToSingle(txtDistortion.Text);
            }
        }

        private void txtCentreOfDistortionX_KeyPress(object sender, KeyPressEventArgs e)
        {
            if (e.KeyChar == 13)
            {
                test.centre_of__distortion_x = Convert.ToSingle(txtCentreOfDistortionX.Text);
            }
        }

        private void txtCentreOfDistortionY_KeyPress(object sender, KeyPressEventArgs e)
        {
            if (e.KeyChar == 13)
            {
                test.centre_of__distortion_y = Convert.ToSingle(txtCentreOfDistortionY.Text);
            }
        }

        private void txtDistToTarget_KeyPress(object sender, KeyPressEventArgs e)
        {
            if (e.KeyChar == 13)
            {
                test.calibration_target_distance_mm = Convert.ToSingle(txtDistToTarget.Text);
                reset = true;
            }
        }

        private void txtTargetWidth_KeyPress(object sender, KeyPressEventArgs e)
        {
            if (e.KeyChar == 13)
            {
                test.calibration_target_width_mm = Convert.ToSingle(txtTargetWidth.Text);
                reset = true;
            }
        }

        private void txtTargetHeight_KeyPress(object sender, KeyPressEventArgs e)
        {
            if (e.KeyChar == 13)
            {
                test.calibration_target_height_mm = Convert.ToSingle(txtTargetHeight.Text);
                reset = true;
            }
        }

        private void updateCameraSettings()
        {
            txtFOV.Text = Convert.ToString(global_variables.CamSettingsLeft.field_of_vision);
            txtDistortion.Text = Convert.ToString(global_variables.CamSettingsLeft.lens_distortion);
            txtCentreOfDistortionX.Text = Convert.ToString(global_variables.CamSettingsLeft.centre_of_distortion_x);
            txtCentreOfDistortionY.Text = Convert.ToString(global_variables.CamSettingsLeft.centre_of_distortion_y);
            txtDistToTarget.Text = Convert.ToString(global_variables.CamSettingsLeft.distance_to_target_mm);
            txtTargetWidth.Text = Convert.ToString(global_variables.CamSettingsLeft.target_width_mm);
            txtTargetHeight.Text = Convert.ToString(global_variables.CamSettingsLeft.target_height_mm);

            test.field_of_vision = Convert.ToSingle(txtFOV.Text);
            test.lens_distortion = Convert.ToSingle(txtDistortion.Text);
            test.centre_of__distortion_x = Convert.ToSingle(txtCentreOfDistortionX.Text);
            test.centre_of__distortion_y = Convert.ToSingle(txtCentreOfDistortionY.Text);
            test.calibration_target_distance_mm = Convert.ToSingle(txtDistToTarget.Text);
            test.calibration_target_width_mm = Convert.ToSingle(txtTargetWidth.Text);
            test.calibration_target_height_mm = Convert.ToSingle(txtTargetHeight.Text);

            reset = true;
        }

        private void saveCameraSettings()
        {
            global_variables.CamSettingsLeft.field_of_vision = Convert.ToSingle(txtFOV.Text);
            global_variables.CamSettingsLeft.lens_distortion = Convert.ToSingle(txtDistortion.Text);
            global_variables.CamSettingsLeft.centre_of_distortion_x = Convert.ToSingle(txtCentreOfDistortionX.Text);
            global_variables.CamSettingsLeft.centre_of_distortion_y = Convert.ToSingle(txtCentreOfDistortionY.Text);
            global_variables.CamSettingsLeft.distance_to_target_mm = Convert.ToSingle(txtDistToTarget.Text);
            global_variables.CamSettingsLeft.target_width_mm = Convert.ToSingle(txtTargetWidth.Text);
            global_variables.CamSettingsLeft.target_height_mm = Convert.ToSingle(txtTargetHeight.Text);
            global_variables.CamSettingsLeft.Save();
        }

        private void cmdBeginTracking_Click(object sender, EventArgs e)
        {
            if ((left_camera_running) && (!simulation_mode))
            {
                test.enable_mapping = true;
                cmdBeginTracking.Visible = false;
            }
        }

        
         
        #region "DirectX stuff for augmented reality"

        private Device d3dDevice = null;
        private VertexBuffer cubeVertexBuffer = null;
        private VertexBuffer lineVertexBuffer = null;
        private Texture texture = null;
        private D3D_background background_image;

        private struct Vertex
        {
            float x, y, z;
            float tu, tv;

            public Vertex(float _x, float _y, float _z, float _tu, float _tv)
            {
                x = _x; y = _y; z = _z;
                tu = _tu; tv = _tv;
            }

            public static readonly VertexFormats FVF_Flags = VertexFormats.Position | VertexFormats.Texture1;
        };

        private Vertex[] lineVertices =
		{
			new Vertex(-1.0f, 1.0f,-1.0f,  0.0f,0.0f ),
			new Vertex( 1.0f, 1.0f,-1.0f,  1.0f,0.0f ),
			new Vertex(-1.0f,-1.0f,-1.0f,  0.0f,1.0f ),
			new Vertex( 1.0f,-1.0f,-1.0f,  1.0f,1.0f ),
            
			new Vertex(-1.0f, 1.0f, 1.0f,  1.0f,0.0f ),
			new Vertex(-1.0f,-1.0f, 1.0f,  1.0f,1.0f ),
			new Vertex( 1.0f, 1.0f, 1.0f,  0.0f,0.0f ),
			new Vertex( 1.0f,-1.0f, 1.0f,  0.0f,1.0f )
        };


        private Vertex[] cubeVertices =
		{
			new Vertex(-1.0f, 1.0f,-1.0f,  0.0f,0.0f ),
			new Vertex( 1.0f, 1.0f,-1.0f,  1.0f,0.0f ),
			new Vertex(-1.0f,-1.0f,-1.0f,  0.0f,1.0f ),
			new Vertex( 1.0f,-1.0f,-1.0f,  1.0f,1.0f ),
            
			new Vertex(-1.0f, 1.0f, 1.0f,  1.0f,0.0f ),
			new Vertex(-1.0f,-1.0f, 1.0f,  1.0f,1.0f ),
			new Vertex( 1.0f, 1.0f, 1.0f,  0.0f,0.0f ),
			new Vertex( 1.0f,-1.0f, 1.0f,  0.0f,1.0f ),
             
            new Vertex(-1.0f, 1.0f, 1.0f,  0.0f,0.0f ),
			new Vertex( 1.0f, 1.0f, 1.0f,  1.0f,0.0f ),
			new Vertex(-1.0f, 1.0f,-1.0f,  0.0f,1.0f ),
			new Vertex( 1.0f, 1.0f,-1.0f,  1.0f,1.0f ),

			new Vertex(-1.0f,-1.0f, 1.0f,  0.0f,0.0f ),
			new Vertex(-1.0f,-1.0f,-1.0f,  1.0f,0.0f ),
			new Vertex( 1.0f,-1.0f, 1.0f,  0.0f,1.0f ),
			new Vertex( 1.0f,-1.0f,-1.0f,  1.0f,1.0f ),

			new Vertex( 1.0f, 1.0f,-1.0f,  0.0f,0.0f ),
			new Vertex( 1.0f, 1.0f, 1.0f,  1.0f,0.0f ),
			new Vertex( 1.0f,-1.0f,-1.0f,  0.0f,1.0f ),
			new Vertex( 1.0f,-1.0f, 1.0f,  1.0f,1.0f ),

			new Vertex(-1.0f, 1.0f,-1.0f,  1.0f,0.0f ),
			new Vertex(-1.0f,-1.0f,-1.0f,  1.0f,1.0f ),
			new Vertex(-1.0f, 1.0f, 1.0f,  0.0f,0.0f ),
			new Vertex(-1.0f,-1.0f, 1.0f,  0.0f,1.0f )             
		};

        /// <summary>
        /// load a texture from file
        /// </summary>
        private void LoadTexture()
        {
            try
            {
                Bitmap testImage = (Bitmap)Bitmap.FromFile("texture1.bmp");
                texture = Texture.FromBitmap(d3dDevice, testImage, 0, Pool.Managed);
            }
            catch
            {
                // We must be running from within Visual Studio. Relocate the 
                // current directory and try again.
                System.IO.Directory.SetCurrentDirectory(
                    System.Windows.Forms.Application.StartupPath + @"\..\..\");

                Bitmap testImage = (Bitmap)Bitmap.FromFile("texture1.bmp");
                texture = Texture.FromBitmap(d3dDevice, testImage, 0, Pool.Managed);
            }

            d3dDevice.SamplerState[0].MinFilter = TextureFilter.Linear;
            d3dDevice.SamplerState[0].MagFilter = TextureFilter.Linear;
        }


        /// <summary>
        /// turns a postion and quaternion based orientation into a matrix format
        /// </summary>
        /// <param name="position"></param>
        /// <param name="orientation"></param>
        /// <returns></returns>
        private Matrix QuaternionToMatrixLookAt(Vector3D position, SceneLibrary.Quaternion orientation)
        {
            Microsoft.DirectX.Quaternion q_orientation = 
                new Microsoft.DirectX.Quaternion((float)orientation.GetX(),
                                                 (float)orientation.GetY(),
                                                 (float)orientation.GetZ(),
                                                 (float)orientation.GetR());
            Matrix _matrixRotation = Matrix.RotationQuaternion(q_orientation);
            Vector3 camPos = new Vector3((float)position.GetX(), -(float)position.GetY(), (float)position.GetZ());
            Vector3 camTY = new Vector3(_matrixRotation.M21, _matrixRotation.M22, _matrixRotation.M23);
            Vector3 camTZ = new Vector3(-(_matrixRotation.M31), -(_matrixRotation.M32), -(_matrixRotation.M33));
            return (Matrix.LookAtLH(camPos, camPos + camTZ, camTY));

            //float yaw = 0, pitch = 0, roll = 0;
            //DecomposeRollPitchYawZXYMatrix(_matrixRotation, ref pitch, ref yaw, ref roll);

            //return (Matrix.RotationYawPitchRoll((float)yaw, (float)pitch, (float)roll) * Matrix.LookAtLH(camPos, camPos + camTZ, camTY));
            //return (Matrix.RotationYawPitchRoll(0, 0, (float)roll) * Matrix.LookAtLH(camPos, camPos + camTZ, camTY));
        }


        /// <summary>
        /// This method basically creates and initialize the Direct3D device and
        /// anything else that doens't need to be recreated after a device 
        /// reset.
        /// </summary>
        private void Init3D()
        {
            //
            // Do we support hardware vertex processing? If so, use it. 
            // If not, downgrade to software.
            //

            Caps caps = Manager.GetDeviceCaps(Manager.Adapters.Default.Adapter,
                                               DeviceType.Hardware);
            CreateFlags flags;

            if (caps.DeviceCaps.SupportsHardwareTransformAndLight)
                flags = CreateFlags.HardwareVertexProcessing;
            else
                flags = CreateFlags.SoftwareVertexProcessing;

            //
            // Everything checks out - create a simple, windowed device.
            //

            PresentParameters d3dpp = new PresentParameters();

            d3dpp.SwapEffect = SwapEffect.Discard;
            d3dpp.Windowed = true;
            d3dpp.EnableAutoDepthStencil = true;
            d3dpp.AutoDepthStencilFormat = DepthFormat.D16;
            d3dpp.PresentationInterval = PresentInterval.Immediate;
            d3dpp.BackBufferFormat = Format.Unknown;

            //TODO:  Why are these hacky multipliers necessary?
            d3dpp.BackBufferWidth = picLeftImage.Image.Width * 16 / 10;
            d3dpp.BackBufferHeight = picLeftImage.Image.Height * 21 / 20;

            d3dDevice = new Device(0, DeviceType.Hardware, picOutput1, flags, d3dpp);

            // Register an event-handler for DeviceReset and call it to continue
            // our setup.
            d3dDevice.DeviceReset += new System.EventHandler(this.OnResetDevice);
            OnResetDevice(d3dDevice, null);
        }

        /// <summary>
        /// This event-handler is a good place to create and initialize any 
        /// Direct3D related objects, which may become invalid during a 
        /// device reset.
        /// </summary>
        public void OnResetDevice(object sender, EventArgs e)
        {
            Device device = (Device)sender;

            if (picLeftImage.Image != null)
            {
                if (global_variables.CamSettingsLeft != null)
                {
                    if (!initialised_3D)
                    {
                        this.SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.Opaque, true);

                        // Setup the lights
                        device.Lights[0].Enabled = true;
                        device.Lights[0].Type = LightType.Directional;
                        device.Lights[0].Direction = new Vector3(0, 0, 1);
                        device.Lights[0].Diffuse = Color.White;
                        device.Lights[0].Position = new Vector3(0, 0, 0);
                        device.RenderState.Lighting = false;
                        device.RenderState.ZBufferEnable = true;

                        // Set projection matrix
                        device.Transform.Projection = Matrix.PerspectiveFovLH(
                            (float)(global_variables.CamSettingsLeft.field_of_vision * Math.PI / 180.0),
                            (float)picLeftImage.Image.Width / (float)picLeftImage.Image.Height,
                            0.1f, 100.0f);
                        device.RenderState.NormalizeNormals = true;

                        cubeVertexBuffer = new VertexBuffer(typeof(Vertex),
                                                         cubeVertices.Length, device,
                                                         Usage.Dynamic | Usage.WriteOnly,
                                                         Vertex.FVF_Flags,
                                                         Pool.Default);

                        lineVertexBuffer = new VertexBuffer(typeof(Vertex),
                                                         cubeVertices.Length, device,
                                                         Usage.Dynamic | Usage.WriteOnly,
                                                         Vertex.FVF_Flags,
                                                         Pool.Default);

                        GraphicsStream gStream = cubeVertexBuffer.Lock(0, 0, LockFlags.None);

                        // Now, copy the vertex data into the vertex buffer
                        gStream.Write(cubeVertices);

                        cubeVertexBuffer.Unlock();

                        LoadTexture();


                        gStream = lineVertexBuffer.Lock(0, 0, LockFlags.None);

                        // Now, copy the vertex data into the vertex buffer
                        gStream.Write(lineVertices);

                        lineVertexBuffer.Unlock();

                        initialised_3D = true;
                    }

                }
            }

        }

        /// <summary>
        /// returns a material of the given colour
        /// </summary>
        /// <param name="c"></param>
        /// <returns></returns>
        private Microsoft.DirectX.Direct3D.Material InitMaterial(Color c)
        {
            Material mtrl = new Material();
            mtrl.Ambient = mtrl.Diffuse = c;
            return (mtrl);
        }

        // The following functions create the transformation matrices for each operation.
        public void RotationMatrices(Device dx, float x, float y, float z)
        {
            dx.Transform.World = Matrix.Multiply
               (dx.Transform.World, Matrix.RotationX(x));

            dx.Transform.World = Matrix.Multiply
              (dx.Transform.World, Matrix.RotationY(y));

            dx.Transform.World = Matrix.Multiply
              (dx.Transform.World, Matrix.RotationZ(z));
        }

        public void TranslationMatrices(Device dx, float x, float y, float z)
        {
            dx.Transform.World = Matrix.Multiply
               (dx.Transform.World, Matrix.Translation(x, y, z));
        }

        public void ScaleMatrices(Device dx, float x, float y, float z)
        {
            dx.Transform.World = Matrix.Multiply
               (dx.Transform.World, Matrix.Scaling(x / 100, y / 100, z / 100));
        }

        /*
        private void DrawBox(float yaw, float pitch, float roll, float x, float y, float z)
        {
            angle += 0.01f;

            device.Transform.World = Matrix.RotationYawPitchRoll(yaw, pitch, roll) * Matrix.Translation(x, y, z);
            device.DrawIndexedPrimitives(PrimitiveType.TriangleList, 0, 0, verts.Length, 0, indices.Length / 3);
        }
        */

        /// <summary>
        /// render 3D objects
        /// </summary>
        /// <param name="d3d"></param>
        /// <param name="dx"></param>
        private void render3D(Device dx)
        {
            if ((initialised_3D) && (selected_display == MonoSLAM.DISPLAY_AUGMENTED_REALITY))
            {
                if ((test.enable_mapping) && (!test.calibrating))
                {
                    // Set the position of the camera
                    Vector3D camera_position = null;
                    SceneLibrary.Quaternion camera_orientation = null;
                    test.getCameraPositionOrientation(ref camera_position, ref camera_orientation);

                    dx.Transform.View = QuaternionToMatrixLookAt(camera_position, camera_orientation);

                    /*
                    Microsoft.DirectX.Quaternion q_orientation =
                        new Microsoft.DirectX.Quaternion((float)camera_orientation.GetX(),
                                     (float)camera_orientation.GetY(),
                                     (float)camera_orientation.GetZ(),
                                     (float)camera_orientation.GetR());
                    Matrix _matrixRotation = Matrix.RotationQuaternion(q_orientation);
                    float yaw = 0, pitch = 0, roll = 0;
                    camera_orientation.GetZYXEuler(ref pitch, ref yaw, ref roll);

                    dx.Transform.View = QuaternionToMatrixLookAt(camera_position, camera_orientation) *
                        Matrix.RotationYawPitchRoll(0,0,(float)(roll));
                    */


                    //dx.Material = InitMaterial(Color.White);

                    dx.Clear(ClearFlags.ZBuffer, Color.FromArgb(255, 0, 0, 0), 1.0f, 0);
                    //dx.Clear(ClearFlags.Target | ClearFlags.ZBuffer,Color.FromArgb(255, 0, 0, 0), 1.0f, 0);
                    //dx.Clear(ClearFlags.Target | ClearFlags.ZBuffer, Color.Transparent, 1.0f, 0);

                    if (background_image == null)
                        background_image = new D3D_background(dx, background_bitmap);
                    else
                        background_image.update(dx, background_bitmap);

                    background_image.Draw();


                    /*
                    dx.BeginScene();

                    dx.Transform.World = Matrix.Identity;
                    ScaleMatrices(dx, 1000, 0.1f, 0.1f);

                    dx.VertexFormat = Vertex.FVF_Flags;
                    dx.SetStreamSource(0, lineVertexBuffer, 0);

                    //dx.SetTexture(0, texture);

                    dx.DrawPrimitives(PrimitiveType.TriangleStrip, 0, 2);
                    dx.DrawPrimitives(PrimitiveType.TriangleStrip, 4, 2);

                    dx.EndScene();

                    // this try/catch prevents an error if the scene is being rendered 
                    // whilst the window is being closed
                    try
                    {
                        dx.Present();
                    }
                    catch
                    {
                    }


                    dx.BeginScene();

                    dx.Transform.World = Matrix.Identity;
                    ScaleMatrices(dx, 0.1f, 1000, 0.1f);

                    dx.VertexFormat = Vertex.FVF_Flags;
                    dx.SetStreamSource(0, lineVertexBuffer, 0);

                    //dx.SetTexture(0, texture);

                    dx.DrawPrimitives(PrimitiveType.TriangleStrip, 0, 2);
                    dx.DrawPrimitives(PrimitiveType.TriangleStrip, 4, 2);

                    dx.EndScene();

                    // this try/catch prevents an error if the scene is being rendered 
                    // whilst the window is being closed
                    try
                    {
                        dx.Present();
                    }
                    catch
                    {
                    }
                    */

                    dx.BeginScene();

                    dx.Transform.World = Matrix.Identity;
                    ScaleMatrices(dx, 0.1f, 0.1f, 1000);

                    dx.VertexFormat = Vertex.FVF_Flags;
                    dx.SetStreamSource(0, lineVertexBuffer, 0);

                    //dx.SetTexture(0, texture);

                    dx.DrawPrimitives(PrimitiveType.TriangleStrip, 0, 2);
                    dx.DrawPrimitives(PrimitiveType.TriangleStrip, 4, 2);

                    dx.EndScene();

                    // this try/catch prevents an error if the scene is being rendered 
                    // whilst the window is being closed
                    try
                    {
                        dx.Present();
                    }
                    catch
                    {
                    }


                    //float xrot = 0;
                    //float yrot = 0;
                    //float zrot = (float)Math.PI / 2;

                    dx.BeginScene();

                    dx.Transform.World = Matrix.Identity; // *Matrix.RotationYawPitchRoll((float)yaw, (float)pitch, (float)pitch);
                    //RotationMatrices(dx, xrot, yrot, zrot);
                    TranslationMatrices(dx, 0.0f, 2.5f, 6.0f);
                    ScaleMatrices(dx, 10, 10, 10);

                    dx.VertexFormat = Vertex.FVF_Flags;
                    dx.SetStreamSource(0, cubeVertexBuffer, 0);

                    dx.SetTexture(0, texture);

                    dx.DrawPrimitives(PrimitiveType.TriangleStrip, 0, 2);
                    dx.DrawPrimitives(PrimitiveType.TriangleStrip, 4, 2);
                    dx.DrawPrimitives(PrimitiveType.TriangleStrip, 8, 2);
                    dx.DrawPrimitives(PrimitiveType.TriangleStrip, 12, 2);
                    dx.DrawPrimitives(PrimitiveType.TriangleStrip, 16, 2);
                    dx.DrawPrimitives(PrimitiveType.TriangleStrip, 20, 2);

                    dx.EndScene();

                    // this try/catch prevents an error if the scene is being rendered 
                    // whilst the window is being closed
                    try
                    {
                        dx.Present();
                    }
                    catch
                    {
                    }

                }
            }
            if (!initialised_3D) Init3D();
        }

        #endregion

        private void augmentedRealityToolStripMenuItem_Click(object sender, EventArgs e)
        {
            trackedFeaturesToolStripMenuItem.Checked = false;
            overheadMToolStripMenuItem.Checked = false;
            searchEllipsesToolStripMenuItem.Checked = false;
            particleProbabilitiesToolStripMenuItem.Checked = false;
            augmentedRealityToolStripMenuItem.Checked = true;
            selected_display = MonoSLAM.DISPLAY_AUGMENTED_REALITY;
        }



    }
}