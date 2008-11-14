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
using System.Diagnostics;
using System.Collections.Generic;
using System.Text;
using sentience;
using SceneLibrary;
using monoSLAM;

namespace WindowsApplication1
{
    public class monoSLAM_test
    {
        public bool initialised = false;
        public bool enable_mapping = false;
        public float frames_per_second = 0;
        public float speed = 0;
        private bool show_ellipses = false;
        public bool simulation_mode = false;
        public bool calibrating = false;
        public bool frame_rate_warning = false;

        //calibration
        public float lens_distortion = 195;
        public float centre_of__distortion_x = 162;
        public float centre_of__distortion_y = 125;
        public float field_of_vision = 90;

        //the amount of time for which the known features were contunuously registered
        private float registered_seconds = 0;

        private Stopwatch stopwatch = new Stopwatch();

        MonoSLAMInterface monoslaminterface;

        // Important parameters
        public float delta_t = 1.0f / 30.0f;

        // Some constants which we could set with Glow sliders etc.
        public uint NUMBER_OF_FEATURES_TO_SELECT = 20;
        public uint NUMBER_OF_FEATURES_TO_KEEP_VISIBLE = 12;
        public uint MAX_FEATURES_TO_INIT_AT_ONCE = 1;
        public float MIN_LAMBDA = 0.5f;
        public float  MAX_LAMBDA = 5.0f;
        public uint NUMBER_OF_PARTICLES = 100;
        public float STANDARD_DEVIATION_DEPTH_RATIO  = 0.3f;
        public uint MIN_NUMBER_OF_PARTICLES = 30;
        public float PRUNE_PROBABILITY_THRESHOLD = 0.05f;
        public uint ERASE_PARTIALLY_INIT_FEATURE_AFTER_THIS_MANY_ATTEMPTS = 10;
        public float MAXIMUM_ANGLE_DIFFERENCE = 3.1415927f * 45.0f / 180.0f;

        float Graphics_Fku, Graphics_Fkv, Graphics_U0, Graphics_V0, Graphics_Kd1;

        // dimensions and distance to the calibration target
        public float calibration_target_width_mm = 210;
        public float calibration_target_height_mm = 148.5f;
        public float calibration_target_distance_mm = 600;

        public void init()
        {
            initialised = false;
            String path = System.Windows.Forms.Application.StartupPath + "\\";

            // Use standard model creators from class model_creators here
            MonoSLAM_Motion_Model_Creator mm_creator;
            MonoSLAM_Feature_Measurement_Model_Creator fmm_creator;

            //initialise the creator objects
            mm_creator = new MonoSLAM_Motion_Model_Creator();
            fmm_creator = new MonoSLAM_Feature_Measurement_Model_Creator();

            //create the interface
            String initialisation_file = "monoslam_state.ini";
            monoslaminterface = new MonoSLAMInterface(initialisation_file, path,
			    mm_creator,
			    fmm_creator,
			    null, // no internal measurement models used here
			    NUMBER_OF_FEATURES_TO_SELECT,
			    NUMBER_OF_FEATURES_TO_KEEP_VISIBLE,
			    MAX_FEATURES_TO_INIT_AT_ONCE,
			    MIN_LAMBDA,
			    MAX_LAMBDA,
			    NUMBER_OF_PARTICLES,
			    STANDARD_DEVIATION_DEPTH_RATIO,
			    MIN_NUMBER_OF_PARTICLES,
			    PRUNE_PROBABILITY_THRESHOLD,
		        ERASE_PARTIALLY_INIT_FEATURE_AFTER_THIS_MANY_ATTEMPTS,
                MAXIMUM_ANGLE_DIFFERENCE,
                calibration_target_width_mm,
                calibration_target_height_mm,
                calibration_target_distance_mm);

            SetUp3DDisplays();

            //has it initialised properly?
            if (monoslaminterface.GetScene() != null)
            {
                initialised = true;
            }
        }


        public void SetUp3DDisplays()
        {
            // Set display virtual camera parameters to match those of the real camera
            // being used in MonoSLAM
            Partially_Initialised_Feature_Measurement_Model default_pifmm =
                monoslaminterface.GetDefaultFeatureTypeForInitialisation();

            if (default_pifmm != null)
            {

                Line_Init_Wide_Point_Feature_Measurement_Model default_wide_pifmm =
                    (Line_Init_Wide_Point_Feature_Measurement_Model)default_pifmm;

                Graphics_Fku = default_wide_pifmm.get_camera().Fku();
                Graphics_Fkv = default_wide_pifmm.get_camera().Fkv();
                Graphics_U0 = default_wide_pifmm.get_camera().U0();
                Graphics_V0 = default_wide_pifmm.get_camera().V0();
                Graphics_Kd1 = default_wide_pifmm.get_camera().Kd1();

                // First display for external 3D view
                /*
                threedtool = new ThreeDToolGlowWidget(this,
                     controlpanel1->Width() + controlpanel2->Width(), 0,
                         monoslaminterface->GetCameraWidth(),
                         monoslaminterface->GetCameraHeight());
                threedtool->DrawEvent.Attach(this, &MonoSLAMGlow::Draw3D);
                threedtool->ProcessHitsEvent.Attach(this, &MonoSLAMGlow::ProcessHits);
                threedtool->SetCameraParameters(Graphics_Fku, Graphics_Fkv, 
                      Graphics_U0, Graphics_V0);
                 */


                // Set start position for GL camera in 3D tool
                // This is x, y, z position
                Vector rthreed = new Vector(0.0f, 0.2f, -1.5f);
                // This is camera orientation in my normal coordinate system
                // (z forward, y up, x left)
                Quaternion qWRthreed = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
                // We need to adjust by this rotation to fit GL coordinate frame
                // (z backward, y up, x right)
                // So rotate by pi about y axis
                Quaternion qROthreed = new Quaternion(0.0f, 1.0f, 0.0f, 0.0f);
                Quaternion qWOthreed = qWRthreed.Multiply(qROthreed);
                //threedtool.SetCameraPositionOrientation(rthreed, qWOthreed);

                // Second 3D display for images and augmented reality
                /*
                image_threedtool = new ThreeDToolGlowWidget(this,
                                           controlpanel1->Width() + controlpanel2->Width(), threedtool->Height(),
                                           monoslaminterface->GetCameraWidth(), monoslaminterface->GetCameraHeight());
                image_threedtool.DrawEvent.Attach(this, &MonoSLAMGlow::ImageDraw3D);
                image_threedtool.ProcessHitsEvent.Attach(this, &MonoSLAMGlow::ImageProcessHits);
                image_threedtool.SetCameraParameters(Graphics_Fku, Graphics_Fkv, Graphics_U0, Graphics_V0);
                */

                // Set up initial state of virtual camera for image display to match
                // state vector
                Scene_Single scene = monoslaminterface.GetScene();
                if (scene != null)
                {
                    Vector v = scene.get_xv();
                    if (v != null)
                    {
                        scene.get_motion_model().func_xp(v);
                        ThreeD_Motion_Model threed_motion_model = (ThreeD_Motion_Model)scene.get_motion_model();
                        Vector3D r = threed_motion_model.get_rRES();
                        Quaternion qWR = threed_motion_model.get_qRES();
                        // q is qWR between world frame and Scene robot frame
                        // We need to adjust by this rotation to fit GL coordinate frame
                        // (z backward, y up, x right)
                        // So rotate by pi about y axis
                        Quaternion qRO = new Quaternion(0.0f, 1.0f, 0.0f, 0.0f);
                        Quaternion qWO = qWR.Multiply(qRO);
                        //image_threedtool.SetCameraPositionOrientation(r, qWO);
                    }
                    else
                    {
                        Debug.WriteLine("Scene xp not defined");
                    }
                }
                else
                {
                    Debug.WriteLine("No scene object defined");
                }
            }
            else
            {
                Debug.WriteLine("No partially initialised feature measurement model found");
            }
        }


        /// <summary>
        /// one update
        /// </summary>
        /// <param name="img"></param>
        public void GoOneStep(classimage_mono img, classimage img_colour, classimage_mono output_img)
        {
            float delta_t;
            MAXIMUM_ANGLE_DIFFERENCE = 3.1415927f * (field_of_vision/2) / 180.0f;

            //set calibration parameters
            if (calibrating)
                monoslaminterface.set_camera_calibration(9e-06f, lens_distortion, lens_distortion, centre_of__distortion_x, centre_of__distortion_y, 1);

            //load in the current raw image
            Robot r = monoslaminterface.GetRobot();
            r.calibrating = (!enable_mapping) | calibrating;
            r.load_new_image(img, img_colour, output_img, show_ellipses);

            stopwatch.Stop();

            //get the elapsed time in seconds
            delta_t = stopwatch.ElapsedMilliseconds / 1000.0f;

            //if using the simulation set the frame rate at a fixed value
            if (simulation_mode) delta_t = 1.0f / 20.0f;

            if (delta_t > 0)
            {
                frames_per_second = (int)((1.0f / delta_t) * 10) / 10.0f;

                //report if its taking a long time
                frame_rate_warning = false;
                if (delta_t > 0.2)
                {
                    frame_rate_warning = true;
                    Debug.WriteLine("Time between frames (sec): " + Convert.ToString(delta_t));
                }

                //update the state of the system
                monoslaminterface.GoOneStep(delta_t, enable_mapping);
            }

            stopwatch.Reset();
            stopwatch.Start();

            speed = (int)(monoslaminterface.Speed() * 100) / 100.0f;

            if ((enable_mapping) && (monoslaminterface.number_of_matched_features < 2))
            {
                enable_mapping = false;
                init();
            }

            if ((!enable_mapping) && (monoslaminterface.number_of_matched_features >= 3) && (!calibrating))
            {
                registered_seconds += delta_t;
                if (registered_seconds > 1)
                {
                    Debug.WriteLine("Ready to go");
                    enable_mapping = true;
                    registered_seconds = 0;
                }                
            }
            else
            {
                if (registered_seconds > 0) Debug.WriteLine("Waiting for initial registration");
                registered_seconds = 0;
            }
        }

        /// <summary>
        /// returns the current camera position and orientation
        /// </summary>
        /// <param name="position"></param>
        /// <param name="orientation"></param>
        public void getCameraPositionOrientation(ref Vector3D position, ref Quaternion orientation)
        {
            monoslaminterface.GetCameraPositionOrientation(ref position, ref orientation);
        }

        /// <summary>
        /// return the value of a benchmark timer
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        public int getBenchmark(int index)
        {
            return ((int)monoslaminterface.benchmark[index]);
        }

        /// <summary>
        /// show detected features within the given image
        /// </summary>
        /// <param name="img"></param>
        public void ShowFeatures(classimage_mono img, int feature_type)
        {
            show_ellipses = false;
            switch (feature_type)
            {
                case MonoSLAM.DISPLAY_FEATURES:
                    {
                        monoslaminterface.ShowFeatures(img);
                        break;
                    }
                case MonoSLAM.DISPLAY_ELLIPSES:
                    {
                        show_ellipses = true;
                        monoslaminterface.ShowFeatures(img);                        
                        break;
                    }
                case MonoSLAM.DISPLAY_PROBABILITIES:
                    {
                        monoslaminterface.showProbabilities(0, img);
                        //monoslaminterface.ShowTriangles(img);
                        //monoslaminterface.ShowLineHistory(img);
                        //monoslaminterface.ShowMesh(img);
                        break;
                    }
                case MonoSLAM.DISPLAY_MAP:
                    {
                        monoslaminterface.ShowOverheadView(img, 2.5f, 2.5f);
                        break;
                    }
            }
        }

    }
}
