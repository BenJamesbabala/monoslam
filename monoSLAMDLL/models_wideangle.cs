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
using System.Collections;
using System.Collections.Generic;
using System.Text;
using SceneLibrary;
using sentience;

namespace monoSLAM
{
// Model for wide angle camera making point measurements
//
// Frames of reference:
// W: world frame
// R: robot (= camera) frame
//
// Spatial vectors: satisfy yi = r + hLi
// r:   robot spatial position vector
// hLi: spatial measurement vector
// yi:  feature position vector
//
// Spatial vectors with frame label is component vector in that frame
//
// Rotation matrices: RAB transforms the component representation of a 
// spatial vector from one frame to the other such that
// vA = RAB * vB
//
// Special vectors: are not real vectors but lists of parameters
// xp: robot position vector = (rW )
//                             (qWR)
// hi:  measurement vector in image coordinates = (u)
//                                                (v)
// ypi: partially initialised feature position vector = (rWi    )
//                                                      (hLhatWi)
//      This represents the position of a line, which is all we know
//      about the position of a feature after only one measurement.
//      Points on the line are described by rWi + constant * hLhatWi
//      rWi is the camera position when the the feature was observed
//      hLhatWi is the normalised direction vector lying along the line


    public class Camera_Constants
    {
        public const float WIDE_MAXIMUM_LENGTH_RATIO = 2.0f;
        public const float WIDE_MAXIMUM_ANGLE_DIFFERENCE = 3.1415927f * 45.0f / 180.0f;
        public const float WIDE_IMAGE_SEARCH_BOUNDARY = 20.0f;
        public const float WIDE_SD_IMAGE_SIMULATION = 1.0f;

        public const String initial_state_file = "initial_state_cameravw";
        public const String known_features_file = "known_features_cameravw";

        public const uint BOXSIZE = 11;      ///< Side length of square image patches
        public const String known_point_patch_stem = "known_patch";

        public const float SEMI_INFINITE_LINE_LENGTH = 10.0f;
        public const float COVARIANCES_NUMBER_OF_SIGMA = 3;
        public const int DRAW_N_OVERLAPPING_ELLIPSES = 10;

        public const bool DEBUGDUMP = true;
    }


    /// <summary>
    /// Base class for feature measurement models that use a camera.
    /// </summary>
    public class Camera_Feature_Measurement_Model : Camera_Feature_Measurement_Model_base
    {        
        /// Returns the camera model. The camera is not stored in this class
        /// since derived classes might want their own specific camera models,
        /// but they must provide this function to return the camera to the user.
        virtual public WideCamera get_camera() { return(null); }

    }


    /// <summary>
    /// Base class for point feature measurement models that use a wide-angle camera.
    /// </summary>
    public class Wide_Camera_Point_Feature_Measurement_Model : Camera_Feature_Measurement_Model
    {  
        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="motion_model">The motion model describing the robot motion. This must be derived from ThreeD_Motion_Model</param>
        /// <param name="cam">The camera model to use</param>
        /// <param name="MAXIMUM_LENGTH_RATIO_"></param>
        /// <param name="MAXIMUM_ANGLE_DIFFERENCE_"></param>
        /// <param name="IMAGE_SEARCH_BOUNDARY_"></param>
        /// <param name="SD_IMAGE_SIMULATION_"></param>       
        public Wide_Camera_Point_Feature_Measurement_Model(
                   Motion_Model motion_model,
                   WideCamera cam,
                   float MAXIMUM_LENGTH_RATIO_,
                   float MAXIMUM_ANGLE_DIFFERENCE_,
                   float IMAGE_SEARCH_BOUNDARY_,
                   float SD_IMAGE_SIMULATION_)
        {
            MAXIMUM_LENGTH_RATIO = MAXIMUM_LENGTH_RATIO_;
            MAXIMUM_ANGLE_DIFFERENCE = MAXIMUM_ANGLE_DIFFERENCE_;
            IMAGE_SEARCH_BOUNDARY = IMAGE_SEARCH_BOUNDARY_;
            SD_IMAGE_SIMULATION = SD_IMAGE_SIMULATION_;
            m_camera = cam;
  
            if ((String)(motion_model.motion_model_dimensionality_type) != "THREED")
            {
                String os;
                os = "Motion Model given to the Feature Measurement Model is of type " + 
                     motion_model.motion_model_dimensionality_type +
                     ", not type THREED.";
                Debug.WriteLine(os);
                //throw Scene::InitialisationError(os.str());
            }
            threed_motion_model = (ThreeD_Motion_Model)motion_model;
        }


        // Read the parameters of the camera from the settings.
        public void read_parameters(Settings.Section section)
        {
            ArrayList values = section.get_entry("CameraParameters");            
            if (values != null)
            {
                String param = (String)values[0];
                m_camera.ReadASCII(param);
            }
            else
            {
                String error;
                error = "No CameraParameters entry found in initialisation file section [" +
                        section.Label() + "].";
                Debug.WriteLine(error);
            }
        }


        /// Returns the camera model.  
        public override WideCamera get_camera() {return m_camera;}
  
        // For visibility tests: we expect to be able to measure feature if
        // distance to feature / distance when initialised is in the range
        // 1/MAXIMUM_LENGTH_RATIO to MAXIMUM_LENGTH_RATIO. There is also a test
        // on MAXIMUM_ANGLE_DIFFERENCE, and both need to pass.  
        public float MAXIMUM_LENGTH_RATIO;

        // For visibility tests: we expect to be able to measure feature if
        // the absolute angle between current viewpoint vector and the
        // initialisation viewpoint vector is less than
        // MAXIMUM_ANGLE_DIFFERENCE. There is also a test on
        // MAXIMUM_LENGTH_RATIO, and both need to pass. 
        public float MAXIMUM_ANGLE_DIFFERENCE;

        // Don't try to search this close to the edge of the image
        public float IMAGE_SEARCH_BOUNDARY;

        // The measurement standard deviation to use for simulations
        public float SD_IMAGE_SIMULATION;

        // The return flag from visibility_test() ORs these together
        // to indicate the reasons for failure.
        public const int LEFT_RIGHT_FAIL = 1;
        public const int UP_DOWN_FAIL = 2;
        public const int DISTANCE_FAIL = 4;
        public const int ANGLE_FAIL = 8;
        public const int BEHIND_CAMERA_FAIL = 16;

        public WideCamera m_camera; ///< The camera model

        // This class assumes 3D motion models. This is checked in the constructor,
        // cast to a ThreeD_Motion_Model, and set at that point.

        public ThreeD_Motion_Model threed_motion_model;
    }


    /// <summary>
    /// Model for a wide-angle camera making point measurements. This derives from
    /// Fully_Initialised_Feature_Measurement_Model, registering a feature graphics
    /// type of THREED_POINT. The feature state  y_i = (x,y,z) , i.
    /// e. a three-vector giving the 3D world position of the feature. The measurement
    /// state  h_i = (u,v)  is the image location of the feature.
    /// </summary>
    public class Fully_Init_Wide_Point_Feature_Measurement_Model :  
                     Fully_Initialised_Feature_Measurement_Model
                     //Wide_Camera_Point_Feature_Measurement_Model 
    {
        private Random rnd = new Random();

        public Fully_Init_Wide_Point_Feature_Measurement_Model(Motion_Model motion_model, float WIDE_MAXIMUM_ANGLE_DIFFERENCE)
            : base(2, 3, 3, motion_model, "CAMERA_WIDE_POINT", "THREED_POINT")
        {
            WideCamera cam = new WideCamera();
            float MAXIMUM_LENGTH_RATIO_ = Camera_Constants.WIDE_MAXIMUM_LENGTH_RATIO;
            float MAXIMUM_ANGLE_DIFFERENCE_ = WIDE_MAXIMUM_ANGLE_DIFFERENCE; //Camera_Constants.WIDE_MAXIMUM_ANGLE_DIFFERENCE;
            float IMAGE_SEARCH_BOUNDARY_ = Camera_Constants.WIDE_IMAGE_SEARCH_BOUNDARY;
            float SD_IMAGE_SIMULATION_ = Camera_Constants.WIDE_SD_IMAGE_SIMULATION;
            wide_model = new Wide_Camera_Point_Feature_Measurement_Model(motion_model, cam, MAXIMUM_LENGTH_RATIO_, MAXIMUM_ANGLE_DIFFERENCE_, IMAGE_SEARCH_BOUNDARY_, SD_IMAGE_SIMULATION_);
        }

        /// <summary>
        /// set camera calibration
        /// </summary>
        /// <param name="Kd1"></param>
        /// <param name="Fku"></param>
        /// <param name="Fkv"></param>
        /// <param name="U0"></param>
        /// <param name="V0"></param>
        /// <param name="measurement_sd"></param>
        public void set_calibration(float Kd1, float Fku, float Fkv,
                                    float U0, float V0, float measurement_sd)
        {
            ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).get_camera().set_calibration(Kd1, Fku, Fkv, U0, V0, measurement_sd);
        }

        // get the camera object
        public WideCamera get_camera() { return (((Wide_Camera_Point_Feature_Measurement_Model)wide_model).get_camera()); }

        /// <summary>
        /// Model for single camera making point measurements
        /// </summary>
        /// <param name="motion_model">The motion model describing the robot motion. This must be derived from ThreeD_Motion_Model</param>
        /// <param name="cam">The camera model to use</param>
        /// <param name="MAXIMUM_LENGTH_RATIO_"></param>
        /// <param name="MAXIMUM_ANGLE_DIFFERENCE_"></param>
        /// <param name="IMAGE_SEARCH_BOUNDARY_"></param>
        /// <param name="SD_IMAGE_SIMULATION_"></param>
        public Fully_Init_Wide_Point_Feature_Measurement_Model(
                   Motion_Model motion_model, WideCamera cam, float MAXIMUM_LENGTH_RATIO_,
                   float MAXIMUM_ANGLE_DIFFERENCE_, float IMAGE_SEARCH_BOUNDARY_,
                   float SD_IMAGE_SIMULATION_)
        :
            base(2, 3, 3, motion_model, "CAMERA_WIDE_POINT", "THREED_POINT")
        {
            wide_model = new Wide_Camera_Point_Feature_Measurement_Model(motion_model, cam, MAXIMUM_LENGTH_RATIO_, MAXIMUM_ANGLE_DIFFERENCE_, IMAGE_SEARCH_BOUNDARY_, SD_IMAGE_SIMULATION_);
        }


        // Initialises the base classes by calling
        // Fully_Initialised_Feature_Measurement_Model::read_parameters() and
        // Wide_Camera_Point_Feature_Measurement_Model::read_parameters().        
        public override void read_parameters(Settings settings)
        {
            // Initialise any base class parameters
            base.read_parameters(settings);

            // Initialise my camera model
            Settings.Section section = settings.get_section(feature_type);
            if (section == null)
            {
                
                Debug.WriteLine("Warning: no [" + feature_type + 
                                "] section in the intialisation data. " +
                                "The feature measurement model will be initialised with the default " +
                                "camera parameters.");
            }
            else
            {
                // Read the camera model. This will be the name of another section
                ArrayList values = section.get_entry("Camera");
                if (values == null)
                {
                    Debug.WriteLine("Warning: no Camera entry in the [" + feature_type +
                                    "] section in the intialisation data. " +
                                    "The feature measurement model will be initialised with the " +
                                    "default camera parameters.");
                }
                else
                {
                    String camera = (String)values[0];
                    section = settings.get_section(camera);
                    if (section == null)
                    {
                        Debug.WriteLine("Warning: no [" + camera + "] section in the intialisation data. " +
                                        "The feature measurement model will be initialised with the " +
                                        " default camera parameters.");
                    }
                    else
                        ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).read_parameters(section);
                }
            }
        }
        

        // In this case the graphics representation  y_i^{graphics}  and its
        // covariance is the same as the feature state  y_i  and covariance.
        public override void func_yigraphics_and_Pyiyigraphics(Vector yi, MatrixFixed Pyiyi)
        {
            // The graphics representation is the same as the state 
            yigraphicsRES.Update(yi);
            PyiyigraphicsRES.Update(Pyiyi);
        }


        public override void func_zeroedyigraphics_and_Pzeroedyigraphics(
                        Vector yi, Vector xv, MatrixFixed Pxx,
                        MatrixFixed Pxyi, MatrixFixed Pyiyi)
        {
            ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).threed_motion_model.func_xp(xv);

            // In this case (where the feature state is the same as the graphics
            // state) zeroedyigraphics is the same as zeroedyi
            func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi,
                      ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).threed_motion_model.get_xpRES());
            zeroedyigraphicsRES.Update(zeroedyiRES);

            MatrixFixed dzeroedyigraphics_by_dxv = dzeroedyi_by_dxpRES * 
                ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).threed_motion_model.get_dxp_by_dxvRES();

  
            PzeroedyigraphicsRES.Update(dzeroedyigraphics_by_dxv * Pxx * dzeroedyigraphics_by_dxv.Transpose() +
                                        dzeroedyi_by_dyiRES * Pxyi.Transpose() * dzeroedyigraphics_by_dxv.Transpose() +
                                        dzeroedyigraphics_by_dxv * Pxyi * dzeroedyi_by_dyiRES.Transpose() +
                                        dzeroedyi_by_dyiRES * Pyiyi * dzeroedyi_by_dyiRES.Transpose());
        }



        public override void func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(Vector yi, Vector xp)
        {
            ThreeD_Motion_Model mm = ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).threed_motion_model;

            // Extract cartesian and quaternion components of xp
            mm.func_r(xp);
            mm.func_q(xp);

            Vector3D yWiminusrW = new Vector3D((new Vector3D(yi)) -(mm.get_rRES()));

            Quaternion qRES = mm.get_qRES();
            Quaternion qRW = qRES.Inverse();
            MatrixFixed dqRW_by_dq = MatrixFixed.dqbar_by_dq();

            // Rotation RRW
            RotationMatrix RRW = qRW.RotationMatrix();

            // Position of feature relative to robot in robot frame
            Vector3D zeroedyi = new Vector3D(RRW * yWiminusrW);
            zeroedyiRES.Update(zeroedyi.GetVNL3());

            // Now calculate Jacobians
            // dzeroedyi_by_dyi is RRW
            dzeroedyi_by_dyiRES.Update(RRW);

            // dzeroedyi_by_dxp has 2 partitions:
            // dzeroedyi_by_dr (3 * 3)
            // dzeroedyi_by_dq (3 * 4)
            MatrixFixed dzeroedyi_by_dr = (MatrixFixed)(RRW);
            dzeroedyi_by_dr *= -1.0f;

            //These should be 3x4 dimension
            MatrixFixed dzeroedyi_by_dqRW = MatrixFixed.dRq_times_a_by_dq(qRW, yWiminusrW);
            MatrixFixed dzeroedyi_by_dq = dzeroedyi_by_dqRW * dqRW_by_dq;

            dzeroedyi_by_dxpRES.Update(dzeroedyi_by_dr, 0, 0);
            dzeroedyi_by_dxpRES.Update(dzeroedyi_by_dq, 0, 3);
        }

        public override void func_Ri(Vector hi)
        {
            RiRES = ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).m_camera.MeasurementNoise(hi);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="xp">current robot position state</param>
        /// <param name="yi">3D position of the feature</param>
        /// <param name="xp_orig">robot position state when the feature was initially observed</param>
        /// <param name="hi">image coordinates of the feature</param>
        /// <returns></returns>
        public override uint visibility_test(Vector xp, Vector yi, Vector xp_orig, Vector hi)
        {
            float image_search_boundary = ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).IMAGE_SEARCH_BOUNDARY;
            uint wdth = ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).m_camera.ImageWidth();
            uint hght = ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).m_camera.ImageHeight();
            uint cant_see_flag = 0;
  
            // Test image boundaries 
            float x = hi[0];
            float y = hi[1];
            if ((x < 0.0f + image_search_boundary) ||
                (x > (float)(wdth - 1 - image_search_boundary))) 
            {
                cant_see_flag |= Wide_Camera_Point_Feature_Measurement_Model.LEFT_RIGHT_FAIL;
                //if (Camera_Constants.DEBUGDUMP) Debug.WriteLine("Visibility test left / right fail.");
            }
            if ((y < 0.0f + image_search_boundary) ||
                (y > (float)(hght - 1 - image_search_boundary))) 
            {
                cant_see_flag |= Wide_Camera_Point_Feature_Measurement_Model.UP_DOWN_FAIL;
                //if (Camera_Constants.DEBUGDUMP) Debug.WriteLine("Visibility test up / down fail.");
            }

            // Do tests on length and angle of predicted view 

            // hLWi is current predicted vector from head to feature in 
            // world frame

            // This function gives relative position of feature
            func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, xp);

            // Test the feature's not behind the camera (because projection
            // may do strange things)
            if (zeroedyiRES[2] <= 0) 
            {
                cant_see_flag |= Wide_Camera_Point_Feature_Measurement_Model.BEHIND_CAMERA_FAIL;
                //if (Camera_Constants.DEBUGDUMP) Debug.WriteLine("Behind camera fail.");
            }

            ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).threed_motion_model.func_q(xp);
            RotationMatrix RWR = ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).threed_motion_model.get_qRES().RotationMatrix();

            Vector3D hLWi = new Vector3D(RWR * (new Point3D(zeroedyiRES)));

            // hLWi_orig is vector from head to feature in world frame
            // WHEN THAT FEATURE WAS FIRST MEASURED: i.e. when the image
            // patch was saved
            func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, xp_orig);

            ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).threed_motion_model.func_q(xp_orig);
            RotationMatrix RWR_orig = ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).threed_motion_model.get_qRES().RotationMatrix();
  
            Vector3D hLWi_orig = new Vector3D(RWR_orig * (new Point3D(zeroedyiRES)));

            // Compare hLWi and hLWi_orig for length and orientation
            float mod_hLWi = hLWi.Norm();
            float mod_hLWi_orig = hLWi_orig.Norm();

            float length_ratio = mod_hLWi / mod_hLWi_orig;
            float max_length_ratio = ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).MAXIMUM_LENGTH_RATIO;
            if ((length_ratio > max_length_ratio) ||
                 (length_ratio < (1.0f / max_length_ratio))) 
            {
                cant_see_flag |= Wide_Camera_Point_Feature_Measurement_Model.DISTANCE_FAIL;
                //if (Camera_Constants.DEBUGDUMP) Debug.WriteLine("Distance fail.");
            }

            float dot_prod = hLWi * hLWi_orig;

            float angle = (float)Math.Acos(dot_prod / (mod_hLWi * mod_hLWi_orig));
            if (angle == float.NaN) Debug.WriteLine("Maths error: " + dot_prod + " / " + (mod_hLWi * mod_hLWi_orig));
            angle = (angle >= 0.0f ? angle : -angle);  // Make angle positive
            if (angle > ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).MAXIMUM_ANGLE_DIFFERENCE) 
            {
                cant_see_flag |= Wide_Camera_Point_Feature_Measurement_Model.ANGLE_FAIL;
                //if (Camera_Constants.DEBUGDUMP) Debug.WriteLine("Angle fail.");
            }

            return cant_see_flag;   // 0 if OK, otherwise error code
        }
        
        public override float selection_score(MatrixFixed Si)
        {
            // Return the trace of the innovation covariance
            return SceneLib.Trace(Si);
        }


        public override void func_hi_and_dhi_by_dxp_and_dhi_by_dyi(Vector yi, Vector xp)
        {
            // This function gives relative position of feature: also call this hR
            // (vector from camera to feature in robot frame)
            func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, xp);

            // Project this from 3D into the 2D image using our camera
            Vector feature_3D_position = get_zeroedyiRES();
            WideCamera cam = ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).m_camera;
            hiRES = cam.Project(feature_3D_position);

            // And ask the camera what the Jacobian of this projection was
            MatrixFixed dhid_by_dzeroedyi = cam.ProjectionJacobian();

            // Form the required Jacobians
            dhi_by_dxpRES = dhid_by_dzeroedyi * dzeroedyi_by_dxpRES;
            dhi_by_dyiRES = dhid_by_dzeroedyi * dzeroedyi_by_dyiRES;
        }


        public override void func_nui(Vector hi, Vector zi)
        {
            nuiRES.Update(zi - hi);
        }

        public override void func_hi_noisy(Vector yi_true, Vector xp_true)
        {
            func_hi_and_dhi_by_dxp_and_dhi_by_dyi(yi_true, xp_true);

            hi_noisyRES.Put(0, SceneLib.SampleNormal(hiRES[0], ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).SD_IMAGE_SIMULATION, rnd));
            hi_noisyRES.Put(1, SceneLib.SampleNormal(hiRES[1], ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).SD_IMAGE_SIMULATION, rnd));
        }

    }





    /// <summary>
    /// Feature measurement model for the partially-initialised stage of point.
    /// When we've only seen a feature once we initialise the 3D ray it lies on
    /// as a SLAM feature. A partially initialised feature has a state
    /// \f[
    /// \vct{y}_{pi} = \evct{\vct{r}_i^W \\ \hat{\vct{h}}_{Li}^W}
    /// \f]
    /// where  \vct{r}_i^W = \vct{x}_P^{orig}  is the camera position when the
    /// feature was observed, and  \hat{\vct{h}}_{Li}^W  is the normalised
    /// direction vector towards the feature. The 3D ray that the feature lies on is
    /// defined by
    /// \f[
    /// \vct{y}_i = \vct{r}_i^W + \lambda \hat{\vct{h}}_{Li}^W
    /// \f]
    /// with  \lambda  being the free parameter which defines the depth of the feature.
    /// </summary>
    public class Line_Init_Wide_Point_Feature_Measurement_Model :
                     Partially_Initialised_Feature_Measurement_Model
                     //Wide_Camera_Point_Feature_Measurement_Model
    {

        // Splitting out the ri and hhati components of ypi
        protected Vector3D riRES = new Vector3D(0,0,0);
        protected Vector3D hhatiRES = new Vector3D(0,0,0);

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="motion_model">The motion model describing the robot motion. This must be derived from ThreeD_Motion_Model</param>
        /// <param name="fully_init_f_m_m">The type of feature measurement model to convert to when the feature is fully initialised.</param>
        /// <param name="cam">The camera model to use</param>
        /// <param name="MAXIMUM_LENGTH_RATIO_"></param>
        /// <param name="MAXIMUM_ANGLE_DIFFERENCE_"></param>
        /// <param name="IMAGE_SEARCH_BOUNDARY_"></param>
        /// <param name="SD_IMAGE_SIMULATION_"></param>
        public Line_Init_Wide_Point_Feature_Measurement_Model(Motion_Model motion_model,
                                                              Feature_Measurement_Model fully_init_f_m_m,
                                                              WideCamera cam, 
                                                              float MAXIMUM_LENGTH_RATIO_,
                                                              float MAXIMUM_ANGLE_DIFFERENCE_,
                                                              float IMAGE_SEARCH_BOUNDARY_,
                                                              float SD_IMAGE_SIMULATION_) :
               base(2, 6, 6, motion_model, "CAMERA_WIDE_LINE_INIT", "THREED_SEMI_INFINITE_LINE", 1, fully_init_f_m_m)
        {
            wide_model = new Wide_Camera_Point_Feature_Measurement_Model(motion_model, cam, MAXIMUM_LENGTH_RATIO_, MAXIMUM_ANGLE_DIFFERENCE_, IMAGE_SEARCH_BOUNDARY_, SD_IMAGE_SIMULATION_);
        }

        public Line_Init_Wide_Point_Feature_Measurement_Model(Motion_Model motion_model,
                                                              Feature_Measurement_Model fully_init_f_m_m) :
            base(2, 6, 6, motion_model, "CAMERA_WIDE_LINE_INIT", "THREED_SEMI_INFINITE_LINE", 1, fully_init_f_m_m)
        {
            WideCamera cam = new WideCamera();
            float MAXIMUM_LENGTH_RATIO_ = Camera_Constants.WIDE_MAXIMUM_LENGTH_RATIO;
            float MAXIMUM_ANGLE_DIFFERENCE_ = Camera_Constants.WIDE_MAXIMUM_ANGLE_DIFFERENCE;
            float IMAGE_SEARCH_BOUNDARY_ = Camera_Constants.WIDE_IMAGE_SEARCH_BOUNDARY;
            float SD_IMAGE_SIMULATION_ = Camera_Constants.WIDE_SD_IMAGE_SIMULATION;
            wide_model = new Wide_Camera_Point_Feature_Measurement_Model(motion_model, cam, MAXIMUM_LENGTH_RATIO_, MAXIMUM_ANGLE_DIFFERENCE_, IMAGE_SEARCH_BOUNDARY_, SD_IMAGE_SIMULATION_);
        }

        // get the camera object
        public WideCamera get_camera() { return (((Wide_Camera_Point_Feature_Measurement_Model)wide_model).get_camera()); }

        /// <summary>
        /// sets camera calibration parameters
        /// </summary>
        /// <param name="Kd1"></param>
        /// <param name="Fku"></param>
        /// <param name="Fkv"></param>
        /// <param name="U0"></param>
        /// <param name="V0"></param>
        /// <param name="measurement_sd"></param>
        public void set_calibration(float Kd1, float Fku, float Fkv,
                            float U0, float V0, float measurement_sd)
        {
            ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).get_camera().set_calibration(Kd1, Fku, Fkv, U0, V0, measurement_sd);
        }


        // Redefined virtual functions
        public override void read_parameters(Settings settings)
        {
                // Initialise any base class parameters
                base.read_parameters(settings);

                // Initialise my camera model
                Settings.Section section = settings.get_section(feature_type);
                if(section == null)
                {
                    Debug.WriteLine("Warning: no [" + feature_type +
                                    "] section in the intialisation data. " +
                                    "The feature measurement model will be initialised with the default " +
                                    "camera parameters.");
                }
                else
                {
                    // Read the camera model. This will be the name of another section
                    ArrayList values = section.get_entry("Camera");                    
                    if (values == null)
                    {
                        Debug.WriteLine("Warning: no Camera entry in the [" + feature_type + 
                                        "] section in the intialisation data. " +
                                        "The feature measurement model will be initialised with the " +
                                        "default camera parameters.");
                    }
                    else
                    {
                        String camera = (String)values[0];
                        section = settings.get_section(camera);
                        if(section == null)
                        {
                            Debug.WriteLine("Warning: no [" + camera +
                                            "] section in the intialisation data. " +
                                            "The feature measurement model will be initialised with the " +
                                            " default camera parameters.");
                        }
                        else
                            ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).read_parameters(section);
                    }
                }
        }


        public override void func_yigraphics_and_Pyiyigraphics(Vector yi, MatrixFixed Pyiyi)
        {
            yigraphicsRES.Update(yi);
            PyiyigraphicsRES.Update(Pyiyi);
        }


        public override void func_zeroedyigraphics_and_Pzeroedyigraphics(Vector yi, Vector xv,
	                    MatrixFixed Pxx, MatrixFixed Pxyi, MatrixFixed Pyiyi)
        {
            ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).threed_motion_model.func_xp(xv);

            // In this case (where the feature state is the same as the graphics
            // state) zeroedyigraphics is the same as zeroedyi
            func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).threed_motion_model.get_xpRES());
            zeroedyigraphicsRES.Update(zeroedyiRES);

            MatrixFixed dzeroedyigraphics_by_dxv = dzeroedyi_by_dxpRES * ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).threed_motion_model.get_dxp_by_dxvRES();

            PzeroedyigraphicsRES.Update(dzeroedyigraphics_by_dxv * Pxx * dzeroedyigraphics_by_dxv.Transpose() +
                                        dzeroedyi_by_dyiRES * Pxyi.Transpose() * dzeroedyigraphics_by_dxv.Transpose() +
                                        dzeroedyigraphics_by_dxv * Pxyi * dzeroedyi_by_dyiRES.Transpose() +
                                        dzeroedyi_by_dyiRES * Pyiyi * dzeroedyi_by_dyiRES.Transpose());
        }



        public override void func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(Vector yi, Vector xp)
        {
            Wide_Camera_Point_Feature_Measurement_Model wm = (Wide_Camera_Point_Feature_Measurement_Model)wide_model;

            // Extract cartesian and quaternion components of xp
            wm.threed_motion_model.func_r(xp);
            wm.threed_motion_model.func_q(xp);

            // Extract ri and hhati components of yi = ypi
            func_ri(yi);
            func_hhati(yi);

            // ri part: transformation is just the same as in the normal point case
            // zeroedri = RRW(rWi - rW)  //commented out in original code

            // ri - r
            Vector3D yWiminusrW = new Vector3D(riRES - wm.threed_motion_model.get_rRES());

            Quaternion qRW = wm.threed_motion_model.get_qRES().Inverse();
            MatrixFixed dqRW_by_dq = MatrixFixed.dqbar_by_dq();

            // Rotation RRW
            RotationMatrix RRW = qRW.RotationMatrix();

            // RRW(rWi - rW)
            Vector3D zeroedri = new Vector3D(RRW * yWiminusrW);

            // Now calculate Jacobians
            // dzeroedri_by_dri is RRW
            // dzeroedri_by_dhhati = 0
            MatrixFixed dzeroedri_by_dri = new MatrixFixed(RRW);

            // dzeroedyi_by_dxp:
            // dzeroedri_by_dr = -RRW
            // dzeroedri_by_dq = d_dq(RRW (ri - r))
            MatrixFixed dzeroedri_by_dr = RRW * -1.0f;

            MatrixFixed dzeroedri_by_dqRW = MatrixFixed.dRq_times_a_by_dq(qRW, yWiminusrW);
            MatrixFixed dzeroedri_by_dq = dzeroedri_by_dqRW * dqRW_by_dq;

            // Now for the hhati part (easier...)
            // zeroedhhati = RRW hhati
            Vector3D zeroedhhati = new Vector3D(RRW * hhatiRES);

            // Jacobians
            // dzeroedhhati_by_dr = 0
            // dzeroedhhati_by_dq = d_dq(RRW hhati)
            // dzeroedhhati_by_dhhati = RRW
            // dzeroedhhati_by_dri = 0
            MatrixFixed dzeroedhhati_by_dqRW = MatrixFixed.dRq_times_a_by_dq(qRW, hhatiRES);
            MatrixFixed dzeroedhhati_by_dq = dzeroedhhati_by_dqRW * dqRW_by_dq;
            MatrixFixed dzeroedhhati_by_dhhati = new MatrixFixed(RRW);

            // And put it all together
            zeroedyiRES.Update(zeroedri.GetVNL3(), 0);
            zeroedyiRES.Update(zeroedhhati.GetVNL3(), 3);

            //cout << "Line: zeroedri = " << zeroedri << "zeroedhhati = " << zeroedhhati;

            dzeroedyi_by_dxpRES.Fill(0.0f);
            dzeroedyi_by_dxpRES.Update(dzeroedri_by_dr, 0, 0);
            dzeroedyi_by_dxpRES.Update(dzeroedri_by_dq, 0, 3);
            dzeroedyi_by_dxpRES.Update(dzeroedhhati_by_dq, 3, 3);

            dzeroedyi_by_dyiRES.Fill(0.0f);
            dzeroedyi_by_dyiRES.Update(dzeroedri_by_dri, 0, 0);
            dzeroedyi_by_dyiRES.Update(dzeroedhhati_by_dhhati, 3, 3);
              
            
        }


        public override void func_Ri(Vector hi)
        {
            RiRES = ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).m_camera.MeasurementNoise(hi);
        }

        public override uint visibility_test(Vector v, Vector v2,
                                    Vector v3, Vector v4)
        {
            // Always visible for now
            return 0;
        }

        public override float selection_score(MatrixFixed m)
        {
            // Always measureable for now
            return 100000.0f;
        }

        /// <summary>
        /// Partial initialisation function
        /// Calculates the state vector  \vct{y}_{pi}  and Jacobians for a new
        /// partially-initialised feature from an initial observation.
        /// @param hi The initial measurement  \vct{h}_i , in this case the (x,y)
        ///  image location of the feature.
        /// </summary>
        /// <param name="hi"></param>
        /// <param name="xp">The robot position state  \vct{x}_p  from which the initial observation was made.</param>
        public override void func_ypi_and_dypi_by_dxp_and_dypi_by_dhi_and_Ri(Vector hi, Vector xp)
        {
            // Representation of a line here:
            // ypi = (rWi    )
            //       (hLhatWi)

            // Form the ray (in camera co-ordinates by unprojecting this image location
            Vector3D hLRi = new Vector3D(((Wide_Camera_Point_Feature_Measurement_Model)wide_model).m_camera.Unproject(hi));
  
            //if (Camera_Constants.DEBUGDUMP) Debug.WriteLine("debug hLRi");

            // Form hLhatRi from hLRi
            // Normalise
            Vector3D hLhatRi = hLRi;
            hLhatRi.Normalise();

            MatrixFixed dhLhatRi_by_dhLRi = MatrixFixed.dvnorm_by_dv(hLRi); 

            //if (Camera_Constants.DEBUGDUMP) Debug.WriteLine("debug hLhatRi" + hLhatRi);

            // Now convert this into a direction in world co-ordinates by rotating
            // Form hLhatWi from hLhatRi
            // Rotate
            ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).threed_motion_model.func_q(xp);
            RotationMatrix RWR = ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).threed_motion_model.get_qRES().RotationMatrix();
            Vector3D hLhatWi = new Vector3D(RWR * hLhatRi);

            //if (Camera_Constants.DEBUGDUMP) Debug.WriteLine("debug hLhatWi" + hLhatWi);

            // Extract rW from xp
            ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).threed_motion_model.func_r(xp);
 
            // And form ypiRES
            ypiRES.Update(((Wide_Camera_Point_Feature_Measurement_Model)wide_model).threed_motion_model.get_rRES().GetVNL3(), 0);
            ypiRES.Update(hLhatWi.GetVNL3(), 3);

            //if (Camera_Constants.DEBUGDUMP) Debug.WriteLine("debug ypiRES" + ypiRES);

            // Form Jacobians dypi_by_dxp and dypi_by_dhi

            // dypi_by_dxp = (drWi_by_dr     drWi_by_dq    )
            //               (dhLhatWi_by_dr dhLhatWi_by_dq)
            //             = (I              0             )
            //               (0              dhLhatWi_by_dq)

            // hLhatWi = RWR * hLhatRi
            // => dhLhatWi_by_dq = d/dq ( R(qWR) * hLhatRi)

            MatrixFixed dhLhatWi_by_dq = MatrixFixed.dRq_times_a_by_dq(((Wide_Camera_Point_Feature_Measurement_Model)wide_model).threed_motion_model.get_qRES(), hLhatRi);

            // Put dypi_by_dxp together
            dypi_by_dxpRES.Fill(0.0f);
            dypi_by_dxpRES.Put(0, 0, 1.0f);
            dypi_by_dxpRES.Put(1, 1, 1.0f);
            dypi_by_dxpRES.Put(2, 2, 1.0f);
            dypi_by_dxpRES.Update(dhLhatWi_by_dq, 3, 3);

            //  cout << "dypi_by_dxpRES" <<  dypi_by_dxpRES;


            // dypi_by_dhi = (drWi_by_dhi    )
            //               (dhLhatWi_by_dhi)
            //             = (0              )
            //               (dhLhatWi_by_dhi)

            // hLhatWi = RWR * hLhatRi
            // Need to work out derivative for this
            // dhLhatWi_by_dhi = RWR * dhLhatRi_by_dhLRi * dhLRi_by_dhi

            MatrixFixed dhLhatWi_by_dhi = RWR * dhLhatRi_by_dhLRi * ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).m_camera.UnprojectionJacobian();
  
            dypi_by_dhiRES.Fill(0.0f);
            dypi_by_dhiRES.Update(dhLhatWi_by_dhi, 3, 0);

            //  cout << "dypi_by_dhiRES" <<  dypi_by_dhiRES;

            // And construct Ri
            func_Ri(hi);

            /*
            if (Camera_Constants.DEBUGDUMP) Debug.WriteLine("func_ypi: hi = " + hi + "," +
                                           "xp = " + xp + "," +
                                           "ypiRES = " + ypiRES);
             */
        }


        /// <summary>
        /// Predict the image location  \vct{h}_i = (x,y)  for
        /// partially-initialised feature with state  \vct{y}_i , given the current
        /// camera location  \vct{x}_p  and the depth parameter  \lambda .
        /// </summary>
        /// <param name="yi"></param>
        /// <param name="xp"></param>
        /// <param name="lambda"></param>
        public override void func_hpi_and_dhpi_by_dxp_and_dhpi_by_dyi(Vector yi, 
                                                             Vector xp,
                                                             Vector lambda)
        {
            
            // This function gives relative position of feature: also call this hR
            // (vector from camera to feature in robot frame)
            func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, xp);
            
            // Parameters of vector hLR from camera to feature in robot frame
            // hLR = zeroedri + lambda * zeroedhhati
            // Calculate the vector from the camera to the feature given the current
            // lambda
            Vector hLR = zeroedyiRES.Extract(3, 0) + zeroedyiRES.Extract(3, 3) * lambda[0];
            
            // Project this into the image
            hpiRES = ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).m_camera.Project(hLR);
            
            // What is the Jacobian of this projection?
            MatrixFixed dhpi_by_dhLRi = ((Wide_Camera_Point_Feature_Measurement_Model)wide_model).m_camera.ProjectionJacobian();
            
            // Calculate the required result Jacobians

            // Now how the vector to the feature depends on the parameterised line
            // (this is a function of lambda)
            float[] b = new float[18];
            b[0] = 1.0f;
            b[1] = 0.0f;
            b[2] = 0.0f;
            b[3] = lambda[0];
            b[4] = 0.0f;
            b[5] = 0.0f;
            b[6] = 0.0f;
            b[7] = 1.0f;
            b[8] = 0.0f;
            b[9] = 0.0f;
            b[10] = lambda[0];
            b[11] = 0.0f;
            b[12] = 0.0f;
            b[13] = 0.0f;
            b[14] = 1.0f;
            b[15] = 0.0f;
            b[16] = 0.0f;
            b[17] = lambda[0];
            MatrixFixed dhLRi_by_dzeroedyi = new MatrixFixed(3, 6, b);
  
            dhpi_by_dxpRES = dhpi_by_dhLRi * dhLRi_by_dzeroedyi * dzeroedyi_by_dxpRES;            
            dhpi_by_dyiRES = dhpi_by_dhLRi * dhLRi_by_dzeroedyi * dzeroedyi_by_dyiRES;
  
            /*
            if (Camera_Constants.DEBUGDUMP) Debug.WriteLine("func_hpi: yi = " + yi + "," +
                                           "xp = " + xp + "," +
                                           "lambda = " + lambda + "," +
                                           "hpiRES = " + hpiRES);
             */
            
        }

        /// <summary>
        /// Conversion function: how to turn partially initialised feature ypi
        /// into fully initialised one yfi
        /// </summary>
        /// <param name="ypi"></param>
        /// <param name="lambda"></param>
        public override void func_yfi_and_dyfi_by_dypi_and_dyfi_by_dlambda(Vector ypi, Vector lambda)
        {
            // Simple: lambda is a scalar
            // yfi = ri + lambda hhati   //commented out in original code

            func_ri(ypi);
            func_hhati(ypi);

            float[] a = new float[3];
            a[0] = riRES.GetX() + lambda[0] * hhatiRES.GetX();
            a[1] = riRES.GetY() + lambda[0] * hhatiRES.GetY();
            a[2] = riRES.GetZ() + lambda[0] * hhatiRES.GetZ();
            yfiRES.CopyIn(a); 

            float[] b = new float[18];
            b[0] = 1.0f;
            b[1] = 0.0f;
            b[2] = 0.0f;
            b[3] = lambda[0];
            b[4] = 0.0f;
            b[5] = 0.0f;
            b[6] = 0.0f;
            b[7] = 1.0f;
            b[8] = 0.0f;
            b[9] = 0.0f;
            b[10] = lambda[0];
            b[11] = 0.0f;
            b[12] = 0.0f;
            b[13] = 0.0f;
            b[14] = 1.0f;
            b[15] = 0.0f;
            b[16] = 0.0f;
            b[17] = lambda[0];
            dyfi_by_dypiRES.CopyIn(b);

            dyfi_by_dlambdaRES.Put(0, 0, hhatiRES.GetX());
            dyfi_by_dlambdaRES.Put(1, 0, hhatiRES.GetY());
            dyfi_by_dlambdaRES.Put(2, 0, hhatiRES.GetZ());
        }


        public void func_ri(Vector ypi)
        {
            riRES.SetVNL3(ypi.Extract(3, 0));
        }

        public void func_hhati(Vector ypi)
        {
            hhatiRES.SetVNL3(ypi.Extract(3, 3));
        }

    }

}
