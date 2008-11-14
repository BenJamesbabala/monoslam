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
using System.Diagnostics;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using SceneLibrary;
using sentience;

namespace monoSLAM
{

    /// <summary>
    /// Master class which defines and runs the MonoSLAM real-time single
    /// camera SLAM application. This class is GUI-independent, and only provides the
    /// core interface, for clarity and simplicity. The class MonoSLAMInterface
    /// derives from this and should usually be used instead, as it provides the rest
    /// of the interface.
    /// </summary>
    public class MonoSLAM
    {
        //constants used to show different views on the unser interface
        public const int DISPLAY_FEATURES = 0;
        public const int DISPLAY_MAP = 1;
        public const int DISPLAY_ELLIPSES = 2;
        public const int DISPLAY_PROBABILITIES = 3;
        public const int DISPLAY_AUGMENTED_REALITY = 4;

        // a feature used simply to calculate ranges for production of depth maps
        private FeatureInitInfo range_feature = null;
        private int range_feature_ctr = 0;
        public ArrayList ranged_features = new ArrayList(); //list of ranged features

        // lines between features
        private const int MAX_LINES = 4;
        private const int LINE_POINTS = 60;
        private const int LINE_HISTORY = 20;
        private linefeature persistent_line = null;
        private ArrayList lines = new ArrayList();

        public bool enable_mesh = false;
        private model_mesh mesh = new model_mesh();

        // for use by stopwatch functions
        private DateTime stopWatchTime;
        public long[] benchmark = new long[10];

        const int TRACE_LENGTH = 100;
        public float[,] camera_trace = new float[TRACE_LENGTH,3];
        int trace_index = 0;

        private Random rnd = new Random();

        public float speed = 0;  //current speed

        // This will be the member of feature_measurement_model_array which we will
        // use for initialisation
        protected Partially_Initialised_Feature_Measurement_Model default_feature_type_for_initialisation;
        protected Sim_Or_Rob sim_or_rob;
        protected Robot robot;
        protected Scene_Single scene;
        protected Kalman kalman;

        /******************************Parameters***********************************/

        //the path where initialisation file and known features are stored
        protected String PATH;

        // Image dimensions
        protected uint CAMERA_WIDTH;
        protected uint CAMERA_HEIGHT;

        // Parameters for map management
        protected uint NUMBER_OF_FEATURES_TO_SELECT;
        protected uint NUMBER_OF_FEATURES_TO_KEEP_VISIBLE;
        protected uint MAX_FEATURES_TO_INIT_AT_ONCE;

        // Parameters for feature initialisation
        // Maximum and minimum feature depth
        protected float MIN_LAMBDA;
        protected float MAX_LAMBDA;
        protected uint NUMBER_OF_PARTICLES;
        // Initialise as point when standard deviation / depth is less than this
        protected float STANDARD_DEVIATION_DEPTH_RATIO;
        // Give up on particles if number drops below this
        protected uint MIN_NUMBER_OF_PARTICLES;
        // Prune particle if probability/(number of particles) falls below this 
        protected float PRUNE_PROBABILITY_THRESHOLD;
        protected uint ERASE_PARTIALLY_INIT_FEATURE_AFTER_THIS_MANY_ATTEMPTS;

        // Corners of box we search for new features: save so we can display it
        protected int init_feature_search_ustart;
        protected int init_feature_search_vstart;
        protected int init_feature_search_ufinish;
        protected int init_feature_search_vfinish;
        protected bool init_feature_search_region_defined_flag;

        // Remember this so that the user can query it
        protected uint number_of_visible_features;
        public uint number_of_matched_features;
        protected Vector velocity;

        private float prev_speed = 0;
        private Vector prev_xv = null;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="initialisation_file">The initialisation file to read. This specifies the motion- and feature-measurement models to use, the initial state and known features.</param>
        /// <param name="mm_creator">The factory to use to create motion models.</param>
        /// <param name="fmm_creator">The factory to use to create feature measurement models</param>
        /// <param name="imm_creator">The factory to use to create internal measurement models</param>
        /// <param name="number_of_features_to_select">The number of features to select for measurement at each time step</param>
        /// <param name="number_of_features_to_keep_visible">The requried number of visible features. If fewer than this number are visible at any time step, the creation of a new feature is initiated</param>
        /// <param name="max_features_to_init_at_once"></param>
        /// <param name="min_lambda">The minimum distance from the camera (in metres) for a new feature</param>
        /// <param name="max_lambda">The maximum distance from the camera (in metres) for a new feature</param>
        /// <param name="number_of_particles">The number of particles to use for new features (distributed evenly in space between min_lambda and max_lambda)</param>
        /// <param name="standard_deviation_depth_ratio">The ratio between standard deviation and mean to use to identify when a partially-initialised feature should be converted to a fully-initialised one</param>
        /// <param name="min_number_of_particles">The minimum number of particles below which a partially-initalised feature is deleted</param>
        /// <param name="prune_probability_threshold">The threshold below which a particle with low probability is deleted</param>
        /// <param name="erase_partially_init_feature_after_this_many_attempts">The number of failed match attempts before a partially initialised feature is deleted.</param>
        public MonoSLAM(String initialisation_file,
                        String path,
                        Motion_Model_Creator mm_creator,
                        Feature_Measurement_Model_Creator fmm_creator,
                        Internal_Measurement_Model_Creator imm_creator,
                        uint number_of_features_to_select,
                        uint number_of_features_to_keep_visible,
                        uint max_features_to_init_at_once,
                        float min_lambda,
                        float max_lambda,
                        uint number_of_particles,
                        float standard_deviation_depth_ratio,
                        uint min_number_of_particles,
                        float prune_probability_threshold,
                        uint erase_partially_init_feature_after_this_many_attempts,
                        float MAXIMUM_ANGLE_DIFFERENCE,
                        float calibration_target_width_mm,
                        float calibration_target_height_mm,
                        float calibration_target_distance_mm)
        {
            PATH = path;
            NUMBER_OF_FEATURES_TO_SELECT = number_of_features_to_select;
            NUMBER_OF_FEATURES_TO_KEEP_VISIBLE = number_of_features_to_keep_visible;
            MAX_FEATURES_TO_INIT_AT_ONCE = max_features_to_init_at_once;
            MIN_LAMBDA = min_lambda;
            MAX_LAMBDA = max_lambda;
            NUMBER_OF_PARTICLES = number_of_particles;
            STANDARD_DEVIATION_DEPTH_RATIO = standard_deviation_depth_ratio;
            MIN_NUMBER_OF_PARTICLES = min_number_of_particles;
            PRUNE_PROBABILITY_THRESHOLD = prune_probability_threshold;
            ERASE_PARTIALLY_INIT_FEATURE_AFTER_THIS_MANY_ATTEMPTS = erase_partially_init_feature_after_this_many_attempts;
            number_of_visible_features = 0;
            number_of_matched_features = 0;
            
            Settings settings = new Settings();

            //if no file exists create some default values
            //if (!File.Exists(PATH + initialisation_file))
            {
                //create a settings file
                settings.createDefault(PATH + initialisation_file, calibration_target_width_mm, 
                                       calibration_target_height_mm, calibration_target_distance_mm);
                //settings.createDefault(PATH + initialisation_file, 210, 148.5, 600);
            }

            //create some known features
            createDefaultKnownFeatures(PATH);
            

            // Create the Settings class by reading from the initialisation file
            if (File.Exists(PATH + initialisation_file))
            {
                StreamReader stream = File.OpenText(PATH + initialisation_file);
                settings.load(stream);

                // Create the Scene class. This also constructs the motion model and 
                // internal measurement models and sets the initial state
                scene = new Scene_Single(settings, mm_creator, imm_creator);

                // Now sort out the feature types
                ArrayList values = settings.get_entry("Models", "NewFeatureMeasurementModel");
                String feature_init_type = (String)values[0];
                Feature_Measurement_Model fm_model =
                    fmm_creator.create_model(feature_init_type, scene.get_motion_model(), MAXIMUM_ANGLE_DIFFERENCE);


                if (fm_model == null)
                {
                    Debug.WriteLine("Unable to create a feature measurement motion model of type " +
                                    feature_init_type + " as requested in initalisation file " +
                                    initialisation_file);
                }
                else
                {
                    // Initialise this motion model
                    fm_model.read_parameters(settings);

                    // Check that this is a partially-initialised feature type
                    if (fm_model.fully_initialised_flag)
                    {
                        Debug.WriteLine("Feature measurement motion model " + feature_init_type +
                                        " as requested in initalisation file " + initialisation_file +
                                        " is not a partially-initialised feature type. ");
                    }

                    default_feature_type_for_initialisation =
                        (Partially_Initialised_Feature_Measurement_Model)fm_model;

                    // We hope that features are viewed through a camera! If so,
                    // the feature measurement class should derive from
                    // Camera_Feature_Measurement_Model
                    // Note the multiple inherritance workaround
                    Camera_Feature_Measurement_Model cfmm =
                        (Camera_Feature_Measurement_Model)(fm_model.wide_model);

                    if (cfmm == null)
                    {
                        // Oops - the feature measurement model is not derived from
                        // Camera_Feature_Measurement_Model!                    
                        Debug.WriteLine("The default feature measurement motion model " +
                                        fm_model.feature_type +
                                        " is not derived from Camera_Feature_Measurement_Model!");
                    }
                    else
                    {

                        CAMERA_WIDTH = cfmm.get_camera().ImageWidth();
                        CAMERA_HEIGHT = cfmm.get_camera().ImageHeight();

                        kalman = new Kalman();
                        robot = new Robot();
                        sim_or_rob = (Sim_Or_Rob)robot;

                        // Initialise any known features
                        SceneLib.initialise_known_features(settings, fmm_creator, sim_or_rob, scene, PATH, MAXIMUM_ANGLE_DIFFERENCE);

                        // Various flags
                        init_feature_search_region_defined_flag = false;
                    }
                }

                stream.Close();
            }
            else
            {
                Debug.WriteLine("File not found:  " + initialisation_file);
            }
        }

        #region "stopwatch functions for monitoring algorithm performance"

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

        #endregion


        /// <summary>
        /// Step the MonoSLAM application on by one frame. This should be called every time
        /// a new frame is captured (and care should be taken to avoid skipping frames).
        /// Before calling this function, Scene_Single::load_new_image() should be called
        /// (e.g. using
        /// <code>monoslaminterface.GetRobotNoConst()->load_new_image()</code>), since the
        /// first parameter to GoOneStep() is currently ignored.
        /// 
        /// GoOneStep() performs the following processing steps:
        /// - Kalman filter prediction step (by calling Kalman::predict_filter_fast(),
        ///   with a zero control vector)
        /// - Select a set of features to make measurements from (set by
        ///   SetNumberOfFeaturesToSelect())
        /// - Predict the locations and and make measurements of those features
        /// - Kalman filter update step
        /// - Delete any bad features (those that have repeatedly failed to be matched)
        /// - If we are not currently initialising a enough new features, and the camera is
        ///   translating, and <code>currently_mapping_flag</code> is set, initialise a new
        ///   feature somewhere sensible
        /// - Update the partially-initialised features
        /// </summary>
        /// <param name="img">camera image</param>
        /// <param name="delta_t">The time between frames in seconds</param>
        /// <param name="currently_mapping_flag">Set to be true if new features should be detected and added to the map.</param>
        /// <returns></returns>
        public bool GoOneStep(float delta_t, bool currently_mapping_flag)
        {
            if (delta_t > 0)
            {
                // update the integral image for use in feature detection
                //img.updateIntegralImage();

                // nullify image selection
                robot.nullify_image_selection();

                init_feature_search_region_defined_flag = false;

                // Control vector of accelerations
                Vector u = new Vector(3);
                u.Fill(0.0f);

                sim_or_rob.set_control(u, delta_t);

                // Record the current position so that I can estimate velocity
                // (We can guarantee that the state vector has position; we can't 
                // guarantee that it has velocity.)
                Vector xv = scene.get_xv();
                scene.get_motion_model().func_xp(xv);
                Vector prev_xp_pos = (scene.get_motion_model().get_xpRES()).Extract(3);

                // Prediction step
                if (currently_mapping_flag)
                    kalman.predict_filter_fast(scene, u, delta_t);

                // if features are not seen the first time try a few more times
                int tries = 0;
                number_of_visible_features = 0;
                while (((tries == 0) || (scene.get_no_selected() < 2)) && (tries < 5))
                {
                    number_of_visible_features = scene.auto_select_n_features(NUMBER_OF_FEATURES_TO_SELECT);

                    if (scene.get_no_selected() > 0)
                    {
                        //scene.predict_measurements(); // commented out in original code                        

                        number_of_matched_features = (uint)SceneLib.make_measurements(scene, sim_or_rob, rnd);

                        if (scene.get_successful_measurement_vector_size() != 0)
                        {
                            // this function is the slowest part of the algorithm
                            kalman.total_update_filter_slow(scene);

                            scene.normalise_state();
                        }

                    }

                    tries++;
                }


                if (currently_mapping_flag)
                    scene.delete_bad_features();

                // Let's enforce symmetry of covariance matrix...
                // Add to transpose and divide by 2                
                uint tot_state_size = scene.get_total_state_size();
                MatrixFixed Pxx = new MatrixFixed(tot_state_size, tot_state_size);
                scene.construct_total_covariance(ref Pxx);
                MatrixFixed PxxT = Pxx.Transpose();                

                Pxx.Update(Pxx * 0.5f + PxxT * 0.5f);
                scene.fill_covariances(Pxx);
                
                // Look at camera speed estimate
                // Get the current position and estimate the speed from it
                xv = scene.get_xv();
                scene.get_motion_model().func_xp(xv);
                Vector xp_pos = scene.get_motion_model().get_xpRES().Extract(3);
                velocity = (xp_pos - prev_xp_pos) / delta_t;
                speed = (float)Math.Sqrt(velocity.SquaredMagnitude());

                // This section of code is a workaround for a situation where
                // the camera suddenly shoots off at high speed, which must be
                // a bug perhaps in the state update.  If the camera suddenly accelerates
                // at a high rate then this just sets the state back to the previous one.
                if (prev_speed == 0) 
                    prev_speed = speed;
                else
                {
                    float speed_change = speed / prev_speed;
                    if ((speed > 1) && (speed_change > 1.2) && (prev_xv != null))
                    {
                        xv.Update(prev_xv);
                    }
                }
                prev_speed = speed;
                if (prev_xv != null)  // TODO: minor bug here with vector != operator
                    prev_xv.Update(xv);
                else
                    prev_xv = new Vector(xv);

                if (currently_mapping_flag)
                {
                    if (speed > 0.2)
                    {
                        if ((number_of_visible_features < NUMBER_OF_FEATURES_TO_KEEP_VISIBLE) &&
                            (scene.get_feature_init_info_vector().Count < (uint)(MAX_FEATURES_TO_INIT_AT_ONCE)))
                        {
                            // if the number of features is low make more attempts
                            // to initialise new ones
                            tries = 1;
                            if (number_of_visible_features < 8) tries = 2;
                            if (number_of_visible_features < 5) tries = 3;
                            for (int i = 0; i < tries; i++) AutoInitialiseFeature(u, delta_t);
                        }

                        //if (range_feature == null) AutoInitialiseRangeFeature();
                        //updateLines();

                        if (enable_mesh)
                        {
                            mesh.image_width = robot.image.width;
                            mesh.image_height = robot.image.height;
                            mesh.update(scene);
                        }
                    }
                }

                MatchPartiallyInitialisedFeatures();                

                recordCameraHistory();
                 
            }
            return true;
        }


        /// <summary>
        /// Initialise a feature at the currently-selected image location. The image
        /// selection is usually either chosen automatically with
        /// Robot::set_image_selection_automatically() or manually by the user by calling
        /// Robot::set_image_selection().
        /// </summary>
        public FeatureInitInfo InitialiseFeature()
        {
            FeatureInitInfo feat = null;
            Vector z = new Vector(default_feature_type_for_initialisation.MEASUREMENT_SIZE);

            // Fill in z and id for the currently-selected feature
            classimage_mono patch = null;
            Vector patch_colour = new Vector(3);
            if (robot.make_initial_measurement_of_feature(z, ref patch, default_feature_type_for_initialisation, patch_colour) == false)
            {
                Debug.WriteLine("Failed in initialisation of feature.");
            }
            else
            {                   
                feat = scene.add_new_partially_initialised_feature(
                                           patch, z, default_feature_type_for_initialisation, patch_colour);
               
                // And set up a particle distribution for this partially initialised feature
                float lambda_step = (1.0f / (float)NUMBER_OF_PARTICLES) * (MAX_LAMBDA - MIN_LAMBDA);
                float uniform_probability = 1.0f / (float)NUMBER_OF_PARTICLES;
                // Initialise particle set with uniform prior
                Vector lambda = new Vector(1);
                lambda[0] = MIN_LAMBDA;
                for (uint i = 0; i < NUMBER_OF_PARTICLES; i++) 
                {
                    scene.add_particle_to_newest_partially_init_feature(lambda, uniform_probability);

                    lambda[0] += lambda_step;
                }
            }
            return (feat);
        }


        /// <summary>
        /// Initialise a feature at a position determined automatically. This predicts
        /// where the image centre will be soon (in 10 frames), and tries initialising a
        /// feature near there. This may not necessarily give a new feature - the score
        /// could not be good enough, or no suitable non-overlapping region might be found.
        /// </summary>
        /// <param name="u">The input control vector (zero in the MonoSLAM application)</param>
        /// <param name="delta_t">The time between frames</param>
        /// <returns>true on success, or false on failure (i.e. if no non-overlapping region is found, or if no feature could be found with a good enough score).</returns>
        public bool AutoInitialiseFeature(Vector u, float delta_t)
        {
            // A cute method for deciding where to centre our search for a new feature
            // Idea: look for a point in a position that we expect to be near the
            // image centre soon

            // Predict the camera position a few steps into the future
            uint FEATURE_INIT_STEPS_TO_PREDICT = 10;

            // Project a point a "reasonable" distance forward from there along
            // the optic axis
            float FEATURE_INIT_DEPTH_HYPOTHESIS = 2.5f;

            // First find a suitable patch
            //float SUITABLE_PATCH_SCORE_THRESHOLD = 20000;
            float SUITABLE_PATCH_SCORE_THRESHOLD = 15000;
            
            Vector local_u = new Vector(scene.get_motion_model().CONTROL_SIZE);
            
            if (non_overlapping_region.FindNonOverlappingRegion(scene, u, delta_t,
                               default_feature_type_for_initialisation,
                               CAMERA_WIDTH,
                               CAMERA_HEIGHT,
                               Camera_Constants.BOXSIZE,
                               ref init_feature_search_ustart,
                               ref init_feature_search_vstart,
                               ref init_feature_search_ufinish,
                               ref init_feature_search_vfinish,
                               FEATURE_INIT_STEPS_TO_PREDICT,
                               FEATURE_INIT_DEPTH_HYPOTHESIS, rnd))
            {
                init_feature_search_region_defined_flag = true;
                if (robot.set_image_selection_automatically(
                            (uint)init_feature_search_ustart, (uint)init_feature_search_vstart,
                            (uint)init_feature_search_ufinish, (uint)init_feature_search_vfinish) >
                            SUITABLE_PATCH_SCORE_THRESHOLD)
                {
                    int min_dist = minimum_distance_to_feature((int)robot.get_uu(), (int)robot.get_vv());
                    if ((min_dist > robot.image.width / 10) || (min_dist == 9999))
                    {
                        int tx = init_feature_search_ustart * robot.outputimage.width / robot.image.width;
                        int ty = init_feature_search_vstart * robot.outputimage.height / robot.image.height;
                        int bx = init_feature_search_ufinish * robot.outputimage.width / robot.image.width;
                        int by = init_feature_search_vfinish * robot.outputimage.height / robot.image.height;

                        // show the search region within which the feature was found
                        robot.outputimage.DrawBox(tx, ty, bx, by);

                        // Then initialise it
                        InitialiseFeature();
                    }
                    else return false;
                }
                else
                {
                    // Score not good enough; feature not initialised.
                    return false;
                }                 
            }
            else
            {
                // No non-overlapping region found; feature not initialised.
                return false;
            }
             
             
            return true;
        }

        /// <summary>
        /// Initialise a range feature, used to build depth maps, at a position determined 
        /// automatically. This predicts where the image centre will be soon (in 10 frames), 
        /// and tries initialising a feature near there. This may not necessarily give a 
        /// new feature - the score could not be good enough, or no suitable non-overlapping 
        /// region might be found.
        /// </summary>
        /// <returns>true on success, or false on failure (i.e. if no non-overlapping region is found, or if no feature could be found with a good enough score).</returns>
        public bool AutoInitialiseRangeFeature()
        {
            // A cute method for deciding where to centre our search for a new feature
            // Idea: look for a point in a position that we expect to be near the
            // image centre soon

            // First find a suitable patch
            float SUITABLE_PATCH_SCORE_THRESHOLD = 15000;

            Vector local_u = new Vector(scene.get_motion_model().CONTROL_SIZE);

            // randomly pick an area within which to search
            int w = robot.image.width / 6;
            int h = robot.image.height / 6;
            int wdth = robot.image.width - (int)(Camera_Constants.BOXSIZE * 2) - w;
            int hght = robot.image.height - (int)(Camera_Constants.BOXSIZE * 2) - h;
            init_feature_search_ustart = (int)Camera_Constants.BOXSIZE + rnd.Next(wdth);
            init_feature_search_ufinish = init_feature_search_ustart + w;
            init_feature_search_vstart = (int)Camera_Constants.BOXSIZE + rnd.Next(hght);
            init_feature_search_vfinish = init_feature_search_vfinish + h;

            init_feature_search_region_defined_flag = true;
            if (robot.set_image_selection_automatically(
                        (uint)init_feature_search_ustart, (uint)init_feature_search_vstart,
                        (uint)init_feature_search_ufinish, (uint)init_feature_search_vfinish) >
                        SUITABLE_PATCH_SCORE_THRESHOLD)
            {
                // Then initialise it
                range_feature = InitialiseFeature();
            }
            else
            {
                // Score not good enough; feature not initialised.
                return false;
            }

            return true;
        }


        public void showProbabilities(int feature_index, classimage_mono img)
        {
            img.clear();
            if (feature_index < scene.get_feature_init_info_vector().Count)
            {
                int i = 0;
                histogram hist = new histogram();
                hist.init(100);

                FeatureInitInfo feat = (FeatureInitInfo)scene.get_feature_init_info_vector()[feature_index];
                foreach (Particle p in feat.particle_vector)
                {
                    hist.add(i, (int)(p.probability*10000));
                    i++;
                }
                hist.Show(img);
            }
        }

        //sets camera calibration parameters
        public void set_camera_calibration(float Kd1, float Fku, float Fkv,
                                           float U0, float V0, float measurement_sd)
        {
            for (int i = 0; i < scene.feature_list.Count; i++)
            {
                Feature f = (Feature)scene.feature_list[i];


                Line_Init_Wide_Point_Feature_Measurement_Model pmm =
                    (Line_Init_Wide_Point_Feature_Measurement_Model)f.get_partially_initialised_feature_measurement_model();
                if (pmm != null)
                {
                    pmm.set_calibration(Kd1, Fku, Fkv, U0, V0, measurement_sd);
                }

                Fully_Init_Wide_Point_Feature_Measurement_Model fmm =
                    (Fully_Init_Wide_Point_Feature_Measurement_Model)f.get_fully_initialised_feature_measurement_model();
                if (fmm != null)
                {
                    fmm.set_calibration(Kd1, Fku, Fkv, U0, V0, measurement_sd);
                }
            }
        }

        /// <summary>
        /// return the minimum distance to a tracked feature
        /// </summary>
        /// <returns></returns>
        private int minimum_distance_to_feature(int u, int v)
        {
            int i;
            int min_dist = 9999;

            for (i = 0; i < scene.selected_feature_list.Count; i++)
            {
                Feature f = (Feature)scene.selected_feature_list[i];

                if ((f.get_successful_measurement_flag()) &&
                    (f.get_feature_measurement_model().fully_initialised_flag))
                {
                    Vector measured_image_position = f.get_z();
                    int dist_u = (int)measured_image_position[0] - u;
                    if (dist_u < 0) dist_u = -dist_u;
                    int dist_v = (int)measured_image_position[1] - v;
                    if (dist_v < 0) dist_v = -dist_v;
                    int dist = dist_u + dist_v;
                    if (dist < min_dist) min_dist = dist;
                }
            }
            return (min_dist);
        }


        public void ShowMesh(classimage_mono img)
        {
            mesh.Show(img);
        }

        /// <summary>
        /// show tracked features within the given image
        /// </summary>
        /// <param name="img"></param>
        public void ShowFeatures(classimage_mono img)
        {
            int x, y, i, j, c;
            Vector position;
            int size_factor = 50;
            int point_radius_x = img.width / size_factor;
            if (point_radius_x < 1) point_radius_x = 1;

            int point_radius_y = img.height / size_factor;
            if (point_radius_y < 1) point_radius_y = 1;

            for (i = 0; i < scene.selected_feature_list.Count; i++)
            {
                Feature f = (Feature)scene.selected_feature_list[i];

                if ((f.get_successful_measurement_flag()) &&
                    (f.get_feature_measurement_model().fully_initialised_flag))
                {

                    Vector measured_image_position = f.get_z(); //measured position within the image (bottom up)
                    Vector map_image_position = f.get_h(); //position from within the 3D map (top down)

                    for (j = 0; j < 1; j++)
                    {
                        if (j == 0)
                            position = measured_image_position;
                        else
                            position = map_image_position;

                        //scale into the given image dimensions
                        position[0] = (position[0] * img.width / CAMERA_WIDTH);
                        position[1] = (position[1] * img.height / CAMERA_HEIGHT);

                        if ((position[0] > point_radius_x) && (position[0] < img.width - point_radius_x))
                            if ((position[1] > point_radius_y) && (position[1] < img.height - point_radius_y))
                            {
                                
                                for (x = (int)position[0] - point_radius_x; x < (int)position[0] + point_radius_x; x++)
                                    for (y = (int)position[1] - point_radius_y; y < (int)position[1] + point_radius_y; y++)
                                    {
                                        if ((x == (int)position[0]) || (y == (int)position[1]))
                                        {
                                            for (c = 0; c < 3; c++) img.image[x, y] = 0;
                                            //if (j == 0)
                                                img.image[x, y] = 255;
                                            //else
                                              //  img.image[x, y] = 255;
                                        }
                                    }
                                

                                /*
                                for (x = 0; x < Camera_Constants.BOXSIZE; x++)
                                {
                                    xx = x + (int)position[0] - (int)(Camera_Constants.BOXSIZE / 2);
                                    if ((xx > 0) && (xx < img.width))
                                    {
                                        for (y = 0; y < Camera_Constants.BOXSIZE; y++)
                                        {
                                            yy = y + (int)position[1] - (int)(Camera_Constants.BOXSIZE / 2);
                                            if ((yy > 0) && (yy < img.height))
                                            {
                                                for (c = 0; c < 3; c++) img.image[xx, yy, c] = f.get_identifier().image[x, y, c];
                                            }
                                        }
                                    }
                                }
                                */

                            }
                    }
                }
            }
        }

        public void ShowTriangles(classimage_mono img)
        {
            int i, j;

            for (i = 0; i < scene.selected_feature_list.Count; i++)
            {
                Feature f1 = (Feature)scene.selected_feature_list[i];

                if ((f1.get_successful_measurement_flag()) &&
                    (f1.get_feature_measurement_model().fully_initialised_flag))
                {

                    Vector measured_image_position1 = f1.get_z(); //measured position within the image (bottom up)

                    //scale into the given image dimensions
                    measured_image_position1[0] = (measured_image_position1[0] * img.width / CAMERA_WIDTH);
                    measured_image_position1[1] = (measured_image_position1[1] * img.height / CAMERA_HEIGHT);

                    for (j = i+1; j < scene.selected_feature_list.Count; j++)
                    {
                            Feature f2 = (Feature)scene.selected_feature_list[j];

                            if ((f2.get_successful_measurement_flag()) &&
                                (f2.get_feature_measurement_model().fully_initialised_flag))
                            {

                                Vector measured_image_position2 = f2.get_z(); //measured position within the image (bottom up)
                    
                                //scale into the given image dimensions                    
                                measured_image_position2[0] = (measured_image_position2[0] * img.width / CAMERA_WIDTH);                    
                                measured_image_position2[1] = (measured_image_position2[1] * img.height / CAMERA_HEIGHT);

                                img.drawLine((int)measured_image_position1[0], (int)measured_image_position1[1],
                                             (int)measured_image_position2[0], (int)measured_image_position2[1], 0);
                            }
                    }
                }
            }

        }

        /// <summary>
        /// Delete the currently-marked feature. This calls Scene_Single::delete_feature().
        /// </summary>
        public void DeleteFeature()
        {
            scene.delete_feature();
        }

        /// <summary>
        /// Save the currently-selected patch as an image file. This calls
        /// Robot::write_patch().
        /// </summary>
        public void SavePatch()
        {
            robot.write_patch();
        }


        /// <summary>
        /// Print out the current robot state  x_v  and covariance  P_{xx} .
        /// This calls Scene_Single::print_robot_state().
        /// </summary>
        public void PrintRobotState()
        {
            //scene.print_robot_state();
        }


        /// <summary>
        /// Try to match the partially-initialised features, then update their
        /// distributions. If possible (if the standard deviation ratio  \Sigma_{11} /
        /// \mu_1  is small enough) this also converts them to fully-initalised 
        /// features. In addition, this also calls 
        /// Scene_Single::delete_partially_initialised_features_past_sell_by_date() to
        /// delete any that have not collapses fast enough.
        /// </summary>
        public void MatchPartiallyInitialisedFeatures()
        {
            
            // Go through all partially initialised features and decide which ones
            // to try to measure; predict measurements for these
            scene.predict_partially_initialised_feature_measurements();
            

            //if (Camera_Constants.DEBUGDUMP) cout << "Match: time after setting up search stuff " 
            //        << timer1 << endl;

            // Loop through partially initialised features
            foreach (FeatureInitInfo feat in scene.get_feature_init_info_vector_noconst())
            {
                if (feat.get_making_measurement_on_this_step_flag())
                {
                    // Making partially initialised measurements

                    // Make search for this feature over overlapping ellipses of particles
                    robot.measure_feature_with_multiple_priors(feat.get_fp().get_identifier(), feat.get_particle_vector_noconst());
                }
            }

            // Update particle distributions with measurements, normalise, prune, etc.
            scene.update_partially_initialised_feature_probabilities(PRUNE_PROBABILITY_THRESHOLD);

            // Now go through the features again and decide whether to convert
            // any to fully-initialised features
            // Do this here rather than generically in Scene because the test
            // for conversion is specific to this application
            for (int i = 0; i < scene.get_feature_init_info_vector_noconst().Count; i++)
            {
                FeatureInitInfo feat = (FeatureInitInfo)scene.get_feature_init_info_vector_noconst()[i];

                // Only try to convert if we've been making measurements!
                if (feat.get_making_measurement_on_this_step_flag())
                {
                    float mean_sd_ratio = (float)Math.Sqrt(feat.get_covariance()[0, 0]) / feat.get_mean()[0];

                    // Checking whether to convert feature 
                    int no_of_particles = feat.get_particle_vector().Count;
                    if ((feat.nonzerovariance) &&
                        (mean_sd_ratio < STANDARD_DEVIATION_DEPTH_RATIO) &&
                        (no_of_particles > MIN_NUMBER_OF_PARTICLES))
                    {
                        if (feat != range_feature)
                        {
                            // add the new feature to the mesh
                            if (enable_mesh) mesh.addFeatureInit(feat.get_fp());

                            // Yes, converting feature                        
                            feat.get_fp_noconst().convert_from_partially_to_fully_initialised(
                                feat.get_mean(), feat.get_covariance(), scene);
                            scene.get_feature_init_info_vector_noconst().Remove(feat);
                            i--;
                        }
                        else
                        {
                            // take the feature's position only
                            feat.get_fp_noconst().update_feature_position(feat.get_mean());
                            feat.number_of_match_attempts = ERASE_PARTIALLY_INIT_FEATURE_AFTER_THIS_MANY_ATTEMPTS + 1;
                            ranged_features.Add(feat.get_fp_noconst());  // TODO: does this get too large?
                            range_feature_ctr = 0;
                            range_feature = null;
                        }
                    }                    
                }
            }

            if (range_feature != null)
            {
                range_feature_ctr++;
                if (range_feature_ctr > ERASE_PARTIALLY_INIT_FEATURE_AFTER_THIS_MANY_ATTEMPTS)
                {
                    range_feature_ctr = 0;
                    range_feature.number_of_match_attempts = ERASE_PARTIALLY_INIT_FEATURE_AFTER_THIS_MANY_ATTEMPTS + 1;
                    range_feature = null;
                }
            }

            // Go through and get rid of any partially initialised features whose
            // distributions haven't collapsed fast enough             
            scene.delete_partially_initialised_features_past_sell_by_date(
                ERASE_PARTIALLY_INIT_FEATURE_AFTER_THIS_MANY_ATTEMPTS,
                MIN_NUMBER_OF_PARTICLES);
                                                             
        }


        /// <summary>
        /// Create some default features for use with a target image
        /// </summary>
        private void createDefaultKnownFeatures(String path)
        {
            Byte value=0;
            Byte high_value = 180;
            Byte low_value = 60;
            classimage_mono known_feature = new classimage_mono();
            known_feature.createImage((int)Camera_Constants.BOXSIZE, (int)Camera_Constants.BOXSIZE);

            for (int i = 0; i < 4; i++)
            {
                for (int x = 0; x < Camera_Constants.BOXSIZE; x++)
                {
                    for (int y = 0; y < Camera_Constants.BOXSIZE; y++)
                    {
                        switch (i)
                        {
                            case 0:
                                {
                                    if ((x > Camera_Constants.BOXSIZE / 2) &&
                                        (y > Camera_Constants.BOXSIZE / 2))
                                        value = low_value;
                                    else
                                        value = high_value;
                                    break;
                                }
                            case 1:
                                {
                                    if ((x < Camera_Constants.BOXSIZE / 2) &&
                                        (y > Camera_Constants.BOXSIZE / 2))
                                        value = low_value;
                                    else
                                        value = high_value;
                                    break;
                                }
                            case 2:
                                {
                                    if ((x > Camera_Constants.BOXSIZE / 2) &&
                                        (y < Camera_Constants.BOXSIZE / 2))
                                        value = low_value;
                                    else
                                        value = high_value;
                                    break;
                                }
                            case 3:
                                {
                                    if ((x < Camera_Constants.BOXSIZE / 2) &&
                                        (y < Camera_Constants.BOXSIZE / 2))
                                        value = low_value;
                                    else
                                        value = high_value;
                                    break;
                                }
                        }

                        known_feature.image[x, y] = value;
                    }
                }
                known_feature.SaveAsBitmapMono(path + "known_patch" + Convert.ToString(i) + ".bmp");
            }
        }

        /// <summary>
        /// record the recent history of camera movement
        /// </summary>
        private void recordCameraHistory()
        {
            Vector xv = scene.get_xv();

            for (int i = 0; i < 3; i++)
            {
                camera_trace[trace_index,i] = xv[i];
            }
            trace_index++;
            if (trace_index >= TRACE_LENGTH) trace_index = 0;
        }

        /// <summary>
        /// returns the position and orientation of the camera
        /// </summary>
        /// <param name="position"></param>
        /// <param name="orientation"></param>
        public void GetCameraPositionOrientation(ref Vector3D position, ref Quaternion orientation)
        {
            /*
            Quaternion qRO = new Quaternion(0.0f, 1.0f, 0.0f, 0.0f);
            ThreeD_Motion_Model threed_motion_model = (ThreeD_Motion_Model)scene.get_motion_model();

            threed_motion_model.func_xp(scene.get_xv());
            threed_motion_model.func_r(threed_motion_model.get_xpRES());
            threed_motion_model.func_q(threed_motion_model.get_xpRES());

            orientation = threed_motion_model.get_qRES(); //.Multiply(qRO);
            position = threed_motion_model.get_rRES();
            */

            Vector state = scene.get_xv();
            position = new Vector3D(state[0], state[1], state[2]);
            //orientation = new Quaternion(state[4], state[5], state[6], state[3]);
            orientation = new Quaternion(state[3], state[4], state[5], state[6]); //.Multiply(qRO);
            
        }

        /// <summary>
        /// update lines between features
        /// </summary>
        public void updateLines()
        {
            bool feature_exists;
            int i;
            Byte max = 0;
            linefeature lf;
            Feature f1, f2;

            // remove any lines which use features which are no longer observed
            persistent_line = null;
            i = 0;
            while (i < lines.Count)
            {
                feature_exists = false;
                lf = (linefeature)lines[i];

                if (lf.marked_for_deletion)
                {
                    lines.Remove(lf);
                }
                else
                {
                    if (scene.selected_feature_list.Contains(lf.feature1))
                    {
                        if (scene.selected_feature_list.Contains(lf.feature2))
                        {
                            feature_exists = true;
                        }
                    }
                    if (!feature_exists)
                    {
                        lines.Remove(lf);
                    }
                    else
                    {
                        // update the line
                        lf.update(robot.image);
                        if (lf.hits > max) persistent_line = lf;
                        i++;
                    }
                }
            }

            // add new lines
            if (lines.Count < MAX_LINES)
            {
                int index_feature1 = rnd.Next(scene.selected_feature_list.Count - 1);
                int index_feature2 = rnd.Next(scene.selected_feature_list.Count - 1);
                if (index_feature1 != index_feature2)
                {
                    f1 = (Feature)scene.selected_feature_list[index_feature1];
                    if ((f1.get_successful_measurement_flag()) &&
                        (f1.get_feature_measurement_model().fully_initialised_flag))
                    {
                        f2 = (Feature)scene.selected_feature_list[index_feature2];
                        if ((f2.get_successful_measurement_flag()) &&
                            (f2.get_feature_measurement_model().fully_initialised_flag))
                        {
                            lf = new linefeature(f1, f2, LINE_POINTS, LINE_HISTORY);
                            lines.Add(lf);
                        }
                    }
                }
            }
        }


        /// <summary>
        /// show the history of a line
        /// </summary>
        /// <param name="img"></param>
        public void ShowLineHistory(classimage_mono img)
        {
            if (persistent_line != null)
            {
                persistent_line.show(img);
            }
        }


        public void ShowOverheadView(classimage_mono img, float world_dimension_x, float world_dimension_z)
        {
            int xx, yy, x, z, feature_size_x, feature_size_z,i,t,prev_x=0,prev_z=0;
            Vector position;
            float half_x = world_dimension_x/2;
            float half_z = world_dimension_z/2;

            img.clear();

            //draw the camera trace
            
            for (i = 0; i < TRACE_LENGTH; i++)
            {
                t = trace_index - i;
                if (t < 0) t += TRACE_LENGTH;
                if (!((camera_trace[t, 0] == 0) && (camera_trace[t, 1] == 0) && (camera_trace[t, 2] == 0)))
                {
                    x = img.width-(int)((camera_trace[t,0] + half_x) / world_dimension_x * img.width);
                    z = img.height-(int)((camera_trace[t, 2] + half_z) / world_dimension_z * img.height);
                    if (i > 1)
                    {
                        img.drawLine(x, z, prev_x, prev_z, 0);
                    }
                    prev_x = x;
                    prev_z = z;
                }
            }
            
            //draw the camera
            ThreeD_Motion_Model threed_motion_model = (ThreeD_Motion_Model)scene.get_motion_model();
            threed_motion_model.func_xp(scene.get_xv());
            threed_motion_model.func_r(threed_motion_model.get_xpRES());
            threed_motion_model.func_q(threed_motion_model.get_xpRES());
            //Quaternion qWO = threed_motion_model.get_qRES() * qRO;
            Vector3D r_local = threed_motion_model.get_rRES();

            position = scene.get_xv();
            float scale_factor = 1.0f + ((position[1] + (world_dimension_x/2))/world_dimension_x);
            feature_size_x = (int)(img.width * scale_factor / 50);
            feature_size_z = (int)(img.height * scale_factor / 50);
            x = (int)((r_local.GetX() + half_x) / world_dimension_x * img.width);
            if ((x > 0) && (x < img.width))
            {
                z = (int)((r_local.GetZ() + half_z) / world_dimension_z * img.height);
                if ((z > 0) && (z < img.height))
                {
                    for (xx = x - feature_size_x; xx < x + feature_size_x; xx++)
                    {
                        if ((xx > 0) && (xx < img.width))
                        {
                            for (yy = z - feature_size_z; yy < z + feature_size_z; yy++)
                            {
                                if ((yy > 0) && (yy < img.height))
                                {                                    
                                    img.image[img.width - xx, img.height - yy] = 255;
                                }
                            }
                        }
                    }
                }
            }

            
            //draw ranged features
            /*
            scale_factor = 1;
            feature_size_x = (int)(img.width * scale_factor / 50);
            feature_size_z = (int)(img.height * scale_factor / 50);
            foreach (Feature v in ranged_features)
            {
                position = v.get_y();
                x = (int)((position[0] + half_x) / world_dimension_x * img.width);
                if ((x > 0) && (x < img.width))
                {
                    z = (int)((position[2] + half_z) / world_dimension_z * img.height);
                    if ((z > 0) && (z < img.height))
                    {
                        for (xx = x - feature_size_x; xx < x + feature_size_x; xx++)
                        {
                            if ((xx > 0) && (xx < img.width))
                            {
                                for (yy = z - feature_size_z; yy < z + feature_size_z; yy++)
                                {
                                    if ((yy > 0) && (yy < img.height))
                                    {
                                        img.image[img.width - xx, img.height - yy] = (Byte)v.colour[0];
                                    }
                                }
                            }
                        }
                    }
                }
            }
            */

            
            //draw the features
            scale_factor = 1;
            feature_size_x = (int)(img.width * scale_factor / 50);
            feature_size_z = (int)(img.height * scale_factor / 50);
            foreach (Feature f in scene.feature_list)
            {
                //if (f.get_fully_initialised_feature_measurement_model() != null)
                if (f.get_successful_measurement_flag())
                {
                    position = f.get_y();
                    x = (int)((position[0] + half_x) / world_dimension_x * img.width);
                    if ((x > 0) && (x < img.width))
                    {
                        z = (int)((position[2] + half_z) / world_dimension_z * img.height);
                        if ((z > 0) && (z < img.height))
                        {
                            for (xx = x - feature_size_x; xx < x + feature_size_x; xx++)
                            {
                                if ((xx > 0) && (xx < img.width))
                                {
                                    for (yy = z - feature_size_z; yy < z + feature_size_z; yy++)
                                    {
                                        if ((yy > 0) && (yy < img.height))
                                        {
                                            img.image[img.width - xx, img.height - yy] = 255;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            


            //show the particles associated with partially initialised features
            /*
            foreach (FeatureInitInfo feat in scene.get_feature_init_info_vector())
            {
                foreach (Particle p in feat.particle_vector)
                {
                    position = p.get_h;
                    if (position != null)
                    {
                        x = (int)((position[0] + half_x) / world_dimension_x * img.width);
                        z = (int)((position[2] + half_z) / world_dimension_z * img.height);
                        if ((x > 0) && (x < img.width) && (z > 0) && (z < img.height))
                        {
                            for (c = 0; c < 3; c++)
                                img.image[img.width - x, img.height - z, c] = 255;
                        }
                    }
                }
            }
            */
            
        }

        /*
        public void ShowOverheadView(classimage_mono img, float world_dimension_x, float world_dimension_z)
        {
            int xx, yy, x, z, c, feature_size_x, feature_size_z, i, t, prev_x = 0, prev_z = 0;
            Vector position;
            float half_x = world_dimension_x / 2;
            float half_z = world_dimension_z / 2;

            img.clear();

            //draw the camera trace

            for (i = 0; i < TRACE_LENGTH; i++)
            {
                t = trace_index - i;
                if (t < 0) t += TRACE_LENGTH;
                if (!((camera_trace[t, 0] == 0) && (camera_trace[t, 1] == 0) && (camera_trace[t, 2] == 0)))
                {
                    x = img.width - (int)((camera_trace[t, 0] + half_x) / world_dimension_x * img.width);
                    z = img.height - (int)((camera_trace[t, 2] + half_z) / world_dimension_z * img.height);
                    if (i > 1)
                    {
                        img.drawLine(x, z, prev_x, prev_z, 0, 0, 255, 0);
                    }
                    prev_x = x;
                    prev_z = z;
                }
            }

            //draw the camera
            position = scene.get_xv();
            float scale_factor = 1.0f + ((position[1] + (world_dimension_x / 2)) / world_dimension_x);
            feature_size_x = (int)(img.width * scale_factor / 50);
            feature_size_z = (int)(img.height * scale_factor / 50);
            x = (int)((position[0] + half_x) / world_dimension_x * img.width);
            if ((x > 0) && (x < img.width))
            {
                z = (int)((position[2] + half_z) / world_dimension_z * img.height);
                if ((z > 0) && (z < img.height))
                {
                    for (xx = x - feature_size_x; xx < x + feature_size_x; xx++)
                    {
                        if ((xx > 0) && (xx < img.width))
                        {
                            for (yy = z - feature_size_z; yy < z + feature_size_z; yy++)
                            {
                                if ((yy > 0) && (yy < img.height))
                                {
                                    for (c = 0; c < 3; c++)
                                        img.image[img.width - xx, img.height - yy, c] = 255;
                                }
                            }
                        }
                    }
                }
            }



            //draw the features
            scale_factor = 1;
            feature_size_x = (int)(img.width * scale_factor / 50);
            feature_size_z = (int)(img.height * scale_factor / 50);
            foreach (Feature f in scene.feature_list)
            {
                //if (f.get_fully_initialised_feature_measurement_model() != null)
                if (f.get_successful_measurement_flag())
                {
                    position = f.get_y();
                    x = (int)((position[0] + half_x) / world_dimension_x * img.width);
                    if ((x > 0) && (x < img.width))
                    {
                        z = (int)((position[2] + half_z) / world_dimension_z * img.height);
                        if ((z > 0) && (z < img.height))
                        {
                            for (xx = x - feature_size_x; xx < x + feature_size_x; xx++)
                            {
                                if ((xx > 0) && (xx < img.width))
                                {
                                    for (yy = z - feature_size_z; yy < z + feature_size_z; yy++)
                                    {
                                        if ((yy > 0) && (yy < img.height))
                                        {
                                            img.image[img.width - xx, img.height - yy, 1] = 255;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

        }
        */


    }


}
