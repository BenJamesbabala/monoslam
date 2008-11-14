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
using System.Collections.Generic;
using System.Text;
using SceneLibrary;
using sentience;

namespace monoSLAM
{
// MonoSLAMInterface class which supplies a lot of extra access functions
// for use by a GUI, especially if the GUI is not part of the same program
// Kept separate here only for clarity of the main MonoSLAM class


    /// <summary>
    /// Provides an enhanced interface to the MonoSLAM real-time single-camera SLAM
    /// application. The MonoSLAM class provides the core interface, while
    /// MonoSLAMInterface provides a richer set of functions that a final application
    /// might need, such as for application control and user-feedback. Both this
    /// class and MonoSLAM are GUI-independent.
    /// </summary>
    public class MonoSLAMInterface : MonoSLAM
    {

        public MonoSLAMInterface(
            String initialisation_file,
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
            : base(initialisation_file, path,
               mm_creator,
               fmm_creator,
               imm_creator,
               number_of_features_to_select,
               number_of_features_to_keep_visible,
               max_features_to_init_at_once,
               min_lambda,
               max_lambda,
               number_of_particles,
               standard_deviation_depth_ratio,
               min_number_of_particles,
               prune_probability_threshold,
               erase_partially_init_feature_after_this_many_attempts,
               MAXIMUM_ANGLE_DIFFERENCE,
               calibration_target_width_mm,
               calibration_target_height_mm,
               calibration_target_distance_mm)
        {
        }


        // Getters and setters for data members of MonoSLAM class that an 
        // interface might want to use
        public void SetNumberOfFeaturesToSelect(uint newval)
        {
            NUMBER_OF_FEATURES_TO_SELECT = newval;
        }


        public uint GetNumberOfFeaturesToSelect()
        {
            return NUMBER_OF_FEATURES_TO_SELECT;
        }

        public void SetNumberOfFeaturesToKeepVisible(uint newval)
        {
            NUMBER_OF_FEATURES_TO_KEEP_VISIBLE = newval;
        }

        public uint GetNumberOfFeaturesToKeepVisible()
        {
            return NUMBER_OF_FEATURES_TO_KEEP_VISIBLE;
        }

        public void SetMaxFeaturesToInitAtOnce(uint newval)
        {
            MAX_FEATURES_TO_INIT_AT_ONCE = newval;
        }

        public uint GetMaxFeaturesToInitAtOnce()
        {
            return MAX_FEATURES_TO_INIT_AT_ONCE;
        }

        public void SetMinLambda(float newval)
        {
            MIN_LAMBDA = newval;
        }

        public float GetMinLambda()
        {
            return MIN_LAMBDA;
        }

        public void SetMaxLambda(float newval)
        {
            MAX_LAMBDA = newval;
        }

        public float GetMaxLambda()
        {
            return MAX_LAMBDA;
        }

        public void SetNumberOfParticles(uint newval)
        {
            NUMBER_OF_PARTICLES = newval;
        }

        public uint GetNumberOfParticles()
        {
            return NUMBER_OF_PARTICLES;
        }

        public void SetStandardDeviationDepthRatio(float newval)
        {
            STANDARD_DEVIATION_DEPTH_RATIO = newval;
        }

        public float GetStandardDeviationDepthRatio()
        {
            return STANDARD_DEVIATION_DEPTH_RATIO;
        }

        public void SetMinNumberOfParticles(uint newval)
        {
            MIN_NUMBER_OF_PARTICLES = newval;
        }

        public uint GetMinNumberOfParticles()
        {
            return MIN_NUMBER_OF_PARTICLES;
        }

        public void SetPruneProbabilityThreshold(float newval)
        {
            PRUNE_PROBABILITY_THRESHOLD = newval;
        }

        public float GetPruneProbabilityThreshold()
        {
            return PRUNE_PROBABILITY_THRESHOLD;
        }

        public void SetErasePartiallyInitFeatureAfterThisManyAttempts(uint newval)
        {
            ERASE_PARTIALLY_INIT_FEATURE_AFTER_THIS_MANY_ATTEMPTS = newval;
        }

        public uint GetErasePartiallyInitFeatureAfterThisManyAttempts()
        {
            return ERASE_PARTIALLY_INIT_FEATURE_AFTER_THIS_MANY_ATTEMPTS;
        }

        public void SetCameraWidth(uint newval)
        {
            CAMERA_WIDTH = newval;
        }

        public uint GetCameraWidth()
        {
            return CAMERA_WIDTH;
        }

        public void SetCameraHeight(uint newval)
        {
            CAMERA_HEIGHT = newval;
        }

        public uint GetCameraHeight()
        {
            return CAMERA_HEIGHT;
        }

        public void SetInitFeatureSearchUStart(uint newval)
        {
            init_feature_search_ustart = (int)newval;
        }

        public uint GetInitFeatureSearchUStart()
        {
            return (uint)init_feature_search_ustart;
        }

        public void SetInitFeatureSearchVStart(uint newval)
        {
            init_feature_search_vstart = (int)newval;
        }

        public uint GetInitFeatureSearchVStart()
        {
            return (uint)init_feature_search_vstart;
        }

        public void SetInitFeatureSearchUFinish(uint newval)
        {
            init_feature_search_ufinish = (int)newval;
        }

        public uint GetInitFeatureSearchUFinish()
        {
            return (uint)init_feature_search_ufinish;
        }

        public void SetInitFeatureSearchVFinish(uint newval)
        {
            init_feature_search_vfinish = (int)newval;
        }

        public uint GetInitFeatureSearchVFinish()
        {
            return (uint)init_feature_search_vfinish;
        }

        public void SetInitFeatureSearchRegionDefinedFlag(bool newstate)
        {
            init_feature_search_region_defined_flag = newstate;
        }

        public bool GetInitFeatureSearchRegionDefinedFlag()
        {
            return init_feature_search_region_defined_flag;
        }

        public bool GetLocationSelectedFlag()
        {
            return robot.location_selected();
        }

        public uint GetUU()
        {
            return robot.get_uu();
        }

        public uint GetVV()
        {
            return robot.get_vv();
        }

        public Partially_Initialised_Feature_Measurement_Model GetDefaultFeatureTypeForInitialisation()
        {
            return default_feature_type_for_initialisation;
        }

        public Scene_Single GetScene()
        {
            return scene;
        }

        public Scene_Single GetSceneNoConst()
        {
            return scene;
        }

        public Robot GetRobot()
        {
            return robot;
        }

        public Robot GetRobotNoConst()
        {
            return robot;
        }

        // What is the total number of features (full and partially-initialised)
        // in the system? 
        public uint GetNumberOfFeatures()
        {
            return (uint)GetScene().get_feature_list().Count;
        }

        // How many features are currently partially initalised? 
        public uint GetNumberOfPartiallyInitialisedFeatures()
        {
            return (uint)(GetScene().get_feature_init_info_vector()).Count;
        }

        // How many features are currently selected? 
        public uint GetNumberOfSelectedFeatures()
        {
            return (uint)GetScene().get_selected_feature_list().Count;
        }

        // How many features were matched in the last call to
        // GoOneStep()
        public uint GetNumberOfVisibleFeatures()
        {
            return number_of_visible_features;
        }

        // How many features were matched in the last call to
        // GoOneStep
        public uint GetNumberOfMatchedFeatures()
        {
            return number_of_matched_features;
        }

        // What was the velocity estimated in the last call to 
        // GoOneStep()?   
        public Vector Velocity()
        {
            return velocity;
        }

        // return the current speed in m/s
        public float Speed()
        {
            return speed;
        }


    }

}
