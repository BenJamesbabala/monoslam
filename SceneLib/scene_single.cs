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
using sentience;

namespace SceneLibrary
{
    /// <summary>
    /// The class to manage a single coupled-covariance SLAM map.
    /// </summary>
    public class Scene_Single 
    {
        //friend class Kalman;  // Nasty but we'll live with it for now


        //***************************Feature Management******************************
        public Feature find_feature_lab(int lab) {return find_feature_lab((uint)lab);}

  
        // Automatically delete features with failed measurements
        // The number of times a match attempt has to be made before
        // a feature is considered for automatic deletion by
        // delete_bad_features() (default = 10) 
        public uint MINIMUM_ATTEMPTED_MEASUREMENTS_OF_FEATURE;
        // The proportion of match attempts that have to fail before a feature is
        // automatically deleted in delete_bad_features() (default = 0.5) 
        public float SUCCESSFUL_MATCH_FRACTION;

        public bool get_successful_internal_measurement_flag(uint i) {return ((Internal_Measurement)internal_measurement_vector[(int)i]).successful_internal_measurement_flag;}

 
        public Motion_Model get_motion_model() {return motion_model;}

        /// Get the robot state x_v
        public Vector get_xv() {return xv;}
        /// Get the robot state x_v (non-const version to allow the state to
        /// be edited).
        public Vector get_xv_noconst() {return xv;}
        /// Get the robot state covariance P_{xx}
        public MatrixFixed get_Pxx() {return Pxx;}

        // Get the list of all the features in the system (initialised and
        // partially-initialised) 
        public ArrayList get_feature_list() {return feature_list;}
  
        // Get the list of all the features in the system (initialised and
        // partially-initialised) 
        public ArrayList get_feature_list_noconst() {return feature_list;}
        // Get the list of the selected features 
        public ArrayList get_selected_feature_list() {return selected_feature_list;}
        public ArrayList get_selected_feature_list_noconst() {return selected_feature_list;}
        public ArrayList get_feature_init_info_vector() {return feature_init_info_vector;}
        public ArrayList get_feature_init_info_vector_noconst() {return feature_init_info_vector;}

        public uint get_no_selected() {return (uint)selected_feature_list.Count;}
        public uint get_no_features() {return (uint)feature_list.Count;}

        public uint get_total_state_size() {return total_state_size;}
        public void increment_total_state_size(int increment) { total_state_size = (uint)((int)total_state_size + increment); }
        public uint get_successful_measurement_vector_size() {return successful_measurement_vector_size;}

        public void set_successful_measurement_vector_size(uint smvs) {successful_measurement_vector_size = smvs;}

        
        // Returns the label of the currently-marked feature.Marking a feature
        // is used to identify a feature for deletion (by calling
        // delete_feature()), or before calling print_marked_feature_state(),
        // get_marked_feature_state() or
        // get_feature_measurement_model_for_marked_feature().
        // @returns The label of the currently-marked feature, or -1 if no feature is
        // selected.
        public int get_marked_feature_label() {return marked_feature_label;}

        public Vector get_hv(uint i) {return ((Internal_Measurement)internal_measurement_vector[(int)i]).hv;}
        public Vector get_zv(uint i) {return ((Internal_Measurement)internal_measurement_vector[(int)i]).zv;}
        public MatrixFixed get_Sv(uint i) {return ((Internal_Measurement)internal_measurement_vector[(int)i]).Sv;}

        //*******************************Data members********************************
        protected Motion_Model motion_model;

        // The robot state x_v, as used by the motion model. The complete 
        // system state (robot and features) is not stored explicitely by Scene, but
        // instead is constructed as required by construct_total_state_and_covariance().
        protected Vector xv; 
        //  The robot state covariance P_{xx}, as used by the motion model.
        protected MatrixFixed Pxx; 

        // Lists of pointers to features
        // feature_list contains all features
        public ArrayList feature_list = new ArrayList();
        // selected_feature_list just lists those currently selected for 
        // measurement
        public ArrayList selected_feature_list = new ArrayList();

        // A separate list here of partially-initialised features with additional
        // probabilistic information. These features are also in the main 
        // feature_list. 
        protected ArrayList feature_init_info_vector = new ArrayList();

        // The size of the total state vector including robot and features
        protected uint total_state_size;
        // The size of the most recent vector of measurements
        protected uint successful_measurement_vector_size;

        // Unique labels to be given to new features
        protected uint next_free_label; 

        // Marked a feature: for deleting, steering round, etc.
        protected int marked_feature_label; // The label of the last feature selected 
                                  // with the mouse. Signed because -1 means
                                  // no marked feature 

        // A vector of numbered internal measurement models
        public ArrayList internal_measurement_vector = new ArrayList();

        // A counter for state output to text file
        protected uint output_counter;

        /// Do we want to print out status information each frame?
        public const bool STATUSDUMP = false;

        /// <summary>
        /// Old style constructor
        /// Constuct the Scene_Single class and set the initial robot state.
        /// </summary>
        /// <param name="initial_xv">The initial robot state</param>
        /// <param name="initial_Pxx">The initial robot state covariance</param>
        /// <param name="m_m">The motion model to use</param>
        public Scene_Single(Vector initial_xv, 
                            MatrixFixed initial_Pxx,
                            Motion_Model m_m)
        {
            motion_model = m_m;
            scene_constructor_bookkeeping(initial_xv, initial_Pxx);
        }

        /// <summary>
        /// Use this version if using internal measurement models defined in settings file.
        /// </summary>
        /// <param name="mm_creator"></param>
        /// <param name="imm_creator"></param>
        public Scene_Single(Settings settings, Motion_Model_Creator mm_creator,
			                Internal_Measurement_Model_Creator imm_creator)
        {
            // What is the motion model?
            ArrayList values = settings.get_entry("Models", "MotionModel");
            String model = (String)values[0];
  
            //assert(mm_creator != NULL);
            motion_model = mm_creator.create_model(model);

            if (motion_model == null)
            {
                Debug.WriteLine("Unable to create a motion model of type " + model +
                                " as requested in the initalisation file. ");
            }

            // Initialise the motion model with any settings
            motion_model.read_parameters(settings);
  
            // Create internal measurement models if required
            if (imm_creator != null) 
            {
                uint imm_number = 0;
                // Read in and create potentially various internal measurement models
                while (true) 
                {
                    // A list of internal measurement models will be given in the 
                    // Models section of settings with entry names 
                    // InternalMeasurementModel0, InternalMeasurementModel1, etc.
                    String entry_name = "InternalMeasurementModel" + Convert.ToString(imm_number);
                    values = settings.get_entry("Models", entry_name);
                    String new_imm_type = (String)values[0];

                    // Exit loop if no more internal measurement models
                    if (new_imm_type == "") break;

                    // Initialise new model and add to Scene
                    Internal_Measurement_Model new_imm = imm_creator.create_model(new_imm_type, motion_model);

                    // Note for future: at this stage we should potentially read in
                    // parameters for new imm; potentially we could have several internal
                    // measurements define which have the same model but different parameters
                    // (just like we have multiple features)
      
                    add_internal_measurement(new_imm);

                    imm_number++;
                }
            }


            // Get the initial state settings
            Vector initial_xv = null;
            MatrixFixed initial_Pxx = null;
            motion_model.read_initial_state(settings, ref initial_xv, ref initial_Pxx);

            scene_constructor_bookkeeping(initial_xv, initial_Pxx);
        }

        /// <summary>
        /// Common stuff for all constructors.
        /// </summary>
        /// <param name="?"></param>
        void scene_constructor_bookkeeping(Vector initial_xv, 
                                           MatrixFixed initial_Pxx)
        {
            // Setting these constants should be moved to read from settings file eventually
            MINIMUM_ATTEMPTED_MEASUREMENTS_OF_FEATURE = 10;
            SUCCESSFUL_MATCH_FRACTION = 0.5f;

            xv = initial_xv;
            Pxx = initial_Pxx;

            // Bookkeeping initialisation
            next_free_label = 0;

            marked_feature_label = -1;

            total_state_size = motion_model.STATE_SIZE;

            output_counter = 0;
        }

        //*****************************Feature Management******************************

        /// <summary>
        /// Returns the feature with a given label. If the feature does not exist, NULL is returned.
        /// </summary>
        /// <param name="lab"></param>
        /// <returns></returns>
        public Feature find_feature_lab(uint lab)
        {
            Feature it, retval = null;
            int i=0;
            while ((i<feature_list.Count) && (retval==null))
            {
                it = (Feature)feature_list[i];
                if (it.get_label() == lab) retval = it;
                i++;
            }
            return(retval);
        }


        /// <summary>
        /// Create a new partially-initialised feature. This creates a new Feature, and
        /// creates a new empty FeatureInitInfo to store the extra initialisation
        /// information, which must be then filled in by the caller of this function.
        /// </summary>
        /// <param name="id">The unique identifier for this feature (e.g. the image patch)</param>
        /// <param name="h">The initial measured state for this feature (e.g. the image location)</param>
        /// <param name="f_m_m">The partially-initialised feature measurement model to apply to this feature.</param>
        /// <returns>A pointer to the FeatureInitInfo object to be filled in with further initialisation information.</returns>
        public FeatureInitInfo add_new_partially_initialised_feature(classimage_mono id, 
                               Vector h, Partially_Initialised_Feature_Measurement_Model f_m_m,
                               Vector feature_colour)
        {
            Feature nf = new Feature(id, next_free_label, (uint)feature_list.Count, this, h, f_m_m, feature_colour);
            
            add_new_feature_bookeeping(nf);

            // Set stuff to store extra probability information for partially
            // initialised feature
            FeatureInitInfo feat = new FeatureInitInfo(this, nf);
            feature_init_info_vector.Add(feat);

            return (feat);
        }


        public void add_new_known_feature(classimage_mono id, Vector y_known, 
                                          Vector xp_o, Feature_Measurement_Model f_m_m,
                                          uint known_feature_label)
        {
            Feature nf = new Feature(id, next_free_label, (uint)feature_list.Count, 
                                     this, y_known, xp_o, f_m_m, known_feature_label);
            add_new_feature_bookeeping(nf);
        }

        /// <summary>
        /// Function to unite the bookeeping stuff for both
        /// add_new_partially_initialised_feature() and add_new_known_feature()
        /// </summary>
        /// <param name="nf"></param>
        public void add_new_feature_bookeeping(Feature nf)
        {  
            feature_list.Add(nf);

            next_free_label++; // Potential millenium-style bug when this overloads
                               // (hello if you're reading this in the year 3000)

            nf.set_position_in_total_state_vector(total_state_size);
            total_state_size += nf.get_feature_measurement_model().FEATURE_STATE_SIZE;
        }


        /// <summary>
        /// Delete the currently-marked feature. Features can be marked using
        /// mark_feature_by_lab(). The function also frees up the identifier.
        /// </summary>
        /// <returns></returns>
        public bool delete_feature()
        {
            if (marked_feature_label == -1)
            {
                Debug.WriteLine("No feature marked to delete.");
                return false;
            }
            else
            {
                Debug.WriteLine("Deleting feature " + marked_feature_label);
            }
             
            Feature it, it_to_delete = null;
            int i=0;
            while ((i<feature_list.Count) && (it_to_delete==null))
            {
                it = (Feature)feature_list[i];
                if (it.get_label() == marked_feature_label) it_to_delete = it;
                i++;
            }


            if (it_to_delete == null) 
            {
                Debug.WriteLine("Error: marked feature not found in list.");
                return false;
            }

            // Remove the covariance elements relating to this feature from the 
            // subsequent features
            while (i < feature_list.Count)
            {
                it = (Feature)feature_list[i];
                if (it != it_to_delete)
                {
                    it.feature_is_removed(it_to_delete.get_position_in_list());
                    it.increment_position_in_total_state_vector(-(int)it_to_delete.get_feature_measurement_model().FEATURE_STATE_SIZE);
                }
                i++;
            }

            if (it_to_delete.get_selected_flag()) deselect_feature(it_to_delete);

            total_state_size -= it_to_delete.get_feature_measurement_model().FEATURE_STATE_SIZE;

            // Delete extra data associated with this feature
            feature_list.Remove(it_to_delete);

            marked_feature_label = -1;  

            return true;
        }

        // Delete all features with scheduled_for_termination_flag set
        public void exterminate_features()
        {
            int currently_marked_feature;
            bool deleting_last_feature_flag = false;
            Feature it, it_to_delete = null;
            int i=0;
            while ((i<feature_list.Count) && (it_to_delete==null))
            {
                it = (Feature)feature_list[i];
                if (it.get_scheduled_for_termination_flag())
                {
                    it_to_delete = it;
                    if (i == feature_list.Count-1) deleting_last_feature_flag=true;
                    // Save currently marked feature so we can mark this scheduled
                    // for termination feature (delete_feature deletes the marked feature)
                    currently_marked_feature = get_marked_feature_label();
                    // Unless it's the one we're about to delete
                    if (currently_marked_feature == (int) (it_to_delete.get_label()))
                        currently_marked_feature = -1;
                    mark_feature_by_lab((int)it_to_delete.get_label());
                    delete_feature();
                    // Now re-mark currently marked feature
                    if (currently_marked_feature != -1)
                        mark_feature_by_lab(currently_marked_feature);
                    if (deleting_last_feature_flag) 
                    {
                        // jump out of loop
                        break;
                    }
                }
                i++;
            }
            
        }

        /// <summary>
        /// Delete any features which are consistently failing measurements. Features are
        /// deleted if there has been more than a certain number of attempts to match them
        /// (set by Scene_Single::MINIMUM_ATTEMPTED_MEASUREMENTS_OF_FEATURE) and they have
        /// been successfully matched on too few of those occasions (set by
        /// Scene_Single::SUCCESSFUL_MATCH_FRACTION).
        /// </summary>
        public void delete_bad_features()
        {
            Feature it;
            int i=0;
            while (i < feature_list.Count)
            {
                it = (Feature)feature_list[i];

                float successful_ratio = (float)((it.get_successful_measurements_of_feature())) / (float)((it.get_attempted_measurements_of_feature()));

                // First: test if this feature needs deleting 
                if ((it.get_attempted_measurements_of_feature() >= MINIMUM_ATTEMPTED_MEASUREMENTS_OF_FEATURE) &&
                    (successful_ratio < SUCCESSFUL_MATCH_FRACTION))
                {
                    it.set_scheduled_for_termination_flag(true);
                    //continue;
                }
                
                i++;
            }
        
            // Delete bad features
            exterminate_features();
        }


        /// <summary>
        /// Set the current marked feature. Marking a feature is used to identify a feature
        /// for deletion (by calling delete_feature()), or before calling
        /// print_marked_feature_state(), get_marked_feature_state() or
        /// get_feature_measurement_model_for_marked_feature().
        /// </summary>
        /// <param name="lab">The label (starting from zero) for the feature to mark. A setting of -1 indicates no selection.</param>
        public void mark_feature_by_lab(int lab)
        {
            // Check this is valid
            // Can we find it?
            Feature it, found = null;
            int i=0;
            while (i < feature_list.Count)
            {
                it = (Feature)feature_list[i];

                if (it.get_label() == (uint)(lab)) { found = it; break; }
                
                i++;
            }
            if ((found == null) && (lab != -1))
            {
                Debug.WriteLine("Attempted to mark feature with unknown label: " + Convert.ToString(lab));
                return;
            }

            marked_feature_label = lab;
        }

        /// <summary>
        /// Mark the first feature in the features list.
        /// </summary>
        /// <returns></returns>
        public bool mark_first_feature()
        {
            if (feature_list.Count == 0)
                return false;
            else
            {
                mark_feature_by_lab((int)((Feature)feature_list[0]).get_label());
                return true;
            }
        }

        //*******************Feature Selection and Measurement************************

        /// <summary>
        /// Add this feature to the list for selection.
        /// </summary>
        /// <param name="fp">The feature to add.</param>
        /// <returns></returns>
        public bool select_feature(Feature fp)
        {
            if (fp.get_selected_flag() == true)
            {
                //if (DEBUGDUMP) cout << "Feature with label " << fp.get_label() 
		            //<< " is already selected." << endl;
                return true;
            }

            fp.set_selected_flag(true);

            selected_feature_list.Add(fp);

            return true;
        }

        /// <summary>
        /// Remove this feature from the list for selection.
        /// </summary>
        /// <param name="fp">The feature to remove.</param>
        /// <returns></returns>
        public bool deselect_feature(Feature fp)
        { 
            if (fp.selected_flag == false)
            {
                //if (DEBUGDUMP) cout << "Feature with label " << fp.label 
		          //  << " is already deselected." << endl;
            }
            fp.set_selected_flag(false);
            selected_feature_list.Remove(fp);
            return true;
        }

        /// <summary>
        /// Toggle the selection of a feature (for making measurements). This is called
        /// to manually select or deselect a feature.
        /// </summary>
        /// <param name="lab">The label (starting from zero) for the feature to toggle.</param>
        /// <returns></returns>
        public bool toggle_feature_lab(uint lab)
        {
            Feature fp;

            fp = find_feature_lab(lab);
            if (fp == null)
            {
                Debug.WriteLine("Feature with label " + Convert.ToString(lab) + " not found.");
                return false;
            }

            if (fp.get_selected_flag())
                return deselect_feature(fp);
            else
                return select_feature(fp);
        }


        public bool auto_select_feature()
        {
            uint cant_see_flag;  // Flag which we will set to 0 if a feature is 
			                     // visible and various other values if it isn't
            float max_score = 0.0f;
            int visible_features = 0;

            Feature it, fp_selected = null;
            int i=0;
            float score=0;
            while (i < feature_list.Count)
            {
                it = (Feature)feature_list[i];

                predict_single_feature_measurements(it);
                // See if the feature is visible 
                cant_see_flag = it.get_feature_measurement_model().visibility_test(
                                motion_model.get_xpRES(), it.get_y(), 
			                    it.get_xp_orig(), it.get_h());

                // Feature has passed visibility tests 
                if (cant_see_flag == 0)
                {
                    visible_features++;
                    score = it.get_feature_measurement_model().selection_score(it.get_feature_measurement_model().get_SiRES());
                    if (score > max_score)
                    { 
	                    max_score = score;
	                    fp_selected = it;
                    }
                }
                
                i++;
            }

            // Deselect all features
            for (i = 0; i < selected_feature_list.Count; i++)
            {
                it = (Feature)feature_list[i];
                deselect_feature(it);
            }

            //if (DEBUGDUMP) cout << "Best score found: " << max_score << ".\n";

            // If we've found a suitable feature, select it
            if (max_score == 0.0)
            {
                //if (DEBUGDUMP) cout << "No visible features: find some new ones." << endl;
                return false;
            }
            else 
            {
                select_feature(fp_selected);
                //if (DEBUGDUMP) cout << "Auto-selected feature with label " << fp_selected.label << ".\n";
                return true;
            }
        }

        /// <summary>
        /// Array of the scores we've found for local use
        /// </summary>
        internal class FeatureAndScore 
        {
            public FeatureAndScore() { score = 0; fp = null; }
            public FeatureAndScore(float s, Feature f) { score = s; fp = f; }
            public float score;
            public Feature fp;
        }

        /// <summary>
        /// Automatically select the n features with the best selection scores. For each
        /// feature, this predicts their location and calls
        /// Feature_Measurement_Model::selection_score(), selecting the n with the best score.
        /// </summary>
        /// <param name="n"></param>
        /// <returns>the number of visible features.</returns>
        public uint auto_select_n_features(uint n)
        {
            uint cant_see_flag;  // Flag which we will set to 0 if a feature is 
                                 // visible and various other values if it isn't

            // Deselect all features
            int j;
            float score=0;

            //deselect features
            foreach (Feature f in selected_feature_list)
            {
                f.set_selected_flag(false);
            }
            selected_feature_list.Clear();

            // Have vector of up to n scores; highest first
            ArrayList feature_and_score_vector = new ArrayList();
            FeatureAndScore fas, fas_it;
            bool already_added = false;

            foreach (Feature it in feature_list)
            {
                if (it.get_feature_measurement_model().fully_initialised_flag)
                {
                    predict_single_feature_measurements(it);
                    // See if the feature is visible 
                    cant_see_flag = it.visibility_test();

                    // Feature has passed visibility tests 
                    if (cant_see_flag == 0)
                    {
                        score = it.get_feature_measurement_model().selection_score(it.get_feature_measurement_model().get_SiRES());

                        fas = new FeatureAndScore(score, it);

                        already_added = false;

                        j=0;
                        while ((!already_added) && (j<feature_and_score_vector.Count))
                        {
                            fas_it = (FeatureAndScore)feature_and_score_vector[j];

                            if (score > fas_it.score)
                            {
                                // Insert new feature before old one it trumps
                                feature_and_score_vector.Insert(j, fas);
                                already_added = true;
                                break;
                            }
                            j++;
                        }

                        if (!already_added)
                            feature_and_score_vector.Add(fas);
                    }
                }
            }


            // See what we've got
            uint n_actual = 0;
            if (feature_and_score_vector.Count == 0)
            {
                Debug.WriteLine("No visible features: find some new ones.");
                return 0;
            }
            else
            {
                for (j = 0; j < feature_and_score_vector.Count; j++)
                {
                    fas_it = (FeatureAndScore)feature_and_score_vector[j];

                    if ((fas_it.score == 0.0) || (n_actual == n)) 
                        return (uint)feature_and_score_vector.Count;
                    else
                    {
                        select_feature(fas_it.fp);
                        //if (DEBUGDUMP) cout << "Auto-selected feature with label "
                            //<< (*fasit).fp.label
                            //<< ", score " << (*fasit).score << endl;
                        n_actual++;
                    }
                }
            }

            // Return the number of visible features
            return (uint)feature_and_score_vector.Count;
        }


        public void starting_measurements()
        {
            successful_measurement_vector_size = 0;
        }

 
        public void failed_measurement_of_feature(Feature sfp)
        {
            //if (STATUSDUMP) cout << "Measurement failed of feature with label " << sfp.label << endl; 
            sfp.set_successful_measurement_flag(false);
            sfp.increment_attempted_measurements_of_feature(1);
        }

        public void successful_measurement_of_feature(Feature sfp)
        {
            sfp.set_successful_measurement_flag(true);
            successful_measurement_vector_size += 
                sfp.get_fully_initialised_feature_measurement_model().MEASUREMENT_SIZE;
 
            sfp.get_fully_initialised_feature_measurement_model().func_nui(sfp.get_h(), sfp.get_z());
            sfp.set_nu(sfp.get_fully_initialised_feature_measurement_model().get_nuiRES());

            sfp.increment_successful_measurements_of_feature(1);
            sfp.increment_attempted_measurements_of_feature(1);
        }


        public void construct_total_measurement_stuff(Vector nu_tot, 
				                                      MatrixFixed dh_by_dx_tot,
				                                      MatrixFixed R_tot)
        {
            uint size = successful_measurement_vector_size;

            //assert (nu_tot.Size() == size && 
	                //dh_by_dx_tot.Rows() == size && 
	                //dh_by_dx_tot.Cols() == total_state_size &&
	                //R_tot.Rows() == size && R_tot.Cols() == size);

            nu_tot.Fill(0.0f);
            dh_by_dx_tot.Fill(0.0f);
            R_tot.Fill(0.0f);

            uint vector_position = 0;

            Feature it;
            for (int i=0;i<selected_feature_list.Count;i++)
            {
                it = (Feature)selected_feature_list[i];

                if (it.get_successful_measurement_flag())
                {
                    nu_tot.Update(it.get_nu(), (int)vector_position);

                    dh_by_dx_tot.Update(it.get_dh_by_dxv(), (int)vector_position, 0);

                    dh_by_dx_tot.Update(it.get_dh_by_dy(), (int)vector_position,
                        (int)it.get_position_in_total_state_vector());

                    R_tot.Update(it.get_R(), (int)vector_position, (int)vector_position);

                    vector_position += it.get_feature_measurement_model().MEASUREMENT_SIZE;
                }
            }
        }

        /// <summary>
        /// Predict the measurement location and innovation covariance for each of the
        /// selected features. This calls predict_single_feature_measurements() for each
        /// feature in the selected_feature_list.
        /// </summary>
        public void predict_measurements()
        {            
            Feature it;
            for (int i=0;i<selected_feature_list.Count;i++)
            {
                it = (Feature)selected_feature_list[i];
                predict_single_feature_measurements(it);
            }
        }

        /// <summary>
        /// For a single feature, work out the predicted feature measurement and the
        /// Jacobians with respect to the vehicle position and the feature position.
        /// This calls the appropriate member functions in Feature to set the measurement
        /// \vct{h}, the feature location Jacobian \partfracv{h}{y}, robot
        /// position Jacobian \partfrac{\vct{h}}{\vct{x}_v}, measurement covariance
        /// \mat{R} and innovation covariance \mat{S} respectively.
        /// </summary>
        /// <param name="sfp"></param>
        public void predict_single_feature_measurements(Feature sfp)
        {
            motion_model.func_xp(xv);
            Vector xpRES = motion_model.get_xpRES();
            sfp.get_fully_initialised_feature_measurement_model().func_hi_and_dhi_by_dxp_and_dhi_by_dyi(sfp.get_y(),xpRES);

            sfp.set_h(sfp.get_fully_initialised_feature_measurement_model().get_hiRES());
            sfp.set_dh_by_dy(sfp.get_fully_initialised_feature_measurement_model().get_dhi_by_dyiRES());

            motion_model.func_dxp_by_dxv(xv);
  
            sfp.set_dh_by_dxv(sfp.get_fully_initialised_feature_measurement_model().get_dhi_by_dxpRES() * motion_model.get_dxp_by_dxvRES());

            sfp.get_fully_initialised_feature_measurement_model().func_Ri(sfp.get_h()); 
            sfp.set_R(sfp.get_fully_initialised_feature_measurement_model().get_RiRES());

            sfp.get_fully_initialised_feature_measurement_model().func_Si(Pxx, sfp.get_Pxy(), sfp.get_Pyy(), 
		                                                sfp.get_dh_by_dxv(), sfp.get_dh_by_dy(), sfp.get_R());
            sfp.set_S(sfp.get_fully_initialised_feature_measurement_model().get_SiRES());
        }

        //*********************Partially Initialised Features**************************

        /// <summary>
        /// Add a particle to the most recent FeatureInitInfo (and so the most
        /// recently-created partially-initialised feature). This just calls
        /// FeatureInitInfo::add_particle().
        /// </summary>
        /// <param name="lambda">The value(s) of lambda to assign to this particle.</param>
        /// <param name="probability">The initial probability to assign to this particle.</param>
        public void add_particle_to_newest_partially_init_feature(Vector lambda, float probability)
        {
            // Particle is added to the last (newest) element of 
            // feature_init_info_vector
            FeatureInitInfo fi = (FeatureInitInfo)feature_init_info_vector[feature_init_info_vector.Count-1];
            fi.add_particle(lambda, probability);
        }


        /// <summary>
        /// Update the measurement states of the partially-initialised features to
        /// correspond to the current robot state. This is called prior to matching them
        /// in a new frame. For each Particle in each FeatureInitInfo, the measurement state
        /// h_{pi} is updated, and the inverse innovation covariance
        /// S_i^{-1}, and the determinant of S_i are stored.
        /// </summary>
        public void predict_partially_initialised_feature_measurements()
        {
            FeatureInitInfo feat;
            Feature fp;

            // Get the current position
            motion_model.func_xp(xv);
            //Vector local_xp = motion_model.get_xpRES();
            Vector local_xp = new Vector(motion_model.get_xpRES());
            motion_model.func_dxp_by_dxv(xv);
            MatrixFixed local_dxp_by_dxv = new MatrixFixed(motion_model.get_dxp_by_dxvRES());
            //MatrixFixed local_dxp_by_dxv = motion_model.get_dxp_by_dxvRES();

            // Loop over different partially initialised features
            for (int i = 0; i < feature_init_info_vector.Count; i++)
            {
                feat = (FeatureInitInfo)feature_init_info_vector[i];
                fp = feat.get_fp();

                
                Partially_Initialised_Feature_Measurement_Model pifmm = 
                    fp.get_partially_initialised_feature_measurement_model();
    
                // We don't try to match a feature immediately after it has been
                // initialised
                if (feat.number_of_match_attempts++ != 0)
                {
                    feat.making_measurement_on_this_step_flag = true;

                    //if (DEBUGDUMP) cout 
                    //    << "Predicting partially initialised measurements for feature label " 
                    //    << fp.get_label() << endl;

                    // Loop over particles for a particular feature
                    // Fill in h, Sinv, detS information predicting measurement
                    foreach (Particle part in feat.particle_vector)
                    {
                        // Get and update the measurement state for the current feature with
                        // this particle's lambda from the current position
                        pifmm.func_hpi_and_dhpi_by_dxp_and_dhpi_by_dyi(
                            fp.get_y(), local_xp, part.lambda);

                        Vector hpiRES = pifmm.get_hpiRES();
                        part.set_h(hpiRES);
                        
                        // Set the measurement covariance from the feature's measurement
                        // state
                        fp.get_feature_measurement_model().func_Ri(part.get_h());  
                        
                        // And calculate the innovation covariance
                        // (i.e. combine the uncertainties in robot state, feature state, and
                        // measurement to give the overall uncertainty in the measurement state
                        // This will be used to determine what area of the image to search
                        fp.get_feature_measurement_model().func_Si(
                            Pxx, fp.get_Pxy(), fp.get_Pyy(), pifmm.get_dhpi_by_dxpRES() * local_dxp_by_dxv, 
                            pifmm.get_dhpi_by_dyiRES(), fp.get_feature_measurement_model().get_RiRES());
                        
                        part.set_S(fp.get_feature_measurement_model().get_SiRES());                                                 
                    }
                }
                else
                {
                    feat.making_measurement_on_this_step_flag = false;
                }
                 
                 
            }
        }


        /// <summary>
        /// Update the probabiluties of the partially-initialised features after
        /// their measurement. For each Particle in each FeatureInitInfo this updates the
        /// probabilities of the particles according to their measurement (i.e how big the
        /// innovation \nu_i = z_i - h_i was compared to the innovation covariance
        /// S_i), and then normalises the probabilities.
        /// Feature::prune_particle_vector() is also called to prune the particles, and any
        /// features where all the particle measurements failed are deleted.
        /// </summary>
        /// <param name="prune_probability_threshold"></param>
        public void update_partially_initialised_feature_probabilities(float prune_probability_threshold)
        {
            // Loop over different partially initialised features
            float likelihood=0,nuT_Sinv_nu=0,pux=0,diff=0,prev_prob=0;
            Vector nu;
            Vector SInv_times_nu;
            int i;

            for (i=0;i<feature_init_info_vector.Count;i++)
            {
                FeatureInitInfo feat = (FeatureInitInfo)feature_init_info_vector[i];

                if (feat.making_measurement_on_this_step_flag)
                {
                    //if (DEBUGDUMP) cout
                    //    << "Updating partially initialised probabilities for feature label "
                    //    << feat.get_fp().get_label() << endl;

                    // Update the probabilities.
                    likelihood = 0;
                    diff = 0;
                    prev_prob = 999;
                    foreach(Particle it in feat.particle_vector)
                    {                        
                        if (it.get_successful_measurement_flag() == true)
                        {
                            // Evaluate Gaussian
                            nu = it.get_z() - it.get_h();
                            SInv_times_nu = it.get_SInv() * nu;
                            // A scalar
                            nuT_Sinv_nu = Vector.DotProduct(nu, SInv_times_nu);
                            // pux is the probability that the true feature lies in this position
                            pux = (1.0f / ((float)Math.Sqrt(2.0f * 3.1415927 * it.get_detS()))) * (float)Math.Exp(-0.5 * nuT_Sinv_nu);

                            likelihood = pux;
                        }
                        else
                        {
                            likelihood = 0.0f;
                        }

                        // And Bayes rule
                        it.probability = it.probability * likelihood;

                        if (prev_prob != 999) diff += Math.Abs(it.probability - prev_prob);
                        prev_prob = it.probability;
                    }

                    if (diff == 0)
                    {
                        feat.nonzerovariance = false;
                    }
                    else
                    {
                        feat.nonzerovariance = true;
                    }

                    
                    if (feat.normalise_particle_vector_and_calculate_cumulative())
                    {
                        feat.prune_particle_vector(prune_probability_threshold);

                        feat.calculate_mean_and_covariance();
                        //if (DEBUGDUMP) cout << "Particles " << feat.particle_vector.size()
                          //  << " Mean " << feat.mean
                            //<< " variance " << feat.covariance
                            //<< endl;
                    }
                    else
                    {
                        // normalise_particle_vector_and_calculate_cumulative
                        // returns false if all probabilities are zero (all failed matches)
                        // so delete this feature
                        delete_partially_initialised_feature(feat);
                        i--;
                    }
                     
                }
            }
        }

        /// <summary>
        /// Go through the list of partially-initialised features and delete any that have
        /// timed out. A feature times out if it has not collapsed after a certain number
        /// of attempts, or if it has fewer than the minimum number of particles.
        /// </summary>
        /// <param name="erase_partially_init_feature_after_this_many_attempts"> The number of match attempts before the particle is deleted.</param>
        /// <param name="min_number_of_particles"> The number of particles below which the feature is deleted.</param>
        public void delete_partially_initialised_features_past_sell_by_date(
            uint erase_partially_init_feature_after_this_many_attempts, uint min_number_of_particles)
        {
            FeatureInitInfo feat;
            for (int i=0;i<feature_init_info_vector.Count;i++)
            {
                feat = (FeatureInitInfo)feature_init_info_vector[i];

                if ((feat.number_of_match_attempts > erase_partially_init_feature_after_this_many_attempts) ||
                    (feat.particle_vector.Count <= (int)min_number_of_particles))
                {
                    //if (DEBUGDUMP) cout << "Deleting feature with label "
                      //    << feat.fp.get_label() << " after "
                        //  << feat.number_of_match_attempts
                          //<< " match attempts." << endl;
                    delete_partially_initialised_feature(feat);
                    i--;
                }
            }
        }


        public void delete_partially_initialised_feature(FeatureInitInfo feat)
        {
            //feat.get_fp().get_identifier();
            // Save currently marked feature
            int currently_marked_feature = get_marked_feature_label();
            // Mark feature to delete
            mark_feature_by_lab((int)feat.get_fp().get_label());
            delete_feature();
            feature_init_info_vector.Remove(feat);
            if (currently_marked_feature != -1)
                mark_feature_by_lab(currently_marked_feature);
        }


        //***************************Internal Measurements*****************************

        public void add_internal_measurement(Internal_Measurement_Model internal_measurement_model)
        {
            Internal_Measurement im = new Internal_Measurement(internal_measurement_model);
            internal_measurement_vector.Add(im);
        }


        public void predict_internal_measurement(uint i)
        {
            Internal_Measurement im = (Internal_Measurement)internal_measurement_vector[(int)i];

            //assert(im.internal_measurement_model != NULL);

            im.predict_internal_measurement(xv, Pxx);
        }


        public void construct_total_internal_measurement_stuff(
				       Vector nu_tot, MatrixFixed dh_by_dx_tot,
				       MatrixFixed R_tot, uint i)
        {
            ((Internal_Measurement)internal_measurement_vector[(int)i]).construct_total_internal_measurement_stuff(
                  nu_tot, dh_by_dx_tot, R_tot, total_state_size);
        }


        public void successful_internal_measurement(Vector zv_in, uint i)
        {
            ((Internal_Measurement)internal_measurement_vector[(int)i]).successful_internal_measurement(zv_in);
        }


        public void failed_internal_measurement(uint i)
        {
            ((Internal_Measurement)internal_measurement_vector[(int)i]).failed_internal_measurement();
        }


        //*************************Some important operations***************************


        public void zero_axes()
        {
            uint size = get_total_state_size();
            Vector x = new Vector(size);
            x.Fill(0.0f);
            MatrixFixed dxnew_by_dxold = new MatrixFixed(size, size);
            dxnew_by_dxold.Fill(0.0f);

            // We form the new state and Jacobian of the new state w.r.t. the old state
            uint state_position = 0;
  
            // Robot part
            motion_model.func_zeroedxv_and_dzeroedxv_by_dxv(xv);
            x.Update(motion_model.get_zeroedxvRES(), 0);
            dxnew_by_dxold.Update(motion_model.get_dzeroedxv_by_dxvRES(), 0, 0);

            state_position += motion_model.STATE_SIZE;

            // Each feature in turn
            uint feature_no = 0;

            // Copy these to be on the safe side
            motion_model.func_xp(xv);
            Vector local_xp = new Vector(motion_model.get_xpRES());
            //Vector local_xp = motion_model.get_xpRES();
            motion_model.func_dxp_by_dxv(xv);
            //MatrixFixed local_dxp_by_dxv = motion_model.get_dxp_by_dxvRES();
            MatrixFixed local_dxp_by_dxv = new MatrixFixed(motion_model.get_dxp_by_dxvRES());

            Feature it;
            MatrixFixed Temp_FS1;
            for (int i=0; i < feature_list.Count; i++)
            {
                it = (Feature)feature_list[i];
                it.get_feature_measurement_model().func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(it.get_y(), local_xp);
                x.Update(it.get_feature_measurement_model().get_zeroedyiRES(), (int)state_position);

                // Calculate dzeroedyi_by_dxv in Temp_FS1
                Temp_FS1 = it.get_feature_measurement_model().get_dzeroedyi_by_dxpRES() * local_dxp_by_dxv;

                dxnew_by_dxold.Update(Temp_FS1, (int)state_position, 0);
                dxnew_by_dxold.Update(it.get_feature_measurement_model().get_dzeroedyi_by_dyiRES(), 
                    (int)state_position, (int)state_position);

                // We also want to redefine xp_orig for this feature: the robot
                // position at which is was initialised into the map (used by
                // functions for checking visibility)
                motion_model.func_xpredef_and_dxpredef_by_dxp_and_dxpredef_by_dxpdef(
                                                        it.get_xp_orig(), local_xp);
                it.set_xp_orig(it.get_feature_measurement_model().get_motion_model().get_xpredefRES());

                feature_no++;
                state_position += it.get_feature_measurement_model().FEATURE_STATE_SIZE;
            }

            // Check we've counted properly
            //assert (feature_no == feature_list.size() && 
	        //  state_position == total_state_size);

            // Do a Jacobian transform to get the new covariance
            MatrixFixed P = new MatrixFixed(size, size);
            construct_total_covariance(ref P);

            P = dxnew_by_dxold * P * dxnew_by_dxold.Transpose();

            // Finally load the scene data structures with the new state and covariance
            fill_states(x);
            fill_covariances(P);
        }


        public void normalise_state()
        {
            // cout << "Normalising state." << endl;

            // Normalising state:

            // This deals with the case where the robot state needs normalising
            // (e.g. if it contains a quaternion)

            // We assume the feature states do not need normalising

            motion_model.func_xvnorm_and_dxvnorm_by_dxv(xv);

            // Change the state vector
            xv.Update(motion_model.get_xvnormRES());

            // Change the vehicle state covariance
            Pxx.Update(motion_model.get_dxvnorm_by_dxvRES() * Pxx * 
	            motion_model.get_dxvnorm_by_dxvRES().Transpose(), 0, 0);

            // Change the covariances between vehicle state and feature states 
            foreach (Feature it in feature_list)
            {
                it.set_Pxy(motion_model.get_dxvnorm_by_dxvRES() * it.get_Pxy());
            }
        }

        //*******************Manipulate total state and covariance*********************

        /// <summary>
        /// Create the overall state vector and covariance matrix by concatenating the robot
        /// and feature states and covariances.
        /// </summary>
        /// <param name="V">The vector to fill with the state</param>
        /// <param name="M">The matrix to fill with the covariance</param>
        public void construct_total_state_and_covariance(ref Vector V, ref MatrixFixed M)
        {
            construct_total_state(ref V);
            construct_total_covariance(ref M);
        }


        public void fill_state_and_covariance(Vector V, MatrixFixed M)
        {  
            fill_states(V);
            fill_covariances(M);
        }

        /// <summary>
        /// Create the overall state vector by concatenating the robot state $x_v$ and all the feature states y_i
        /// </summary>
        /// <param name="V">The vector to fill with the state</param>
        public void construct_total_state(ref Vector V)
        {
            //assert(V.Size() == total_state_size);

            int y_position = 0;

            V.Update(xv, y_position);
            y_position += (int)motion_model.STATE_SIZE;

            uint y_feature_no = 0;

            Feature it;
            for (int i=0;i<feature_list.Count;i++)
            {
                it = (Feature)feature_list[i];
                V.Update(it.get_y(), y_position);
                y_feature_no++;
                y_position += (int)it.get_feature_measurement_model().FEATURE_STATE_SIZE;
            }

            //assert (y_feature_no == feature_list.size() && y_position == total_state_size);
        }

        /// <summary>
        /// this is a test function used for debugging
        /// </summary>
        /// <returns></returns>
        public uint get_tot_state_size()
        {
            Feature it;
            uint total = motion_model.STATE_SIZE;

            for (int i = 0; i < feature_list.Count; i++)
            {
                it = (Feature)feature_list[i];
                total += it.get_feature_measurement_model().FEATURE_STATE_SIZE;
            }

            return (total);
        }

        /// <summary>
        /// Create the overall covariance matrix by concatenating the robot
        /// covariance P_{xx} and all the feature covariances P_{xy_i}, P_{y_iy_i}
        /// and P_{y_iy_j}.
        /// </summary>
        /// <param name="M">The matrix to fill with the state</param>
        public void construct_total_covariance(ref MatrixFixed M)
        {
            if ((M.Rows != total_state_size) || (M.Columns != total_state_size))
            {
                Debug.WriteLine("Error: Matrix is the wrong size");
            }

            //test
            uint test_size = get_tot_state_size();
            if (test_size != total_state_size)
            {
                Debug.WriteLine("State size discrepancy");
            }

            M.Update(Pxx, 0, 0);

            uint x_position = motion_model.STATE_SIZE;

            uint x_feature_no = 0;
            
            Feature it;
            MatrixFixed itmat;
            uint y_feature_no, y_position;
            for (int i = 0; i < feature_list.Count; i++)
            {
                it = (Feature)feature_list[i];
                y_feature_no = 0;
                y_position = 0;

                M.Update(it.get_Pxy(), (int)y_position, (int)x_position);
                M.Update(it.get_Pxy().Transpose(), (int)x_position, (int)y_position);
                y_position += motion_model.STATE_SIZE;

                for (int j = 0; j < it.matrix_block_list.Count; j++)
                {
                    itmat = (MatrixFixed)(it.matrix_block_list[j]);

                    //if ((M.Rows > itmat.Rows + y_position) &&
                    //    (M.Columns > itmat.Columns + x_position))
                    //if (y_feature_no < x_feature_no)
                    //if (itmat.Rows == itmat.Columns)
                    {
                        M.Update(itmat, (int)y_position, (int)x_position);
                        M.Update(itmat.Transpose(), (int)x_position, (int)y_position);
                        y_position += (uint)itmat.Rows;
                        y_feature_no++;
                    }
                    /*
                    else
                    {
                        Debug.WriteLine("Error: construct_total_covariance y_feature_no >= x_feature_no");
                    }
                     */
                }
                M.Update(it.get_Pyy(), (int)y_position, (int)x_position);

                x_feature_no++;
                uint fstate_size = it.get_feature_measurement_model().FEATURE_STATE_SIZE;
                x_position += fstate_size;
            }

            //assert (x_position == total_state_size);
            if (x_position != total_state_size)
            {
                Debug.WriteLine("Error: construct_total_covariance 2");
            }
        }


        public void fill_states(Vector V)
        {
            //assert (V.Size() == total_state_size);

            uint y_position = 0;

            xv.Update(V.Extract((int)motion_model.STATE_SIZE, (int)y_position));
            y_position += motion_model.STATE_SIZE;
  
            uint y_feature_no = 0;

            Feature it;
            for (int i=0;i<feature_list.Count;i++)
            {
                if (y_position < V.Size())
                {
                    it = (Feature)feature_list[i];
                    it.set_y(V.Extract((int)it.get_feature_measurement_model().FEATURE_STATE_SIZE, (int)y_position));
                    y_feature_no++;
                    y_position += it.get_feature_measurement_model().FEATURE_STATE_SIZE;
                }
            }

            //assert (y_feature_no <= feature_list.size() && y_position <= total_state_size);
        }


        public void fill_covariances(MatrixFixed M)
        {
            //assert (M.Rows() == total_state_size && M.Cols() == total_state_size);

            Pxx = M.Extract((int)motion_model.STATE_SIZE, (int)motion_model.STATE_SIZE, 0, 0);

            uint x_position = motion_model.STATE_SIZE;

            uint x_feature_no = 0;
            uint y_feature_no, y_position;
            Feature it;
            MatrixFixed itmat;
            for (int i=0;i<feature_list.Count;i++)
            {
                if (x_position < M.Columns)
                {
                    it = (Feature)feature_list[i];
                
                    y_feature_no = 0;
                    y_position = 0;

                    it.set_Pxy(M.Extract((int)motion_model.STATE_SIZE, (int)it.get_feature_measurement_model().FEATURE_STATE_SIZE, (int)y_position, (int)x_position));
                    y_position += motion_model.STATE_SIZE;

                    for (int j=0; j < it.matrix_block_list.Count; j++)
                    {
                        itmat = (MatrixFixed)(it.matrix_block_list[j]);

                        //assert (y_feature_no < x_feature_no);

                        itmat.Update(M.Extract(itmat.Rows, itmat.Columns, (int)y_position, (int)x_position));
                        y_position += (uint)itmat.Rows;
                        y_feature_no++;
                    }
                    it.set_Pyy(M.Extract((int)it.get_feature_measurement_model().FEATURE_STATE_SIZE,
                                         (int)it.get_feature_measurement_model().FEATURE_STATE_SIZE,
                                         (int)y_position, (int)x_position));

                    x_feature_no++;
                    x_position += it.get_feature_measurement_model().FEATURE_STATE_SIZE;                
                }
            }
            //assert (x_position <= total_state_size);
        }




        //******************************Access functions*******************************

        /// <summary>
        /// Fills a vector with the state \vct{y}_i of the currently-marked feature.
        /// </summary>
        /// <param name="y_to_fill">The vector to fill with the state.</param>
        /// <returns>false if no feature is currently marked (or the marked label is unknown); true otherwise.</returns>
        public bool get_marked_feature_state(Vector y_to_fill)
        {
            if (marked_feature_label == -1)
            {
                Debug.WriteLine("No feature marked.");
                return false;
            }

            Feature it = null;
            bool found = false;
            for (int i=0;i<feature_list.Count;i++)
            {
                it = (Feature)feature_list[i];

                if ((int)it.get_label() == marked_feature_label) { found = true; break; }
            }

            if (!found)
            {
                Debug.WriteLine("Error: marked feature not found in list.");
                return false;
            }
            else
            {
                y_to_fill.Update(it.get_y());
                return true;
            }
        }

        /// <summary>
        /// Fills a vector with the measurement state vct{h}_i of the currently-marked feature
        /// </summary>
        /// <param name="h_to_fill">The vector to fill with the state.</param>
        /// <returns>false if no feature is currently marked (or the marked label is unknown); true otherwise.</returns>
        public bool get_marked_feature_measurement_state(Vector h_to_fill)
        {
            if (marked_feature_label == -1)
            {
                Debug.WriteLine("No feature marked.");
                return false;
            }

            Feature_Measurement_Model fmm = get_feature_measurement_model_for_marked_feature();
            if (!fmm.fully_initialised())
            {
                Debug.WriteLine("Selected feature is not a fully-initialised feature! ");
                return false;
            }
  
            Fully_Initialised_Feature_Measurement_Model fmm2 = (Fully_Initialised_Feature_Measurement_Model)fmm;
            if (fmm2 == null)
            {
                Debug.WriteLine("Selected feature claims to be a fully-initialised feature, " +
                                "but it cannot be converted to one! " +
                                "Instead, it is of type "); // + fmm.name());
                return false;
            }
            // We know that with this fmm, features are 3-vectors (world
            // position), and feature measurements are 2-vectors
            //VectorFixed<3> yi;
            Vector yi = new Vector(3);
            get_marked_feature_state(yi);

            // Where is the camera?
            fmm.get_motion_model().func_xp(get_xv());
            Vector xp = fmm.get_motion_model().get_xpRES();
            // So what is the image location?
            fmm2.func_hi_and_dhi_by_dxp_and_dhi_by_dyi(yi, xp);
            h_to_fill = fmm2.get_hiRES();

            return true;
        }

        /// <summary>
        /// Returns the feature measurement model of the currently-marked feature.
        /// </summary>
        /// <returns>The feature measurement model, or NULL if no feature is currently marked (or the marked label is unknown).</returns>
        public Feature_Measurement_Model get_feature_measurement_model_for_marked_feature()
        {
            if (marked_feature_label == -1)
            {
                Debug.WriteLine("No feature marked.");
                return null;
            }

            Feature it = null;
            bool found = false;
            for (int i=0;i<feature_list.Count;i++)
            {
                it = (Feature)feature_list[i];

                if ((int)(it.get_label()) == marked_feature_label)
                {
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                Debug.WriteLine("Error: marked feature not found in list.");
                return null;
            }
            else
                return (it.get_feature_measurement_model());
        }

    }


}

