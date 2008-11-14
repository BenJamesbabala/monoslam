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
using System.Collections;
using System.Collections.Generic;
using System.Text;
using sentience;

namespace SceneLibrary
{
    /// <summary>
    /// A class to manage information about a feature. This stores the feature's
    /// Identifier, its state and covariances and its current measurement and
    /// prediction. This class is used whether the feature is fully- or
    /// partially-initialised; if it is partially-initialised, further state
    /// information (e.g. particles representing the free parameters) are stored in a
    /// separate FeatureInitInfo class, which points to this one. This class also knows
    /// how to convert from a partially- to a fully-initialised feature.
    /// </summary>
    public class Feature
    {
        // traingles within meshes
        public ArrayList triangles = new ArrayList();

        //*****************************Access functions******************************
        /// Returns the feature measurement model for this feature.
        public Feature_Measurement_Model get_feature_measurement_model() { return feature_measurement_model; }
        // Returns the fully-initialised feature measurement model for this
        // feature. This will be NULL if the feature is currently only partially
        // initialised.
        public Fully_Initialised_Feature_Measurement_Model get_fully_initialised_feature_measurement_model() { return fully_initialised_feature_measurement_model; }
        // Returns the partially-initialised feature measurement model for this
        // feature. This will be NULL if the feature is fully-initialised.
        public Partially_Initialised_Feature_Measurement_Model get_partially_initialised_feature_measurement_model() { return partially_initialised_feature_measurement_model; }

        // Returns the feature state, \vct{y} (for example its 3D location).
        public Vector get_y() { return y; }
        // Set the feature state, \vct{y} (for example its 3D location). 
        public void set_y(Vector new_y) { y.Update(new_y); }
        // Returns the feature state covariance, \mat{P}_{yy}. 
        public MatrixFixed get_Pyy() { return Pyy; }
        // Set the feature state covariance, \mat{P}_{yy}. 
        public void set_Pyy(MatrixFixed new_Pyy) { Pyy.Update(new_Pyy); }
        // Returns the covariance, \mat{P}_{xy}, between the feature state
        // and the robot state. 
        public MatrixFixed get_Pxy() { return Pxy; }
        // Set the covariance, \mat{P}_{xy},  between the feature state and
        // the robot state. 
        public void set_Pxy(MatrixFixed new_Pxy) { Pxy.Update(new_Pxy); }
        // Returns the robot position state, \vct{x}_p, from which the
        // feature was initially observed. This is used to determine whether it is still
        // acceptable to try to match it from the current robot position (for example,
        // in a visual-SLAM system, the viewing angle may have changed too much). 
        public Vector get_xp_orig() { return xp_orig; }
        // Set the robot position state, \vct{x}_p, from which the
        // feature was initially observed.
        public void set_xp_orig(Vector new_xp_orig) { xp_orig.Update(new_xp_orig); }

        // Get the covariance matrices between this feature and all the preceeding
        // features in the system. Each matrix in this list represents the matrix
        // \mat{P}_{y_iy_j} between this feature and another feature. The columns of
        // the total state covariance matrix corresponding to the current feature
        // \vct{y}_n are given by
        // \emat{\mat{P}_{xy_n} \\ \mat{P}_{y_1y_n} \\ \mat{P}_{y_2y_n} \\ \vdots \\
        // \mat{P}_{y_ny_n} }
        // with \mat{P}_{xy_n} given by get_Pxy(), \mat{P}_{y_ny_n} given
        // by get_Pyy() and the matrices \mat{P}_{y_iy_n} given
        // by get_matrix_block_list(). The corresponding rows of the total state
        // covariance matrix are given by the transpose of these.
        public ArrayList get_matrix_block_list() { return matrix_block_list; }

        // Returns the feature measurement state, \vct{h} (for example its
        // image  location). 
        public Vector get_h() { return h; }
        // Set the feature measurement state, \vct{h} (for example its
        // image  location). 
        public void set_h(Vector new_h) { h.Update(new_h); }
        // Returns the most recent measurement of this feature, \vct{z} (for
        // example its detected image location). 
        public Vector get_z() { return z; }
        // update the 2D velocity of the feature
        public void update_velocity(bool successfully_measured) 
        {
            if (successfully_measured)
            {
                // only do this is a previous position is available
                if (!((prev_z[0] == 0) && (prev_z[1] == 0)))
                {
                    //update acceleration
                    float ax = (z[0] - prev_z[0]) - vz[0];
                    float ay = (z[1] - prev_z[1]) - vz[1];
                    accn_z[0] = ((accn_z[0] * 8) + (ax * 2))/10.0f;
                    accn_z[1] = ((accn_z[1] * 8) + (ay * 2))/10.0f;
                    // update the feature velocity
                    vz[0] = vz[0] + accn_z[0];
                    vz[1] = vz[1] + accn_z[1];
                }
                prev_z.Update(z);
            }
            else
            {
                // The feature was not measured, so reset
                prev_z[0] = 0;
                prev_z[1] = 0;
                vz[0] = 0;
                vz[1] = 0;
                //accn_z[0] = 0;
                //accn_z[1] = 0;
            }
        }
        // A non-const version of the above function. 
        public Vector get_z_noconst() { return z; }
        // Return the 2D velocity of the feature
        public Vector get_vz() { return vz; }
        // Set the most recent measurement of this feature, \vct{z} (for
        // example its detected image location). 
        public void set_z(Vector new_z) { z.Update(new_z); }
        // Returns the innovation , \vct{\nu}, the difference between
        // the expected state \vct{h} and measured state \vct{z}. 
        public Vector get_nu() { return nu; }
        public void set_nu(Vector new_nu) { nu.Update(new_nu); }
        // Returns the Jacobian \partfrac{\vct{h}}{\vct{x}_v}
        // between the measurement state and the robot state. 
        public MatrixFixed get_dh_by_dxv() { return dh_by_dxv; }
        // Set the Jacobian \partfrac{\vct{h}}{\vct{x}_v}
        // between the measurement state and the robot state. */
        public void set_dh_by_dxv(MatrixFixed new_dh_by_dxv) { dh_by_dxv.Update(new_dh_by_dxv); }
        // Returns the Jacobian \partfracv{h}{y}
        // between the feature measurement state and the feature state.
        public MatrixFixed get_dh_by_dy() { return dh_by_dy; }
        // Set the Jacobian \partfracv{h}{y}
        // between the feature measurement state and the feature state. 
        public void set_dh_by_dy(MatrixFixed new_dh_by_dy) { dh_by_dy.Update(new_dh_by_dy); }
        // Returns the covariance of the measurement \mat{R}
        // (for example, its uncertainty due to image resolution). 
        public MatrixFixed get_R() { return R; }
        // Set the covariance of the measurement \mat{R}
        // (for example, its uncertainty due to image resolution). 
        public void set_R(MatrixFixed new_R) { R.Update(new_R); }
        // Returns the innovation covariance \mat{S} 
        // (for example, the overall uncertainty in image location). 
        public MatrixFixed get_S() { return S; }
        // Set the innovation covariance \mat{S}
        // (for example, the overall uncertainty in image location). 
        public void set_S(MatrixFixed new_S) { S.Update(new_S); }

        // Returns true if this feature is currently selected. 
        public bool get_selected_flag() { return selected_flag; }
        // Flag this feature as currently selected. 
        public void set_selected_flag(bool newstate) { selected_flag = newstate; }
        public uint get_position_in_list() { return position_in_list; }
        public uint get_position_in_total_state_vector() { return position_in_total_state_vector; }
        public void set_position_in_total_state_vector(uint newval) { position_in_total_state_vector = newval; }
        public void increment_position_in_total_state_vector(int increment) { position_in_total_state_vector += (uint)increment; }
        public classimage_mono get_identifier() { return identifier; }
        public uint get_label() { return label; }
        public bool get_scheduled_for_termination_flag() { return scheduled_for_termination_flag; }
        public void set_scheduled_for_termination_flag(bool newstate) { scheduled_for_termination_flag = newstate; }

        public bool get_successful_measurement_flag() { return successful_measurement_flag; }
        public void set_successful_measurement_flag(bool newstate) { successful_measurement_flag = newstate; }
        public uint get_attempted_measurements_of_feature() { return attempted_measurements_of_feature; }
        public void increment_attempted_measurements_of_feature(int inc) { attempted_measurements_of_feature += (uint)inc; }
        public uint get_successful_measurements_of_feature() { return successful_measurements_of_feature; }
        public void increment_successful_measurements_of_feature(int inc) { successful_measurements_of_feature += (uint)inc; }

        // Perform a visibility test for this feature. This makes use of
        // Feature_Measurement_Model::visibility_test(), and returns zero if visible,
        // and non-zero otherwise, like that function.
        public uint visibility_test() { return get_feature_measurement_model().visibility_test(get_feature_measurement_model().get_motion_model().get_xpRES(), get_y(), get_xp_orig(), get_h()); }

        //******************************Data members*******************************

        /// Pointer to the current feature measurement model
        protected Feature_Measurement_Model feature_measurement_model;

        // The partially-initialised feature measurement model for this feature. Set
        // to <code>NULL</code> if not relevant. 
        protected Partially_Initialised_Feature_Measurement_Model partially_initialised_feature_measurement_model;

        // The fully-initialised feature measurement model for this feature. Set
        // to <code>NULL</code> if not relevant. 
        protected Fully_Initialised_Feature_Measurement_Model fully_initialised_feature_measurement_model;

        // Important geometrical data
        // The estimate of the feature state y_i. (e.g. the feature's 3D
        // position) 
        protected Vector y;
        // The covariance of this feature P_{y_iy_i} 
        protected MatrixFixed Pyy;
        // The covariance between the robot state and this feature's state
        // P_{x_vy_i}
        protected MatrixFixed Pxy;
        // The robot position x_p state from which we initialised this
        // feature 
        protected Vector xp_orig;

        // The covariances between this feature and others. 
        public ArrayList matrix_block_list = new ArrayList();

        // Places to store predicted (h) and actual (z) measurements and
        // Jacobians
        // The predicted feature measurement h_i state. 
        protected Vector h;
        // The actual feature measurement state z_i. 
        protected Vector z;
        // The previous actual feature measurement state prev_z_i. 
        protected Vector prev_z;
        // The velocity of the feature within the image
        protected Vector vz;
        protected Vector accn_z;
        // The feature innovation \nu_i = z_i - h_i. 
        protected Vector nu; // Innovation
        // The Jacobian between the predicted measurement and the vehicle state
        // \frac{\partial h_i}{\partial x_v}. 
        protected MatrixFixed dh_by_dxv;
        // The Jacobian between the feature measurement and its state
        // \frac{\partial h_i}{\partial y_i}. 
        protected MatrixFixed dh_by_dy;
        // The innovation covariance R_i 
        protected MatrixFixed R;
        // The feature measurement covariance S_i 
        protected MatrixFixed S;

        // Important bookkeeping data
        // Is this feature currently selected? 
        public bool selected_flag;
        // The current position of this feature in the list of features. 
        protected uint position_in_list;
        // With general feature state sizes, need this to identify the starting
        // position of this feature in total state vector. 
        protected uint position_in_total_state_vector;

        // The identifier for this feature. Identifier is a pointer to something in
        // the real world which identifies this feature uniquely. 
        protected classimage_mono identifier;
        // The label for this feature within scene class. 
        protected uint label;

        // Set if we want shortly to delete this feature but can't do it
        // right now 
        protected bool scheduled_for_termination_flag;

        // Keep track of the attempted and successful measurements made of a 
        // feature over time 
        // Was the last measurement of this feature successful? 
        protected bool successful_measurement_flag;
        // The number of times that a measurement of this feature has been
        // attempted. Used together with successful_measurements_of_feature to
        // determine whether the feature is a bad one that should be deleted. 
        protected uint attempted_measurements_of_feature = 0;
        // The number of times that this feature has been successfully measured. Used
        // together with attempted_measurements_of_feature to determine whether the
        // feature is a bad one that should be deleted. 
        protected uint successful_measurements_of_feature = 0;

        // Set to a <code>label >= 0</code> if this is a known feature; otherwise
        // <code>-1</code> 
        protected int known_feature_label;

        // local colour of the feature
        public Vector colour;


        /// <summary>
        /// Constructor for partially-initialised features.
        /// </summary>
        /// <param name="id">reference to the feature</param>
        /// <param name="lab"></param>
        /// <param name="list_pos"></param>
        /// <param name="scene"></param>
        /// <param name="h"></param>
        /// <param name="p_i_f_m_m"></param>
        public Feature(classimage_mono id, uint lab, uint list_pos,
                       Scene_Single scene, Vector h,
                       Partially_Initialised_Feature_Measurement_Model p_i_f_m_m,
                       Vector feature_colour)
        {
            feature_measurement_model = p_i_f_m_m;
            partially_initialised_feature_measurement_model = p_i_f_m_m;
            fully_initialised_feature_measurement_model = null;

            // Stuff below substituted from Feature_common
            //   Feature_common(id, lab, list_pos, scene, h);

            feature_constructor_bookeeping();

            identifier = id;
            label = lab;
            position_in_list = list_pos;   // Position of new feature in list
            position_in_total_state_vector = 0; // This should be set properly
            colour = feature_colour;
            //when feature is added 

            // Save the vehicle position where this feature was acquired 
            scene.get_motion_model().func_xp(scene.get_xv());
            //xp_orig = scene.get_motion_model().get_xpRES();
            xp_orig = new Vector(scene.get_motion_model().get_xpRES());

            // Call model functions to calculate feature state, measurement noise
            // and associated Jacobians. Results are stored in RES matrices 

            // First calculate "position state" and Jacobian
            scene.get_motion_model().func_xp(scene.get_xv());
            scene.get_motion_model().func_dxp_by_dxv(scene.get_xv());

            // Now ask the model to initialise the state vector and calculate Jacobians
            // so that I can go and calculate the covariance matrices
            partially_initialised_feature_measurement_model.func_ypi_and_dypi_by_dxp_and_dypi_by_dhi_and_Ri(h, scene.get_motion_model().get_xpRES());

            // State y
            //y = partially_initialised_feature_measurement_model.get_ypiRES();
            y = new Vector(partially_initialised_feature_measurement_model.get_ypiRES());

            // Temp_FS1 will store dypi_by_dxv
            MatrixFixed Temp_FS1 =
                     partially_initialised_feature_measurement_model.get_dypi_by_dxpRES() *
                     scene.get_motion_model().get_dxp_by_dxvRES();

            // Pxy  
            Pxy = scene.get_Pxx() * Temp_FS1.Transpose();

            // Pyy
            Pyy = Temp_FS1 * scene.get_Pxx() * Temp_FS1.Transpose()
                  + partially_initialised_feature_measurement_model.get_dypi_by_dhiRES()
                  * partially_initialised_feature_measurement_model.get_RiRES()
                  * partially_initialised_feature_measurement_model.get_dypi_by_dhiRES().Transpose();

            // Covariances of this feature with others
            int j = 0;
            foreach (Feature it in scene.get_feature_list_noconst())
            {
                if (j < position_in_list)
                {
                    // new Pypiyj = dypi_by_dxv . Pxyj
                    // Size of this is FEATURE_STATE_SIZE(new) by FEATURE_STATE_SIZE(old)
                    MatrixFixed m = it.get_Pxy();
                    MatrixFixed newPyjypi_to_store = (Temp_FS1 * m).Transpose();

                    //add to the list
                    matrix_block_list.Add(newPyjypi_to_store);
                }
                j++;
            }

            known_feature_label = -1;
        }


        /// <summary>
        /// Constructor for known features. The different number of 
        /// arguments differentiates it from the constructor for partially-initialised
        /// features
        /// </summary>
        /// <param name="id">reference to the feature identifier</param>
        /// <param name="?"></param>
        public Feature(classimage_mono id, uint lab, uint list_pos,
                       Scene_Single scene, Vector y_known,
                       Vector xp_o,
                       Feature_Measurement_Model f_m_m, uint k_f_l)
        {
            feature_measurement_model = f_m_m;
            feature_constructor_bookeeping();

            identifier = id;
            label = lab;
            position_in_list = list_pos;   // Position of new feature in list

            // Save the vehicle position where this feature was acquired 
            xp_orig = new Vector(xp_o);

            // Straighforward initialisation of state and covariances
            y = y_known;
            Pxy = new MatrixFixed(scene.get_motion_model().STATE_SIZE, feature_measurement_model.FEATURE_STATE_SIZE);
            Pxy.Fill(0.0f);
            Pyy = new MatrixFixed(feature_measurement_model.FEATURE_STATE_SIZE, feature_measurement_model.FEATURE_STATE_SIZE);
            Pyy.Fill(0.0f);

            int i = 0;
            MatrixFixed newPyjyi_to_store;
            foreach (Feature it in scene.get_feature_list_noconst())
            {
                if (i < position_in_list)
                {
                    newPyjyi_to_store = new MatrixFixed(
                        it.get_feature_measurement_model().FEATURE_STATE_SIZE,
                        feature_measurement_model.FEATURE_STATE_SIZE);

                    //add to the list
                    matrix_block_list.Add(newPyjyi_to_store);
                }

                i++;
            }

            known_feature_label = (int)k_f_l;

            if (feature_measurement_model.fully_initialised_flag)
            {
                partially_initialised_feature_measurement_model = null;
                fully_initialised_feature_measurement_model =
                    (Fully_Initialised_Feature_Measurement_Model)feature_measurement_model;
            }
            else
            {
                fully_initialised_feature_measurement_model = null;
                partially_initialised_feature_measurement_model =
                    (Partially_Initialised_Feature_Measurement_Model)feature_measurement_model;
            }
        }

        /// <summary>
        /// Function which unites common stuff for constructors below
        /// Constructor stuff which is common to more than one constructor
        /// </summary>
        protected void feature_constructor_bookeeping()
        {
            selected_flag = false;        // Feature is unselected when first detected
            scheduled_for_termination_flag = false;
            attempted_measurements_of_feature = 0;
            successful_measurements_of_feature = 0;

            // Allocate matrices for storing predicted and actual measurements
            h = new Vector(feature_measurement_model.MEASUREMENT_SIZE);
            z = new Vector(feature_measurement_model.MEASUREMENT_SIZE);
            prev_z = new Vector(feature_measurement_model.MEASUREMENT_SIZE);
            nu = new Vector(feature_measurement_model.MEASUREMENT_SIZE);

            vz = new Vector(2);
            accn_z = new Vector(2);

            dh_by_dxv = new MatrixFixed(feature_measurement_model.MEASUREMENT_SIZE, feature_measurement_model.get_motion_model().STATE_SIZE);
            dh_by_dy = new MatrixFixed(feature_measurement_model.MEASUREMENT_SIZE, feature_measurement_model.FEATURE_STATE_SIZE);
            R = new MatrixFixed(feature_measurement_model.MEASUREMENT_SIZE, feature_measurement_model.MEASUREMENT_SIZE);
            S = new MatrixFixed(feature_measurement_model.MEASUREMENT_SIZE, feature_measurement_model.MEASUREMENT_SIZE);
        }


        //****************************Feature management******************************

        // This function is called when a feature earlier in the feature list
        // is deleted
        public void feature_is_removed(uint list_pos)
        {
            matrix_block_list.RemoveAt((int)list_pos);
            position_in_list--;
        }

        /// <summary>
        /// update the position of the feature, but don't add it to the state
        /// </summary>
        /// <param name="lambda"></param>
        public void update_feature_position(Vector lambda)
        {
            partially_initialised_feature_measurement_model.func_yfi_and_dyfi_by_dypi_and_dyfi_by_dlambda(y, lambda);

            MatrixFixed dyfi_by_dypiT = partially_initialised_feature_measurement_model.get_dyfi_by_dypiRES().Transpose();
            MatrixFixed dyfi_by_dlambdaT = partially_initialised_feature_measurement_model.get_dyfi_by_dlambdaRES().Transpose();

            // Replace y            
            y = new Vector(partially_initialised_feature_measurement_model.get_yfiRES());
        }



        /// <summary>
        /// Convert a partially-initialised feature to a fully-initialised feature,
        /// given information about the free parameters \vct{\lambda}.
        /// The new state \vct{y}_{fi} is given by calling
        /// Partially_Initialised_Feature_Measurement_Model::func_yfi_and_dyfi_by_dypi_and_dyfi_by_dlambda().
        /// where the various Jacobians are returned by calls to
        /// Partially_Initialised_Feature_Measurement_Model, and the covariance matrices
        /// \mat{P}_{kl} are already known and stored in the class, except for
        /// \mat{P}_{\vct{\lambda}}, which is passed to the function.
        /// </summary>
        /// <param name="lambda">The mean value for \vct{\lambda}</param>
        /// <param name="Plambda">The covariance for \vct{\lambda}</param>
        /// <param name="scene">The SLAM map</param>
        public void convert_from_partially_to_fully_initialised(
                Vector lambda, MatrixFixed Plambda, Scene_Single scene)
        {
            
            // We'll do all the work here in feature.cc though probably this only
            // works with scene_single...

            // We calculate new state yfi(ypi, lambda)
            // New feature covariance 
            // Pyfiyfi = dyfi_by_dypi Pypiypi dyfi_by_dypiT + 
            //           dyfi_by_dlambda Plambda dyfi_by_dlambdaT
            // And we change cross covariances as follows:
            // Pxyfi = Pxypi dyfi_by_dypiT
            // Pyjyfi = Pyjypi dyfi_by_dypiT   for j < i (since we only store top-right
            // Pyfiyj = dyfi_by_dypi Pypiyj    for j > i  part of covariance matrix)

            partially_initialised_feature_measurement_model.func_yfi_and_dyfi_by_dypi_and_dyfi_by_dlambda(y, lambda);

            MatrixFixed dyfi_by_dypiT = partially_initialised_feature_measurement_model.get_dyfi_by_dypiRES().Transpose();
            MatrixFixed dyfi_by_dlambdaT = partially_initialised_feature_measurement_model.get_dyfi_by_dlambdaRES().Transpose();

            // Replace y            
            y = new Vector(partially_initialised_feature_measurement_model.get_yfiRES());

            // Replace Pxy
            Pxy = Pxy * dyfi_by_dypiT;

            // Replace Pyy
            MatrixFixed Pypiypi_1 = partially_initialised_feature_measurement_model.get_dyfi_by_dypiRES() *
                        Pyy * dyfi_by_dypiT;
            MatrixFixed Pypiypi_2 = partially_initialised_feature_measurement_model.get_dyfi_by_dlambdaRES() *
                        Plambda * dyfi_by_dlambdaT;
            Pyy = Pypiypi_1 + Pypiypi_2;

            // Pyjyi elements for j < i (covariance with features before i in list)
            uint i = position_in_list;

            MatrixFixed m_it;
            int j;
            for (j = 0; j < position_in_list; j++)
            {
                m_it = (MatrixFixed)matrix_block_list[j];
                matrix_block_list[j] = m_it * dyfi_by_dypiT;
            }


            Feature it;
            int idx = scene.feature_list.IndexOf(this);
            for (j = idx + 1; j < scene.feature_list.Count; j++)
            {
                it = (Feature)(scene.feature_list[j]);
                it.matrix_block_list[(int)i] = partially_initialised_feature_measurement_model.get_dyfi_by_dypiRES() * (MatrixFixed)it.matrix_block_list[(int)i];
                it.increment_position_in_total_state_vector(-(int)feature_measurement_model.FEATURE_STATE_SIZE);
            }


            // Change the total state size in scene, here with a negative increment
            uint size1 = partially_initialised_feature_measurement_model.more_initialised_feature_measurement_model.FEATURE_STATE_SIZE;
            uint size2 = partially_initialised_feature_measurement_model.FEATURE_STATE_SIZE;
            scene.increment_total_state_size((int)size1 - (int)size2);

            // Change fmm for this model to fully-initialised one
            feature_measurement_model =
                partially_initialised_feature_measurement_model.more_initialised_feature_measurement_model;

            partially_initialised_feature_measurement_model = null;
            fully_initialised_feature_measurement_model =
                (Fully_Initialised_Feature_Measurement_Model)feature_measurement_model;


            //assert(fully_initialised_feature_measurement_model != NULL);

            // Need to reallocate any other matrices
            // Assume that measurement size doesn't change 
            dh_by_dy.Resize(feature_measurement_model.MEASUREMENT_SIZE, feature_measurement_model.FEATURE_STATE_SIZE);
                         
             
        }

    }


}
