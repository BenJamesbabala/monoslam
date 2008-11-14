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

namespace SceneLibrary
{
    /// <summary>
    /// Implements an Extended Kalman Filter for the Scene_Single class. This is a
    /// friend class of Scene_Single, and extracts the state vectors and covariances
    /// from that class.
    /// </summary>
    public class Kalman
    {
        #region "Predict New Vehicle Position"

        /// <summary>
        /// Simple overall prediction 
        /// </summary>
        /// <param name="scene"></param>
        /// <param name="u"></param>
        /// <param name="delta_t"></param>
        public void predict_filter_slow (Scene_Single scene, Vector u, float delta_t)
        {
            Debug.WriteLine("*** SLOW PREDICTION ***");

            // What we need to do for the prediction:
     
            //    Calculate f and grad_f_x
            //    Calculate Q
            //    Form x(k+1|k) and P(k+1|k)

            int size = (int)scene.get_total_state_size();

            // First form original total state and covariance
            Vector x = new Vector(size);
            MatrixFixed P = new MatrixFixed(size, size);
            scene.construct_total_state_and_covariance(ref x, ref P);

            // Make model calculations: store results in RES matrices
            Vector xv = scene.get_xv();
            //Vector xv = new Vector(scene.get_xv());
            scene.get_motion_model().func_fv_and_dfv_by_dxv(xv, u, delta_t);
            scene.get_motion_model().func_Q(scene.get_xv(), u, delta_t);

            // Find new state f
            Vector f = new Vector(size);

            // Feature elements of f are the same as x 
            f.Update(x);
            f.Update(scene.get_motion_model().get_fvRES(), 0);

            // Find new P

            // Since most elements of df_by_dx are zero... 
            MatrixFixed df_by_dx = new MatrixFixed(size, size);
            df_by_dx.Fill(0.0f);

            // Fill the rest of the elements of df_by_dx: 1 on diagonal for features
            for (int i = (int)scene.get_motion_model().STATE_SIZE; i < df_by_dx.Rows; i++)
                df_by_dx[i,i] = 1.0f;

            df_by_dx.Update(scene.get_motion_model().get_dfv_by_dxvRES(), 0, 0);

            // Calculate the process noise
            MatrixFixed Q = new MatrixFixed(size, size);
            Q.Fill(0.0f);
            Q.Update(scene.get_motion_model().get_QxRES(), 0, 0);

            P.Update(df_by_dx * P * df_by_dx.Transpose());

            P += Q;

            scene.fill_state_and_covariance(f, P);
        }



        /// <summary>
        /// Fast version of predict filter
        /// </summary>
        /// <param name="scene"></param>
        /// <param name="u">Control vector of accelerations</param>
        /// <param name="delta_t">time step in seconds</param>
        public void predict_filter_fast(Scene_Single scene, Vector u, float delta_t)
        {
            Vector xv = scene.get_xv();

            // Make model calculations: results are stored in RES matrices
            scene.get_motion_model().func_fv_and_dfv_by_dxv(xv, u, delta_t);
            scene.get_motion_model().func_Q(xv, u, delta_t);

            Vector fvRES = scene.get_motion_model().get_fvRES();
            xv.Update(fvRES);

            Motion_Model mm = scene.get_motion_model();
            MatrixFixed Pxx = scene.get_Pxx();
            MatrixFixed dfv_by_dxvRES = mm.get_dfv_by_dxvRES();
            MatrixFixed dfv_by_dxvRES_transposed = dfv_by_dxvRES.Transpose();
            MatrixFixed QxRES = mm.get_QxRES();
            
            Pxx.Update((dfv_by_dxvRES * Pxx * dfv_by_dxvRES_transposed) + QxRES);
                      
            // Change the covariances between vehicle state and feature states 
            foreach (Feature it in scene.feature_list)
            {
                MatrixFixed Pxy = it.get_Pxy();
                Pxy.Update(dfv_by_dxvRES * Pxy);
            }            
            
        }
        #endregion

        #region "Update Filter"

        /// <summary>
        /// Update the filter in a simple overall way 
        /// </summary>
        /// <param name="scene"></param>
        public void total_update_filter_slow(Scene_Single scene)
        {
            // Steps to update the total filter:     
            // 1. Form h and dh_by_dx and R(k+1) and z
            // 2. Calculate S(k+1)
            // 3. Calculate W(k+1)
            // 4. Calculate x(k+1|k+1)
            // 5. Calculate P(k+1|k+1)

            uint size = scene.get_successful_measurement_vector_size(); // Size of measurement vector
                                                  
            uint size2 = scene.get_total_state_size();                  // Size of state vector

            Vector x = new Vector(size2);
            MatrixFixed P = new MatrixFixed(size2, size2);

            scene.construct_total_state_and_covariance(ref x, ref P);

            // 1. Form nu and dh_by_dx 
            Vector nu_tot = new Vector(size);
            MatrixFixed dh_by_dx_tot = new MatrixFixed(size, size2);
            MatrixFixed R_tot = new MatrixFixed(size, size);

            scene.construct_total_measurement_stuff(nu_tot, dh_by_dx_tot, R_tot);

            // 2. Calculate S(k+1)
            MatrixFixed temp_matrix = P * dh_by_dx_tot.Transpose();  //pre-calculate to speed up subsequent stuff
            MatrixFixed S = dh_by_dx_tot * temp_matrix;
            S += R_tot;

            // 3. Calculate W(k+1) 
            Cholesky S_cholesky = new Cholesky(S);

            MatrixFixed W = temp_matrix * S_cholesky.Inverse();

            // 4. Calculate x(k+1|k+1) 
            x += W * nu_tot;

            // 5. Calculate P(k+1|k+1) 
            P -= W * S * W.Transpose();

            scene.fill_state_and_covariance(x, P);

        }

        public void update_filter_internal_measurement_slow(Scene_Single scene, uint i)
        {
            // Size of measurement vector
            uint size = ((Internal_Measurement)(scene.internal_measurement_vector[(int)i])).get_internal_measurement_model().MEASUREMENT_SIZE;

            uint size2 = scene.get_total_state_size();   // Size of state vector

            Vector x = new Vector(size2);
            MatrixFixed P = new MatrixFixed(size2, size2);

            scene.construct_total_state_and_covariance(ref x, ref P);

            // cout << "x:" << x;
            // cout << "P:" << P;

            // 1. Form nu and dh_by_dx 
            Vector nu_tot = new Vector(size);
            MatrixFixed dh_by_dx_tot = new MatrixFixed(size, size2);
            MatrixFixed R_tot = new MatrixFixed(size, size);

            scene.construct_total_internal_measurement_stuff(nu_tot, dh_by_dx_tot, R_tot, i);

            // 2. Calculate S(k+1) 
            MatrixFixed S = new MatrixFixed(size, size);

            MatrixFixed Tempss2 = new MatrixFixed(size, size2);
            MatrixFixed Temps2s = new MatrixFixed(size2, size);

            MatrixFixed dh_by_dx_totT = dh_by_dx_tot.Transpose();

            S.Update(dh_by_dx_tot * P * dh_by_dx_totT);
            S += R_tot;

            // cout << "R_tot:" << R_tot;
            //  cout << "S:" << S;
            // cout << "dh_by_dx_tot:" << dh_by_dx_tot;
            // cout << "dh_by_dx_totT:" << dh_by_dx_totT;

            // 3. Calculate W(k+1) 
            Cholesky S_cholesky = new Cholesky(S);
            MatrixFixed W = P * dh_by_dx_totT * S_cholesky.Inverse();

            // cout << "W:" << W;

            // 4. Calculate x(k+1|k+1) 
            x += W * nu_tot;

            // 5. Calculate P(k+1|k+1) 
            P -= W * S * W.Transpose();

            scene.fill_state_and_covariance(x, P);

            // cout << "x after update:" << x;
            // cout << "P after update:" << P;
        }

        #endregion

    }

}
