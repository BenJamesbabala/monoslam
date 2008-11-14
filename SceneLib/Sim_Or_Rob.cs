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

namespace SceneLibrary
{
    /// <summary>
    /// Base Class for Simulation or Robot mode classes. These classes provide the
    /// functionality of initialising and measuring features, making internal
    /// measurements, and controlling a vehicle.
    /// </summary>
    public class Sim_Or_Rob
    {

        public Sim_Or_Rob(String s_o_r_type)
        {
            simulation_or_robot_type = s_o_r_type;
        }

        // Default NULL version
        public virtual bool make_internal_measurement(Internal_Measurement_Model m,
                                              Vector v,
                                              Vector v2,
                                              MatrixFixed mt)
        {
            Debug.WriteLine("WARNING: make_internal_measurement not implemented.");

            return false;
        }

        /// <summary>
        /// Make a measurement of a feature.
        /// </summary>
        /// <param name="id">The identifier for this feature</param>
        /// <param name="z">The measurement of the state, to be filled in by this function</param>
        /// <param name="h">The expected measurement</param>
        /// <param name="S">The measurement covariance.</param>
        /// <returns></returns>
        public virtual bool measure_feature(classimage_mono id, ref Vector z, Vector vz, Vector h, MatrixFixed S, Random rnd) { return (false); }
        public virtual bool set_control(Vector u, float delta_t) { return (false); }
        public virtual bool continue_control(Vector u, float delta_t) { return (false); }
        public virtual float wait_for_end_of_motion(Vector u) { return (0); }
        public virtual bool stop_vehicle() { return (false); }
        public virtual classimage_mono initialise_known_feature(Feature_Measurement_Model f_m_m,
                                                    Vector yi, uint known_feature_label, String path) { return (null); }
        public virtual classimage_mono initialise_known_feature(Feature_Measurement_Model f_m_m,
                                                           Vector yi, Settings.Section section, String path) { return (null); }

        /// <summary>
        /// Make the first measurement of a feature.
        /// </summary>
        /// <param name="z">The location of the feature to measure</param>
        /// <param name="id">The Identifier to fill with the feature measurements</param>
        /// <param name="f_m_m">The feature measurement model to use for this partially-initialised feature</param>
        /// <returns></returns>
        public virtual bool make_initial_measurement_of_feature(Vector z,
                                                         ref classimage_mono id,
                                                         Partially_Initialised_Feature_Measurement_Model f_m_m,
                                                         Vector patch_colour) { return (false); }

        public String simulation_or_robot_type;
    }

}
