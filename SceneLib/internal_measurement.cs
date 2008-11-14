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

namespace SceneLibrary
{
    /// <summary>
    /// A class which holds the information Scene needs to handle an internal
    /// measurement. We will have an array of these in Scene for different
    /// internal measurement capabilities.
    /// </summary>
    public class Internal_Measurement
    {
        //friend class Scene_Single;
        //friend class Kalman;

        public Internal_Measurement(Internal_Measurement_Model i_m_m)
        {
            internal_measurement_model = i_m_m;
            hv = new Vector(internal_measurement_model.MEASUREMENT_SIZE);
            zv = new Vector(internal_measurement_model.MEASUREMENT_SIZE);
            nuv = new Vector(internal_measurement_model.MEASUREMENT_SIZE);

            dhv_by_dxv = new MatrixFixed(internal_measurement_model.MEASUREMENT_SIZE, internal_measurement_model.get_motion_model().STATE_SIZE);
            Rv = new MatrixFixed(internal_measurement_model.MEASUREMENT_SIZE, internal_measurement_model.MEASUREMENT_SIZE);
            Sv = new MatrixFixed(internal_measurement_model.MEASUREMENT_SIZE, internal_measurement_model.MEASUREMENT_SIZE);

            // if (DEBUGDUMP) cout << "Adding Internal Measurement type " 
		    // << internal_measurement_model.internal_type << endl;
        }

        public void predict_internal_measurement(Vector xv, MatrixFixed Pxx)
        {
            internal_measurement_model.func_hv_and_dhv_by_dxv(xv);

            hv.Update(internal_measurement_model.get_hvRES());
            dhv_by_dxv.Update(internal_measurement_model.get_dhv_by_dxvRES());

            internal_measurement_model.func_Rv(hv);
            Rv.Update(internal_measurement_model.get_RvRES());

            internal_measurement_model.func_Sv(Pxx, dhv_by_dxv, Rv);
            Sv.Update(internal_measurement_model.get_SvRES());

            //if (DEBUGDUMP) cout << "Internal measurement prediction: hv " << endl 
		    //<< hv << endl;
        }


        public void successful_internal_measurement(Vector zv_in)
        {
            //assert(zv.Size() == internal_measurement_model.MEASUREMENT_SIZE);
            zv.Update(zv_in);

            //if (DEBUGDUMP) cout << "Internal measurement made      : zv " << endl << zv << endl;

            successful_internal_measurement_flag = true;

            internal_measurement_model.func_nuv(hv, zv);
            nuv.Update(internal_measurement_model.get_nuvRES());
        }

        public void failed_internal_measurement()
        {
            successful_internal_measurement_flag = false;
        }


        public void construct_total_internal_measurement_stuff(
                       Vector nu_tot, MatrixFixed dh_by_dx_tot,
                       MatrixFixed R_tot, uint total_state_size)
        {
            uint size = internal_measurement_model.MEASUREMENT_SIZE;

            //assert (nu_tot.Size() == size && 
            //dh_by_dx_tot.Rows() == size && 
            //dh_by_dx_tot.Cols() == total_state_size &&
            //R_tot.Rows() == size && R_tot.Cols() == size);

            //assert(successful_internal_measurement_flag);

            nu_tot.Fill(0.0f);
            dh_by_dx_tot.Fill(0.0f);
            R_tot.Fill(0.0f);

            nu_tot.Update(nuv, 0);

            dh_by_dx_tot.Update(dhv_by_dxv, 0, 0);

            R_tot.Update(Rv, 0, 0);

        }


        public Internal_Measurement_Model get_internal_measurement_model() { return internal_measurement_model; }

        protected Internal_Measurement_Model internal_measurement_model;
        public Vector hv, zv, nuv;
        public MatrixFixed dhv_by_dxv, Rv, Sv;
        public bool successful_internal_measurement_flag;
    }

  }
