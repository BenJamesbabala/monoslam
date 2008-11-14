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

    public class non_overlapping_region
    {

        // Function to find a region in an image guided by current motion prediction
        // which doesn't overlap existing features
        public static bool FindNonOverlappingRegion(Scene_Single scene,
                  Vector local_u,
                  float delta_t,
                  Partially_Initialised_Feature_Measurement_Model default_feature_type_for_initialisation,
                  uint camera_width,
                  uint camera_height,
                  uint BOXSIZE,
                  ref int init_feature_search_ustart,
                  ref int init_feature_search_vstart,
                  ref int init_feature_search_ufinish,
                  ref int init_feature_search_vfinish,
                  uint FEATURE_INIT_STEPS_TO_PREDICT,
                  float FEATURE_INIT_DEPTH_HYPOTHESIS,
                  Random rnd)
        {
            
            ThreeD_Motion_Model threed_motion_model = (ThreeD_Motion_Model)scene.get_motion_model();

            Vector local_xv = new Vector(scene.get_xv());
            
            for (uint i = 0; i < FEATURE_INIT_STEPS_TO_PREDICT; i++)
            {
                scene.get_motion_model().func_fv_and_dfv_by_dxv(local_xv, local_u, delta_t);
                local_xv.Update(scene.get_motion_model().get_fvRES());
            }
            
            threed_motion_model.func_xp(local_xv);
            Vector local_xp = new Vector(threed_motion_model.get_xpRES());

            threed_motion_model.func_r(local_xp);
            Vector3D rW = threed_motion_model.get_rRES();
            threed_motion_model.func_q(local_xp);
            Quaternion qWR = threed_motion_model.get_qRES();

            // yW =  rW + RWR hR
            Vector3D hR = new Vector3D(0.0f, 0.0f, FEATURE_INIT_DEPTH_HYPOTHESIS);

            // Used Inverse + transpose because this was't compiling the normal way
            Vector3D yW = new Vector3D(rW + qWR.RotationMatrix() * hR);

            // Then project that point into the current camera position
            scene.get_motion_model().func_xp(scene.get_xv());

            Fully_Initialised_Feature_Measurement_Model fully_init_fmm =
                (Fully_Initialised_Feature_Measurement_Model)(default_feature_type_for_initialisation.more_initialised_feature_measurement_model);


            Vector yWVNL = yW.GetVNL3();
            fully_init_fmm.func_hi_and_dhi_by_dxp_and_dhi_by_dyi(yWVNL, scene.get_motion_model().get_xpRES());

            // Now, this defines roughly how much we expect a feature initialised to move
            float suggested_u = fully_init_fmm.get_hiRES()[0];
            float suggested_v = fully_init_fmm.get_hiRES()[1];
            float predicted_motion_u = camera_width / 2.0f - suggested_u;
            float predicted_motion_v = camera_height / 2.0f - suggested_v;

            // So, the limits of a "safe" region within which we can initialise
            // features so that they end up staying within the screen
            // (Making the approximation that the whole screen moves like the 
            // centre point)
            int safe_feature_search_ustart = (int)(-predicted_motion_u);
            int safe_feature_search_vstart = (int)(-predicted_motion_v);
            int safe_feature_search_ufinish = (int)(camera_width - predicted_motion_u);
            int safe_feature_search_vfinish = (int)(camera_height - predicted_motion_v);

            if (safe_feature_search_ustart < ((int)((BOXSIZE - 1) / 2) + 1))
                safe_feature_search_ustart = (int)((BOXSIZE - 1) / 2 + 1);
            if (safe_feature_search_ufinish > (int)camera_width - ((int)((BOXSIZE - 1) / 2) + 1))
                safe_feature_search_ufinish = (int)(camera_width - (BOXSIZE - 1) / 2 - 1);
            if (safe_feature_search_vstart < ((int)((BOXSIZE - 1) / 2) + 1))
                safe_feature_search_vstart = (int)((BOXSIZE - 1) / 2 + 1);
            if (safe_feature_search_vfinish > (int)camera_height - ((int)((BOXSIZE - 1) / 2) + 1))
                safe_feature_search_vfinish = (int)(camera_height - (BOXSIZE - 1) / 2 - 1);

            return FindNonOverlappingRegionNoPredict(safe_feature_search_ustart,
                       safe_feature_search_vstart,
                       safe_feature_search_ufinish,
                       safe_feature_search_vfinish,
                       scene,
                       ref init_feature_search_ustart,
                       ref init_feature_search_vstart,
                       ref init_feature_search_ufinish,
                       ref init_feature_search_vfinish, rnd);
        }

        // Function to find non-overlapping region within whole image (no prediction)
        public static bool FindNonOverlappingRegionWholeImage(Scene_Single scene,
                    uint camera_width,
                    uint camera_height,
                    uint BOXSIZE,
                    ref int init_feature_search_ustart,
                    ref int init_feature_search_vstart,
                    ref int init_feature_search_ufinish,
                    ref int init_feature_search_vfinish, 
                    Random rnd)
        {
            int safe_feature_search_ustart = (int)((BOXSIZE - 1) / 2 + 1);
            int safe_feature_search_ufinish = (int)(camera_width - (BOXSIZE - 1) / 2 - 1);
            int safe_feature_search_vstart = (int)((BOXSIZE - 1) / 2 + 1);
            int safe_feature_search_vfinish = (int)(camera_height - (BOXSIZE - 1) / 2 - 1);

            return FindNonOverlappingRegionNoPredict(safe_feature_search_ustart,
                       safe_feature_search_vstart,
                       safe_feature_search_ufinish,
                       safe_feature_search_vfinish,
                       scene,
                       ref init_feature_search_ustart,
                       ref init_feature_search_vstart,
                       ref init_feature_search_ufinish,
                       ref init_feature_search_vfinish, rnd);
        }


        /// <summary>
        /// Function to find non-overlapping region over without prediction 
        /// this is really the service function called by both the above
        /// </summary>
        /// <param name="safe_feature_search_ustart"></param>
        /// <param name="safe_feature_search_vstart"></param>
        /// <param name="safe_feature_search_ufinish"></param>
        /// <param name="safe_feature_search_vfinish"></param>
        /// <param name="scene"></param>
        /// <param name="init_feature_search_ustart"></param>
        /// <param name="init_feature_search_vstart"></param>
        /// <param name="init_feature_search_ufinish"></param>
        /// <param name="init_feature_search_vfinish"></param>
        /// <param name="rnd"></param>
        /// <returns></returns>
        public static bool FindNonOverlappingRegionNoPredict(
                       int safe_feature_search_ustart,
                       int safe_feature_search_vstart,
                       int safe_feature_search_ufinish,
                       int safe_feature_search_vfinish,
                       Scene_Single scene,
                       ref int init_feature_search_ustart,
                       ref int init_feature_search_vstart,
                       ref int init_feature_search_ufinish,
                       ref int init_feature_search_vfinish,
                       Random rnd)
        {
            int i, j;
            //if (Camera_Constants.DEBUGDUMP) cout << "FNOLRNP timer start: " << timerlocal << endl;

            int INIT_FEATURE_SEARCH_WIDTH = 80;
            int INIT_FEATURE_SEARCH_HEIGHT = 60;

            // Within this, choose a random region
            // Check that we've got some room for manouevre
            if ((safe_feature_search_ufinish - safe_feature_search_ustart > INIT_FEATURE_SEARCH_WIDTH) &&
                (safe_feature_search_vfinish - safe_feature_search_vstart > INIT_FEATURE_SEARCH_HEIGHT))
            {
                // Try a few times to get one that's not overlapping with any features
                // we know about
                int NUMBER_OF_RANDOM_INIT_FEATURE_SEARCH_REGION_TRIES = 5;
                int FEATURE_SEPARATION_MINIMUM = (int)Camera_Constants.BOXSIZE*3;

                // Build vectors of feature positions so we only have to work them out once
                ArrayList u_array = new ArrayList();
                ArrayList v_array = new ArrayList();

                Feature it;
                for (j = 0; j < scene.get_feature_list().Count; j++)
                {
                    it = (Feature)(scene.get_feature_list())[j];

                    //Vector z = it.get_z();
                    //u_array.Add(z[0]);
                    //v_array.Add(z[1]);


                    if (it.get_feature_measurement_model().fully_initialised_flag) 
                    {
                        //Vector z = it.get_z();
                        //u_array.Add(z[0]);
                        //v_array.Add(z[1]);

                        
	                    Fully_Initialised_Feature_Measurement_Model fifmm = 
	                        (Fully_Initialised_Feature_Measurement_Model)(it.get_feature_measurement_model());
	                    fifmm.func_hi_and_dhi_by_dxp_and_dhi_by_dyi(it.get_y(), scene.get_motion_model().get_xpRES());
	  	  
	                    // Check that this is not a feature behind the camera
                        if (it.get_feature_measurement_model().feature_graphics_type == "THREED_POINT") 
                        {	  
                            fifmm.func_zeroedyigraphics_and_Pzeroedyigraphics(it.get_y(),
							    scene.get_xv(), scene.get_Pxx(), it.get_Pxy(), it.get_Pyy());

	                        if (fifmm.get_zeroedyigraphicsRES()[2] > 0) 
                            {
	                            u_array.Add(fifmm.get_hiRES()[0]);
	                            v_array.Add(fifmm.get_hiRES()[1]);
                            }
                        } 
                        
                    }     
                }

                //if (Camera_Constants.DEBUGDUMP) cout << "FNOLRNP timer after functions: " << timerlocal << endl;
                
                bool feature_found = false;
                i = 0;
                while (i < NUMBER_OF_RANDOM_INIT_FEATURE_SEARCH_REGION_TRIES)
                {                    
                    int u_offset = (int)((safe_feature_search_ufinish - safe_feature_search_ustart 
                                          - INIT_FEATURE_SEARCH_WIDTH) * (rnd.Next(10000) / 10000.0f));
                    int v_offset = (int)((safe_feature_search_vfinish - safe_feature_search_vstart
                                          - INIT_FEATURE_SEARCH_HEIGHT) * (rnd.Next(10000) / 10000.0f));
      
                    init_feature_search_ustart = safe_feature_search_ustart + u_offset;
                    init_feature_search_ufinish = init_feature_search_ustart + INIT_FEATURE_SEARCH_WIDTH;
                    init_feature_search_vstart = safe_feature_search_vstart + v_offset;
                    init_feature_search_vfinish = init_feature_search_vstart + INIT_FEATURE_SEARCH_HEIGHT;

                    bool found_a_feature_in_region_flag = false;

                    // These arrays will be the same size                   
                    float uit, vit;
                    for (j = 0; j < u_array.Count; j++)
                    {
                        uit = (float)u_array[j];
                        vit = (float)v_array[j];

                        if ((uit >= init_feature_search_ustart - FEATURE_SEPARATION_MINIMUM) &&
                            (uit < init_feature_search_ufinish + FEATURE_SEPARATION_MINIMUM) &&
                            (vit >= init_feature_search_vstart - FEATURE_SEPARATION_MINIMUM) &&
                            (vit < init_feature_search_vfinish + FEATURE_SEPARATION_MINIMUM))
                        {
                            found_a_feature_in_region_flag = true;
                            feature_found = true;
	                        break;
	                    }
                    }

                    if (!found_a_feature_in_region_flag) break;
      
                    i++;
                }

                if (!feature_found) 
                {
                    return false;
                }

            }
            else 
            {
                return false;
            }

            return true;
        }

    }

}
