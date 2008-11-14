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

namespace monoSLAM
{

    /// <summary>
    /// Class to create an instance of a motion model class given its type string. 
    /// Default creator for MonoSLAM; make your own if you need other models.
    /// </summary>
    public class MonoSLAM_Motion_Model_Creator : Motion_Model_Creator
    {
        /// <summary>
        /// Returns an instance of the requested motion model (using new). Returns null if there was no match.
        /// </summary>
        /// <param name="type"></param>
        /// <returns></returns>
        public override Motion_Model create_model(String type)
        {        
            // Try creating each model that we can and see if the name is the same
            // std::cout << "Creating a " << type << " motion model" << std::endl;
            Motion_Model pModel;
	
            pModel = new ZeroOrder_ThreeD_Motion_Model();
            if (pModel.motion_model_type == type)
                return pModel;
            else
                pModel = null;

            pModel = new Impulse_ThreeD_Motion_Model();
            if (pModel.motion_model_type == type)
                return pModel;
            else
                pModel = null;

            return null;
        }

    }

    /// <summary>
    /// Class to create an instance of a feature measurement model class 
    /// given its type string. 
    /// Default creator for MonoSLAM; make your own if you need other models.
    /// </summary>
    public class MonoSLAM_Feature_Measurement_Model_Creator : Feature_Measurement_Model_Creator
    {
        /// <summary>
        /// Returns an instance of the requested feature measurement model (using new). 
        /// Returns null if there was no match.
        /// </summary>
        /// <param name="type"></param>
        /// <param name="motion_model"></param>
        /// <returns></returns>
        public override Feature_Measurement_Model create_model(String type, Motion_Model motion_model, float MAXIMUM_ANGLE_DIFFERENCE)
        {
            // Try creating each model that we can and see if the name is the same  
            // std::cout << "Creating a " << type << " feature measurement model" << std::endl;
            Feature_Measurement_Model pModel;

            pModel = new Fully_Init_Wide_Point_Feature_Measurement_Model(motion_model, MAXIMUM_ANGLE_DIFFERENCE);
            if (pModel.feature_type == type)
                return pModel;

            // The partially-initialised model also needs the fully-initalised
            // one that we have just created. Don't delete it - just store it
            Feature_Measurement_Model pFullModel = pModel;
            pModel = new Line_Init_Wide_Point_Feature_Measurement_Model(motion_model, pFullModel);
            if (pModel.feature_type == type)
                return pModel;
            else
            {
                pModel = null;
                pFullModel = null;
            }

            return null;
        }

    }


}
