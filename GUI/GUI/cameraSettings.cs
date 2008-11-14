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
using System.Collections.Generic;
using System.Text;

namespace WindowsApplication1
{
    public class cameraSettings
    {
        public String cameraName = "";
        public String frameRate = "";
        public String resolution = "";
        public bool firstTime = true;
        public bool leftImage;
        public float field_of_vision = 40;
        public float lens_distortion = 440;
        public float centre_of_distortion_x = 162;
        public float centre_of_distortion_y = 125;
        public float distance_to_target_mm = 900;
        public float target_width_mm = 210;
        public float target_height_mm = 148.5f;

        public void Load()
        {
            //System.IO.File oFile;
            StreamReader oRead=null;
            String str;
            bool filefound = true;
            String filename;

            filename = "leftCamera.txt";
            cameraName = "";

            try
            {
                oRead = File.OpenText(System.Windows.Forms.Application.StartupPath + "\\" + filename);
            }
            catch //(Exception ex)
            {
                filefound = false;
            }

            if (filefound)
            {
                str = oRead.ReadLine();
                if (str!=null) cameraName = str;

                str = oRead.ReadLine();
                if (str != null) frameRate = str;

                str = oRead.ReadLine();
                if (str != null) resolution = str;

                str = oRead.ReadLine();
                if (str != null) field_of_vision = Convert.ToSingle(str);

                str = oRead.ReadLine();
                if (str != null) lens_distortion = Convert.ToSingle(str);

                str = oRead.ReadLine();
                if (str != null) centre_of_distortion_x = Convert.ToSingle(str);

                str = oRead.ReadLine();
                if (str != null) centre_of_distortion_y = Convert.ToSingle(str);

                str = oRead.ReadLine();
                if (str != null) distance_to_target_mm = Convert.ToSingle(str);

                str = oRead.ReadLine();
                if (str != null) target_width_mm = Convert.ToSingle(str);

                str = oRead.ReadLine();
                if (str != null) target_height_mm = Convert.ToSingle(str);

                oRead.Close();
            }
        }


        public void Save()
        {
            StreamWriter oWrite=null;
            bool allowWrite = true;
            String filename;

            if (leftImage)
                filename = "leftCamera.txt";
                else
                filename = "rightCamera.txt";

            try
            {
                oWrite = File.CreateText(System.Windows.Forms.Application.StartupPath + "\\" + filename);
            }
            catch //(Exception ex)
            {
                allowWrite = false;
            }

            if (allowWrite)
            {
                oWrite.WriteLine(cameraName);
                oWrite.WriteLine(frameRate);
                oWrite.WriteLine(resolution);
                oWrite.WriteLine(field_of_vision);
                oWrite.WriteLine(lens_distortion);
                oWrite.WriteLine(centre_of_distortion_x);
                oWrite.WriteLine(centre_of_distortion_y);
                oWrite.WriteLine(distance_to_target_mm);
                oWrite.WriteLine(target_width_mm);
                oWrite.WriteLine(target_height_mm);
                oWrite.Close();
            }

        }

    }
}
