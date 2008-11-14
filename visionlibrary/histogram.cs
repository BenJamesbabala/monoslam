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

namespace sentience
{
    public class histogram
    {
        private int maxval;
        private int[] temp_level;

        public int no_of_levels;
        public int[] level;
        public int[] min_value;
        public int[] max_value;

        public void init(int NoOfLevels)
        {
            no_of_levels = NoOfLevels;
            level = new int[no_of_levels];
            temp_level = new int[no_of_levels];
            min_value = new int[no_of_levels];
            max_value = new int[no_of_levels];
            clear();
        }

        /// <summary>
        /// show the histogram in an image object
        /// </summary>
        /// <param name="img"></param>
        public void Show(classimage img)
        {
            int prev_x = 0, prev_y = 0;
            img.clear();
            for (int i = 0; i < no_of_levels; i++)
            {
                int x = i * img.width / no_of_levels;
                int y = img.height - (level[i] * (img.height * 8 / 10) / maxval);
                if (i > 0)
                {
                    img.drawLine(prev_x, prev_y, x, y, 0, 255, 0, 0);
                }
                prev_x = x;
                prev_y = y;
            }
        }

        public void Show(classimage_mono img)
        {
            int prev_x = 0, prev_y = 0;
            img.clear();
            for (int i = 0; i < no_of_levels; i++)
            {
                int x = i * img.width / no_of_levels;
                int y = img.height - (level[i] * (img.height * 8 / 10) / maxval);
                if (i > 0)
                {
                    img.drawLine(prev_x, prev_y, x, y, 0);
                }
                prev_x = x;
                prev_y = y;
            }
        }


        //shuffle backwards or forwards by the given number of levels
        public void shuffle(int levels)
        {
            int i, j;

            for (i = 0; i < no_of_levels; i++) temp_level[i] = level[i];
            for (i = 0; i < no_of_levels; i++)
            {
                j = i + levels;
                if (j >= no_of_levels) j -= no_of_levels;
                if (j < 0) j += no_of_levels;
                level[j] = temp_level[i];
            }
        }

        public void clear()
        {
            int i;
            for (i = 0; i < no_of_levels; i++) level[i] = 0;
            maxval = 1;
        }

        public void add(int hist_level, int value)
        {
            if (hist_level >= no_of_levels) hist_level = no_of_levels - 1;
            level[hist_level] += value;
            if (level[hist_level] > maxval) maxval = level[hist_level];
        }

        public void add(int hist_level, int value, int minvalue, int maxvalue)
        {
            if (hist_level < no_of_levels)
            {
                add(hist_level, value);
                min_value[hist_level] = minvalue;
                max_value[hist_level] = maxvalue;
            }
        }

        //reduce the number of levels in the histogram
        public void reduceLevels()
        {
            int i, loser, min, activity;

            if (no_of_levels > 2)
            {
                loser = 0;
                min = 999999;
                //look for the two least active levels
                for (i = 0; i < no_of_levels - 1; i++)
                {
                    activity = level[i] + level[i + 1];
                    if (activity < min)
                    {
                        min = activity;
                        loser = i;
                    }
                }

                if (level[loser + 1] > 0)
                {
                    if (level[loser] > 0)
                    {
                        max_value[loser] = max_value[loser + 1];
                        level[loser] += level[loser + 1];
                    }
                    else
                    {
                        min_value[loser] = min_value[loser + 1];
                        max_value[loser] = max_value[loser + 1];
                        level[loser] = level[loser + 1];
                    }
                }

                if (level[loser] > maxval) maxval = level[loser];
                for (i = loser + 1; i < no_of_levels - 1; i++)
                {
                    level[i] = level[i + 1];
                    min_value[i] = min_value[i + 1];
                    max_value[i] = max_value[i + 1];
                }
                no_of_levels--;
                normalise();
            }
        }

        public void normalise()
        {
            int i;
            for (i = 0; i < no_of_levels; i++) level[i] = (level[i] * 100) / maxval;
            maxval = 100;
        }

    }

}
