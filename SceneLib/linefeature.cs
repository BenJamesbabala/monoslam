using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using sentience;

namespace SceneLibrary
{
    public class linefeature
    {
        public Feature feature1, feature2;
        public int no_of_points;
        public Byte[,] pixel_intensity;
        public Vector[,] feature_position;
        private int curr_index;
        public int history;
        public Byte hits;
        public bool marked_for_deletion;

        public linefeature(Feature feature1, Feature feature2, int no_of_points, int history)
        {
            marked_for_deletion = false;
            curr_index = 0;
            hits = 0;
            this.feature1 = feature1;
            this.feature2 = feature2;
            this.no_of_points = no_of_points;
            this.history = history;
            pixel_intensity = new Byte[no_of_points, history];
            feature_position = new Vector[history, 2];

            for (int i = 0; i < history; i++)
            {
                feature_position[i, 0] = new Vector(3);
                feature_position[i, 1] = new Vector(3);
            }
        }

        /// <summary>
        /// update the line history
        /// </summary>
        /// <param name="img"></param>
        public void update(classimage_mono img)
        {
            int i;
            Vector position1 = feature1.get_z();
            Vector position2 = feature2.get_z();

            // store the feature positions
            for (i = 0; i < 3; i++)
            {
                feature_position[curr_index, 0][i] = feature1.get_y()[i];
                feature_position[curr_index, 1][i] = feature2.get_y()[i];
            }

            // distances between features in image coordinates
            float dist_u = position1[0] - position2[0];
            float dist_v = position1[1] - position2[1];

            marked_for_deletion = false;
            for (i = 0; i < no_of_points; i++)
            {
                int x = (int)(position1[0] + (i * dist_u / no_of_points));
                if ((x > 0) && (x < img.width))
                {
                    int y = (int)(position1[1] + (i * dist_v / no_of_points));
                    if ((y > 0) && (y < img.height))
                        pixel_intensity[i, curr_index] = img.image[x, y];
                    else 
                        marked_for_deletion = true;
                }
                else marked_for_deletion = true;
            }
            curr_index++;
            if (curr_index >= history) curr_index = 0;
            if (hits < 254) hits++;
        }

        /// <summary>
        /// display the line history as an image
        /// </summary>
        /// <param name="img"></param> 
        public void show(classimage_mono img)
        {
            for (int x = 0; x < img.width; x++)
            {
                int xx = x * history / img.width;
                for (int y = 0; y < img.height; y++)
                {                    
                    int yy = y * no_of_points / img.height;
                    img.image[x, y] = pixel_intensity[yy, xx];
                }
            }
        }

    }
}
