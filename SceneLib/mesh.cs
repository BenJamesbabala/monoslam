using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using sentience;

namespace SceneLibrary
{
    public class model_vertex
    {
        public Feature vertex_feature;
        Vector position;

        public model_vertex(Feature feat)
        {
            position = new Vector(3);
            vertex_feature = feat;
            position[0] = feat.get_y()[0];
            position[1] = feat.get_y()[1];
            position[2] = feat.get_y()[2];
        }
    }

    public class model_triangle
    {
        public model_vertex[] vertices;
        public float[,,] offsets;
        public model_mesh mesh;


        public void Show(classimage_mono img)
        {
            int i, j;
            int[,] screen_position = new int[3, 2];
            for (i = 0; i < 3; i++)
            {
                Vector screen_pos = ((model_vertex)vertices[i]).vertex_feature.get_h();
                screen_position[i, 0] = (int)screen_pos[0];
                screen_position[i, 1] = (int)screen_pos[1];
            }

            for (i = 0; i < 3; i++)
            {
                for (j = 0; j < 3; j++)
                {
                    if (i != j)
                    {
                        img.drawLine(screen_position[i, 0], screen_position[i, 1],
                                     screen_position[j, 0], screen_position[j, 1], 0);
                    }
                }
            }
        }

        /// <summary>
        /// returns true if the triangle is currently visible
        /// </summary>
        /// <returns></returns>
        public bool isVisible()
        {
            bool visible = true;
            int i = 0;
            Feature feat;

            while ((i < 3) && (visible))
            {
                feat = ((model_vertex)vertices[i]).vertex_feature;
                if (!mesh.featureVisible(feat))
                    visible = false;
                i++;
            }
            return (visible);
        }


        /// <summary>
        /// is the given feature inside the triangle ?
        /// </summary>
        /// <param name="feat"></param>
        /// <returns></returns>
        public bool isInside(Feature feat)
        {
            Vector screen_position = feat.get_h();
            return (isInside((int)screen_position[0], (int)screen_position[1]));
        }

        /// <summary>
        /// is the given x,y point inside the triangle?
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns></returns>
        public bool isInside(int x, int y)
        {
            int i, j;
            bool c = false;
            int[] x_points = new int[3];
            int[] y_points = new int[3];

            for (i = 0; i < 3; i++)
            {
                Vector screen_position = ((model_vertex)vertices[i]).vertex_feature.get_h();
                x_points[i] = (int)(screen_position[0]);
                y_points[i] = (int)(screen_position[1]);
            }

            for (i = 0, j = 3 - 1; i < 3; j = i++)
            {
                if ((((y_points[i] <= y) && (y < y_points[j])) ||
                     ((y_points[j] <= y) && (y < y_points[i]))) &&
                     (x < (x_points[j] - x_points[i]) * (y - y_points[i]) / 
                     (y_points[j] - y_points[i]) + x_points[i]))
                     c = !c;
            }
            return (c);
        }

        /// <summary>
        /// update the distances of each vertex from the others
        /// </summary>
        public void update_offsets()
        {
            for (int i = 0; i < 3; i++)
            {
                Vector pos1 = ((model_vertex)vertices[i]).vertex_feature.get_y();
                for (int j = i+1; j < 3; j++)
                {
                    Vector pos2 = ((model_vertex)vertices[j]).vertex_feature.get_y();
                    for (int k = 0; k < 3; k++)
                    {
                        float dist = pos2[k] - pos1[k];
                        offsets[i, j, k] = dist;
                        offsets[j, i, k] = -dist;
                    }
                }
            }
        }

        public model_triangle(model_mesh mesh)
        {
            this.mesh = mesh;
            vertices = new model_vertex[3];
            offsets = new float[3, 3, 3];
        }
    }

    public class model_mesh
    {
        // maximum separation between points in the mesh
        public float max_separation = 1;

        public ArrayList triangles;
        public ArrayList features;
        private ArrayList features_pending;

        public Scene_Single scene;
        public int image_width = 320;
        public int image_height = 240;

        public model_mesh()
        {
            triangles = new ArrayList();
            features = new ArrayList();
            features_pending = new ArrayList();
        }

        /// <summary>
        /// returns the closest two neighbours to the given feature
        /// </summary>
        /// <param name="feat"></param>
        /// <param name="neighbour1"></param>
        /// <param name="neighbour2"></param>
        private void closest_neighbours(Feature feat, ref Feature neighbour1, ref Feature neighbour2)
        {
            int i, x, y;
            int dist, min_dist;

            x = (int)feat.get_h()[0];
            y = (int)feat.get_h()[1];

            neighbour1 = null;
            neighbour2 = null;

            i = 0;
            min_dist = 9999;
            while (i < features.Count)
            {
                Feature f = (Feature)features[i];
                if (f != feat)
                {
                    if (featureVisible(f))
                    {
                        Vector screen_position = f.get_h();
                        dist = (int)Math.Abs(screen_position[0] - x);
                        dist += (int)Math.Abs(screen_position[1] - y);
                        if (dist < min_dist)
                        {
                            min_dist = dist;
                            neighbour2 = neighbour1;
                            neighbour1 = f;
                        }
                    }
                }
                i++;
            }
        }

        private void reconstructFeaturePosition(Feature feat)
        {
            Vector fpos;
            Vector position = new Vector(3);
            int i, k, hits = 1;

            fpos = feat.get_y();
            for (k = 0; k < 3; k++) position[k] = fpos[k];

            for (i = 0; i < feat.triangles.Count; i++)
            {
                Feature f;
                int j, index = 0;
                model_triangle tri = (model_triangle)feat.triangles[i];
                for (j = 0; j < 3; j++)
                {
                    f = ((model_vertex)tri.vertices[j]).vertex_feature;
                    if (f == feat) index = j;
                }
                for (j = 0; j < 3; j++)
                {
                    if (j != index)
                    {
                        f = ((model_vertex)tri.vertices[j]).vertex_feature;
                        if (!((f.get_successful_measurement_flag()) && (f.get_feature_measurement_model().fully_initialised_flag)))
                        {
                            fpos = f.get_y();
                            for (k = 0; k < 3; k++)
                            {
                                position[k] += fpos[k] - tri.offsets[index, j, k];
                            }
                            hits++;
                        }
                    }
                }

            }

            if (hits > 0)
            {
                for (k = 0; k < 3; k++) position[k] /= hits;
                fpos = feat.get_y();
                for (k = 0; k < 3; k++) fpos[k] = position[k];
            }
        }

        /// <summary>
        /// returns true if the given feature is visible
        /// </summary>
        /// <param name="feat"></param>
        /// <returns></returns>
        public bool featureVisible(Feature feat)
        {
            if (!((feat.get_successful_measurement_flag()) && (feat.get_feature_measurement_model().fully_initialised_flag)))
            {
                if (feat.get_feature_measurement_model().fully_initialised_flag)
                {
                    reconstructFeaturePosition(feat);
                    int screen_x = 0;
                    int screen_y = 0;
                    projectFeature(feat, ref screen_x, ref screen_y);
                    if ((screen_x > 0) && (screen_y > 0) && (screen_x < image_width) && (screen_y < image_height))
                    {
                        Vector v = new Vector(2);
                        v[0] = screen_x;
                        v[1] = screen_y;
                        feat.set_h(v);
                        return true;
                    }
                    else
                        return false;
                }
                else
                    return false;
            }
            else
                return true;
        }

        /// <summary>
        /// adds a new triangle
        /// </summary>
        /// <param name="f1"></param>
        /// <param name="f2"></param>
        /// <param name="f3"></param>
        private void add_triangle(Feature f1, Feature f2, Feature f3)
        {
            model_triangle new_tri = new model_triangle(this);
            new_tri.vertices[0] = new model_vertex(f1);
            new_tri.vertices[1] = new model_vertex(f2);
            new_tri.vertices[2] = new model_vertex(f3);
            new_tri.update_offsets();
            triangles.Add(new_tri);
            f1.triangles.Add(new_tri);
            f2.triangles.Add(new_tri);
            f3.triangles.Add(new_tri);
        }

        /// <summary>
        /// Insert the given feature into the given triangle.
        /// This creates three new triangles and removes the original
        /// </summary>
        /// <param name="feat"></param>
        /// <param name="tri"></param>
        private void update(Feature feat, model_triangle tri)
        {
            Feature f1 = ((model_vertex)tri.vertices[0]).vertex_feature;
            Feature f2 = ((model_vertex)tri.vertices[1]).vertex_feature;
            Feature f3 = ((model_vertex)tri.vertices[2]).vertex_feature;

            add_triangle(feat, f1, f2);
            add_triangle(feat, f2, f3);
            add_triangle(feat, f3, f1);
            f1.triangles.Remove(tri);
            f2.triangles.Remove(tri);
            f3.triangles.Remove(tri);
            triangles.Remove(tri);
        }

        /// <summary>
        /// show the mesh in the given image
        /// </summary>
        /// <param name="img"></param>
        public void Show(classimage_mono img)
        {
            for (int i = 0; i < features.Count; i++)
            {
                Feature feat = (Feature)features[i];
                if (featureVisible(feat))
                {
                    for (int j = 0; j < feat.triangles.Count; j++)
                    {
                        model_triangle tri = (model_triangle)feat.triangles[j];
                        if (tri.isVisible()) tri.Show(img);
                    }
                }
            }
        }

        public void addFeatureInit(Feature feat)
        {
            features_pending.Add(feat);
        }

        public void update(Scene_Single scene)
        {
            this.scene = scene;
            if (features_pending.Count > 0)
            {
                bool found = false;
                int i = 0;

                while ((i < features_pending.Count) && (!found))
                {
                    Feature f = (Feature)features_pending[i];
                    Vector pos = f.get_h();
                    if ((pos[0] != 0) || (pos[1] != 0))
                    {
                        if (featureVisible(f))
                        {
                            addFeature(f);
                            //found = true;                            
                        }
                        features_pending.Remove(f);
                    }
                    i++;
                }
            }
        }

        /// <summary>
        /// project a feature from 3D into 2D image coordinates
        /// </summary>
        /// <param name="feat"></param>
        public void projectFeature(Feature feat, ref int screen_x, ref int screen_y)
        {
            Vector yi = feat.get_y();
            Vector xp = scene.get_motion_model().get_xpRES();

            Fully_Initialised_Feature_Measurement_Model fully_init_fmm =
                feat.get_fully_initialised_feature_measurement_model();
            fully_init_fmm.func_hi_and_dhi_by_dxp_and_dhi_by_dyi(yi, xp);

            screen_x = (int)fully_init_fmm.get_hiRES()[0];
            screen_y = (int)fully_init_fmm.get_hiRES()[1];
        }

        private void addFeature(Feature feat)
        {
            if (!features.Contains(feat))
            {
                int j, i = 0;
                model_triangle insideTriangle = null;
                Feature f;

                // is this feature inside any visible triangle ?
                while ((i < features.Count) && (insideTriangle == null))
                {
                    f = (Feature)features[i];
                    if (featureVisible(f))
                    {
                        // examine the triangles to which this feature belongs
                        j = 0;
                        while ((j < f.triangles.Count) && (insideTriangle == null))
                        {
                            model_triangle tri = (model_triangle)f.triangles[j];
                            if (tri.isVisible())
                            {
                                if (tri.isInside(feat))
                                {
                                    insideTriangle = tri;
                                }
                            }
                            j++;
                        }
                    }
                    i++;
                }

                // add the feature
                features.Add(feat);

                if (insideTriangle != null)
                {
                    // insert the feature into the triangle
                    update(feat, insideTriangle);
                }
                else
                {
                    // connect the feature to its nearest neighbours
                    if (features.Count > 2)
                    {
                        Feature neighbour1 = null;
                        Feature neighbour2 = null;
                        /*
                        i = 0;
                        while (i < features.Count)
                        {
                            Feature f1 = (Feature)features[i];
                            if (f1 != feat)
                            {
                                if (featureVisible(f1))
                                {
                                    neighbour1 = f1;
                                    j = i + 1;
                                    while (j < features.Count)
                                    {
                                        Feature f2 = (Feature)features[j];
                                        if ((f2 != feat) && (f2 != neighbour1))
                                        {
                                            if (featureVisible(f2))
                                            {
                                                neighbour2 = f2;
                                                add_triangle(feat, neighbour1, neighbour2);
                                            }
                                        }
                                        j++;
                                    }
                                }
                            }
                            i++;
                        }
                        */
                        
                        closest_neighbours(feat, ref neighbour1, ref neighbour2);
                        if ((neighbour1 != null) && (neighbour2 != null))
                            add_triangle(feat, neighbour1, neighbour2);
                        
                    }
                }
            }
        }
    }
}
