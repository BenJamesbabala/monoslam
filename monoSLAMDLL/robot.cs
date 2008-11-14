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
using System.Collections;
using System.Collections.Generic;
using System.Text;
using SceneLibrary;
using sentience;

namespace monoSLAM
{

    /// <summary>
    /// Class to model a Robot that is a hand-held greyscale camera, such as for the
    /// MonoSLAM real-time single-camera SLAM. This class measures features of with an
    /// Identifier of type ImageMonoExtraData, and has no control input.
    /// </summary>
    public class Robot : Sim_Or_Rob
    {

        public override bool set_control(Vector v, float d)
        {
            return true;
        }

        public override bool continue_control(Vector v, float d)
        {
            return true;
        }

        public override float wait_for_end_of_motion(Vector v)
        {
            return 0.0f;
        }

        public override bool stop_vehicle()
        {
            return true;
        }

        /// <summary>
        /// Set the current image selection (the co-ordinates of the centre of the image patch.
        /// </summary>
        /// <param name="u">The x-coordinate of the current image selection</param>
        /// <param name="v">The y-coordinate of the current image selection</param>
        public void set_image_selection(uint u, uint v) { uu = u; vv = v; location_selected_flag = true; }
        // Cancel the current image selection.
        public void nullify_image_selection() { location_selected_flag = false; }
        // Is there currently an image location selected?
        public bool location_selected() { return location_selected_flag; }
        // Returns the x-coordinate of the current image selection.
        public uint get_uu() { return uu; }
        // Returns the y-coordinate of the current image selection.
        public uint get_vv() { return vv; }


        // Coordinates of currently selected point in image
        protected uint uu, vv;
        protected bool location_selected_flag;
        public classimage_mono image;
        public classimage image_colour;
        public classimage_mono outputimage;
        protected bool show_ellipses;
        public bool calibrating;

        /// <summary>
        /// Constructor
        /// </summary>
        public Robot()
            : base("ROBOT")
        {
            location_selected_flag = false;
        }


        /// <summary>
        /// Make a measurement of a feature. This function calls elliptical_search() to
        /// find the best match within three standard deviations of the predicted location.
        /// </summary>
        /// <param name="patch">The identifier for this feature (in this case an image patch)</param>
        /// <param name="z">The best image location match for the feature, to be filled in by this function</param>
        /// <param name="h">The expected image location</param>
        /// <param name="S">The expected location covariance, used to specify the search region.</param>
        /// <returns></returns>
        public override bool measure_feature(classimage_mono patch, ref Vector z, Vector vz, Vector h, MatrixFixed S, Random rnd)
        {
            Cholesky S_cholesky = new Cholesky(S);
            MatrixFixed Sinv = S_cholesky.Inverse();

            uint u_found = 0, v_found = 0;
            
            if (SceneLib.elliptical_search(image, patch,
                    h, Sinv, ref u_found, ref v_found, vz, Camera_Constants.BOXSIZE, outputimage, show_ellipses, calibrating, rnd) != true)
            {
                // Feature not successfully matched
                return false;
            }

            z.Put(0, (float)u_found);
            z.Put(1, (float)v_found);
            
            return true;
        }

        /// <summary>
        /// Initialise a known feature. In this case it is assumed that a known feature
        /// is an image patch to be loaded from a file <code>known_patch?.bmp</code>.
        /// </summary>
        /// <param name="fmm"></param>
        /// <param name="v"></param>
        /// <param name="known_feature_label"></param>
        /// <returns></returns>
        public override classimage_mono initialise_known_feature(Feature_Measurement_Model fmm,
                                                   Vector v, uint known_feature_label, String path)
        {
            String name = Camera_Constants.known_point_patch_stem + known_feature_label + ".bmp";

            

            classimage_mono patch = new classimage_mono();
            if (!patch.loadFromBitmapMono(path + name, (int)Camera_Constants.BOXSIZE, (int)Camera_Constants.BOXSIZE))
            {
                patch = null;
            }

            return patch;
        }


        /// <summary>
        /// Initialise a known feature, in this case by loading an image file. The name of
        /// the file is read from the Settings Section passed to this function, with the
        /// entry Identifier.
        /// </summary>
        /// <param name="fmm"></param>
        /// <param name="v"></param>
        /// <param name="section"></param>
        /// <returns></returns>
        public override classimage_mono initialise_known_feature(Feature_Measurement_Model fmm,
                                                            Vector v, Settings.Section section,
                                                            String path)
        {
            ArrayList values = section.get_entry("Identifier");
            String name = (String)values[0];            

            //cout << "Reading patch " << name << endl;

            classimage_mono patch = new classimage_mono();
            if (!(patch.loadFromBitmapMono(path + name, (int)Camera_Constants.BOXSIZE, (int)Camera_Constants.BOXSIZE)))
            {
                patch = null;
            }

            return patch;
        }

        /// <summary>
        /// load a new image from the camera
        /// </summary>
        /// <param name="imageptr">camera image</param>
        public virtual void load_new_image(classimage_mono imageptr, classimage imageptr_colour, classimage_mono outputimageptr, bool show_ellipses)
        {
            image = imageptr;
            image_colour = imageptr_colour;
            outputimage = outputimageptr;
            this.show_ellipses = show_ellipses;
        }

        /// <summary>
        /// Little service function to copy image region centred at uu, vv into patch
        /// </summary>
        /// <param name="im">the image to extract the patch from</param>
        /// <param name="patch">the patch to be returned</param>
        /// <param name="uu">x coordinate of the centre of the patch within the image</param>
        /// <param name="vv">y coordinate of the centre of the patch within the image</param>
        public void copy_into_patch(classimage_mono im, classimage_mono patch,
                                    uint uu, uint vv)
        {
            for (uint r = 0; r < Camera_Constants.BOXSIZE; r++)
                for (uint c = 0; c < Camera_Constants.BOXSIZE; c++)
                {
                    int x = (int)(c + uu - (Camera_Constants.BOXSIZE - 1) / 2);
                    int y = (int)(r + vv - (Camera_Constants.BOXSIZE - 1) / 2);

                    patch.image[c, r] = im.image[x, y];
                }
        }


        //**************************Write Image Patch to Disk**************************

        /// <summary>
        /// Save the currently-selected patch to a file.
        /// </summary>
        /// <param name="name"></param>
        public void write_patch(String name)
        {
            classimage_mono hip = new classimage_mono();
            hip.createImage((int)Camera_Constants.BOXSIZE, (int)Camera_Constants.BOXSIZE);

            if (location_selected_flag)
            {
                // Copy the selected patch to the save space patch 
                copy_into_patch(image, hip, uu, vv);
                hip.SaveAsBitmapMono(name);
            }
        }

        /// <summary>
        /// Save the currently-selected patch to a file.
        /// </summary>
        /// <param name="name"></param>
        public void write_patch()
        {
            String name = "patch.bmp";
            write_patch(name);
        }


        /// <summary>
        /// Make the initial measurement of the currently-selected feature. This fills in
        /// the location and the identifier (a copy of the image patch) for the current
        /// feature. The feature location is set using set_image_selection_automatically()
        /// or manually by the user using set_image_selection(). This function just calls
        /// partially_initialise_point_feature() to fill in the measurement and identifier.
        /// </summary>
        /// <param name="z">The measurement vector to be filled in.</param>
        /// <param name="id_ptr">The identifier to be filled in.</param>
        /// <param name="m"></param>
        /// <returns>true on success, or <code>false</code> on failure (e.g. no patch is currently selected).</returns>
        public override bool make_initial_measurement_of_feature(Vector z, ref classimage_mono patch, Partially_Initialised_Feature_Measurement_Model m, Vector patch_colour)
        {
            patch = partially_initialise_point_feature(z);

            for (int c = 0; c < 3; c++) patch_colour[c] = image_colour.image[uu, vv, c];

            if (patch != null)
                return true;
            else
                return false;
        }

        /// <summary>
        /// Creates a new ImageMonoExtraData to represent the currently-selected image
        /// patch, and also returns its location in the parameter z. The current
        /// image patch is set using set_image_selection_automatically() or manually by the
        /// user using set_image_selection().
        /// </summary>
        /// <param name="z">The measurement vector to be filled in</param>
        /// <returns>The classimage holding this image patch information (created using new, so must be deleted elsewhere), or null if no patch is currently selected.</returns>
        public classimage_mono partially_initialise_point_feature(Vector z)
        {
            if (location_selected_flag) // Patch selected
            {
                // Go and initialise it in scene + 3D  
                classimage_mono hip = new classimage_mono();
                hip.createImage((int)Camera_Constants.BOXSIZE, (int)Camera_Constants.BOXSIZE);

                // Final save of fixated image patch
                copy_into_patch(image, hip, uu, vv);

                // And set measurement
                z.Put(0, uu);
                z.Put(1, vv);

                // return the patch
                return hip;
            }
            else
            {
                // No patch selected
                return null;
            }
        }

        /// <summary>
        /// Search a region for the best image patch, and set the current selection to
        /// this. This just calls find_best_patch_inside_region() to find a patch using the
        /// Shi and Tomasi criterion.
        /// </summary>
        /// <param name="ustart">The x-cordinate of the start of the region</param>
        /// <param name="vstart">The y-cordinate of the start of the region</param>
        /// <param name="ufinish">The x-cordinate of the end of the region</param>
        /// <param name="vfinish">The y-cordinate of the end of the region</param>
        /// <returns>The smallest eigenvalue of the best patch (high means good for correlation)</returns>
        public float set_image_selection_automatically(uint ustart, uint vstart, uint ufinish, uint vfinish)
        {
            float evbest = 0;

            SceneLib.find_best_patch_inside_region(image, ref uu, ref vv, ref evbest, Camera_Constants.BOXSIZE, ustart, vstart, ufinish, vfinish);
            location_selected_flag = true;
            //if (Camera_Constants.DEBUGDUMP) cout << "Found patch with score " << evbest << endl;

            return evbest;
        }


        /// <summary>
        /// Make measurements of a feature which is represented by a set of particles.
        /// This is typically a partially-initialised feature (see
        /// Partially_Initialised_Feature_Measurement_Model), where the particles represent
        /// the probability distribution in the direction of the free parameters.
        /// </summary>
        /// <param name="id">The Identified for this feature (in this case this will be an image patch)</param>
        /// <param name="particle_vector">The vector of particles. The covariance of each particle is used to define the region over which the feature is matched and measured.</param>
        public void measure_feature_with_multiple_priors(classimage_mono patch, ArrayList particle_vector)
        {
            
            SearchMultipleOverlappingEllipses ellipse_search =
                new SearchMultipleOverlappingEllipses(image, patch, Camera_Constants.BOXSIZE);

            SearchDatum e;
            foreach (Particle part in particle_vector)
            {
                ellipse_search.add_ellipse(part.get_SInv(), part.get_h());

                //if (Camera_Constants.DEBUGDUMP)  cout << "Particle at " << part->get_lambda()
                  //           << " h " << part->get_h()
                    //         << " SInv " << part->get_SInv() << endl;
            }

            // cout << "MFWMP timer before search_multiple: " << timerlocal << endl;
            
            ellipse_search.search();

            // cout << "MFWMP timer after search_multiple: " << timerlocal << endl;

            // Turn results into matrix forms
            int i = 0;
            foreach (Particle it in particle_vector)
            {
                e = (SearchDatum)ellipse_search.m_searchdata[i];

                if (e.result_flag)
                {
                    // Save the measurement location back into the particle
                    Vector z_local = new Vector(2);
                    z_local[0] = e.result_u;
                    z_local[1] = e.result_v;
                    it.set_z(z_local);
                    it.set_successful_measurement_flag(true);
                }
                else
                {
                    it.set_successful_measurement_flag(false);
                }
                i++;
            }

            // cout << "MFWMP timer end: " << timerlocal << endl;
            
        }


    }

}
