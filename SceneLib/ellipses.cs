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
using sentience;

namespace SceneLibrary
{
    // Structure to hold the data for a particular ellipse
    public class SearchDatum
    {
        public SearchDatum(MatrixFixed _PuInv, Vector _search_centre)
        {
            PuInv = _PuInv;
            search_centre = _search_centre;
            result_flag = false;
            result_u = 0;
            result_v = 0;

            halfwidth = (uint)(SceneLib.NO_SIGMA /
                        Math.Sqrt(PuInv[0, 0] - PuInv[0, 1] * PuInv[0, 1] / PuInv[1, 1]));
            halfheight = (uint)(SceneLib.NO_SIGMA /
                         Math.Sqrt(PuInv[1, 1] - PuInv[0, 1] * PuInv[0, 1] / PuInv[0, 0]));

            if (halfwidth > 10) halfwidth = 10;
            if (halfheight > 10) halfheight = 10;
        }

        public int minx() { return (int)((search_centre[0] + 0.5) - halfwidth); }
        public int maxx() { return (int)((search_centre[0] + 0.5) + halfwidth); }
        public int miny() { return (int)((search_centre[1] + 0.5) - halfwidth); }
        public int maxy() { return (int)((search_centre[1] + 0.5) + halfwidth); }

        /// <summary>
        /// Checks whether the point(u,v)is inside the bounding box of the
        //  (three standard deviation) ellipse.
        /// </summary>
        /// <param name="u">The x-co-ordinate to test</param>
        /// <param name="v">The x-co-ordinate to test</param>
        /// <returns></returns>
        public bool inside_fast(int u, int v) 
        {
            u = Math.Abs(u - (int)(search_centre[0] + 0.5));
            if (u > (int)halfwidth)
                return false;
            v = Math.Abs(v - (int)(search_centre[1] + 0.5));
            if (v > (int)halfheight)
                return false;
            return true;
        }

        /// <summary>
        /// Checks whether the point (u,v) is inside the ellipse (within three
        /// standard deviations).
        /// </summary>
        /// <param name="u">The x-co-ordinate to test</param>
        /// <param name="v">The x-co-ordinate to test</param>
        /// <returns></returns>
        public bool inside(int u, int v)
        {
            u = Math.Abs(u - (int)(search_centre[0] + 0.5));
            v = Math.Abs(v - (int)(search_centre[1] + 0.5));
            return inside_relative(u,v);
        }


        /// <summary>
        /// Checks whether the point (u,v) is inside the ellipse (within three
        /// standard deviations). This version assume that distances are relative to the
        /// centre of the ellipse, not absolute co-ordinates.
        /// </summary>
        /// <param name="u">The horizontal distance from the search centre</param>
        /// <param name="v">The vertical distance from the search centre</param>
        /// <returns></returns>
        public bool inside_relative(int u, int v)
        {
            return (PuInv.Get(0, 0) * u * u
                    + 2 * PuInv.Get(0, 1) * u * v
                    + PuInv.Get(1, 1) * v * v < SceneLib.NO_SIGMA * SceneLib.NO_SIGMA);
        }

        /// The inverse covariance. Set in the constructor via add_ellipse()
        public MatrixFixed PuInv;
        /// The search centre. Set in the constructor via add_ellipse()
        public Vector search_centre;
        /// Set to <code>true</code> after search() if a successful match was found
        public bool result_flag;
        /// The x-location of the successful match, if found
        public uint result_u;
        /// The y-location of the successful match, if found
        public uint result_v;
        /** Half the width of the search ellipse. Used to quickly decide where to
        search */
        public uint halfwidth;
        /** Half the height of the search ellipse. Used to quickly decide where to
        search */
        public uint halfheight;
    }


    public class SearchMultipleOverlappingEllipses
    {
        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="image">The image to search</param>
        /// <param name="patch">The image patch to search for</param>
        /// <param name="BOXSIZE">The size of the image patch to use</param>
        public SearchMultipleOverlappingEllipses(classimage_mono image, classimage_mono patch, uint BOXSIZE)
        {
            m_image = image;
            m_patch = patch;
            m_boxsize = BOXSIZE;

            // Rather than working out an overall bounding box, we'll be slightly
            // lazy and make an array for correlation results which is the same size 
            // as the image
            // Pixels in this array in raster order
            correlation_image = new float[m_image.width, m_image.height];
        }

        /// <summary>
        /// Add a search ellipse to the class.
        /// @param PuInv The inverse covariance of the feature location. The search will be
        /// over the ellipse defined by NO_SIGMA standard deviations from the centre of the
        /// search.
        /// </summary>
        /// <param name="PuInv"></param>
        /// <param name="search_centre">The centre of the search</param>
        public void add_ellipse(MatrixFixed PuInv, Vector search_centre)
        {
            m_searchdata.Add(new SearchDatum(PuInv, search_centre));
        }

        /// <summary>
        /// Search for the image patch over all of the ellipses registered with the class.
        /// Since correlation is expensive we locally cache the correlation results so that
        /// we only do it once at each image location.
        /// </summary>
        public virtual void search()
        {
            // Set all positions to impossible correlation value
            for (int x = 0; x < m_image.width; x++)
                for (int y = 0; y < m_image.height; y++)
                    correlation_image[x,y] = -1.0f;

            // Now, we loop over ellipses
            foreach (SearchDatum i in m_searchdata)
            {
                // Limits of search 
                int urelstart = -(int)(i.halfwidth);
                int urelfinish = (int)(i.halfwidth);
                int vrelstart = -(int)(i.halfheight);
                int vrelfinish = (int)i.halfheight;

                int ucentre = (int)(i.search_centre[0]);
                int vcentre = (int)(i.search_centre[1]);

                // Check these limits aren't outside the image 
                if (ucentre + urelstart - (int)(m_boxsize - 1) / 2 < 0)
                {
                    urelstart = (int)(m_boxsize - 1) / 2 - ucentre;
                }
                if (ucentre + urelfinish - (int)(m_boxsize - 1) / 2 >
                    (int)(m_image.width) - (int)(m_boxsize))
                {
                    urelfinish = (int)(m_image.width) - (int)(m_boxsize) -
                                 ucentre + (int)(m_boxsize - 1) / 2;
                }
                if (vcentre + vrelstart - (int)(m_boxsize - 1) / 2 < 0)
                {
                    vrelstart = (int)(m_boxsize - 1) / 2 - vcentre;
                }
                if (vcentre + vrelfinish - (int)(m_boxsize - 1) / 2 >
                    (int)(m_image.height) - (int)(m_boxsize))
                {
                    vrelfinish = (int)(m_image.height) - (int)(m_boxsize) -
                                 vcentre + (int)(m_boxsize - 1) / 2;
                }

                // Search counters 
                int urel, vrel; 

                float corrmax = 1000000.0f;
                float corr;

                // For passing to and_correlate2_warning
                float sdpatch=0, sdimage=0;

                // Do the search 
                for (urel = urelstart; urel <= urelfinish; urel++)
                {
                    for(vrel = vrelstart; vrel <= vrelfinish; vrel++)
                    {
                        if (i.inside_relative(urel, vrel))
                        {
                            //if (DEBUGDUMP) cout << "Searching at "
                            //    << ucentre + urel << ", "
                            //    << vcentre + vrel
                            //    << endl;
    
                            // We are inside ellipse
                            // Has this place been searched before?
                            float corr_ptr = correlation_image[ucentre + urel, vcentre + vrel];

                            if(corr_ptr != -1.0)
                            {
                                corr = corr_ptr;
                                //if (DEBUGDUMP) cout << "Searched before, score " << corr << endl;
                            }
                            else
                            {
                                corr = SceneLib.correlate2_warning(0, 0, (int)m_boxsize, (int)m_boxsize, 
                                                          ucentre + urel - (int)(m_boxsize - 1) / 2,
                                                          vcentre + vrel - (int)(m_boxsize - 1) / 2,
                                                          ref m_patch, ref m_image, ref sdpatch, ref sdimage);

                                if (sdimage < SceneLib.CORRELATION_SIGMA_THRESHOLD)
                                {
                                    //if (DEBUGDUMP) cout << "Low image sigma sdimage " << sdimage << endl;
                                    corr += SceneLib.LOW_SIGMA_PENALTY;
                                }

                                correlation_image[ucentre + urel, vcentre + vrel] = corr;

                                //if (DEBUGDUMP) cout << "New search, score " << corr << endl;
                            }

                            if (corr <= corrmax)
                            {
                                corrmax = corr;
                                i.result_u = (uint)(urel + ucentre);
                                i.result_v = (uint)(vrel + vcentre);
                            }
                        }
                    }
                }



                //Debug.WriteLine("Best match correlation: " + corrmax);

                // Threshold correlation score: check if good enough 
                if (corrmax > SceneLib.CORRTHRESH2)
                {
                    // cout << "Matching correlation  not good enough." << endl;
                    i.result_flag = false;
                }
                else
                    i.result_flag = true;

                //if (DEBUGDUMP) cout << "Search " << count++ << " correlation " << corrmax
                        //<< " location " << i.result_u << ", "
                        //<< i.result_v << endl;
            }
        }


        /// Typdef the SearchDatum container for convenience
        public ArrayList m_searchdata = new ArrayList();

        /// How many ellipses are registered with the class?
        public uint size() { return (uint)(m_searchdata.Count); }
        /// Iterator set to the start of the container of ellipse data
        //public SearchDatum.const_iterator begin() { return m_searchdata.begin(); }
        /// Iterator set to one past the end of the container of ellipse data
        //public SearchData.const_iterator end() { return m_searchdata.end(); }

        public classimage_mono image() { return m_image; }
        public classimage_mono patch() { return m_patch; }
        public uint boxsize() { return m_boxsize; }

        private classimage_mono m_image;
        private classimage_mono m_patch;
        private uint m_boxsize;
        private float[,] correlation_image;

    }
}
