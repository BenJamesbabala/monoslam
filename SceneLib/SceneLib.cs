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

    public class SceneLib
    {
        /// Threshold for stereo correlation using correlate2()
        public const float CORRTHRESH2 = 0.40f;
        /// Half-width of the strip we will search for stereo matches
        public const uint LINESEARCHWIDTH = 8;
        /// Value for the standard deviation of the intensity values in a patch
        /// below which we deem it unsuitable for correlation
        public const float CORRELATION_SIGMA_THRESHOLD = 10.0f;
        /// Number of standard deviations to search within images
        public const float NO_SIGMA = 3.0f;
        /// Hackish value we add to correlation score to penalise if patch sigma is low
        public const float LOW_SIGMA_PENALTY = 5.0f;
        /// test
        public const long MIN_PATCH_DIFFERENCE = 50;

        /// <summary>
        /// Calculate trace of a matrix
        /// </summary>
        /// <param name="m"></param>
        /// <returns></returns>
        public static float Trace(MatrixFixed M)
        {
            float sum = 0;
            int N = (M.Rows < M.Columns ? M.Rows : M.Columns);
            for (int i=0; i<N; ++i)
                sum += M[i, i];
            return sum;
        }

        /// <summary>
        /// return a random number uniformly drawn on [a, b).
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static float SampleUniform(float a, float b, Random rnd)
        {
            // it's your lucky day.
            float u = rnd.Next(10000)/10000.0f; // uniform on [0, 1)
            return ((1.0f - u)*a + u*b);
        }


        private static void SampleNormal2(ref float x, ref float y, Random rnd)
        {
            float u     = SampleUniform(0, 1, rnd);
            float theta = SampleUniform(0, 2 * 3.1415927f, rnd);

            float r = (float)Math.Sqrt(-2 * Math.Log(u));

            if (x != 0) x = r * (float)Math.Cos(theta);
            if (y != 0) y = r * (float)Math.Sin(theta);
        }


        public static float SampleNormal(float mean, float sigma, Random rnd)
        {
            float x=0, y=0;
            SampleNormal2(ref x, ref y, rnd);
            return mean + sigma * x;
        }



        /// <summary>
        /// Normalise sum-of-squared-difference between two patches. Assumes that the images
        /// are continuous memory blocks in raster order. The two pointer arguments return
        /// the standard deviations of the patches: if either of these are low, the results
        /// can be misleading.
        /// </summary>
        /// <param name="x0">Start x-co-ordinate of patch in first image</param>
        /// <param name="y0">End x-co-ordinate of patch in first image</param>
        /// <param name="x0lim">End x-co-ordinate of patch in first image</param>
        /// <param name="y0lim">End y-co-ordinate of patch in first image</param>
        /// <param name="x1">Start x-co-ordinate of patch in second image</param>
        /// <param name="y1">Start y-co-ordinate of patch in second image</param>
        /// <param name="p0">First image</param>
        /// <param name="p1">Second image</param>
        /// <param name="sd0ptr">Standard deviation of patch from first image</param>
        /// <param name="sd1ptr">Standard deviation of patch from second image</param>
        /// <returns></returns>
        public static float correlate2_warning(
                          int x0, int y0,
                          int x0lim, int y0lim,
                          int x1, int y1,
                          ref classimage_mono p0, ref classimage_mono p1,
                          ref float sd0ptr, ref float sd1ptr)
        {
            // Make these int rather than unsigned int for speed
            int patchwidth = x0lim - x0;
            int p0skip = p0.width - patchwidth;
            int p1skip = p1.width - patchwidth;
            int Sg0 = 0, Sg1 = 0, Sg0g1 = 0, Sg0sq = 0, Sg1sq = 0;

            float n = (x0lim - x0) * (y0lim - y0);     // to hold total no of pixels

            float varg0 = 0.0f, varg1 = 0.0f, sigmag0 = 0.0f, sigmag1 = 0.0f, g0bar = 0.0f, g1bar = 0.0f;

            // variances, standard deviations, means 
            float Sg0doub = 0.0f, Sg1doub = 0.0f, Sg0g1doub = 0.0f, Sg0sqdoub = 0.0f, Sg1sqdoub = 0.0f;

            float C = 0.0f;       // to hold the final result 
            float k = 0.0f;       // to hold an intermediate result 

            // at the moment the far right and bottom pixels aren't included 
            int v0, v1;

            
            for (int y0copy = y0lim - 1; y0copy >= 0; y0copy--)
            {
                int yy0 = y0 + y0copy;
                int yy1 = y1 + y0copy;
                for (int x0copy = x0lim - 1; x0copy >= 0; x0copy--)
                {

                    v0 = p0.image[x0 + x0copy, yy0];
                    v1 = p1.image[x1 + x0copy, yy1];
                    
                    Sg0 += v0;
                    Sg1 += v1;
                    Sg0g1 += v0 * v1;
                    Sg0sq += v0 * v0;
                    Sg1sq += v1 * v1;
                    
                }
            }
            

            Sg0doub = Sg0;
            Sg1doub = Sg1;
            Sg0g1doub = Sg0g1;
            Sg0sqdoub = Sg0sq;
            Sg1sqdoub = Sg1sq;

            g0bar = Sg0doub / n;
            g1bar = Sg1doub / n;

            varg0 = Sg0sqdoub / n - (g0bar * g0bar);
            varg1 = Sg1sqdoub / n - (g1bar * g1bar);

            sigmag0 = (float)Math.Sqrt(varg0);
            sigmag1 = (float)Math.Sqrt(varg1);

            sd0ptr = (float)sigmag0;
            sd1ptr = (float)sigmag1;

            if (sigmag0 == 0.0f)     // special checks for this algorithm 
            {                       // to avoid division by zero 
                if (sigmag1 == 0.0f)
                    return 0.0f;
                else
                    return 1.0f;
            }

            if (sigmag1 == 0.0f)
                return 1.0f;

            k = g0bar / sigmag0 - g1bar / sigmag1;

            C = Sg0sqdoub / varg0 + Sg1sqdoub / varg1 + n * (k * k)
                - Sg0g1doub * 2.0f / (sigmag0 * sigmag1)
                - Sg0doub * 2.0f * k / sigmag0 + Sg1doub * 2.0f * k / sigmag1;

            return (C / n);    // returns mean square no of s.d. from mean of pixels 
        }

        /// <summary>
        /// Simple function to find the eigenvalues of the 2*2 symmetric matrix
        /// </summary>
        /// <param name="A"></param>
        /// <param name="B"></param>
        /// <param name="C"></param>
        /// <param name="eval1ptr"></param>
        /// <param name="eval2ptr"></param>
        public static void find_eigenvalues(float A, float B, float C,
              ref float eval1ptr, ref float eval2ptr)
        {
            float BB = (float)Math.Sqrt((A + C) * (A + C) - 4 * (A * C - B * B));

            eval1ptr = (A + C + BB) / 2.0f;
            eval2ptr = (A + C - BB) / 2.0f;
        }

        public static void find_best_patch_inside_region_test(
                          classimage_mono image,
                          ref uint ubest, ref uint vbest, ref float evbest,
                          uint BOXSIZE, uint ustart, uint vstart,
                          uint ufinish, uint vfinish)
        {
            long corr;
            long corrmax = MIN_PATCH_DIFFERENCE * BOXSIZE * BOXSIZE / 2;
            int tx, ty, bx, by;

            for (int x = (int)ustart; x < ufinish; x++)
            {
                for (int y = (int)vstart; y < vfinish; y++)
                {
                    tx = (int)(x - (BOXSIZE - 1) / 2);
                    ty = (int)(y - (BOXSIZE - 1) / 2);
                    bx = (int)(tx + BOXSIZE);
                    by = (int)(ty + BOXSIZE);
                    long leftval = image.getIntegral(tx, ty, x, by);
                    long rightval = image.getIntegral(x, ty, bx, by);
                    long topval = image.getIntegral(tx, ty, bx, y);
                    long bottomval = image.getIntegral(tx, y, bx, by);

                    corr = Math.Abs(leftval - rightval) +
                           Math.Abs(topval - bottomval) + 
                           Math.Abs(leftval - topval) +
                           Math.Abs(rightval - topval) +
                           Math.Abs(leftval - bottomval) +
                           Math.Abs(rightval - bottomval);
                    if (corr > corrmax)
                    {
                        corrmax = corr;
                        ubest = (uint)x;
                        vbest = (uint)y;
                        evbest = corr;
                    }
                    

                }
            }
        }


        /// <summary>
        /// Function to scan over (a window in an) image and find the best patch by the Shi
        /// and Tomasi criterion. 
        /// Method: as described in notes from 1/7/97. Form sums in an incremental
        /// way to speed things up.
        /// </summary>
        /// <param name="image">The image to scan</param>
        /// <param name="ubest">The x-co-ordinate of the best patch</param>
        /// <param name="vbest">The y-co-ordinate of the best patch</param>
        /// <param name="evbest">The smallest eigenvalue of the best patch (larger is better)</param>
        /// <param name="BOXSIZE">The size of the patch to use</param>
        /// <param name="ustart">The x-co-ordinate of the start of the search window</param>
        /// <param name="vstart">The y-co-ordinate of the start of the search window</param>
        /// <param name="ufinish">The x-co-ordinate of the end of the search window</param>
        /// <param name="vfinish">The v-co-ordinate of the end of the search window</param>
        public static void find_best_patch_inside_region(
                          classimage_mono image,
                          ref uint ubest, ref uint vbest, ref float evbest,
                          uint BOXSIZE, uint ustart, uint vstart,
                          uint ufinish, uint vfinish)
        {
            // Check that these limits aren't too close to the image edges.
            // Note that we can't use the edge pixels because we can't form 
            // gradients here. 
            if (ustart < (BOXSIZE - 1) / 2 + 1)
                ustart = (BOXSIZE - 1) / 2 + 1;
            if (ufinish > image.width - (BOXSIZE - 1) / 2 - 1)
                ufinish = (uint)(image.width - (BOXSIZE - 1) / 2 - 1);
            if (vstart < (BOXSIZE - 1) / 2 + 1)
                vstart = (BOXSIZE - 1) / 2 + 1;
            if (vfinish > image.height - (BOXSIZE - 1) / 2 - 1)
                vfinish = (uint)(image.height - (BOXSIZE - 1) / 2 - 1);

            // Is there anything left to search? If not, set the score to zero and return.
            if ((vstart >= vfinish) || (ustart >= ufinish))
            {
                ubest = ustart;
                vbest = vstart;
                evbest = 0;
                return;
            }

            // Calculate the width we need to find gradients in. 
            uint calc_width = ufinish - ustart + BOXSIZE - 1;

            // Arrays which store column sums of height BOXSIZE 
            float[] CSgxsq = new float[calc_width];
            float[] CSgysq = new float[calc_width];
            float[] CSgxgy = new float[calc_width];

            // For the final sums at each position (u, v)
            float TSgxsq = 0.0f, TSgysq = 0.0f, TSgxgy = 0.0f;

            float gx, gy;
            uint u = ustart, v = vstart;
            float eval1 = 0, eval2 = 0;

            // Initial stage: fill these sums for the first horizontal position 
            uint cstart = ustart - (BOXSIZE - 1) / 2;
            uint cfinish = ufinish + (BOXSIZE - 1) / 2;
            uint rstart = vstart - (BOXSIZE - 1) / 2;
            uint i;
            uint c, r;
            for (c = cstart, i = 0; c < cfinish; c++, i++)
            {
                CSgxsq[i] = 0; CSgysq[i] = 0; CSgxgy[i] = 0;
                for (r = rstart; r < rstart + BOXSIZE; r++)
                {
                    gx = (image.image[c + 1, r] -
                        image.image[c - 1, r]) / 2.0f;
                    gy = (image.image[c, r + 1] -
                        image.image[c, r - 1]) / 2.0f;

                    CSgxsq[i] += gx * gx;
                    CSgysq[i] += gy * gy;
                    CSgxgy[i] += gx * gy;
                }
            }


            // Now loop through u and v to form sums 
            evbest = 0;
            for (v = vstart; v < vfinish; v++)
            {
                u = ustart;

                // Form first sums for u = ustart 
                TSgxsq = 0.0f;
                TSgysq = 0.0f;
                TSgxgy = 0.0f;
                for (i = 0; i < BOXSIZE; i++)
                {
                    TSgxsq += CSgxsq[i];
                    TSgysq += CSgysq[i];
                    TSgxgy += CSgxgy[i];
                }

                for (u = ustart; u < ufinish; u++)
                {
                    if (u != ustart)
                    {
                        // Subtract old column, add new one 
                        TSgxsq += CSgxsq[u - ustart + BOXSIZE - 1] - CSgxsq[u - ustart - 1];
                        TSgysq += CSgysq[u - ustart + BOXSIZE - 1] - CSgysq[u - ustart - 1];
                        TSgxgy += CSgxgy[u - ustart + BOXSIZE - 1] - CSgxgy[u - ustart - 1];
                    }

                    find_eigenvalues(TSgxsq, TSgxgy, TSgysq, ref eval1, ref eval2);

                    // eval2 will be the smaller eigenvalue. Compare it with the one
                    // we've already got 
                    if (eval2 > evbest)
                    {
                        ubest = u;
                        vbest = v;
                        evbest = eval2;
                    }
                }

                if (v != vfinish - 1)
                {
                    // Update the column sums for the next v 
                    for (c = cstart, i = 0; c < cfinish; c++, i++)
                    {
                        // Subtract the old top pixel
                        gx = (image.image[c + 1, v - (BOXSIZE - 1) / 2] -
                              image.image[c - 1, v - (BOXSIZE - 1) / 2]) / 2.0f;
                        gy = (image.image[c, v - (BOXSIZE - 1) / 2 + 1] -
                              image.image[c, v - (BOXSIZE - 1) / 2 - 1]) / 2.0f;
                        CSgxsq[i] -= gx * gx;
                        CSgysq[i] -= gy * gy;
                        CSgxgy[i] -= gx * gy;

                        // Add the new bottom pixel
                        gx = (image.image[c + 1, v + (BOXSIZE - 1) / 2 + 1] -
                              image.image[c - 1, v + (BOXSIZE - 1) / 2 + 1]) / 2.0f;
                        gy = (image.image[c, v + (BOXSIZE - 1) / 2 + 1 + 1] -
                              image.image[c, v + (BOXSIZE - 1) / 2 + 1 - 1]) / 2.0f;
                        CSgxsq[i] += gx * gx;
                        CSgysq[i] += gy * gy;
                        CSgxgy[i] += gx * gy;
                    }
                }
            }
        }

        /// <summary>
        /// Version to find the best n patches within an image.
        /// Not very efficiently implemented I'm afraid.
        /// Now we expect the arguments ubest, vbest, evbest to be arrays of
        /// size n.
        /// </summary>
        /// <param name="image">The image to scan</param>
        /// <param name="n">Number of patches</param>
        /// <param name="ubest">Array containing x-co-ordinates of n best patches</param>
        /// <param name="vbest">Array containing y-co-ordinates of n best patches</param>
        /// <param name="evbest">Array containing smallest eigenvalues of best patches (larger is better)</param>
        /// <param name="BOXSIZE">The size of the patch to use</param>
        /// <param name="ustart">The x-co-ordinate of the start of the search window</param>
        /// <param name="vstart">The y-co-ordinate of the start of the search window</param>
        /// <param name="ufinish">The x-co-ordinate of the end of the search window</param>
        /// <param name="vfinish">The v-co-ordinate of the end of the search window</param>
        public void find_best_n_patches_inside_region(
                          classimage_mono image,
                          uint n,
                          uint[] ubest, uint[] vbest,
                          float[] evbest,
                          uint BOXSIZE,
                          uint ustart, uint vstart,
                          uint ufinish, uint vfinish)
        {
            // Check that these limits aren't too close to the image edges.
            // Note that we can't use the edge pixels because we can't form 
            // gradients here.
            if (ustart < (BOXSIZE - 1) / 2 + 1)
                ustart = (BOXSIZE - 1) / 2 + 1;
            if (ufinish > image.width - (BOXSIZE - 1) / 2 - 1)
                ufinish = (uint)(image.width - (BOXSIZE - 1) / 2 - 1);
            if (vstart < (BOXSIZE - 1) / 2 + 1)
                vstart = (BOXSIZE - 1) / 2 + 1;
            if (vfinish > image.height - (BOXSIZE - 1) / 2 - 1)
                vfinish = (uint)(image.height - (BOXSIZE - 1) / 2 - 1);

            // Calculate the width we need to find gradients in. 
            uint calc_width = ufinish - ustart + BOXSIZE - 1;

            // Arrays which store column sums of height BOXSIZE 
            float[] CSgxsq = new float[calc_width];
            float[] CSgysq = new float[calc_width];
            float[] CSgxgy = new float[calc_width];

            // For the final sums at each position (u, v) 
            float TSgxsq = 0.0f, TSgysq = 0.0f, TSgxgy = 0.0f;

            float gx, gy;
            uint u = ustart, v = vstart;
            float eval1 = 0, eval2 = 0;

            // Initial stage: fill these sums for the first horizontal position 
            uint cstart = ustart - (BOXSIZE - 1) / 2;
            uint cfinish = ufinish + (BOXSIZE - 1) / 2;
            uint rstart = vstart - (BOXSIZE - 1) / 2;
            uint i;
            uint c, r;
            for (c = cstart, i = 0; c < cfinish; c++, i++)
            {
                CSgxsq[i] = 0;
                CSgysq[i] = 0;
                CSgxgy[i] = 0;
                for (r = rstart; r < rstart + BOXSIZE; r++)
                {
                    gx = (image.image[c + 1, r] - image.image[c - 1, r]) / 2.0f;
                    gy = (image.image[c, r + 1] - image.image[c, r - 1]) / 2.0f;

                    CSgxsq[i] += gx * gx;
                    CSgysq[i] += gy * gy;
                    CSgxgy[i] += gx * gy;
                }
            }

            float[,] evarray = new float[vfinish - vstart, ufinish - ustart];

            // Now loop through u and v to form sums 
            for (v = vstart; v < vfinish; v++)
            {
                u = ustart;

                // Form first sums for u = ustart 
                TSgxsq = 0.0f;
                TSgysq = 0.0f;
                TSgxgy = 0.0f;
                for (i = 0; i < BOXSIZE; i++)
                {
                    TSgxsq += CSgxsq[i];
                    TSgysq += CSgysq[i];
                    TSgxgy += CSgxgy[i];
                }

                for (u = ustart; u < ufinish; u++)
                {
                    if (u != ustart)
                    {
                        /* Subtract old column, add new one */
                        TSgxsq += CSgxsq[u - ustart + BOXSIZE - 1] - CSgxsq[u - ustart - 1];
                        TSgysq += CSgysq[u - ustart + BOXSIZE - 1] - CSgysq[u - ustart - 1];
                        TSgxgy += CSgxgy[u - ustart + BOXSIZE - 1] - CSgxgy[u - ustart - 1];
                    }

                    find_eigenvalues(TSgxsq, TSgxgy, TSgysq, ref eval1, ref eval2);

                    // eval2 will be the smaller eigenvalue. Store in the array 
                    evarray[v - vstart, u - ustart] = eval2;
                }

                if (v != vfinish - 1)
                {
                    // Update the column sums for the next v 
                    for (c = cstart, i = 0; c < cfinish; c++, i++)
                    {
                        // Subtract the old top pixel
                        gx = (image.image[c + 1, v - (BOXSIZE - 1) / 2]
                             - image.image[c - 1, v - (BOXSIZE - 1) / 2]) / 2.0f;
                        gy = (image.image[c, v - (BOXSIZE - 1) / 2 + 1]
                             - image.image[c, v - (BOXSIZE - 1) / 2 - 1]) / 2.0f;
                        CSgxsq[i] -= gx * gx;
                        CSgysq[i] -= gy * gy;
                        CSgxgy[i] -= gx * gy;

                        // Add the new bottom pixel
                        gx = (image.image[c + 1, v + (BOXSIZE - 1) / 2 + 1]
                             - image.image[c - 1, v + (BOXSIZE - 1) / 2 + 1]) / 2.0f;
                        gy = (image.image[c, v + (BOXSIZE - 1) / 2 + 1 + 1]
                             - image.image[c, v + (BOXSIZE - 1) / 2 + 1 - 1]) / 2.0f;
                        CSgxsq[i] += gx * gx;
                        CSgysq[i] += gy * gy;
                        CSgxgy[i] += gx * gy;
                    }
                }
            }

            // Now: work out the best n patches which don't overlap each other 
            float best_so_far;
            int xdist, ydist;
            bool OKflag;
            float next_highest = 1000000000000.0f;
            for (i = 0; i < n; i++)
            {
                best_so_far = 0.0f;

                for (uint y = 0; y < vfinish - vstart; y++)
                    for (uint x = 0; x < ufinish - ustart; x++)
                    {

                        if (evarray[y, x] > best_so_far && evarray[y, x] < next_highest)
                        {
                            // We've found a high one: check it doesn't overlap with higher ones

                            OKflag = true;
                            for (uint j = 0; j < i; j++)
                            {
                                xdist = (int)(x + ustart - ubest[j]);
                                ydist = (int)(y + vstart - vbest[j]);
                                xdist = (xdist >= 0 ? xdist : -xdist);
                                ydist = (ydist >= 0 ? ydist : -ydist);
                                if ((xdist < (int)BOXSIZE) && (ydist < (int)BOXSIZE))
                                {
                                    OKflag = false;
                                    break;
                                }
                            }
                            if (OKflag)
                            {
                                ubest[i] = x + ustart;
                                vbest[i] = y + vstart;
                                evbest[i] = evarray[y, x];
                                best_so_far = evarray[y, x];
                            }

                        }

                    }
                next_highest = evbest[i];
            }
        }

        private static float search_probability(float distance)
        {
            float prob = 0;

            prob = ((float)Math.Cos(Math.PI * distance) + 1) / 2;
            return (prob*1000);
        }

        /// <summary>
        /// Do a search for patch in image within an elliptical region. The
        /// search region is parameterised an inverse covariance matrix (a distance of
        /// NO_SIGMA is used). The co-ordinates returned are those of centre of the patch.
        /// </summary>
        /// <param name="image">The image to search</param>
        /// <param name="patch">The patch  to search for</param>
        /// <param name="centre">The centre of the search ellipse</param>
        /// <param name="PuInv">The inverse covariance matrix to use</param>
        /// <param name="u">The x-co-ordinate of the best patch location</param>
        /// <param name="v">The y-co-ordinate of the best patch location</param>
        /// <param name="uBOXSIZE">The size of the image patch (TODO: Shouldn't this be the same as the size of patch?)</param>
        /// <returns>true if the a good match is found (above CORRTHRESH2), false otherwise</returns>
        public static bool elliptical_search(
                          classimage_mono image,
                          classimage_mono patch,
                          Vector centre,                    //1 dimensional x,y coords
                          MatrixFixed PuInv,                //2x2 matrix
                          ref uint u, ref uint v,
                          Vector vz,
                          uint uBOXSIZE,
                          classimage_mono outputimage,
                          bool show_ellipses,
                          bool calibrating,
                          Random rnd)
        {
            float aspect;
            if ((vz[1] != 0) && (vz[0] != 0))
            {
                aspect = Math.Abs(vz[0] / vz[1]);
                if (aspect < 0.5) aspect = 0.5f;
                if (aspect > 1.5) aspect = 1.5f;
            }
            else
                aspect = 1;
            int vz_x = 0; // (int)(vz[0] / 2);
            int vz_y = 0; // (int)(vz[1] / 2);

            // We want to pass BOXSIZE as an unsigned int since it is,
            // but if we use it in the if statements below then C++ casts the
            // whole calculation to unsigned ints, so it is never < 0!
            // Force it to get the right answer by using an int version of BOXSIZE
            int BOXSIZE = (int)uBOXSIZE;

            // The dimensions of the bounding box of the ellipse we want to search in 
            uint halfwidth = (uint) (NO_SIGMA * aspect / 
                Math.Sqrt( PuInv[0, 0] - PuInv[0, 1] * PuInv[0, 1] / PuInv[1, 1] ));

            uint halfheight = (uint) (NO_SIGMA / aspect / 
                Math.Sqrt( PuInv[1, 1] - PuInv[0, 1] * PuInv[0, 1] / PuInv[0, 0] ));

            // stop the search ellipse from expanding to large sizes!
            uint MAX_SEARCH_RADIUS = uBOXSIZE * 2;
            if (halfwidth > MAX_SEARCH_RADIUS) halfwidth = MAX_SEARCH_RADIUS;
            if (halfheight > MAX_SEARCH_RADIUS) halfheight = MAX_SEARCH_RADIUS;
            // don't allow the ellipse to get too small
            uint MIN_SEARCH_RADIUS = uBOXSIZE / 2;
            if (halfwidth < MIN_SEARCH_RADIUS) halfwidth = MIN_SEARCH_RADIUS;
            if (halfheight < MIN_SEARCH_RADIUS) halfheight = MIN_SEARCH_RADIUS;

            int ucentre = (int)(centre[0] + 0.5);
            int vcentre = (int)(centre[1] + 0.5);

            // Limits of search 
            int urelstart = -(int)(halfwidth);
            int urelfinish = (int)(halfwidth);
            int vrelstart = -(int)(halfheight);
            int vrelfinish = (int)(halfheight);

            // Check these limits aren't outside the image 
            if (ucentre + urelstart + vz_x - (BOXSIZE - 1) / 2 < 0)
                urelstart = (BOXSIZE - 1) / 2 - ucentre - vz_x;
            if (ucentre + urelfinish + vz_x - (BOXSIZE - 1) / 2 > (int)(image.width) - BOXSIZE)
                urelfinish = image.width - BOXSIZE - ucentre - vz_x + (BOXSIZE - 1) / 2;
            if (vcentre + vrelstart + vz_y - (BOXSIZE - 1) / 2 < 0)
                vrelstart = (BOXSIZE - 1) / 2 - vcentre - vz_y;
            if (vcentre + vrelfinish + vz_y - (BOXSIZE - 1) / 2 > (int)(image.height) - BOXSIZE)
                vrelfinish = (int)(image.height) - BOXSIZE - vcentre - vz_y + (BOXSIZE - 1) / 2;

            // Search counters 
            int urel, vrel;

            float corrmax = 1000000.0f;
            float corr;

            // For passing to and_correlate2_warning
            float sdpatch=0, sdimage=0;

            // Do the search 
            float max_dist = MAX_SEARCH_RADIUS;
            float v1 = PuInv[0, 0];
            float v2 = PuInv[0, 1];
            float v3 = PuInv[1, 1];
            float urelsq, vrelsq;
            bool inside_ellipse;
            int xx, yy, xx2, yy2;
            for (urel = urelstart; urel <= urelfinish; urel+=1)
            {
                urelsq = urel * urel;
                float urel2 = v1 * urelsq;
                inside_ellipse = false;
                for (vrel = vrelstart; vrel <= vrelfinish; vrel+=1)
                {
                        vrelsq = vrel * vrel;
                        if (urel2
                            + 2 * v2 * urel * vrel
                            + v3 * vrelsq < NO_SIGMA * NO_SIGMA)
                        {
                            if ((show_ellipses) || (calibrating))
                            {
                                if (!inside_ellipse)
                                {
                                    xx = ucentre + urel + vz_x - (BOXSIZE - 1) / 2;
                                    yy = vcentre + vrel + vz_y - (BOXSIZE - 1) / 2;
                                    xx2 = xx * outputimage.width / image.width;
                                    yy2 = yy * outputimage.height / image.height;
                                    outputimage.image[xx2, yy2] = (Byte)255;
                                }
                                inside_ellipse = true;
                            }


                            // searching within the ellipse is probablistic,
                            // using something like a gaussian distribution
                            float dist = (float)Math.Sqrt(urelsq + vrelsq) / max_dist;
                            if (rnd.Next(1000) < search_probability(dist))
                            {

                                int offset_x = ucentre + urel + vz_x;
                                int offset_y = vcentre + vrel + vz_y;

                                corr = correlate2_warning(0, 0, BOXSIZE, BOXSIZE,
                                    offset_x - (BOXSIZE - 1) / 2,
                                    offset_y - (BOXSIZE - 1) / 2,
                                    ref patch, ref image, ref sdpatch, ref sdimage);

                                if (corr <= corrmax)
                                {
                                    if (sdpatch < CORRELATION_SIGMA_THRESHOLD)
                                    {
                                        // cout << "Low patch sigma." << endl;
                                    }
                                    else
                                        if (sdimage < CORRELATION_SIGMA_THRESHOLD)
                                        {
                                            // cout << "Low image sigma." << endl;
                                        }
                                        else
                                        {
                                            corrmax = corr;
                                            u = (uint)offset_x;
                                            v = (uint)offset_y;
                                        }
                                }


                                //show ellipses in the output image
                                /*
                                if ((show_ellipses) || (calibrating))
                                {
                                    int xx = offset_x;
                                    int yy = offset_y;
                                    int xx2 = xx * outputimage.width / image.width;
                                    int yy2 = yy * outputimage.height / image.height;
                                    
                                    if (!calibrating)
                                    {
                                        outputimage.image[xx2, yy2, 2] = (Byte)255;
                                    }
                                    else
                                    {
                                        outputimage.image[xx2, yy2, 0] = (Byte)255;
                                        outputimage.image[xx2, yy2, 1] = (Byte)255;
                                        outputimage.image[xx2, yy2, 2] = 0;
                                    }
                                }
                                */
                            }

                        }
                        else
                        {
                            if ((show_ellipses) || (calibrating))
                            {
                                if (inside_ellipse)
                                {
                                    xx = ucentre + urel + vz_x; 
                                    yy = vcentre + vrel + vz_y; 
                                    xx2 = xx * outputimage.width / image.width;
                                    yy2 = yy * outputimage.height / image.height;
                                    outputimage.image[xx2, yy2] = (Byte)255;
                                }
                                inside_ellipse = false;
                            }
                        }
                }
            }

            

            if (corrmax > CORRTHRESH2)
            {
                return false;
            }

            return true;
        }


        #region "stuff from control_general"

        /// <summary>
        /// Make measurements of all the currently-selected features. Features can be
        /// selected using Scene_Single::auto_select_n_features(), or manually using
        /// Scene_Single::select_feature(). This calls
        /// Scene_Single::starting_measurements() and then Sim_Or_Rob::measure_feature()
        /// for each selected feature. Each feature for which a measurement
        /// attempt is made has its Feature::attempted_measurements_of_feature and
        /// Feature::successful_measurements_of_feature counts updated.
        /// </summary>
        /// <param name="scene">The SLAM map to use</param>
        /// <param name="sim_or_rob">The class to use for measuring features.</param>
        /// <returns>The number of features successfully measured.</returns>
        public static int make_measurements(Scene_Single scene, Sim_Or_Rob sim_or_rob, Random rnd)
        {
            int count = 0;
            if (scene.get_no_selected() == 0)
            {
                Debug.WriteLine("No features selected.");
                return 0;
            }

            scene.starting_measurements();
            
            Feature it;
            Vector z;    //best position for the feature within the image
            for (int i = 0; i < scene.selected_feature_list.Count; i++)
            {
                it = (Feature)scene.selected_feature_list[i];

                z = it.get_z_noconst();                
                if (sim_or_rob.measure_feature(it.get_identifier(), ref z, it.get_vz(), it.get_h(), it.get_S(), rnd) == false)
                {
                    // couldn't locate the feature
                    scene.failed_measurement_of_feature(it);
                    it.update_velocity(false);
                }
                else
                {
                    // the feature was found!
                    scene.successful_measurement_of_feature(it);
                    it.update_velocity(true);
                    count++;
                }
            }             

            return count;
        }


        /**************************Initialise Known Features**************************/

        /// <summary>
        /// Initialise the Scene_Single class with some known features, read from the
        /// Settings. Each known feature has its own section, starting with
        /// <code>[KnownFeature1]</code> and counting upwards. The feature type is
        /// identified with the entry <code>FeatureMeasurementModel=</code>. Further
        /// settings are loaded by the feature measurement model itself.
        /// </summary>
        /// <param name="model_creator"></param>
        /// <param name="sim_or_rob"></param>
        /// <param name="scene"></param>
        /// <returns></returns>
        public static uint initialise_known_features(Settings settings,
                                              Feature_Measurement_Model_Creator model_creator,
                                              Sim_Or_Rob sim_or_rob,
                                              Scene_Single scene,
                                              String path,
                                              float MAXIMUM_ANGLE_DIFFERENCE)
        {
            uint feature_no = 1;
            uint num_features = 0;
            Settings.Section section = null;

            do
            {
                // Step through the section names
                String section_name = "KnownFeature" + Convert.ToString(feature_no);
                section = settings.get_section(section_name);
                // Does this section exist?
                if (section == null)
                {
                    return num_features;
                }

                ArrayList values = section.get_entry("FeatureMeasurementModel");
                if (values == null)
                {
                    Debug.WriteLine("No FeatureMeasurementModel entry under the section [" +
                                    section_name + "] in initalisation file.");
                }
                else
                {
                    String type = (String)values[0];
                    Feature_Measurement_Model f_m_m =
                        model_creator.create_model(type, scene.get_motion_model(), MAXIMUM_ANGLE_DIFFERENCE);
                    if (f_m_m == null)
                    {
                        Debug.WriteLine("Unable to create a feature measurement model of type " +
                                        type + " as requested in initalisation file.");
                    }
                    else
                    {
                        // Initialise the feature measurement model with any settings
                        f_m_m.read_parameters(settings);
                        // Read the feature state
                        Vector yi = new Vector(3);
                        Vector xp_orig = new Vector(7);
                        f_m_m.read_initial_state(section, yi, xp_orig);

                        // Initialise the feature
                        classimage_mono identifier =
                            sim_or_rob.initialise_known_feature(f_m_m, yi, section, path);
                        if (identifier == null)
                        {
                            Debug.WriteLine("Trouble reading known feature " +
                                            section_name + " : skipping.");
                        }
                        else
                        {
                            scene.add_new_known_feature(identifier, yi, xp_orig, f_m_m, feature_no);
                            Debug.WriteLine("Added known feature " + Convert.ToString(feature_no));
                            num_features++;
                        }
                    }
                }
                feature_no++;
            }
            while (section != null);

            return num_features;
        }


        #endregion

        #region "stuff from determinant.h"

        public static float Determinant(float[] row0, float[] row1) 
        {        
            return ((row0[0]*row1[1]) - (row0[1]*row1[0]));
        }

        public static float Determinant(float[] row0, float[] row1, float[] row2) 
        {
            return // the extra '+' makes it work nicely with emacs indentation.
                   + row0[0]*row1[1]*row2[2]
                   - row0[0]*row2[1]*row1[2]
                   - row1[0]*row0[1]*row2[2]
                   + row1[0]*row2[1]*row0[2]
                   + row2[0]*row0[1]*row1[2]
                   - row2[0]*row1[1]*row0[2];
        }

        public static float Determinant(float[] row0, float[] row1, float[] row2, float[] row3) 
        {
            return (+ row0[0]*row1[1]*row2[2]*row3[3]
                    - row0[0]*row1[1]*row3[2]*row2[3]
                    - row0[0]*row2[1]*row1[2]*row3[3]
                    + row0[0]*row2[1]*row3[2]*row1[3]
                    + row0[0]*row3[1]*row1[2]*row2[3]
                    - row0[0]*row3[1]*row2[2]*row1[3]
                    - row1[0]*row0[1]*row2[2]*row3[3]
                    + row1[0]*row0[1]*row3[2]*row2[3]
                    + row1[0]*row2[1]*row0[2]*row3[3]
                    - row1[0]*row2[1]*row3[2]*row0[3]
                    - row1[0]*row3[1]*row0[2]*row2[3]
                    + row1[0]*row3[1]*row2[2]*row0[3]
                    + row2[0]*row0[1]*row1[2]*row3[3]
                    - row2[0]*row0[1]*row3[2]*row1[3]
                    - row2[0]*row1[1]*row0[2]*row3[3]
                    + row2[0]*row1[1]*row3[2]*row0[3]
                    + row2[0]*row3[1]*row0[2]*row1[3]
                    - row2[0]*row3[1]*row1[2]*row0[3]
                    - row3[0]*row0[1]*row1[2]*row2[3]
                    + row3[0]*row0[1]*row2[2]*row1[3]
                    + row3[0]*row1[1]*row0[2]*row2[3]
                    - row3[0]*row1[1]*row2[2]*row0[3]
                    - row3[0]*row2[1]*row0[2]*row1[3]
                    + row3[0]*row2[1]*row1[2]*row0[3]);
        }


        public static float Determinant(MatrixFixed M)
        {
            return(Determinant(M, false));
        }


        public static float Determinant(MatrixFixed M, bool balance)
        {
            int n = M.Rows;
            //assert(M.Columns() == n);

            switch (n)
            {
                case 1: return M[0, 0];
                case 2: return Determinant(M[0], M[1]);
                case 3: return Determinant(M[0], M[1], M[2]);
                case 4: return Determinant(M[0], M[1], M[2], M[3]);
                default:
                    if (balance)
                    {
                        MatrixFixed tmp = new MatrixFixed(M);
                        float scalings = 1;
                        for (int t = 0; t < 5; ++t)
                        {
                            float rn;
                            // normalize rows.
                            for (uint i = 0; i < n; ++i)
                            {
                                rn = tmp.GetRow((int)i).RMS();
                                if (rn > 0)
                                {
                                    scalings *= rn;
                                    tmp.ScaleRow((int)i, 1.0f / rn);
                                }
                            }
                            // normalize columns.
                            for (uint i = 0; i < n; ++i)
                            {
                                rn = tmp.GetColumn((int)i).RMS();
                                if (rn > 0)
                                {
                                    scalings *= rn;
                                    tmp.ScaleColumn((int)i, 1.0f / rn);
                                }
                            }
                            /*
                            #if 0
                                    // pivot
                                    for (int k=0; k<n-1; ++k) {
                                      // find largest element after (k, k):
                                      int i0 = k, j0 = k;
                                      abs_t v0(0);
                                      for (int i=k; i<n; ++i) {
                                        for (int j=k; j<n; ++j) {
                                          abs_t v = std::abs(tmp[i][j]);
                                          if (v > v0) {
                                            i0 = i;
                                            j0 = j;
                                            v0 = v;
                                          }
                                        }
                                      }
                                      // largest element is in position (i0, j0).
                                      if (i0 != k) {
                                        for (int j=0; j<n; ++j)
                                          std::swap(tmp[k][j], tmp[i0][j]);
                                        scalings = -scalings;
                                      }
                                      if (j0 != k) {
                                        for (int i=0; i<n; ++i)
                                          std::swap(tmp[i][k], tmp[i][j0]);
                                        scalings = -scalings;
                                      }
                                    }
                            #endif
                            */

                        }
                        float balanced_det = (new QR(tmp)).Determinant();
                        //std::clog << __FILE__ ": scalings, balanced_det = " << scalings << ", " << balanced_det << std::endl;
                        return (float)(scalings) * balanced_det;
                    }
                    else
                        return (new QR(M)).Determinant();
            }
        }

        #endregion

    }
}
