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



namespace sentience
{
    public class classimage_mono
    {
        public long ID = 0;

        //dimensions of the image
        public int width, height;

        //used as a divisor to subsample the image
        public int reductionFactor_x, reductionFactor_y;

        //average intensity over the whole image
        public int averageIntensity;

        public Byte[,] image;
        long[,] Integral;

        Random rnd = new Random();

        //feature detection stuff
        public int OptimalNoOfFeatures;

        //---------------------------------------------------------------------------------------------
        // constructor
        //---------------------------------------------------------------------------------------------
        public classimage_mono()
        {
            init();
        }


        public void init()
        {
            reductionFactor_x = 1;
            reductionFactor_y = 1;
            image = null;
            width = 0;
            height = 0;
            OptimalNoOfFeatures = 0;
        }


        //---------------------------------------------------------------------------------------------
        // create a new image
        //---------------------------------------------------------------------------------------------
        public void createImage(int wdth, int hght)
        {
            width = wdth;
            height = hght;

            image = new Byte[width, height];
            Integral = new long[width, height];
        }


        //---------------------------------------------------------------------------------------------
        // clear the image
        //---------------------------------------------------------------------------------------------
        public void clear()
        {
            int x, y;

            for (x = 0; x < width; x++)
                for (y = 0; y < height; y++)
                    image[x, y] = 0;
        }


        //---------------------------------------------------------------------------------------------
        // clear an area of the image
        //---------------------------------------------------------------------------------------------
        public void clearArea(int tx, int ty, int bx, int by)
        {
            int x, y;

            for (x = tx; x <= bx; x++)
                for (y = ty; y <= by; y++)
                    image[x, y] = 0;
        }



        //---------------------------------------------------------------------------------------------
        // update the integral image
        //---------------------------------------------------------------------------------------------
        public void updateIntegralImage()
        {
            int x, y;
            long p;

            for (y = 0; y < height; y++)
            {
                p = 0;

                for (x = 0; x < width; x++)
                {
                    p += image[x, y];

                    if (y > 0)
                        Integral[x, y] = p + Integral[x, y - 1];
                    else
                        Integral[x, y] = p;
                }
            }
        }


        //------------------------------------------------------------------------------------------------------------------------
        //  get the total pixel value for the given area
        //------------------------------------------------------------------------------------------------------------------------
        public long getIntegral(int tx, int ty, int bx, int by)
        {
            return (Integral[bx, by] + Integral[tx, ty] - (Integral[tx, by] + Integral[bx, ty]));
        }




        //------------------------------------------------------------------------------------------------------------------------
        //  sample the given image within the given bounding box
        //------------------------------------------------------------------------------------------------------------------------
        public void sampleFromImage(classimage example_img, int tx, int ty, int bx, int by)
        {
            int x, y, xx, yy, dx, dy;

            dx = bx - tx;
            dy = by - ty;

            for (x = 0; x < width; x++)
            {
                xx = tx + ((x * dx) / width);

                for (y = 0; y < height; y++)
                {
                    yy = ty + ((y * dy) / height);

                    image[x, y] = example_img.image[xx, yy, 0];
                }
            }
            //updateIntegralImage();
        }

        //------------------------------------------------------------------------------------------------------------------------
        //  sample the given image within the given bounding box
        //------------------------------------------------------------------------------------------------------------------------
        public void sampleFromImage(classimage_mono example_img, int tx, int ty, int bx, int by)
        {
            int x, y, xx, yy, dx, dy;

            dx = bx - tx;
            dy = by - ty;

            for (x = 0; x < width; x++)
            {
                xx = tx + ((x * dx) / width);

                for (y = 0; y < height; y++)
                {
                    yy = ty + ((y * dy) / height);

                    image[x, y] = example_img.image[xx, yy];
                }
            }
            //updateIntegralImage();
        }



        //------------------------------------------------------------------------------------------------------------------------
        //  returns the average intensity within the given region
        //------------------------------------------------------------------------------------------------------------------------
        public int getAverageIntensity(int tx, int ty, int bx, int by)
        {
            int x, y;
            long pixels, i;

            pixels = 0;
            i = 0;
            for (x = tx; x < bx; x++)
            {
                if ((x > -1) && (x < width))
                {
                    for (y = ty; y < by; y++)
                    {
                        if ((y > -1) && (y < height))
                        {
                            i += image[x, y];
                            pixels ++;
                        }
                    }
                }
            }

            if (pixels > 0)
                return ((int)(i / pixels));
            else
                return (0);
        }



        //------------------------------------------------------------------------------------------------------------------------
        //  get image from a bitmap
        //------------------------------------------------------------------------------------------------------------------------
        public void updateFromBitmap(Byte[] bmp, int RGBformat, int wdth, int hght)
        {
            int x, y, xx, yy;
            Byte r, g, b;
            int p;

            //create a new image array if necessary
            if (width == 0) createImage(wdth / reductionFactor_x, hght / reductionFactor_y);

            //populate the image array from the bitmap
            p = 0;
            for (y = 0; y < hght; y++)
            {
                for (x = 0; x < wdth; x++)
                {
                    if (RGBformat == 0)  //pixels in RGB order
                    {
                        r = (Byte)bmp[p]; p++;
                        g = (Byte)bmp[p]; p++;
                        b = (Byte)bmp[p]; p++;
                    }
                    else
                    { //pixels in BGR order
                        b = (Byte)bmp[p]; p++;
                        g = (Byte)bmp[p]; p++;
                        r = (Byte)bmp[p]; p++;
                    }

                    if ((reductionFactor_x==1) && (reductionFactor_y==1))
                    {
                        image[x, y] = (Byte)((r + g + b) / 3);
                    }
                    else
                    {
                        xx = x / reductionFactor_x;
                        yy = y / reductionFactor_y;
                        image[xx, yy] = (Byte)((r + g + b) / 3);
                    }
                }
            }

            //updateIntegralImage();
        }

        /// <summary>
        /// write the image to file in RAW format
        /// </summary>
        /// <param name="filename"></param>
        public void saveToRAW(String filename)
        {
            FileStream fp = new FileStream(filename, FileMode.Create);
            BinaryWriter binfile = new BinaryWriter(fp);

            binfile.Write(width);
            binfile.Write(height);
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    binfile.Write(image[x, y]);
                }
            }

            binfile.Close();
            fp.Close();
        }


        /// <summary>
        /// load the image from file in RAW format
        /// </summary>
        /// <param name="filename"></param>
        public bool loadFromRAW(String filename)
        {
            if (File.Exists(filename))
            {
                FileStream fp = new FileStream(filename, FileMode.Open);
                BinaryReader binfile = new BinaryReader(fp);

                int wdth = binfile.ReadInt32();
                int hght = binfile.ReadInt32();
                if ((wdth != width) || (hght != height)) createImage(wdth, hght);
                for (int y = 0; y < height; y++)
                {
                    for (int x = 0; x < width; x++)
                    {
                        image[x, y] = binfile.ReadByte();
                    }
                }

                binfile.Close();
                fp.Close();
                return (true);
            }
            else return (false);
        }


        /// <summary>
        /// load data from a mono bitmap
        /// </summary>
        /// <param name="filename"></param>
        /// <param name="wdth"></param>
        /// <param name="hght"></param>
        /// <returns></returns>
        public bool loadFromBitmapMono(String filename, int wdth, int hght)
        {
            if (File.Exists(filename))
            {
                FileStream fp = new FileStream(filename, FileMode.Open);
                BinaryReader binfile = new BinaryReader(fp);

                wdth = binfile.ReadInt32();
                hght = binfile.ReadInt32();

                if ((wdth != width) || (hght != height)) createImage(wdth, hght);

                int n = 0;
                for (int y = 0; y < height; y++)
                    for (int x = 0; x < width; x++)
                    {
                        image[x, y] = binfile.ReadByte();
                        n++;
                    }
                binfile.Close();
                fp.Close();
                return true;
            }
            else return false;
        }


        /// <summary>
        /// save the image as a mono bitmap
        /// </summary>
        /// <param name="filename"></param>
        /// <returns></returns>
        public bool SaveAsBitmapMono(String filename)
        {
            FileStream fp = new FileStream(filename, FileMode.Create);
            BinaryWriter binfile = new BinaryWriter(fp);

            binfile.Write(width);
            binfile.Write(height);

            int n = 0;
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                {
                    binfile.Write(image[x, y]);
                    n++;
                }
            binfile.Close();
            fp.Close();
            return true;
        }


        /// <summary>
        /// load image from a PGM file
        /// </summary>
        /// <param name="filename"></param>
        /// <returns></returns>
        public bool loadFromPGM(String filename)
        {
            if (File.Exists(filename))
            {
                StreamReader stream = new StreamReader(filename, Encoding.ASCII);

                String magic_number = stream.ReadLine();
                if (magic_number == "P5")
                {
                    String line = stream.ReadLine();
                    String[] values = line.Split(' ');
                    int wdth = Convert.ToInt32(values[0]);
                    int hght = Convert.ToInt32(values[1]);
                    int max_value = Convert.ToInt32(stream.ReadLine());
                    if ((wdth != width) || (hght != height)) createImage(wdth, hght);
                    Char[] image_data = new Char[wdth * hght];
                    stream.Read(image_data, 0, wdth * hght);

                    //note: these bytes need to be fed through a tedious gamma transfer function

                    int n = 0;
                    for (int y = 0; y < height; y++)
                    {
                        for (int x = 0; x < width; x++)
                        {
                            image[x, y] = Convert.ToByte(image_data[n]);
                            n++;
                        }
                    }
                }

                stream.Close();
                return (true);
            }
            else return (false);
        }

        /// <summary>
        /// save image to a PGM file
        /// </summary>
        /// <param name="filename"></param>
        public void saveAsPGM(String filename)
        {
            StreamWriter stream = new StreamWriter(filename, false, Encoding.ASCII);

            stream.WriteLine("P5");
            stream.WriteLine(Convert.ToString(width) + " " + Convert.ToString(height));
            stream.WriteLine("255");
            Char[] image_data = new Char[width * height];
            int n = 0;
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    image_data[n] = (Char)image[x, y];
                    n++;
                }
            }
            stream.Write(image_data);
            stream.Close();
        }


        //------------------------------------------------------------------------------------------------------------------------
        //  save the image to a bitmap
        //------------------------------------------------------------------------------------------------------------------------
        public void saveToBitmap(Byte[] bmp, int wdth, int hght)
        {
            int x, y, xx, yy;
            Byte v;
            int p;
            int factor;

            //create a new image array if necessary
            if (width == 0) createImage(wdth / reductionFactor_x, hght / reductionFactor_y);

            factor = wdth / width;

            p = 0;
            for (y = 0; y < hght; y++)
            {
                for (x = 0; x < wdth; x++)
                {
                    if (factor == 1)
                    {
                        xx = x;
                        yy = y;
                    }
                    else
                    {
                        xx = x * factor;
                        yy = y * factor;
                        if (xx > width - 1) xx = width - 1;
                        if (yy > height - 1) yy = height - 1;
                    }

                    v = image[xx, yy];

                    bmp[p] = v; p++;
                    bmp[p] = v; p++;
                    bmp[p] = v; p++;
                }
            }
        }


        //------------------------------------------------------------------------------------------------------------------------
        //random number generator
        //------------------------------------------------------------------------------------------------------------------------
        public float Rnd()
        {
            return (rnd.Next(10000) / 10000.0f);
        }


        //------------------------------------------------------------------------------------------------------------------------
        //update from another image
        //------------------------------------------------------------------------------------------------------------------------
        public void updateFromImage(classimage img)
        {
            int x, y, xx, yy;

            for (x = 0; x < width; x++)
            {
                xx = (x * img.width) / width;
                for (y = 0; y < height; y++)
                {
                    yy = (y * img.height) / height;
                    image[x, y] = img.image[xx, yy, 0];
                }
            }
        }

        //------------------------------------------------------------------------------------------------------------------------
        //update from another image
        //------------------------------------------------------------------------------------------------------------------------
        public void updateFromImage(classimage_mono img)
        {
            int x, y, xx, yy;

            for (x = 0; x < width; x++)
            {
                xx = (x * img.width) / width;
                for (y = 0; y < height; y++)
                {
                    yy = (y * img.height) / height;
                    image[x, y] = img.image[xx, yy];
                }
            }
        }


        /// <summary>
        /// draws a white box in a simple way
        /// </summary>
        /// <param name="tx"></param>
        /// <param name="ty"></param>
        /// <param name="bx"></param>
        /// <param name="by"></param>
        public void DrawBox(int tx, int ty, int bx, int by)
        {
            if (bx >= width) bx = width - 1;
            if (by >= height) by = height - 1;

            for (int x = tx; x < bx; x++)
            {
                image[x, ty] = 255;

                image[x, by] = 255;
            }

            for (int y = ty; y < by; y++)
            {
                image[tx, y] = 255;

                image[bx, y] = 255;
            }
        }


        //------------------------------------------------------------------------------------------------------------------------
        //draws a white box
        //------------------------------------------------------------------------------------------------------------------------
        public void DrawBox(int cx, int cy, int boxwidth, int boxheight, int boxrotation, int r, int g, int b, int lineWidth)
        {
            int[] x = new int[4];
            int[] y = new int[4];
            int dist, i;
            float[] angle = new float[4];
            float rot;

            rot = (boxrotation * 3.1415927f) / 180;
            x[0] = boxwidth;
            y[0] = boxheight;
            x[1] = boxwidth;
            y[1] = -boxheight;
            x[2] = -boxwidth;
            y[2] = -boxheight;
            x[3] = -boxwidth;
            y[3] = boxheight;

            dist = (int)Math.Sqrt((boxwidth * boxwidth) + (boxheight * boxheight));
            angle[0] = (float)Math.Acos(y[0] / (float)dist);
            angle[1] = (3.1415927f / 2) + ((3.1415927f / 2) - angle[0]);
            angle[2] = -angle[1];
            angle[3] = -angle[0];

            for (i = 0; i < 4; i++)
            {
                x[i] = cx + (int)(dist * Math.Sin(angle[i] + rot));
                y[i] = cy + (int)(dist * Math.Cos(angle[i] + rot));
                if (i > 0) drawLine(x[i], y[i], x[i - 1], y[i - 1], lineWidth);
                if (i == 3) drawLine(x[0], y[0], x[3], y[3], lineWidth);
            }
        }


        //------------------------------------------------------------------------------------------------------------------------
        //returns the 'centre of gravity' for the given region relative to the given colour
        //------------------------------------------------------------------------------------------------------------------------
        public void CG(int tx, int ty, int bx, int by, Byte targ_r, Byte targ_g, Byte targ_b, ref int cx, ref int cy, int maxval)
        {
            int x, y, c;
            int[] p = new int[3];
            long tot, tot_x, tot_y, dp;
            int min_value = 150;

            tot = 1;
            tot_x = 0;
            tot_y = 0;
            for (x = tx; x < bx; x++)
            {
                if ((x >= 0) && (x < width))
                {
                    for (y = ty; y < by; y++)
                    {
                        if ((y >= 0) && (y < width))
                        {
                            for (c = 0; c < 3; c++) p[c] = image[x, y];
                            dp = Math.Abs(p[0] - (int)targ_r);
                            dp += Math.Abs(p[1] - (int)targ_g);
                            dp += Math.Abs(p[2] - (int)targ_b);
                            if (dp < min_value) dp = 0;
                            dp *= dp;

                            tot += dp;
                            tot_x += x * dp;
                            tot_y += y * dp;
                        }
                    }
                }
            }

            cx = (int)(tot_x / tot);
            cy = (int)(tot_y / tot);
        }




        //---------------------------------------------------------------------------------------------
        //copy from an image
        //---------------------------------------------------------------------------------------------
        public void copyImage(classimage source_img)
        {
            int x, y;
            Byte v;

            for (x = 0; x < width; x++)
            {
                for (y = 0; y < height; y++)
                {
                    v = (Byte)((source_img.image[x, y, 0] + 
                                source_img.image[x, y, 1] + 
                                source_img.image[x, y, 2]) / 3);
                    image[x, y] = v;
                }
            }
        }

        //---------------------------------------------------------------------------------------------
        //copy from an image
        //---------------------------------------------------------------------------------------------
        public void copyImage(classimage_mono source_img)
        {
            int x, y;
            Byte v;

            for (x = 0; x < width; x++)
            {
                for (y = 0; y < height; y++)
                {
                    v = source_img.image[x, y];
                    image[x, y] = v;
                }
            }
        }


        //---------------------------------------------------------------------------------------------
        //draw a line between two points in the given image
        //---------------------------------------------------------------------------------------------
        public void drawLine(int x1, int y1, int x2, int y2, int linewidth)
        {
            int w, h, x, y, step_x, step_y, dx, dy, xx2, yy2;
            float m;

            dx = x2 - x1;
            dy = y2 - y1;
            w = Math.Abs(dx);
            h = Math.Abs(dy);
            if (x2 >= x1) step_x = 1; else step_x = -1;
            if (y2 >= y1) step_y = 1; else step_y = -1;

            if (w > h)
            {
                if (dx != 0)
                {
                    m = dy / (float)dx;
                    x = x1;
                    while (x != x2 + step_x)
                    {
                        y = (int)(m * (x - x1)) + y1;

                        for (xx2 = x - linewidth; xx2 <= x + linewidth; xx2++)
                            for (yy2 = y - linewidth; yy2 <= y + linewidth; yy2++)
                            {
                                if ((xx2 >= 0) && (xx2 < width) && (yy2 >= 0) && (yy2 < height))
                                {
                                    image[xx2, yy2] = (Byte)255;
                                }
                            }

                        x += step_x;
                    }
                }
            }
            else
            {
                if (dy != 0)
                {
                    m = dx / (float)dy;
                    y = y1;
                    while (y != y2 + step_y)
                    {
                        x = (int)(m * (y - y1)) + x1;
                        for (xx2 = x - linewidth; xx2 <= x + linewidth; xx2++)
                            for (yy2 = y - linewidth; yy2 <= y + linewidth; yy2++)
                            {
                                if ((xx2 >= 0) && (xx2 < width) && (yy2 >= 0) && (yy2 < height))
                                {
                                    image[xx2, yy2] = (Byte)255;
                                }
                            }

                        y += step_y;
                    }
                }
            }
        }


    }
}
