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
    public class classimage
    {
        public long ID = 0;

        bool initialised;
        int cx,cy;
	
        //dimensions of the image
        public int width, height;

        //used as a divisor to subsample the image
        public int reductionFactor_x,reductionFactor_y;

        //average intensity over the whole image
        public int averageIntensity;

        //lookup table for polar transformation
        int[,,] polarLookup;
        int currPolarRadius;

        public int imageRotation;
        public Byte[,,] image;
        int[,,] rotationLookup;
        long[,] Integral;

        Random rnd = new Random();

        //attention points within the image
        //int attentionPoint[100][5],prev_attentionPoint[100][5];
        public int[,] attentionPoint;
        int[,] prev_attentionPoint=null;

        public int NoOfAttentionPoints;
        public int selectedAttentionPoint;
        public int attentionWinner;

        //feature detection stuff
        public int OptimalNoOfFeatures;

        //---------------------------------------------------------------------------------------------
        // constructor
        //---------------------------------------------------------------------------------------------
        public classimage()
        {
            init();
        }


        public void init()
        {
            initialised=false;
            reductionFactor_x=1;
            reductionFactor_y=1;
            image=null;
            width=0;
            height=0;
            rotationLookup=null;
            OptimalNoOfFeatures=0;
            currPolarRadius=0;
            polarLookup=null;
        }


        //---------------------------------------------------------------------------------------------
        // create a new image
        //---------------------------------------------------------------------------------------------
        public void createImage(int wdth, int hght)
        {
            width = wdth;
            height = hght;

            image = new Byte [width,height,3];
            Integral = new long [width,height];
            NoOfAttentionPoints=0;
        }


        //---------------------------------------------------------------------------------------------
        // clear the image
        //---------------------------------------------------------------------------------------------
        public void clear()
        {
            int x,y,c;

            for (x=0;x<width;x++)
                for (y=0;y<height;y++)
	                for (c=0;c<3;c++)
		                image[x,y,c]=0;
        }


        //---------------------------------------------------------------------------------------------
        // clear an area of the image
        //---------------------------------------------------------------------------------------------
        public void clearArea(int tx, int ty, int bx, int by)
        {
            int x,y,c;

            for (x=tx;x<=bx;x++)
                for (y=ty;y<=by;y++)
	                for (c=0;c<3;c++)
		                image[x,y,c]=0;
        }



        //---------------------------------------------------------------------------------------------
        // update the integral image
        //---------------------------------------------------------------------------------------------
        public void updateIntegralImageMono()
        {
            int x, y;
            long p;

            for (y = 0; y < height; y++)
            {
                p = 0;

                for (x = 0; x < width; x++)
                {
                    p += image[x, y, 0];
                    //p += image[x, y, 1];
                    //p += image[x, y, 2];

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
            int x,y,xx,yy,dx,dy,c;
  
            dx = bx - tx;
            dy = by - ty;
  
            for (x = 0; x < width; x++)
            {
                xx = tx + ((x * dx) / width);
    
                for (y = 0;y<height;y++)
	            {
                    yy = ty + ((y * dy) / height);
    
                    for (c=0;c<3;c++) image[x,y,c] = example_img.image[xx,yy,c];
                }
            }  
            //updateIntegralImage();
        }



        //------------------------------------------------------------------------------------------------------------------------
        //  returns the average colour within the given region
        //------------------------------------------------------------------------------------------------------------------------
        public void averageColour(int tx, int ty, int bx, int by, ref int av_r, ref int av_g, ref int av_b)
        {
            int x,y,c;
            long r,g,b,pixels,i;
  
            pixels = 0;
            r=0;
            g=0;
            b=0;
            i=0;
            for (x = tx;x<bx;x++)
            {
	            if ((x>-1) && (x<width))
	            {
                    for (y = ty;y<by;y++)
	                {
		                if ((y>-1) && (y<height))
		                {
	                        for (c=0;c<3;c++) i += image[x,y,c];
                            r += image[x,y,0];
                            g += image[x,y,1];
                            b += image[x,y,2];
		                    pixels++;
		                }
	                }
                }
            }
  
            if (pixels>0)
            {
                averageIntensity = (int)(i/(pixels*3));
                av_r = (int)(r/pixels);
                av_g = (int)(g/pixels);
                av_b = (int)(b/pixels);
            }
        }


        //------------------------------------------------------------------------------------------------------------------------
        //  returns the average colour within the given region (ignoring black)
        //------------------------------------------------------------------------------------------------------------------------
        public void averageColourNonBlack(int tx, int ty, int bx, int by, ref int av_r, ref int av_g, ref int av_b)
        {
            int x,y,c;
            long r,g,b,pixels,i;
            int min=20;
  
            pixels = 0;
            r=0;
            g=0;
            b=0;
            i=0;
            for (x = tx;x<bx;x++)
            {
	            if ((x>-1) && (x<width))
	            {
                    for (y = ty;y<by;y++)
	                {
		                if ((y>-1) && (y<height))
		                {
	                        for (c=0;c<3;c++) i += image[x,y,c];
		                    if ((image[x,y,0]>min) || (image[x,y,1]>min) || (image[x,y,2]>min))
		                    {
                                r += image[x,y,0];
                                g += image[x,y,1];
                                b += image[x,y,2];
		                        pixels++;
		                    }
		                }
	                }
                }
            }
  
            if (pixels>0)
            {
                averageIntensity = (int)(i/(pixels*3));
                av_r = (int)(r/pixels);
                av_g = (int)(g/pixels);
                av_b = (int)(b/pixels);
            }
        }



        //------------------------------------------------------------------------------------------------------------------------
        //  returns the average intensity within the given region
        //------------------------------------------------------------------------------------------------------------------------
        public int getAverageIntensity(int tx, int ty, int bx, int by)
        {
            int x,y,c;
            long pixels,i;
  
            pixels = 0;
            i=0;
            for (x = tx; x < bx; x++)
            {
	            if ((x>-1) && (x<width))
	            {
                    for (y = ty;y<by;y++)
	                {
		                if ((y>-1) && (y<height))
		                {
	                        for (c=0;c<3;c++) i += image[x,y,c];
		                    pixels +=3;
		                }
	                }
	            }
            }

            if (pixels>0)
                return((int)(i/pixels));
                else
	            return(0);
        }



        //------------------------------------------------------------------------------------------------------------------------
        //  returns the average intensity within the given region
        //------------------------------------------------------------------------------------------------------------------------
        public int getAverageIntensityThresholded(int tx, int ty, int bx, int by, int threshold, int thresh_type)
        {
            int x,y,c,intensity;
            long pixels,i;
  
            pixels = 0;
            i=0;
            for (x = tx;x<bx;x++)
            {
	            if ((x>-1) && (x<width))
	            {
                    for (y = ty;y<by;y++)
	                {
		                if ((y>-1) && (y<height))
		                {
		                    intensity=0;
		                    for (c=0;c<3;c++) intensity += image[x,y,c];
		                    intensity/=3;
		                    if (((thresh_type==0) && (intensity<threshold)) ||
			                    ((thresh_type==1) && (intensity>threshold)))
		                    {
		                        for (c=0;c<3;c++) i += image[x,y,c];
		                        pixels +=3;
		                    }
		                }
	                }
	            }
            }

            if (pixels>0)
                return((int)(i/pixels));
                else
	            return(0);
        }



        //------------------------------------------------------------------------------------------------------------------------
        //  get image from a bitmap
        //------------------------------------------------------------------------------------------------------------------------
        public void updateFromBitmap(Byte[] bmp, int RGBformat, int wdth, int hght)
        {
            int x,y,xx,yy;
            Byte r,g,b;
            int p;

            //create a new image array if necessary
            if (width==0) createImage(wdth/reductionFactor_x,hght/reductionFactor_y);

            //populate the image array from the bitmap
            p=0;
            for (y=0;y<hght;y++)
            {
                for (x=0;x<wdth;x++)
                {
	                if (RGBformat==0)  //pixels in RGB order
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

	                xx = x/reductionFactor_x;
	                yy = y/reductionFactor_y;

                    image[xx,yy,0] = r;
                    image[xx,yy,1] = g;
                    image[xx,yy,2] = b;
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
                    for (int c = 0;c < 3; c++)
                        binfile.Write(image[x, y, c]);
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
                        for (int c = 0; c < 3; c++)
                            image[x, y, c] = binfile.ReadByte();
                    }
                }

                binfile.Close();
                fp.Close();
                return (true);
            }
            else return (false);
        }


        /// <summary>
        /// write the image to file in mono RAW format
        /// </summary>
        /// <param name="filename"></param>
        public void saveToRAW_Mono(String filename)
        {
            FileStream fp = new FileStream(filename, FileMode.Create);
            BinaryWriter binfile = new BinaryWriter(fp);

            binfile.Write(width);
            binfile.Write(height);
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    int value = ((int)image[x, y, 0] + (int)image[x, y, 1] + (int)image[x, y, 2]) / 3;
                    binfile.Write((Byte)value);
                }
            }

            binfile.Close();
            fp.Close();
        }

        /// <summary>
        /// load the image from file in mono RAW format
        /// </summary>
        /// <param name="filename"></param>
        public bool loadFromRAW_Mono(String filename)
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
                        image[x, y, 0] = binfile.ReadByte();
                        image[x, y, 1] = image[x, y, 0];
                        image[x, y, 2] = image[x, y, 0];
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
                        image[x, y, 0] = binfile.ReadByte();
                        image[x, y, 1] = image[x, y, 0];
                        image[x, y, 2] = image[x, y, 0];
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
                    binfile.Write(image[x, y, 0]);
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
                            image[x, y, 0] = Convert.ToByte(image_data[n]);
                            image[x, y, 1] = image[x, y, 0];
                            image[x, y, 2] = image[x, y, 0];
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
                    image_data[n] = (Char)image[x, y, 0];
                    n++;
                }
            }
            stream.Write(image_data);
            stream.Close();
        }


        //------------------------------------------------------------------------------------------------------------------------
        //  save the image to a bitmap
        //------------------------------------------------------------------------------------------------------------------------
        public void saveToBitmap(Byte[] bmp, int RGBformat, int wdth, int hght)
        {
            int x,y,xx,yy;
            Byte r,g,b;
            int p;
            int factor;

            //create a new image array if necessary
            if (width==0) createImage(wdth/reductionFactor_x,hght/reductionFactor_y);

            factor = wdth/width;

            p=0;
            for (y=0;y<hght;y++)
            {
                for (x=0;x<wdth;x++)
	            {
                    xx = x * factor;
		            yy = y * factor;
		            if (xx>width-1) xx = width-1;
		            if (yy>height-1) yy = height-1;

                    r = image[xx,yy,0];
                    g = image[xx,yy,1];
                    b = image[xx,yy,2];
	  
  	                if (RGBformat==0)  //pixels in RGB order
		            {
	                    bmp[p] = (Byte)r; p++;
	                    bmp[p] = (Byte)g; p++;
	                    bmp[p] = (Byte)b; p++;
		            }
	                else
		            { //pixels in BGR order
	                    bmp[p] = (Byte)b; p++;
	                    bmp[p] = (Byte)g; p++;
	                    bmp[p] = (Byte)r; p++;
		            }
                }
            }
        }


        //------------------------------------------------------------------------------------------------------------------------
        //random number generator
        //------------------------------------------------------------------------------------------------------------------------
        public float Rnd()
        {
	        return(rnd.Next(10000)/10000.0f);
        }


        //------------------------------------------------------------------------------------------------------------------------
        //update from another image
        //------------------------------------------------------------------------------------------------------------------------
        public void updateFromImage(classimage img)
        {
            int x, y, xx, yy, c;

            for (x = 0; x < width; x++)
            {
                xx = (x * img.width) / width;
                for (y = 0; y < height; y++)
                {
                    yy = (y * img.height) / height;
                    for (c = 0; c < 3; c++) image[x, y, c] = img.image[xx, yy, c];
                }
            }
        }



        //------------------------------------------------------------------------------------------------------------------------
        //  polar transform
        //------------------------------------------------------------------------------------------------------------------------
        public void updatePolar(classimage source_img)
        {
            int x,y,xx,yy,x2,y2,c;
            float angle;

            if (!initialised)
            {
                createImage(source_img.width/reductionFactor_x,source_img.height/reductionFactor_y);
                initialised=true;
            }

            if (polarLookup==null)
            {
                //generate the lookup array if necessary
                polarLookup = new int [source_img.width,source_img.height,2];
                for (x=0;x<source_img.width;x++)
	            {
                    for (y=0;y<source_img.height;y++)
	                {
	                    polarLookup[x,y,0]=0;
	                    polarLookup[x,y,1]=0;
	                }
                }
            }

            if (currPolarRadius != source_img.width/2)
            {
	            //re-create the lookup table
                for (x=0;x<source_img.width;x++)
	            {
	                angle = (x * 3.1415927f * 2) / source_img.width;
                    for (y=0;y<source_img.height;y++)
	                {
		                xx = (source_img.width/2) + (int)(y * Math.Sin(angle));
		                yy = (source_img.height/2) + (int)(y * Math.Cos(angle));
		                if ((xx>=0) && (yy>=0) && (xx<source_img.width) && (yy<source_img.height))
		                {
		                    polarLookup[xx,yy,0]=x;
		                    polarLookup[xx,yy,1]=y;
		                }
	                }
	            }

                currPolarRadius = source_img.width/2;
            }
 

            for (x=0;x<width;x++)
            {
                x2 = (x * source_img.width) / width;
                for (y=0;y<height;y++)
                {
 	                y2 = (y * source_img.height) / height;

	                if ((x2>=0) && (y2>=0) && (x2<source_img.width) && (y2<source_img.height))
	                {
	                    xx = (polarLookup[x2,y2,0] * width) / source_img.width;
	                    yy = (polarLookup[x2,y2,1] * height) / source_img.height;

		                if ((xx>=0) && (yy>=0) && (xx<width) && (yy<height))
		                {
                            for (c=0;c<3;c++) 
	                            image[xx,yy,c] = source_img.image[x2,y2,c];
		                }
	                }
                } 
            }

            updateIntegralImageMono();
        }




        //------------------------------------------------------------------------------------------------------------------------
        //rotate the image
        //------------------------------------------------------------------------------------------------------------------------
        public void Rotate90()
        {
            int x,y,c;
            Byte temp;
  
            for (x=0;x<width;x++)
            {
                for (y=0;y<height;y++)
	            {
                    for (c=0;c<3;c++)
	                {
                        temp = image[y,x,c];
                        image[y,x,c] = image[x,y,c];
                        image[x,y,c] = temp;
                    }
                }
            }
        }

        /// <summary>
        /// draws a yellow box in a simple way
        /// </summary>
        /// <param name="tx"></param>
        /// <param name="ty"></param>
        /// <param name="bx"></param>
        /// <param name="by"></param>
        public void DrawBox(int tx, int ty, int bx, int by)
        {
            for (int x = tx; x < bx; x++)
            {
                image[x, ty, 0] = 255;
                image[x, ty, 1] = 255;
                image[x, ty, 2] = 0;

                image[x, by, 0] = 255;
                image[x, by, 1] = 255;
                image[x, by, 2] = 0;
            }

            for (int y = ty; y < by; y++)
            {
                image[tx, y, 0] = 255;
                image[tx, y, 1] = 255;
                image[tx, y, 2] = 0;

                image[bx, y, 0] = 255;
                image[bx, y, 1] = 255;
                image[bx, y, 2] = 0;
            }
        }


        //------------------------------------------------------------------------------------------------------------------------
        //draws a green box
        //------------------------------------------------------------------------------------------------------------------------
        public void DrawBox(int cx, int cy, int boxwidth, int boxheight,int boxrotation, int r, int g, int b, int lineWidth)
        {
            int[] x = new int[4];
            int[] y = new int[4];
            int dist,i;
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

            dist = (int)Math.Sqrt((boxwidth*boxwidth)+(boxheight*boxheight));
            angle[0] = (float)Math.Acos(y[0]/(float)dist);
            angle[1] = (3.1415927f/2) + ((3.1415927f/2)-angle[0]);
            angle[2] = -angle[1];
            angle[3] = -angle[0];

            for (i=0;i<4;i++)
            {
	            x[i] = cx + (int)(dist * Math.Sin(angle[i]+rot));
	            y[i] = cy + (int)(dist * Math.Cos(angle[i]+rot));
	            if (i>0) drawLine(x[i],y[i],x[i-1],y[i-1],r,g,b,lineWidth);
	            if (i==3) drawLine(x[0],y[0],x[3],y[3],r,g,b,lineWidth);
            }
        }


        //------------------------------------------------------------------------------------------------------------------------
        //rotate the image
        //------------------------------------------------------------------------------------------------------------------------
        public void Rotate(classimage sourceImage, int angle)
        {
            float rot,hyp,ang;
            int cx,cy,x,y,c,xx,yy;

            if (rotationLookup==null)
            {
                imageRotation = angle;

	            //create the lookup table
                rotationLookup = new int [width,height,2];
                for (x=0;x<width;x++)
	            {
	                for (y=0;y<height;y++)
	                {
		                rotationLookup[x,y,0]=999;
		                rotationLookup[x,y,1]=999;
	                }
	            }

                //rotate the image using floating point maths
	            rot = (float)((float)angle / 180.0 * 3.1415927);
                cx = sourceImage.width/2;
                cy = sourceImage.height/2;

                for (x=0;x<sourceImage.width;x++)
	            {
                    for (y=0;y<sourceImage.height;y++)
	                {
		                hyp = (float)Math.Sqrt(((x-cx)*(x-cx))+((y-cy)*(y-cy)));
		                if (hyp>0)
		                {
		                    ang = (float)Math.Acos((y-cy)/hyp);
		                    if (x-cx>0) ang = (float)((2*3.1415927)-ang);
	                        xx = cx + (int)(hyp * Math.Sin(rot-ang));
	                        yy = cy + (int)(hyp * Math.Cos(rot-ang));
		                }
		                else
		                {
                            xx=x;
		                    yy=y;
                        }

     	                if ((xx>=0) && (xx<width) && (yy>=0) && (yy<height))
		                {
   		                    rotationLookup[x,y,0]=xx;
		                    rotationLookup[x,y,1]=yy;
		                }
		                else
		                {
 		                    rotationLookup[x,y,0] = 0;
		                    rotationLookup[x,y,1] = 0;
		                }
	                }
	            }
            }
            else
            {
  	            //just use the lookup table as a quick method to rotate the image
                for (x=0;x<width;x++)
	            {
	                for (y=0;y<height;y++)
	                {
  	                    if (rotationLookup[x,y,0]!=999)
		                {
		                    xx = rotationLookup[x,y,0];
		                    yy = rotationLookup[x,y,1];
		                    for (c=0;c<3;c++)
		                        image[x,y,c] = sourceImage.image[xx,yy,c];
		                }
	                }
	            }

                //recalculate the integral image
                updateIntegralImageMono();
            }
        }



        //------------------------------------------------------------------------------------------------------------------------
        //flood fill from the given point using the given colour
        //------------------------------------------------------------------------------------------------------------------------
        public void floodFill(int x, int y, int r, int g, int b, int depth, ref int tx, ref int ty, ref int bx, ref int by, ref long pixels, ref long av_r, ref long av_g, ref long av_b, classimage sourceImage, ref long av_x, ref long av_y, bool EdgeDetection, int minEdgeLength, classimage tempimage, classimage colour_image)
        {
            int c,xx,yy;

            if ((image[x,y,0]==0) && (image[x,y,1]==0) && (image[x,y,2]==0) && (sourceImage.image[x,y,0]>0) && (depth<7000))
            {
                if (x<tx) tx=x;
	            if (x>bx) bx=x;
	            if (y<ty) ty=y;
	            if (y>by) by=y;

	            if (colour_image.width!=width)
	            {
	                xx = (x*colour_image.width)/width;
	                yy = (y*colour_image.height)/height;
	            }
	            else
	            {
	                xx=x;
	                yy=y;
	            }
                av_r += colour_image.image[xx,yy,0];
	            av_g += colour_image.image[xx,yy,1];
	            av_b += colour_image.image[xx,yy,2];

	            av_x += x;
	            av_y += y;

	            pixels++;

	            if ((r==0) && (g==0) && (b==0))
	            {
	                r=1;
	                g=1;
	                b=1;
	            }

                for (c=0;c<3;c++) tempimage.image[x,y,c]=255;

                image[x,y,0]=(Byte)r;
	            image[x,y,1]=(Byte)g;
	            image[x,y,2]=(Byte)b;

	            if (x>0)
	            {
                    floodFill(x-1,y,r,g,b,depth+1,ref tx,ref ty,ref bx,ref by,ref pixels,ref av_r,ref av_g,ref av_b,sourceImage,ref av_x,ref av_y,EdgeDetection,minEdgeLength,tempimage,colour_image);

	                if (y>0)
	                {
                        floodFill(x-1,y-1,r,g,b,depth+1,ref tx,ref ty,ref bx,ref by,ref pixels,ref av_r,ref av_g,ref av_b,sourceImage,ref av_x,ref av_y,EdgeDetection,minEdgeLength,tempimage,colour_image);

		                if (x<width-1)
		                {
                            floodFill(x+1,y-1,r,g,b,depth+1,ref tx,ref ty,ref bx,ref by,ref pixels,ref av_r,ref av_g,ref av_b,sourceImage,ref av_x,ref av_y,EdgeDetection,minEdgeLength,tempimage,colour_image);
		                }
	                }

	                if (x<width-1)
	                {
                        floodFill(x+1,y,r,g,b,depth+1,ref tx,ref ty,ref bx,ref by,ref pixels,ref av_r,ref av_g,ref av_b,sourceImage,ref av_x,ref av_y,EdgeDetection,minEdgeLength,tempimage,colour_image);

		                if (y<height-1)
		                {
                            floodFill(x+1,y+1,r,g,b,depth+1,ref tx,ref ty,ref bx,ref by,ref pixels,ref av_r,ref av_g,ref av_b,sourceImage,ref av_x,ref av_y,EdgeDetection,minEdgeLength,tempimage,colour_image);
		                }
	                }
	            }

                if (y>0)
	            {
                    floodFill(x,y-1,r,g,b,depth+1,ref tx,ref ty,ref bx,ref by,ref pixels,ref av_r,ref av_g,ref av_b,sourceImage,ref av_x,ref av_y,EdgeDetection,minEdgeLength,tempimage,colour_image);
	            }

	            if (y<height-1)
	            {
                    floodFill(x,y+1,r,g,b,depth+1,ref tx,ref ty,ref bx,ref by,ref pixels,ref av_r,ref av_g,ref av_b,sourceImage,ref av_x,ref av_y,EdgeDetection,minEdgeLength,tempimage,colour_image);
	            }

            }
        }


        //------------------------------------------------------------------------------------------------------------------------
        //detects the centre points of image
        //------------------------------------------------------------------------------------------------------------------------
        public int detectBlobCentres(classimage sourceimage, classimage tempimage, int min_volume, ref int average_volume, ref int average_radius, int[,] blob_data, int no_of_angles, classimage colour_image, int[,,] radius_lookup)
        {
            int x,y,no_of_blobs;
            int tx,ty,bx,by,cx,cy;
            long pixels,av_r,av_g,av_b,av_x,av_y,av_vol,av_rad;

            clear();
            tempimage.clear();
            no_of_blobs=0;
            av_vol=0;
            av_rad=0;
            for (x=1;x<width-1;x++)
            {
                for (y=1;y<height-1;y++)
                {
	                if (sourceimage.image[x,y,0]>0)
	                {
		                pixels=0;
	                    av_x=0; av_y=0;
		                av_r=0; av_g=0; av_b=0;
		                tx=x; ty=y; bx=x; by=y;
	                    floodFill(x,y,0,100,100,100,ref tx,ref ty,ref bx,ref by,ref pixels,ref av_r,ref av_g,ref av_b,sourceimage,ref av_x,ref av_y,false,0,tempimage,colour_image);
		                if (pixels>min_volume)
		                {
		                    cx = (int)(av_x / pixels);
		                    cy = (int)(av_y / pixels);
		                    image[cx,cy,0]=255;
		                    image[cx,cy,1]=255;
		                    image[cx,cy,2]=255;
		                    av_vol += pixels;
		                    av_rad += (((bx-tx) + (by-ty))/2);

                            //update blob data
		                    if (no_of_blobs<2000)
		                    {
		                        blob_data[no_of_blobs,0] = (int)pixels;
		                        blob_data[no_of_blobs,1] = cx;
		                        blob_data[no_of_blobs,2] = cy;
		                        blob_data[no_of_blobs,3] = bx-tx;
		                        blob_data[no_of_blobs,4] = by-ty;
			                    blob_data[no_of_blobs,5] = (int)(av_r / pixels);
			                    blob_data[no_of_blobs,6] = (int)(av_g / pixels);
			                    blob_data[no_of_blobs,7] = (int)(av_b / pixels);
                                if (no_of_angles>1) tempimage.getBlobShape(no_of_angles,cx,cy,tx,ty,bx,by,blob_data,no_of_blobs,8,radius_lookup);
		                    }

		                    no_of_blobs++;
		                }
		                tempimage.clearArea(tx,ty,bx,by);
	                }
	            }
            }

            if (no_of_blobs>0)
            {
                average_volume = (int)(av_vol / no_of_blobs);
	            average_radius = (int)(av_rad / no_of_blobs);
            }
            return(no_of_blobs);
        }


        //highlights high contrast points within the image and returns the number of points detected
        public int highContrastPoints(classimage sourceimage, int radius, int threshold)
        {
            int x,y,xx,yy,c,intensity1,intensity2,contrast,hits,pixels;
            int pointsfound=0;

            pixels = radius*radius*4;
            threshold = threshold * pixels * 3;

            for (x=0;x<width;x++)
            {
                for (y=0;y<height;y++)
	            {
	                intensity1=0;
                    for (c=0;c<3;c++) intensity1 += sourceimage.image[x,y,c];

	                contrast=0;
	                hits=0;
	                for (xx=x-radius;xx<x+radius;xx++)
	                {
	                    if ((xx>-1) && (xx<width))
		                {
	                        for (yy=y-radius;yy<y+radius;yy++)
	                        {
	                            if ((yy>-1) && (yy<height))
		                        {
			                        if (!((xx==x) && (yy==y)))
			                        {
			                            intensity2=0;
			                            for (c=0;c<3;c++) intensity2 += sourceimage.image[xx,yy,c];
				                        if (intensity1 >= intensity2)
				                            contrast += intensity1 - intensity2;
				                            else
				                            contrast += intensity2 - intensity1;
				                        hits+=3;
			                        }
			                    }
		                    }
		                }
	                }

	                if (hits>0)
	                {
		                if (contrast > threshold)
		                {
		                    pointsfound++;
		                    for (c=0;c<3;c++) image[x,y,c]=255;
		                }
	                }
	            }
            }
            return(pointsfound);
        }


        public void getOrientations(classimage sourceimage, classimage edgesimage, int radius, int no_of_angles, int tx, int ty, int bx, int by, int[,,] radius_lookup, int trim, ref int average_confidence, classimage secondaryimage)
        {
            int x,y,xx,yy,xx2,yy2,r,prev_ang,c,ang,min,max,max_ang,min_ang,value,ang_diff,min_diff,half_angles,min_radius;
            int[] angle_values = new int[30];
            int centre_value,max_r,min_r,diff,max_diff;
            int positive_gradient,negative_gradient,local_radius,next_ang;
            int hits,av,confidence,confidence_thresh;
            int variance,max_variance,hits2;

            clearArea(tx,ty,bx-1,by-1);
            secondaryimage.clearArea(tx,ty,bx-1,by-1);

            hits=0;
            av=0;
            r = 0;
            //confidence_thresh = average_confidence*40/100;
            confidence_thresh = average_confidence*10/100;
            local_radius=1;
            min_radius=radius*8/10;
            min_diff = 0; //no_of_angles/6;
            half_angles = no_of_angles/2;
            if (min_diff<1) min_diff=1;
            for (y=ty;y<by;y++)
            {
                max_variance=0;
                for (x=tx;x<bx;x++)
                {
	                if (edgesimage.image[x,y,0]>0)
	                {
	                    max_ang=0;
		                min_ang=0;
		                max_r=0;
		                min_r=0;
		                max=-99999;
		                min=99999;

		                //get the value at this location
		                hits2=0;
		                centre_value=0;
		                for (xx2=x-local_radius;xx2<=x+local_radius;xx2++)
		                    for (yy2=y-local_radius;yy2<=y+local_radius;yy2++)
		                        if ((xx2>-1) && (xx2<width) && (yy2>-1) && (yy2<height))
	                                for (c=0;c<3;c++)
			                        {
			                            centre_value += sourceimage.image[xx2,yy2,c];
				                        hits2++;
			                        }

		                //get the values at the periphery for each rotation
	                    for (ang=0;ang<no_of_angles;ang++)
	                    {
		                    hits2=0;
	                        value=0;
		                    r = radius-1;
	                        xx = x+radius_lookup[ang,r,0];
		                    yy = y+radius_lookup[ang,r,1];
		                    for (xx2=xx-local_radius;xx2<xx+local_radius;xx2++)
		                        for (yy2=yy-local_radius;yy2<yy+local_radius;yy2++)
		                            if ((xx2>-1) && (xx2<width) && (yy2>-1) && (yy2<height))
			                        {
		                                value += sourceimage.image[xx2,yy2,0];
				                        hits2++;
			                        }
		  
		                    angle_values[ang] = value; // /hits2;
	                    }

		                //get minimum and maximum values
		                max_diff=0;
		                min_diff=0;
		                negative_gradient=0;
                        positive_gradient=0;
		                for (ang=0;ang<no_of_angles;ang++)
	                    {
		                    prev_ang = ang-1;
		                    if (prev_ang<0) prev_ang = no_of_angles-1;
		                    next_ang = ang+1;
		                    if (next_ang>=no_of_angles) next_ang=0;
		                    value = angle_values[ang] + angle_values[prev_ang] + angle_values[next_ang];
		                    diff = value - centre_value;

		                    if (diff>max_diff) max_diff = diff;
		                    if (diff<min_diff) min_diff = diff;
		                    if (value>max)
		                    {
		                        max=value;
		                        max_ang = ang;
			                    max_r = r;
			                    if (diff>0) 
			                        positive_gradient=diff;
			                        else
                                    negative_gradient=-diff;
	                        }
		                    if (value<min)
		                    {
 		                        min=value;
                                min_ang = ang;
		                        min_r = min;
	                        }
		                }

		                variance = (max-min)/6;
		                if (variance>max_variance) max_variance=variance;

		                confidence=max-min;
		                av += confidence;
		                hits++;

		                if (confidence>confidence_thresh)
		                {
		                    ang_diff = max_ang - min_ang;
		                    if (ang_diff<0) ang_diff = -ang_diff;
		                    if (ang_diff>half_angles) ang_diff = no_of_angles - ang_diff;
		                    if (max_diff<-min_diff) max_diff=-min_diff;
		                    if (-min_diff<max_diff) min_diff=-max_diff;
		                    image[x,y,0] = 127;
		                    if (positive_gradient>negative_gradient)
		                    {
		                        if (max_diff>0) image[x,y,0] = (Byte)(127+((positive_gradient*127)/max_diff));		  
		                    }
		                    else
		                    {
		                        if (min_diff<0) image[x,y,0] = (Byte)(127-((negative_gradient*127)/(-min_diff)));
		                    }
  		                    image[x,y,1] = (Byte)((ang_diff*255) / half_angles);
		                    image[x,y,2] = (Byte)variance;

		                    secondaryimage.image[x,y,0] = (Byte)((max_ang*255) / no_of_angles);
		                    secondaryimage.image[x,y,1] = (Byte)((min_ang*255) / no_of_angles);
		                    //if (min_ang>half_angles)
		                    //  secondaryimage.image[x,y,1] = 255;
		                    //  else
		                    //  secondaryimage.image[x,y,1] = 0;
		                    if (max_ang>half_angles)
		                        secondaryimage.image[x,y,2] = 100;
		                        else
		                        secondaryimage.image[x,y,2] = 0;
		                }
		                else edgesimage.image[x,y,0]=0;
	                }
	            }  

	            //normalise variance
                if (max_variance>0)
	            {
                    for (x=tx;x<bx;x++)
                    {
	                    if (edgesimage.image[x,y,0]>0)
	                    {
		                    image[x,y,2] = (Byte)(((int)(image[x,y,2])*255)/max_variance);
		                }
	                }
	            }
            }

            if (hits>0)
                average_confidence = (average_confidence + (av/hits))/2;
        }


        public void getBlobShape(int no_of_angles, int cx, int cy, int tx, int ty, int bx, int by, int[,] angles_data, int index, int offset, int[,,] radius_lookup)
        {
            int i,radius,xx,yy,wx,wy,r,xx2,yy2;
            bool found;

            for (i=0;i<no_of_angles;i++)
            {
                //calculate the angle in radians
	            radius=1;
	            r = 2;
	            xx = cx;
	            yy = cy;
	            wx = cx;
	            wy = cy;
	            while ((radius<width) && (xx>=tx) && (xx<=bx) && (yy>=ty) && (yy<=by) && (xx>0) && (xx<width) && (yy>0) && (yy<height))
	            {
	                radius++;
	                xx = cx + radius_lookup[i,radius,0];
	                yy = cy + radius_lookup[i,radius,1];
	                found=false;
	                xx2=xx-1;
	                while ((xx2<=xx+1) && (!found))
	                {
	                    yy2=yy-1;
                        while ((yy2<=yy+1) && (!found))
		                {
	                        if ((xx2>0) && (xx2<width) && (yy2>0) && (yy2<height))
	                        if (image[xx2,yy2,0]>0) { found=true; wx = xx; wy = yy; r = radius; }
		                    yy2++;
		                }
		                xx2++;
	                }
	            }
	            angles_data[index,(i*3)+offset] = r;
	            angles_data[index,(i*3)+offset+1] = wx;
	            angles_data[index,(i*3)+offset+2] = wy;
            }
        }


        //------------------------------------------------------------------------------------------------------------------------
        //fill in any small gaps in the given binary image
        //------------------------------------------------------------------------------------------------------------------------
        public void fillin(classimage binary, int tx, int ty, int bx, int by)
        {
            int x,y,c,xx,yy,hits,minhits;
            int radius=1;
            int pixelstate;

            minhits=(radius*5)-1;
            for (x=tx;x<bx;x++)
                for (y=ty;y<by;y++)
	            {
	                if (binary.image[x,y,0]==0)
	                {
	                    pixelstate=0;
	                    hits=0;
                        xx=x-radius;
	                    while ((xx<=x+radius) && (hits<minhits))
		                {
		                    if ((xx>=0) && (xx<width))
		                    {
		                        yy=y-radius;
	                            while ((yy<=y+radius) && (hits<minhits))
		                        {
			                        if ((yy>=0) && (yy<height))
			                        {
			                            if (binary.image[xx,yy,0]>0) hits++;
			                        }
			                        yy++;
			                    }
		                    }
		                    xx++;
		                }

		                if (hits>=minhits) pixelstate=255;
	                }
	                else pixelstate=255;

	                for (c=0;c<3;c++) image[x,y,c]=(Byte)pixelstate;
	            }
        }


        //------------------------------------------------------------------------------------------------------------------------
        //fill in any small gaps in the a normal image using an everage value
        //------------------------------------------------------------------------------------------------------------------------
        public void fillinAverage(classimage source_img, int threshold)
        {
            int x,y,c,xx,yy,hits,minhits;
            int[] average = new int[3];
            int radius=1;

            copyImage(source_img);
            minhits=(radius*5)-1;
            for (x=0;x<width;x++)
                for (y=0;y<height;y++)
	            {
	                if (source_img.image[x,y,0]<threshold)
	                {
	                    hits=0;
                        xx=x-radius;
		                for (c=0;c<3;c++) average[c]=0;
	                    while ((xx<=x+radius) && (hits<minhits))
		                {
		                    if ((xx>=0) && (xx<width))
		                    {
		                        yy=y-radius;
	                            while ((yy<=y+radius) && (hits<minhits))
		                        {
			                        if ((yy>=0) && (yy<height))
			                        {
			                            if (source_img.image[xx,yy,0]>threshold)
				                        {
				                            for (c=0;c<3;c++) average[c]+=source_img.image[xx,yy,c];
				                            hits++;
				                        }
			                        }
			                        yy++;
			                    }
		                    }
		                    xx++;
		                }

		                if (hits>=minhits)
		                {
		                    for (c=0;c<3;c++)
		                    {
		                        average[c] /= hits;
			                    image[x,y,c] = (Byte)average[c];
		                    }
		                }
	                }
	            }
        }



        //------------------------------------------------------------------------------------------------------------------------
        //replace one colour with another
        //------------------------------------------------------------------------------------------------------------------------
        public void replaceColour(int r, int g, int b, int new_r, int new_g, int new_b)
        {
            int x,y;

            if ((new_r==0) && (new_g==0) && (new_b==0))
            {
                new_r=1;
	            new_g=1;
	            new_b=1;
            }
  
            for (x=0;x<width;x++)
            {
                for (y=0;y<height;y++)
	            {
                    if ((image[x,y,0]==r) && (image[x,y,1]==g) && (image[x,y,2]==b))
	                {
                        image[x,y,0] = (Byte)new_r;
                        image[x,y,1] = (Byte)new_g;
                        image[x,y,2] = (Byte)new_b;
                    }
                }
            }
        }


        //------------------------------------------------------------------------------------------------------------------------
        //apply an absolute threshold
        //------------------------------------------------------------------------------------------------------------------------
        public void applyThreshold(int value, bool above)
        {
            int x,y,c,intensity;

            value *= 3;
            for (x=0;x<width;x++)
            {
                for (y=0;y<height;y++)
	            {
	                intensity=0;
                    for (c=0;c<3;c++) intensity += image[x,y,c];

	                if (above)
	                {
	                    if (intensity<value)
		                    for (c=0;c<3;c++) image[x,y,c]=0;
	                }
	                else
	                {
	                    if (intensity>value)
		                {
		                    for (c=0;c<3;c++) image[x,y,c]=0;
		                }
	                }
	            }
            }
        }


        //------------------------------------------------------------------------------------------------------------------------
        //create a binary image by thresholding the given image within the given region
        //------------------------------------------------------------------------------------------------------------------------
        public void createBinary(classimage sourceimg, int value, bool above, int tx, int ty, int bx, int by)
        {
            int x,y,c,intensity;

            value *= 3;
            for (x=tx;x<bx;x++)
            {
                for (y=ty;y<by;y++)
	            {
	                intensity=0;
                    for (c=0;c<3;c++) intensity += sourceimg.image[x,y,c];

	                if (above)
	                {
	                    if (intensity<value)
		                    for (c=0;c<3;c++) image[x,y,c]=0;
		                    else
		                    for (c=0;c<3;c++) image[x,y,c]=255;
	                }
	                else
	                {
	                    if (intensity>value)
		                    for (c=0;c<3;c++) image[x,y,c]=0;
		                    else
		                    for (c=0;c<3;c++) image[x,y,c]=255;
	                }
	            }
            }
        }


        //------------------------------------------------------------------------------------------------------------------------
        //replace one colour with another within the given area
        //------------------------------------------------------------------------------------------------------------------------
        public void replaceColourArea(int r, int g, int b, int new_r, int new_g, int new_b, int tx, int ty, int bx, int by)
        {
            int x,y;
  
            for (x=tx;x<bx;x++)
            {
                for (y=ty;y<by;y++)
	            {
                    if ((image[x,y,0]==r) && (image[x,y,1]==g) && (image[x,y,2]==b))
	                {
                        image[x,y,0] = (Byte)new_r;
                        image[x,y,1] = (Byte)new_g;
                        image[x,y,2] = (Byte)new_b;
                    }
                }
            }
        }


        //------------------------------------------------------------------------------------------------------------------------
        //filter using the given colour
        //------------------------------------------------------------------------------------------------------------------------
        public long filterColour(classimage sourceImage, int r, int g, int b, int tollerance, ref int centre_x, ref int centre_y)
        {
            int x,y,dr,dg,db,count;
            long pixels,cx,cy;
  
            cx=0;
            cy=0;
            pixels=0;
            count=1;
            for (x=0;x<width;x++)
            {
                for (y=0;y<height;y++)
	            {
	                dr = Math.Abs(sourceImage.image[x,y,0] - r);
	                dg = Math.Abs(sourceImage.image[x,y,1] - g);
	                db = Math.Abs(sourceImage.image[x,y,2] - b);
                    if ((dr<tollerance) && (dg<tollerance) && (db<tollerance))
	                {
                        image[x,y,0] = 255;
                        image[x,y,1] = 255;
                        image[x,y,2] = 255;
		                pixels++;
		                cx += x;
		                cy += y;
		                count++;
                    }
	                else
	                {
                        image[x,y,0] = 0;
                        image[x,y,1] = 0;
                        image[x,y,2] = 0;
	                }

                }
            }

            centre_x = (Byte)(cx / count);
            centre_y = (Byte)(cy / count);
            updateIntegralImageMono();

            return(pixels);
        }



        //------------------------------------------------------------------------------------------------------------------------
        //filter using skin colour
        //------------------------------------------------------------------------------------------------------------------------
        public long filterSkinColour(classimage sourceImage, int tollerance, ref int centre_x, ref int centre_y)
        {
            int x,y,dr,dg,count;
            long pixels,cx,cy,r,g,b;
  
            cx=0;
            cy=0;
            pixels=0;
            count=1;
            for (x=0;x<width;x++)
            {
                for (y=0;y<height;y++)
	            {
	                r = sourceImage.image[x,y,0];
	                g = sourceImage.image[x,y,1];
	                b = sourceImage.image[x,y,2];

	                dr = (Byte)(2*r);
	                dg = (Byte)(g+b);

                   if ((dr-dg>tollerance) && (dr-dg<100))
	               {
                       image[x,y,0] = 255;
                       image[x,y,1] = 255;
                       image[x,y,2] = 255;
		               pixels++;
		               cx += x;
		               cy += y;
		               count++;
                   }
	               else
	               {
                       image[x,y,0] = 0;
                       image[x,y,1] = 0;
                       image[x,y,2] = 0;
	               }
               }
           }

           centre_x = (int)(cx / count);
           centre_y = (int)(cy / count);
           updateIntegralImageMono();

           return(pixels);
       }



       //------------------------------------------------------------------------------------------------------------------------
       //
       //------------------------------------------------------------------------------------------------------------------------
       public int relativeThreshold(int value, int tx, int ty, int bx, int by, ref int averageWidth)
       {
           int av,pixels,NoOfRows;
           int x,y,min,hits,rowhits,totrowhits,minrowhits;
  
           pixels = width * height;
           hits = 0;
           av = 0;
           for (x=0;x<width;x++)
               for (y=0;y<height;y++) av = av + image[x,y,0];
  
           av = av / pixels;
           min = (int)((av * (100 - value)) / 100);
  
           NoOfRows=1;
           minrowhits = (bx-tx)/5;
           totrowhits=0;
           for (y=ty;y<by;y++)
           {
	           rowhits=0;
               for (x=tx;x<bx;x++)
	           {
                   if (image[x,y,0] < min)
		           rowhits++;
	           }

	           hits+=rowhits;
	           if (rowhits>minrowhits)
	           {
                   totrowhits+=rowhits;
	               NoOfRows++;
	           }
           }
           totrowhits /= NoOfRows;
           averageWidth = (((by-ty) - NoOfRows) + totrowhits)/2;
  
           return((hits * 100) / pixels);
       }

       //------------------------------------------------------------------------------------------------------------------------
       //returns the 'centre of gravity' for the given region relative to the given colour
       //------------------------------------------------------------------------------------------------------------------------
       public void CG(int tx, int ty, int bx, int by, Byte targ_r, Byte targ_g, Byte targ_b, ref int cx, ref int cy, int maxval)
       {
           int x,y,c;
           int[] p = new int[3];
           long tot,tot_x,tot_y,dp;
           int min_value=150;
  
           tot = 1;
           tot_x = 0;
           tot_y = 0;
           for (x=tx;x<bx;x++)
           {
	           if ((x>=0) && (x<width))
	           {
                   for (y=ty;y<by;y++)
	               {
		               if ((y>=0) && (y<width))
		               {
                           for (c=0;c<3;c++) p[c] = image[x,y,c];
		                   dp = Math.Abs(p[0] - (int)targ_r);
                           dp += Math.Abs(p[1] - (int)targ_g);
                           dp += Math.Abs(p[2] - (int)targ_b);
	                       if (dp<min_value) dp=0;
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
       //return the centre point of the darkest area within the image
       //---------------------------------------------------------------------------------------------
       public void darkestArea(int tx, int ty, int bx, int by, int search_width, int search_height, ref int cx, ref int cy)
        {
            int x, y;
            long intensity;
            long min_intensity = 0;

            cx = 0;
            cy = 0;
            for (x=tx;x<bx - search_width;x++)
                for (y = ty; y < by - search_height; y++)
                {
                    intensity = getIntegral(x, y, x + search_width, y + search_height);
                    if ((min_intensity == 0) || (intensity < min_intensity))
                    {
                        min_intensity = intensity;
                        cx = x + (search_width / 2);
                        cy = y + (search_height / 2);
                    }
                }
        }


        //---------------------------------------------------------------------------------------------
        //detect vertical edges
        //---------------------------------------------------------------------------------------------
        public void filterVertical(classimage sourceimage)
       {
           int x,y,c,xx,yy,left,right,dp;
           int horizontal = sourceimage.width/8;
           int vertical;
           long pixels;

           if (horizontal<1) horizontal=1;
           vertical = horizontal*2;
           pixels = horizontal*vertical*3;

           for (x=0;x<width;x++)
           {
               for (y=0;y<height;y++)
	           {
	               if ((x>=horizontal) && (x<width-horizontal) && (y>=vertical) && (y<height-vertical))
	               {
	                   left=0;
	                   right=0;
	                   for (xx=x-horizontal;xx<x+horizontal;xx++)
		               {
	                       for (yy=y-vertical;yy<y+vertical;yy++)
		                   {
			                   for (c=0;c<3;c++)
			                   {
		                           if (xx<x) 
		                               left += sourceimage.image[xx,yy,c];
		                               else
			                           right += sourceimage.image[xx,yy,c];
			                   }
		                   }
		               }
		               dp = (int)((Math.Abs(left-right) * 2) / pixels);
	                   if (dp>255) dp=255;
	                   for (c=0;c<3;c++) image[x,y,c] = (Byte)dp;
	               }
	               else for (c=0;c<3;c++) image[x,y,c] = 0;
	           }
           }
       }



       //---------------------------------------------------------------------------------------------
       //detect horizontal edges
       //---------------------------------------------------------------------------------------------
       public void filterHorizontal(classimage sourceimage)
       {
           int x,y,c,xx,yy,above,below,dp;
           int horizontal;
           int vertical = sourceimage.width/8;
           long pixels;

           if (vertical<1) vertical=1;
           horizontal = vertical*2;
           pixels = horizontal*vertical*3;

           for (x=0;x<width;x++)
           {
               for (y=0;y<height;y++)
	           {
	               if ((x>=horizontal) && (x<width-horizontal) && (y>=vertical) && (y<height-vertical))
	               {
	                   above=0;
	                   below=0;
	                   for (xx=x-horizontal;xx<x+horizontal;xx++)
		               {
	                       for (yy=y-vertical;yy<y+vertical;yy++)
		                   {
			                   for (c=0;c<3;c++) 
			                   {
		                           if (yy<y) 
		                               above += sourceimage.image[xx,yy,c];
		                               else
			                           below += sourceimage.image[xx,yy,c];
			                   }
		                   }
		               }

		               dp = (int)((Math.Abs(above-below) * 2) / pixels);
	                   if (dp>255) dp=255;
	                   for (c=0;c<3;c++) image[x,y,c] = (Byte)dp;
	               }
	               else for (c=0;c<3;c++) image[x,y,c] = 0;
	           }
           }
       }



       //---------------------------------------------------------------------------------------------
       //detect blobs
       //---------------------------------------------------------------------------------------------
       public void filterBlobs(classimage sourceimage)
       {
           int x,y,c,xx,yy,av,variance,n,v;
           int horizontal = sourceimage.width/6;
           int vertical = horizontal;
           long pixels;

           if (vertical<1) vertical=1;
           if (horizontal<1) horizontal=1;
           pixels = horizontal*vertical;

           for (x=0;x<width;x++)
           {
               for (y=0;y<height;y++)
	           {
	               if ((x>=horizontal) && (x<width-horizontal) && (y>=vertical) && (y<height-vertical))
	               {
	                   av = sourceimage.getAverageIntensity(x-horizontal,y-vertical,x+horizontal,y+vertical);
	                   variance=0;
	                   n=0;
	  
	                   for (xx=x-horizontal;xx<x+horizontal;xx++)
		               {
	                       for (yy=y-vertical;yy<y+vertical;yy++)
		                   {
		                       v=0;
		                       for (c=0;c<3;c++) v+=sourceimage.image[xx,yy,c];
			                   variance += Math.Abs((v/3) - av);
		                       n++;
		                   }
		               }

	                   variance = 255 - (variance*4 / n);
	                   if (variance<0) variance=0;
	                   variance = 255 - variance;

	                   for (c=0;c<3;c++) image[x,y,c] = (Byte)variance;
	               }
	               else for (c=0;c<3;c++) image[x,y,c] = 0;
	           }
           }
       }




       //---------------------------------------------------------------------------------------------
       //detect blobs
       //---------------------------------------------------------------------------------------------
       public int countPixels()
       {
           int x,y,p=0;

           for (x=0;x<width;x++)
               for (y=0;y<height;y++)
	           {
	               if (image[x,y,0]>0) p++;
	           }

           return(p);
       }


       //---------------------------------------------------------------------------------------------
       // Detect edges 
       //---------------------------------------------------------------------------------------------
       public void detectEdgesStandard(classimage sourceimage, int minThresh, int patchSize)
       {
           int x,y,c;
           int halfPatch = patchSize/2;
           int quarterPatch = halfPatch/2;
           int p1,p2,diff,pixels;

           if (halfPatch<2) halfPatch=2;
           if (quarterPatch<1) quarterPatch=1;
           pixels = patchSize*patchSize;
           sourceimage.updateIntegralImageMono();
           for (y=halfPatch;y<height-halfPatch;y++)
           {
               for (x=halfPatch;x<width-halfPatch;x++)
	           {
	               p1 = (int)sourceimage.getIntegral(x-halfPatch,y-halfPatch,x+halfPatch,y+halfPatch);
	               p2 = (int)sourceimage.getIntegral(x-halfPatch,y-halfPatch,x+halfPatch,y+halfPatch);
	               diff = Math.Abs((p2*2)-p1);	  
	               diff/=(pixels*1);
	               diff*=diff;
	               if (diff>255) diff=255;
	               for (c=0;c<3;c++) image[x,y,c] = (Byte)diff;
	           }
           }
       }


       //---------------------------------------------------------------------------------------------
       // Detect edges ON 
       //---------------------------------------------------------------------------------------------
       public void detectEdgesOn(classimage sourceimage, int minThresh, int patchSize)
       {
           int x,y,c;
           int halfPatch = patchSize/2;
           int quarterPatch = halfPatch/2;
           int p1,p2,diff,pixels;

           if (halfPatch<2) halfPatch=2;
           if (quarterPatch<1) quarterPatch=1;
           pixels = patchSize*patchSize;
           sourceimage.updateIntegralImageMono();
           for (y=halfPatch;y<height-halfPatch;y++)
           {
               for (x=halfPatch;x<width-halfPatch;x++)
	           {
	               p1 = (int)sourceimage.getIntegral(x-halfPatch,y-halfPatch,x+halfPatch,y+halfPatch);
	               p2 = (int)sourceimage.getIntegral(x-quarterPatch,y-quarterPatch,x+quarterPatch,y+quarterPatch);
	               diff = ((p2*2)-p1);
	               if (diff<0)
	               {
	                   diff = Math.Abs(diff);
	                   diff/=(pixels*1);
	                   diff*=diff;
	                   if (diff>255) diff=255;
	               }
	               else diff=0;
                   diff=255-diff;
	               if (diff<minThresh) diff=0;

	               for (c=0;c<3;c++) image[x,y,c] = (Byte)diff;
	           }
           }
       }

       //---------------------------------------------------------------------------------------------
       // Detect edges OFF 
       //---------------------------------------------------------------------------------------------
       public void detectEdgesOff(classimage sourceimage, int minThresh, int patchSize)
       {
           int x,y,c;
           int halfPatch = patchSize/2;
           int quarterPatch = halfPatch/2;
           int p1,p2,diff,pixels;

           if (halfPatch<2) halfPatch=2;
           if (quarterPatch<1) quarterPatch=1;
           pixels = patchSize*patchSize;
           sourceimage.updateIntegralImageMono();
           for (y=halfPatch;y<height-halfPatch;y++)
           {
               for (x=halfPatch;x<width-halfPatch;x++)
	           {
	                p1 = (int)sourceimage.getIntegral(x-halfPatch,y-halfPatch,x+halfPatch,y+halfPatch);
	                p2 = (int)sourceimage.getIntegral(x-quarterPatch,y-quarterPatch,x+quarterPatch,y+quarterPatch);
	                diff = ((p2*2)-p1);
	                if (diff>0)
	                {
	                    diff = Math.Abs(diff);
	                    diff/=(pixels*10);
	                    diff*=diff;
	                    if (diff>255) diff=255;
	                }
	                else diff=0;
	                diff=255-diff;
	                if (diff<minThresh) diff=0;
	                for (c=0;c<3;c++) image[x,y,c] = (Byte)diff;
	           }
           }
       }


       //---------------------------------------------------------------------------------------------
       // Detect edges within the given binary image
       //---------------------------------------------------------------------------------------------
       public void detectEdgesBinary(classimage sourceimage, int tx, int ty, int bx, int by)
       {
           int x,y,xx,yy,c;
           bool found;

           if (tx<1) tx=1;
           if (ty<1) ty=1;
           if (bx>=width-1) bx=width-2;
           if (by>=height-1) by=height-2;
           for (y=ty;y<by;y++)
           {
               for (x=tx;x<bx;x++)
	           {
	               found=false;
	               if (sourceimage.image[x,y,0]>0)
	               {
	                   xx=x-1;
                       while ((!found) && (xx<=x+1))
	                   {
	                       yy=y-1;
                           while ((!found) && (yy<=y+1))
	                       {
		                       if (!((xx==x) && (yy==y)))
		                       {
		                           if (sourceimage.image[xx,yy,0] != sourceimage.image[x,y,0]) found=true;
		                       }
		                       yy++;
		                   }
		                   xx++;
	                   }
	               }

                   for (c=0;c<3;c++)
	                   if (found) image[x,y,c]=255; else image[x,y,c]=0;
	           }
           }
       }



       //---------------------------------------------------------------------------------------------
       // skeletonises the given image
       //---------------------------------------------------------------------------------------------
       public void skeletonizeHorizontal(classimage binaryimage)
       {
           int x,y,start_x=0,cx,c;
           int state;
           const int STATE_SEARCHING=0;
           const int STATE_FOUND=1;

           clear();
           for (y=1;y<height-1;y++)
           {
               state=STATE_SEARCHING;
               for (x=1;x<width-1;x++)
	           {
                   if ((binaryimage.image[x,y,0] > 0) && (state==STATE_SEARCHING))
	               {
	                   state=STATE_FOUND;
		               start_x=x;
	               }
                   if ((binaryimage.image[x+1,y,0] == 0) && (binaryimage.image[x,y,0] == 0) && (state==STATE_FOUND))
	               {
	                   state=STATE_SEARCHING;
		               cx = start_x + (((x-1)-start_x)/2);
		               if (binaryimage.image[cx,y,0]>0) for (c=0;c<3;c++) image[cx,y,c]=255;
	               }
	           }
           }
       }


       //---------------------------------------------------------------------------------------------
       // removes pixels with less than the given number of neighbours from a binary image
       //---------------------------------------------------------------------------------------------
       public void removeSinglePixels(int min_neighbours)
       {
           int x,y,xx,yy,neighbours,c;

           for (y=1;y<height-1;y++)
           {
               for (x=1;x<width-1;x++)
	           {
	               if (image[x,y,0]>0)
	               {
	                   neighbours=0;
	                   for (xx=x-1;xx<=x+1;xx++)
	                   {
	                       for (yy=y-1;yy<=y+1;yy++)
	                       {
		                       if (!((x==xx) && (y==yy)))
			                   {
			                       if (image[xx,yy,0]>0) neighbours++;
			                   }
		                   }
	                   }
		               if (neighbours<min_neighbours) for (c=0;c<3;c++) image[x,y,c]=0;
	               }
	           }
           }
       }


       //---------------------------------------------------------------------------------------------
       // skeletonises the given image
       //---------------------------------------------------------------------------------------------
       public void skeletonize(classimage binaryimage)
       {
           int x,y,start_x=0,start_y=0,cx,cy,c;
           int state;
           const int STATE_SEARCHING=0;
           const int STATE_FOUND=1;

           clear();
           for (y=1;y<height-1;y++)
           {
               state=STATE_SEARCHING;
               for (x=1;x<width-1;x++)
	           {
                   if ((binaryimage.image[x,y,0] > 0) && (state==STATE_SEARCHING))
	               {
	                   state=STATE_FOUND;
		               start_x=x;
	               }
                   if ((binaryimage.image[x+1,y,0] == 0) && (binaryimage.image[x,y,0] == 0) && (state==STATE_FOUND))
	               {
	                   state=STATE_SEARCHING;
		               cx = start_x + (((x-1)-start_x)/2);
		               if (binaryimage.image[cx,y,0]>0) for (c=0;c<3;c++) image[cx,y,c]=255;
	               }
	           }

               state=STATE_SEARCHING;
               for (x=width-2;x>0;x--)
	           {
                   if ((binaryimage.image[x,y,0] > 0) && (state==STATE_SEARCHING))
	               {
	                   state=STATE_FOUND;
		               start_x=x;
	               }
                   if ((binaryimage.image[x+1,y,0] == 0) && (binaryimage.image[x,y,0] == 0) && (state==STATE_FOUND))
	               {
	                   state=STATE_SEARCHING;
		               cx = start_x + (((x-1)-start_x)/2);
		               if (binaryimage.image[cx,y,0]>0) for (c=0;c<3;c++) image[cx,y,c]=255;
	               }
	           }

           }

           for (x=1;x<width-1;x++)
           {
               state=STATE_SEARCHING;
               for (y=1;y<height-1;y++)  
	           {
                   if ((binaryimage.image[x,y,0] > 0) && (state==STATE_SEARCHING))
	               {
	                   state=STATE_FOUND;
		               start_y=y;
	               }
                   if ((binaryimage.image[x,y+1,0] == 0) && (binaryimage.image[x,y,0] == 0) && (state==STATE_FOUND))
	               {
	                   state=STATE_SEARCHING;
		               cy = start_y + (((y-1)-start_y)/2);
		               if (binaryimage.image[x,cy,0]>0)
		               {
		                   for (c=0;c<3;c++) image[x,cy,c]=255;
		               }
	               }
	           }
           }
       }
 

       //---------------------------------------------------------------------------------------------
       // detects circle shapes and highlights the centre points
       // requires a binary blob image and binary edges image
       //---------------------------------------------------------------------------------------------
       public void detectCircles(classimage binaryimage, classimage edgesimage, int radius)
       {
           int x,y,xx,yy,c,hits,maxhits;
           int innerRadius=radius/3;

           if (innerRadius<1) innerRadius=1;
           clear();
           for (x=radius;x<width-radius;x++)
           {
               for (y=radius;y<height-radius;y++)
               {
	               if (binaryimage.image[x,y,0]>0)
	               {
	                   //check that there is nothing in the middle
	                   hits=0;
		               maxhits=2;
		               xx=x-innerRadius;
		               while ((hits<maxhits) && (xx<=x+innerRadius))
		               {		  
		                   yy=y-innerRadius;
	                       while ((hits<maxhits) && (yy<=y+innerRadius))
		                   {
		                       if (edgesimage.image[xx,yy,0]>0) hits++;
			                   yy++;
		                   }
	                       xx++;
		               }

		               if (hits<maxhits)
		               {
		                   for (c=0;c<3;c++) image[x,y,c]=255;
		               }

	               }
	           }
           }
       }


       //---------------------------------------------------------------------------------------------
       //find maximal gravity wells in the image
       //---------------------------------------------------------------------------------------------
       public void findGravityWells(classimage source_img, int search_radius)
       {
           int x,y,c,border,cx=0,cy=0,tx,ty,bx,by,inhibit_radius;
           //bool found;

           border = search_radius/2;
           inhibit_radius = border/2;

           for (x=0;x<width;x++)
               for (y=0;y<height;y++)
	               for (c=0;c<3;c++)
	                   image[x,y,c]=0;

           for (x=0;x<width;x++)
               for (y=0;y<height;y++)
	           {
	               tx = x-border;
	               ty = y-border;
	               bx = x+border;
	               by = y+border;
 
	               source_img.CG(tx,ty,bx,by,0,0,0,ref cx,ref cy,0);
	               if (!((cx==0) && (cy==0)))
		           {
		               //are there any pixels within the inhibition region?
		               //found=false;

                       if ((cx>tx+inhibit_radius) && (cx<bx-inhibit_radius))
		               {
                           if ((cy>ty+inhibit_radius) && (cy<by-inhibit_radius))
			               {
                               for (c=0;c<3;c++) image[cx,cy,c]=255;
			               }
		               }
		           }
	           }
       }


       //---------------------------------------------------------------------------------------------
       //enclosure image
       //---------------------------------------------------------------------------------------------
       public void enclosure(classimage source_img, int search_radius, int threshold)
       {
           int x,y,tx,ty,bx,by,r;
           int[] p = new int[4];
           int c,value,pixels,minval;

           source_img.updateIntegralImageMono();
           r = search_radius;

           for (x=0;x<width;x++)
           {
	           for (y=0;y<height;y++)
	           {
                   tx = x-r;
	               ty = y-r;
	               bx = x+r;
	               by = y+r;
	               if (tx<0) tx=0;
	               if (bx>=width) bx=width-1;
	               if (ty<0) ty=0;
	               if (by>=height) by=height-1;
	               pixels = (bx-tx)*(by-ty);
	               minval = pixels*threshold/100;
	  
	               value=0;
	               p[0] = (int)source_img.getIntegral(tx,ty,x,y);
	               if (p[0]>minval)
	               {
	                   p[1] = (int)source_img.getIntegral(x,ty,bx,y);
		               if (p[1]>minval)
		               {
	                       p[2] = (int)source_img.getIntegral(tx,y,x,by);
		                   if (p[2]>minval)
		                   {
	                           p[3] = (int)source_img.getIntegral(x,y,bx,by);
			                   if (p[3]>minval)
			                   {
			                       value = (Math.Abs(p[0]-p[1]) + Math.Abs(p[2]-p[3]) + Math.Abs(p[0]-p[3]) + Math.Abs(p[0]-p[2]))/(pixels/2);
                                   value = value*value;
                                   if (value>255) value=255;
			                       value=255-value;
			                   }
		                   }
		               }
	               }
	               for (c=0;c<3;c++) image[x,y,c]=(Byte)value;
	           }
           }
       }



       //---------------------------------------------------------------------------------------------
       //show optical flow directions
       //---------------------------------------------------------------------------------------------
       public void showFlow()
       {
           int x,y,i,dx,dy,xx,yy,j;

           for (i=0;i<NoOfAttentionPoints;i++)
           {
               x = attentionPoint[i,0];
	           y = attentionPoint[i,1];
               dx = attentionPoint[i,3]*4;
	           dy = attentionPoint[i,4]*4;

	           if (!((dx==0) && (dy==0)))
	           {
  	               j=0;
	               if (Math.Abs(dx) > Math.Abs(dy))
	               {
	                   xx=x;
		               while (xx!=x+dx)
		               {
		                   yy = y + ((j*dy)/dx);
                           if ((xx>0) && (xx<width) && (yy>0) && (yy<height)) image[xx,yy,1]=255;
		                   if (dx>0) xx++; else xx--;
		                   j++;
		               }
	               }
	               else
	               {
	                   yy=y;
		               while (yy!=y+dy)
		               {
		                   xx = x + ((j*dx)/dy);                           
                           if ((xx>0) && (xx<width) && (yy>0) && (yy<height)) image[xx,yy,1]=255;
		                   if (dy>0) yy++; else yy--;
		                   j++;
		               }
	               }
	           }
           }
       }




       //---------------------------------------------------------------------------------------------
       //estimate the horizontal and vertical optical flow with a crude subtraction method
       //---------------------------------------------------------------------------------------------
       public void estimateMovement(classimage prev_img, int search_radius, ref int horizontal, ref int vertical)
       {
           int horizontal_search,vertical_search,half_horizontal,half_vertical;
           int x,y,xx,yy,c,w,h,diff,min_diff;
           const int increment=3;

           horizontal_search = (width*search_radius)/100;
           half_horizontal = horizontal_search/2;
           vertical_search = (height*search_radius)/100;
           half_vertical = vertical_search/2;
           w = width-horizontal_search-1;
           h = height-vertical_search-1;

           horizontal=0;
           vertical=0;
           min_diff=-1;
           for (x=0;x<horizontal_search;x+=increment)
           {
               for (y=0;y<vertical_search;y+=increment)
               {
	               diff=0;
	               for (xx=0;xx<w;xx+=increment)
	               {
	                   for (yy=0;yy<h;yy+=increment)
	                   {
		                   for (c=0;c<3;c++)
		                       diff += Math.Abs(image[x+xx,y+yy,c] - prev_img.image[half_horizontal+xx,half_vertical+yy,c]);
		               }
	               }
                   if ((min_diff==-1) || (diff<min_diff))
	               {
	                   min_diff=diff;
		               horizontal = x - half_horizontal;
		               vertical = y - half_vertical;
	               }
	           }
           }
           if (Math.Abs(horizontal)<2) horizontal=0;
           if (Math.Abs(vertical)<2) vertical=0;
       }


       //---------------------------------------------------------------------------------------------
       //finds maximal points within the image - used for attention system
       //---------------------------------------------------------------------------------------------
       public void findMaximalPoints(classimage source_img, int search_radius)
       {
           int i,j,k,x,y,tx,ty,bx,by,r,c,xx,yy,max,mx=0,my=0,dx,dy,tot_x,tot_y,hits,mindist,dist;
           int prev_NoOfAttentionPoints,flow_radius,winner;

           //store the previous attention points, so that they can be compared against
           //new ones to estimate direction of movement
           prev_NoOfAttentionPoints = NoOfAttentionPoints;
           for (i=0;i<prev_NoOfAttentionPoints;i++)
           for (c=0;c<3;c++) prev_attentionPoint[i,c] = attentionPoint[i,c];

           r = search_radius;

           //clear the image
           for (x=0;x<width;x++)
	           for (y=0;y<height;y++)
	               for (c=0;c<3;c++) image[x,y,c]=0;

           NoOfAttentionPoints=0;
           for (x=r;x<width-r;x+=r/2)
           {
	           for (y=r;y<height-r;y+=r/2)
	           {
                   tx = x-r;
	               ty = y-r;
	               bx = x+r;
	               by = y+r;

	               max=0;
	               for (xx=tx;xx<bx;xx++)
	               {
	                   for (yy=ty;yy<by;yy++)
		               {
		                   if (source_img.image[xx,yy,0]>max)
		                   {
                               mx = xx;
			                   my = yy;
			                   max = source_img.image[xx,yy,0];
		                   }
		               }
	               }
	               if (max>0) 
	               {
		               if (NoOfAttentionPoints<100)
		               {
		                   attentionPoint[NoOfAttentionPoints,0] = mx;
		                   attentionPoint[NoOfAttentionPoints,1] = my;
		                   attentionPoint[NoOfAttentionPoints,2] = 1;
                           NoOfAttentionPoints++;
		               }
	               }	  
	           }
           }
  

           mindist=width/6;
           for (i=0;i<NoOfAttentionPoints;i++)
           {
	           if (attentionPoint[i,2]==1)
	           {
                   mx = attentionPoint[i,0];
                   my = attentionPoint[i,1];
	               hits=1;
	               tot_x = mx;
	               tot_y = my;
                   for (j=0;j<NoOfAttentionPoints;j++)
	               {
	                   if ((i!=j) && (attentionPoint[j,2]==1))
		               {
                           x = attentionPoint[j,0];
                           y = attentionPoint[j,1];

                           dx = mx-x;
		                   dy = my-y;
		                   dist = (int)Math.Sqrt((dx*dx)+(dy*dy));
		                   if (dist<mindist)
		                   {
		                       attentionPoint[j,2]=0;
	                           tot_x += x;
	                           tot_y += y;
		                       hits++;
		                   }
		               }
	               }

                   mx = tot_x / hits;
	               my = tot_y / hits;


       	           attentionPoint[i,0] = mx;
	               attentionPoint[i,1] = my;

                   for (c=0;c<3;c++) image[mx,my,c]=255;
	           }
           }

           flow_radius=search_radius;
           j=0;
           for (i=0;i<NoOfAttentionPoints;i++)
           {
	           if (attentionPoint[i,2]==1)
	           {
	               for (c=0;c<3;c++) attentionPoint[j,c] = attentionPoint[i,c];

                   //find the most similar previous point
	               mindist=9999;
	               winner=-1;
	               for (k=0;k<prev_NoOfAttentionPoints;k++)
	               {
	                   dx = Math.Abs(attentionPoint[j,0] - prev_attentionPoint[k,0]);
		               dy = Math.Abs(attentionPoint[j,1] - prev_attentionPoint[k,1]);
		               if ((dx<flow_radius) && (dy<flow_radius))
		               {
		                   dist = dx + dy;
		                   if (dist<mindist)
		                   {
			                   mindist=dist;
			                   winner=k;
		                   }
		               }
	               }
	               //update the direction of movement of the attention point
	               if (winner>-1)
	               {
		               dx = attentionPoint[j,0] - prev_attentionPoint[winner,0];
		               dy = attentionPoint[j,1] - prev_attentionPoint[winner,1];
		               attentionPoint[j,3] = dx;
                       attentionPoint[j,4] = dy;
	               }
	               else
                   {
		               attentionPoint[j,3] = 0;
                       attentionPoint[j,4] = 0;
	               }

 	               j++;
	           }
           }
           NoOfAttentionPoints=j;
       }



       //---------------------------------------------------------------------------------------------
       //returns the attention point with the maximum response in the given source image
       //---------------------------------------------------------------------------------------------
       public int getMaxAttentionPoint(classimage source_img, int search_radius)
       {
           int i,x,y,c,winner,cx=0,cy=0,mx,my;
           long max,pixels,p;

           max=0;
           winner=-1;
           for (i=0;i<NoOfAttentionPoints;i++)
           {
	           mx = attentionPoint[i,0];
               my = attentionPoint[i,1];

	           source_img.CG(mx-search_radius,my-search_radius,mx+search_radius,my+search_radius,0,0,0,ref cx,ref cy,0);

	           attentionPoint[i,0] = cx;
               attentionPoint[i,1] = cy;

	           p=0;
	           pixels=1;
    
	           for (x=cx-search_radius;x<cx+search_radius;x++)
	           {
	               if ((x>=0) && (x<width))
	               {
	                   for (y=cy-search_radius;y<cy+search_radius;y++)
		               {
	                       if ((y>=0) && (y<height))
		                   {
			                   for (c=0;c<3;c++) p += source_img.image[x,y,c];
		                   }
		                   pixels++;
		               }
	               }
	           }

	           if (p>max)
	           {
	               max = p;
	               winner=i;
	           }
           }

           attentionWinner=winner;
           return(winner);
       }


       //---------------------------------------------------------------------------------------------
       //produces an image with the given attention point at its centre
       //---------------------------------------------------------------------------------------------
       public void attentionCentredImage(int attentionIndex, classimage source_img)
       {
           int x,y,c,xx,yy,dx,dy,w,h;

           if (NoOfAttentionPoints>attentionIndex)
           {
               w = width/6;
	           h = height/6;

               //clear the image
               for (x=0;x<width;x++)
	               for (y=0;y<height;y++)
	                   for (c=0;c<3;c++) image[x,y,c]=0;

	           if (attentionIndex>=0)
	           {
	               //select the given attention point
	               cx = attentionPoint[attentionIndex,0];
                   cy = attentionPoint[attentionIndex,1];
	           }
	           else
	           {
	               //randomly shift the attention point
	               cx += (int)(((Rnd()-0.5f)*width)/5);
	               cy += (int)(((Rnd()-0.5f)*height)/5);
	               if (cx<0) cx=0;
	               if (cx>=width) cx=width-1;
	               if (cy<0) cy=0;
	               if (cy>=height) cy=height-1;
	           }

	           clear();
               for (x=0;x<width;x++)
	           {
	               dx = x - cx;
	               xx = (width/2) + dx;
	               //if (xx>-1)
	               {
	                   for (y=0;y<height;y++)
		               {
		                   dy = y - cy;
	                       yy = (height/2) + dy;

		                   //if (yy>-1)
		                   {
			                   if ((xx>w) && (xx<width-w) && (yy>h) && (yy<height-h))
		                       for (c=0;c<3;c++) image[xx,yy,c] = source_img.image[x,y,c];
		                   }
		               }
	               }
	           }
           }
       }


       //---------------------------------------------------------------------------------------------
       //returns the average difference for the two images
       //---------------------------------------------------------------------------------------------
       public int averageDifference(classimage otherimage, int multiplier)
       {
           int x,y,c,diff,p1,p2,hits;
           int av=0;
           const int minval=0;

           hits=0;
           for (x=0;x<width;x++)
           {
	           for (y=0;y<height;y++)
	           {
	               p1=0;
	               p2=0;
	               for (c=0;c<3;c++)
	               {
		               p1 += image[x,y,c];
		               p2 += otherimage.image[x,y,c];
	               }
	               diff = Math.Abs((p1 * multiplier)-(p2 * multiplier));
	               if (diff>minval)
	               {
	                   //if (diff>255) diff=255;
	                   av+=diff;
	                   hits++;
	               }
	           }
           }
           if (hits>0) av /= hits; //else av=255;
           return(av);
       }




       //---------------------------------------------------------------------------------------------
       //adjust contrast
       //---------------------------------------------------------------------------------------------
       public void adjustContrast(classimage sourceimage, int contrastPercent, int centrepoint)
       {
           int x,y,c,diff,p;
           int illumination;
           int illumination_centre;

           if (centrepoint!=0)
               illumination_centre = centrepoint;
               else
               illumination_centre = sourceimage.getAverageIntensity(0,0,width,height);

           for (x=0;x<width;x++)
	           for (y=0;y<height;y++)
	           {
	               illumination=0;
	               for (c=0;c<3;c++) illumination += sourceimage.image[x,y,c];
	               illumination /=3;

                   diff = ((illumination - illumination_centre)*contrastPercent)/100;
	               for (c=0;c<3;c++)
	               {
	                   p = sourceimage.image[x,y,c];
	                   p += diff;
		               if (p<0) p=0;
		               if (p>255) p=255;
		               image[x,y,c]=(Byte)p;
	               }
	           }
       }


       //---------------------------------------------------------------------------------------------
       //subtracts two images to produce a difference image
       //---------------------------------------------------------------------------------------------
       public void subtractImages(classimage source1, classimage source2, int multiplier)
       {
           int x,y,c,diff,p1,p2;

           for (x=0;x<width;x++)
	           for (y=0;y<height;y++)
	               for (c=0;c<3;c++)
	               {
		               p1 = source1.image[x,y,c];
		               p2 = source2.image[x,y,c];
		               diff = Math.Abs(p1-p2);
	                   diff *= multiplier;
		               if (diff>255) diff=255;
		               image[x,y,c] = (Byte)diff;
	               }
       }


       //---------------------------------------------------------------------------------------------
       //subtracts two images to produce a difference image in mono only
       //---------------------------------------------------------------------------------------------
       public void subtractImagesMono(classimage source1, classimage source2, int multiplier,int minThreshold)
       {
           int x,y,c,diff,p1,p2,dx,dy;

           for (x=1;x<source1.width-1;x++)
           {
	           for (y=1;y<source1.height-1;y++)
	           {
	               p1=0;
	               p2=0;
	               for (dx=-1;dx<=1;dx++)
	               {
	                   for (dy=-1;dy<=1;dy++)
		               {
	                       for (c=0;c<3;c++)
		                   {
		                       p1 += source1.image[x+dx,y+dy,c];
		                       p2 += source2.image[x+dx,y+dy,c];
		                   }
		               }
	               }
	               p1 /= 9;
	               p2 /= 9;
	               diff=0;
	               if ((p1>minThreshold) || (p2>minThreshold))
	               {
	                   diff = (Math.Abs(p1-p2)*multiplier)/3;
	                   if (diff>255) diff=255;	   
	               }
	               for (c=0;c<3;c++) image[x,y,c] = (Byte)diff;
	           }
           }
       }


       //---------------------------------------------------------------------------------------------
       //copy from an image
       //---------------------------------------------------------------------------------------------
       public void copyImage(classimage source_img)
       {
           int x,y,c;
           Byte v;

           for (x=0;x<width;x++)
           {
	           for (y=0;y<height;y++)
	           {
	               for (c=0;c<3;c++)
	               {
		               v = source_img.image[x,y,c];
		               image[x,y,c] = v;
	               }
	           }
           }
       }



       //---------------------------------------------------------------------------------------------
       //returns a value indicating the texture of the given area in the range 0-255
       //---------------------------------------------------------------------------------------------
       public int texture(int tx, int ty, int bx, int by)
       {
           int texture=0,pixels=0;
           int x,y,c,dx,dy,p1,p2,xx,yy,dp;

           for (x=tx;x<bx;x++)
           {
	           if ((x>=0) && (x<width))
	           {
	               for (y=ty;y<by;y++)
	               {
		               if ((y>=0) && (y<height))
		               {
	                       p1=0;
                           for (c=0;c<3;c++) p1 += image[x,y,c];

	                       for (dx=-1;dx<=1;dx++)
		                   {
		                       xx = x+dx;
		                       if ((xx>=0) && (xx<width))
			                   {
		                           for (dy=-1;dy<=1;dy++)
			                       {		  
			                           yy = y+dy;
                                       if ((yy>=0) && (yy<height))
				                       {
			                               if (!((dx==0) && (dy==0)))
				                           {
			                                   p2=0;
	                                           for (c=0;c<3;c++) p2 += image[xx,yy,c];
					                           dp = Math.Abs(p1-p2);
			                                   texture += dp*20;
			                                   pixels++;
				                           }
				                       }
			                       }
			                   }
		                   }
		               }
	               }
	           }
           }

           if (pixels>0) texture /= (pixels*3);
           if (texture>255) texture=255;
           return(texture);
       }



       //---------------------------------------------------------------------------------------------
       //create a texture image
       //---------------------------------------------------------------------------------------------
       public void textureImage(classimage source_img)
       {
           int x,y,w,h,xx,yy,texture,c;

           w = source_img.width / width;
           if (w<1) w=1;
           h = source_img.height / height;
           if (h<1) h=1;

           for (x=0;x<width;x++)
           {
	           xx = (x*source_img.width)/width;
	           for (y=0;y<height;y++)
	           {
	               yy = (y*source_img.height)/height;
	               texture = source_img.texture(xx-w,yy-h,xx+w,yy+h);
	               for (c=0;c<3;c++) image[x,y,c] = (Byte)texture;
	           }
           }
       }


       //---------------------------------------------------------------------------------------------
       //produces a texture hisogram
       //---------------------------------------------------------------------------------------------
       public void textureHistogram(int areaSize, int[] histogram, int NoOfBuckets)
       {
           int x,y,w,h,xx,yy,t,bucket,max;

           w = width / (areaSize);
           if (w<1) w=1;
           h = height / (areaSize);
           if (h<1) h=1;

           for (bucket=0;bucket<NoOfBuckets;bucket++) histogram[bucket]=0;
           max=0;

           for (x=0;x<areaSize;x++)
           {
	           xx = (x*width)/areaSize;
	           for (y=0;y<areaSize;y++)
	           {
	               yy = (y*height)/areaSize;
	               t = texture(xx-w,yy-h,xx+w,yy+h);
	               bucket = t / (255/NoOfBuckets);
	               if (bucket>=NoOfBuckets) bucket=NoOfBuckets-1;
	               histogram[bucket]++;
	               if (histogram[bucket]>max) max = histogram[bucket];
	           }
           }

           if (max>0)
               for (bucket=0;bucket<NoOfBuckets;bucket++) histogram[bucket]=(histogram[bucket]*255)/max;
       }



       //---------------------------------------------------------------------------------------------
       //produces a colour hisogram
       //---------------------------------------------------------------------------------------------
       public void colourHistogram(int areaSize, int[] histogram, int NoOfBuckets)
       {
           int x,y,w,h,xx,yy,bucket,max,n;
           int av_r=0,av_g=0,av_b=0,c,p;
           int[] col = new int[3];
           int min_value=20;

           w = width / (areaSize);
           if (w<3) w=3;
           h = height / (areaSize);
           if (h<3) h=3;
 
           p=0;
           for (c=0;c<3;c++)
               for (bucket=0;bucket<NoOfBuckets;bucket++) 
	           {
	               histogram[p]=0;
	               p++;
               }

           max=0;
           n=0;

           for (x=0;x<areaSize;x++)
           {
	           xx = (x*width)/areaSize;
	           for (y=0;y<areaSize;y++)
	           {
	               yy = (y*height)/areaSize;
	               averageColourNonBlack(xx-w,yy-h,xx+w,yy+h,ref av_r,ref av_g,ref av_b);
                   col[0] = av_r;
	               col[1] = av_g;
	               col[2] = av_b;
	               //if (!((col[0]<min_value) && (col[1]<min_value) && (col[2]<min_value)))
	               if ((col[0]>min_value) || (col[1]>min_value) || (col[2]>min_value))
	               {
	                   for (c=0;c<3;c++)
		               {
	                       bucket = col[c] / (255/NoOfBuckets);
	                       if (bucket>=NoOfBuckets) bucket=NoOfBuckets-1;
		                   p = (c*NoOfBuckets) + bucket;
	                       histogram[p]++;
		                   n++;
	                       if (histogram[p]>max) max = histogram[p];
		               }
	               }
	           }
           }

           max = n/5;

           p=0;
           if (max>0)
	       for (c=0;c<3;c++)
               for (bucket=0;bucket<NoOfBuckets;bucket++) 
	           {
		           if (histogram[p]>max) histogram[p]=max;
	               histogram[p]=(histogram[p]*255)/max;
		           p++;
	           }
       }

       //---------------------------------------------------------------------------------------------
       //draw a line between two points in the given image
       //---------------------------------------------------------------------------------------------
       public void drawLine(int x1, int y1, int x2, int y2, int r, int g, int b, int linewidth)
       {
           int w,h,x,y,step_x,step_y,dx,dy,xx2,yy2;
           float m;

           dx=x2-x1;
           dy=y2-y1;
           w = Math.Abs(dx);
           h = Math.Abs(dy);
           if (x2>=x1) step_x=1; else step_x=-1;
           if (y2>=y1) step_y=1; else step_y=-1;        

           if (w>h)
           {
               if (dx!=0)
	           {
                   m = dy/(float)dx;
                   x=x1;
                   while (x!=x2+step_x)
                   {
                       y = (int)(m*(x-x1)) + y1;

                       for (xx2=x-linewidth;xx2<=x+linewidth;xx2++)
	                       for (yy2=y-linewidth;yy2<=y+linewidth;yy2++)
	                       {
                               if ((xx2>=0) && (xx2<width) && (yy2>=0) && (yy2<height))
		                       {
		                           image[xx2,yy2,2]=(Byte)b;
		                           image[xx2,yy2,1]=(Byte)g;
		                           image[xx2,yy2,0]=(Byte)r;
		                       }
		                   }

 	                   x += step_x;
                   }
	           }
           }
           else
           {
               if (dy!=0)
	           {
                   m = dx/(float)dy;
                   y=y1;
                   while (y!=y2+step_y)
                   {
                       x = (int)(m*(y-y1)) + x1; 
                       for (xx2=x-linewidth;xx2<=x+linewidth;xx2++)
                           for (yy2=y-linewidth;yy2<=y+linewidth;yy2++)
	                       {
                               if ((xx2>=0) && (xx2<width) && (yy2>=0) && (yy2<height))
	                           {
		                           image[xx2,yy2,2]=(Byte)b;
		                           image[xx2,yy2,1]=(Byte)g;
		                           image[xx2,yy2,0]=(Byte)r;
		                       }
	                       }

	                   y += step_y;
	               }
	           }
           }
       }


       //---------------------------------------------------------------------------------------------
       // colour constancy
       //---------------------------------------------------------------------------------------------
       public void colourConstancy(classimage sourceimg, int multiplier)
       {
           int x,y,c,p;
           int[] col= new int[3];

           for (x=0;x<width;x++)
               for (y=0;y<height;y++)
	           {
	               for (c=0;c<3;c++) col[c] = sourceimg.image[x,y,c];

	               p = Math.Abs(col[0]-col[1]);
	               p += Math.Abs(col[0]-col[2]);
	               p += Math.Abs(col[1]-col[2]);
	               p = (p*multiplier)/3;
	               if (p>254) p=254;
	               p = 255-p;
	               for (c=0;c<3;c++) image[x,y,c]=(Byte)p;
	           }
       }

       //---------------------------------------------------------------------------------------------
       // magnetize the image
       //---------------------------------------------------------------------------------------------
       public void magnetize(classimage left_image, classimage right_image, classimage temp_image, int radius)
       {
           int x, y, xx, x1=0, x2=0, diff1, diff2, c;
           bool found;

           radius=width/4;
           clear();
           for (y=1;y<height-1;y++)
           {
               for (x=1;x<width-1;x++)
               {
                   //min_distance=0;

	               xx=x+0;
	               found=false;
                   while ((!found) && (xx>0) && (x-xx<radius))
	               {
	                   if (xx<width)
		                   if (left_image.image[xx,y,0]>0) { found=true; x1=xx; }
		               xx--;
	               }
	               if (found)
	               {
	                   xx=x-0;
	                   found=false;
                       while ((!found) && (xx<width) && (xx-x<radius))
	                   {
		                   if (xx>0)
		                   if (right_image.image[xx,y,0]>0) { found=true; x2=xx; }
		                   xx++;
	                   }
		               if (found)
		               {
		                   //balance
		                   diff1 = (x2-x)-(x-x1);
		                   if (diff1<0) diff1=-diff1;
		                   diff1 = ((radius-diff1)*255)/radius;

		                   //distance
		                   diff2 = x2-x1;
		                   if (diff2<0) diff2=-diff1;
		                   diff2 = (diff2*diff2)/50;
		                   //diff2 = (radius-diff2);
		                   if (diff2>255) diff2=255;
		                   if (diff2<0) diff2=0;
		                   diff2 = (diff2*255)/radius;
		                   if (diff2>255) diff2=255;

                           diff1 = ((diff1*5) + (diff2*5))/10;

		                   //if (diff<254) diff=0;
		                   for (c=0;c<3;c++) image[x,y,c]=(Byte)diff1;
		               }
	               }
	           }
           }
       }

       //---------------------------------------------------------------------------------------------
       // convert the image from RGB to YUV
       //---------------------------------------------------------------------------------------------
       public void convertToYUV()
       {
           int xx,yy;
           float r,g,b,y,u,v;

           for (xx=0;xx<width;xx++)
	           for (yy=0;yy<height;yy++)
	           {
	               r = (float)image[xx,yy,0];
		           g = (float)image[xx,yy,1];
		           b = (float)image[xx,yy,2];
                   y = 0.257f*r + 0.504f*g + 0.098f*b + 16;
                   u = 0.439f*r - 0.368f*g - 0.071f*b + 128;
                   v = -0.148f*r - 0.291f*g + 0.439f*b + 128;
	               image[xx,yy,0] = (Byte)y;
		           image[xx,yy,1] = (Byte)u;
		           image[xx,yy,2] = (Byte)v;
	           }
       }


       //---------------------------------------------------------------------------------------------
       // simple stereo using a subtracted binary image
       //---------------------------------------------------------------------------------------------
       public void stereoSimple(classimage binaryimage, int max_disparity, classimage edges_left, classimage edges_right)
       {
           int x,y,start_x=0,cx,c,xx,max_width,x2,x3;
           int state,disparity,p;
           bool found_left,found_right;
           const int STATE_SEARCHING=0;
           const int STATE_FOUND=1;

           max_width=width/2;
           clear();
           for (y=1;y<height-1;y++)
           {
               state=STATE_SEARCHING;
               for (x=1;x<width-1;x++)
	           {
                   if ((binaryimage.image[x,y,0] > 0) && (state==STATE_SEARCHING))
	               {
	                   state=STATE_FOUND;
		               start_x=x;
	               }
                   if ((binaryimage.image[x+1,y,0] == 0) && (binaryimage.image[x,y,0] == 0) && (state==STATE_FOUND))
	               {
	                   state=STATE_SEARCHING;
		               cx = start_x + (((x-1)-start_x)/2);
		               disparity = cx - start_x;
		               p = (disparity*255)/max_disparity;

                       x2=x+((disparity*130)/100)+1;
		               found_right=false;
		               while ((!found_right) && (x2-x<max_width) && (x2<width))
		               {
		                   if (edges_right.image[x2,y,0] > 0)
		                       found_right=true;
		                       else
		                       x2++;
		               }

                       x3=x+((disparity*130)/100)+1;
		               found_left=false;
		               while ((!found_left) && (x3-x<max_width) && (x3<width))
		               {
		                   if (edges_left.image[x3,y,0] > 0)
		                       found_left=true;
		                       else
		                       x3++;
		               }
		               x3= (x3 - x)+start_x+disparity;

		               if ((found_left) || (found_right))
		               {
                           if (x3>x2) x2=x3;
		                   if (x2>=width) x2=width-1;
		                   for (xx=start_x;xx<x2;xx++) for (c=0;c<3;c++) image[xx,y,c]=(Byte)p;
		               }

	               }
	           }
           }
       }


       //---------------------------------------------------------------------------------------------
       // stereo matching
       //---------------------------------------------------------------------------------------------
       public void stereoMatch(classimage left_image_raw, classimage right_image_raw, classimage left_image, classimage left_image2, classimage left_image3, classimage left_image4, classimage right_image, classimage right_image2, classimage right_image3, classimage right_image4, int max_disparity, int weight_distance, int weight_colour, int weight_sequence, int match_threshold, int blobsize, int offset_x, int offset_y, int[,,] disparity_map)
       {
           int x,y,x2,xx,yy,c,dx,score,diff,i,xx2,yy2;
           int max_score,val1=0,val2=0,disp=0,prev_disp,prev_disp2;
           int[,] probableMatches = new int[20,2];
           int prob_index,pp,prob_hits,pp2,pp3,max_rays;
           long av_score,av_hits,quality_threshold,overall_max_score,max_diff;
           classimage image1;
           classimage image2;

           //the maximum number of rays per correlation search
           max_rays=3;

           av_score=0;
           av_hits=0;
           overall_max_score=0;
           image1 = left_image;
           image2 = right_image;
           for (y=0;y<height-1-offset_y;y++)
           {
               prev_disp=0;
	           prev_disp2=0;
               for (x=blobsize;x<width-blobsize-1;x++)
               {
	               if (image1.image[x+offset_x,y+offset_y,0]>0)
	               {
		               max_score=match_threshold*100;
		               max_diff=999999;

		               //clear probabilities
		               prob_hits=0;
		               prob_index=0;
		               for (pp=0;pp<20;pp++) probableMatches[pp,0]=-1;

	                   for (x2=x-max_disparity+offset_x;x2<x+max_disparity+offset_x;x2+=3)
	                   {
	                       if ((x2>0) && (x2<width))
		                   {                            
		                       score=0;
		                       if (image2.image[x2,y+offset_y,0]>0)
		                       {
		                           dx = x2-x;
		                           if (dx<0) dx=-dx;
			                       diff = (max_disparity-dx);
	                               score = (diff*diff*weight_distance)/max_disparity;

			                       //compare single points
			                       diff=0;
			                       for (c=0;c<3;c++)
				                   {
				                       //colour matching
				                       for (xx2=-1;xx2<1;xx2++)
				                           for (yy2=-1;yy2<1;yy2++)
					                       {
				                               if ((x+xx2>blobsize) && (x2+xx2>blobsize) && (y+yy2>0) && (y+yy2<height)) diff += Math.Abs(left_image_raw.image[x-blobsize+xx2,y+yy2,c] - right_image_raw.image[x2-blobsize+xx2,y+yy2,c]); else diff+=100;
				                               if ((x+xx2<width-blobsize) && (x2+xx2<width-blobsize) && (y+yy2>0) && (y+yy2<height)) diff += Math.Abs(left_image_raw.image[x+blobsize+xx2,y+yy2,c] - right_image_raw.image[x2+blobsize+xx2,y+yy2,c]); else diff+=100;
					                           if ((x+xx2>0) && (x2+xx2>0) && (y+yy2>0) && (x+xx2<width) && (y+yy2<height))
				                                   diff += Math.Abs(left_image_raw.image[x+xx2,y+yy2,c] - right_image_raw.image[x2+xx2,y+yy2,c]);
					                               else diff+=50;
					                       }
 			      
				                       //orientation matching
				                       diff += Math.Abs(image1.image[x+offset_x,y+offset_y,c] - image2.image[x2,y+offset_y,c]);
				                       diff += Math.Abs(left_image2.image[x+offset_x,y+offset_y,c] -right_image2.image[x2,y+offset_y,c]);
				                       if ((y>blobsize+offset_y) && (diff<max_diff))
				                       {
				                           diff += Math.Abs(image1.image[x+offset_x,y-blobsize+offset_y,c] - image2.image[x2,y-blobsize+offset_y,c]);
				                           diff += Math.Abs(left_image2.image[x+offset_x,y-blobsize+offset_y,c] -right_image2.image[x2,y-blobsize+offset_y,c]);
				                       } else val1+=50;
				                       if ((y<height-blobsize-offset_y) && (diff<max_diff))
				                       {
				                           diff += Math.Abs(image1.image[x+offset_x,y+blobsize+offset_y,c] - image2.image[x2,y+blobsize+offset_y,c]);
				                           diff += Math.Abs(left_image2.image[x+offset_x,y+blobsize+offset_y,c] -right_image2.image[x2,y+blobsize+offset_y,c]);
				                       } else val1+=50;
				                       if ((x2>=blobsize) && (y<height-offset_y) && (diff<max_diff))
				                       {
				                           diff += Math.Abs(image1.image[x+offset_x-blobsize,y+offset_y,c] - image2.image[x2-blobsize,y+offset_y,c]);
				                           diff += Math.Abs(left_image2.image[x+offset_x-blobsize,y+offset_y,c] -right_image2.image[x2-blobsize,y+offset_y,c]);
				                       } else val1+=50;
				                       if ((x+offset_x>=blobsize+blobsize) && (x2>=blobsize+blobsize) && (y<height-offset_y) && (diff<max_diff))
				                       {
				                           diff += Math.Abs(image1.image[x+offset_x-blobsize-blobsize,y+offset_y,c] - image2.image[x2-blobsize-blobsize,y+offset_y,c]);
				                           diff += Math.Abs(left_image2.image[x+offset_x-blobsize-blobsize,y+offset_y,c] -right_image2.image[x2-blobsize-blobsize,y+offset_y,c]);
				                       } else val1+=50;
				                       if ((x>=blobsize*3+offset_x) && (x2>=blobsize*3) && (y<height-offset_y) && (diff<max_diff)) 
				                       {
				                           diff += Math.Abs(image1.image[x+offset_x-blobsize-blobsize-blobsize,y+offset_y,c] - image2.image[x2-blobsize-blobsize-blobsize,y+offset_y,c]);
				                           diff += Math.Abs(left_image2.image[x+offset_x-blobsize-blobsize-blobsize,y+offset_y,c] -right_image2.image[x2-blobsize-blobsize-blobsize,y+offset_y,c]);
				                       } else val1+=50;
				                       if ((x<width-offset_x-blobsize) && (x2<width-blobsize) && (y<height-offset_y) && (diff<max_diff))
				                       {
					                       diff += Math.Abs(image1.image[x+offset_x+blobsize,y+offset_y,c] - image2.image[x2+blobsize,y+offset_y,c]);
				                           diff += Math.Abs(left_image2.image[x+offset_x+blobsize,y+offset_y,c] -right_image2.image[x2+blobsize,y+offset_y,c]);
				                       } else val1+=50;
				                       if ((x<width-blobsize-blobsize-offset_x) && (x2<width-blobsize-blobsize) && (y<height-offset_y) && (diff<max_diff))
				                       {
					                       diff += Math.Abs(image1.image[x+offset_x+blobsize+blobsize,y+offset_y,c] - image2.image[x2+blobsize+blobsize,y+offset_y,c]);
				                           diff += Math.Abs(left_image2.image[x+offset_x+blobsize+blobsize,y+offset_y,c] -right_image2.image[x2+blobsize+blobsize,y+offset_y,c]);
				                       } else val1+=50;
				                       if ((x<width-blobsize-blobsize-blobsize-offset_x) && (x2<width-blobsize-blobsize-blobsize) && (y<height-offset_y) && (diff<max_diff))
				                       {
					                       diff += Math.Abs(image1.image[x+offset_x+blobsize+blobsize+blobsize,y+offset_y,c] - image2.image[x2+blobsize+blobsize+blobsize,y+offset_y,c]);
				                           diff += Math.Abs(left_image2.image[x+offset_x+blobsize+blobsize+blobsize,y+offset_y,c] -right_image2.image[x2+blobsize+blobsize+blobsize,y+offset_y,c]);
				                       } else val1+=50;

				                   }

				                   if (diff<max_diff)
				                   {
				                       disp = (Math.Abs(x2-x)*255)/max_disparity;
				                       if (prev_disp<prev_disp2)
				                           val1 = Math.Abs(disp - prev_disp);
				                           else
				                           val1 = Math.Abs(disp - prev_disp2);
				                       prev_disp2=prev_disp;
				                       prev_disp=disp;
				                       val2 = diff + (val1*val1/4);

				                       score += (5000-val2)*weight_colour;
				                   }

				                   if (score>=max_score)
			                       {
				                       max_score = score;
				                       max_diff=diff;
				                       if (score>overall_max_score) overall_max_score=score;
				                       probableMatches[prob_index,0]=disp;
				                       probableMatches[prob_index,1]=score;
				                       prob_index++;
				                       prob_hits++;
				                       if (prob_index>=20) prob_index=0;
			                       }
			                   }
		                   }
		               }

		               //select the top probabilities
		               pp=prob_index-1;
		               pp2=prob_hits;
		               pp3=0;		 
		               while ((pp>-1) && (pp2>0) && (pp2>prob_hits-max_rays))
		               {
		                   score=probableMatches[pp,1];
			               av_score += score;
			               av_hits++;
 	                       for (xx=x;xx<x+blobsize;xx++)
 	                           for (yy=y;yy<y+blobsize;yy++)
			                   {
			                       //update the disparity map
				                   disparity_map[xx,yy,1+(pp3*2)]=probableMatches[pp,0];
				                   disparity_map[xx,yy,2+(pp3*2)]=score;
			                   }
		                   pp3++;
		                   pp2--;
			               pp--;
			               if (pp<0) pp = 19;
		               }

	                   for (xx=x;xx<x+blobsize;xx++)
 	                       for (yy=y;yy<y+blobsize;yy++)
		                       disparity_map[xx,yy,0]=pp3;
		           }
	           }
           }

           //calculate average matching score
           if (av_hits>0) av_score /= av_hits;

           //eliminate poor quality disparities
           clear();
           quality_threshold = 50000;
           //if (quality_threshold<240000) quality_threshold=240000;
           for (y=0;y<height;y++)
           {
               for (x=0;x<width;x++)
               {
	               max_score=-1;
	               for (i=0;i<disparity_map[x,y,0];i++)
	               {
	                   score = disparity_map[x,y,2+(2*i)];
		               //if ((score<quality_threshold) || (disparity_map[x,y,1+(2*i)]>255))
		               if (score<quality_threshold)
		               {
		                   disparity_map[x,y,1+(2*i)]=-1;
		                   //if (i==0)
		                   //  for (c=0;c<3;c++) image[x,y,c] = 0;
		               }
		               else
		               {
		                   disparity_map[x,y,2+(2*i)]=(int)((score*100)/overall_max_score);
		                   if (score>max_score)
		                   {
		                       for (c=0;c<3;c++) image[x,y,c] = (Byte)(disparity_map[x,y,1+(2*i)]);
			                   max_score=score;
		                   }
		               }
	               }
	           }
           }
           //clean up
           for (y=1;y<height-blobsize-1;y++)
           {
               for (x=1;x<width-blobsize-1;x++)
               {
                   if (image[x,y,0]>180)
	               {
	                   if (((image[x-1,y,0]<80) && (image[x+blobsize+1,y,0]<80)) ||
	 	                   ((image[x,y-1,0]<80) && (image[x,y+blobsize+1,0]<80)))
		               {
		                   for (c=0;c<3;c++) image[x,y,c] = 0;
		                   disparity_map[x,y,1]=0;
		               }
	               }
	           }
           }
       }



       //---------------------------------------------------------------------------------------------
       // produces a smoothed depth map using shape from shading
       //---------------------------------------------------------------------------------------------
       public void smoothDepthMap(classimage image_raw, classimage disparity)
       {
           int x,y,c,start_x,min_width,i;
           int localintensity,min_intensity,max_intensity;
           int start_depth,end_depth,min_depth,max_depth,depth,grad_x;
           int deviation,max_deviation,min_deviation,depth_grad_x;
           int min_grad_x,max_grad_x,min_depth_grad_x,max_depth_grad_x;

           clear();
           min_width=width/30;
           if (min_width<2) min_width=2;
           for (y=1;y<height;y++)
           {
               start_x=-1;
               for (x=0;x<width;x++)
	           {
	               if (disparity.image[x,y,0]>0)
	               {
	                   if (start_x==-1) start_x=x;
	               }
	               else
	               {
	                   if ((start_x>-1) && (x-start_x-1>min_width))
		               {
		                   //get the depth range
		                   start_depth = (disparity.image[start_x,y,0] + disparity.image[start_x+1,y,0])/2;
		                   end_depth = (disparity.image[x-1,y,0] + disparity.image[x-2,y,0])/2;
		  
		                   min_intensity=255;
		                   max_intensity=0;
		                   min_depth=255;
		                   max_depth=0;
		                   max_deviation=-999;
		                   min_deviation=999;
		                   max_grad_x=-999;
		                   min_grad_x=999;
		                   max_depth_grad_x=-999;
		                   min_depth_grad_x=999;
		                   grad_x=0;
		                   deviation=0;
		                   for (i=start_x;i<x;i++)
		                   {
		                       //get the local intensity of the raw image at this point
		                       localintensity=0;
			                   for (c=0;c<3;c++)
			                   {
		                           localintensity += image_raw.image[i,y,c];
			                       localintensity += image_raw.image[i,y-1,c];
			                   }         
			                   localintensity /= 6;
		                       //calculate gradient
		                       if (i>start_x)
			                   {
			                       grad_x = localintensity - (int)(disparity.image[i-1,y,1]);
			                       if (grad_x>max_grad_x) max_grad_x=grad_x;
			                       if (grad_x<min_grad_x) min_grad_x=grad_x;
			                       depth_grad_x = (int)(disparity.image[i,y,0]) - (int)(disparity.image[i-1,y,0]);
			                       if (depth_grad_x>max_grad_x) max_depth_grad_x=depth_grad_x;
			                       if (depth_grad_x<min_grad_x) min_depth_grad_x=depth_grad_x;
			                   }
			                   deviation += grad_x;
			                   if (deviation>max_deviation) max_deviation = deviation;
			                   if (deviation<min_deviation) min_deviation = deviation;
			                   //temporarily store the value in the disparity map for later use
			                   disparity.image[i,y,1]=(Byte)localintensity;
			                   //get the depth range
			                   depth = disparity.image[i,y,0];
			                   if (depth>max_depth) max_depth=depth;
			                   if (depth<min_depth) min_depth=depth;
			                   //get the intensity range
			                   if (localintensity>max_intensity) max_intensity=localintensity;
			                   if (localintensity<min_intensity) min_intensity=localintensity;
		                   }

		                   deviation=0;
	                       for (i=start_x;i<x;i++)
		                   {
		                       if (max_deviation>min_deviation)
		                       {
		                           //curved surface
			                       if (i>start_x)
			                       {
			                           //calculate gradient
			                           grad_x = (int)(disparity.image[i,y,1]) - (int)(disparity.image[i-1,y,1]);
				                       deviation += grad_x;
				                       grad_x = min_depth_grad_x + (((grad_x - min_grad_x)*(max_depth_grad_x-min_depth_grad_x))/(max_grad_x-min_grad_x));
				                       grad_x = ((grad_x - min_grad_x)*150)/(max_grad_x-min_grad_x);
				                       //this probably isn't right
				                       depth_grad_x = (int)(disparity.image[i,y,0]) - (int)(disparity.image[i-1,y,0]);
				                       depth = (int)(disparity.image[i-1,y,0]) + (((depth_grad_x*8) + (grad_x*2))/10);
			                       }
			                       else depth=start_depth;
			                   }
			                   else
			                   {
		                           //flat surface
			                       depth = start_depth + (((i-start_x)*(end_depth-start_depth))/(x-start_x-1));
			                   }
			                   if (depth<0) depth=0;
			                   if (depth>255) depth=255;
			                   for (c=0;c<3;c++) image[i,y,c]=(Byte)depth;
		                   }
		               }
		               start_x=-1;
	               }      
	           }
           }
       }


       //---------------------------------------------------------------------------------------------
       // colour normalisation
       //---------------------------------------------------------------------------------------------
       public void colourNormalise(classimage sourceimg, int multiplier)
       {
           int x,y,c,p,av_r=0,av_g=0,av_b=0;
           int[] col = new int[3];

           sourceimg.averageColour(0,0,width,height,ref av_r,ref av_g,ref av_b);

           for (x=0;x<width;x++)
               for (y=0;y<height;y++)
	           {
	               for (c=0;c<3;c++) col[c] = sourceimg.image[x,y,c];

	               col[0] -= av_r;
	               col[1] -= av_g;
	               col[2] -= av_b;

	               for (c=0;c<3;c++)
	               {
	                   p = 127 + (col[c]*multiplier);
	                   if (p<1) p=1;
	                   if (p>255) p=255;
		               image[x,y,c]=(Byte)p;
	               }
	           }
       }


       //---------------------------------------------------------------------------------------------
       // show a histogram within the image
       //---------------------------------------------------------------------------------------------
       public void showHistogram(int[] histogram, int NoOfBuckets)
       {
           int bucket,x,y,c,w,h;

           //clear the image
           for (x=0;x<width;x++)
               for (y=0;y<height;y++)
	               for (c=0;c<3;c++) image[x,y,c]=0;

           //show the histogram levels
           w = width / NoOfBuckets;
           for (bucket=0;bucket<NoOfBuckets;bucket++)
           {
	           h = (histogram[bucket]*height)/255;
	           for (x=(w*bucket);x<(w*(bucket+1));x++)
               {
	               for (y=height-1;y>height-h-1;y--)
	               {
		               for (c=0;c<3;c++) image[x,y,c]=255;
	               }
               }
           }
       }


       public void smoothImage(classimage sourceimg, int regionSize, int multiplier)
       {
           const int max_mask_size = 10;
           int x,y,xx,yy,c;
           int dx,dy,dist,pixels;
           int[] tot = new int[3];
           int halfRegion = regionSize/2;
           int[] mask = new int[max_mask_size*max_mask_size];
           int[,] distLookup= new int[max_mask_size,max_mask_size];
           float angle,fraction;
           int i1;

           if (halfRegion<max_mask_size)
           {
               //create the mask
	           for (dist=0;dist<=halfRegion;dist++)
	           {
	               angle = (dist*3.1415927f) / halfRegion;
	               fraction = ((float)Math.Cos(angle) + 1)/2.0f;
	               fraction *= 1.3f;
	               fraction -= 0.3f;
	               mask[dist] = (int)(255 * fraction);
	           }

               for (xx=0;xx<regionSize;xx++)
	           {
	               dx = xx - halfRegion;
                   for (yy=0;yy<regionSize;yy++)
	               {
	                   dy = yy - halfRegion;
	                   distLookup[xx,yy] = (int)Math.Sqrt((dx*dx)+(dy*dy));
	               }
	           }


               //filter using the mask
               for (x=0;x<width;x++)
                   for (y=0;y<height;y++)
	               {
	                   pixels=0;
	                   for (c=0;c<3;c++) tot[c]=0;
		               dx=0;
	                   for (xx=x-halfRegion;xx<x+halfRegion;xx++)
	                   {
	                       if ((xx>=0) && (xx<width))
		                   {
		                       dy=0;
	                           for (yy=y-halfRegion;yy<y+halfRegion;yy++)
	                           {
	                               if ((yy>=0) && (yy<height))
		                           {
			                           //dy = yy - (y-halfRegion);
				                       dist = distLookup[dx,dy];

				                       for (c=0;c<3;c++)
				                       {
				                           i1 = sourceimg.image[xx,yy,c];
				                           tot[c] += (i1 * mask[dist]);
				                       }
				                       pixels++;
			                       }
			                       dy++;
		                       }
		                   }
                           dx++;
	  	               }

		           for (c=0;c<3;c++)
		           {
	                   tot[c] /= (pixels*255);
		               if (tot[c]<0) tot[c]=0;
		               tot[c] = (tot[c] * multiplier)/10;
		               if (tot[c]>255) tot[c]=255;
		               image[x,y,c] = (Byte)tot[c];
		           }

	           }
	       }
       }

    }
}
