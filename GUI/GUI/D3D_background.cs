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
using System.Drawing;
using Microsoft.DirectX;
using Microsoft.DirectX.Direct3D;

namespace WindowsApplication1
{
    /// <summary>
    /// This is a basic Class that will be used to display a
    /// background image on a directX Device.
    /// Used for 2D Drawing on a D3D Device.
    /// </summary>
    public class D3D_background
    {
        // Full path and file name to be used as the background
        private string backgroundImagePath = "";
        private Texture backgroundSpriteTexture;
        private Sprite backgroundSprite;
        private Rectangle backgroundTextureSize;

        /// <summary>
        /// Sets up the background Object
        /// </summary>
        /// <param name="device">The DirectX Device that the Background will be assigned to.</param>
        /// <param name="BackgroundImagePath">The Full path and filename to the Image file to be used as the background</param>
        public D3D_background(Device device, string BackgroundImagePath)
        {
            backgroundImagePath = BackgroundImagePath;
            // Load in the background texture from the file and assign it to
            // the sprite
            backgroundSpriteTexture = TextureLoader.FromFile(device, backgroundImagePath);
            using (Surface s = backgroundSpriteTexture.GetSurfaceLevel(0))
            {
                SurfaceDescription desc = s.Description;
                backgroundTextureSize = new Rectangle(0, 0, desc.Width, desc.Height);
            }
            backgroundSprite = new Sprite(device);
        }

        /// <summary>
        /// Sets up the background Object
        /// </summary>
        /// <param name="device">The DirectX Device that the Background will be assigned to.</param>
        /// <param name="image">Bitmap object containing the image</param>
        public D3D_background(Device device, Bitmap image)
        {
            backgroundImagePath = BackgroundImagePath;
            // Load in the background texture from a bitmap object and assign it to
            // the sprite.  Turning the bitmap into a stream is crude, but looks
            // like the only way to do it
            MemoryStream loMS = new MemoryStream();
            image.Save(loMS, System.Drawing.Imaging.ImageFormat.Bmp);
            loMS.Seek(0, 0); // give the texture loader a "clear run" at the stream
            backgroundSpriteTexture = TextureLoader.FromStream(device, loMS);
            using (Surface s = backgroundSpriteTexture.GetSurfaceLevel(0))
            {
                SurfaceDescription desc = s.Description;
                backgroundTextureSize = new Rectangle(0, 0, desc.Width, desc.Height);
            }
            backgroundSprite = new Sprite(device);
        }


        public void update(Device device, Bitmap image)
        {
            // Load in the background texture from a bitmap object and assign it to
            // the sprite.  Turning the bitmap into a stream is crude, but looks
            // like the only way to do it
            MemoryStream loMS = new MemoryStream();
            image.Save(loMS, System.Drawing.Imaging.ImageFormat.Bmp);
            loMS.Seek(0, 0); // give the texture loader a "clear run" at the stream
            backgroundSpriteTexture = TextureLoader.FromStream(device, loMS);
        }


        /// <summary>
        /// Full file name and Path to the Image
        /// to be used as the background image.
        /// </summary>
        public string BackgroundImagePath
        {
            get { return backgroundImagePath; }
            set { backgroundImagePath = value; }
        }
        /// <summary>
        /// Draw the Background to the Device
        /// </summary>
        public void Draw()
        {
            try
            {
                backgroundSprite.Begin(SpriteFlags.AlphaBlend);
                backgroundSprite.Draw(backgroundSpriteTexture, backgroundTextureSize,
                                      new Vector3(0, 0, 0), new Vector3(0, 0, 1), Color.White);
                backgroundSprite.End();
            }
            catch
            {
            }
        }
    }

}
