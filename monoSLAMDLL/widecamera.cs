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
    /// A model of a pinhole camera with cubic distortion, able to project image rays
    /// into the camera image, and vice versa. This is the basic camera type, but
    /// different, more or less specific, camera types could also be derived from this.

    /// The distortion model used in this camera is the Swaminathan and Nayar
    /// (2000) model, which is invertible to a good approximation:
    /// \f[
    /// \begin{aligned}
    /// \vct{u}_{\text{image}} - \vct{u}_0 &=
    /// \frac{\vct{u}_{\text{ideal}} - \vct{u}_0}{\sqrt{1+2 k_1 r^2}} \\
    /// \vct{u}_{\text{ideal}} - \vct{u}_0 &=
    /// \frac{\vct{u}_{\text{image}} - \vct{u}_0}{\sqrt{1-2 k_1 r^2}}
    /// \end{aligned}
    /// \f]
    /// where  \vct{u}_0  is the centre of distortion (assumed to be the camera
    /// centre  (u_0, v_0) , and  r = |\vct{u}_{\text{ideal}} -
    /// \vct{u}_0|  or  r = |\vct{u}_{\text{image}} - \vct{u}_0|  as
    /// appropriate.
    /// </summary>
    public class WideCamera
    {      
        const uint DEFAULT_IMAGE_WIDTH = 320;
        const uint DEFAULT_IMAGE_HEIGHT = 240;
        const float DEFAULT_FKU = 195;        //160;//195.0f;
        const float DEFAULT_FKV = 195;        //160;//195.0f;
        const float DEFAULT_U0 = 162;         //162.0f;
        const float DEFAULT_V0 = 125;         //125.0f;
        const float DEFAULT_K1 = 0.000009f;
        const float DEFAULT_MEASUREMENT_SD = 1.0f;

        private void init()
        {
            m_centre = new Vector(2);
            m_C = new MatrixFixed(3, 3);
            m_Cinv = new MatrixFixed(3, 3);
            m_last_camera = new Vector(3);
            m_last_image_centred = new Vector(2);
        }

        /// <summary>
        /// Default constructor. If this constructor is used, read_parameters() should be
        /// used to set the parameters.
        /// </summary>
        public WideCamera()
        {
            init();
            m_image_width = DEFAULT_IMAGE_WIDTH;
            m_image_height = DEFAULT_IMAGE_WIDTH;
            m_Kd1 = DEFAULT_K1;
            m_Fku = DEFAULT_FKU;
            m_Fkv = DEFAULT_FKV;
            m_measurement_sd = DEFAULT_MEASUREMENT_SD;

            m_centre[0] = DEFAULT_U0;
            m_centre[1] = DEFAULT_V0;
  
            MakeProjectionMatrices();
        }


        public void set_calibration(float Kd1, float Fku, float Fkv,
                          float U0, float V0, float measurement_sd)
        {
            m_Kd1 = Kd1;
            m_Fku = Fku;
            m_Fkv = Fkv;
            m_measurement_sd = measurement_sd;

            m_centre[0] = U0;
            m_centre[1] = V0;

            MakeProjectionMatrices();
        }

        public WideCamera(int w, int h, float Kd1, float Fku, float Fkv,
                          float U0, float V0, float measurement_sd)
        {
            init();
            m_image_width = (uint)w;
            m_image_height = (uint)h;
            set_calibration(Kd1, Fku, Fkv, U0, V0, measurement_sd);
        }

        /// <summary>
        /// Form the projection matrix  \mat{C}  and its inverse.
        /// \f[
        /// \begin{aligned}
        /// \mat{C} &= \emat{-F_{ku} & 0 & U_0 \\
        ///         0 & -F_{kv} & V_0 \\
        ///         0 & 0 & 1}
        /// \mat{C}^{-1} &= \emat{-\frac{1}{F_{ku}} & 0 & \frac{U_0}{F_{ku}} \\
        ///         0 & -\frac{1}{F_{kv}} & \frac{V_0}{F_{kv}} \\
        ///         0 & 0 & 1}
        /// \end{aligned}
        /// </summary>
        public void MakeProjectionMatrices()
        {
            // Camera calibration matrix and inverse
            float[] m = {-m_Fku,  0.0f,  m_centre[0],
                  0.0f, -m_Fkv,  m_centre[1],
                  0.0f,  0.0f, 1.0f};
            m_C.CopyIn(m);
            float[] n = {-1.0f / m_Fku,   0.0f,        m_centre[0] / m_Fku,
                    0.0f,        -1.0f / -m_Fkv,  m_centre[1] / m_Fkv,
                    0.0f,         0.0f,        1.0f};
            m_Cinv.CopyIn(n);
        }

        /// <summary>
        /// Project a point from an ideal (Euclidean) camera frame into image co-ordinates
        /// with radial distortion.
        ///
        /// This first transforms the input position vector  vct{y}  (relative to
        /// the camera position) into an undistorted image location  \vct{u}_c 
        /// (relative to an origin at the optical centre)
        /// 
        ///     vct{u}_c = evct{u_c \ v_c} = evct{-F_{ku} y_x / y_z \ -F_{kv} y_y / y_z} =
        ///     emat{-F_{ku} & 0 \ 0 & -F_{kv}}\evct{y_x/y_z \ y_y/y_z}
        /// 
        /// Then radial distortion is applied to give the final image location  h 
        /// (with the origin back in the normal location of of the top left of the image).
        /// 
        ///     vct{h} = frac{vct{u}_c}{sqrt{1 + 2 k_1 |vct{u}_c|^2}} + evct{u_0 \ v_0}
        /// </summary>
        /// <param name="camera"></param>
        /// <returns></returns>
        public virtual Vector Project(Vector camera)
        {
            // Remember this position in case we are asked about the Jacobians of this
            // transformation
            m_last_camera = camera;
  
            // First do perspective projection
            // This turns the position vector into an undistorted image location (with the
            // origin at the centre)
            // Use -Fkuv to swap the image co-ordinates (0,0) at the top to
            // camera co-ordinates (0,0) at the bottom

            Vector imagepos_centred = new Vector(2);
            imagepos_centred[0] = -m_Fku * camera[0] / camera[2];
            imagepos_centred[1] = -m_Fkv * camera[1] / camera[2];

            m_last_image_centred = imagepos_centred;

            // 1 distortion coefficient model
            float radius2 = imagepos_centred.SquaredMagnitude();
            float factor = (float)Math.Sqrt(1 + 2 * m_Kd1 * radius2);
            return imagepos_centred / factor + m_centre;
        }


        /// <summary>
        /// Project a point from image co-ordinates into the ideal (Euclidean) camera frame.
        ///
        /// The origin for the image point  \vct{h}  is moved to the optical centre
        /// and the inverse distortion applied to give the undistorted location
        ///  \vct{u}_c :
        /// 
        /// \begin{aligned}
        /// \vct{u}_0 &= \evct{u_0 \\ v_0} \\
        /// \vct{u}_c = \evct{u_c \\ v_c} &=
        /// \frac{\vct{h} - \vct{u}_0}{\sqrt{1 - 2 k_1 |\vct{h} - \vct{u}_0|^2}}
        /// \end{aligned}
        /// 
        /// This is then unprojected to give the world location
        ///  \vct{y} = \evct{y_x & y_y & 1}^T  in homogeneous co-ordinates.
        ///
        /// \vct{y} = \evct{\frac{1}{-F_{ku}} u_c \\ \frac{1}{-F_{kv}} v_c \\ 1}
        /// </summary>
        /// <param name="image"></param>
        /// <returns></returns>
        public virtual Vector Unproject(Vector image)
        {
            Vector centred;
            centred = image - m_centre;

            m_last_image_centred = centred;

            float radius2 = centred.SquaredMagnitude();
            float factor = (float)Math.Sqrt(1 - 2 * m_Kd1 * radius2);

            Vector undistorted = centred / factor;

            Vector camera = new Vector(3);
  
            camera[0] = undistorted[0] / -m_Fku;
            camera[1] = undistorted[1] / -m_Fkv;
            camera[2] = 1.0f;
  
            return camera;
        }


        /// <summary>
        /// Calculate the Jacobian  \partfrac{\vct{h}}{\vct{y}}  for the
        /// Project() operation, for the most recent point that was projected.

        /// This is calculated in two stages:
        /// \f[
        /// \begin{aligned}
        /// \partfrac{ \vct{u} }{ \vct{y} } &= \emat{
        /// \frac{ -f_{ku} }{ y_z } & 0 & \frac{ f_{ku} y_x }{ y_z^2 } \\
        /// 0 & \frac{ -f_{kv} }{ y_z } & \frac{ f_{kv} y_y }{ y_z^2 } } \\
        /// \partfrac{\vct{h}}{\vct{u}} &= \emat{
        /// -\frac{ 2 u_c^2 k_1 }{ f^{\frac{3}{2}} } +
        /// \frac{ 1 }{ f^{\frac{1}{2}} } &
        /// -\frac{ 2 u_c v_c k_1 }{ f^{\frac{3}{2}} } \\
        /// -\frac{ 2 v_c u_c k_1 }{ f^{\frac{3}{2}} } &
        /// -\frac{ 2 v_c^2 k_1 }{ f^{\frac{3}{2}} } +
        /// \frac{ 1 }{ f^{\frac{1}{2}} } } =
        /// -\frac{2 k_1}{f^\frac{3}{2}} \vct{u}_c \vct{u}_c^T +
        /// \emat{\frac{1}{\sqrt{f}} & 0 \\
        /// 0 & \frac{1}{\sqrt{f}}}
        /// \end{aligned}
        /// 
        /// where  f = 1 + 2 k_1 |\vct{u}_c|^2 . The final Jacobian is then given by
        /// combining these two
        /// 
        /// \partfrac{\vct{h}}{\vct{y}} =
        /// \partfrac{\vct{h}}{\vct{u}} \partfrac{\vct{u}}{\vct{y}}
        /// </summary>
        /// <returns></returns>
        public virtual MatrixFixed ProjectionJacobian()
        {  
            // Jacobians
            // Normal image measurement
            float fku_yz = m_Fku/m_last_camera[2];
            float fkv_yz = m_Fkv/m_last_camera[2];
            float[] a =
                         {-fku_yz,
                          0.0f,
                          fku_yz * m_last_camera[0] / m_last_camera[2],
                          0.0f,
                          -fkv_yz,
                          fkv_yz * m_last_camera[1] / m_last_camera[2]};
            MatrixFixed du_by_dy = new MatrixFixed(2,3,a);
  
            // Distortion model Jacobians
            // Generate the outer product matrix first
            MatrixFixed dh_by_du = m_last_image_centred.AsColumn() * m_last_image_centred.AsRow();

            // this matrix is not yet dh_by_du, it is just
            // [ uc*uc  uc*vc ]
            // [ vc*uc  vc*vc ]
            // The trace of this matrix gives the magnitude of the vector
            float radius2 = dh_by_du[0,0] + dh_by_du[1,1];
            // Calculate various constants to save typing
            float distor = 1 + 2 * m_Kd1 * radius2;
            float distor1_2 = (float)Math.Sqrt(distor);
            float distor3_2 = distor1_2 * distor;

            // Now form the proper dh_by_du by manipulating the outer product matrix
            dh_by_du *= -2 * m_Kd1 / distor3_2;
            dh_by_du[0,0] += (1.0f / distor1_2);
            dh_by_du[1,1] += (1.0f / distor1_2);

            return dh_by_du * du_by_dy;
        }


        /// <summary>
        /// Calculate the Jacobian  \partfrac{\vct{y}}{\vct{h}}  for the
        /// Unproject() operation, for the most recent point that was projected.
        /// 
        /// This is calculated in two stages: the Jacobian between undistorted and
        /// distorted image co-ordinates
        /// 
        /// \partfrac{\vct{u}}{\vct{h}} = \emat{
        /// \frac{ 2 u_c^2 k_1 }{ f^{\frac{3}{2}} } +
        /// \frac{ 1 }{ f^{\frac{1}{2}} } &
        /// \frac{ 2 u_c v_c k_1 }{ f^{\frac{3}{2}} } \\
        /// \frac{ 2 v_c u_c k_1 }{ f^{\frac{3}{2}} } &
        /// \frac{ 2 v_c^2 k_1 }{ f^{\frac{3}{2}} } +
        /// \frac{ 1 }{ f^{\frac{1}{2}} } } =
        /// \frac{2 k_1}{f^\frac{3}{2}} \vct{u}_c \vct{u}_c^T +
        /// \emat{\frac{1}{\sqrt{f}} & 0 \\
        /// 0 & \frac{1}{\sqrt{f}}}
        ///
        /// where  f = 1 - 2 k_1 |\vct{u}_c|^2  and the Jacobian for the unprojection
        /// operation between image rays and (undistorted, centred) image co-ordinates
        ///
        /// \partfrac{ \vct{y} }{ \vct{u} } = \emat{
        /// -\frac{ 1 }{ f_{ku} } & 0 \\
        /// 0 & -\frac{ 1}{ f_{kv}} \\
        /// 0 & 0 } 
        ///
        /// The final Jacobian is then given by combining these two
        ///
        /// \partfrac{\vct{y}}{\vct{h}} =
        /// \partfrac{\vct{y}}{\vct{u}} \partfrac{\vct{u}}{\vct{h}}
        /// </summary>
        /// <returns></returns>
        public virtual MatrixFixed UnprojectionJacobian()
        {
            float[] a = {-1/m_Fku, 0.0f,
                          0.0f, -1/m_Fkv,
                          0.0f, 0.0f};
            MatrixFixed dy_by_du = new MatrixFixed(3,2,a);
  
            // Generate the outer product matrix first
            MatrixFixed du_by_dh = m_last_image_centred.AsColumn() * m_last_image_centred.AsRow();
            // this matrix is not yet du_by_dh, it is just
            // [ uc*uc  uc*vc ]
            // [ vc*uc  vc*vc ]
            // The trace of this matrix gives the magnitude of the vector
            float radius2 = du_by_dh[0,0] + du_by_dh[1,1];
            // Calculate various constants to save typing
            float distor = 1 - 2 * m_Kd1 * radius2;
            float distor1_2 = (float)Math.Sqrt(distor);
            float distor3_2 = distor1_2 * distor;

            // Now form the proper du_by_dh by manipulating the outer product matrix
            du_by_dh *= 2 * m_Kd1 / distor3_2;
            du_by_dh[0,0] += (1/distor1_2);
            du_by_dh[1,1] += (1/distor1_2);

            return dy_by_du * du_by_dh;
        }


        /// <summary>
        /// Calculate the image position measurement noise at this location.
        /// </summary>
        /// <param name="h">The image location.  This is not constant across the image. It has the value of m_measurement_sd at the centre, increasing with radial distance to 2*m_measurement_sd at the corners</param>
        /// <returns></returns>
        public virtual MatrixFixed MeasurementNoise(Vector h)
        {
            // Distance of point we are considering from image centre
            float distance = (h - m_centre).Magnitude();
            float max_distance = m_centre.Magnitude();
            float ratio = distance / max_distance; // goes from 0 to 1

            float SD_image_filter_to_use = m_measurement_sd * (1.0f + ratio);

            float measurement_noise_variance = SD_image_filter_to_use * SD_image_filter_to_use;
  
            // RiRES is diagonal
            MatrixFixed noise = new MatrixFixed(2,2);
            noise.SetIdentity(measurement_noise_variance);

            return noise;
        }


  
        /// What is the image width?
        public uint ImageWidth() { return m_image_width; }
        /// What is the image height?
        public uint ImageHeight() { return m_image_height; }
  
        /// First radial distrotion coefficient
        public float Kd1() { return m_Kd1; } 
        /// Focal length  \times  pixel density in  u  image direction.
        public float Fku() { return m_Fku; } 
        /// Focal length  \times  pixel density in  v  image direction.
        public float Fkv() { return m_Fkv; }
        /// Optical centre
        public Vector Centre() { return m_centre; }
        /// Optical centre in  u  image direction.
        public float U0() { return m_centre[0]; }
        /// Optical centre in  v  image direction.
        public float V0() { return m_centre[1]; }   
        // Measurement standard deviation. This is the noise at the centre of
        // the image - see  MeasurementNoise() for a description of how this is used. */
        public float MeasurementNoise() { return m_measurement_sd; }
  
        /// What is the camera calibration matrix?
        public MatrixFixed C() { return m_C; }
        /// What is the inverse camera calibration matrix?
        public MatrixFixed Cinv() { return m_Cinv; }

        // Image size
        private uint m_image_width;  ///< Image width
        private uint m_image_height; ///< Image height
        // Camera calibration parameters
        public float m_Kd1; ///< First radial distortion coefficient.
        public float m_Fku; ///< Focal length  \times  pixel density in  u  image direction.
        public float m_Fkv; ///< Focal length  \times  pixel density in  v  image direction.
        // Principal point
        public Vector m_centre;  ///< Optical centre

        /// The standard deviation of image measurement noise
        public float m_measurement_sd;

        public MatrixFixed m_C;    ///< The camera calibration matrix
        public MatrixFixed m_Cinv; ///< The inverse camera calibration matrix

        // The most recent camera point that was projected. (Stored so that we can
        // return the Jacobian for this transformation if it is asked for immediately
        // afterwards). 
        public Vector m_last_camera;
        public Vector m_last_image_centred;


        /// <summary>
        /// Write the camera parameters to a stream in the format
        /// Width Height U0 V0 Fku Fkv Kd1 SD
        /// </summary>
        /// <param name="stream"></param>
        public void Write(StreamWriter stream)
        {
            stream.WriteLine(WriteASCII());
        }

        public String WriteASCII()
        {
            return(Convert.ToString(ImageWidth()) + "," +
                   Convert.ToString(ImageHeight()) + "," +
                   Convert.ToString(U0()) + "," +
                   Convert.ToString(V0()) + "," +
                   Convert.ToString(Fku()) + "," +
                   Convert.ToString(Fkv()) + "," +
                   Convert.ToString(Kd1()) + "," +
                   Convert.ToString(MeasurementNoise()));
        }


        /// <summary>
        /// Read camera parameters from file
        /// </summary>
        /// <param name="stream"></param>
        public void Read(StreamReader stream)
        {
            String line = stream.ReadLine();
            ReadASCII(line);
        }

        public void ReadASCII(String line)
        {
            String[] values = line.Split(',');

            m_image_width = Convert.ToUInt32(values[0]);
            m_image_height = Convert.ToUInt32(values[1]);
            m_centre[0] = Convert.ToSingle(values[2]);
            m_centre[1] = Convert.ToSingle(values[3]);
            m_Fku = Convert.ToSingle(values[4]);
            m_Fkv = Convert.ToSingle(values[5]);
            m_Kd1 = Convert.ToSingle(values[6]);
            m_measurement_sd = Convert.ToSingle(values[7]);
        }

    }

}

