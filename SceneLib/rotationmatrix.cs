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
using System.Collections.Generic;
using System.Text;

namespace SceneLibrary
{
    public class RotationMatrix : MatrixFixed
    {
        public RotationMatrix() : base(3,3)
        {
            SetIdentity();
        }
        
        public RotationMatrix(RotationMatrix R) : base(3,3)
        {
            this.Equals(R);
        }
        

        public RotationMatrix(Vector3D axis, float angle) : base(3,3)
        {
            Set(axis, angle);
        }
        /*
        public RotationMatrix(AngleAxis angleaxis) : base(3,3)
        {
            Set(angleaxis);
        }
        */
        public RotationMatrix(Quaternion q) : base(3,3)
        {
            Set(q);
        }

        /// <summary>
        /// Note that this can be a costly operation, as the matrix is
        /// checked for orthonormality using the SVD. If the matrix also
        /// includes a reflection (ie. has negative determinant) then
        /// it is impossible to tell which row or col needs inverting
        /// and the function will abort.
        /// </summary>
        /// <param name="M">the matrix to convert to a rotation matrix</param>      
        /*
        public RotationMatrix(MatrixFixed M) : base(3,3)
        {
            Set(M);
        }
        */

        public Quaternion Quaternion()
        {
            return (new Quaternion(this));
        }

        public AngleAxis AngleAxis()
        {
            return (new AngleAxis(this));
        }

        public RotationMatrix Inverse()
        {
            return ((RotationMatrix)(Transpose()));
        }


        public override MatrixFixed Transpose()
        {
            RotationMatrix R = new RotationMatrix();
            R[0, 0] = this[0, 0]; R[0, 1] = this[1, 0]; R[0, 2] = this[2, 0];
            R[1, 0] = this[0, 1]; R[1, 1] = this[1, 1]; R[1, 2] = this[2, 1];
            R[2, 0] = this[0, 2]; R[2, 1] = this[1, 2]; R[2, 2] = this[2, 2];

            return R;
        }


        public void Set(Quaternion q)
        {
            this.Equals(q.RotationMatrix());
        }


        public void Set(Vector3D angleaxis)
        {
            float angle = angleaxis.Norm();
            Set(angleaxis, angle);
        }

        /// <summary>
        /// Note that the axis doesn't have to be a unit vector.
        /// </summary>
        /// <param name="axis">the axis around which to rotate</param>
        /// <param name="angle">the angle of rotation in radians</param>
        public void Set(Vector3D axis, float angle)
        {
            // start with an identity matrix.
            this.SetIdentity();

            // Identity is all that can be deduced from zero angle
            if (angle == 0)
                return;

            // normalize to a unit vector u
            float[] u = new float[3];
            float norm;
            norm = 1.0f / axis.Norm();
            u[0]=axis.GetX()*norm;
            u[1]=axis.GetY()*norm;
            u[2]=axis.GetZ()*norm;
  
            // add (cos(angle)-1)*(1 - u u').
            float cos_angle = (float)Math.Cos(angle);
            for (uint i=0; i<3; ++i)
                for (uint j=0; j<3; ++j)
                    this[(int)i,(int)j] += (cos_angle-1) * ((i==j ? 1:0) - u[i]*u[j]);

            // add sin(angle) * [u]
            float sin_angle = (float)Math.Sin(angle);
            this[0,1] -= sin_angle*u[2]; 
            this[0,2] += sin_angle*u[1];
            this[1,0] += sin_angle*u[2];
            this[1,2] -= sin_angle*u[0];
            this[2,0] -= sin_angle*u[1]; 
            this[2,1] += sin_angle*u[0];
        }

        /// <summary>
        /// Note that this can be a costly operation, as the matrix is
        /// checked for orthonormality using the SVD. If the matrix also
        /// includes a reflection (ie. has negative determinant) then
        /// it is impossible to tell which row or col needs inverting
        /// and the function will abort.
        /// </summary>
        /// <param name="M">the matrix to convert to a rotation matrix</param>
        /*
        public void Set(MatrixFixed M)
        {
            // If M is even close to being a rotation it should have
            // determinant close to 1. Negative determinant means a
            // row or col is inverted
            float d = Determinant(M);  //VNL

            if (d < 0) 
            {
                Debug.WriteLine("VW::RotationMatrix::Set(MatrixFixed<3,3>& M): Error - determinant of M is negative (ie. one or more rows/cols are inverted). Impossible to tell which one, so aborting.");
                //exit(0);
            }

            // Doing SVD and setting all singular values to 1 gives the
            // rotation matrix closest to M in the Mahalanobis distance sense.
            SVD<float> svd(M);   //VNL

            this = svd.U() * svd.V().Transpose();
        }
        */

        public void SetFromXYZEuler(float theta_x,
					         float theta_y,
					         float theta_z)
        {
            float sx = (float)Math.Sin(theta_x), cx = (float)Math.Cos(theta_x);
            float sy = (float)Math.Sin(theta_y), cy = (float)Math.Cos(theta_y);
            float sz = (float)Math.Sin(theta_z), cz = (float)Math.Cos(theta_z);
            this[0,0] =            cy*cz;   
            this[0,1] = -cx*sz +sx*sy*cz;   
            this[0,2] =  sx*sz +cx*sy*cz; 

            this[1,0] =            cy*sz;   
            this[1,1] =  cx*cz +sx*sy*sz;   
            this[1,2] = -sx*cz +cx*sy*sz;
 
            this[2,0] =   -sy;   
            this[2,1] = sx*cy;   
            this[2,2] = cx*cy; 
        }


        public void SetFromZXYEuler(float theta_z, float theta_x, float theta_y)
        {
            float sx = (float)Math.Sin(theta_x), cx = (float)Math.Cos(theta_x);
            float sy = (float)Math.Sin(theta_y), cy = (float)Math.Cos(theta_y);
            float sz = (float)Math.Sin(theta_z), cz = (float)Math.Cos(theta_z);
            this[0,0] =  cy*cz +sx*sy*sz;   
            this[0,1] = -cy*sz +sx*sy*cz;   
            this[0,2] =  cx*sy; 

            this[1,0] =  cx*sz;   
            this[1,1] =  cx*cz;   
            this[1,2] = -sx;
 
            this[2,0] = -sy*cz +sx*cy*sz;   
            this[2,1] =  sy*sz +sx*cy*cz;   
            this[2,2] =  cx*cy; 
        }

        public void SetFromYXZEuler(float theta_y, float theta_x, float theta_z)
        {
            float sx = (float)Math.Sin(theta_x), cx = (float)Math.Cos(theta_x);
            float sy = (float)Math.Sin(theta_y), cy = (float)Math.Cos(theta_y);
            float sz = (float)Math.Sin(theta_z), cz = (float)Math.Cos(theta_z);
            this[0,0] =  cy*cz -sx*sy*sz;   
            this[0,1] = -cx*sz;   
            this[0,2] =  sy*cz +sx*cy*sz;   

            this[1,0] =  cy*sz +sx*sy*cz;   
            this[1,1] =  cx*cz;   
            this[1,2] =  sy*sz -sx*cy*cz;   
 
            this[2,0] = -cx*sy; 
            this[2,1] =  sx;
            this[2,2] =  cx*cy; 
        }


        public void SetFromZYXEuler(float theta_z, float theta_y, float theta_x)
        {
            float sx = (float)Math.Sin(theta_x), cx = (float)Math.Cos(theta_x);
            float sy = (float)Math.Sin(theta_y), cy = (float)Math.Cos(theta_y);
            float sz = (float)Math.Sin(theta_z), cz = (float)Math.Cos(theta_z);
            this[0,0] =  cy*cz;   
            this[0,1] = -cy*sz;   
            this[0,2] =  sy;   

            this[1,0] =  cx*sz +sx*sy*cz;   
            this[1,1] =  cx*cz -sx*sy*sz;   
            this[1,2] =        -sx*cy;   
 
            this[2,0] =  sx*sz -cx*sy*cz; 
            this[2,1] =  sx*cz +cx*sy*sz;
            this[2,2] =         cx*cy; 
        }

        public void SetFromZXZEuler(float theta_z1, float theta_x, float theta_z2)
        {
            float sz1 = (float)Math.Sin(theta_z1), cz1 = (float)Math.Cos(theta_z1);
            float sx = (float)Math.Sin(theta_x), cx = (float)Math.Cos(theta_x);
            float sz2 = (float)Math.Sin(theta_z2), cz2 = (float)Math.Cos(theta_z2);
            this[0,0] =  cz1*cz2 -cx*sz1*sz2;   
            this[0,1] = -sz1*cz2 -cx*cz1*sz2;   
            this[0,2] =  sx*sz2; 

            this[1,0] =  cz1*sz2 +cx*sz1*cz2;   
            this[1,1] = -sz1*sz2 +cx*cz1*cz2;   
            this[1,2] = -sx*cz2;
 
            this[2,0] =  sx*sz1;   
            this[2,1] =  sx*cz1;   
            this[2,2] =  cx; 
        }

        /// <summary>
        /// Repeatedly multiplying rotations together can lead to round-off error
        /// which causes the rotation to become invalid. This function uses the
        /// SVD to force orthonormality, giving the closest valid rotation matrix
        /// in a Mahalanobis distance sense. Call it every now and then if you're 
        /// worried about errors accumulating.
        /// </summary>
        public void Normalise()
        {
            // Doing SVD and setting all singular values to 1 gives the
            // rotation matrix closest to M in the Mahalanobis distance sense.
            SVD svd = new SVD(this);

            // Only update if the SVD was successful (sometimes fails to converge
            // if already within precision of solution)
            if (!svd.Valid()) 
            {
                Debug.WriteLine("VW::RotationMatrix::Normalise(): SVD failed.");
            } 
            else 
            {
                //this = svd.U() * svd.V().Transpose();
            }
        }
        

        public float Angle()
        {
            float angle = 0;

            float d0 = this[0,0], d1 = this[1,1], d2 = this[2,2];

            // The trace determines the method of decomposition
            float rr = 1.0f + d0 + d1 + d2;

            if (rr > 0) 
            {
                angle = (float)Math.Acos(Math.Sqrt(rr));
            } 
            else 
            {
                // Trace is less than zero, so need to determine which
                // major diagonal is largest
                if ((d0 > d1) && (d0 > d2)) 
                {
                    angle = 2 * (float)Math.Acos(0.5 * (this[1, 2] + this[2, 1]) / (float)Math.Sqrt(1 + d0 - d1 - d2));
                } 
                else 
                    if (d1 > d2) 
                    {
                        angle = 2 * (float)Math.Acos(0.5 * (this[0, 2] + this[2, 0]) / (float)Math.Sqrt(1 + d0 - d1 - d2));
                    } 
                    else 
                    {
                        angle = 2 * (float)Math.Acos(0.5 * (this[0, 1] + this[1, 0]) / (float)Math.Sqrt(1 + d0 - d1 - d2));
                    } 
            }
            return angle;  
        }

        public Vector3D Axis()
        {
            Vector3D axis = new Vector3D(0,0,0);

            float d0 = this[0,0], d1 = this[1,1], d2 = this[2,2];

            // The trace determines the method of decomposition
            float rr = 1.0f + d0 + d1 + d2;

            if (rr > 0) 
            {
                axis.SetX( this[2,1] - this[1,2] );
                axis.SetY( this[0,2] - this[2,0] );
                axis.SetZ( this[1,0] - this[0,1] );
            } 
            else 
            {
                // Trace is less than zero, so need to determine which
                // major diagonal is largest
                if ((d0 > d1) && (d0 > d2)) 
                {
                    axis.SetX( 0.5f );
                    axis.SetY( this[0,1] + this[1,0] );
                    axis.SetZ( this[0,2] + this[2,0] );
                } 
                else 
                    if (d1 > d2) 
                    {
                        axis.SetX( this[0,1] + this[1,0] );
                        axis.SetY( 0.5f );
                        axis.SetZ( this[1,2] + this[2,1] );
                    } 
                    else 
                    {
                        axis.SetX( this[0,2] + this[2,0] );
                        axis.SetY( this[1,2] + this[2,1] );
                        axis.SetZ( 0.5f );
                    }
            }
            axis.Normalise();

            return (axis);
        }

 
        public Vector3D Rotate(Vector3D v)
        {
            return 
                new Vector3D(this[0,0]*v.GetX()+this[0,1]*v.GetY()+this[0,2]*v.GetZ(),
		                     this[1,0]*v.GetX()+this[1,1]*v.GetY()+this[1,2]*v.GetZ(),
		                     this[2,0]*v.GetX()+this[2,1]*v.GetY()+this[2,2]*v.GetZ());		      
        }

        public Vector3D InverseRotate(Vector3D v)
        {
            return 
                new Vector3D(this[0,0]*v.GetX()+this[1,0]*v.GetY()+this[2,0]*v.GetZ(),
		                     this[0,1]*v.GetX()+this[1,1]*v.GetY()+this[2,1]*v.GetZ(),
              		         this[0,2]*v.GetX()+this[1,2]*v.GetY()+this[2,2]*v.GetZ());		      
        }

    }



}
