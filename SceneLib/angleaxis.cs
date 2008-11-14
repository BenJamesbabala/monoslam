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
using System.Collections.Generic;
using System.Text;

namespace SceneLibrary
{
    public class AngleAxis
    {
        protected float _x,_y,_z;


        public AngleAxis()
        {
            _x = 0;
            _y = 0;
            _z = 0;
        }

        public AngleAxis(float x,float y,float z)
        {
            _x = x;
            _y = y;
            _z = z;
        }

        public AngleAxis(Vector3D axis, float angle)
        {
            Set(axis,angle);
        }

        public AngleAxis(Vector3D angleaxis)
        {
            Set(angleaxis);
        }

        public AngleAxis(RotationMatrix R)
        {
            Set(R);
        }

        public AngleAxis(Quaternion q)
        {
            Set(q);
        }

        public void Set(float x,float y,float z)
        {
            _x = x;
            _y = y;
            _z = z;
        }

        public float GetX()
        {
            return(_x);
        }

        public float GetY()
        {
            return(_y);
        }

        public float GetZ()
        {
            return(_z);
        }


        public void Get(ref float x, ref float y, ref float z)
        {
            x = _x;
            y = _y;
            z = _z;
        }

        // returns a reference to the first imaginary part of the quaternion 
        public float X()
        {
            return _x;
        }

        // returns a reference to the second imaginary part of the quaternion 
        public float Y()
        {
            return _y;
        }

        // @returns a reference to the third imaginary part of the quaternion 
        public float Z()
        {
            return _z;
        }


        public AngleAxis Multiply (float scale)
        {
            AngleAxis aa = this;
            aa.Multiply(scale);
            return aa;
        }

        public AngleAxis Divide (float scale)
        {
            AngleAxis aa = this;
            aa.Divide(scale);
            return aa;
        }

        public AngleAxis Inverse()
        {
            return(new AngleAxis(-_x, -_y, -_z));
        }


        public bool Equals (AngleAxis aa)
        {
            return ((aa._x == _x) && (aa._y == _y) && (aa._z == _z));
        }

        public bool NotEqualTo (AngleAxis aa)
        {
            return ((aa._x != _x) || (aa._y != _y) || (aa._z != _z));
        }



        /// <summary>
        /// 
        /// </summary>
        /// <param name="axis">the axis (doesn't need to be a unit vector) around which to rotate</param>
        /// <param name="angle">the rotation angle in radians</param>
        public void Set(Vector3D axis, float angle)
        {
            _x = angle * axis.GetX();
            _y = angle * axis.GetY();
            _z = angle * axis.GetZ();
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="angleaxis">the 3D vector whose size gives the angle and direction</param>
        public void Set(Vector3D angleaxis)
        {
            _x = angleaxis.GetX();
            _y = angleaxis.GetY();
            _z = angleaxis.GetZ();
        } 

        /// <summary>
        /// 
        /// </summary>
        /// <param name="R">the rotation matrix from which the quaternion will be extracted</param>
        public void Set(RotationMatrix R)
        {
            // The trace determines the angle
            float diag = 0.5f*(R[0,0] + R[1,1] + R[2,2] - 1.0f);
            if (diag >= 1.0) 
            {
                // zero angle
                _x = _y = _z = 0.0f;
                return;
            } 
            else 
            {
                float angle = (float)Math.Acos(diag);
                _x = R[2,1] - R[1,2];
                _y = R[0,2] - R[2,0];
                _z = R[1,0] - R[0,1];
                float axisnorm = (float)Math.Sqrt(_x * _x + _y * _y + _z * _z);
                if (axisnorm < 1e-8) 
                {
                    // Axis is undefined
                    _x = _y = _z = 0.0f;
                    return;
                } 
                else 
                {
                    // All OK, so scale it
                    float scale = angle/axisnorm;
                    _x *= scale;
                    _y *= scale;
                    _z *= scale;
                }
            }
        }

        public void Set(Quaternion q)
        {
            float r = q.GetR();
            _x = q.GetX();
            _y = q.GetY();
            _z = q.GetZ();
            float norm = (float)Math.Sqrt(_x * _x + _y * _y + _z * _z);
            if (norm > 0) 
            {
                float scaling = 2.0f * (float)Math.Atan2(norm, r) / norm;
                _x *= scaling;
                _y *= scaling;
                _z *= scaling;
            }
        }

        public Quaternion Quaternion()
        {
            float angle = (float)Math.Sqrt(_x * _x + _y * _y + _z * _z);
            if (angle < 1e-8) 
            {
                return new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
            }
            float s = (float)Math.Sin(angle / 2.0) / angle;
            float c = (float)Math.Cos(angle / 2.0f);
            return new Quaternion(s*_x,s*_y,s*_z,c);
        }

        public RotationMatrix RotationMatrix()
        {
            // start with an identity matrix.
            RotationMatrix R = new RotationMatrix();
            float angle = (float)Math.Sqrt(_x * _x + _y * _y + _z * _z);

            // Identity is all that can be deduced from zero angle
            if (angle == 0)
                return R;

            float c = (float)Math.Cos(angle) - 1.0f;
            float s = (float)Math.Sin(angle);

            // normalize axis to a unit vector u
            float norm = 1.0f/angle;
            float nx = _x*norm;
            float ny = _y*norm;
            float nz = _z*norm;
  
            // add (cos(angle)-1)*(1 - u u').
            R[0,0] += c* (nx*nx - 1.0f);
            R[0,1] += c*  nx*ny;
            R[0,2] += c*  nx*nz;

            R[1,0] += c*  ny*nx;
            R[1,1] += c* (ny*ny - 1.0f);
            R[1,2] += c*  ny*nz;

            R[2,0] += c*  nz*nx;
            R[2,1] += c*  nz*ny;
            R[2,2] += c* (nz*nz - 1.0f);

            // add sin(angle) * [u]
            R[0,1] -= s*nz; R[0,2] += s*ny;
            R[1,0] += s*nz;
            R[1,2] -= s*nx;
            R[2,0] -= s*ny; R[2,1] += s*nx;

            return(R);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns>the rotation matrix corresponding to the inverse rotation by this quaternion</returns>
        public RotationMatrix RotationMatrixTranspose()
        {
            // start with an identity matrix.
            RotationMatrix R = new RotationMatrix();
            float angle = (float)Math.Sqrt(_x * _x + _y * _y + _z * _z);

            // Identity is all that can be deduced from zero angle
            if (angle == 0)
                return R;

            float c = (float)Math.Cos(angle) - 1.0f;
            float s = (float)Math.Sin(angle);

            // normalize axis to a unit vector u
            float norm = 1.0f/angle;
            float nx = _x*norm;
            float ny = _y*norm;
            float nz = _z*norm;
 
            // add (cos(angle)-1)*(1 - u u').
            R[0,0] += c* (nx*nx - 1.0f);
            R[0,1] += c*  nx*ny;
            R[0,2] += c*  nx*nz;

            R[1,0] += c*  ny*nx;
            R[1,1] += c* (ny*ny - 1.0f);
            R[1,2] += c*  ny*nz;

            R[2,0] += c*  nz*nx;
            R[2,1] += c*  nz*ny;
            R[2,2] += c* (nz*nz - 1.0f);

            // add sin(angle) * [u]
            R[0,1] += s*nz; 
            R[0,2] -= s*ny;
            R[1,0] -= s*nz; 
            R[1,2] += s*nx;
            R[2,0] += s*ny; 
            R[2,1] -= s*nx;

            return(R);
        }

        // @returns the angle of rotation in radians.
        public float Angle()
        {
            return ((float)Math.Sqrt(_x * _x + _y * _y + _z * _z));
        }


        // @returns the axis of rotation. 
        public Vector3D Axis()
        {
            float ang = (float)Math.Sqrt(_x * _x + _y * _y + _z * _z);
            if (ang < 1e-8) 
            {
                return (new Vector3D(0, 0, 0));
            }
            float scale = 1.0f/ang;
            return (new Vector3D(_x * scale, _y * scale, _z * scale));
        }

        // Rotate a vector.
        // @param v the vector to be rotated
        // @returns the resulting rotated vector
        public Vector3D Rotate(Vector3D v)
        {
            float angle = (float)Math.Sqrt(_x * _x + _y * _y + _z * _z);
            if (angle < 1e-8) 
            {
                return v;
            }
            float sn = (float)Math.Sin(angle);
            float cs = (float)Math.Cos(angle);
            Vector3D axis = new Vector3D(_x / angle, _y / angle, _z / angle);
            Vector3D n = new Vector3D(Point3D.VectorProduct(axis, v));
            Vector3D delta = new Vector3D(sn * n + (cs-1.0f) * (Point3D.VectorProduct(n, axis)));
            return (new Vector3D(v + delta));
        }

        // Rotate a line segment.
        // @param l the line segment to be rotated
        // @returns the resulting rotated line segment
        public LineSeg3D Rotate(LineSeg3D l)
        {
            // Simply rotate both lines
            float angle = (float)Math.Sqrt(_x * _x + _y * _y + _z * _z);
            if (angle < 1e-8) 
            {
                return l;
            }
            float sn = (float)Math.Sin(angle);
            float cs = (float)Math.Cos(angle);
            Vector3D axis = new Vector3D(_x / angle, _y / angle, _z / angle);
            Vector3D ns = new Vector3D(Point3D.VectorProduct(axis, l.StartPoint()));
            Vector3D ne = new Vector3D(Point3D.VectorProduct(axis, l.EndPoint()));
            Vector3D deltas = new Vector3D(sn * ns + (cs - 1.0f) * (Point3D.VectorProduct(ns, axis)));
            Vector3D deltae = new Vector3D(sn * ne + (cs - 1.0f) * (Point3D.VectorProduct(ne, axis)));
            return new LineSeg3D(l.StartPoint() + deltas, l.EndPoint() + deltae);
        }


        // Combine two rotations (this is VERY costly).
        // @param aa the inner rotation
        // @returns the resulting rotation (like this*q for matrices)
        public AngleAxis Multiply (AngleAxis aa)
        {
            // This is best done as quaternions
            Quaternion q1 = new Quaternion(this);
            Quaternion q2 = new Quaternion(aa);
            return new AngleAxis(q1.Multiply(q2));
        }


    }

}
