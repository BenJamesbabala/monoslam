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


namespace SceneLibrary
{
    /// <summary>
    /// This quaternion class represents a 3D rotation as a complex vector
    /// with three imaginary parts x,y,z and real part r: q = ix + jy + kz + r.
    /// Whilst designed for simlarity to the (partially incorrect) VNL quaternion
    /// class, it provides additional functions for rotating VW::Vector3D's and for
    /// initialisation from and recovery of Euler angles.
    /// </summary>
    public class Quaternion
    {
        // Construct
        public Quaternion()
        {
            _x = 0;
            _y = 0;
            _z = 0;
            _r = 1;
        }

        public Quaternion(float x, float y, float z, float r)
        {
            // Make sure that this is a unit vector
            _x = x;
            _y = y;
            _z = z;
            _r = r;
            Normalise();
        }


        public Quaternion(Point3D axis)
        {
            Set(axis, 0);
        }

        public Quaternion(Point3D axis, float angle)
        {
            Set(axis, angle);
        }

        public Quaternion(AngleAxis angleaxis)
        {
            Set(angleaxis);
        }

        public Quaternion(RotationMatrix R)
        {
            Set(R);
        }

        public void ReadASCII(String line)
        {
            String[] values = line.Split(',');
            _x = Convert.ToSingle(values[0]);
            _y = Convert.ToSingle(values[1]);
            _z = Convert.ToSingle(values[2]);
            _r = Convert.ToSingle(values[3]);
        }

        public void ReadASCII(StreamReader stream)
        {
            String line = stream.ReadLine();
            ReadASCII(line);
        }

        public String WriteASCII()
        {
            String output_str = Convert.ToString(_x) + "," +
                                Convert.ToString(_y) + "," +
                                Convert.ToString(_z) + "," +
                                Convert.ToString(_r);
            return (output_str);
        }

        public void WriteASCII(StreamWriter stream)
        {
            stream.WriteLine(WriteASCII());
        }


        /// <summary>
        /// 
        /// </summary>
        /// <param name="axis">the axis (doesn't need to be a unit vector) around which to rotate</param>
        /// <param name="angle">the rotation angle in radians</param>
        public void Set(Point3D axis, float angle)
        {
            if (Math.Abs(angle) < 1e-8)
            {
                _x = 0.0f;
                _y = 0.0f;
                _z = 0.0f;
                _r = 1.0f;
            }
            else
            {
                float a = angle / 2.0f;  // half angle
                float s = (float)Math.Sin(a);
                float c = (float)Math.Cos(a);

                // Make sure the axis is a unit vector
                float n = 1.0f / axis.Norm();

                _x = s * axis.GetX() * n;  // half angle multiplied with axis
                _y = s * axis.GetY() * n;  // half angle multiplied with axis
                _z = s * axis.GetZ() * n;  // half angle multiplied with axis
                _r = c;                    // real part is cosine of half angle
            }
        }

        /// <summary>
        /// The angle is recovered as angleaxis = angle and
        /// the axis as axis = angleaxis / angle. 
        /// </summary>
        /// <param name="angleaxis">the 3D vector whose size gives the angle and direction </param>        
        void Set(AngleAxis angleaxis)
        {
            float x = angleaxis.GetX();
            float y = angleaxis.GetY();
            float z = angleaxis.GetZ();
            float angle = (float)Math.Sqrt(x*x + y*y + z*z);
            if (angle > 0.0) 
            {
                float s = (float)Math.Sin(angle/2.0)/angle;
                float c = (float)Math.Cos(angle/2.0f);
                _x = s * x;
                _y = s * y;
                _z = s * z;
                _r = c;
            } 
            else 
            {
                _x = _y = _z = 0.0f;
                _r = 1.0f;
            }
        }        


        //param R the rotation matrix from which the quaternion will be extracted
        public void Set(RotationMatrix R)
        {
            float d0 = R[0,0], d1 = R[1,1], d2 = R[2,2];

            // The trace determines the method of decomposition
            float rr = 1.0f + d0 + d1 + d2;

            if (rr > 0) 
            {
                float s = 0.5f / (float)Math.Sqrt(rr);
                _x = (R[2,1] - R[1,2]) * s;
                _y = (R[0,2] - R[2,0]) * s;
                _z = (R[1,0] - R[0,1]) * s;
                _r = 0.25f / s;
            } 
            else 
            {
                // Trace is less than zero, so need to determine which
                // major diagonal is largest
                if ((d0 > d1) && (d0 > d2)) 
                {
                    float s = 0.5f / (float)Math.Sqrt(1 + d0 - d1 - d2);
                    _x = 0.5f * s;
                    _y = (R[0,1] + R[1,0]) * s;
                    _z = (R[0,2] + R[2,0]) * s;
                    _r = (R[1,2] + R[2,1]) * s;
                } 
                else 
                    if (d1 > d2) 
                    {
                        float s = 0.5f / (float)Math.Sqrt(1 + d0 - d1 - d2);
                        _x = (R[0,1] + R[1,0]) * s;
                        _y = 0.5f * s;
                        _z = (R[1,2] + R[2,1]) * s;
                        _r = (R[0,2] + R[2,0]) * s;
                    } 
                    else 
                    {
                        float s = 0.5f / (float)Math.Sqrt(1 + d0 - d1 - d2);
                        _x = (R[0,2] + R[2,0]) * s;
                        _y = (R[1,2] + R[2,1]) * s;
                        _z = 0.5f * s;
                        _r = (R[0,1] + R[1,0]) * s;
                    }
            }
        }

        public RotationMatrix RotationMatrix()
        {
            RotationMatrix R = new RotationMatrix();

            float x2 = _x * _x;
            float y2 = _y * _y;
            float z2 = _z * _z;
            float r2 = _r * _r;
            R[0,0] = r2 + x2 - y2 - z2;         // fill diagonal terms
            R[1,1] = r2 - x2 + y2 - z2;
            R[2,2] = r2 - x2 - y2 + z2;
            float xy = _x * _y;
            float yz = _y * _z;
            float zx = _z * _x;
            float rx = _r * _x;
            float ry = _r * _y;
            float rz = _r * _z;
            R[0,1] = 2 * (xy - rz);             // fill off diagonal terms
            R[0,2] = 2 * (zx + ry);
            R[1,0] = 2 * (xy + rz);
            R[1,2] = 2 * (yz - rx);
            R[2,0] = 2 * (zx - ry);
            R[2,1] = 2 * (yz + rx);

            return(R);
        }

        // @returns the rotation matrix corresponding to the inverse rotation by this
        // quaternion.
        public RotationMatrix RotationMatrixTranspose()
        {
            RotationMatrix R = new RotationMatrix();

            float x2 = _x * _x;
            float y2 = _y * _y;
            float z2 = _z * _z;
            float r2 = _r * _r;
            R[0,0] = r2 + x2 - y2 - z2;         // fill diagonal terms
            R[1,1] = r2 - x2 + y2 - z2;
            R[2,2] = r2 - x2 - y2 + z2;
            float xy = _x * _y;
            float yz = _y * _z;
            float zx = _z * _x;
            float rx = _r * _x;
            float ry = _r * _y;
            float rz = _r * _z;
            R[0,1] = 2 * (xy + rz);             // fill off diagonal terms
            R[0,2] = 2 * (zx - ry);
            R[1,2] = 2 * (yz + rx);
            R[1,0] = 2 * (xy - rz);
            R[2,0] = 2 * (zx + ry);
            R[2,1] = 2 * (yz - rx);

            return(R);
        }


        // @returns the angle of rotation in radians
        public float Angle()
        {
            return (2.0f * (float)Math.Atan2(Math.Sqrt(_x*_x + _y*_y + _z*_z), _r));
        }

        // @returns a vector whose magnitude gives the angle of rotation and direction 
        //  gives the rotation axis 
        public AngleAxis AngleAxis()
        {
            float a = Angle();
            if (a > 0.0) 
            {
                float scaling = a / (float)Math.Sin(a / 2.0f);
                return new AngleAxis(_x*scaling, _y*scaling, _z*scaling);
            }
            return new AngleAxis(0,0,0);
        }


        // @returns the axis of rotation. */
        public Vector3D Axis()
        {
            Vector3D _i = Imaginary();
            float l = _i.Norm();
            if (l < 1e-8) 
            {
                _i.Set(0.0f, 0.0f, 0.0f);
            }
            else 
            {
                Point3D _i2 = (Point3D)_i / l;
                _i = new Vector3D(_i2);
            }
            return _i;
        }

        // @returns the imaginary part (sin(theta/2)*(x,y,z)) of the quaternion */
        public Vector3D Imaginary()
        {
            //return( (Vector3D)(new Point3D(_x,_y,_z)) );
            return (new Vector3D(_x, _y, _z));
        }

        // @param v the vector to be rotated
        // @returns the resulting rotated vector 
        public Vector3D Multiply (Vector3D v)
        {
            return (this.Rotate(v));
        }

        // Rotate a vector
        // @param v the vector to be rotated
        // @returns the resulting rotated vector
        public Vector3D Rotate(Vector3D v)
        {
            Vector3D _i = this.Imaginary();
            Vector3D _n = new Vector3D(Vector3D.VectorProduct(_i, v));
            Vector3D delta = new Vector3D(_r * _n - (Vector3D.VectorProduct(_n, _i)));
            return (new Vector3D(v + 2.0f * delta));
        }

        // Rotate a line segment.
        // param l the line segment to be rotated 
        // @returns the resulting rotated line segment
        public LineSeg3D Rotate(LineSeg3D l)
        {
            // Simply rotate both lines
            Vector3D i = this.Imaginary();
            Vector3D ns = new Vector3D(Vector3D.VectorProduct(i, l.StartPoint()));
            Vector3D ne = new Vector3D(Vector3D.VectorProduct(i, l.EndPoint()));
            Vector3D deltas = new Vector3D(_r * ns - (Vector3D)(Vector3D.VectorProduct(ns, i)));
            Vector3D deltae = new Vector3D(_r * ne - (Vector3D)(Vector3D.VectorProduct(ne, i)));
            LineSeg3D result = new LineSeg3D(l.StartPoint() + 2.0f * deltas, l.EndPoint()   + 2.0f * deltae);
            return (result);
        }

        // Rotate a line segment.
        // @param l the line segment to be rotated
        // @returns the resulting rotated line segment
        public LineSeg3D Multiply (LineSeg3D l)
        {
            return (this.Rotate(l));
        }

        // Combine two rotations.
        // @param q the inner rotation
        // @returns the resulting rotation (like this*q for matrices)
        public Quaternion Multiply (Quaternion q)
        {
            // real and img parts of args
            float r1 = this._r;                  
            float r2 = q._r;
            Vector3D i1 = this.Imaginary();
            Vector3D i2 = q.Imaginary();

            // Real is product of real, and dot-product of imag
            float real_v = (r1 * r2) - (i1 * i2);

            // Imag is cross product of imag + real*imag for each
            Point3D img = Point3D.VectorProduct(i1, i2);
            Point3D img2 = (img + (i2 * r1) + (i1 * r2));

            // Finally, build the result
            Quaternion prod = new Quaternion(img.GetX(), img.GetY(), img.GetZ(), real_v);
            return prod;
        }

        protected float _x,_y,_z,_r;



        public void Set(float x,float y,float z,float r)
        {
            _x = x;
            _y = y;
            _z = z;
            _r = r;

            // Make sure that this is a unit vector
            Normalise();
        }

        // Accessors
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

        public float GetR()
        {
            return(_r);
        }

        public void Get(ref float x, ref float y, ref float z, ref float r)
        {
            x = _x;
            y = _y;
            z = _z;
            r = _r;
        }

        // @returns a reference to the first imaginary part of the quaternion */
        public float X()
        {
            return _x;
        }

        // @returns a reference to the second imaginary part of the quaternion */
        public float Y()
        {
            return _y;
        }

        // @returns a reference to the third imaginary part of the quaternion 
        public float Z()
        {
            return _z;
        }

        // @returns a reference to the real part of the quaternion 
        public float R()
        {
            return _r;
        }

        // @returns the real part (cos(theta/2)) of the quaternion 
        public float Real()
        {
            return(_r);
        }

        public Quaternion Conjugate()
        {
            return new Quaternion(-_x, -_y, -_z, _r);
        }

        public Quaternion Inverse()
        {
            // Same as conjugate, unless not a unit vector, in which case
            // length is inverted
            float l = 1/(_x*_x + _y*_y + _z*_z + _r*_r);
            return( new Quaternion(-_x*l, -_y*l, -_z*l, _r*l) );
        }

        public void Normalise()
        {
            float n = 1/(float)Math.Sqrt(_x*_x + _y*_y + _z*_z + _r*_r);
            _x *= n;
            _y *= n;
            _z *= n;
            _r *= n;
        }



        // Euler conversions based on
        // http://www.mathworks.com/access/helpdesk/help/toolbox/aeroblks/euleranglestoquaternions.html
        // but note that Mathworks defines yaw=X and pitch=Y (we do the other way round), 
        // and that they do not mention that the multiplication:
        // (cx-isx)(cy-isy)(cz-isz) gives the conjugate of the desired quaternion!

        public void SetFromXYZEuler(float theta_x, float theta_y, float theta_z)
        {
            float cx = ((float)Math.Cos(0.5*theta_x));
            float cy = ((float)Math.Cos(0.5 * theta_y));
            float cz = ((float)Math.Cos(0.5 * theta_z));
            float sx = ((float)Math.Sin(0.5 * theta_x));
            float sy = ((float)Math.Sin(0.5 * theta_y));
            float sz = ((float)Math.Sin(0.5 * theta_z));

            this._r = cx*cy*cz + sx*sy*sz;
            this._x = sx*cy*cz - cx*sy*sz;
            this._y = cx*sy*cz + sx*cy*sz;
            this._z = cx*cy*sz - sx*sy*cz;
        }


        public void SetFromZXYEuler(float theta_z, float theta_x, float theta_y)
        {
            float cx = ((float)Math.Cos(0.5 * theta_x));
            float cy = ((float)Math.Cos(0.5 * theta_y));
            float cz = ((float)Math.Cos(0.5 * theta_z));
            float sx = ((float)Math.Sin(0.5 * theta_x));
            float sy = ((float)Math.Sin(0.5 * theta_y));
            float sz = ((float)Math.Sin(0.5 * theta_z));

            this._r =  cx*cy*cz + sx*sy*sz;
            this._x =  sx*cy*cz + cx*sy*sz;
            this._y =  cx*sy*cz - sx*cy*sz;
            this._z =  cx*cy*sz - sx*sy*cz;
        }

        public void SetFromYXZEuler(float theta_y, float theta_x, float theta_z)
        {
            float cx = ((float)Math.Cos(0.5 * theta_x));
            float cy = ((float)Math.Cos(0.5 * theta_y));
            float cz = ((float)Math.Cos(0.5 * theta_z));
            float sx = ((float)Math.Sin(0.5 * theta_x));
            float sy = ((float)Math.Sin(0.5 * theta_y));
            float sz = ((float)Math.Sin(0.5 * theta_z));

            this._r =  cx*cy*cz - sx*sy*sz;
            this._x =  sx*cy*cz - cx*sy*sz;
            this._y =  cx*sy*cz + sx*cy*sz;
            this._z =  cx*cy*sz + sx*sy*cz;
        }



        public void SetFromZYXEuler(float theta_z, float theta_y, float theta_x)
        {
            float cx = ((float)Math.Cos(0.5 * theta_x));
            float cy = ((float)Math.Cos(0.5 * theta_y));
            float cz = ((float)Math.Cos(0.5 * theta_z));
            float sx = ((float)Math.Sin(0.5 * theta_x));
            float sy = ((float)Math.Sin(0.5 * theta_y));
            float sz = ((float)Math.Sin(0.5 * theta_z));

            this._r = cx*cy*cz - sx*sy*sz;
            this._x = sx*cy*cz + cx*sy*sz;
            this._y = cx*sy*cz - sx*cy*sz;
            this._z = cx*cy*sz + sx*sy*cz;
        }
 
        public void SetFromZXZEuler(float theta_z1, float theta_x, float theta_z2)
        {
            float cz1 = ((float)Math.Cos(0.5 * theta_z1));
            float cx = ((float)Math.Cos(0.5 * theta_x));
            float cz2 = ((float)Math.Cos(0.5 * theta_z2));
            float sz1 = ((float)Math.Sin(0.5 * theta_z1));
            float sx = ((float)Math.Sin(0.5 * theta_x));
            float sz2 = ((float)Math.Sin(0.5 * theta_z2));

            this._r = cz2*cx*cz1 - sz2*cx*sz1;
            this._x = cz2*sx*cz1 + sz2*sx*sz1;
            this._y = sz2*sx*cz1 - cz2*sx*sz1;
            this._z = cz2*cx*sz1 + sz2*cx*cz1;
        }

        public void SetRXYZ(Vector vRXYZ) 
        {
            _r = vRXYZ[0];
            _x = vRXYZ[1];
            _y = vRXYZ[2];
            _z = vRXYZ[3];
        }

        public void GetXYZEuler(ref float theta_x, ref float theta_y, ref float theta_z)
        {
            theta_x = (float)Math.Atan2(2 * (_y * _z + _r * _x), _r * _r - _x * _x - _y * _y + _z * _z);

            theta_y = (float)Math.Asin(-2 * (_x * _z - _r * _y));

            theta_z = (float)Math.Atan2(2 * (_x * _y + _r * _z), _r * _r + _x * _x - _y * _y - _z * _z);
        }

        // Based on
        // http://www.mathworks.com/access/helpdesk/help/toolbox/aeroblks/quaternionstoeulerangles.html
        public void GetZXYEuler(ref float theta_z, ref float theta_x, ref float theta_y)
        {
            theta_x = (float)Math.Asin(-2 * (_y * _z - _r * _x));

            theta_y = (float)Math.Atan2(2 * (_x * _z + _r * _y), _r * _r - _x * _x - _y * _y + _z * _z);

            theta_z = (float)Math.Atan2(2 * (_x * _y + _r * _z), _r * _r - _x * _x + _y * _y - _z * _z);
        }

        public void GetYXZEuler(ref float theta_y, ref float theta_x, ref float theta_z)
        {
            theta_x = (float)Math.Asin(2 * (_y * _z + _r * _x));

            theta_y = (float)Math.Atan2(-2 * (_x * _z - _r * _y), _r * _r - _x * _x - _y * _y + _z * _z);

            theta_z = (float)Math.Atan2(-2 * (_x * _y - _r * _z), _r * _r - _x * _x + _y * _y - _z * _z);
        }

        public void GetZYXEuler(ref float theta_z, ref float theta_y, ref float theta_x)
        {
            theta_x = (float)Math.Atan2(-2 * (_y * _z - _r * _x), _r * _r - _x * _x - _y * _y + _z * _z);

            theta_y = (float)Math.Asin(2 * (_x * _z + _r * _y));

            theta_z = (float)Math.Atan2(-2 * (_x * _y - _r * _z), _r * _r + _x * _x - _y * _y - _z * _z);		   
        }

        public void GetZXZEuler(ref float theta_z1, ref float theta_x, ref float theta_z2)
        {
            theta_z2 = (float)Math.Atan2(_x * _z + _r * _y, _r * _x - _y * _z);
            theta_x = (float)Math.Acos(_r * _r - _x * _x - _y * _y + _z * _z);
            theta_z1 = (float)Math.Atan2(_x * _z - _r * _y, _y * _z + _r * _x);
        }

        public Vector GetRXYZ()
        {
            Vector vRXYZ = new Vector(4);
            vRXYZ[0] = _r;
            vRXYZ[1] = _x;
            vRXYZ[2] = _y;
            vRXYZ[3] = _z;
  
            return vRXYZ;
        }

        public bool EqualTo(Quaternion q)
        {
            return ((q._x == _x) && (q._y == _y) && (q._z == _z) && (q._r == _r));
        }

        public bool NotEqualTo (Quaternion q)
        {
            return ((q._x != _x) || (q._y != _y) || (q._z != _z) || (q._r != _r));
        }
    }

}
