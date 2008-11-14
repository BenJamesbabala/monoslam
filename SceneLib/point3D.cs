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
    public class Point3D
    {
        public Point3D (float x_coord, float y_coord, float z_coord) 
        {
            _x = x_coord;
            _y = y_coord;
            _z = z_coord;
        }

        public Point3D(Vector v)
        {
            _x = v[0];
            _y = v[1];
            _z = v[2];
        }

        public Point3D(Point3D p)
        {
            this.Equals(p);
        }

        public Point3D Equals (Point3D p) 
        {
            if (this != p) 
            {
                _x = p._x;
                _y = p._y;
                _z = p._z;
            } 
            else 
            {
                //cout << "Error: copying point into itself!" << endl;
            }
            return (this);
        }


        public bool EqualTo (Point3D p) 
        {
            return ((_x == p._x) && (_y == p._y) && (_z == p._z));
        }


        public bool NotEqualTo (Point3D p) 
        {
            return ((_x != p._x) || (_y != p._y) || (_z != p._z));
        }

        public static Point3D operator + (Point3D p1, Point3D p2) 
        {
            Point3D p3 =new Point3D(0,0,0);
            p3._x = p1._x + p2._x;
            p3._y = p1._y + p2._y;
            p3._z = p1._z + p2._z;
            return p3;
        }


        public static Point3D operator - (Point3D p)
        {
            Point3D p2 = new Point3D(-p._x,-p._y,-p._z);
            return(p2);
        }

        public static Point3D operator - (Point3D p1, Point3D p2) 
        {
            Point3D p3 = new Point3D(0,0,0);
            p3._x = p1._x - p2._x;
            p3._y = p1._y - p2._y;
            p3._z = p1._z - p2._z;
            return p3;
        }


        public static float operator * (Point3D p1, Point3D p2) 
        {
            return (p1._x*p2._x + p1._y*p2._y + p1._z*p2._z);
        }


        public static Point3D operator * (Point3D p1, float a) 
        {
            Point3D p3 = new Point3D(0,0,0);
            p3._x = a*p1._x;
            p3._y = a*p1._y;
            p3._z = a*p1._z;
            return p3;
        }


        public static Point3D operator * (float a, Point3D p1) 
        {
            Point3D p3 = new Point3D(0,0,0);
            p3._x = a*p1._x;
            p3._y = a*p1._y;
            p3._z = a*p1._z;
            return p3;
        }

        public static Point3D operator * (MatrixFixed M, Point3D p1) 
        {
            Point3D p3 = new Point3D(0,0,0);
            p3._x = M[0,0]*p1._x + M[0,1]*p1._y + M[0,2]*p1._z;
            p3._y = M[1,0]*p1._x + M[1,1]*p1._y + M[1,2]*p1._z;
            p3._z = M[2,0]*p1._x + M[2,1]*p1._y + M[2,2]*p1._z;
            return p3;
        }


        public static Point3D operator / (Point3D p1, float a) 
        {
            Point3D p3 = new Point3D(0,0,0);
            p3._x = p1._x / a;
            p3._y = p1._y / a;
            p3._z = p1._z / a;
            return p3;
        }


        public void Get(ref float x_coord, ref float y_coord, ref float z_coord)
        {
            z_coord = _z;
            y_coord = _y;
            x_coord = _x;
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

        public Vector GetVNL3()
        {
            Vector vnl3 = new Vector(3);
            vnl3[0] = _x;
            vnl3[1] = _y;
            vnl3[2] = _z;

            return vnl3;
        }


        public void Set(float x_coord, float y_coord, float z_coord) 
        {
            _x = x_coord;
            _y = y_coord;
            _z = z_coord;
        }

        public void SetX(float x_coord) 
        {
            _x = x_coord;
        }

        public void SetY(float y_coord) 
        {
            _y = y_coord;
        }

        public void SetZ(float z_coord) 
        {
            _z = z_coord;
        }

        public void SetVNL3(Vector vnl3)
        {
            _x = vnl3[0];
            _y = vnl3[1];
            _z = vnl3[2];
        }

        public float Norm()
        {
            return (float)Math.Sqrt(_x * _x + _y * _y + _z * _z);
        }

        public float NormSquared()
        {
            return (_x*_x + _y*_y + _z*_z);
        }

        public void Normalise()
        {
            float n = 1/Norm();
            _x *= n;
            _y *= n;
            _z *= n;
        }

	    public static Point3D VectorProduct(Point3D p1, Point3D p2)
	    {
	        return(new Point3D(p1._y*p2._z - p1._z*p2._y,
				 p1._z*p2._x - p1._x*p2._z,
				 p1._x*p2._y - p1._y*p2._x));
	    }

	    public float ScalarProduct(Point3D p1, Point3D p2)
	    {
	        return(p1._x * p2._x
		           + p1._y * p2._y
		           + p1._z * p2._z);
	    }

	    public float ScalarTripleProduct(Point3D p1, 
					  Point3D p2, 
					  Point3D p3)
	    {
	        float result=0;
	        result += p1._x * (p2._y*p3._z - p3._y*p2._z);
	        result -= p2._x * (p1._y*p3._z - p3._y*p1._z);
	        result += p3._x * (p1._y*p2._z - p2._y*p1._z);
	        return(result);
	    }

                       
        public float GetDistance(Point3D testpoint) 
        {
            return (float)Math.Sqrt((testpoint._x - _x) * (testpoint._x - _x) + 
		               (testpoint._y - _y)*(testpoint._y - _y) + 
		               (testpoint._z - _z)*(testpoint._z - _z));
        }		  

        // Data
        public float _x;
        public float _y;
        public float _z;
    }


    public class Vector3D : Point3D
    {
        public Vector3D (float x_coord, float y_coord, float z_coord) : base(x_coord,y_coord,z_coord)
        {
            // move along, nothing to see...
        }

        public Vector3D(Vector v) : base(v)
        {
            // move along, nothing to see...
        }

        public Vector3D(Point3D p) : base(p)
        {
            // move along, nothing to see...
        }
    }
}
