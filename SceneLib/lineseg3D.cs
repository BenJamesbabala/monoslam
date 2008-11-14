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
    public class LineSeg3D
    {
        protected Point3D _sp, _ep;


        public float GetLength()
        {
            return (float)Math.Sqrt(GetLength2());
        }

        public float GetLength2()
        {
            return ((_ep._x - _sp._x) * (_ep._x - _sp._x)
                    + (_ep._y - _sp._y) * (_ep._y - _sp._y)
                    + (_ep._z - _sp._z) * (_ep._z - _sp._z));
        }

        public void GetEndPoints(ref float x0, ref float y0, ref float z0,
                                 ref float x1, ref float y1, ref float z1)
        {
            _sp.Get(ref x0, ref y0, ref z0);
            _ep.Get(ref x1, ref y1, ref z1);
        }

        public void GetEndPoints(Point3D p0, Point3D p1)
        {
            p0 = _sp;
            p1 = _ep;
        }

        public Point3D GetStartPoint()
        {
            return _sp;
        }

        public Point3D GetEndPoint()
        {
            return _ep;
        }

        public Point3D StartPoint()
        {
            return _sp;
        }

        public Point3D EndPoint()
        {
            return _ep;
        }

        public void SetEndPoints(float x0, float y0, float z0,
                                 float x1, float y1, float z1)
        {
            _sp.Set(x0, y0, z0);
            _ep.Set(x1, y1, z1);
        }

        public void SetEndPoints(Point3D p0, Point3D p1)
        {
            _sp = p0;
            _ep = p1;
        }

        public void SetStartPoint(Point3D p)
        {
            _sp = p;
        }

        public void SetEndPoint(Point3D p)
        {
            _ep = p;
        }

        public static LineSeg3D operator + (LineSeg3D line, Point3D p)
        {
            line._sp += p;
            line._ep += p;
            return line;
        }

        public static LineSeg3D operator - (LineSeg3D line, Point3D p)
        {
            line._sp -= p;
            line._ep -= p;
            return line;
        }

        public LineSeg3D Add (LineSeg3D line, Point3D point)
        {
            LineSeg3D l = new LineSeg3D();
            l.Equals(line);
            l += point;
            return l;
        }

        public LineSeg3D Subtract (LineSeg3D line, Point3D point)
        {
            LineSeg3D l = new LineSeg3D();
            l.Equals(line);
            l -= point;
            return l;
        }



        public LineSeg3D()
        {
            // Nothing much to create...
            _sp = new Point3D(0,0,0);
            _ep = new Point3D(0,0,0);
        }


        public LineSeg3D(float x0, float y0, float z0, 
		                 float x1, float y1, float z1)
        {
            // Nothing much to create...
            _sp = new Point3D(x0,y0,z0);
            _ep = new Point3D(x1,y1,z1);
        }

        public LineSeg3D(Point3D p0, Point3D p1)
        {
            // Nothing much to create...
            _sp = p0;
            _ep = p1;
        }


        public LineSeg3D Equals (LineSeg3D ls) 
        {
            if (this != ls) 
            {
                _sp = ls._sp;
                _ep = ls._ep;
            } 
            else 
            {
                Debug.WriteLine("Error: copying LineSeg3D into itself!");
            }
            return this;
        }


        public bool EqualTo (LineSeg3D ls) 
        {
            return ((_sp==ls._sp) && (_ep==ls._ep));
        }


        public float Multiply(LineSeg3D ls1, LineSeg3D ls2) 
        {
            Vector3D d1 = new Vector3D(ls1._ep - ls1._sp);
            Vector3D d2 = new Vector3D(ls2._ep - ls2._sp);

            return (d1.GetX() * d2.GetX() + d1.GetY() * d2.GetY() + d1.GetZ() * d2.GetZ());
        }


        public float Norm2(LineSeg3D ls)
        {
            return ((ls._ep._x-ls._sp._x)*(ls._ep._x-ls._sp._x) 
	                + (ls._ep._y-ls._sp._y)*(ls._ep._y-ls._sp._y) 
	                + (ls._ep._z-ls._sp._z)*(ls._ep._z-ls._sp._z));
        }


        public float Norm(LineSeg3D ls)
        {
            return (float)Math.Sqrt((ls._ep._x - ls._sp._x) * (ls._ep._x - ls._sp._x) 
	                         + (ls._ep._y-ls._sp._y)*(ls._ep._y-ls._sp._y) 
	                         + (ls._ep._z-ls._sp._z)*(ls._ep._z-ls._sp._z));
        }

    }

}
