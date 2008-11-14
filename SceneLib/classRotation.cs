/*Title:      mjbWorld
Copyright (c) 1998-2003 Martin John Baker

This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   as published by the Free Software Foundation; either version 2
   of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

For information about the GNU General Public License see http://www.gnu.org/

To discuss this program http://sourceforge.net/forum/forum.php?forum_id=122133
   also see website http://www.euclideanspace.com/
   */

namespace mjbModel
{
    using System;
    using System.Collections;
    using System.Text;

    /// <summary>
    /// This class can represent a 3D vector. For instance a point in 3D space,    or
    /// a relative position or movement.
    /// 
    /// The class has methods to add, subtact, (cross and dot) multiply with other
    /// vectors. also many other methods, including the ability to load and save
    /// to from VRML and x3d 
    /// </summary>
    class vector3D
    {
        /// <summary>
        /// VRML only supports float but allow override if higher resolution required
        /// </summary>
        public static bool saveAsDouble = false;
        /// <summary>
        /// x coordinate
        /// </summary>
        public double x;
        /// <summary>
        /// y coordinate
        /// </summary>
        public double y;
        /// <summary>
        /// z coordinate
        /// </summary>
        public double z;

        /// <summary>
        /// a constructor to set initial values of x,y and z coodinates
        /// </summary>
        /// <param name="x1">value of x coordinate</param>
        /// <param name="y1">value of y coordinate</param>
        /// <param name="z1">value of z coordinate</param>
        public vector3D(double x1, double y1, double z1)
        {
            x = x1;
            y = y1;
            z = z1;
        }

        /// <summary>
        /// copy constructor 
        /// </summary>
        /// <param name="in1">set values to save value in1</param>
        public vector3D(vector3D in1)
        {
            x = (in1 != null) ? in1.x : 0;
            y = (in1 != null) ? in1.y : 0;
            z = (in1 != null) ? in1.z : 1;
        }

        /// <summary>
        /// construct a vector with initial value zero.
        /// </summary>
        public vector3D()
        {
        }

        /// <summary>
        /// static method to return sum of two vectors
        /// </summary>
        /// <param name="a">first vector to be added</param>
        /// <param name="b">second vector to be added</param>
        /// <returns>the sum</returns>
        public static vector3D add(vector3D a, vector3D b)
        {
            return new vector3D(a.x + b.x, a.y + b.y, a.z + b.z);
        }

        /// <summary>
        /// static method to return difference of two vectors
        /// </summary>
        /// <param name="a">first vector</param>
        /// <param name="b">subract this vector</param>
        /// <returns>result</returns>
        public static vector3D sub(vector3D a, vector3D b)
        {
            return new vector3D(a.x - b.x, a.y - b.y, a.z - b.z);
        }


        /// <summary>
        /// set the value of this instance to the value of other
        /// this can be used to reuse an instance without the overheads of garbidge    collection
        /// </summary>
        /// <param name="other">instace we want to use value of, if    null then set to zero</param>
        public void copy(vector3D other)
        {
            if (other == null)
            {
                x = y = z = 0;
                return;
            }
            x = other.x;
            y = other.y;
            z = other.z;
        }

        /// <summary>
        /// subtract other vector from this
        /// for theory see:
        /// http://www.euclideanspace.com/maths/algebra/vectors/index.htm
        /// </summary>
        /// <param name="other">vector to be subtracted from this</param>
        public void sub(vector3D other)
        {
            if (other == null) return;
            x -= other.x;
            y -= other.y;
            z -= other.z;
        }

        /// <summary>
        /// add other vector to this
        /// for theory see:
        /// http://www.euclideanspace.com/maths/algebra/vectors/index.htm
        /// </summary>
        /// <param name="other">vector to be added to this</param>
        public void add(vector3D other)
        {
            if (other == null) return;
            x += other.x;
            y += other.y;
            z += other.z;
        }

        /// <summary>
        /// convert this vector to unit length
        /// for theory see:
        /// http://www.euclideanspace.com/maths/algebra/vectors/normals/index.htm
        /// </summary>
        public void normalize()
        {
            double t = Math.Sqrt(x * x + y * y + z * z);
            x /= t;
            y /= t;
            z /= t;
        }

        /// <summary>
        /// scale this vector equally in all directions
        /// </summary>
        /// <param name="sc">scale factor</param>
        public void scale(double sc)
        {
            x *= sc;
            y *= sc;
            z *= sc;
        }

        /// <summary>
        /// scale this vector posibly different in x,y and z directions
        /// </summary>
        /// <param name="other">scale value</param>
        public void scale(vector3D other)
        {
            x *= other.x;
            y *= other.y;
            z *= other.z;
        }

        /// <summary>
        /// scale this vector by inverse of other
        /// </summary>
        /// <param name="other">scale value</param>
        public void scaleInv(vector3D other)
        {
            x /= other.x;
            y /= other.y;
            z /= other.z;
        }

        /// <summary>
        /// return square of distance from end of this vector to end of other
        /// </summary>
        /// <param name="other">calcules distance from this vector</param>
        /// <returns>square of distance from end of this vector to end of other</returns>
        public double distanceSquared(vector3D other)
        {
            double x1 = other.x - x;
            double y1 = other.y - y;
            double z1 = other.z - z;
            return x1 * x1 + y1 * y1 + z1 * z1;
        }

        /// <summary>
        /// cross product
        /// for theory see:
        /// http://www.euclideanspace.com/maths/algebra/vectors/index.htm
        /// </summary>
        /// <param name="other">other vector to take cross product with</param>
        public void cross(vector3D other)
        {
            double xh = y * other.z - other.y * z;
            double yh = z * other.x - other.z * x;
            double zh = x * other.y - other.x * y;
            x = xh;
            y = yh;
            z = zh;
        }

        /// <summary>
        /// dot product
        /// for theory see:
        /// http://www.euclideanspace.com/maths/algebra/vectors/index.htm
        /// </summary>
        /// <param name="other">vector to take dot product with</param>
        /// <returns>dot product</returns>
        public double dot(vector3D other)
        {
            return (x * other.x) + (y * other.y) + (z * other.z);
        }

        /// <summary>
        /// set the x value only without althering the other dimentions
        /// </summary>
        /// <param name="v">new value for x</param>
        public void setX(double v)
        {
            x = v;
        }

        /// <summary>
        /// set the y value only without althering the other dimentions
        /// </summary>
        /// <param name="v">new value for y</param>
        public void setY(double v)
        {
            y = v;
        }

        /// <summary>
        /// set the z value only without althering the other dimentions
        /// </summary>
        /// <param name="v">new value for z</param>
        public void setZ(double v)
        {
            z = v;
        }

        /// <summary>
        /// gets the value in the x dimension
        /// </summary>
        /// <returns>the value in the x dimension</returns>
        public double getx()
        {
            return x;
        }

        /// <summary>
        /// gets the value in the y dimension
        /// </summary>
        /// <returns>the value in the y dimension</returns>
        public double gety()
        {
            return y;
        }

        /// <summary>
        /// gets the value in the z dimension
        /// </summary>
        /// <returns>the value in the z dimension</returns>
        public double getz()
        {
            return z;
        }

        /// <summary>
        /// inverts the direction of the vector
        /// </summary>
        public void minus()
        {
            x = -x; y = -y; z = -z;
        }

        /// <summary>
        /// return a vector pointing on the opposite direction to this without affecting    the
        /// value of this instance
        /// </summary>
        /// <returns></returns>
        public vector3D getMinus()
        {
            return new vector3D(-x, -y, -z);
        }


        /// <summary>
        /// returns true if any dimension of other vector is greater than the same dimension
        /// of this
        /// </summary>
        /// <param name="other">vector to compare with this</param>
        /// <returns>true if greater</returns>
        public bool greaterThan(vector3D other)
        {
            if (other.x > x) return true;
            if (other.y > y) return true;
            if (other.z > z) return true;
            return false;
        }

        /// <summary>
        /// returns true if any dimension of other vector is less than the same dimension
        /// of this
        /// </summary>
        /// <param name="other">vector to compare with this</param>
        /// <returns>true if less</returns>
        public bool lessThan(vector3D other)
        {
            if (other.x < x) return true;
            if (other.y < y) return true;
            if (other.z < z) return true;
            return false;
        }

        /// <summary>
        /// returns true if this vector has an equal value to other vector
        /// </summary>
        /// <param name="v2"></param>
        /// <returns></returns>
        public bool Equals(vector3D other)
        {
            if (other == null) return false;
            if (x != other.x) return false;
            if (y != other.y) return false;
            if (z != other.z) return false;
            return true;
        }

        /// <summary>
        /// calcultes new translation when combining translations
        /// 
        /// Rt = Ra Rb
        /// Ct = Cb
        /// Tt = Ra (Cb + Tb - Ca) + Ca + Ta - Cb
        /// 
        /// for theory:
        /// http://www.euclideanspace.com/maths/geometry/rotations/rotationAndTranslation/nonMatrix/index.htm
        /// </summary>
        /// <param name="ta">Ta = translation of transform a in absolute    coordinates</param>
        /// <param name="ra">Ra = rotation function of transform a in    absolute coordinates</param>
        /// <param name="ca">Ca = centre of rotation of transform a    in absolute coordinates</param>
        /// <param name="tb">Tb = translation of transform b in coordinates    of transform a</param>
        /// <param name="cb">Cb = centre of rotation of transform b    in coordinates of transform a</param>
        /// <returns>Tt total offset</returns>
        public vector3D rotationOffset(vector3D ta, classRotation ra, vector3D ca, vector3D tb, vector3D cb)
        {
            vector3D result = new vector3D(cb);
            result.add(tb);
            result.sub(ca);
            if (ra != null) ra.transform(result);
            result.add(ca);
            result.add(ta);
            result.sub(cb);
            return result;
        }

        /// <summary>
        /// use to combine bounds
        /// if i=0 take minimum otherwise maximum
        /// </summary>
        /// <param name="other">vector to combine with</param>
        /// <param name="i">if i=0 take minimum otherwise maximum</param>
        public void bounds(vector3D other, int i)
        {
            if (i == 0)
            { // take minimum
                if (other.x < x) x = other.x;
                if (other.y < y) y = other.y;
                if (other.z < z) z = other.z;
            }
            else
            { // take maximum
                if (other.x > x) x = other.x;
                if (other.y > y) y = other.y;
                if (other.z > z) z = other.z;
            }
        }

    } // class

    /// <summary>
    /// a class to represent a rotation, internally the class may code the rotation    as an
    /// axis angle:
    /// http://www.euclideanspace.com/maths/geometry/rotations/axisAngle/index.htm
    /// or a quaternion:
    /// http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
    /// or as euler angles
    /// http://www.euclideanspace.com/maths/geometry/rotations/euler/index.htm
    /// </summary>
    class classRotation
    {

        /// <summary>
        /// defines the resolution at which the rotation will be saved to file
        /// </summary>
        public static bool saveAsDouble = false;

        /// <summary>
        /// x element of axis angle or quaternion
        /// </summary>
        public double x;

        /// <summary>
        /// y element of axis angle or quaternion
        /// </summary>
        public double y;

        /// <summary>
        /// z element of axis angle or quaternion
        /// </summary>
        public double z;

        /// <summary>
        /// angle element of axis angle or w element of quaternion
        /// </summary>
        public double angle;

        /// <summary>
        /// VRML always uses axis-angle to represent rotations
        /// but quaternions are more efficient for some applications
        /// </summary>
        public int coding = (int)cde.CODING_AXISANGLE;
        /// <summary>
        /// possible values for coding variable
        /// </summary>
        public enum cde
        {
            CODING_AXISANGLE,
            CODING_QUATERNION,
            CODING_EULER,
            CODING_AXISANGLE_SAVEASQUAT,
            CODING_QUATERNION_SAVEASQUAT,
            CODING_EULER_SAVEASQUAT
        };

        /// <summary>
        /// constructor which allows initial value to be suplied as axis angle
        /// </summary>
        /// <param name="x1">x dimention of normalised axis</param>
        /// <param name="y1">y dimention of normalised axis</param>
        /// <param name="z1">z dimention of normalised axis</param>
        /// <param name="a1">angle</param>
        public classRotation(double x1, double y1, double z1, double a1)
        {
            x = x1;
            y = y1;
            z = z1;
            angle = a1;
        }

        /// <summary>
        /// constructor which allows initial value to be suplied as axis angle,quaternion
        /// or axis angle as defined by c1 whoes possible values are given by enum cde
        /// </summary>
        /// <param name="x1">if quaternion or axis angle holds x dimention    of normalised axis</param>
        /// <param name="y1">if quaternion or axis angle holds y dimention    of normalised axis</param>
        /// <param name="z1">if quaternion or axis angle holds z dimention    of normalised axis</param>
        /// <param name="a1">if quaternion holds w, if axis angle holds    angle</param>
        /// <param name="c1">possible values are given by enum cde</param>
        public classRotation(double x1, double y1, double z1, double a1, int c1)
        {
            x = x1;
            y = y1;
            z = z1;
            angle = a1;
            coding = c1;
        }

        /// <summary>
        /// constructor to create classRotation from euler angles.
        /// </summary>
        /// <param name="heading">rotation about z axis</param>
        /// <param name="attitude">rotation about y axis</param>
        /// <param name="bank">rotation about x axis</param>
        public classRotation(double heading, double attitude, double bank)
        {
            double c1 = Math.Cos(heading / 2);
            double s1 = Math.Sin(heading / 2);
            double c2 = Math.Cos(attitude / 2);
            double s2 = Math.Sin(attitude / 2);
            double c3 = Math.Cos(bank / 2);
            double s3 = Math.Sin(bank / 2);
            double c1c2 = c1 * c2;
            double s1s2 = s1 * s2;
            angle = c1c2 * c3 + s1s2 * s3;
            x = c1c2 * s3 - s1s2 * c3;
            y = c1 * s2 * c3 + s1 * c2 * s3;
            z = s1 * c2 * c3 - c1 * s2 * s3;
            coding = (int)cde.CODING_QUATERNION;
            saveAsDouble = false;
        }

        /// <summary>
        /// copy constructor
        /// </summary>
        /// <param name="in1"></param>
        public classRotation(classRotation in1)
        {
            x = (in1 != null) ? in1.x : 0;
            y = (in1 != null) ? in1.y : 0;
            z = (in1 != null) ? in1.z : 1;
            angle = (in1 != null) ? in1.angle : 0;
            coding = (in1 != null) ? in1.coding : (int)cde.CODING_AXISANGLE;
        }

        /// <summary>
        /// constructor
        /// </summary>
        public classRotation()
        {
        }

        /// <summary>
        /// calculates the effect of this rotation on a point
        /// the new point is given by=q * P1 * q'
        /// this version does not alter P1 but returns the result.
        /// 
        /// for theory see:
        /// http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
        /// </summary>
        /// <param name="point">point to be transformed</param>
        /// <returns>translated point</returns>
        public vector3D getTransform(vector3D p1)
        {
            double wh = angle;
            double xh = x;
            double yh = y;
            double zh = z;
            if (coding == (int)cde.CODING_AXISANGLE)
            {
                double s = Math.Sin(angle / 2);
                xh = x * s;
                yh = y * s;
                zh = z * s;
                wh = Math.Cos(angle / 2);
            }
            vector3D p2 = new vector3D();
            p2.x = wh * wh * p1.x + 2 * yh * wh * p1.z - 2 * zh * wh * p1.y + xh * xh * p1.x + 2 * yh * xh * p1.y + 2 * zh * xh * p1.z - zh * zh * p1.x - yh * yh * p1.x;
            p2.y = 2 * xh * yh * p1.x + yh * yh * p1.y + 2 * zh * yh * p1.z + 2 * wh * zh * p1.x - zh * zh * p1.y + wh * wh * p1.y - 2 * xh * wh * p1.z - xh * xh * p1.y;
            p2.z = 2 * xh * zh * p1.x + 2 * yh * zh * p1.y + zh * zh * p1.z - 2 * wh * yh * p1.x - yh * yh * p1.z + 2 * wh * xh * p1.y - xh * xh * p1.z + wh * wh * p1.z;
            return p2;
        }

        /// <summary>
        /// calculates the effect of this rotation on a point
        /// the new point is given by=q * P1 * q'
        /// this version returns the result in p1
        /// 
        /// for theory see:
        /// http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
        /// </summary>
        /// <param name="point">point to be transformed</param>
        public void transform(vector3D p1)
        {
            double wh = angle;
            double xh = x;
            double yh = y;
            double zh = z;
            if (coding == (int)cde.CODING_AXISANGLE)
            {
                double s = Math.Sin(angle / 2);
                xh = x * s;
                yh = y * s;
                zh = z * s;
                wh = Math.Cos(angle / 2);
            }
            double resultx = wh * wh * p1.x + 2 * yh * wh * p1.z - 2 * zh * wh * p1.y + xh * xh * p1.x + 2 * yh * xh * p1.y + 2 * zh * xh * p1.z - zh * zh * p1.x - yh * yh * p1.x;
            double resulty = 2 * xh * yh * p1.x + yh * yh * p1.y + 2 * zh * yh * p1.z + 2 * wh * zh * p1.x - zh * zh * p1.y + wh * wh * p1.y - 2 * xh * wh * p1.z - xh * xh * p1.y;
            double resultz = 2 * xh * zh * p1.x + 2 * yh * zh * p1.y + zh * zh * p1.z - 2 * wh * yh * p1.x - yh * yh * p1.z + 2 * wh * xh * p1.y - xh * xh * p1.z + wh * wh * p1.z;
            p1.x = resultx;
            p1.y = resultx;
            p1.z = resultx;
        }


        /// <summary>
        /// invert direction of rotation
        /// </summary>
        public void minus()
        {
            if (coding == (int)cde.CODING_AXISANGLE)
            {
                angle = -angle;
                return;
            }
            x = -x;
            y = -y;
            z = -z;
        }

        /// <summary>
        /// get a clone of the rotation
        /// </summary>
        /// <returns></returns>
        public classRotation getMinus()
        {
            if (coding == (int)cde.CODING_AXISANGLE) return new classRotation(x, y, z, -angle, coding);
            return new classRotation(-x, -y, -z, angle, coding);
        }

        /// <summary>
        /// set the axis of rotation
        /// </summary>
        /// <param name="tx"></param>
        /// <param name="ty"></param>
        /// <param name="tz"></param>
        public void set(double tx, double ty, double tz)
        {
            angle = Math.Sqrt(tx * tx + ty * ty + tz * tz);
            if (angle == 0) { x = 1; y = z = 0; return; }
            x = tx / angle;
            y = ty / angle;
            z = tz / angle;
        }

        /// <summary>
        /// set the values of this rotation
        /// </summary>
        /// <param name="tx"></param>
        /// <param name="ty"></param>
        /// <param name="tz"></param>
        /// <param name="tangle"></param>
        public void set(double tx, double ty, double tz, double tangle)
        {
            x = tx;
            y = ty;
            z = tz;
            angle = tangle;
        }

        /// <summary>
        /// returns axis in x dimention
        /// </summary>
        /// <returns>axis in x dimention</returns>
        public double getTx()
        {
            return x * angle;
        }

        /// <summary>
        /// returns axis in y dimention
        /// </summary>
        /// <returns>returns axis in y dimention</returns>
        public double getTy()
        {
            return y * angle;
        }

        /// <summary>
        /// returns axis in z dimention
        /// </summary>
        /// <returns>returns axis in z dimention</returns>
        public double getTz()
        {
            return z * angle;
        }

        /// <summary>
        /// calculate total rotation by taking current rotation and then
        /// apply rotation r
        /// 
        /// if both angles are quaternions then this is a multiplication
        /// </summary>
        /// <param name="r"></param>
        public void combine(classRotation r)
        {
            toQuaternion();
            if (r == null) return;
            double qax = x;
            double qay = y;
            double qaz = z;
            double qaw = angle;
            double qbx;
            double qby;
            double qbz;
            double qbw;

            if (r.coding == (int)cde.CODING_QUATERNION)
            {
                qbx = r.x;
                qby = r.y;
                qbz = r.z;
                qbw = r.angle;
            }
            else
            {
                double s = Math.Sin(r.angle / 2);
                qbx = r.x * s;
                qby = r.y * s;
                qbz = r.z * s;
                qbw = Math.Cos(r.angle / 2);
            }
            // now multiply the quaternions
            angle = qaw * qbw - qax * qbx - qay * qby - qaz * qbz;
            x = qax * qbw + qaw * qbx + qay * qbz - qaz * qby;
            y = qaw * qby - qax * qbz + qay * qbw + qaz * qbx;
            z = qaw * qbz + qax * qby - qay * qbx + qaz * qbw;
            coding = (int)cde.CODING_QUATERNION;
        }

        /// <summary>
        /// combine a rotation expressed as euler angle with current rotation.
        /// first convert both values to quaternoins then combine and convert back to    
        /// axis angle. Theory about these conversions shown here:
        /// http://www.euclideanspace.com/maths/geometry/rotations/conversions/index.htm
        /// </summary>
        /// <param name="heading">angle about x axis</param>
        /// <param name="attitude">angle about y axis</param>
        /// <param name="bank">angle about z axis</param>
        public void combine(double heading, double attitude, double bank)
        {
            // first calculate quaternion qb from heading, attitude and bank
            double c1 = Math.Cos(heading / 2);
            double s1 = Math.Sin(heading / 2);
            double c2 = Math.Cos(attitude / 2);
            double s2 = Math.Sin(attitude / 2);
            double c3 = Math.Cos(bank / 2);
            double s3 = Math.Sin(bank / 2);
            double c1c2 = c1 * c2;
            double s1s2 = s1 * s2;
            double qbw = c1c2 * c3 + s1s2 * s3;
            double qbx = c1c2 * s3 - s1s2 * c3;
            double qby = c1 * s2 * c3 + s1 * c2 * s3;
            double qbz = s1 * c2 * c3 - c1 * s2 * s3;
            // then convert axis-angle to quaternion if required
            toQuaternion();
            double qax = x;
            double qay = y;
            double qaz = z;
            double qaw = angle;
            // now multiply the quaternions
            angle = qaw * qbw - qax * qbx - qay * qby - qaz * qbz;
            x = qax * qbw + qaw * qbx + qay * qbz - qaz * qby;
            y = qaw * qby - qax * qbz + qay * qbw + qaz * qbx;
            z = qaw * qbz + qax * qby - qay * qbx + qaz * qbw;
            coding = (int)cde.CODING_QUATERNION;
            //Console.WriteLine("classRotation.add(h={0} a={1} b={2} angle={3} x={4} y={5}    z={6}",heading,attitude,bank,angle,x,y,z);
        }

        /// <summary>
        /// if this rotation is not already coded as axis angle then convert it to axis    angle
        /// </summary>
        public void toAxisAngle()
        {
            if (coding == (int)cde.CODING_AXISANGLE) return;
            double s = Math.Sqrt(1 - angle * angle);
            if (Math.Abs(s) < 0.001) s = 1;
            angle = 2 * Math.Acos(angle);
            x = x / s;
            y = y / s;
            z = z / s;
        }

        /// <summary>
        /// if this rotation is not already coded as quaternion then convert it to quaternion
        /// </summary>
        public void toQuaternion()
        {
            if (coding == (int)cde.CODING_QUATERNION) return;
            double s = Math.Sin(angle / 2);
            x = x * s;
            y = y * s;
            z = z * s;
            angle = Math.Cos(angle / 2);
        }


    }
} 