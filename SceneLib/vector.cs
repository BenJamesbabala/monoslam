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
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Text;

namespace SceneLibrary
{
    
    public class Vector
    {
        protected float[] data;

        public Vector(float x, float y, float z)
        {
            data = new float[3];
            data[0] = x;
            data[1] = y;
            data[2] = z;
        }

        public Vector(int length)
        {
            data = new float[length];
            //Fill(0);
        }

        public Vector(Vector v)
        {
            data = new float[v.data.Length];
            for (int i = 0; i < data.Length; i++) data[i] = v[i];
        }

        public Vector(uint length)
        {
            data = new float[(int)length];
            //Fill(0);
        }

        public float[] Datablock()
        {
            return (data);
        }

        /// <summary>
        /// Read vector as a string, typically from a text file
        /// </summary>
        /// <param name="s"></param>
        /// <returns></returns>
        public bool ReadASCII(String s)
        {
            if (size() > -1)
            {
                String[] values = s.Split(',');
                if (values.Length > 0)
                {
                    for (int i = 0; i < size(); ++i)
                        this[i] = Convert.ToSingle(values[i]);
                }
                return true;
            }
            else
                return false;
        }

        /// <summary>
        /// returns a string representing the vector, which may be written to a text file
        /// </summary>
        /// <returns></returns>
        public String WriteASCII()
        {
            String ASCII_vector = "";
            for (int i = 0; i < size(); ++i)
            {
                ASCII_vector += Convert.ToString(this[i]);
                if (i < size() - 1) ASCII_vector += ",";
            }
            return (ASCII_vector);
        }


        public void Fill(float v)
        {
            for (int i = 0; i < data.Length; i++) data[i] = v;
        }

        public bool Resize(int length)
        {
            bool resized = true;

            if (data.Length != length)
            {
                int l = data.Length;
                data = new float[length];
                for (int i = l; i < length; i++) data[i] = 0;
            }
            else
                resized = false;

            return (resized);
        }

        public bool Resize(uint length)
        {
            return (Resize((int)length));
        }

        public override bool Equals(object obj)
        {
            if (!(obj is Vector))
                return false;

            return this == (Vector)obj;
        }

        public override Int32 GetHashCode()
        {
            return 1;
        }

        // Return the length, number of elements, dimension of this vector.
        public int size() { return data.Length; }

        // Return the length, number of elements, dimension of this vector.
        public int Size() { return data.Length; }

        // Sets elements to ptr[i].
        public void Set(float v)
        {  
            for (int i=0;i<data.Length;i++) data[i] = v;
        }

        // put a value in the given index
        public void Put(int index, float v)
        {
            if (index < data.Length)
                data[index] = v;
        }

        #region "operators"

        public static bool operator == (Vector v1, Vector v2)
        {  
            int i=0;
            bool different = false;

            try
            {
                while ((i < v1.data.Length) && (!different))
                {
                    if (v1[i] != v2[i]) different = true;
                    i++;
                }
            }
            catch
            {
                different = true;
            }

            return(!different);
        }

        public static bool operator != (Vector v1, Vector v2)
        {
           //int state = 0;
            int i = 0;
            bool different = false;

            try
            {
                while ((i < v1.data.Length) && (!different))
                {
                    //state = 1;
                    if (v1[i] != v2[i]) different = true;                        
                    i++;
                }
            }
            catch
            {
                //if (state == 0)
                    different = false;
                //else
                    //different = true;
            }

            return(different);
        }

        public static Vector operator +(Vector v1, Vector v2)
        {
            Vector result = new Vector(v1.data.Length);
            int i = 0;

            while (i < v1.data.Length)
            {
                result[i] = v1[i] + v2[i];
                i++;
            }

            return (result);
        }

        public static Vector operator +(Vector v1, float v2)
        {
            Vector result = new Vector(v1.data.Length);
            int i = 0;

            while (i < v1.data.Length)
            {
                result[i] = v1[i] + v2;
                i++;
            }

            return (result);
        }

        public static Vector operator -(Vector v1, Vector v2)
        {
            Vector result = new Vector(v1.data.Length);
            int i = 0;

            while (i < v1.data.Length)
            {
                result[i] = v1[i] - v2[i];
                i++;
            }

            return (result);
        }

        public static Vector operator -(Vector v1, float v2)
        {
            Vector result = new Vector(v1.data.Length);
            int i = 0;

            while (i < v1.data.Length)
            {
                result[i] = v1[i] - v2;
                i++;
            }

            return (result);
        }

        public static Vector operator /(Vector v1, Vector v2)
        {
            Vector result = new Vector(v1.data.Length);
            int i = 0;

            while (i < v1.data.Length)
            {
                result[i] = v1[i] / v2[i];
                i++;
            }

            return (result);
        }

        public static Vector operator /(Vector v1, float v2)
        {
            Vector result = new Vector(v1.data.Length);
            int i = 0;

            while (i < v1.data.Length)
            {
                result[i] = v1[i] / v2;
                i++;
            }

            return (result);
        }

        public static Vector operator *(Vector v1, Vector v2)
        {
            Vector result = new Vector(v1.data.Length);
            int i = 0;

            while (i < v1.data.Length)
            {
                result[i] = v1[i] * v2[i];
                i++;
            }

            return (result);
        }

        public static Vector operator *(Vector v1, float v2)
        {
            Vector result = new Vector(v1.data.Length);
            int i = 0;

            while (i < v1.data.Length)
            {
                result[i] = v1[i] * v2;
                i++;
            }

            return (result);
        }

        public void Equals (Vector v1)
        {
            int i=0;
            while (i < v1.data.Length)
            {
                data[i] = v1[i];
                i++;
            }
        }

        #endregion

        // Return reference to the element at specified index.\ No range checking.        
        public float this[int i]
        {
            get
            {
                return data[i];
            }
            set
            {
                data[i] = value;
            }
        }

        // return the data
        //public float[] DataBlock () { return data; }

        // Euclidean Distance between two vectors.
        // Sum of Differences squared.
        public float VectorSSD (Vector v1, Vector v2)
        {
            int i=0;
            float result=0, diff;

            while (i<data.Length)
            {
                diff = v1[i] - v2[i];
                diff *= diff;
                if (i==0)
                    result = diff;
                else
                    result += diff;
                i++;
            }
            return(result);
        }

        /// <summary>
        /// Project a homogeneous vector down to a non-homogeneous one. Returns a
        /// new vector containing all the elements of the input vector, apart from the
        /// last one, divided by the last one.
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public Vector Project(Vector v)
        {
            int newsize = v.Size()-1;
            float factor = v[newsize];
            Vector ret = new Vector(newsize);
            for (int i = 0; i < newsize; i++)
                ret[i] = v[i] / factor;
            return ret;
        }

        /// <summary>
        /// Convert a non-homogeneous vector into a homogeneous vector. Returns a
        /// new vector which is the input vector augmented by one more element
        /// of value 1.0.
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public Vector Unproject(Vector v)
        {
            int n = v.Size();
            Vector ret = new Vector(n+1);
            for (int i = 0; i < n; i++)
                ret[i] = v[i];
            ret[n] = 1;
            return ret;
        }

        /// <summary>
        /// Replaces elements with index begining at start, by values of v.
        /// </summary>
        /// <param name="v"></param>
        /// <param name="start"></param>
        /// <returns></returns>
        public Vector Update (Vector v, int start) 
        {
            int end = start + v.size();
            if ((start < data.Length) && (end <= data.Length))
            {
                for (int i = start; i < end; i++)
                {
                    data[i] = v.data[i - start];
                }
            }
            return this;
        }

        /// <summary>
        /// Convert a non-homogeneous vector into a homogeneous vector. Returns a
        /// new vector which is the input vector augmented by one more element
        /// of value 1.0.
        /// </summary>
        public Vector Update(Vector v)
        {
            return(Update(v,0));
        }


        /// <summary>
        /// Returns a subvector specified by the start index and length.
        /// </summary>
        /// <param name="len">length of the subvector</param>
        /// <param name="start">start position</param>
        /// <returns></returns>
        public Vector Extract (int len, int start) 
        {
            Vector result = new Vector(len);
            for (int i = 0; i < len; i++)
                result[i] = data[start+i];
            return result;
        }

        public Vector Extract(int len)
        {
            return (Extract(len, 0));
        }

        public void CopyIn(float[] ptr)
        {
            for (int i = 0; i < data.Length; ++i)
                data[i] = ptr[i];
        }
        
        public void CopyIn(Vector v)
        {
            for (int i = 0; i < data.Length; ++i)
                data[i] = v[i];
        }

        public void CopyOut(float[] ptr)
        {
            for (int i = 0; i < data.Length; ++i)
                ptr[i] = data[i];
        }

        public void CopyOut(Vector v)
        {
            for (int i = 0; i < data.Length; ++i)
                v[i] = data[i];
        }

        /// <summary>
        /// Returns new vector which is the multiplication of row vector v with matrix m.\ O(m*n).
        /// </summary>
        /// <param name="m"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        public static Vector operator *(Vector v, MatrixFixed m) 
        {  
            Vector result = new Vector(m.Columns);              // Temporary
            MatrixFixed mm = m;                                 // Drop const for get()
            float[] result_data = result.Datablock();
            float[] v_data = v.Datablock();
            float[,] mm_data = mm.Datablock();

            for (int i = 0; i < m.Columns; i++) 
            {  // For each index
                result_data[i] = 0;                                 // Initialize element value
                for (int k = 0; k < result_data.Length; k++)        // Loop over column values
                    result_data[i] += (v_data[k] * mm_data[k, i]);  // Multiply  
            }
            return result;
        }


        public static Vector operator* (MatrixFixed m, Vector v) 
        {
            Vector result = new Vector(m.Rows);        // Temporary
            MatrixFixed mm = m;                        // Drop const for get()
            float[] result_data = result.Datablock();
            float[] v_data = v.Datablock();
            float[,] mm_data = mm.Datablock();
            int vsize = v.size();

            for (int i = 0; i < m.Rows; i++) 
            {                                                     // For each index
                result_data[i] = 0;                               // Initialize element value
                for (int k = 0; k < vsize; k++)                // Loop over column values
                    result_data[i] += (mm_data[i,k] * v_data[k]); // Multiply
            }
            return result;
        }


        /// <summary>
        /// return root mean squares
        /// </summary>
        /// <returns></returns>
        public float RMS()
        {
            float result = 0;
            for (int i = 0; i < data.Length; i++)
                result += (data[i] * data[i]);
            if (result > 0)
                return ((float)Math.Sqrt(result / data.Length));
            else
                return (0.0f);
        }

        /// <summary>
        /// return the squared magnitude of the vector
        /// </summary>
        /// <returns></returns>
        public float SquaredMagnitude()
        {
            float result = 0;
            for (int i = 0; i < data.Length; i++)
                result += (data[i] * data[i]);
            return (result);
        }

        /// <summary>
        /// return the magnitude of the vector
        /// </summary>
        /// <returns></returns>
        public float Magnitude()
        {
            float result = 0;
            for (int i = 0; i < data.Length; i++)
                result += (data[i] * data[i]);
            return ((float)Math.Sqrt(result));
        }

        /// <summary>
        /// return the dot product of two vectors
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static float DotProduct(Vector a, Vector b)
        {
            float ip = 0;
            for (int i = 0; i < a.data.Length; ++i)
                ip += a[i] * b[i];
            return ip;
        }


        /// <summary>
        /// Generate a fortran column-storage matrix from the given matrix.
        /// </summary>
        /// <param name="M"></param>
        public static Vector fortran_copy(MatrixFixed M)
        {
            int n = M.Rows;
            int p = M.Columns;
            Vector result = new Vector(n * p);

            int v = 0;
            for (int j = 0; j < p; ++j)
                for (int i = 0; i < n; ++i)
                {
                    result[v] = M[i, j];
                    v++;
                }
            return (result);
        }


        /// <summary>
        /// Returns the nxn outer product of two nd-vectors, or [v1]^T*[v2].\ O(n).
        /// </summary>
        /// <param name="v1"></param>
        /// <param name="v2"></param>
        /// <returns></returns>
        public static MatrixFixed OuterProduct(Vector v1, Vector v2) 
        {
            MatrixFixed outp = new MatrixFixed(v1.size(), v2.size());
            for (int i = 0; i < outp.Rows; i++)             // v1.column() * v2.row()
                for (int j = 0; j < outp.Columns; j++)
                    outp[i,j] = v1[i] * v2[j];
            return outp;
        }


        /// <summary>
        /// Convert a vector into a 1-by-size matrix (i.e. a row vector).
        /// </summary>
        /// <returns></returns>
        public MatrixFixed AsRow()
        {
            MatrixFixed ret = new MatrixFixed(1,data.Length);
            ret.SetRow(0,this);
            return ret;
        }
  
        /// <summary>
        /// Convert a vector into a 1-by-size matrix (i.e. a column vector).
        /// </summary>
        /// <returns></returns>
        public MatrixFixed AsColumn()
        {
            MatrixFixed ret = new MatrixFixed(data.Length, 1);
            ret.SetColumn(0, this);
            return ret;
        }

    }


    /*
    public class Vector<dataType>
    {
        protected dataType[] data;

        public Vector(int length)
        {
            data = new dataType[length];
        }

        public Vector(uint length)
        {
            data = new dataType[(int)length];
        }

        public IEnumerator<dataType> GetEnumerator()
        {
            foreach (dataType v in data)
            {
                yield return v;
            }
        }

        public override bool Equals(object obj)
        {
            if (!(obj is Vector<dataType>))
                return false;

            return this == (Vector<dataType>)obj;
        }

        public override Int32 GetHashCode()
        {
            return 1;
        }

        // Return the length, number of elements, dimension of this vector.
        public int size() { return data.Length; }

        // Return the length, number of elements, dimension of this vector.
        public int Size() { return data.Length; }

        // Sets elements to ptr[i].
        public void Set(dataType v)
        {
            for (int i = 0; i < data.Length; i++) data[i] = v;
        }

        // put a value in the given index
        public void Put(int index, dataType v)
        {
            if (index < data.Length)
                data[index] = v;
        }

        #region "operators"

        public static bool operator==(Vector v1, Vector v2)
        {
            int i = 0;
            bool different = false;

            while ((i < v1.data.Length) && (!different))
            {
                if (v1[i] != v2[i]) different = true;
                i++;
            }

            return (!different);
        }

        public static bool operator !=(Vector v1, Vector v2)
        {
            int i = 0;
            bool different = false;

            while ((i < v1.data.Length) && (!different))
            {
                if (v1[i] != v2[i]) different = true;
                i++;
            }

            return (different);
        }

        public static Vector<dataType> operator +(Vector<dataType> v1, Vector<dataType> v2)
        {
            Vector<dataType> result = new Vector<dataType>(v1.data.Length);
            int i = 0;

            while (i < v1.data.Length)
            {
                result[i] = v1[i] + v2[i];
                i++;
            }

            return (result);
        }

        public static Vector<dataType> operator -(Vector<dataType> v1, Vector<dataType> v2)
        {
            Vector<dataType> result = new Vector<dataType>(v1.data.Length);
            int i = 0;

            while (i < v1.data.Length)
            {
                result[i] = v1[i] - v2[i];
                i++;
            }

            return (result);
        }

        public static Vector<dataType> operator /(Vector<dataType> v1, Vector<dataType> v2)
        {
            Vector<dataType> result = new Vector<dataType>(v1.data.Length);
            int i = 0;

            while (i < v1.data.Length)
            {
                result[i] = v1[i] / v2[i];
                i++;
            }

            return (result);
        }

        public static Vector<dataType> operator *(Vector<dataType> v1, Vector<dataType> v2)
        {
            Vector<dataType> result = new Vector<dataType>(v1.data.Length);
            int i = 0;

            while (i < v1.data.Length)
            {
                result[i] = v1[i] * v2[i];
                i++;
            }

            return (result);
        }

        public void Equals(Vector<dataType> v1)
        {
            int i = 0;
            while (i < v1.data.Length)
            {
                data[i] = v1[i];
                i++;
            }
        }

        #endregion

        // Return reference to the element at specified index.\ No range checking.        
        public dataType this[int i]
        {
            get
            {
                return data[i];
            }
            set
            {
                data[i] = value;
            }
        }

        // return the data
        //public dataType[] DataBlock () { return data; }

        // Euclidean Distance between two vectors.
        // Sum of Differences squared.
        public dataType VectorSSD(Vector<dataType> v1, Vector<dataType> v2)
        {
            int i = 0;
            dataType result = 0, diff;

            while (i < data.Length)
            {
                diff = v1[i] - v2[i];
                diff *= diff;
                if (i == 0)
                    result = diff;
                else
                    result += diff;
                i++;
            }
            return (result);
        }

        /// <summary>
        /// Project a homogeneous vector down to a non-homogeneous one. Returns a
        /// new vector containing all the elements of the input vector, apart from the
        /// last one, divided by the last one.
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public Vector<dataType> Project(Vector<dataType> v)
        {
            int newsize = v.Size() - 1;
            dataType factor = v[newsize];
            Vector<dataType> ret = new Vector<dataType>(newsize);
            for (int i = 0; i < newsize; i++)
                ret[i] = v[i] / factor;
            return ret;
        }

        /// <summary>
        /// Convert a non-homogeneous vector into a homogeneous vector. Returns a
        /// new vector which is the input vector augmented by one more element
        /// of value 1.0.
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public Vector<dataType> Unproject(Vector<dataType> v)
        {
            int n = v.Size();
            Vector<dataType> ret = new Vector<dataType>(n + 1);
            for (int i = 0; i < n; i++)
                ret[i] = v[i];
            ret[n] = 1;
            return ret;
        }

        /// <summary>
        /// Replaces elements with index begining at start, by values of v.
        /// </summary>
        /// <param name="v"></param>
        /// <param name="start"></param>
        /// <returns></returns>
        public Vector<dataType> Update(Vector<dataType> v, int start)
        {
            int end = start + v.size();
            if ((start < data.Length) && (end < data.Length))
            {
                for (int i = start; i < end; i++)
                {
                    data[i] = v.data[i - start];
                }
            }
            return this;
        }

        /// <summary>
        /// Convert a non-homogeneous vector into a homogeneous vector. Returns a
        /// new vector which is the input vector augmented by one more element
        /// of value 1.0.
        /// </summary>
        public Vector<dataType> Update(Vector<dataType> v)
        {
            return (Update(v, 0));
        }


        /// <summary>
        /// Returns a subvector specified by the start index and length.
        /// </summary>
        /// <param name="len">length of the subvector</param>
        /// <param name="start">start position</param>
        /// <returns></returns>
        public Vector<dataType> Extract(int len, int start)
        {
            Vector<dataType> result = new Vector<dataType>(len);
            for (int i = 0; i < len; i++)
                result[i] = data[start + i];
            return result;
        }

        public void CopyIn(dataType[] ptr)
        {
            for (int i = 0; i < data.Length; ++i)
                data[i] = ptr[i];
        }

        public void CopyIn(Vector<dataType> v)
        {
            for (int i = 0; i < data.Length; ++i)
                data[i] = v[i];
        }

        public void CopyOut(dataType[] ptr)
        {
            for (int i = 0; i < data.Length; ++i)
                ptr[i] = data[i];
        }

        public void CopyOut(Vector<dataType> v)
        {
            for (int i = 0; i < data.Length; ++i)
                v[i] = data[i];
        }

    }
    */

}
