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
using System.Text;

namespace SceneLibrary
{
    /// <summary>
    /// Summary description for MatrixFixed.
    /// </summary>
    public class MatrixFixed
    {
        protected int m_rows;
        protected int m_columns;
        protected float[,] m_matrix;

        private void init(int rows, int columns)
        {
            //
            // TODO: Add constructor logic here
            //

            m_rows = rows;
            m_columns = columns;

            m_matrix = new float[rows, columns];
            //Fill(0);
        }

        public MatrixFixed(uint rows, uint columns)
        {
            init((int)rows,(int)columns);
        }

        public MatrixFixed(int rows, int columns)
        {
            init(rows, columns);
        }

        public MatrixFixed(MatrixFixed m)
        {
            init(m.Rows, m.Columns);
            for (int i = 0; i < Rows; i++)
                for (int j = 0; j < Columns; j++)
                    m_matrix[i, j] = m[i, j];
        }


        public MatrixFixed(int rows, int columns, float[] ary)
        {
            init(rows, columns);
            CopyIn(ary);
        }

        public MatrixFixed(float[] ary)
        {
            int m_dim = (int)Math.Sqrt(ary.Length);
            init(m_dim, m_dim);
            CopyIn(ary);             
        }

        /// <summary>
        /// read the matrix from a string.  This is used when reading from file
        /// </summary>
        /// <param name="s"></param>
        /// <returns></returns>
        public bool ReadASCII(String s)
        {
            int n = 0;
            String[] values = s.Split(',');

            if (values.Length > 0)
            {
                for (int j = 0; j < Columns; j++)
                {
                    for (int i = 0; i < Rows; i++)
                    {
                        m_matrix[i, j] = Convert.ToSingle(values[n]);
                        n++;
                    }
                }
                return true;
            }
            else return false;
        }

        /// <summary>
        /// returns a string representation of the matrix values.  This is used when writing to file.
        /// </summary>
        /// <returns></returns>
        public String WriteASCII()
        {
            String ASCII_value = "";

            for (int j = 0; j < Columns; j++)
            {
                for (int i = 0; i < Rows; i++)
                {
                    ASCII_value += Convert.ToString(m_matrix[i, j]);
                    if (!((i == Rows-1) && (j == Columns-1))) ASCII_value += ",";
                }
            }
            return (ASCII_value);
        }

        public bool Resize(int rows, int columns)
        {
            bool resized = true;

            if (!((Rows == rows) && (Columns == columns)))
                init(rows, columns);
            else
                resized = false;

            return (resized);
        }

        public bool Resize(uint rows, uint columns)
        {
            return(Resize((int)rows,(int)columns));
        }

        public float this[int row, int column]
        {
            get
            {
                return m_matrix[row, column];
            }
            set
            {
                m_matrix[row, column] = value;
            }
        }

        /// <summary>
        /// return a single column of the matrix at the given row number
        /// </summary>
        /// <param name="row"></param>
        /// <returns></returns>
        public float[] this[int row]
        {
            get
            {
                float[] col = new float[Columns];
                for (int i = 0; i < Columns; i++) col[i] = m_matrix[row, i];
                return col;
            }
            set
            {
                for (int i = 0; i < Columns; i++) m_matrix[row, i] = value[i];
            }
        }

        public float[,] Datablock()
        {
            return (m_matrix);
        }

        /*
        public float this[int index]
        {            
            get
            {
                int row = index / Columns;
                int column = index - (row * Columns);
                return m_matrix[row, column];
            }
            set
            {
                int row = index / Columns;
                int column = index - (row * Columns);
                m_matrix[row, column] = value;
            }
        }
        */

        public void Put(int row, int col, float v)
        {
            if ((row >= 0) && (row < Rows) && (col >= 0) && (col < Columns))
            {
                m_matrix[row, col] = v;
            }
        }

        public void Fill(float v)
        {
            for (int i = 0; i < Rows; i++)
                for (int j = 0; j < Columns; j++)
                    m_matrix[i, j] = v;
        }

        /// <summary>
        /// get value with boundary checking
        /// </summary>
        public float Get(int row, int column)
        {
            if ((row >= 0) && (row < m_rows) && (column >= 0) && (column < m_columns))
                return m_matrix[row, column];
            else
                return (0);
        }

        public int Rows
        {
            get
            {
                return m_rows;
            }
        }

        public int Columns
        {
            get
            {
                return m_columns;
            }
        }

        #region overrides

        public override string ToString()
        {
            int maxSpace = 0;

            //find the string length of the longest number in the matrix
            for (int i = 0; i < m_rows; i++)
            {
                for (int j = 0; j < m_columns; j++)
                {
                    int currentLen = m_matrix[i, j].ToString().Length;

                    if (maxSpace < currentLen)
                    {
                        maxSpace = currentLen;
                    }
                }
            }

            //max space needed is one char more than the longest number
            maxSpace++;

            //calculate an approximate value for the string builder length
            StringBuilder sb = new StringBuilder(maxSpace + (m_rows * m_columns));

            for (int i = 0; i < m_rows; i++)
            {
                for (int j = 0; j < m_columns; j++)
                {
                    string currentEle = m_matrix[i, j].ToString();

                    sb.Append(' ', maxSpace - currentEle.Length);
                    sb.Append(currentEle);
                }

                sb.Append("\n");
            }

            return sb.ToString();
        }

        public override int GetHashCode()
        {
            float result = 0;

            for (int i = 0; i < m_rows; i++)
            {
                for (int j = 0; j < m_columns; j++)
                {
                    result += m_matrix[i, j];
                }
            }

            return (int)result;
        }

        public override bool Equals(Object obj)
        {
            MatrixFixed mtx = (MatrixFixed)obj;

            int rows = mtx.Rows;
            int cols = mtx.Columns;

            float[,] matrix_data = mtx.Datablock();

            if (this.Rows != rows || this.Columns != cols)
                return false;

            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    if (m_matrix[i, j] != matrix_data[i, j])
                        return false;
                }
            }

            return true;
        }

        #endregion

        #region operators

        //equality
        public static bool operator ==(MatrixFixed lmtx, MatrixFixed rmtx)
        {
            return Equals(lmtx, rmtx);
        }

        //inequality
        public static bool operator !=(MatrixFixed lmtx, MatrixFixed rmtx)
        {
            return !(lmtx == rmtx);
        }


        //multiplication by matrix
        public static MatrixFixed operator *(MatrixFixed lmtx, MatrixFixed rmtx)
        {
            int left_rows = lmtx.Rows;
            int left_columns = lmtx.Columns;
            int right_rows = rmtx.Rows;
            int right_columns = rmtx.Columns;

            if (left_columns != right_rows)
                throw new MatrixFixedException("Attempt to multiply matrices with unmatching row and column indexes");
            //return null;

            MatrixFixed result = new MatrixFixed(left_rows, right_columns);

            float[,] result_matrix = result.Datablock();
            float[,] left_matrix = lmtx.Datablock();
            float[,] right_matrix = rmtx.Datablock();

            for (int i = 0; i < left_rows; i++)
                for (int j = 0; j < right_columns; j++)
                    for (int k = 0; k < right_rows; k++)
                        result_matrix[i, j] += left_matrix[i, k] * right_matrix[k, j];

            return result;
        }

        //multiplication by const
        public static MatrixFixed operator *(MatrixFixed mtx, float val)
        {
            int rows = mtx.Rows;
            int cols = mtx.Columns;

            MatrixFixed result = new MatrixFixed(rows, cols);

            float[,] result_matrix = result.Datablock();
            float[,] matrix_data = mtx.Datablock();

            for (int i = 0; i < rows; i++)
                for (int j = 0; j < cols; j++)
                    result_matrix[i, j] = matrix_data[i, j] * val;

            return result;
        }

        //multiplication by const
        public static MatrixFixed operator *(float val, MatrixFixed mtx)
        {
            return mtx * val;
        }

        //devision by const
        public static MatrixFixed operator /(MatrixFixed mtx, float val)
        {
            int rows = mtx.Rows;
            int cols = mtx.Columns;

            MatrixFixed result = new MatrixFixed(rows, cols);

            float[,] result_matrix = result.Datablock();
            float[,] matrix_data = mtx.Datablock();

            if (val == 0)
                throw new MatrixFixedException("Attempt to devide the matrix by zero");
            //return null;

            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    result_matrix[i, j] = matrix_data[i, j] / val;
                }
            }

            return result;
        }

        //n th power
        public static MatrixFixed operator ^(MatrixFixed mtx, float val)
        {

            if (mtx.Rows != mtx.Columns)
                throw new MatrixFixedException("Attempt to find the power of a non square matrix");
            //return null;

            MatrixFixed result = mtx;

            for (int i = 0; i < val; i++)
            {
                result = result * mtx;
            }

            return result;
        }

        //addition
        public static MatrixFixed operator +(MatrixFixed lmtx, MatrixFixed rmtx)
        {
            int left_rows = lmtx.Rows;
            int left_columns = lmtx.Columns;
            int right_rows = rmtx.Rows;
            int right_columns = rmtx.Columns;

            if (left_rows != right_rows || left_columns != right_columns)
                throw new MatrixFixedException("Attempt to add matrixes of different sizes");
            //return null;

            MatrixFixed result = new MatrixFixed(left_rows, left_columns);

            float[,] result_matrix = result.Datablock();
            float[,] left_matrix = lmtx.Datablock();
            float[,] right_matrix = rmtx.Datablock();

            for (int i = 0; i < left_rows; i++)
                for (int j = 0; j < left_columns; j++)
                    result_matrix[i, j] = left_matrix[i, j] + right_matrix[i, j];

            return result;
        }

        //subtraction
        public static MatrixFixed operator -(MatrixFixed lmtx, MatrixFixed rmtx)
        {
            int left_rows = lmtx.Rows;
            int left_columns = lmtx.Columns;
            int right_rows = rmtx.Rows;
            int right_columns = rmtx.Columns;

            if (left_rows != right_rows || left_columns != right_columns)
                throw new MatrixFixedException("Attempt to subtract matrixes of different sizes");
            //return null;

            MatrixFixed result = new MatrixFixed(left_rows, left_columns);

            float[,] result_matrix = result.Datablock();
            float[,] left_matrix = lmtx.Datablock();
            float[,] right_matrix = rmtx.Datablock();

            for (int i = 0; i < left_rows; i++)
                for (int j = 0; j < left_columns; j++)
                    result_matrix[i, j] = left_matrix[i, j] - right_matrix[i, j];

            return result;
        }

        //transpose
        public static MatrixFixed operator ~(MatrixFixed mtx)
        {

            MatrixFixed result = new MatrixFixed(mtx.Columns, mtx.Rows);

            for (int i = 0; i < mtx.Rows; i++)
            {
                for (int j = 0; j < mtx.Columns; j++)
                {
                    result[j, i] = mtx[i, j];
                }
            }

            return result;
        }

        //inverse
        public static MatrixFixed operator !(MatrixFixed mtx)
        {

            if (mtx.Determinant() == 0)
                throw new MatrixFixedException("Attempt to invert a singular matrix");
            //return null;

            //inverse of a 2x2 matrix
            if (mtx.Rows == 2 && mtx.Columns == 2)
            {
                MatrixFixed tempMtx = new MatrixFixed(2, 2);

                tempMtx[0, 0] = mtx[1, 1];
                tempMtx[0, 1] = -mtx[0, 1];
                tempMtx[1, 0] = -mtx[1, 0];
                tempMtx[1, 1] = mtx[0, 0];

                return tempMtx / mtx.Determinant();
            }

            return mtx.Adjoint() / mtx.Determinant();

        }

        #endregion

        #region methods

        //determinent
        public float Determinant()
        {

            float determinent = 0;

            if (this.Rows != this.Columns)
                throw new MatrixFixedException("Attempt to find the determinent of a non square matrix");
            //return 0;

            //get the determinent of a 2x2 matrix
            if (this.Rows == 2 && this.Columns == 2)
            {
                determinent = (m_matrix[0, 0] * m_matrix[1, 1]) - (m_matrix[0, 1] * m_matrix[1, 0]);
                return determinent;
            }

            MatrixFixed tempMtx = new MatrixFixed(Rows - 1, Columns - 1);

            //find the determinent with respect to the first row
            for (int j = 0; j < this.Columns; j++)
            {

                tempMtx = this.Minor(0, j);

                //recursively add the determinents
                determinent += (int)Math.Pow(-1, j) * m_matrix[0, j] * tempMtx.Determinant();

            }

            return determinent;
        }

        //adjoint matrix / conjugate transpose
        public MatrixFixed Adjoint()
        {

            if (this.Rows < 2 || this.Columns < 2)
                throw new MatrixFixedException("Adjoint matrix not available");

            MatrixFixed tempMtx = new MatrixFixed(this.Rows - 1, this.Columns - 1);
            MatrixFixed adjMtx = new MatrixFixed(this.Columns, this.Rows);

            for (int i = 0; i < this.Rows; i++)
            {
                for (int j = 0; j < this.Columns; j++)
                {

                    tempMtx = this.Minor(i, j);

                    //put the determinent of the minor in the transposed position
                    adjMtx[j, i] = (int)Math.Pow(-1, i + j) * tempMtx.Determinant();
                }
            }

            return adjMtx;
        }

        //returns a minor of a matrix with respect to an element
        public MatrixFixed Minor(int row, int column)
        {

            if (this.Rows < 2 || this.Columns < 2)
                throw new MatrixFixedException("Minor not available");

            int i, j = 0;

            MatrixFixed minorMtx = new MatrixFixed(this.Rows - 1, this.Columns - 1);

            //find the minor with respect to the first element
            for (int k = 0; k < minorMtx.Rows; k++)
            {

                if (k >= row)
                    i = k + 1;
                else
                    i = k;

                for (int l = 0; l < minorMtx.Columns; l++)
                {
                    if (l >= column)
                        j = l + 1;
                    else
                        j = l;

                    minorMtx[k, l] = m_matrix[i, j];
                }

            }

            return minorMtx;
        }

        //returns true if the matrix is identity
        public bool IsIdentity()
        {

            if (Rows != Columns)
                return false;

            for (int i = 0; i < Rows; i++)
            {
                for (int j = 0; j < Columns; j++)
                {
                    if (i == j)
                    {
                        if (m_matrix[i, j] != 1.0f) return false;
                    }
                    else
                    {
                        if (m_matrix[i, j] != 0.0f) return false;
                    }
                }
            }

            return true;
        }

        //returns true if the matrix is non singular
        public bool IsInvertible()
        {

            if (this.Determinant() == 0)
            {
                return false;
            }

            return true;
        }

        //makes the matrix an identity matrix
        public void Reset()
        {
            if (m_rows != m_columns)
                throw new MatrixFixedException("Attempt to make non square matrix identity");

            for (int j = 0; j < m_rows; j++)
            {
                for (int i = 0; i < m_columns; i++)
                {
                    if (i == j)
                    {
                        m_matrix[i, j] = 1.0f;
                    }
                    else
                    {
                        m_matrix[i, j] = 0.0f;
                    }
                }
            }
        }

        //makes the matrix a zero matrix
        public void Clear()
        {
            for (int j = 0; j < m_rows; j++)
            {
                for (int i = 0; i < m_columns; i++)
                {
                    m_matrix[i, j] = 0.0f;
                }
            }
        }

        /// <summary>
        /// Set values of this matrix to those of M, starting at [top,left]
        /// </summary>
        /// <param name="m">source matrix</param>
        /// <param name="top">top y coord</param>
        /// <param name="left">left x coord</param>
        /// <returns>the updated matrix</returns>
        public MatrixFixed Update(MatrixFixed m, int top, int left)
        {
            int bottom = top + m.m_rows;
            int right = left + m.m_columns;

            for (int i = top; i < bottom; i++)
                for (int j = left; j < right; j++)
                    m_matrix[i,j] = m[i-top,j-left];
            return this;
        }

        public MatrixFixed Update(MatrixFixed m)
        {
            return (Update(m, 0, 0));
        }

        public void and_pi_range(ref float angle)
        {
            while (angle >= 3.1415927)
                angle -= 2 * 3.1415927f;
            while (angle < -3.1415927)
                angle += 2 * 3.1415927f;
        }

        public static float and_pi_range(float angle)
        {
            while (angle >= 3.1415927)
                angle -= 2 * 3.1415927f;
            while (angle < -3.1415927)
                angle += 2 * 3.1415927f;
            return (angle);
        }

        public static MatrixFixed dq3_by_dq1(Quaternion q2)
        {
            float[] a = new float[16];
            
            a[0] = q2.R();
            a[1] = -q2.X();
            a[2] = -q2.Y();
            a[3] = -q2.Z();

            a[4] = q2.X();
            a[5] = q2.R();
            a[6] = -q2.Z();
            a[7] = q2.Y();

            a[8] = q2.Y();
            a[9] = q2.Z();
            a[10] = q2.R();
            a[11] = -q2.X();

            a[12] = q2.Z();
            a[13] = -q2.Y();
            a[14] = q2.X();
            a[15] = q2.R();

            MatrixFixed M = new MatrixFixed(a);
            return M;
        }

        public static MatrixFixed dq3_by_dq2(Quaternion q1)
        {
            float[] a = new float[16];
            
            a[0] = q1.R();
            a[1] = -q1.X();
            a[2] = -q1.Y();
            a[3] = -q1.Z();

            a[4] = q1.X();
            a[5] = q1.R();
            a[6] = q1.Z();
            a[7] = -q1.Y();

            a[8] = q1.Y();
            a[9] = -q1.Z();
            a[10] = q1.R();
            a[11] = q1.X();

            a[12] = q1.Z();
            a[13] = q1.Y();
            a[14] = -q1.X();
            a[15] = q1.R();

            MatrixFixed M = new MatrixFixed(a);
            return M;
        }

        public static MatrixFixed dqbar_by_dq()
        {
            float[] a = new float[16];
            
            a[0] = 1.0f;
            a[1] = 0.0f;
            a[2] = 0.0f;
            a[3] = 0.0f;

            a[4] = 0.0f;
            a[5] = -1.0f;
            a[6] = 0.0f;
            a[7] = 0.0f;

            a[8] = 0.0f;
            a[9] = 0.0f;
            a[10] = -1.0f;
            a[11] = 0.0f;

            a[12] = 0.0f;
            a[13] = 0.0f;
            a[14] = 0.0f;
            a[15] = -1.0f;

            MatrixFixed M = new MatrixFixed(a);
            return M;
        }

        public static MatrixFixed dRq_times_a_by_dq(Quaternion q, Vector a)
        {
            Vector3D v = new Vector3D(a);
            return (dRq_times_a_by_dq(q, v));
        }

        //returns matrix of dimensions (3,4)
        public static MatrixFixed dRq_times_a_by_dq (Quaternion q, Vector3D a)
        {
            MatrixFixed aMat = new MatrixFixed(3,1);
            aMat[0, 0] = a.GetX(); 
            aMat[1, 0] = a.GetY(); 
            aMat[2, 0] = a.GetZ(); 

            MatrixFixed TempR = new MatrixFixed(3,3);
            MatrixFixed Temp31 = new MatrixFixed(3,1);
            MatrixFixed dRq_times_a_by_dq = new MatrixFixed(3,4);

            // Make Jacobian by stacking four vectors together side by side
            TempR = dR_by_dq0(q);
            Temp31 = TempR * aMat;
            dRq_times_a_by_dq.Update(Temp31, 0, 0);

            TempR = dR_by_dqx(q);
            Temp31 = TempR * aMat;
            dRq_times_a_by_dq.Update(Temp31, 0, 1);

            TempR = dR_by_dqy(q);
            Temp31 = TempR * aMat;
            dRq_times_a_by_dq.Update(Temp31, 0, 2);

            TempR = dR_by_dqz(q);
            Temp31 = TempR * aMat;
            dRq_times_a_by_dq.Update(Temp31, 0, 3);

            return dRq_times_a_by_dq;
        }


        public static MatrixFixed dR_by_dq0(Quaternion q)
        {
            float q0 = q.R();
            float qx = q.X();
            float qy = q.Y();
            float qz = q.Z();

            float[] a = new float[9];
            a[0] = 2*q0;
            a[1] = -2*qz;
            a[2] = 2*qy;

            a[3] = 2*qz;
            a[4] = 2*q0;
            a[5] = -2*qx;

            a[6] = -2*qy;
            a[7] = 2*qx;
            a[8] = 2*q0;

 
            MatrixFixed M = new MatrixFixed(3,3);
            return M;
        }

        public static MatrixFixed dR_by_dqx(Quaternion q)
        {
            float q0 = q.R();
            float qx = q.X();
            float qy = q.Y();
            float qz = q.Z();

            float[] a = new float[9];
            a[0] = 2*qx;
            a[1] = 2*qy;
            a[2] = 2*qz;

            a[3] = 2*qy;
            a[4] = -2*qx;
            a[5] = -2*q0;

            a[6] = 2*qz;
            a[7] = 2*q0;
            a[8] = -2*qx;

            MatrixFixed M = new MatrixFixed(3,3);
            return M;
        }

        public static MatrixFixed dR_by_dqy(Quaternion q)
        {
            float q0 = q.R();
            float qx = q.X();
            float qy = q.Y();
            float qz = q.Z();

            float[] a = new float[9];
            a[0] = -2*qy;
            a[1] = 2*qx;
            a[2] = 2*q0;

            a[3] = 2*qx;
            a[4] = 2*qy;
            a[5] = 2*qz;

            a[6] = -2*q0;
            a[7] = 2*qz;
            a[8] = -2*qy;

            MatrixFixed M = new MatrixFixed(3,3);
            return M;
        }

        public static MatrixFixed dR_by_dqz(Quaternion q)
        {
            float q0 = q.R();
            float qx = q.X();
            float qy = q.Y();
            float qz = q.Z();

            float[] a = new float[9];
            a[0] = -2*qz;
            a[1] = -2*q0;
            a[2] = 2*qx;

            a[3] = 2*q0;
            a[4] = -2*qz;
            a[5] = 2*qy;

            a[6] = 2*qx;
            a[7] = 2*qy;
            a[8] = 2*qz;

            MatrixFixed M = new MatrixFixed(3,3);
            return M;
        }

        public Quaternion normalise_quaternion(Quaternion q)
        {
            // qq is current magnitude of q
            float qq = (float)Math.Sqrt(q.GetR() * q.GetR() + 
		                     q.GetX()*q.GetX() + q.GetY()*q.GetY() + q.GetZ()*q.GetZ());

            Quaternion qnorm = new Quaternion(
                q.GetX() / qq,
		        q.GetY() / qq,
		        q.GetZ() / qq,
		        q.GetR() / qq);
            return qnorm;
        }


        // Auxiliary functions used by dqnorm_by_dq()
        // Value of diagonal element of Jacobian
        public static float dqi_by_dqi(float qi, float qq)
        {
            return (1 - qi*qi / (qq*qq)) / qq;
        }

        // Value of off-diagonal element of Jacobian
        public static float dqi_by_dqj(float qi, float qj, float qq)
        {
            return -qi * qj / (qq*qq*qq);
        }


        public static MatrixFixed dqnorm_by_dq(Quaternion q)
        {
            float qq = q.GetR()*q.GetR() + 
                        q.GetX()*q.GetX() + 
                        q.GetY()*q.GetY() + 
                        q.GetZ()*q.GetZ();


            float[] a = new float[16];
            
            a[0] = dqi_by_dqi(q.GetR(), qq);
            a[1] = dqi_by_dqj(q.GetR(), q.GetX(), qq);
            a[2] = dqi_by_dqj(q.GetR(), q.GetY(), qq);
            a[3] = dqi_by_dqj(q.GetR(), q.GetZ(), qq);

            a[4] = dqi_by_dqj(q.GetX(), q.GetR(), qq);
            a[5] = dqi_by_dqi(q.GetX(), qq);
            a[6] = dqi_by_dqj(q.GetX(), q.GetY(), qq);
            a[7] = dqi_by_dqj(q.GetX(), q.GetZ(), qq);

            a[8] = dqi_by_dqj(q.GetY(), q.GetR(), qq);
            a[9] = dqi_by_dqj(q.GetY(), q.GetX(), qq);
            a[10] = dqi_by_dqi(q.GetY(), qq);
            a[11] = dqi_by_dqj(q.GetY(), q.GetZ(), qq);

            a[12] = dqi_by_dqj(q.GetZ(), q.GetR(), qq);
            a[13] = dqi_by_dqj(q.GetZ(), q.GetX(), qq);
            a[14] = dqi_by_dqj(q.GetZ(), q.GetY(), qq);
            a[15] = dqi_by_dqi(q.GetZ(), qq);

            MatrixFixed M = new MatrixFixed(a);
            return M;		
        }
  

        public static MatrixFixed dvnorm_by_dv(Vector3D v)
        {
            float vv = v.GetX()*v.GetX() + v.GetY()*v.GetY() + v.GetZ()*v.GetZ();

            // We can use dqi_by_dqi and dqi_by_dqj functions because they are the same
            float[] a = new float[9];
            a[0] = dqi_by_dqi(v.GetX(), vv);
            a[1] = dqi_by_dqj(v.GetX(), v.GetY(), vv);
            a[2] = dqi_by_dqj(v.GetX(), v.GetZ(), vv);

            a[3] = dqi_by_dqj(v.GetY(), v.GetX(), vv);
            a[4] = dqi_by_dqi(v.GetY(), vv);
            a[5] = dqi_by_dqj(v.GetY(), v.GetZ(), vv);

            a[6] = dqi_by_dqj(v.GetZ(), v.GetX(), vv);
            a[7] = dqi_by_dqj(v.GetZ(), v.GetY(), vv);
            a[8] = dqi_by_dqi(v.GetZ(), vv);

            MatrixFixed M = new MatrixFixed(3,3);
            return M;
        }


        /// <summary>
        /// Fill this matrix with a row*row identity matrix 
        /// </summary>
        /// <param name="val"></param>
        public void SetIdentity(float val)
        {
            for (int i = 0; i < Rows; i++)    // For each row in the Matrix
                for (int j = 0; j < Columns; j++)  // For each element in column
                    if (i == j)
                        m_matrix[i,j] = val;
                    else
                        m_matrix[i,j] = 0;
        }

        public void SetIdentity()
        {
            SetIdentity(1);
        }

        /// <summary>
        /// Make each row of the matrix have unit norm.
        /// All-zero rows are ignored.
        /// </summary>
        public void NormalizeRows()
        {
            float norm, scale;

            for (int i = 0; i < Rows; i++)
            {  // For each row in the Matrix
                norm = 0;
                for (int j = 0; j < Columns; j++)  // For each element in row
                    norm += (m_matrix[i, j] * m_matrix[i, j]);

                if (norm != 0)
                {
                    scale = 1 / (float)Math.Sqrt(norm);
                    for (int j = 0; j < Columns; j++)
                        m_matrix[i, j] *= scale;
                }
            }
        }

        /// <summary>
        /// Make each column of the matrix have unit norm.
        /// All-zero columns are ignored.
        /// </summary>
        public void NormalizeColumns()
        {
            float norm, scale;

            for (int j = 0; j < Columns; j++)
            {
                // For each column in the Matrix
                norm = 0; // float will not do for all types.
                for (int i = 0; i < Rows; i++)
                    norm += (m_matrix[i, j] * m_matrix[i, j]);

                if (norm != 0)
                {
                    scale = 1 / (float)Math.Sqrt(norm);
                    for (int i = 0; i < Rows; i++)
                    {
                        m_matrix[i, j] *= scale;
                    }
                }
            }
        }


        /// <summary>
        /// Multiply row[row_index] by value.
        /// </summary>
        /// <param name="row_index"></param>
        /// <param name="value"></param>
        public void ScaleRow(int row_index, float value)
        {
            for (int j = 0; j < Columns; j++)    // For each element in row
                m_matrix[row_index,j] *= value;
        }

        /// <summary>
        /// Multiply column[column_index] by value.
        /// </summary>
        /// <param name="column_index"></param>
        /// <param name="value"></param>
        public void ScaleColumn(int column_index, float value)
        {
            for (int j = 0; j < Rows; j++)    // For each element in column
                m_matrix[j,column_index] *= value;
        }

        // Returns a copy of n rows, starting from "row".
        public MatrixFixed GetNRows (int row, int n) 
        {
            MatrixFixed result = new MatrixFixed(n,Columns);
            for (int i=row;i<row+n;i++)
            {
                for (int j=0;j<Columns;j++)
                {
                    result[i-row,j] = m_matrix[i,j];
                }
            }
            return result;
        }

        // Returns a copy of n columns, starting from "column".
        public MatrixFixed GetNColumns (int column, int n) 
        {
            MatrixFixed result = new MatrixFixed(Rows,n);
            for (int i=0;i<Rows;i++)
            {
                for (int j=column;j<column+n;j++)
                {
                    result[i,j-column] = m_matrix[i,j];
                }
            }
            return result;
        }


        /// <summary>
        /// Create a vector out of row[row_index].
        /// </summary>
        /// <param name="row_index"></param>
        /// <returns></returns>
        public Vector GetRow(int row_index)
        {
            Vector v = new Vector(Columns);
            for (int j = 0; j < Columns; j++)    // For each element in row
                v[j] = m_matrix[row_index,j];
            return v;
        }

        /// <summary>
        /// Create a vector out of column[column_index].
        /// </summary>
        /// <param name="column_index"></param>
        /// <returns></returns>
        public Vector GetColumn(int column_index)
        {
            Vector v = new Vector(Rows);
            for (int j = 0; j < Rows; j++)    // For each element in row
                v[j] = m_matrix[j,column_index];
            return v;
        }



        /// <summary>
        /// Set row[row_index] to given value.
        /// </summary>
        /// <param name="row_index"></param>
        /// <param name="v"></param>
        public void SetRow(int row_index, float v)
        {
            for (int j = 0; j < Columns; j++)    // For each element in row
                m_matrix[row_index,j] = v;
        }

        /// <summary>
        /// Set column[column_index] to data at given address.
        /// </summary>
        /// <param name="column_index"></param>
        /// <param name="v"></param>
        public void SetColumn(int column_index, float[] v)
        {
            for (int i = 0; i < Rows; i++)    // For each element in row
                m_matrix[i,column_index] = v[i];
        }

        /// <summary>
        /// Set column[column_index] to given vector.\ No bounds check.
        /// </summary>
        /// <param name="column_index"></param>
        /// <param name="v"></param>
        public void SetColumn(int column_index, Vector v)
        {
            for (int i = 0; i < Rows; i++)    // For each element in row
                m_matrix[i,column_index] = v[i];
        }

        /// <summary>
        /// Set row[row_index] to data at given address.\ No bounds check.
        /// </summary>
        /// <param name="row_index"></param>
        /// <param name="v"></param>
        public void SetRow(int row_index, float[] v)
        {
            for (int j = 0; j < Columns; j++)    // For each element in row
                m_matrix[row_index, j] = v[j];
        }

        /// <summary>
        /// Set row[row_index] to given vector.\ No bounds check.
        /// </summary>
        /// <param name="row_index"></param>
        /// <param name="v"></param>
        public void SetRow(int row_index, Vector v)
        {
            for (int j = 0; j < Columns; j++)    // For each element in row
                m_matrix[row_index, j] = v[j];
        }


        /// <summary>
        /// Set column[column_index] to given value.
        /// </summary>
        /// <param name="column_index"></param>
        /// <param name="v"></param>
        public void SetColumn(int column_index, float v)
        {
            for (int i = 0; i < Rows; i++)    // For each element in row
                m_matrix[i,column_index] = v;
        }

        // Set columns starting at starting_column to given matrix.
        public void SetColumns(int starting_column, MatrixFixed m)
        {
            for (int j = 0; j < Columns; ++j)
                for (int i = 0; i < Rows; i++)    // For each element in row
                    m_matrix[i,starting_column + j] = m[i,j];
        }


        /// <summary>
        /// Return true if maximum absolute deviation of M from identity is <= tol. 
        /// </summary>
        /// <param name="tol"></param>
        /// <returns></returns>
        public bool IsIdentity(float tol)
        {
            bool retval = true;
            float absdev;

            for (int i = 0; i < Rows; ++i)
                for (int j = 0; j < Columns; ++j) 
                {
                    absdev = (i == j) ? (float)Math.Abs(m_matrix[i, j] - 1.0) : (float)Math.Abs(m_matrix[i, j]);
                    if (absdev > tol)
                        retval = false;
                }
            return retval;
        }

        public bool IsZero()
        {
            bool retval = true;

            for (int i = 0; i < Rows; ++i)
                for (int j = 0; j < Columns; ++j)
                    if (!(m_matrix[i, j] == 0))
                        retval = false;

            return retval;
        }

        /// <summary>
        /// Return true if max(abs((*this))) <= tol. 
        /// </summary>
        /// <param name="tol"></param>
        /// <returns></returns>
        public bool IsZero(float tol)
        {
            bool retval = true;

            for (int i = 0; i < Rows; ++i)
                for (int j = 0; j < Columns; ++j)
                    if (Math.Abs(m_matrix[i, j]) > tol)
                        retval = false;

            return retval;
        }

        /// <summary>
        /// Return true if any element of (*this) is nan.
        /// </summary>
        /// <returns></returns>
        public bool HasNaNs()
        {
            bool retval = false;

            for (int i = 0; i < Rows; ++i)
                for (int j = 0; j < Columns; ++j)
                    if (float.IsNaN(m_matrix[i, j]))
                        retval = true;
            return retval;
        }

        /// <summary>
        /// Return false if any element of (*this) is inf or nan.
        /// </summary>
        /// <returns></returns>
        public bool IsFinite()
        {
            bool retval = true;

            for (int i = 0; i < Rows; ++i)
                for (int j = 0; j < Columns; ++j)
                    if (float.IsInfinity(m_matrix[i, j]))
                        retval = false;

            return retval;
        }


        /// <summary>
        /// swap two matrices
        /// </summary>
        /// <param name="that"></param>
        public void Swap(MatrixFixed that)
        {
            int r = that.Rows;
            int c = that.Columns;
            float[,] temp = that.m_matrix;

            that.m_rows = Rows;
            that.m_columns = Columns;
            that.m_matrix = m_matrix;

            this.m_rows = r;
            this.m_columns = c;
            this.m_matrix = temp;
        }

        /// <summary>
        /// Reverse order of rows.\  Name is from Matlab, meaning "flip upside down".
        /// </summary>
        public void FlipUD()
        {
            int n = Rows;
            int colz = Columns;

            int m = n / 2;
            for (int r = 0; r < m; ++r) 
            {
                int r1 = r;
                int r2 = n - 1 - r;
                for (int c = 0; c < colz; ++c) 
                {
                    float tmp = m_matrix[r1, c];
                    m_matrix[r1, c] = m_matrix[r2, c];
                    m_matrix[r2, c] = tmp;
                }
            }
        }

        /// <summary>
        /// Reverse order of columns.
        /// </summary>
        public void FlipLR()
        {
            int n = Columns;
            int rowz = Rows;

            int m = n / 2;
            for (int c = 0; c < m; ++c) 
            {
                int c1 = c;
                int c2 = n - 1 - c;
                for (int r = 0; r < rowz; ++r) 
                {
                    float tmp = m_matrix[r, c1];
                    m_matrix[r, c1] = m_matrix[r, c2];
                    m_matrix[r, c2] = tmp;
                }
            }
        }


        public float operatorOneNorm()
        {    
            float max = 0;
            for (int j=0; j<Columns; ++j) 
            {
                float tmp = 0;
                for (int i=0; i<Rows; ++i)
                    tmp += Math.Abs(m_matrix[i, j]);
                if (tmp > max) max = tmp;
            }
            return max;
        }

        public float operatorInfNorm()
        {
            float max = 0;
            for (int i = 0; i < Rows; ++i) 
            {
                float tmp = 0;
                for (int j = 0; j < Columns; ++j)
                    tmp += Math.Abs(m_matrix[i, j]);
                if (tmp > max)
                    max = tmp;
            }
            return max;
        }


        /// <summary>
        /// Replaces the submatrix of THIS matrix, starting at top left corner, by the elements of matrix m.
        /// This is the reverse of extract().
        /// </summary>
        /// <param name="m"></param>
        /// <param name="top"></param>
        /// <param name="left"></param>
        /// <returns></returns>
        /*
        public MatrixFixed Update(MatrixFixed m,
                                  int top, int left) 
        {
            int bottom = top + m.Rows;
            int right = left + m.Columns;
            for (int i = top; i < bottom; i++)
                for (int j = left; j < right; j++)
                    this[i,j] = m[i-top,j-left];
            return this;
        }
        */

        /// <summary>
        /// Returns a copy of submatrix of THIS matrix, specified by the top-left corner and size in rows, cols.\ O(m*n).
        /// Use update() to copy new values of this submatrix back into THIS matrix.
        /// </summary>
        /// <param name="rowz"></param>
        /// <param name="colz"></param>
        /// <param name="top"></param>
        /// <param name="left"></param>
        /// <returns></returns>
        public MatrixFixed Extract (int rowz, int colz,
                                    int top, int left) 
        {
            MatrixFixed result = new MatrixFixed(rowz, colz);
            for (int i = 0; i < rowz; i++)      // actual copy of all elements
                for (int j = 0; j < colz; j++)    // in submatrix
                    result[i, j] = m_matrix[top + i, left + j];
            return result;
        }

        /// <summary>
        /// Returns the dot product of the two matrices.\ O(m*n).
        /// This is the sum of all pairwise products of the elements m1[i,j]*m2[i,j].
        /// </summary>
        /// <param name="m1"></param>
        /// <param name="m2"></param>
        /// <returns></returns>        
        public static float DotProduct(MatrixFixed m1, MatrixFixed m2) 
        {
            float result = 0;

            for (int i = 0; i < m1.Rows; i++)
            {
                for (int j = 0; j < m1.Columns; j++)
                {
                    result += m1[i, j] * m2[i, j];
                }
            }

            return result;
        }
        

        /// <summary>
        /// Hermitian inner product. O(mn).
        /// </summary>
        /// <param name="m1"></param>
        /// <param name="m2"></param>
        /// <returns></returns>
        /*
        public float InnerProduct (MatrixFixed m1, MatrixFixed m2) 
        {
            return InnerProduct(m1.begin(), m2.begin(), m1.Rows*m1.Columns);
        }
        */

        // conjugating "dot" product.

        private float Conjugate(float x) { return x; }

        public float InnerProduct(MatrixFixed a, float[] b, int n)
        {
            float ip = 0;
            int row = 0, col = 0;
            for (int i = 0; i < n; ++i)
            {
                ip += a[row, col] * Conjugate(b[i]);
                col++;
                if (col >= a.Columns)
                {
                    col = 0;
                    row++;
                }
            }
            return ip;
        }


        public float InnerProduct(MatrixFixed a, MatrixFixed b)
        {
            float ip = 0;
            for (int i = 0; i < a.Rows; i++)
                for (int j = 0; j < a.Columns; j++)
                    ip += a[i, j] * Conjugate(b[i, j]);
            return ip;
        }

        /// <summary>
        /// cos_angle. O(mn).
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public float CosAngle (MatrixFixed a, MatrixFixed b) 
        {
            float ab = InnerProduct(a,b);
            float a_b = (float)Math.Sqrt(Math.Abs(InnerProduct(a, a) * InnerProduct(b, b)));

            return ( ab / a_b );
        }

        /// <summary>
        /// Returns new matrix whose elements are the products m1[ij]*m2[ij].
        /// </summary>
        /// <param name="m1"></param>
        /// <param name="m2"></param>
        /// <returns></returns>
        public MatrixFixed ElementProduct (MatrixFixed  m1, MatrixFixed m2) 
        {
            MatrixFixed result = new MatrixFixed(m1.Rows, m1.Columns);
            for (int i = 0; i < m1.Rows; i++)
                for (int j = 0; j < m1.Columns; j++)
                    result.Put(i,j, m1.Get(i,j) * m2.Get(i,j) );
            return result;
        }

        /// <summary>
        /// Returns new matrix whose elements are the quotients m1[ij]/m2[ij].
        /// </summary>
        /// <param name="m1"></param>
        /// <param name="m2"></param>
        /// <returns></returns>
        public MatrixFixed ElementQuotient (MatrixFixed m1, MatrixFixed m2) 
        {
            MatrixFixed result = new MatrixFixed(m1.Rows, m1.Columns);
            for (int i = 0; i < m1.Rows; i++)
                for (int j = 0; j < m1.Columns; j++)
                    result.Put(i,j, m1.Get(i,j) / m2.Get(i,j) );
            return result;
        }

        /// <summary>
        /// Fill this matrix with the given data.
        /// We assume that p points to a contiguous rows*cols array, stored rowwise.
        /// </summary>
        /// <param name="p"></param>
        public void CopyIn(float[] p)
        {
            int n = 0;

            for (int j = 0; j < m_rows; j++)
                for (int i = 0; i < m_columns; i++)
                {
                    m_matrix[j, i] = p[n];
                    n++;
                }   
        }

        public void CopyIn(Vector p)
        {
            int n = 0;

            for (int j = 0; j < m_rows; j++)
                for (int i = 0; i < m_columns; i++)
                {
                    m_matrix[j, i] = p[n];
                    n++;
                }   
        }

        /// <summary>
        /// Fill the given array with this matrix.
        /// We assume that p points to a contiguous rows*cols array, stored rowwise.
        /// </summary>
        /// <param name="p"></param>
        public void CopyOut(float[] p)
        {
            int n = 0;

            for (int j = 0; j < m_rows; j++)
                for (int i = 0; i < m_columns; i++)
                {
                    p[n] = m_matrix[j, i];
                    n++;
                }   
        }


        // adjoint/hermitian transpose
        public MatrixFixed ConjugateTranspose() 
        {
            return(Adjoint());
        }


        public float FrobeniusNorm()
        {
            float val = 0;
            for (int i = 0; i < m_rows; i++)
                for (int j = 0; j < m_columns; j++)
                    val += (m_matrix[i, j] * m_matrix[i, j]);
            return (float)Math.Sqrt(val);
        }


        /// <summary>
        /// Returns new matrix with rows and columns transposed.
        /// </summary>
        /// <returns></returns>
        public virtual MatrixFixed Transpose()
        {
            MatrixFixed result = new MatrixFixed(m_columns, m_rows);
            for (int i = 0; i < m_columns; i++)
                for (int j = 0; j < m_rows; j++)
                    result[i, j] = m_matrix[j, i];
            return result;
        }

        /// <summary>
        /// Generate a fortran column-storage matrix from the given matrix.
        /// </summary>
        /// <param name="M"></param>
        /*
        public void fortran_copy(MatrixFixed M)
        {
            int n = M.Rows;
            int p = M.Columns;

            for (int j = 0; j < p; ++j)
                for (int i = 0; i < n; ++i)
                    m_matrix[j, j] = M[i, j];
        }
        */


        public void assert_finite()
        {
            if (IsFinite())
                return;
            else
            {
                //matrix has non-finite elements
                Debug.WriteLine("matrix has non-finite elements");
            }
        }



        /// <summary>
        /// Create a vector out of row[row_index].
        /// </summary>
        /// <param name="row_index"></param>
        /// <returns></returns>
        /*
        public Vector GetRow(int row_index)
        {
            Vector v = new Vector(Columns);
            for (int j = 0; j < Columns; j++)    // For each element in row
                v[j] = this[row_index,j];
            return v;
        }

        /// <summary>
        /// Create a vector out of column[column_index].
        /// </summary>
        /// <param name="column_index"></param>
        /// <returns></returns>
        public Vector GetColumn(int column_index)
        {
            Vector v = new Vector(Rows);
            for (int j = 0; j < Rows; j++)    // For each element in row
                v[j] = this[j, column_index];
            return v;
        }
        */



        #endregion


    }

    public class MatrixFixedException : Exception
    {
        public MatrixFixedException(string message)
            : base(message)
        {

        }
    }
}
