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

    /// <summary>
    /// stores a diagonal matrix as a single vector.
    /// DiagMatrix stores a diagonal matrix for time and space efficiency.
    /// Specifically, only the diagonal elements are stored, and some matrix
    /// operations (currently *, + and -) are overloaded to use more efficient
    /// algorithms.
    /// </summary>
    public class DiagMatrix
    {
        /// <summary>
        /// Construct an empty diagonal matrix.
        /// </summary>
        /// <param name="nn"></param>
        public DiagMatrix(int nn) { diagonal_ = new Vector(nn); }

        /// <summary>
        /// Construct a diagonal matrix with diagonal elements equal to value.
        /// </summary>
        /// <param name="nn"></param>
        /// <param name="value"></param>
        public DiagMatrix(int nn, float value)
        {
            diagonal_ = new Vector(nn);
            diagonal_.Fill(value);
        }

        /// <summary>
        /// Construct a diagonal matrix from a vnl_vector.
        /// The vector elements become the diagonal elements.
        /// </summary>
        /// <param name="that"></param>
        public DiagMatrix(Vector that)
        {
            diagonal_ = that;
        }

  
        public DiagMatrix Equals(DiagMatrix that) 
        {
            diagonal_ = that.diagonal_;
            return this;
        }

  
        // Operations----------------------------------------------------------------

        /// <summary>
        /// In-place arithmetic operations.
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public DiagMatrix Multiply(float v) 
        { 
            diagonal_ *= v; 
            return this; 
        }

        /// <summary>
        /// In-place arithmetic operations.
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public DiagMatrix Divide(float v) 
        { 
            diagonal_ /= v; 
            return this; 
        }
  

  
        // Data Access---------------------------------------------------------------

        public float this[int i, int j]
        {
            get
            {
                if (i != j)
                    return (0.0f);
                else
                    return(diagonal_[i]);
            }
            set
            {
                if (i == j) diagonal_[i] = value;
            }
        }

        public float this[int i]
        {
            get
            {
                return(diagonal_[i]);
            }
            set
            {
                diagonal_[i] = value;
            }
        }


  
        // STL style iterators
        public int size() { return diagonal_.size(); }
        public int Size() { return diagonal_.size(); }

        public int Rows
        {
            get
            {
                return diagonal_.size();
            }
        }

        public int Columns
        {
            get
            {
                return diagonal_.size();
            }
        }

        public void Resize(int n) { diagonal_.Resize(n); }
        public void Clear() { diagonal_.Fill(0); }
        public void Fill(float x) { diagonal_.Fill(x); }

        // Return pointer to the diagonal elements as a contiguous 1D C array;.
        public float[] Datablock() { return diagonal_.Datablock(); }

        // Return diagonal elements as a vector.
        public Vector Diagonal() { return diagonal_; }

        // Set diagonal elements using vector.
        public void Set(Vector v)  { diagonal_=v; }

 
        protected Vector diagonal_;


        /// <summary>
        /// Convert a DiagMatrix to a Matrix.
        /// </summary>
        /// <returns></returns>
        public MatrixFixed AsMatrix()
        {
            int len = diagonal_.size();
            MatrixFixed ret = new MatrixFixed(len, len);
            for (int i = 0; i < len; ++i)
            {
                int j;
                for (j = 0; j < i; ++j)
                    ret[i,j] = 0;
                for (j = i+1; j < len; ++j)
                    ret[i,j] = 0;
                ret[i,i] = diagonal_[i];
            }
            return ret;
        }

        /// <summary>
        /// Invert a DiagMatrix in-situ.
        /// Just replaces each element with its reciprocal.
        /// </summary>
        public void InvertInPlace()
        {
            int len = diagonal_.size();
            float[] d = Datablock();
            float one = 1.0f;
            for (int i = 0; i < len; ++i)
                d[i] = one / d[i]; 
        }

        /// <summary>
        /// Return determinant as product of diagonal values.
        /// </summary>
        /// <returns></returns>
        public float Determinant()
        {
            float det = 1.0f;
            float[] d = Datablock();
            int len = diagonal_.size();
            for (int i = 0; i < len; ++i)
                det *= d[i];
            return det;
        }


    }

/*

// Multiply a Matrix by a DiagMatrix.\  Just scales the columns - mn flops.
template <class T> inline VNL::Matrix<T> 
operator* (VNL::Matrix<T> const& A, VNL::DiagMatrix<T> const& D)
{
  VNL::Matrix<T> ret(A.Rows(), A.Columns());
  for (unsigned i = 0; i < A.Rows(); ++i)
    for (unsigned j = 0; j < A.Columns(); ++j)
      ret(i,j) = A(i,j) * D(j,j);
  return ret;
}

// Multiply a DiagMatrix by a Matrix.\  Just scales the rows - mn flops.
template <class T>
inline VNL::Matrix<T> operator* (VNL::DiagMatrix<T> const& D, VNL::Matrix<T> const& A)
{
  VNL::Matrix<T> ret(A.Rows(), A.Columns());
  T const* d = D.DataBlock();
  for (unsigned i = 0; i < A.Rows(); ++i)
    for (unsigned j = 0; j < A.Columns(); ++j)
      ret(i,j) = A(i,j) * d[i];
  return ret;
}

// Add a DiagMatrix to a Matrix.\  n adds, mn copies.
template <class T>
inline VNL::Matrix<T> operator + (VNL::Matrix<T> const& A, VNL::DiagMatrix<T> const& D)
{
  const unsigned n = D.size();
  VNL::Matrix<T> ret(A);
  T const* d = D.DataBlock();
  for (unsigned j = 0; j < n; ++j)
    ret(j,j) += d[j];
  return ret;
}

// Add a Matrix to a DiagMatrix.\  n adds, mn copies.
template <class T>
inline VNL::Matrix<T> operator + (VNL::DiagMatrix<T> const& D, VNL::Matrix<T> const& A)
{
  return A + D;
}

// Subtract a DiagMatrix from a Matrix.\  n adds, mn copies.
template <class T>
inline VNL::Matrix<T> operator - (VNL::Matrix<T> const& A, 
				  VNL::DiagMatrix<T> const& D)
{
  const unsigned n = D.size();
  VNL::Matrix<T> ret(A);
  T const* d = D.DataBlock();
  for (unsigned j = 0; j < n; ++j)
    ret(j,j) -= d[j];
  return ret;
}

// Subtract a Matrix from a DiagMatrix.\  n adds, mn copies.
template <class T>
inline VNL::Matrix<T> operator - (VNL::DiagMatrix<T> const& D, 
				  VNL::Matrix<T> const& A)
{
  const unsigned n = D.size();
  VNL::Matrix<T> ret(n, n);
  T const* d = D.DataBlock();
  for (unsigned i = 0; i < n; ++i)
  {
    for (unsigned j = 0; j < i; ++j)
      ret(i,j) = -A(i,j);
    for (unsigned k = i+1; k < n; ++k)
      ret(i,k) = -A(i,k);
    ret(i,i) = d[i] - A(i,i);
  }
  return ret;
}

// Multiply a DiagMatrix by a Vector.\  n flops.
template <class T>
inline VNL::Vector<T> operator* (VNL::DiagMatrix<T> const& D, 
				 VNL::Vector<T> const& A)
{
  return ElementProduct(D.Diagonal(), A);
}

// Multiply a Vector by a DiagMatrix.\  n flops.
template <class T>
inline VNL::Vector<T> operator* (VNL::Vector<T> const& A, 
				 VNL::DiagMatrix<T> const& D)
{
  return ElementProduct(D.Diagonal(), A);
}

 */
 
}
