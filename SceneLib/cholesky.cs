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
    /// <summary>
    /// Decomposition of symmetric matrix.
    /// A class to hold the Cholesky decomposition of a symmetric matrix and
    /// use that to solve linear systems, compute determinants and inverses.
    /// The cholesky decomposition decomposes symmetric A = L*L.transpose()
    /// where L is lower triangular, and it is therefore possible to save a
    /// little time and pass it a matrix with only the lower triangle filled
    /// in (the upper is assumed symmetric). No test for symmetry is performed.
    ///
    /// To check that the decomposition can be used safely for solving a linear
    /// equation it is wise to construct with mode==estimate_condition and
    /// check that rcond()>sqrt(machine precision).  If this is not the case
    /// it might be a good idea to use VNL::SVD instead.
    /// </summary>
    public class Cholesky
    {
        // Modes of computation.\  See constructor for details.
        public enum Operation { quiet, verbose, estimate_condition }

        // Return the decomposition matrix.
        public MatrixFixed LBadlyNamedMethod() { return A_; }

        // A Success/failure flag.
        public int RankDeficiency() { return num_dims_rank_def_; }

        /// <summary>
        /// Return reciprocal condition number (smallest/largest singular values).
        /// As long as rcond()>sqrt(precision) the decomposition can be used for
        /// solving equations safely.
        /// Not calculated unless Operaton mode at construction was estimate_condition.
        /// </summary>
        /// <returns></returns>
        public float RCond() { return rcond_; }

        // Return computed nullvector.
        // Not calculated unless Operaton mode at construction was estimate_condition.
        public Vector Nullvector() { return nullvector_; }

        protected MatrixFixed A_;
        protected float rcond_;
        protected int num_dims_rank_def_;
        protected Vector nullvector_;

        /// <summary>
        /// Cholesky decomposition.
        /// Make cholesky decomposition of M optionally computing
        /// the reciprocal condition number.  If mode is estimate_condition, the
        /// condition number and an approximate nullspace are estimated, at a cost
        /// of a factor of (1 + 18/n).  Here's a table of 1 + 18/n:
        ///<pre>
        /// n:              3      5     10     50    100    500   1000
        /// slowdown:     7.0f    4.6    2.8    1.4   1.18   1.04   1.02
        /// </summary>
        /// <param name="M"></param>
        /// <param name="mode"></param>
        public unsafe void init(MatrixFixed M, Operation mode)
        {
            A_ = new MatrixFixed(M);

            int n = M.Columns;
            //assert(n == (int)(M.Rows()));
            num_dims_rank_def_ = -1;
            int num_dims_rank_def_temp = num_dims_rank_def_;

            // BJT: This warning is pointless - it often doesn't detect non symmetry and
            // if you know what you're doing you don't want to be slowed down
            // by a cerr
            /*
               if (Math.Abs(M[0,n-1] - M[n-1,0]) > 1e-8) 
               {
                   Debug.WriteLine("cholesky: WARNING: unsymmetric: " + M);
               }
            */

            if (mode != Operation.estimate_condition) 
            {
                // Quick factorization
                fixed (float* data = A_.Datablock())
                {
                    Netlib.dpofa_(data, &n, &n, &num_dims_rank_def_temp);                    
                }
                //if ((mode == Operation.verbose) && (num_dims_rank_def_temp != 0))
                //    Debug.WriteLine("cholesky:: " + Convert.ToString(num_dims_rank_def_temp) + " dimensions of non-posdeffness");
            } 
            else 
            {
                Vector nullvector = new Vector(n);
                float rcond_temp = rcond_;
                fixed (float* data = A_.Datablock())
                {
                    fixed (float* data2 = nullvector.Datablock())
                    {
                        Netlib.dpoco_(data, &n, &n, &rcond_temp, data2, &num_dims_rank_def_temp);
                    }
                }
                rcond_ = rcond_temp;
            }
            num_dims_rank_def_ = num_dims_rank_def_temp;
        }

        public unsafe Cholesky(MatrixFixed M, Operation mode)
        {
            init(M, mode);
        }

        public Cholesky(MatrixFixed M)
        {
            init(M, Operation.verbose);
        }

        /// <summary>
        /// Solve least squares problem M x = b.
        /// The right-hand-side std::vector x may be b,
        /// which will give a fractional increase in speed.
        /// </summary>
        /// <param name="b"></param>
        /// <param name="x"></param>
        public unsafe void Solve(Vector b, Vector x)
        {
            //assert(b.size() == A_.Columns());

            x = b;
            int n = A_.Columns;
            fixed (float* data = A_.Datablock())
            {
                fixed (float* data2 = x.Datablock())
                {
                    Netlib.dposl_(data, &n, &n, data2);
                }
            }
        }

        /// <summary>
        /// Solve least squares problem M x = b.
        /// </summary>
        /// <param name="b"></param>
        /// <returns></returns>
        public unsafe Vector Solve(Vector b)
        {
            //assert(b.size() == A_.Columns());

            int n = A_.Columns;
            Vector ret = new Vector(b);
            fixed (float* data = A_.Datablock())
            {
                fixed (float* data2 = ret.Datablock())
                {
                    Netlib.dposl_(data, &n, &n, data2);
                }
            }
            return ret;
        }

        /// <summary>
        /// Compute determinant.
        /// </summary>
        /// <returns></returns>
        public unsafe float Determinant()
        {
            int n = A_.Columns;
            MatrixFixed I = new MatrixFixed(A_);
            float[] det = new float[2];
            int job = 10;
            fixed (float* data = I.Datablock())
            {
                fixed (float* data2 = det)
                {
                    Netlib.dpodi_(data, &n, &n, data2, &job);
                }
            }
            return det[0] * (float)Math.Pow(10.0, det[1]);
        }

        /// <summary>
        /// Compute inverse.\  Not efficient.
        /// </summary>
        /// <returns></returns>
        public unsafe MatrixFixed Inverse()
        {
            int n = A_.Columns;
            MatrixFixed I = new MatrixFixed(A_);
            float[] det = new float[2];
            int job = 01;
            fixed (float* data = I.Datablock())
            {
                fixed (float* data2 = det)
                {
                    Netlib.dpodi_(data, &n, &n, data2, &job);
                }
            }

            // Copy lower triangle into upper
            for (int i = 0; i < n; ++i)
                for (int j = i+1; j < n; ++j)
                    I[i,j] = I[j,i];

            return I;
        }

  
        /// <summary>
        /// Return lower-triangular factor.
        /// </summary>
        /// <returns></returns>
        public MatrixFixed LowerTriangle()
        {
            int n = A_.Columns;
            MatrixFixed L = new MatrixFixed(n,n);
            // Zap upper triangle and transpose
            for (int i = 0; i < n; ++i) 
            {
                L[i,i] = A_[i,i];
                for (int j = i+1; j < n; ++j) 
                {
                    L[j,i] = A_[j,i];
                    L[i,j] = 0;
                }
            }
            return L;
        }


        /// <summary>
        /// Return upper-triangular factor.
        /// </summary>
        /// <returns></returns>
        private MatrixFixed UpperTriangle()
        {
            int n = A_.Columns;
            MatrixFixed U = new MatrixFixed(n, n);
            // Zap lower triangle and transpose
            for (int i = 0; i < n; ++i)
            {
                U[i, i] = A_[i, i];
                for (int j = i + 1; j < n; ++j)
                {
                    U[i, j] = A_[j, i];
                    U[j, i] = 0;
                }
            }
            return U;
        }


    }

}
