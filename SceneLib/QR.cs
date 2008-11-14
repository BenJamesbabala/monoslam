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
    /// Extract the Q*R decomposition of matrix M.
    /// The decomposition is stored in a compact and time-efficient
    /// packed form, which is most easily used via the "solve" and
    /// "determinant" methods.
    /// </summary>
    public class QR
    {
        private MatrixFixed qrdc_out_;
        private Vector qraux_;
        private int[] jpvt_;
        private MatrixFixed Q_;
        private MatrixFixed R_;

        public unsafe QR(MatrixFixed M)
        {
            qrdc_out_ = new MatrixFixed(M.Columns, M.Rows);
            qraux_ = new Vector(M.Columns);
            jpvt_ = new int[M.Rows];
            Q_ = null;
            R_ = null;

            // Fill transposed O/P matrix
            int c = M.Columns;
            int r = M.Rows;
            for (int i = 0; i < r; ++i)
                for (int j = 0; j < c; ++j)
                    qrdc_out_[j,i] = M[i,j];

            int do_pivot = 0; // Enable[!=0]/disable[==0] pivoting.
            for (int i = 0; i < jpvt_.Length; i++) jpvt_[i] = 0;

            Vector work = new Vector(M.Rows);

            fixed (float* data = qrdc_out_.Datablock())
            {
                fixed (float* data2 = qraux_.Datablock())
                {
                    fixed (int* data3 = jpvt_)
                    {
                        fixed (float* data4 = work.Datablock())
                        {
                            Netlib.dqrdc_(data,       // On output, UT is R, below diag is mangled Q
                                          &r, &r, &c,
                                          data2,      // Further information required to demangle Q
                                          data3,
                                          data4,
                                          &do_pivot);
                        }
                    }
                }
            }
        }


        /// <summary>
        /// Return the determinant of M.\  This is computed from M = Q R as follows:.
        /// |M| = |Q| |R|
        /// |R| is the product of the diagonal elements.
        /// |Q| is (-1)^n as it is a product of Householder reflections.
        /// So det = -prod(-r_ii).
        /// </summary>
        /// <returns></returns>
        public float Determinant()
        {  
            int m = Netlib.min((int)qrdc_out_.Columns, (int)qrdc_out_.Rows);
            float det = qrdc_out_[0,0];

            for (int i = 1; i < m; ++i)
                det *= -qrdc_out_[i,i];

            return det;
        }

        /// <summary>
        /// Unpack and return unitary part Q.
        /// </summary>
        /// <returns></returns>
        public MatrixFixed Q()
        {
            int m = qrdc_out_.Columns; // column-major storage
            int n = qrdc_out_.Rows;

            bool verbose = false;

            if (Q_ == null) 
            {
                Q_ = new MatrixFixed(m,m);
                // extract Q.
                if (verbose) 
                {
                    Debug.WriteLine("__FILE__: VNL::QR<T>Q()");
                    Debug.WriteLine("m,n = " + Convert.ToString(m) + ", " + Convert.ToString(n));
                    Debug.WriteLine("qr0 = [" + qrdc_out_ + "];");
                    Debug.WriteLine("aux = [" + qraux_ + "];");
                }

                Q_.SetIdentity();
                MatrixFixed Q = Q_;

                Vector v = new Vector(m);
                v.Fill(0);
                Vector w = new Vector(m);
                w.Fill(0);

                // Golub and vanLoan, p199.  backward accumulation of householder matrices
                // Householder vector k is [zeros(1,k-1) qraux_[k] qrdc_out_[k,:]]
                for (int k = n-1; k >= 0; --k) 
                {
                    if (k >= m) continue;
                    // Make housevec v, and accumulate norm at the same time.
                    v[k] = qraux_[k];
                    float sq = (v[k] * v[k]);
                    for (int j = k+1; j < m; ++j) 
                    {
                        v[j] = qrdc_out_[k,j];
                        sq += (v[j] * v[j]);
                    }
                    //if (verbose) MatlabPrint(std::cerr, v, "v");

                    // Premultiply emerging Q by house(v), noting that v[0..k-1] == 0.
                    // Q_new = (1 - (2/v'*v) v v')Q
                    // or Q -= (2/v'*v) v (v'Q)
                    if (sq > 0.0) 
                    {
                        float scale = 2.0f/sq;
                        // w = (2/v'*v) v' Q
                        for (int i = k; i < m; ++i) 
                        {
                            w[i] = 0.0f;
                            for (int j = k; j < m; ++j)
                                w[i] += scale * v[j] * Q[j, i];
                                //w[i] += scale * ComplexTraits.Conjugate(v[j]) * Q[j, i];
                        }
                        //if (verbose) VNL::MatlabPrint(std::cerr, w, "w");

                        // Q -= v w
                        for (int i = k; i < m; ++i)
                            for (int j = k; j < m; ++j)
                                Q[i,j] -= (v[i]) * (w[j]);
                    }
                }
            }
            return Q_;
        }

        /// <summary>
        /// Unpack and return R.
        /// </summary>
        /// <returns></returns>
        public MatrixFixed R()
        {
            if (R_ == null) 
            {
                int m = qrdc_out_.Columns; // column-major storage
                int n = qrdc_out_.Rows;
                R_ = new MatrixFixed(m,n);
                MatrixFixed R = R_;

                for (int i = 0; i < m; ++i)
                    for (int j = 0; j < n; ++j)
                        if (i > j)
                            R[i, j] = 0.0f;
                        else
                            R[i, j] = qrdc_out_[j,i];
            }

            return R_;
        }


        /// <summary>
        /// JOB: ABCDE decimal
        /// A     B     C     D              E
        /// ---   ---   ---   ---            ---
        /// Qb    Q'b   x     norm(A*x - b)  A*x
        /// 
        /// Solve equation M x = b for x using the computed decomposition.
        /// </summary>
        /// <param name="b"></param>
        /// <returns></returns>
        public unsafe Vector Solve(Vector b)
        {
            int n = qrdc_out_.Columns;
            int p = qrdc_out_.Rows;
            float[] b_data = b.Datablock();
            Vector QtB = new Vector(n);
            Vector x = new Vector(p);

            // see comment above
            int JOB = 100;

            int info = 0;

            fixed (float* data = qrdc_out_.Datablock())
            {
                fixed (float* data2 = qraux_.Datablock())
                {
                    fixed (float* data3 = b_data)
                    {
                        fixed (float* data4 = QtB.Datablock())
                        {
                            fixed (float* data5 = x.Datablock())
                            {

                                Netlib.dqrsl_(data, &n, &n, &p, data2, data3,
                                       (float*)0, data4, data5,
                                       (float*)0, // residual*
                                       (float*)0, // Ax*
                                       &JOB,
                                       &info);
                            }
                        }
                    }
                }
            }

            if (info > 0)
                Debug.WriteLine("__FILE__ : VNL::QR<T>::Solve() : matrix is rank-deficient by " + Convert.ToString(info));
  
            return x;
        }

        /// <summary>
        /// Return residual vector d of M x = b -> d = Q'b.
        /// </summary>
        /// <param name="b"></param>
        /// <returns></returns>
        public unsafe Vector QtB(Vector b)
        {
            int n = qrdc_out_.Columns;
            int p = qrdc_out_.Rows;
            float[] b_data = b.Datablock();
            Vector QtB = new Vector(n);

            // see comment above
            int JOB = 1000;

            int info = 0;

            fixed (float* data = qrdc_out_.Datablock())
            {
                fixed (float* data2 = qraux_.Datablock())
                {
                    fixed (float* data3 = b_data)
                    {
                        fixed (float* data4 = QtB.Datablock())
                        {
                            Netlib.dqrsl_(data, &n, &n, &p, data2, data3,
                                   (float*)0,         // A: Qb
                                   data4,              // B: Q'b
                                   (float*)0,         // C: x
                                   (float*)0,         // D: residual
                                   (float*)0,         // E: Ax
                                   &JOB,
                                   &info);
                        }
                    }
                }
            }
     
            if (info > 0)
                Debug.WriteLine(" __FILE__ : VNL::QR<T>::QtB() -- matrix is rank-def by " + Convert.ToString(info));
  
            return QtB;
        }


    }


// Compute determinant of matrix "M" using QR.
/*
public float QRDeterminant(MatrixFixed m)
{
  return (QR(m).Determinant());
}
*/


}
