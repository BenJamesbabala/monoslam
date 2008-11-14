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
    ///  Holds the singular value decomposition of a VNL::Matrix.
    ///
    ///  The class holds three matrices U, W, V such that the original matrix
    ///  \f$M = U W V^\top\f$.  The DiagMatrix W stores the singular values in decreasing
    ///  order.  The columns of U which correspond to the nonzero singular values
    ///  form a basis for range of M, while the columns of V corresponding to the
    ///  zero singular values are the nullspace.
    ///
    ///  The SVD is computed at construction time, and enquiries may then be made
    ///  of the SVD.  In particular, this allows easy access to multiple
    ///  right-hand-side solves without the bother of putting all the RHS's into a
    ///  Matrix.
    ///
    ///  This class is supplied even though there is an existing VNL::Matrix method
    ///  for several reasons:
    ///
    ///  It is more convenient to use as it manages all the storage for
    ///  the U,S,V matrices, allowing repeated queries of the same SVD
    ///  results.
    ///
    ///  It avoids namespace clutter in the Matrix class.   While svd()
    ///  is a perfectly reasonable method for a Matrix, there are many other
    ///  decompositions that might be of interest, and adding them all would
    ///  make for a very large Matrix class.
    ///
    ///  It demonstrates the holder model of compute class, implementing an
    ///  algorithm on an object without adding a member that may not be of
    ///  general interest.  A similar pattern can be used for other
    ///  decompositions which are not defined as members of the library Matrix
    ///  class.
    ///
    ///  It extends readily to n-ary operations, such as generalized
    ///  eigensystems, which cannot be members of just one matrix.
    /// </summary>
    public class SVD
    {

        /// Construct an vnl_svd<T> object from \f$m \times n\f$ matrix \f$M\f$.  The
        /// vnl_svd<T> object contains matrices \f$U\f$, \f$W\f$, \f$V\f$ such that
        /// \f$U W V^\top = M\f$.
        ///
        /// Uses linpack routine DSVDC to calculate an ``economy-size'' SVD
        /// where the returned \f$U\f$ is the same size as \f$M\f$, while \f$W\f$
        /// and \f$V\f$ are both \f$n \times n\f$.  This is efficient for
        /// large rectangular solves where \f$m > n\f$, typical in least squares.
        ///
        /// The optional argument zero_out_tol is used to mark the zero singular
        /// values: If nonnegative, any s.v. smaller than zero_out_tol in
        /// absolute value is set to zero.  If zero_out_tol is negative, the
        /// zeroing is relative to |zero_out_tol| * sigma_max();


        // Data Access---------------------------------------------------------------

        const bool test_heavily = false;

        // find weights below tol*max(w) and zero them out.
        public int    Singularities () { return W_.Rows - Rank(); }
        public int    Rank ()          { return rank_; }
        public float WellCondition () { return SigmaMin()/SigmaMax(); }

        // Return the matrix U.  
        public MatrixFixed   U() { return U_; }

        // Return the matrix U's (i,j)th entry (to avoid svd.U()(i,j); ).
        public float U(int i, int j) { return U_[i,j]; }

        // Get at DiagMatrix (q.v.) of singular values, sorted from largest to smallest.
        public DiagMatrix    W()      { return W_; }

        // Get at DiagMatrix (q.v.) of singular values, sorted from largest to smallest.
        public DiagMatrix WInverse() { return Winverse_; }
        public DiagMatrix Winverse() { return Winverse_; }
        public float W(int i, int j) { return W_[i,j]; }
        public float W(int i)        { return W_[i,i]; }
        public float SigmaMax() { return W_[0,0]; }       // largest
        public float SigmaMin() { return W_[n_-1,n_-1]; } // smallest

        // Return the matrix V.
        public MatrixFixed V() { return V_; }

        // Return the matrix V's (i,j)th entry (to avoid svd.V()(i,j); ).
        public float V(int i, int j) { return V_[i,j]; }
  
        public bool Valid() { return valid_; }

        private int m_, n_;  // Size of M, local cache.
        MatrixFixed U_;      // Columns Ui are basis for range of M for Wi != 0
        DiagMatrix W_;       // Singular values, sorted in decreasing order
        DiagMatrix Winverse_;
        MatrixFixed V_;      // Columns Vi are basis for nullspace of M for Wi = 0
        int rank_;
        //bool have_max_;
        //float max_;
        //bool have_min_;
        //bool min_;
        float last_tol_;
        bool valid_;         // false if the NETLIB call failed.


        public SVD(MatrixFixed M)
        {
            init(M, 0.0f);
        }

        public SVD(MatrixFixed M, float zero_out_tol)
        {
            init(M, zero_out_tol);
        }

        private unsafe void init(MatrixFixed M, float zero_out_tol)
        {
            m_ = M.Rows;
            n_ = M.Columns;
            U_ = new MatrixFixed(m_, n_);
            W_ = new DiagMatrix(n_);
            Winverse_ = new DiagMatrix(n_);
            V_ = new MatrixFixed(n_, n_);

            //assert(m_ > 0);  
            //assert(n_ > 0);
		
            int n = M.Rows;    
            int p = M.Columns;
            int mm = Netlib.min(n+1,p);

            // Copy source matrix into fortran storage
            // SVD is slow, don't worry about the cost of this transpose.
            Vector X = Vector.fortran_copy(M);

            // Make workspace vectors
            Vector work = new Vector(n);
            work.Fill(0);
            Vector uspace = new Vector(n*p);
            uspace.Fill(0);
            Vector vspace = new Vector(p*p);
            vspace.Fill(0);
            Vector wspace = new Vector(mm);
            wspace.Fill(0); // complex fortran routine actually _wants_ complex W!
            Vector espace = new Vector(p);
            espace.Fill(0);
    
            // Call Linpack SVD
            int info = 0;
            int job = 21;

            fixed (float* data = X.Datablock())
            {
                fixed (float* data2 = wspace.Datablock())
                {
                    fixed (float* data3 = espace.Datablock())
                    {
                        fixed (float* data4 = uspace.Datablock())
                        {
                            fixed (float* data5 = vspace.Datablock())
                            {
                                fixed (float* data6 = work.Datablock())
                                {
                                    Netlib.dsvdc_(data, &n, &n, &p,
                                             data2,
                                             data3,
                                             data4, &n,
                                             data5, &p,
                                             data6,
                                             &job, &info);
                                }
                            }
                        }
                    }
                }
            }

            // Error return?
            if (info != 0) 
            {
                // If info is non-zero, it contains the number of singular values
                // for this the SVD algorithm failed to converge. The condition is
                // not bogus. Even if the returned singular values are sensible,
                // the singular vectors can be utterly wrong.

                // It is possible the failure was due to NaNs or infinities in the
                // matrix. Check for that now.
                M.assert_finite();

                // If we get here it might be because
                // 1. The scalar type has such
                // extreme precision that too few iterations were performed to
                // converge to within machine precision (that is the svdc criterion).
                // One solution to that is to increase the maximum number of
                // iterations in the netlib code.
                //
                // 2. The LINPACK dsvdc_ code expects correct IEEE rounding behaviour,
                // which some platforms (notably x86 processors)
                // have trouble doing. For example, gcc can output
                // code in -O2 and static-linked code that causes this problem.
                // One solution to this is to persuade gcc to output slightly different code
                // by adding and -fPIC option to the command line for v3p\netlib\dsvdc.c. If
                // that doesn't work try adding -ffloat-store, which should fix the problem
                // at the expense of being significantly slower for big problems. Note that
                // if this is the cause, vxl/vnl/tests/test_svd should have failed.
                //
                // You may be able to diagnose the problem here by printing a warning message.
                Debug.WriteLine("__FILE__ : suspicious return value (" + Convert.ToString(info) + ") from SVDC" +
                                "__FILE__ : M is " + Convert.ToString(M.Rows) + "x" + Convert.ToString(M.Columns));

                valid_ = false;
            }
            else
                valid_ = true;

            // Copy fortran outputs into our storage     
            int ctr = 0;
            for (int j = 0; j < p; ++j)
                for (int i = 0; i < n; ++i)
                {
                    U_[i, j] = uspace[ctr];
                    ctr++;
                }
    

            for (int j = 0; j < mm; ++j)
                W_[j, j] = Math.Abs(wspace[j]); // we get rid of complexness here.

            for (int j = mm; j < n_; ++j)
                W_[j, j] = 0;

            ctr = 0;
            for (int j = 0; j < p; ++j)
                for (int i = 0; i < p; ++i)
                {
                    V_[i, j] = vspace[ctr];
                    ctr++;
                }
    
        

            if (test_heavily) 
            {
                // Test that recomposed matrix == M
                //float recomposition_residual = Math.Abs((Recompose() - M).FrobeniusNorm());
                //float n2 = Math.Abs(M.FrobeniusNorm());
                //float thresh = m_ * (float)(eps) * n2;
                //if (recomposition_residual > thresh) 
                {
                    //std::cerr << "VNL::SVD<T>::SVD<T>() -- Warning, recomposition_residual = "
                    //<< recomposition_residual << std::endl
                    //<< "FrobeniusNorm(M) = " << n << std::endl
                    //<< "eps*FrobeniusNorm(M) = " << thresh << std::endl
                    //<< "Press return to continue\n";
                    //char x;
                    //std::cin.get(&x, 1, '\n');
                }
            }

            if (zero_out_tol >= 0)
                // Zero out small sv's and update rank count.
                ZeroOutAbsolute((float)(+zero_out_tol));
            else
                // negative tolerance implies relative to max elt.
                ZeroOutRelative((float)(-zero_out_tol));
        }


        //-----------------------------------------------------------------------------
        // Chunky bits.

        /// <summary>
        /// find weights below threshold tol, zero them out, and update W_ and Winverse_.
        /// </summary>
        /// <param name="tol"></param>
        public void ZeroOutAbsolute(float tol)
        {
            last_tol_ = tol;
            rank_ = W_.Rows;
            for (int k = 0; k < W_.Rows; k++) 
            {
                float weight = W_[k, k];
                if (Math.Abs(weight) <= tol) 
                {
                    Winverse_[k,k] = 0;
                    weight = 0;
                    --rank_;
                } 
                else 
                {
                    Winverse_[k,k] = (float)1.0/weight;
                }
            }
        }

        /// <summary>
        /// find weights below tol*max(w) and zero them out.
        /// </summary>
        /// <param name="tol"></param>
        public void ZeroOutRelative(float tol) // sqrt(machine epsilon)
        {
            ZeroOutAbsolute(tol * Math.Abs(SigmaMax()));
        }


        /// <summary>
        /// Calculate determinant as product of diagonals in W.
        /// </summary>
        /// <returns></returns>
        public float DeterminantMagnitude()
        {
            
            bool warned = false;
            if ((!warned) && (m_ != n_)) 
            {
                Debug.WriteLine("__FILE__ : called determinant_magnitude() on SVD of non-square matrix");
                warned = true;
            }
  
            float product = W_[0, 0];
            for (int k = 1; k < W_.Columns; k++)
                product *= W_[k, k];

            return product;
        }


        public float Norm()
        {
            return Math.Abs(SigmaMax());
        }

        /// <summary>
        /// Recompose SVD to U*W*V'.
        /// </summary>
        /// <returns></returns>
        public MatrixFixed Recompose()
        {
            MatrixFixed W = new MatrixFixed(W_.Rows,W_.Columns);
            W.Fill(0);
            for (int i=0;i<rank_;i++)
                W[i,i]=W_[i,i];

            return (U_*W*V_.ConjugateTranspose());
        }



        public MatrixFixed Inverse()
        {
            return PseudoInverse();
        }


        /// <summary>
        /// Calculate pseudo-inverse.
        /// </summary>
        /// <returns></returns>
        public MatrixFixed PseudoInverse()
        {
            MatrixFixed Winverse = new MatrixFixed(Winverse_.Rows,Winverse_.Columns);
            Winverse.Fill(0);
            for (int i=0;i<rank_;i++)
                Winverse[i,i]=Winverse_[i,i];

            return (V_ * Winverse * U_.ConjugateTranspose());
        }

        /// <summary>
        /// Calculate pseudo-inverse.
        /// </summary>
        /// <param name="rank"></param>
        /// <returns></returns>
        public MatrixFixed PseudoInverse(int rank)
        {
            MatrixFixed Winverse = new MatrixFixed(Winverse_.Rows,Winverse_.Columns);
            Winverse.Fill(0);
            for (int i=0;i<rank;i++)
                Winverse[i,i]=Winverse_[i,i];

            return (V_ * Winverse * U_.ConjugateTranspose());
        }


        /// <summary>
        /// Calculate inverse of transpose.
        /// </summary>
        /// <returns></returns>
        public MatrixFixed TransposeInverse()
        {
            MatrixFixed Winverse = new MatrixFixed(Winverse_.Rows,Winverse_.Columns);
            Winverse.Fill(0);
            for (int i=0;i<rank_;i++)
                Winverse[i,i]=Winverse_[i,i];

            return (U_ * Winverse * V_.ConjugateTranspose());
        }


        /// <summary>
        /// Solve the matrix equation M X = B, returning X.
        /// </summary>
        /// <param name="B"></param>
        /// <returns></returns>
        public MatrixFixed Solve(MatrixFixed B)
        {
            MatrixFixed x;   // solution matrix
            if (U_.Rows < U_.Columns) 
            {
                // augment y with extra rows of
                MatrixFixed yy = new MatrixFixed(U_.Rows, B.Columns);     // zeros, so that it matches
                yy.Fill(0);
                yy.Update(B);  // cols of u.transpose. ???
                x = U_.ConjugateTranspose() * yy;
            } 
            else
                x = U_.ConjugateTranspose() * B;
            
            int i, j;
            for (i = 0; i < x.Rows; i++) 
            {                      // multiply with diagonal 1/W
                float weight = W_[i, i];
                if (weight != 0) //vnl_numeric_traits<T>::zero)
                    weight = 1.0f / weight;
                for (j = 0; j < x.Columns; j++)
                    x[i, j] *= weight;
            }
            x = V_ * x;  // premultiply with v.
            return x;
        }

        /// <summary>
        /// Solve the matrix-vector system M x = y, returning x.
        /// </summary>
        /// <param name="y"></param>
        /// <returns></returns>
        public Vector Solve(Vector y)
        {
            // fsm sanity check :
            if (y.size() != U_.Rows) 
            {
                
                Debug.WriteLine("__FILE__ : size of rhs is incompatible with no. of rows in U_ " +
                                "y =" + Convert.ToString(y) + "\n" +
                                "m_=" + Convert.ToString(m_) + "\n" +
                                "n_=" + Convert.ToString(n_) + "\n" +
                                "U_=\n" + Convert.ToString(U_) + "V_=\n" + Convert.ToString(V_) +
                                "W_=\n" + W_);
                
            }

  
            Vector x = new Vector(V_.Rows);   // Solution matrix.
            if (U_.Rows < U_.Columns) 
            {  // Augment y with extra rows of
                Vector yy = new Vector(U_.Rows);  // zeros, so that it matches
                yy.Fill(0);
                if (yy.size() < y.size()) 
                { // fsm
                    Debug.WriteLine("yy=" + Convert.ToString(yy));
                    Debug.WriteLine("y =" + Convert.ToString(y));
                    // the update() call on the next line will abort...
                }
    
                yy.Update(y);     // cols of u.transpose.    
                x = U_.ConjugateTranspose() * yy;
            }
            else
                x = U_.ConjugateTranspose() * y;

            for (int i = 0; i < x.size(); i++) 
            {  // multiply with diagonal 1/W
                float weight = W_[i, i], zero_ = 0.0f;
                if (weight != zero_)
                    x[i] /= weight;
                else
                    x[i] = zero_;
            }
  
            return V_ * x;  // premultiply with v.
        }

        // FIXME. this should implement the above, not the other way round.
        /*
        public void Solve(float[] y, float[] x)
        {
            Solve(new Vector(y, m_)).CopyOut(x);
        }
        */


        /// <summary>
        /// Solve the matrix-vector system M x = y.
        /// Assume that the singular values W have been preinverted by the caller.
        /// </summary>
        /// <param name="y"></param>
        /// <param name="x_out"></param>
        public void SolvePreinverted(Vector y, Vector x_out)
        {
            Vector x;  // solution matrix
            if (U_.Rows < U_.Columns) 
            {   // augment y with extra rows of
                //std::cout << "svd<T>::solve_preinverted() -- Augmenting y\n";
                Vector yy = new Vector(U_.Rows);  // zeros, so that it match
                yy.Fill(0);
                yy.Update(y);                     // cols of u.transpose. ??
                x = U_.ConjugateTranspose() * yy;
            } 
            else
                x = U_.ConjugateTranspose() * y;
            
            for (int i = 0; i < x.size(); i++)  // multiply with diagonal W, assumed inverted
                x[i] *= W_[i, i];

            x_out = V_ * x;    // premultiply with v.
        }

        /// <summary>
        /// Return N s.t.\ M * N = 0.
        /// </summary>
        /// <returns></returns>
        public MatrixFixed Nullspace()
        {
            int k = Rank();
            if (k == n_) Debug.WriteLine("svd<T>::nullspace() -- Matrix is full rank." + Convert.ToString(last_tol_));
            return Nullspace(n_-k);
        }

        /// <summary>
        /// Return N s.t.\ M * N = 0.
        /// </summary>
        /// <param name="required_nullspace_dimension"></param>
        /// <returns></returns>
        public MatrixFixed Nullspace(int required_nullspace_dimension)
        {
            return V_.Extract(V_.Rows, required_nullspace_dimension, 0, n_ - required_nullspace_dimension);
        }

        /// <summary>
        /// Return N s.t.\ M' * N = 0.
        /// </summary>
        /// <returns></returns>
        public MatrixFixed LeftNullspace()
        {
            int k = Rank();
            if (k == n_)
                Debug.WriteLine("VNL::SVD<T>::LeftNullspace() -- Matrix is full rank." + Convert.ToString(last_tol_));
            return U_.Extract(U_.Rows, n_-k, 0, k);
        }

        /// <summary>
        /// Implementation to be done yet; currently returns left_nullspace().\ - PVR.
        /// </summary>
        /// <param name="?"></param>
        /// <returns></returns>
        public MatrixFixed LeftNullspace(int d) //required_nullspace_dimension*/
        {
            return LeftNullspace();
        }


        /// <summary>
        /// Return the rightmost column of V.
        /// Does not check to see whether or not the matrix actually was rank-deficient -
        /// the caller is assumed to have examined W and decided that to his or her satisfaction.
        /// </summary>
        /// <returns></returns>
        public Vector Nullvector()
        {
            Vector ret = new Vector(n_);
            for (int i = 0; i < n_; ++i)
                ret[i] = V_[i, n_-1];
            return ret;
        }

        /// <summary>
        /// Return the rightmost column of U.
        /// Does not check to see whether or not the matrix actually was rank-deficient.
        /// </summary>
        /// <returns></returns>
        public Vector LeftNullvector()
        {
            Vector ret = new Vector(m_);
            int col = Netlib.min(m_, n_) - 1;
            for (int i = 0; i < m_; ++i)
                ret[i] = U_[i, col];
            return ret;
        }


    }

/*
    public MatrixFixed SVDInverse(MatrixFixed m)
    {
        return (new SVD(m)).Inverse();
    }
*/
}
