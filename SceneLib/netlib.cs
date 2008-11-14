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
    public class Netlib
    {
        /// <summary>
        /// Some lousy unix function
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        private static unsafe float d_sign(float *a, float *b)
        {
            float x;
            x = (*a >= 0 ? *a : - *a);
            return( *b >= 0 ? x : -x);
        }

        /// <summary>
        /// return the maximum of the two given values
        /// </summary>
        /// <param name="v1"></param>
        /// <param name="v2"></param>
        /// <returns></returns>
        public static float max(float v1, float v2)
        {
            if (v1 > v2)
                return (v1);
            else
                return (v2);
        }
        
        /// <summary>
        /// return the maximum of the two given values
        /// </summary>
        /// <param name="v1"></param>
        /// <param name="v2"></param>
        /// <returns></returns>
        public static int max(int v1, int v2)
        {
            if (v1 > v2)
                return (v1);
            else
                return (v2);
        }

        /// <summary>
        /// return the minimum of the two given values
        /// </summary>
        /// <param name="v1"></param>
        /// <param name="v2"></param>
        /// <returns></returns>
        public static float min(float v1, float v2)
        {
            if (v1 < v2)
                return (v1);
            else
                return (v2);
        }

        /// <summary>
        /// return the minimum of the two given values
        /// </summary>
        /// <param name="v1"></param>
        /// <param name="v2"></param>
        /// <returns></returns>
        public static int min(int v1, int v2)
        {
            if (v1 < v2)
                return (v1);
            else
                return (v2);
        }


        /// <summary>
        /// applies a plane rotation. 
        /// jack dongarra, linpack, 3/11/78. 
        /// modified 12/3/93, array(1) declarations changed to array(*) 
        /// </summary>
        /// <param name="n"></param>
        /// <param name="dx"></param>
        /// <param name="incx"></param>
        /// <param name="dy"></param>
        /// <param name="incy"></param>
        /// <param name="c"></param>
        /// <param name="s"></param>
        private static unsafe void drot_(int* n, float* dx, int* incx, float* dy, int* incy, float* c, float* s)
        {
            // Local variables 
            int i;
            float dtemp;
            int ix, iy;

            if (*n <= 0)
            {
                return;
            }
            if ((*incx == 1) && (*incy == 1))
            {
                for (i = 0; i < *n; ++i)
                {
                    dtemp = *c * dx[i] + *s * dy[i];
                    dy[i] = *c * dy[i] - *s * dx[i];
                    dx[i] = dtemp;
                }
            }
            else
            {
                ix = 0; iy = 0;
                if (*incx < 0)
                {
                    ix = (1 - (*n)) * *incx;
                }
                if (*incy < 0)
                {
                    iy = (1 - (*n)) * *incy;
                }
                for (i = 0; i < *n; ++i)
                {
                    dtemp = *c * dx[ix] + *s * dy[iy];
                    dy[iy] = *c * dy[iy] - *s * dx[ix];
                    dx[ix] = dtemp;
                    ix += *incx; iy += *incy;
                }
            }
        }



        /// <summary>
        /// construct givens plane rotation.
        /// jack dongarra, linpack, 3/11/78.
        /// </summary>
        /// <param name="da"></param>
        /// <param name="db"></param>
        /// <param name="c"></param>
        /// <param name="s"></param>
        private static unsafe void drotg_(float* da, float* db, float* c, float* s)
        {
            // Table of constant values 
            float c_b4 = 1.0f;

            // Local variables 
            float r, scale, z, roe;

            scale = Math.Abs(*da) + Math.Abs(*db);
            if (scale == 0.0)
            {
                *c = 1.0f; *s = 0.0f;
                *da = *db = 0.0f;
            }
            else
            {
                roe = *db;
                if (Math.Abs(*da) > Math.Abs(*db))
                {
                    roe = *da;
                }
                r = *da / scale;
                z = *db / scale;
                r = scale * (float)Math.Sqrt(r * r + z * z);
                r *= d_sign(&c_b4, &roe);
                *c = *da / r;
                *s = *db / r;
                z = 1.0f;
                if (Math.Abs(*da) > Math.Abs(*db))
                {
                    z = *s;
                }
                if ((Math.Abs(*db) >= Math.Abs(*da)) && (*c != 0.0))
                {
                    z = 1.0f / *c;
                }
                *da = r;
                *db = z;
            }
        }

        /// <summary>
        /// copies a vector, x, to a vector, y. 
        /// uses unrolled loops for increments equal to 1. 
        /// jack dongarra, linpack, 3/11/78.
        /// modified 12/3/93, array(1) declarations changed to array(*) 
        /// </summary>
        /// <param name="n"></param>
        /// <param name="dx"></param>
        /// <param name="incx"></param>
        /// <param name="dy"></param>
        /// <param name="incy"></param>
        private static unsafe void dcopy_(int* n, float* dx, int* incx, float* dy, int* incy)
        {
            // Local variables
            int i, ix, iy;

            if (*n <= 0)
            {
                return;
            }
            if ((*incx == 1) && (*incy == 1))
            {
                for (i = 0; i < *n; ++i)
                {
                    dy[i] = dx[i];
                }
            }
            else
            {
                ix = 0; iy = 0;
                if (*incx < 0)
                {
                    ix = (1 - (*n)) * *incx;
                }
                if (*incy < 0)
                {
                    iy = (1 - (*n)) * *incy;
                }
                for (i = 0; i < *n; ++i)
                {
                    dy[iy] = dx[ix];
                    ix += *incx; iy += *incy;
                }
            }
            return;
        }


        /// <summary>
        ///  DNRM2 returns the euclidean norm of a vector via the function       
        ///  name, so that                                                       
        ///                                                                      
        ///    DNRM2 := sqrt( x'*x )                                            
        ///                                                                      
        ///  -- This version written on 25-October-1982.                         
        ///     Modified on 14-October-1993 to inline the call to DLASSQ.        
        ///     Sven Hammarling, Nag Ltd.                                        
        /// </summary>
        /// <param name="n"></param>
        /// <param name="x"></param>
        /// <param name="incx"></param>
        /// <returns></returns>
        private static unsafe float dnrm2_(int* n, float* x, int* incx) 
        {
            // System generated locals 
            float d__1;

            // Local variables 
            float norm, scale, absxi;
            int ix;
            float ssq;

            if ((*n < 1) || (*incx < 1)) 
            {
                norm = 0.0f;
            } 
            else 
                if (*n == 1) 
                {
                    norm = Math.Abs(x[0]);
                } 
                else 
                {
                    scale = 0.0f;
                    ssq = 1.0f;
                    // The following loop is equivalent to this call to the LAPACK 
                    // auxiliary routine: 
                    // CALL DLASSQ( N, X, INCX, SCALE, SSQ ) 

                    for (ix = 0; ix < *n * *incx; ix += *incx) 
                    {
                        if (x[ix] != 0.0) 
                        {
                            absxi = Math.Abs(x[ix]);
                            if (scale < absxi) 
                            {
                                d__1 = scale / absxi;
                                ssq = ssq * d__1 * d__1 + 1.0f;
                                scale = absxi;
                            } 
                            else 
                            {
                                d__1 = absxi / scale;
                                ssq += d__1 * d__1;
                            }
                        }
                    }
                    norm = scale * (float)Math.Sqrt(ssq);
                }

            return norm;
        }


        /// <summary>
        /// interchanges two vectors.                                    
        /// uses unrolled loops for increments equal to 1.               
        /// jack dongarra, linpack, 3/11/78.                             
        /// modified 12/3/93, array(1) declarations changed to array(*)  
        /// </summary>
        /// <param name="n"></param>
        /// <param name="dx"></param>
        /// <param name="incx"></param>
        /// <param name="dy"></param>
        /// <param name="incy"></param>
        private static unsafe void dswap_(int* n, float* dx, int* incx, float* dy, int* incy)
        {
            // Local variables 
            int i, m;
            float dtemp;
            int ix, iy;

            if (*n <= 0)
            {
                return;
            }
            if (*incx == 1 && *incy == 1)
            {
                m = *n % 3;
                for (i = 0; i < m; ++i)
                {
                    dtemp = dx[i];
                    dx[i] = dy[i];
                    dy[i] = dtemp;
                }
                for (i = m; i < *n; i += 3)
                {
                    dtemp = dx[i];
                    dx[i] = dy[i];
                    dy[i] = dtemp;
                    dtemp = dx[i + 1];
                    dx[i + 1] = dy[i + 1];
                    dy[i + 1] = dtemp;
                    dtemp = dx[i + 2];
                    dx[i + 2] = dy[i + 2];
                    dy[i + 2] = dtemp;
                }
            }
            else
            {
                ix = 0; iy = 0;
                if (*incx < 0)
                {
                    ix = (1 - (*n)) * *incx;
                }
                if (*incy < 0)
                {
                    iy = (1 - (*n)) * *incy;
                }
                for (i = 0; i < *n; ++i)
                {
                    dtemp = dx[ix];
                    dx[ix] = dy[iy];
                    dy[iy] = dtemp;
                    ix += *incx; iy += *incy;
                }
            }
        }


        /// <summary>
        /// constant times a vector plus a vector. 
        /// uses unrolled loops for increments equal to one. 
        /// jack dongarra, linpack, 3/11/78. 
        /// modified 12/3/93, array(1) declarations changed to array(*) 
        /// </summary>
        /// <param name="n"></param>
        /// <param name="da"></param>
        /// <param name="dx"></param>
        /// <param name="incx"></param>
        /// <param name="dy"></param>
        /// <param name="incy"></param>
        private static unsafe void daxpy_(int* n, float* da, float* dx, int* incx, float* dy, int* incy)
        {
            // Local variables 
            int i, ix, iy;

            if (*n <= 0)
            {
                return;
            }
            if (*da == 0.0)
            {
                return;
            }
            if ((*incx == 1) && (*incy == 1))
            {
                for (i = 0; i < *n; ++i)
                {
                    dy[i] += *da * dx[i];
                }
            }
            else
            {
                ix = 0;
                iy = 0;
                if (*incx < 0)
                {
                    ix = (1 - (*n)) * *incx;
                }
                if (*incy < 0)
                {
                    iy = (1 - (*n)) * *incy;
                }
                for (i = 0; i < *n; ++i)
                {
                    dy[iy] += *da * dx[ix];
                    ix += *incx; iy += *incy;
                }
            }
            return;
        }

        /// <summary>
        /// scales a vector by a constant.                                   
        /// uses unrolled loops for increment equal to 1.                    
        /// jack dongarra, linpack, 3/11/78.                                 
        /// modified 3/93 to return if incx .le. 0.                          
        /// modified 12/3/93, array(1) declarations changed to array(*)      
        /// </summary>
        /// <param name="n"></param>
        /// <param name="da"></param>
        /// <param name="dx"></param>
        /// <param name="incx"></param>
        private static unsafe void dscal_(int* n, float* da, float* dx, int* incx)
        {
            // Local variables 
            int i, m, nincx;

            if ((*n <= 0) || (*incx <= 0))
            {
                return;
            }
            // code for increment equal to 1 
            if (*incx == 1)
            {
                m = *n % 5;
                for (i = 0; i < m; ++i)
                {
                    dx[i] *= *da;
                }
                for (i = m; i < *n; i += 5)
                {
                    dx[i] *= *da; dx[i + 1] *= *da; dx[i + 2] *= *da; dx[i + 3] *= *da; dx[i + 4] *= *da;
                }
            }
            // code for increment not equal to 1 
            else
            {
                nincx = *n * *incx;
                for (i = 0; i < nincx; i += *incx)
                {
                    dx[i] *= *da;
                }
            }
        }



        /// <summary>
        /// forms the dot product of two vectors.
        /// uses unrolled loops for increments equal to one.
        /// jack dongarra, linpack, 3/11/78.
        /// modified 12/3/93, array(1) declarations changed to array(*)
        /// </summary>
        /// <param name="n"></param>
        /// <param name="dx"></param>
        /// <param name="incx"></param>
        /// <param name="dy"></param>
        /// <param name="incy"></param>
        /// <returns></returns>
        public static unsafe float ddot_(int* n, float* dx, int* incx, float* dy, int* incy)
        {
            // Local variables 
            int i, m;
            float dtemp;
            int ix, iy;

            dtemp = 0.0f;
            if (*n <= 0)
            {
                return dtemp;
            }
            if ((*incx == 1) && (*incy == 1))
            {
                m = *n % 5;
                for (i = 0; i < m; ++i)
                {
                    dtemp += dx[i] * dy[i];
                }
                for (i = m; i < *n; i += 5)
                {
                    dtemp += dx[i] * dy[i] +
                             dx[i + 1] * dy[i + 1] +
                             dx[i + 2] * dy[i + 2] +
                             dx[i + 3] * dy[i + 3] +
                             dx[i + 4] * dy[i + 4];
                }
            }
            else
            {
                ix = 0; iy = 0;
                if (*incx < 0)
                {
                    ix = (1 - (*n)) * *incx;
                }
                if (*incy < 0)
                {
                    iy = (1 - (*n)) * *incy;
                }
                for (i = 0; i < *n; ++i)
                {
                    dtemp += dx[ix] * dy[iy];
                    ix += *incx; iy += *incy;
                }
            }
            return dtemp;
        }


        /// <summary>
        ///     dpofa factors a float precision symmetric positive definite     
        ///     matrix.                                                          
        ///                                                                      
        ///     dpofa is usually called by dpoco, but it can be called           
        ///     directly with a saving in time if  rcond  is not needed.         
        ///     (time for dpoco) = (1 + 18/n)*(time for dpofa) .                 
        ///                                                                      
        ///     on entry                                                         
        ///                                                                      
        ///        a       float precision(lda, n)                              
        ///                the symmetric matrix to be factored.  only the        
        ///                diagonal and upper triangle are used.                 
        ///                                                                      
        ///        lda     integer                                               
        ///                the leading dimension of the array  a .               
        ///                                                                      
        ///        n       integer                                               
        ///                the order of the matrix  a .                          
        ///                                                                      
        ///     on return                                                        
        ///                                                                      
        ///        a       an upper triangular matrix  r  so that a = trans(r)*r 
        ///                where  trans(r)  is the transpose.                    
        ///                the strict lower triangle is unaltered.               
        ///                if  info .ne. 0 , the factorization is not complete.  
        ///                                                                      
        ///        info    integer                                               
        ///                = 0  for normal return.                               
        ///                = k  signals an error condition.  the leading minor   
        ///                     of order  k  is not positive definite.           
        ///                                                                      
        ///     linpack.  this version dated 08/14/78 .                          
        ///     cleve moler, university of new mexico, argonne national lab.     
        /// </summary>
        /// <param name="a"></param>
        /// <param name="lda"></param>
        /// <param name="n"></param>
        /// <param name="info"></param>
        public static unsafe void dpofa_(float* a, int* lda, int* n, int* info)
        {
            // Local variables 
            int j, k;
            float s, t;
            int c__1 = 1;

            for (j = 0; j < *n; ++j)
            {
                *info = j + 1;
                s = 0.0f;
                for (k = 0; k < j; ++k)
                {
                    t = a[k + j * *lda] - ddot_(&k, &a[k * *lda], &c__1, &a[j * *lda], &c__1);
                    t /= a[k + k * *lda];
                    a[k + j * *lda] = t;
                    s += t * t;
                }
                s = a[j + j * *lda] - s;
                if (s <= 0.0)
                {
                    return;
                }
                a[j + j * *lda] = (float)Math.Sqrt(s);
            }
            *info = 0;
        }


        /// <summary>
        ///     dposl solves the float precision symmetric positive definite    
        ///     system a * x = b                                                 
        ///     using the factors computed by dpoco or dpofa.                    
        ///                                                                      
        ///     on entry                                                         
        ///                                                                      
        ///        a       float precision(lda, n)                              
        ///                the output from dpoco or dpofa.                       
        ///                                                                      
        ///        lda     integer                                               
        ///                the leading dimension of the array  a .               
        ///                                                                      
        ///        n       integer                                               
        ///                the order of the matrix  a .                          
        ///                                                                      
        ///        b       float precision(n)                                   
        ///                the right hand side vector.                           
        ///                                                                      
        ///     on return                                                        
        ///                                                                      
        ///        b       the solution vector  x .                              
        ///                                                                      
        ///     error condition                                                  
        ///                                                                      
        ///        a division by zero will occur if the input factor contains    
        ///        a zero on the diagonal.  technically this indicates           
        ///        singularity but it is usually caused by improper subroutine   
        ///        arguments.  it will not occur if the subroutines are called   
        ///        correctly and  info .eq. 0 .                                  
        ///                                                                      
        ///     to compute  inverse(a) * c  where  c  is a matrix                
        ///     with  p  columns                                                 
        ///           call dpoco(a,lda,n,rcond,z,info)                           
        ///           if (rcond is too small .or. info .ne. 0) go to ...         
        ///           do 10 j = 1, p                                             
        ///              call dposl(a,lda,n,c(1,j))                              
        ///        10 continue                                                   
        ///                                                                      
        ///     linpack.  this version dated 08/14/78 .                          
        ///     cleve moler, university of new mexico, argonne national lab.     

        ///     subroutines and functions 
        ///     blas daxpy,ddot 
        /// </summary>
        /// <param name="a"></param>
        /// <param name="lda"></param>
        /// <param name="n"></param>
        /// <param name="b"></param>
        public static unsafe void dposl_(float *a, int *lda, int *n, float *b)
        {
            // Local variables 
            int k;
            float t;
            int c__1 = 1;

            //     solve trans(r)*y = b 

            for (k = 0; k < *n; ++k) 
            {
                t = ddot_(&k, &a[k * *lda], &c__1, b, &c__1);
                b[k] = (b[k] - t) / a[k + k * *lda];
            }

            //     solve r*x = y 

            for (k = *n - 1; k >= 0; --k) 
            {
                b[k] /= a[k + k * *lda];
                t = -b[k];
                daxpy_(&k, &t, &a[k * *lda], &c__1, b, &c__1);
            }
        }


        /// <summary>
        ///     dpodi computes the determinant and inverse of a certain 
        ///     float precision symmetric positive definite matrix (see below) 
        ///     using the factors computed by dpoco, dpofa or dqrdc. 
        ///     
        ///     on entry 
        ///     
        ///        a       float precision(lda, n) 
        ///                the output  a  from dpoco or dpofa 
        ///                or the output  x  from dqrdc. 
        ///                
        ///        lda     integer 
        ///                the leading dimension of the array  a . 
        ///                
        ///        n       integer 
        ///                the order of the matrix  a . 
        ///                
        ///        job     integer 
        ///                = 11   both determinant and inverse. 
        ///                = 01   inverse only. 
        ///                = 10   determinant only. 
        ///                
        ///     on return 
        ///     
        ///        a       if dpoco or dpofa was used to factor  a  then 
        ///                dpodi produces the upper half of inverse(a) . 
        ///                if dqrdc was used to decompose  x  then 
        ///                dpodi produces the upper half of inverse(trans(x)*x) 
        ///                where trans(x) is the transpose. 
        ///                elements of  a  below the diagonal are unchanged. 
        ///                if the units digit of job is zero,  a  is unchanged. 
        ///                
        ///        det     float precision(2) 
        ///                determinant of  a  or of  trans(x)*x  if requested. 
        ///                otherwise not referenced. 
        ///                determinant = det(1) * 10.0**det(2) 
        ///                with  1.0f .le. det(1) .lt. 10.0f 
        ///                or  det(1) .eq. 0.0f . 
        ///                
        ///     error condition 
        ///     
        ///        a division by zero will occur if the input factor contains 
        ///        a zero on the diagonal and the inverse is requested. 
        ///        it will not occur if the subroutines are called correctly 
        ///        and if dpoco or dpofa has set info .eq. 0 . 
        ///        
        ///     linpack.  this version dated 08/14/78 . 
        ///     cleve moler, university of new mexico, argonne national lab. 
        ///     
        ///     subroutines and functions 
        ///     
        ///     blas daxpy,dscal 
        ///     fortran mod */
        /// </summary>
        /// <param name="a"></param>
        /// <param name="lda"></param>
        /// <param name="n"></param>
        /// <param name="det"></param>
        /// <param name="job"></param>
        public static unsafe void dpodi_(float *a, int *lda, int *n, float *det, int *job)
        {
            // System generated locals 
            int i__1;
            float d__1;

            // Local variables 
            int i, j, k;
            float s, t;
            int c__1 = 1;


            // compute determinant 

            if (*job / 10 == 0) 
            {
                goto L70;
            }

            det[0] = 1.0f;
            det[1] = 0.0f;
            s = 10.0f;
            for (i = 0; i < *n; ++i) 
            {
                d__1 = a[i + i * *lda];
                det[0] *= d__1 * d__1;
                if (det[0] == 0.0) 
                {
                    break;
                }
                while (det[0] < 1.0) 
                {
                    det[0] *= s;
                    det[1] += -1.0f;
                }
                while (det[0] >= s) 
                {
                    det[0] /= s;
                    det[1] += 1.0f;
                }
            }

            // compute inverse(r) 

L70:
            if (*job % 10 == 0) 
            {
                return;
            }
            for (k = 0; k < *n; ++k) 
            {
                a[k + k * *lda] = 1.0f / a[k + k * *lda];
                t = -a[k + k * *lda];
                i__1 = k;
                dscal_(&i__1, &t, &a[k * *lda], &c__1);
                if (*n < k + 2) 
                {
                    continue;
                }
                for (j = k + 1; j < *n; ++j) 
                {
                    t = a[k + j * *lda];
                    a[k + j * *lda] = 0.0f;
                    i__1 = k+1;
                    daxpy_(&i__1, &t, &a[k * *lda], &c__1, &a[j * *lda], &c__1);
                }
            }


            // form  inverse(r) * trans(inverse(r)) 

            for (j = 0; j < *n; ++j) 
            {
                for (k = 0; k < j; ++k) 
                {
                    t = a[k + j * *lda];
                    i__1 = k+1;
                    daxpy_(&i__1, &t, &a[j * *lda], &c__1, &a[k * *lda], &c__1);
                }
                t = a[j + j * *lda];
                i__1 = j+1;

	            // Change by BJT to fix diagonals - seems Vanroose broke it whe compared to 
	            // the fortran code. Duh. 
	            //        dscal_(&j, &t, &a[j * *lda], &c__1); 
                dscal_(&i__1, &t, &a[j * *lda], &c__1);
            }

        } 

        /// <summary>
        ///     dpoco factors a float precision symmetric positive definite     
        ///     matrix and estimates the condition of the matrix.                
        ///                                                                      
        ///     if  rcond  is not needed, dpofa is slightly faster.              
        ///     to solve  a*x = b , follow dpoco by dposl.                       
        ///     to compute  inverse(a)*c , follow dpoco by dposl.                
        ///     to compute  determinant(a) , follow dpoco by dpodi.              
        ///     to compute  inverse(a) , follow dpoco by dpodi.                  
        ///                                                                      
        ///     on entry                                                         
        ///                                                                      
        ///        a       float precision(lda, n)                              
        ///                the symmetric matrix to be factored.  only the        
        ///                diagonal and upper triangle are used.                 
        ///                                                                      
        ///        lda     integer                                               
        ///                the leading dimension of the array  a .               
        ///                                                                      
        ///        n       integer                                               
        ///                the order of the matrix  a .                          
        ///                                                                      
        ///     on return                                                        
        ///                                                                      
        ///        a       an upper triangular matrix  r  so that a = trans(r)*r 
        ///                where  trans(r)  is the transpose.                    
        ///                the strict lower triangle is unaltered.               
        ///                if  info .ne. 0 , the factorization is not complete.  
        ///                                                                      
        ///        rcond   float precision                                      
        ///                an estimate of the reciprocal condition of  a .       
        ///                for the system  a*x = b , relative perturbations      
        ///                in  a  and  b  of size  epsilon  may cause            
        ///                relative perturbations in  x  of size  epsilon/rcond .
        ///                if  rcond  is so small that the logical expression    
        ///                           1.0f + rcond .eq. 1.0f                       
        ///                is true, then  a  may be singular to working          
        ///                precision.  in particular,  rcond  is zero  if        
        ///                exact singularity is detected or the estimate         
        ///                underflows.  if info .ne. 0 , rcond is unchanged.     
        ///                                                                      
        ///        z       float precision(n)                                   
        ///                a work vector whose contents are usually unimportant. 
        ///                if  a  is close to a singular matrix, then  z  is     
        ///                an approximate null vector in the sense that          
        ///                norm(a*z) = rcond*norm(a)*norm(z) .                   
        ///                if  info .ne. 0 , z  is unchanged.                    
        ///                                                                      
        ///        info    integer                                               
        ///                = 0  for normal return.                               
        ///                = k  signals an error condition.  the leading minor   
        ///                     of order  k  is not positive definite.           
        ///                                                                      
        ///     linpack.  this version dated 08/14/78 .                          
        ///     cleve moler, university of new mexico, argonne national lab.     

        ///     subroutines and functions      
        ///                                    
        ///     linpack dpofa                  
        ///     blas daxpy,ddot,dscal,dasum    
        ///     fortran dabs,dmax1,dreal,dsign 
        /// </summary>
        /// <param name="a"></param>
        /// <param name="lda"></param>
        /// <param name="n"></param>
        /// <param name="rcond"></param>
        /// <param name="z"></param>
        /// <param name="info"></param>
        public static unsafe void dpoco_(float* a, int* lda, int* n, float* rcond, float* z, int* info)
        {
            // System generated locals 
            int a_dim1, a_offset, i__1;
            float d__1;

            // Local variables 
            int i, j, k;
            float s, t;
            float anorm;
            float ynorm;
            int kb;
            float ek, sm, wk;
            int jm1, kp1;
            float wkm;
            int c__1 = 1;

            // Parameter adjustments 
            --z;
            a_dim1 = *lda;
            a_offset = a_dim1 + 1;
            a -= a_offset;

            // find norm of a using only upper half 

            for (j = 1; j <= *n; ++j) 
            {
                z[j] = dasum_(&j, &a[j * a_dim1 + 1], &c__1);
                jm1 = j - 1;
                if (jm1 < 1) 
                {
                    goto L20;
                }
                for (i = 1; i <= jm1; ++i) 
                {
                    z[i] += Math.Abs(a[i + j * a_dim1]);
                }
                L20:
                    ;
            }
            anorm = 0.0f;
            for (j = 1; j <= *n; ++j) 
            {
                anorm = max(anorm,z[j]);
            }

            // factor 

            dpofa_(&a[a_offset], lda, n, info);
            if (*info != 0) 
            {
                goto L180;
            }

            // rcond = 1/(norm(a)*(estimate of norm(inverse(a)))) . 
            // estimate = norm(z)/norm(y) where  a*z = y  and  a*y = e . 
            // the components of  e  are chosen to cause maximum local 
            // growth in the elements of w  where  trans(r)*w = e . 
            // the vectors are frequently rescaled to avoid overflow. 

            // solve trans(r)*w = e 

            ek = 1.0f;
            for (j = 1; j <= *n; ++j) 
            {
                z[j] = 0.0f;
            }
            for (k = 1; k <= *n; ++k) 
            {
                if (z[k] != 0.0) 
                {
                    d__1 = -z[k];
                    ek = d_sign(&ek, &d__1);
                }
                if (Math.Abs(ek - z[k]) <= a[k + k * a_dim1]) 
                {
                    goto L60;
                }
                s = a[k + k * a_dim1] / Math.Abs(ek - z[k]);
                dscal_(n, &s, &z[1], &c__1);
                ek *= s;
        L60:
                wk = ek - z[k];
                wkm = -ek - z[k];
                s = Math.Abs(wk);
                sm = Math.Abs(wkm);
                wk /= a[k + k * a_dim1];
                wkm /= a[k + k * a_dim1];
                kp1 = k + 1;
                if (kp1 > *n) 
                {
                    goto L100;
                }
                for (j = kp1; j <= *n; ++j) 
                {
                    sm += Math.Abs(z[j] + wkm * a[k + j * a_dim1]);
                    z[j] += wk * a[k + j * a_dim1];
                    s += Math.Abs(z[j]);
                }
                if (s >= sm) 
                {
                    goto L90;
                }
                t = wkm - wk;
                wk = wkm;
                for (j = kp1; j <= *n; ++j) 
                {
                    z[j] += t * a[k + j * a_dim1];
                }
L90:
L100:
                z[k] = wk;
            }
            s = 1.0f / dasum_(n, &z[1], &c__1);
            dscal_(n, &s, &z[1], &c__1);

            // solve r*y = w 

            for (kb = 1; kb <= *n; ++kb) 
            {
                k = *n + 1 - kb;
                if (Math.Abs(z[k]) <= a[k + k * a_dim1]) 
                {
                    goto L120;
                }
                s = a[k + k * a_dim1] / Math.Abs(z[k]);
                dscal_(n, &s, &z[1], &c__1);
L120:
                z[k] /= a[k + k * a_dim1];
                t = -z[k];
                i__1 = k - 1;
                daxpy_(&i__1, &t, &a[k * a_dim1 + 1], &c__1, &z[1], &c__1);
            }
            s = 1.0f / dasum_(n, &z[1], &c__1);
            dscal_(n, &s, &z[1], &c__1);

            ynorm = 1.0f;

            // solve trans(r)*v = y 

            for (k = 1; k <= *n; ++k) 
            {
                i__1 = k - 1;
                z[k] -= ddot_(&i__1, &a[k * a_dim1 + 1], &c__1, &z[1], &c__1);
                if (Math.Abs(z[k]) <= a[k + k * a_dim1]) 
                {
                    goto L140;
                }
                s = a[k + k * a_dim1] / Math.Abs(z[k]);
                dscal_(n, &s, &z[1], &c__1);
                ynorm *= s;
L140:
                z[k] /= a[k + k * a_dim1];
            }
            s = 1.0f / dasum_(n, &z[1], &c__1);
            dscal_(n, &s, &z[1], &c__1);
            ynorm *= s;

            // solve r*z = v 

            for (kb = 1; kb <= *n; ++kb) 
            {
                k = *n + 1 - kb;
                if (Math.Abs(z[k]) <= a[k + k * a_dim1]) 
                {
                    goto L160;
                }
                s = a[k + k * a_dim1] / Math.Abs(z[k]);
                dscal_(n, &s, &z[1], &c__1);
                ynorm *= s;
L160:
                z[k] /= a[k + k * a_dim1];
                t = -z[k];
                i__1 = k - 1;
                daxpy_(&i__1, &t, &a[k * a_dim1 + 1], &c__1, &z[1], &c__1);
            }
            // make znorm = 1.0f 
            s = 1.0f / dasum_(n, &z[1], &c__1);
            dscal_(n, &s, &z[1], &c__1);
            ynorm *= s;

            if (anorm != 0.0) 
            {
                *rcond = ynorm / anorm;
            }
            if (anorm == 0.0) 
            {
                *rcond = 0.0f;
            }
L180:
            return;
        }


        /// <summary>
        /// takes the sum of the absolute values. 
        /// jack dongarra, linpack, 3/11/78. 
        /// modified 3/93 to return if incx .le. 0. 
        /// modified 12/3/93, array(1) declarations changed to array(*) 
        /// </summary>
        /// <param name="n"></param>
        /// <param name="dx"></param>
        /// <param name="incx"></param>
        /// <returns></returns>
        private static unsafe float dasum_(int* n, float* dx, int* incx)
        {
            // Local variables 
            int i, m, nincx;
            float dtemp;

            dtemp = 0.0f;
            if ((*n <= 0) || (*incx <= 0))
            {
                return dtemp;
            }
            if (*incx == 1)
            {

                // code for increment equal to 1 

                m = *n % 6;
                for (i = 0; i < m; ++i)
                {
                    dtemp += Math.Abs(dx[i]);
                }
                for (i = m; i < *n; i += 6)
                {
                    dtemp += Math.Abs(dx[i]) + Math.Abs(dx[i + 1]) + Math.Abs(dx[i + 2]) + Math.Abs(dx[i + 3]) + Math.Abs(dx[i + 4]) + Math.Abs(dx[i + 5]);
                }
                return dtemp;
            }

            // code for increment not equal to 1 

            nincx = *n * *incx;
            for (i = 0; i < nincx; i += *incx)
            {
                dtemp += Math.Abs(dx[i]);
            }
            return dtemp;
        }



        /// <summary>
        ///     dqrdc uses householder transformations to compute the qr         
        ///     factorization of an n by p matrix x.  column pivoting            
        ///     based on the 2-norms of the reduced columns may be               
        ///     performed at the users option.                                   
        ///                                                                      
        ///     on entry                                                         
        ///                                                                      
        ///        x       float precision(ldx,p), where ldx .ge. n.            
        ///                x contains the matrix whose decomposition is to be    
        ///                computed.                                             
        ///                                                                      
        ///        ldx     integer.                                              
        ///                ldx is the leading dimension of the array x.          
        ///                                                                      
        ///        n       integer.                                              
        ///                n is the number of rows of the matrix x.              
        ///                                                                      
        ///        p       integer.                                              
        ///                p is the number of columns of the matrix x.           
        ///                                                                      
        ///        jpvt    integer(p).                                           
        ///                jpvt contains integers that control the selection     
        ///                of the pivot columns.  the k-th column x(k) of x      
        ///                is placed in one of three classes according to the    
        ///                value of jpvt(k).                                     
        ///                                                                      
        ///                   if jpvt(k) .gt. 0, then x(k) is an initial         
        ///                                      column.                         
        ///                                                                      
        ///                   if jpvt(k) .eq. 0, then x(k) is a free column.     
        ///                                                                      
        ///                   if jpvt(k) .lt. 0, then x(k) is a final column.    
        ///                                                                      
        ///                before the decomposition is computed, initial columns 
        ///                are moved to the beginning of the array x and final   
        ///                columns to the end.  both initial and final columns   
        ///                are frozen in place during the computation and only   
        ///                free columns are moved.  at the k-th stage of the     
        ///                reduction, if x(k) is occupied by a free column       
        ///                it is interchanged with the free column of largest    
        ///                reduced norm.  jpvt is not referenced if              
        ///                job .eq. 0.                                           
        ///                                                                      
        ///        work    float precision(p).                                  
        ///                work is a work array.  work is not referenced if      
        ///                job .eq. 0.                                           
        ///                                                                      
        ///        job     integer.                                              
        ///                job is an integer that initiates column pivoting.     
        ///                if job .eq. 0, no pivoting is done.                   
        ///                if job .ne. 0, pivoting is done.                      
        ///                                                                      
        ///     on return                                                        
        ///                                                                      
        ///        x       x contains in its upper triangle the upper            
        ///                triangular matrix r of the qr factorization.          
        ///                below its diagonal x contains information from        
        ///                which the orthogonal part of the decomposition        
        ///                can be recovered.  note that if pivoting has          
        ///                been requested, the decomposition is not that         
        ///                of the original matrix x but that of x                
        ///                with its columns permuted as described by jpvt.       
        ///                                                                      
        ///        qraux   float precision(p).                                  
        ///               qraux contains further information required to recover 
        ///                the orthogonal part of the decomposition.             
        ///                                                                      
        ///        jpvt    jpvt(k) contains the index of the column of the       
        ///                original matrix that has been interchanged into       
        ///                the k-th column, if pivoting was requested.           
        ///                                                                      
        ///     linpack. this version dated 08/14/78 .                           
        ///     g.w. stewart, university of maryland, argonne national lab.      
        ///     
        ///     dqrdc uses the following functions and subprograms. 
        ///                                                         
        ///     blas daxpy,ddot,dscal,dswap,dnrm2                   
        ///     fortran dabs,dmax1,min0,dsqrt                       
        /// </summary>
        /// <param name="x"></param>
        /// <param name="ldx"></param>
        /// <param name="n"></param>
        /// <param name="p"></param>
        /// <param name="qraux"></param>
        /// <param name="jpvt"></param>
        /// <param name="work"></param>
        /// <param name="job"></param>
        public static unsafe void dqrdc_(float* x, int* ldx, int* n, int* p, float* qraux, int* jpvt, float* work, int* job)
        {
            // System generated locals 
            int i__1;
            float d__1;
            int c__1 = 1;

            // Local variables 
            bool negj;
            int maxj;
            int j, l;
            float t;
            bool swapj;
            float nrmxl;
            int jp, pl, pu;
            float tt, maxnrm;

            // internal variables 
            pl = 0;
            pu = -1;
            if (*job == 0)
            {
                goto L60;
            }

            // pivoting has been requested.  rearrange the columns 
            // according to jpvt. 

            for (j = 0; j < *p; ++j)
            {
                swapj = jpvt[j] > 0;
                negj = jpvt[j] < 0;
                jpvt[j] = j + 1;
                if (negj)
                {
                    jpvt[j] = -j - 1;
                }
                if (!swapj)
                {
                    continue;
                }
                if (j != pl)
                {
                    dswap_(n, &x[pl * *ldx], &c__1, &x[j * *ldx], &c__1);
                }
                jpvt[j] = jpvt[pl];
                jpvt[pl] = j + 1;
                ++pl;
            }
            pu = *p - 1;
            for (j = pu; j >= 0; --j)
            {
                if (jpvt[j] >= 0)
                {
                    continue;
                }
                jpvt[j] = -jpvt[j];
                if (j != pu)
                {
                    dswap_(n, &x[pu * *ldx], &c__1, &x[j * *ldx], &c__1);
                    jp = jpvt[pu];
                    jpvt[pu] = jpvt[j];
                    jpvt[j] = jp;
                }
                --pu;
            }
        L60:

            // compute the norms of the free columns. 

            for (j = pl; j <= pu; ++j)
            {
                qraux[j] = dnrm2_(n, &x[j * *ldx], &c__1);
                work[j] = qraux[j];
            }

            // perform the householder reduction of x. 

            for (l = 0; l < *n && l < *p; ++l)
            {
                if (l < pl || l >= pu)
                {
                    goto L120;
                }

                // locate the column of largest norm and bring it 
                // into the pivot position. 

                maxnrm = 0.0f;
                maxj = l;
                for (j = l; j <= pu; ++j)
                {
                    if (qraux[j] <= maxnrm)
                    {
                        continue;
                    }
                    maxnrm = qraux[j];
                    maxj = j;
                }
                if (maxj != l)
                {
                    dswap_(n, &x[l * *ldx], &c__1, &x[maxj * *ldx], &c__1);
                    qraux[maxj] = qraux[l];
                    work[maxj] = work[l];
                    jp = jpvt[maxj]; jpvt[maxj] = jpvt[l]; jpvt[l] = jp;
                }
            L120:
                qraux[l] = 0.0f;
                if (l + 1 == *n)
                {
                    continue;
                }

                // compute the householder transformation for column l. 

                i__1 = *n - l;
                nrmxl = dnrm2_(&i__1, &x[l + l * *ldx], &c__1);
                if (nrmxl == 0.0)
                {
                    continue;
                }
                if (x[l + l * *ldx] != 0.0)
                {
                    nrmxl = d_sign(&nrmxl, &x[l + l * *ldx]);
                }
                i__1 = *n - l;
                d__1 = 1.0f / nrmxl;
                dscal_(&i__1, &d__1, &x[l + l * *ldx], &c__1);
                x[l + l * *ldx] += 1.0f;

                // apply the transformation to the remaining columns, 
                // updating the norms. 

                for (j = l + 1; j < *p; ++j)
                {
                    i__1 = *n - l;
                    t = -ddot_(&i__1, &x[l + l * *ldx], &c__1,
                               &x[l + j * *ldx], &c__1) / x[l + l * *ldx];
                    daxpy_(&i__1, &t, &x[l + l * *ldx], &c__1, &x[l + j * *ldx], &c__1);
                    if ((j < pl) || (j > pu))
                    {
                        continue;
                    }
                    if (qraux[j] == 0.0)
                    {
                        continue;
                    }
                    tt = Math.Abs(x[l + j * *ldx]) / qraux[j];
                    tt = 1.0f - tt * tt;
                    tt = max(tt, 0.0f);
                    t = tt;
                    d__1 = qraux[j] / work[j];
                    tt = tt * 0.05f * d__1 * d__1 + 1.0f;
                    if (tt != 1.0)
                    {
                        qraux[j] *= (float)Math.Sqrt(t);
                        continue;
                    }
                    i__1 = *n - l - 1;
                    qraux[j] = dnrm2_(&i__1, &x[l + 1 + j * *ldx], &c__1);
                    work[j] = qraux[j];
                }

                // save the transformation. 

                qraux[l] = x[l + l * *ldx];
                x[l + l * *ldx] = -nrmxl;
            }
        }


        /// <summary>
        ///     dqrsl applies the output of dqrdc to compute coordinate 
        ///     transformations, projections, and least squares solutions. 
        ///     for k .le. min(n,p), let xk be the matrix 
        ///     
        ///            xk = (x(jpvt(1)),x(jpvt(2)), ... ,x(jpvt(k))) 
        ///            
        ///     formed from columnns jpvt(1), ... ,jpvt(k) of the original 
        ///     n x p matrix x that was input to dqrdc (if no pivoting was 
        ///     done, xk consists of the first k columns of x in their 
        ///     original order).  dqrdc produces a factored orthogonal matrix q 
        ///     and an upper triangular matrix r such that 
        ///     
        ///              xk = q * (r) 
        ///                       (0) 
        ///                       
        ///     this information is contained in coded form in the arrays 
        ///     x and qraux. 
        ///     
        ///     on entry 
        ///     
        ///        x      float precision(ldx,p). 
        ///               x contains the output of dqrdc. 
        ///               
        ///        ldx    integer. 
        ///               ldx is the leading dimension of the array x. 
        ///               
        ///        n      integer. 
        ///               n is the number of rows of the matrix xk.  it must 
        ///               have the same value as n in dqrdc. 
        ///               
        ///        k      integer. 
        ///               k is the number of columns of the matrix xk.  k 
        ///               must nnot be greater than min(n,p), where p is the 
        ///               same as in the calling sequence to dqrdc. 
        ///               
        ///        qraux  float precision(p). 
        ///               qraux contains the auxiliary output from dqrdc. 
        ///               
        ///        y      float precision(n) 
        ///               y contains an n-vector that is to be manipulated 
        ///               by dqrsl. 
        ///               
        ///        job    integer. 
        ///               job specifies what is to be computed.  job has 
        ///               the decimal expansion abcde, with the following 
        ///               meaning. 
        ///               
        ///                    if a.ne.0, compute qy. 
        ///                    if b,c,d, or e .ne. 0, compute qty. 
        ///                    if c.ne.0, compute b. 
        ///                    if d.ne.0, compute rsd. 
        ///                    if e.ne.0, compute xb. 
        ///                    
        ///               note that a request to compute b, rsd, or xb 
        ///               automatically triggers the computation of qty, for 
        ///               which an array must be provided in the calling 
        ///               sequence. 
        ///               
        ///     on return 
        ///     
        ///        qy     float precision(n). 
        ///               qy contains q*y, if its computation has been 
        ///               requested. 
        ///               
        ///        qty    float precision(n). 
        ///               qty contains trans(q)*y, if its computation has 
        ///               been requested.  here trans(q) is the 
        ///               transpose of the matrix q. 
        ///               
        ///        b      float precision(k) 
        ///               b contains the solution of the least squares problem 
        ///               
        ///                    minimize norm2(y - xk*b), 
        ///                    
        ///               if its computation has been requested.  (note that 
        ///               if pivoting was requested in dqrdc, the j-th 
        ///               component of b will be associated with column jpvt(j) 
        ///               of the original matrix x that was input into dqrdc.) 
        ///               
        ///        rsd    float precision(n). 
        ///               rsd contains the least squares residual y - xk*b, 
        ///               if its computation has been requested.  rsd is 
        ///               also the orthogonal projection of y onto the 
        ///               orthogonal complement of the column space of xk. 
        ///               
        ///        xb     float precision(n). 
        ///               xb contains the least squares approximation xk*b, 
        ///               if its computation has been requested.  xb is also 
        ///               the orthogonal projection of y onto the column space 
        ///               of x. 
        ///               
        ///        info   integer. 
        ///               info is zero unless the computation of b has 
        ///               been requested and r is exactly singular.  in 
        ///               this case, info is the index of the first zero 
        ///               diagonal element of r and b is left unaltered. 
        ///               
        ///     the parameters qy, qty, b, rsd, and xb are not referenced 
        ///     if their computation is not requested and in this case 
        ///     can be replaced by dummy variables in the calling program. 
        ///     to save storage, the user may in some cases use the same 
        ///     array for different parameters in the calling sequence.  a 
        ///     frequently occuring example is when one wishes to compute 
        ///     any of b, rsd, or xb and does not need y or qty.  in this 
        ///     case one may identify y, qty, and one of b, rsd, or xb, while 
        ///     providing separate arrays for anything else that is to be 
        ///     computed.  thus the calling sequence 
        ///     
        ///          call dqrsl(x,ldx,n,k,qraux,y,dum,y,b,y,dum,110,info) 
        ///          
        ///     will result in the computation of b and rsd, with rsd 
        ///     overwriting y.  more generally, each item in the following 
        ///     list contains groups of permissible identifications for 
        ///     a single callinng sequence. 
        ///     
        ///          1. (y,qty,b) (rsd) (xb) (qy) 
        ///          
        ///          2. (y,qty,rsd) (b) (xb) (qy) 
        ///          
        ///          3. (y,qty,xb) (b) (rsd) (qy) 
        ///          
        ///          4. (y,qy) (qty,b) (rsd) (xb) 
        ///          
        ///          5. (y,qy) (qty,rsd) (b) (xb) 
        ///          
        ///          6. (y,qy) (qty,xb) (b) (rsd) 
        ///          
        ///     in any group the value returned in the array allocated to 
        ///     the group corresponds to the last member of the group. 
        ///     
        ///     linpack. this version dated 08/14/78 . 
        ///     g.w. stewart, university of maryland, argonne national lab. 
        ///     
        ///     dqrsl uses the following functions and subprograms. 
        ///     
        ///     blas daxpy,dcopy,ddot 
        ///     fortran dabs,min0,mod 
        /// </summary>
        /// <param name="x"></param>
        /// <param name="ldx"></param>
        /// <param name="n"></param>
        /// <param name="k"></param>
        /// <param name="qraux"></param>
        /// <param name="y"></param>
        /// <param name="qy"></param>
        /// <param name="qty"></param>
        /// <param name="b"></param>
        /// <param name="rsd"></param>
        /// <param name="xb"></param>
        /// <param name="job"></param>
        /// <param name="info"></param>
        public static unsafe void dqrsl_(float *x, int *ldx, int *n, int *k, float *qraux, float *y, 
                                  float *qy, float *qty, float *b, float *rsd, float *xb, 
                                  int *job, int *info)
        {
            // System generated locals 
            int i__1;
            int c__1 = 1;

            // Local variables 
            float temp;
            bool cqty;
            int i, j;
            float t;
            bool cb;
            bool cr;
            int ju;
            bool cxb, cqy;

            // set info flag. 
            *info = 0;

            // determine what is to be computed. 

            cqy = *job / 10000 != 0;
            cqty = *job % 10000 != 0;
            cb = *job % 1000 / 100 != 0;
            cr = *job % 100 / 10 != 0;
            cxb = *job % 10 != 0;
            ju = min(*k,*n - 1);

            // special action when n=1. 

            if (ju == 0) 
            {
                if (cqy)  qy[0] = y[0];
                if (cqty) qty[0] = y[0];
                if (cxb)  xb[0] = y[0];
                if (cb) 
                {
                    if (x[0] == 0.0) 
                        *info = 1;
                    else            
                        b[0] = y[0] / x[0];
                }
                if (cr) rsd[0] = 0.0f;
                return;
            }

            // set up to compute qy or qty. 

            if (cqy) 
            {
                dcopy_(n, y, &c__1, qy, &c__1);
            }
            if (cqty) 
            {
                dcopy_(n, y, &c__1, qty, &c__1);
            }

            // compute qy. 

            if (cqy)
                for (j = ju-1; j >= 0; --j) 
                {
                    if (qraux[j] == 0.0)
                        continue;
                    temp = x[j + j * *ldx];
                    ((float*)x)[j + j * *ldx] = qraux[j]; // breaks const-ness, but will be restored 
                    i__1 = *n - j;
                    t = -ddot_(&i__1, &x[j + j * *ldx], &c__1, &qy[j], &c__1) / x[j + j * *ldx];
                    daxpy_(&i__1, &t, &x[j + j * *ldx], &c__1, &qy[j], &c__1);
                    ((float*)x)[j + j * *ldx] = temp; // restore original 
                }

            // compute trans(q)*y. 

            if (cqty)
                for (j = 0; j < ju; ++j) 
                {
                    if (qraux[j] == 0.0)
                        continue;
                    temp = x[j + j * *ldx];
                    ((float*)x)[j + j * *ldx] = qraux[j]; // breaks const-ness, but will be restored 
                    i__1 = *n - j;
                    t = -ddot_(&i__1, &x[j + j * *ldx], &c__1, &qty[j], &c__1) / x[j + j * *ldx];
                    daxpy_(&i__1, &t, &x[j + j * *ldx], &c__1, &qty[j], &c__1);
                    ((float*)x)[j + j * *ldx] = temp; // restore original 
                }

            // set up to compute b, rsd, or xb. 

            if (cb) 
            {
                dcopy_(k, qty, &c__1, b, &c__1);
            }
            if (cxb) 
            {
                dcopy_(k, qty, &c__1, xb, &c__1);
            }
            if (cr && *k < *n) 
            {
                i__1 = *n - *k;
                dcopy_(&i__1, &qty[*k], &c__1, &rsd[*k], &c__1);
            }
            if (cxb)
                for (i = *k; i < *n; ++i) 
                {
                    xb[i] = 0.0f;
                }
            if (cr)
                for (i = 0; i < *k; ++i) 
                {
                    rsd[i] = 0.0f;
                }

            // compute b. 

            if (cb)
                for (j = *k-1; j >= 0; --j) 
                {
                    if (x[j + j * *ldx] == 0.0) 
                    {
                        *info = j+1;
                        break;
                    }
                    b[j] /= x[j + j * *ldx];
                    if (j != 0) 
                    {
                        t = -b[j];
                        daxpy_(&j, &t, &x[j * *ldx], &c__1, b, &c__1);
                    }
                }
                if (! cr && ! cxb)
                    return;

            // compute rsd or xb as required. 

            for (j = ju-1; j >= 0; --j) 
            {
                if (qraux[j] == 0.0) 
                {
                    continue;
                }
                temp = x[j + j * *ldx];
                ((float*)x)[j + j * *ldx] = qraux[j]; // breaks const-ness, but will be restored 
                i__1 = *n - j;
                if (cr) 
                {
                    t = -ddot_(&i__1, &x[j + j * *ldx], &c__1, &rsd[j], &c__1) / x[j + j * *ldx];
                    daxpy_(&i__1, &t, &x[j + j * *ldx], &c__1, &rsd[j], &c__1);
                }
                if (cxb) 
                {
                    t = -ddot_(&i__1, &x[j + j * *ldx], &c__1, &xb[j], &c__1) / x[j + j * *ldx];
                    daxpy_(&i__1, &t, &x[j + j * *ldx], &c__1, &xb[j], &c__1);
                }
                ((float*)x)[j + j * *ldx] = temp; // restore original 
            }
        }



        /// <summary>
        ///     dsvdc is a subroutine to reduce a float precision nxp matrix x  
        ///     by orthogonal transformations u and v to diagonal form.  The     
        ///     diagonal elements s(i) are the singular values of x.  The        
        ///     columns of u are the corresponding left singular vectors,        
        ///     and the columns of v the right singular vectors.                 
        ///                                                                      
        ///     on entry                                                         
        ///                                                                      
        ///         x         float precision(ldx,p), where ldx.ge.n.           
        ///                   x contains the matrix whose singular value         
        ///                   decomposition is to be computed.  x is             
        ///                   destroyed by dsvdc.                                
        ///                                                                      
        ///         ldx       integer.                                           
        ///                   ldx is the leading dimension of the array x.       
        ///                                                                      
        ///         n         integer.                                           
        ///                   n is the number of rows of the matrix x.           
        ///                                                                      
        ///         p         integer.                                           
        ///                   p is the number of columns of the matrix x.        
        ///                                                                      
        ///         ldu       integer.                                           
        ///                   ldu is the leading dimension of the array u.       
        ///                   (see below).                                       
        ///                                                                      
        ///         ldv       integer.                                           
        ///                   ldv is the leading dimension of the array v.       
        ///                   (see below).                                       
        ///                                                                      
        ///         work      float precision(n).                               
        ///                   work is a scratch array.                           
        ///                                                                      
        ///         job       integer.                                           
        ///                   job controls the computation of the singular       
        ///                   vectors.  it has the decimal expansion ab          
        ///                   with the following meaning                         
        ///                                                                      
        ///                        a.eq.0f    do not compute the left singular    
        ///                                  vectors.                            
        ///                        a.eq.1    return the n left singular vectors  
        ///                                  in u.                               
        ///                        a.ge.2    return the first min(n,p) singular  
        ///                                  vectors in u.                       
        ///                        b.eq.0f    do not compute the right singular   
        ///                                  vectors.                            
        ///                        b.eq.1    return the right singular vectors   
        ///                                  in v.                               
        ///                                                                      
        ///     on return                                                        
        ///                                                                      
        ///         s         float precision(mm), where mm=min(n+1,p).         
        ///                   the first min(n,p) entries of s contain the        
        ///                   singular values of x arranged in descending        
        ///                   order of magnitude.                                
        ///                                                                      
        ///         e         float precision(p).                               
        ///                   e ordinarily contains zeros.  however see the      
        ///                   discussion of info for exceptions.                 
        ///                                                                      
        ///         u         float precision(ldu,k), where ldu.ge.n.  if       
        ///                               joba.eq.1 then k.eq.n, if joba.ge.2    
        ///                               then k.eq.min(n,p).                    
        ///                   u contains the matrix of left singular vectors.    
        ///                   u is not referenced if joba.eq.0.  if n.le.p       
        ///                   or if joba.eq.2, then u may be identified with x   
        ///                   in the subroutine call.                            
        ///                                                                      
        ///         v         float precision(ldv,p), where ldv.ge.p.           
        ///                   v contains the matrix of right singular vectors.   
        ///                   v is not referenced if job.eq.0.  if p.le.n,       
        ///                   then v may be identified with x in the             
        ///                   subroutine call.                                   
        ///                                                                      
        ///         info      integer.                                           
        ///                   the singular values (and their corresponding       
        ///                   singular vectors) s(info+1),s(info+2),...,s(m)     
        ///                   are correct (here m=min(n,p)).  thus if            
        ///                   info.eq.0, all the singular values and their       
        ///                   vectors are correct.  in any event, the matrix     
        ///                   b = trans(u)*x*v is the bidiagonal matrix          
        ///                   with the elements of s on its diagonal and the     
        ///                   elements of e on its super-diagonal (trans(u)      
        ///                   is the transpose of u).  thus the singular         
        ///                   values of x and b are the same.                    
        ///                                                                      
        ///     linpack. this version dated 08/14/78 .                           
        ///              correction made to shift 2/84.                          
        ///     g.w. stewart, university of maryland, argonne national lab.      

        ///     dsvdc uses the following functions and subprograms. 
        ///                                                         
        ///     external drot                                       
        ///     blas daxpy,ddot,dscal,dswap,dnrm2,drotg             
        ///     fortran dabs,dmax1,max0,min0,mod,dsqrt              
        /// </summary>
        /// <param name="x"></param>
        /// <param name="ldx"></param>
        /// <param name="n"></param>
        /// <param name="p"></param>
        /// <param name="s"></param>
        /// <param name="e"></param>
        /// <param name="u"></param>
        /// <param name="ldu"></param>
        /// <param name="v"></param>
        /// <param name="ldv"></param>
        /// <param name="work"></param>
        /// <param name="job"></param>
        /// <param name="info"></param>
        public static unsafe void dsvdc_(float *x, int *ldx, int *n, int *p, float *s, 
                                         float *e, float *u, int *ldu, float *v, int *ldv, 
                                         float *work, int *job, int *info)
        {
            // System generated locals 
            int i__1;
            float d__1;

            // Table of constant values 
            int c__1 = 1;
            float c_m1 = -1.0f;

            // Local variables 
            int kase, jobu, iter;
            float test;
            float b, c;
            float f, g;
            int i, j, k, l, m;
            float t, scale;
            float shift;
            int maxit;
            bool wantu, wantv;
            float t1, ztest, el;
            float cs;
            int mm, ls;
            float sl;
            int lu;
            float sm, sn;
            int lp1, nct, ncu, nrt;
            float emm1, smm1;

            // set the maximum number of iterations. 

            maxit = 30;

            // determine what is to be computed. 

            wantu = false;
            wantv = false;
            jobu = *job % 100 / 10;
            ncu = *n;
            if (jobu > 1) 
            {
                ncu = min(*n,*p);
            }
            if (jobu != 0) 
            {
                wantu = true;
            }
            if (*job % 10 != 0) 
            {
                wantv = true;
            }

            //     reduce x to bidiagonal form, storing the diagonal elements 
            //     in s and the super-diagonal elements in e. 

            *info = 0;
            nct = min(*n-1,*p);
            nrt = max(0,min(*p-2,*n));
            lu = max(nct,nrt);
            for (l = 0; l < lu; ++l) 
            {
                lp1 = l+1;
                if (lp1 > nct) 
                {
                    goto L20;
                }

                // compute the transformation for the l-th column and 
                // place the l-th diagonal in s(l). 

                i__1 = *n - l;
                s[l] = dnrm2_(&i__1, &x[l + l * *ldx], &c__1);
                if (s[l] == 0.0) 
                {
                    goto L10;
                }
                if (x[l + l * *ldx] != 0.0) 
                {
                    s[l] = d_sign(&s[l], &x[l + l * *ldx]);
                }
                i__1 = *n - l;
                d__1 = 1.0f / s[l];
                dscal_(&i__1, &d__1, &x[l + l * *ldx], &c__1);
                x[l + l * *ldx] += 1.0f;
                L10:
                s[l] = -s[l];
                L20:
                for (j = lp1; j < *p; ++j) 
                {
                    //  apply the transformation. 
                    if ((l < nct) && (s[l] != 0.0)) 
                    {
                        i__1 = *n - l;
                        t = -ddot_(&i__1, &x[l + l * *ldx], &c__1, &x[l + j * *ldx], &c__1) / x[l + l * *ldx];
                        daxpy_(&i__1, &t, &x[l + l * *ldx], &c__1, &x[l + j * *ldx], &c__1);
                    }

                    // place the l-th row of x into  e for the 
                    // subsequent calculation of the row transformation. 

                    e[j] = x[l + j * *ldx];
                }

                // place the transformation in u for subsequent back 
                // multiplication. 

                if (wantu && l < nct)
                    for (i = l; i < *n; ++i) 
                    {
                        u[i + l * *ldu] = x[i + l * *ldx];
                    }
                if (lp1 > nrt) 
                {
                    continue;
                }

                // compute the l-th row transformation and place the 
                // l-th super-diagonal in e(l). */

                i__1 = *p - lp1;
                e[l] = dnrm2_(&i__1, &e[lp1], &c__1);
                if (e[l] == 0.0) 
                {
                    goto L80;
                }
                if (e[lp1] != 0.0) 
                {
                    e[l] = d_sign(&e[l], &e[lp1]);
                }
                i__1 = *p - lp1;
                d__1 = 1.0f / e[l];
                dscal_(&i__1, &d__1, &e[lp1], &c__1);
                e[lp1] += 1.0f;
                L80:
                e[l] = -e[l];
                if (l+2 > *n || e[l] == 0.0) 
                {
                    goto L120;
                }

                //  apply the transformation.

                for (i = lp1; i < *n; ++i) 
                {
                    work[i] = 0.0f;
                }
                for (j = lp1; j < *p; ++j) 
                {
                    i__1 = *n - lp1;
                    daxpy_(&i__1, &e[j], &x[lp1 + j * *ldx], &c__1, &work[lp1], &c__1);
                }
                for (j = lp1; j < *p; ++j) 
                {
                    i__1 = *n - lp1;
                    d__1 = -e[j] / e[lp1];
                    daxpy_(&i__1, &d__1, &work[lp1], &c__1, &x[lp1 + j * *ldx], &c__1);
                }
                L120:

                // place the transformation in v for subsequent 
                // back multiplication. 

                if (wantv)
                    for (i = lp1; i < *p; ++i) 
                    {
                        v[i + l * *ldv] = e[i];
                    }
            }

            // set up the final bidiagonal matrix or order m. 

            m = min(*p-1,*n);
            if (nct < *p) 
            {
                s[nct] = x[nct + nct * *ldx];
            }
            if (*n < m+1) 
            {
                s[m] = 0.0f;
            }
            if (nrt < m) 
            {
                e[nrt] = x[nrt + m * *ldx];
            }
            e[m] = 0.0f;

            //     if required, generate u. 

            if (wantu)
                for (j = nct; j < ncu; ++j) 
                {
                    for (i = 0; i < *n; ++i) 
                    {
                        u[i + j * *ldu] = 0.0f;
                    }
                    u[j + j * *ldu] = 1.0f;
                }
            if (wantu)
                for (l = nct-1; l >= 0; --l) 
                {
                    if (s[l] == 0.0) 
                    {
                        for (i = 0; i < *n; ++i) 
                        {
                            u[i + l * *ldu] = 0.0f;
                        }
                        u[l + l * *ldu] = 1.0f;
                        continue;
                    }
                    for (j = l+1; j < ncu; ++j) 
                    {
                        i__1 = *n - l;
                        t = -ddot_(&i__1, &u[l + l * *ldu], &c__1, &u[l + j * *ldu], &c__1) / u[l + l * *ldu];
                        daxpy_(&i__1, &t, &u[l + l * *ldu], &c__1, &u[l + j * *ldu], &c__1);
                    }
                    i__1 = *n - l;
                    dscal_(&i__1, &c_m1, &u[l + l * *ldu], &c__1);
                    u[l + l * *ldu] += 1.0f;
                    for (i = 0; i < l; ++i) 
                    {
                        u[i + l * *ldu] = 0.0f;
                    }
                }

            // if it is required, generate v. 

            if (wantv)
                for (l = *p-1; l >= 0; --l) 
                {
                    lp1 = l+1;
                    if (l < nrt && e[l] != 0.0)
                        for (j = lp1; j < *p; ++j) 
                        {
                            i__1 = *p - lp1;
                            t = -ddot_(&i__1, &v[lp1 + l * *ldv], &c__1, &v[lp1 + j * *ldv], &c__1) / v[lp1 + l * *ldv];
                            daxpy_(&i__1, &t, &v[lp1 + l * *ldv], &c__1, &v[lp1 + j * *ldv], &c__1);
                        }
                    for (i = 0; i < *p; ++i) 
                    {
                        v[i + l * *ldv] = 0.0f;
                    }
                    v[l + l * *ldv] = 1.0f;
                }

            // main iteration loop for the singular values. 

            mm = m;
            iter = 0;
            L360:

            // quit if all the singular values have been found. 

            if (m < 0) 
            {
                return;
            }

            // if too many iterations have been performed, set 
            // flag and return. 

            if (iter >= maxit) 
            {
                *info = m+1;
                return;
            }

            // this section of the program inspects for 
            // negligible elements in the s and e arrays.  on 
            // completion the variables kase and l are set as follows. 

            //   kase = 1     if s(m) and e(l-1) are negligible and l.lt.m 
            //   kase = 2     if s(l) is negligible and l.lt.m 
            //   kase = 3     if e(l-1) is negligible, l.lt.m, and 
            //                s(l), ..., s(m) are not negligible (qr step). 
            //   kase = 4     if e(m-1) is negligible (convergence). 

            for (l = m-1; l >= 0; --l) 
            {
                test = Math.Abs(s[l]) + Math.Abs(s[l+1]);
                ztest = test + Math.Abs(e[l]);
                if (fsm_ieee_floats_equal(&ztest, &test)) 
                {
                    // WAS: if (ztest == test) { 
                    e[l] = 0.0f;
                    break;
                }
            }
            if (l == m-1) 
            {
                kase = 4;
                goto L480;
            }
            for (ls = m; ls > l; --ls) 
            {
                test = 0.0f;
                if (ls != m) 
                {
                    test += Math.Abs(e[ls]);
                }
                if (ls != l+1) 
                {
                    test += Math.Abs(e[ls-1]);
                }
                ztest = test + Math.Abs(s[ls]);
                if (fsm_ieee_floats_equal(&ztest, &test)) 
                {
                    // WAS: if (ztest == test) { 
                    s[ls] = 0.0f;
                    break;
                }
            }
            if (ls == l) 
            {
                kase = 3;
            }
            else 
                if (ls == m) 
                {
                    kase = 1;
                }
                else 
                {
                    kase = 2;
                    l = ls;
                }
            L480:
            ++l;

            // perform the task indicated by kase. 

            switch ((int)kase) 
            {
                case 1:  goto L490;
                case 2:  goto L520;
                case 3:  goto L540;
                case 4:  goto L570;
            }

            // deflate negligible s(m). 

            L490:
            f = e[m-1];
            e[m-1] = 0.0f;
            for (k = m-1; k >= l; --k) 
            {
                t1 = s[k];
                drotg_(&t1, &f, &cs, &sn);
                s[k] = t1;
                if (k != l) 
                {
                    f = -sn * e[k-1];
                    e[k-1] *= cs;
                }
                if (wantv) 
                {
                    drot_(p, &v[k * *ldv], &c__1, &v[m * *ldv], &c__1, &cs, &sn);
                }
            }
            goto L360;

            // split at negligible s(l). 

            L520:
            f = e[l-1];
            e[l-1] = 0.0f;
            for (k = l; k <= m; ++k) 
            {
                t1 = s[k];
                drotg_(&t1, &f, &cs, &sn);
                s[k] = t1;
                f = -sn * e[k];
                e[k] *= cs;
                if (wantu) 
                {
                    drot_(n, &u[k * *ldu], &c__1, &u[(l-1) * *ldu], &c__1, &cs, &sn);
                }
            }
            goto L360;

            // perform one qr step. 

            L540:

            // calculate the shift. 

            scale = max(max(max(max(Math.Abs(s[m]),Math.Abs(s[m-1])),Math.Abs(e[m-1])),Math.Abs(s[l])),Math.Abs(e[l]));
            sm = s[m] / scale;
            smm1 = s[m-1] / scale;
            emm1 = e[m-1] / scale;
            sl = s[l] / scale;
            el = e[l] / scale;
            b = ((smm1 + sm) * (smm1 - sm) + emm1 * emm1) / 2.0f;
            c = sm * emm1; c *= c;
            if ((b == 0.0) && (c == 0.0))
            {
                shift = 0.0f;
            }
            else 
            {
                shift = (float)Math.Sqrt(b * b + c);
                if (b < 0.0) 
                {
                    shift = -shift;
                }
                shift = c / (b + shift);
            }
            f = (sl + sm) * (sl - sm) + shift;
            g = sl * el;

            // chase zeros. 

            for (k = l; k < m; ++k) 
            {
                drotg_(&f, &g, &cs, &sn);
                if (k != l) 
                {
                    e[k-1] = f;
                }
                f = cs * s[k] + sn * e[k];
                e[k] = cs * e[k] - sn * s[k];
                g = sn * s[k+1];
                s[k+1] *= cs;
                if (wantv) 
                {
                    drot_(p, &v[k * *ldv], &c__1, &v[(k+1) * *ldv], &c__1, &cs, &sn);
                }
                drotg_(&f, &g, &cs, &sn);
                s[k] = f;
                f = cs * e[k] + sn * s[k+1];
                s[k+1] = -sn * e[k] + cs * s[k+1];
                g = sn * e[k+1];
                e[k+1] *= cs;
                if (wantu && k+1 < *n) 
                {
                    drot_(n, &u[k * *ldu], &c__1, &u[(k+1) * *ldu], &c__1, &cs, &sn);
                }
            }
            e[m-1] = f;
            ++iter;
            goto L360;

            // convergence. 

            L570:

            // make the singular value  positive. 

            if (s[l] < 0.0) 
            {
                s[l] = -s[l];
                if (wantv) 
                {
                    dscal_(p, &c_m1, &v[l * *ldv], &c__1);
                }
            }

            // order the singular value. 

            L590:
            if (l == mm) 
            {
                goto L600;
            }
            if (s[l] >= s[l+1]) 
            {
                goto L600;
            }
            t = s[l];
            s[l] = s[l+1];
            ++l;
            s[l] = t;
            if (wantv && l < *p) 
            {
                dswap_(p, &v[(l-1) * *ldv], &c__1, &v[l * *ldv], &c__1);
            }
            if (wantu && l < *n) 
            {
                dswap_(n, &u[(l-1) * *ldu], &c__1, &u[l * *ldu], &c__1);
            }
            goto L590;
            L600:
            iter = 0;
            --m;
            goto L360;
        }

        /// <summary>
        /// Calling this ensures that the operands are spilled to
        /// memory and thus avoids excessive precision when compiling
        /// for x86 with heavy optimization (gcc). It is better to do
        /// this than to turn on -ffloat-store.
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns></returns>
        private static unsafe bool fsm_ieee_floats_equal(float *x, float *y)
        {
            return (*x == *y);
        }


    }
}
