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
using SceneLibrary;

namespace monoSLAM
{

    /// <summary>
    /// General motion model in 3D
    /// Assumes a random impulse changes the velocity at each time step
    /// State vector: 13 elements
    ///                 x 
    ///  r              y 
    ///                 z 
    ///  -              - 
    ///                 q0 
    ///  q              qx
    ///                 qy
    ///                 qz
    ///  -      =       - 
    ///                 vx
    ///  v              vy
    ///                 vz
    ///  -              - 
    ///                 omegax
    ///  omega          omegay
    ///                 omegaz
    ///                 
    /// Control vector has 3 elements, and represents a measurement of 
    /// acceleration if a linear accelerometer is present. Otherwise set it
    /// to zero.
    /// 
    /// Noise vector n = V
    ///                  Omega
    /// Update:
    /// rnew     =   r + (v + V) delta_t
    /// qnew     =   q x q((omega + Omega) delta_t)
    /// vnew     =   v + V
    /// omeganew =   omega + Omega
    /// 
    /// Impulse ThreeD Motion Model
    /// 
    /// Class to represent a constant-velocity motion model. 
    /// A constant-velocity motion model does not mean that the camera moves at a constant velocity over all time, but that the statistical model of its motion in a timestep is that on average we expect undetermined accelerations occur with a Gaussian profile. At each timestep there is a random impulse, but these are normally-distributed.
    /// 
    /// The dimension of the state vector is 13: seven for the usual position state vector, plus three for the translational velocity and three for the rotational velocity.
    /// </summary>
    public class Impulse_ThreeD_Motion_Model : ThreeD_Motion_Model
    {
        private Random rnd = new Random();

        // Constants  
        // May need a lot of tweaking!
        /// Standard deviation of linear acceleration (in ms^{-2})
        public float SD_A_component_filter;
        /// Standard deviation of linear acceleration, for simulation.
        public float SD_A_component;
        // Standard deviation of angular acceleration
        /// Standard deviation of linear acceleration (in rad s^{-2})
        public float SD_alpha_component_filter;
        /// Standard deviation of angular acceleration, for simulation.
        public float SD_alpha_component;


        /// <summary>
        /// Constructor. Sets standard deviations of linear acceleration (6ms^{-2}) and angular acceleration (4rads^{-1}).
        /// </summary>
        public Impulse_ThreeD_Motion_Model() : base(13, 3, "IMPULSE_THREED")
        {            
            SD_A_component_filter = 4.0f;      // m s^-2
            SD_A_component = 4.0f;             // for simulation
            SD_alpha_component_filter = 6.0f;  // rad s^-2
            SD_alpha_component = 6.0f;         // for simulation
        }


        /// <summary>
        /// 
        /// </summary>
        /// <param name="xv">position and orientation of the camera</param>
        /// <param name="u">Control vector of accelerations</param>
        /// <param name="delta_t">time step in seconds</param>
        public override void func_fv_and_dfv_by_dxv(Vector xv, Vector u, float delta_t)
        {
            Vector3D rold, vold, omegaold, rnew, vnew, omeganew;
            Quaternion qold, qnew;            

            // Separate things out to make it clearer
            rold = new Vector3D(0, 0, 0);
            vold = new Vector3D(0, 0, 0);
            omegaold = new Vector3D(0, 0, 0);
            qold = new Quaternion();
            extract_r_q_v_omega(xv, rold, qold, vold, omegaold);

            Vector3D acceleration = new Vector3D(u);
           
            // rnew = r + v * delta_t
            //rnew = (Vector3D)((Point3D)rold + (Point3D)vold * delta_t);
            rnew = new Vector3D(rold + vold * delta_t);

            // qnew = q x q(omega * delta_t)

            // Keep qwt ( = q(omega * delta_t)) for later use
            Quaternion qwt = new Quaternion(omegaold * delta_t);

            qnew = qold.Multiply(qwt);

            // vnew = v
            vnew = new Vector3D(vold + acceleration * delta_t);
  
            // omeganew = omega
            omeganew = omegaold;

            

            // Put it all together
            compose_xv(ref rnew, ref qnew, ref vnew, ref omeganew, ref fvRES);

            // cout << "rold qold vold omegaold" << rold << qold 
            //      << vold << omegaold;
            // cout << "rnew qnew vnew omeganew" << rnew << qnew 
            //      << vnew << omeganew;

            // Now on to the Jacobian...
            // Identity is a good place to start since overall structure is like this
            // I       0             I * delta_t   0
            // 0       dqnew_by_dq   0             dqnew_by_domega
            // 0       0             I             0
            // 0       0             0             I
            dfv_by_dxvRES.SetIdentity();

            // Fill in dxnew_by_dv = I * delta_t
            MatrixFixed Temp33A = new MatrixFixed(3,3);
            Temp33A.SetIdentity();
            Temp33A *= delta_t;
            dfv_by_dxvRES.Update(Temp33A, 0, 7);

            // Fill in dqnew_by_dq
            // qnew = qold x qwt  ( = q3 = q2 x q1 in Scene/newbits.cc language)
            MatrixFixed Temp44A = MatrixFixed.dq3_by_dq2(qwt); //4,4
            dfv_by_dxvRES.Update(Temp44A, 3, 3);

            // Fill in dqnew_by_domega = d(q x qwt)_by_dqwt . dqwt_by_domega
            Temp44A = MatrixFixed.dq3_by_dq1(qold); // Temp44A is d(q x qwt) by dqwt
 
            // Use function below for dqwt_by_domega
            MatrixFixed Temp43A = new MatrixFixed(4,3);
            dqomegadt_by_domega(omegaold, delta_t, Temp43A);
            // Multiply them together
            MatrixFixed Temp43B = Temp44A * Temp43A;
            // And plug it in
            dfv_by_dxvRES.Update(Temp43B, 3, 10);
             
             

            // cout << "dfv_by_dxvRES" << dfv_by_dxvRES;
        }


        /// <summary>
        /// Fill noise covariance matrix Pnn: this is the covariance of 
        /// the noise vector (V)
        ///                  (Omega)
        /// that gets added to the state. 
        /// Form of this could change later, but for now assume that 
        /// V and Omega are independent, and that each of their components is
        /// independent... 
        /// </summary>
        /// <param name="xv"></param>
        /// <param name="v"></param>
        /// <param name="delta_t"></param>
        public override void func_Q(Vector xv, Vector v, float delta_t)
        {
            float linear_velocity_noise_variance = 
                SD_A_component_filter * SD_A_component_filter * delta_t * delta_t;
            float angular_velocity_noise_variance =
                SD_alpha_component_filter * SD_alpha_component_filter * delta_t * delta_t;

            // Independence means that the matrix is diagonal
            MatrixFixed Pnn = new MatrixFixed(6,6);
            Pnn.Fill(0.0f);
            Pnn.Put(0, 0, linear_velocity_noise_variance);
            Pnn.Put(1, 1, linear_velocity_noise_variance);
            Pnn.Put(2, 2, linear_velocity_noise_variance);
            Pnn.Put(3, 3, angular_velocity_noise_variance);
            Pnn.Put(4, 4, angular_velocity_noise_variance);
            Pnn.Put(5, 5, angular_velocity_noise_variance);

            // Form Jacobian dxnew_by_dn
            // Is like this:
            // I * delta_t     0
            // 0               dqnew_by_dOmega
            // I               0
            // 0               I

            // Start by zeroing
            MatrixFixed dxnew_by_dn = new MatrixFixed(13,6);
            dxnew_by_dn.Fill(0.0f);

            // Fill in easy bits first
            MatrixFixed Temp33A = new MatrixFixed(3,3);
            Temp33A.SetIdentity();
  
            dxnew_by_dn.Update(Temp33A, 7, 0);
            dxnew_by_dn.Update(Temp33A, 10, 3);
            Temp33A *= delta_t;
            dxnew_by_dn.Update(Temp33A, 0, 0);

            // Tricky bit is dqnew_by_dOmega
            // Is actually the same calculation as in func_fv...
            // Since omega and Omega are additive...?
            Vector3D rold = new Vector3D(0, 0, 0);
            Vector3D vold = new Vector3D(0, 0, 0);
            Vector3D omegaold = new Vector3D(0, 0, 0);
            Quaternion qold=new Quaternion();
            extract_r_q_v_omega(xv, rold, qold, vold, omegaold); // overkill but easy
            // Fill in dqnew_by_domega = d(q x qwt)_by_dqwt . dqwt_by_domega
            // Temp44A is d(q x qwt) by dqwt
            MatrixFixed Temp44A = MatrixFixed.dq3_by_dq1(qold); 
            // Use function below for dqwt_by_domega
            MatrixFixed Temp43A = new MatrixFixed(4,3);
            dqomegadt_by_domega(omegaold, delta_t, Temp43A);
            // Multiply them together
            MatrixFixed Temp43B = Temp44A * Temp43A;
            // And then plug into Jacobian
            dxnew_by_dn.Update(Temp43B, 3, 3);

            // Finally do Q = dxnew_by_dn . Pnn . dxnew_by_dnT
            QxRES.Update( dxnew_by_dn * Pnn * dxnew_by_dn.Transpose() );

            //  cout << "QxRES" << QxRES;
        }

        /// <summary>
        /// Extract the position and orientation from the state vector. 
        /// (This is the first seven elements in this case.)
        /// </summary>
        /// <param name="xv"></param>
        public override void func_xp(Vector xv)
        {
            xpRES = xv.Extract(7, 0);
        }

        /// <summary>
        /// Calculate Jacobian for func_xp. (This is just the identity in this case.)
        /// </summary>
        /// <param name="v"></param>
        public override void func_dxp_by_dxv(Vector v)
        {
            dxp_by_dxvRES.Fill(0.0f);

            dxp_by_dxvRES.Put(0, 0, 1.0f);
            dxp_by_dxvRES.Put(1, 1, 1.0f);
            dxp_by_dxvRES.Put(2, 2, 1.0f);
            dxp_by_dxvRES.Put(3, 3, 1.0f);
            dxp_by_dxvRES.Put(4, 4, 1.0f);
            dxp_by_dxvRES.Put(5, 5, 1.0f);
            dxp_by_dxvRES.Put(6, 6, 1.0f);
        }


        /// <summary>
        /// Noisy process equation for simulation
        /// Simply perturb xv with Gaussian noise and send it through func_fv
        /// </summary>
        /// <param name="xv_true"></param>
        /// <param name="u_true"></param>
        /// <param name="delta_t"></param>
        public override void func_fv_noisy(Vector xv_true, Vector u_true, float delta_t)
        {
            Vector xv_noisy = xv_true;

            // Linear velocity
            for (int row = 7; row < 10; row++)
                xv_noisy[row] = SceneLib.SampleNormal(xv_true[row], SD_A_component * delta_t, rnd);
            // Angular velocity
            for (int row = 10; row < 13; row++)
                xv_noisy[row] = SceneLib.SampleNormal(xv_true[row], SD_alpha_component * delta_t, rnd);

            // Now send through normal process equaion
            func_fv_and_dfv_by_dxv(xv_noisy, u_true, delta_t);

            // And copy result
            fv_noisyRES.Update(fvRES);
        }


        public override void func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef(
                        Vector xv, Vector xpdef)
        {
            // When we redefine axes:
            // r and q change as normal
            // v and omega are vectors so they change in the same way as r

            // We can mainly use the stuff from the general redefinition of axes in
            // position coordinates, but need to change it a little bit
            // State is          
            //               x, y, z, q0, qx, qy, qz, vx, vy, vz, omegax, omegay, omegaz
            // Position state is 
            //               x, y, z, q0, qx, qy, qz

            func_xp(xv);
            //Vector local_xp = xpRES;
            Vector local_xp = new Vector(xpRES);

            func_xpredef_and_dxpredef_by_dxp_and_dxpredef_by_dxpdef(local_xp, xpdef);

            Debug.WriteLine("func_xvredef... not yet implemented for Impulse_ThreeD_Motion_Model" +
                             "... though surely it's not too hard?");
            //assert(0);
        }

        public override void func_xvnorm_and_dxvnorm_by_dxv(Vector xv)
        {
            // Normalise the state vector: since quaternion is redundant we sometimes
            // need to enforce that it stays with size 1

            // Most parts of the state vector don't change so copy as starting point
            xvnormRES.Update(xv);

            // Most parts of Jacobian are identity
            dxvnorm_by_dxvRES.SetIdentity();

            // Extract quaternion  
            func_xp(xv);
            func_q(xpRES);
  
            Quaternion Tempqa = qRES;

            Quaternion Tempqb = (Tempqa);
            MatrixFixed Temp44a = MatrixFixed.dqnorm_by_dq(Tempqa);

            xvnormRES.Update(Tempqb.GetRXYZ(), 3);
            dxvnorm_by_dxvRES.Update(Temp44a, 3, 3);
        }


        /// <summary>
        /// Extract the component parts of the state  x_v . Fills matrices r, q, v, omega with values.
        /// </summary>
        /// <param name="xv"></param>
        /// <param name="r"></param>
        /// <param name="q"></param>
        /// <param name="v"></param>
        /// <param name="omega"></param>
        public void extract_r_q_v_omega(Vector xv, Vector3D r, Quaternion q, Vector3D v, Vector3D omega)
        {
            r.SetVNL3(xv.Extract(3, 0));

            Vector qRXYZ = xv.Extract(4, 3);
            q.SetRXYZ(qRXYZ);

            v.SetVNL3(xv.Extract(3, 7));

            omega.SetVNL3(xv.Extract(3, 10));
        }

        /// <summary>
        /// Create a state vector x_v from its component parts. Puts the matrices r, q, v, omega into their right places.
        /// </summary>
        /// <param name="r"></param>
        /// <param name="q"></param>
        /// <param name="v"></param>
        /// <param name="omega"></param>
        /// <param name="xv"></param>
        public void compose_xv(ref Vector3D r, ref Quaternion q, ref Vector3D v, ref Vector3D omega, ref Vector xv)
        {  
            xv.Update(r.GetVNL3(), 0);

            xv.Update(q.GetRXYZ(), 3);

            xv.Update(v.GetVNL3(), 7);
   
            xv.Update(omega.GetVNL3(), 10);
        }

        /// <summary>
        /// Calculate commonly used Jacobian part  \partial q(\omega * \Delta t) / \partial \omega .
        /// </summary>
        /// <param name="omega"></param>
        /// <param name="delta_t"></param>
        /// <param name="dqomegadt_by_domega"></param>
        public void dqomegadt_by_domega(Vector3D omega, float delta_t, MatrixFixed dqomegadt_by_domega)
        {
            // Modulus
            float omegamod = (float)Math.Sqrt(omega.GetX() * omega.GetX() + 
			                  omega.GetY() * omega.GetY() + 
			                  omega.GetZ() * omega.GetZ());

            // Use generic ancillary functions to calculate components of Jacobian  
            dqomegadt_by_domega.Put(0, 0, dq0_by_domegaA(omega.GetX(), omegamod, delta_t));
            dqomegadt_by_domega.Put(0, 1, dq0_by_domegaA(omega.GetY(), omegamod, delta_t));
            dqomegadt_by_domega.Put(0, 2, dq0_by_domegaA(omega.GetZ(), omegamod, delta_t));
            dqomegadt_by_domega.Put(1, 0, dqA_by_domegaA(omega.GetX(), omegamod, delta_t));
            dqomegadt_by_domega.Put(1, 1, dqA_by_domegaB(omega.GetX(), omega.GetY(), omegamod, delta_t));
            dqomegadt_by_domega.Put(1, 2, dqA_by_domegaB(omega.GetX(), omega.GetZ(), omegamod, delta_t));
            dqomegadt_by_domega.Put(2, 0, dqA_by_domegaB(omega.GetY(), omega.GetX(), omegamod, delta_t));
            dqomegadt_by_domega.Put(2, 1, dqA_by_domegaA(omega.GetY(), omegamod, delta_t));
            dqomegadt_by_domega.Put(2, 2, dqA_by_domegaB(omega.GetY(), omega.GetZ(), omegamod, delta_t));
            dqomegadt_by_domega.Put(3, 0, dqA_by_domegaB(omega.GetZ(), omega.GetX(), omegamod, delta_t));
            dqomegadt_by_domega.Put(3, 1, dqA_by_domegaB(omega.GetZ(), omega.GetY(), omegamod, delta_t));
            dqomegadt_by_domega.Put(3, 2, dqA_by_domegaA(omega.GetZ(), omegamod, delta_t));
        }


// Ancillary functions: calculate parts of Jacobian dq_by_domega
// which are repeatable due to symmetry.
// Here omegaA is one of omegax, omegay, omegaz
// omegaB, omegaC are the other two
// And similarly with qA, qB, qC

        /// <summary>
        /// Ancillary function to calculate part of Jacobian  \partial q / \partial \omega  which is repeatable due to symmetry. Here omegaA is one of omegax, omegay, omegaz.
        /// </summary>
        /// <param name="omegaA"></param>
        /// <param name="omega"></param>
        /// <param name="delta_t"></param>
        /// <returns></returns>
        public float dq0_by_domegaA(float omegaA, float omega, float delta_t)
        {
            return (-delta_t / 2.0f) * (omegaA / omega) * (float)Math.Sin(omega * delta_t / 2.0f);
        }

        /// <summary>
        /// Ancillary function to calculate part of Jacobian  \partial q / \partial \omega  which is repeatable due to symmetry. Here omegaA is one of omegax, omegay, omegaz and similarly with qA.
        /// </summary>
        /// <param name="omegaA"></param>
        /// <param name="omega"></param>
        /// <param name="delta_t"></param>
        /// <returns></returns>
        public float dqA_by_domegaA(float omegaA, float omega, float delta_t)
        {
            return (delta_t / 2.0f) * omegaA * omegaA / (omega * omega)
                   * (float)Math.Cos(omega * delta_t / 2.0f)
                   + (1.0f / omega) * (1.0f - omegaA * omegaA / (omega * omega))
                   * (float)Math.Sin(omega * delta_t / 2.0f);
        }

        /// <summary>
        /// Ancillary function to calculate part of Jacobian  \partial q / \partial \omega  which is repeatable due to symmetry. Here omegaB is one of omegax, omegay, omegaz and similarly with qA.
        /// </summary>
        /// <param name="omegaA"></param>
        /// <param name="omegaB"></param>
        /// <param name="omega"></param>
        /// <param name="delta_t"></param>
        /// <returns></returns>
        public float dqA_by_domegaB(float omegaA, float omegaB, float omega, float delta_t)
        {
            return (omegaA * omegaB / (omega * omega)) *
                   ((delta_t / 2.0f) * (float)Math.Cos(omega * delta_t / 2.0f)
                   - (1.0f / omega) * (float)Math.Sin(omega * delta_t / 2.0f));
        }


    }
}
