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
// General motion model in 3D
// Assumes a random motion changes the pose at each time step
// State vector: 7 elements
//                 x 
//  r              y 
//                 z 
//  -              - 
//                 q0 
//  q              qx
//                 qy
//                 qz
//
// Control vector is ignored
//
// Noise vector n = R
//                  Q
// Update:
// rnew     =   r + R
// qnew     =   q x Q


    /// <summary>
    /// Class to represent a zero-order motion model. 
    /// A zero-order motion model assumes that the camera stays where it was. At each timestep it moves from its location with a Gaussian profile.
    /// The dimension of the state vector is the standard seven of position - nothing
    /// else is stored (that's why it's zero order!)
    /// </summary>
    public class ZeroOrder_ThreeD_Motion_Model : ThreeD_Motion_Model
    {

        // Constants
        // May need a lot of tweaking!
        // Standard deviation of linear acceleration
        // Standard deviation of position change (in ms^{-1})
        public float SD_A_component_filter;
        // Standard deviation of linear velocity, for simulation.
        public float SD_A_component;
        // Standard deviation of angular acceleration
        // Standard deviation of angular velocity (in rad s^{-1})
        public float SD_alpha_component_filter;
        // Standard deviation of angular velocity, for simulation.
        public float SD_alpha_component;

        /// <summary>
        /// Constructor. Sets standard deviations of linear velocity (6ms^{-1}) and angular acceleration (4rads^{-1}).
        /// </summary>
        public ZeroOrder_ThreeD_Motion_Model()
            : base(7, 3, "ZEROORDER_THREED")
        {
            SD_A_component_filter = 12.0f; // m s^-1
            SD_A_component = 6.0f; // for simulation
            SD_alpha_component_filter = 4.0f;  // rad s^-1
            SD_alpha_component = 4.0f; // for simulation
        }

        /// <summary>
        /// Calculate the new state. Calculates  f_v  as a function of the old state  x_v , control vector 
        ///  u  and time interval  \Delta t , and also the Jacobian  \partial f_v / \partial x_v  
        /// </summary>
        /// <param name="xv"></param>
        /// <param name="v"></param>
        /// <param name="d"></param>
        public override void func_fv_and_dfv_by_dxv(Vector xv, Vector v, float d)
        {
            // The new state is just the old state!
            fvRES = new Vector(xv);

            // The Jacobian is just the identity
            dfv_by_dxvRES.SetIdentity();
        }

        /// <summary>
        /// Form the covariance matrix  Q  of the process noise associated with  x_v .
        /// </summary>
        /// <param name="xv"></param>
        /// <param name="v"></param>
        /// <param name="delta_t"></param>
        public override void func_Q(Vector xv, Vector v, float delta_t)
        {
            // Fill noise covariance matrix Pnn: this is the covariance of 
            // the noise vector (V)
            //                  (Omega)
            // that gets added to the state. 
            // Form of this could change later, but for now assume that 
            // V and Omega are independent, and that each of their components is
            // independent... 
            float linear_velocity_noise_variance =
                       SD_A_component_filter * SD_A_component_filter * delta_t * delta_t;
            float angular_velocity_noise_variance =
                       SD_alpha_component_filter * SD_alpha_component_filter * delta_t * delta_t;

            // Independence means that the matrix is diagonal
            MatrixFixed Pnn = new MatrixFixed(6, 6);
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

            // Start by zeroing
            MatrixFixed dxnew_by_dn = new MatrixFixed(7, 6);
            dxnew_by_dn.Fill(0.0f);

            // The translation part is just I \Delta t
            MatrixFixed Temp33A = new MatrixFixed(3, 3);
            Temp33A.SetIdentity();
            Temp33A *= delta_t;
            dxnew_by_dn.Update(Temp33A, 0, 0);

            // qnew = q x \Omega \Deltat
            // dqnew_by_d\Omega = dqnew_by_d\Omega\Delta t . d\Omega\Delta t_by_d\Omega

            // Get the first part
            Vector qRXYZ = xv.Extract(4, 3);
            Quaternion qold = new Quaternion();
            qold.SetRXYZ(qRXYZ);
            MatrixFixed Temp44A = MatrixFixed.dq3_by_dq1(qold);

            // Use function below for dqwt_by_dOmega
            Vector Omega = new Vector(3);
            Omega.Fill(SD_alpha_component_filter);

            MatrixFixed Temp43A = new MatrixFixed(4, 3);
            dqomegadt_by_domega(new Vector3D(Omega), delta_t, Temp43A);
            // Multiply them together
            MatrixFixed Temp43B = Temp44A * Temp43A;
            // And then plug into Jacobian
            dxnew_by_dn.Update(Temp43B, 3, 3);

            // Finally do Q = dxnew_by_dn . Pnn . dxnew_by_dnT
            QxRES.Update(dxnew_by_dn * Pnn * dxnew_by_dn.Transpose());

        }

        /// <summary>
        /// Extract the position and orientation from the state vector (they are the same!).
        /// </summary>
        /// <param name="xv"></param>
        public override void func_xp(Vector xv)
        {
            xpRES = xv;
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
        /// Not implemented yet.
        /// </summary>
        /// <param name="v"></param>
        /// <param name="v2"></param>
        /// <param name="d"></param>
        public override void func_fv_noisy(Vector v, Vector v2, float d)
        {
            // TODO: Implement this.
        }

        /// <summary>
        /// Redefine x_v and its Jacobian for a new co-ordinate frame. Not implemented yet!
        /// </summary>
        /// <param name="v"></param>
        /// <param name="v2"></param>
        public override void func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef(
                        Vector v, Vector v2)
        {
            Debug.WriteLine("func_xvredef... not yet implemented for ZeroOrder_ThreeD_Motion_Model" +
                            "... though surely it's not too hard?");
            //assert(0);
        }

        /// <summary>
        /// Normalise the state vector: since quaternion is redundant we sometimes
        /// need to enforce that it stays with magnitude 1.
        /// </summary>
        /// <param name="xv"></param>
        public override void func_xvnorm_and_dxvnorm_by_dxv(Vector xv)
        {
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
