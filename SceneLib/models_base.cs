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
    /// <summary>
    /// A class describing the movement of a robot or sensor platform.
    /// </summary>
    public class Motion_Model
    {
        /// <summary>
        /// Base-class constructor. This should be called in the constructor of any 
        /// derived class.
        /// </summary>
        /// <param name="position_state_size">The number of dimensions in the position state</param>
        /// <param name="state_size">The total state size</param>
        /// <param name="control_size">The control state size</param>
        /// <param name="m_m_d_t">A string representing the motion model dimensionality type </param>
        /// <param name="m_m_t">A unique string representing this particular motion model type</param>
        public Motion_Model(uint position_state_size, 
			   uint state_size, 
			   uint control_size, 
			   String m_m_d_t, String m_m_t)
        {
            POSITION_STATE_SIZE = position_state_size;
            STATE_SIZE = state_size;
            CONTROL_SIZE = control_size;
            motion_model_dimensionality_type = m_m_d_t;
            motion_model_type = m_m_t;

            fvRES = new Vector(STATE_SIZE);
            xpRES = new Vector(POSITION_STATE_SIZE);
            fv_noisyRES = new Vector(STATE_SIZE);
            xvredefRES = new Vector(STATE_SIZE);
            xvnormRES = new Vector(STATE_SIZE);
            zeroedxvRES = new Vector(STATE_SIZE);
            xpredefRES = new Vector(POSITION_STATE_SIZE);

            dfv_by_dxvRES = new MatrixFixed(STATE_SIZE, STATE_SIZE);
            QxRES = new MatrixFixed(STATE_SIZE, STATE_SIZE);
            dxp_by_dxvRES = new MatrixFixed(POSITION_STATE_SIZE, STATE_SIZE);           
            dxvredef_by_dxvRES = new MatrixFixed(STATE_SIZE, STATE_SIZE);
            dxvredef_by_dxpdefRES = new MatrixFixed(STATE_SIZE, POSITION_STATE_SIZE);
            dxvnorm_by_dxvRES = new MatrixFixed(STATE_SIZE, STATE_SIZE);
            dzeroedxv_by_dxvRES = new MatrixFixed(STATE_SIZE, STATE_SIZE);
            dxpredef_by_dxpRES = new MatrixFixed(POSITION_STATE_SIZE, POSITION_STATE_SIZE);
            dxpredef_by_dxpdefRES = new MatrixFixed(POSITION_STATE_SIZE, POSITION_STATE_SIZE);
        }

        /// <summary>
        /// Calculate the new state. Calculates the new state \f$ f_v \f$ as a function 
        /// of the old state \f$ x_v \f$, the control vector \f$ u \f$ and time interval 
        /// \f$ \Delta t \f$, and also calculates the Jacobian \f$ \partial f_v / 
        /// \partial x_v \f$. 
        /// This function just applies the process equations (without noise).
        /// </summary>
        /// <param name="?"></param>
        public virtual void func_fv_and_dfv_by_dxv(Vector xv, Vector u, float delta_t) {}

        /// <summary>
        /// Form the covariance matrix Q of the process noise associated with 
        /// f_v. Do this by forming the covariance matrix of the noise, 
        /// P_{nn}and the Jacobian \frac{\partial f_v}{n} and then 
        /// calculating Q = \frac{\partial f_v}{n} P_{nn}
        /// \frac{\partial f_v}{n}^T.
        /// </summary>
        /// <param name="xv"></param>
        /// <param name="u"></param>
        /// <param name="delta_t"></param>
        public virtual void func_Q(Vector xv, Vector u, float delta_t)
        {
        }

        /// <summary>
        /// Extract the position and orientation from the state vector.
        /// </summary>
        /// <param name="xv"></param>
        public virtual void func_xp(Vector xv)
        {
        }

        /// <summary>
        /// Calculate Jacobian for func_xp. (This is just the identity in this case.)
        /// </summary>
        /// <param name="?"></param>
        public virtual void func_dxp_by_dxv(Vector xv)
        {
        }

        /// <summary>
        /// A noisy version of the process equation func_fv_and_dfv_by_dxv() used in 
        /// simulation. Usually, this simply perturbs x_v with Gaussian noise 
        /// and sends it through func_fv_and_dfv_by_dxv().
        /// </summary>
        /// <param name="xv_true"></param>
        /// <param name="u_true"></param>
        /// <param name="delta_t"></param>
        public virtual void func_fv_noisy(Vector xv_true, 
                             Vector u_true,
                             float delta_t)
        {
        }

        /// <summary>
        /// Redefine x_v and its Jacobian for a new co-ordinate frame.
        /// </summary>
        /// <param name="xv"></param>
        /// <param name="xpdef"></param>
        public virtual void func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef  
            (Vector xv, Vector xpdef)
        {
        }

  //******These virtual functions are optional; not pure virtual functions*****
  //**********and they have default null instances. Define if necessary.*******

        // Normalise state vector if there is a redundant representation:
        // Default null version
        public virtual void func_xvnorm_and_dxvnorm_by_dxv(Vector xv)
        {
            xvnormRES.Update(xv);
            dxvnorm_by_dxvRES.SetIdentity();
        }

  //******This function is defined in models base.cc, and should only be*******
  //***********************redefined in special cases**************************

        // Zero the axes at the current state: i.e. redefine axes such that 
        // the current estimate's position state becomes the new origin
        public virtual void func_zeroedxv_and_dzeroedxv_by_dxv(Vector xv)
        {
            func_xp(xv);

            func_xvredef_and_dxvredef_by_dxv_and_dxvredef_by_dxpdef(xv, xpRES);

            zeroedxvRES.Update(xvredefRES);

            // This is a bit subtle: we need to know dzeroedxv_by_dxv,
            // and we know the two Jacobians dxvredef_by_dxv and dxvredef_by_dxpdef
            // --- we must remember that xv and xpdef are not independent though
            func_dxp_by_dxv(xv);

            MatrixFixed Temp_SS1 = dxvredef_by_dxpdefRES * dxp_by_dxvRES;
            dzeroedxv_by_dxvRES = Temp_SS1 + dxvredef_by_dxvRES;
        }

  //******These functions are defined in the dimension model types below*******
  //****and therefore do not need to be individually defined for each model****

        /// <summary>
        /// How to adjust the position state if the coordinate frame is redefined so 
        /// that xpdef (in position coordinates) becomes the new origin
        /// </summary>
        /// <param name="xp"></param>
        /// <param name="xpdef"></param>
        public virtual void func_xpredef_and_dxpredef_by_dxp_and_dxpredef_by_dxpdef  
            (Vector xp, Vector xpdef)
        {
        }

        /// <summary>
        /// Pose parameters: 3D position and rotation matrix for graphics, etc.
        /// Defined for each dimensionality
        /// </summary>
        /// <param name="xv"></param>
        public virtual void func_xpose_and_Rpose(Vector xv)
        {
        }

        // Test whether a position state lies withing a bounding box
        public virtual bool bounding_box_test(Vector xp1, Vector xp2, Vector xptest)
        {
            return (false);
        }

        /// <summary>
        /// Read the initial state vector and covariance from a stream. 
        /// Since state \f$ x_v \f$ and covariance \f$ P_{xx} \f$ are not stored in 
        /// the class, these are passed by reference to be filled in by this function.
        /// </summary>
        /// <param name="stream"></param>
        /// <param name="initial_xv"></param>
        /// <param name="initial_Pxx"></param>
        public void read_initial_state(BinaryReader stream,
                                       ref Vector initial_xv,
                                       ref MatrixFixed initial_Pxx)
        {
            // Make sure the vector and matrix are the correct sizes
            initial_xv.Resize(STATE_SIZE);
            initial_Pxx.Resize(STATE_SIZE, STATE_SIZE);

            initial_xv.Fill(0.0f);
            initial_Pxx.Fill(0.0f);

            for (int r = 0; r < STATE_SIZE; r++)
                initial_xv[r] = stream.ReadSingle();

            for (int r = 0; r < STATE_SIZE; r++)
                for (int c = 0; c < STATE_SIZE; c++)
                    initial_Pxx[r, c] = stream.ReadSingle();
        }

        /// <summary>
        /// Read the initial state vector and covariance from the settings class.
        /// Since state \f$ x_v \f$ and covariance \f$ P_{xx} \f$ are not stored in 
        /// the class, these are passed by reference to be filled in by this function.
        /// </summary>
        /// <param name="settings"></param>
        /// <param name="initial_xv"></param>
        /// <param name="initial_Pxx"></param>
        public void read_initial_state(Settings settings, 
                                       ref Vector initial_xv,
                                       ref MatrixFixed initial_Pxx)
        {
            ArrayList values;

            // Check that the motion model is correct
            values = settings.get_entry("InitialState", "MotionModel");
            if ((String)values[0] != motion_model_type)
            {
                Debug.WriteLine("Attempted to read an initial state with a motion model of type " +
                                motion_model_type + " where the initialisation data in the [InitialState] section" +
                                " reports the type " + settings.get_entry("InitialState", "MotionModel") + ".");
                //throw Scene::InitialisationError(error.str());
            }

            // Make sure the vector and matrix are the correct sizes
            initial_xv = new Vector(STATE_SIZE);
            initial_Pxx = new MatrixFixed(STATE_SIZE, STATE_SIZE);

            //initial_xv.Resize(STATE_SIZE);
            //initial_Pxx.Resize(STATE_SIZE, STATE_SIZE);

            initial_xv.Fill(0.0f);
            initial_Pxx.Fill(0.0f);

            values = settings.get_entry("InitialState", "xv");
            String xv_stream = (String)values[0];
            initial_xv.ReadASCII(xv_stream);

            values = settings.get_entry("InitialState", "Pxx");
            String Pxx_stream = (String)values[0];
            initial_Pxx.ReadASCII(Pxx_stream);
        }

        /// <summary>
        /// Read the parameters of this motion model from the settings. If this
        /// function is overloaded, you should call the base class read_parameters in
        /// the course of your function.
        /// </summary>
        /// <param name="settings"></param>
        public virtual void read_parameters(Settings settings) {}


        public const float INCREMENT_SIZE = 0.00000001f;
        // The number of parameters to minimally represent position in this
        // dimension
        public uint POSITION_STATE_SIZE; 
        // The number of parameters we will actually use in the state vector (allows
        // for more generality) 
        public uint STATE_SIZE;
        // Number of parameters in control vector 
        public uint CONTROL_SIZE;
        // String identifier for the number of dimensions
        public String motion_model_dimensionality_type; 
        // String identifier for this particular motion model
        public String motion_model_type;

        //****The following virtual functions must be supplied by a derived class****

        protected Vector fvRES;
        protected Vector xpRES;
        protected Vector fv_noisyRES;
        protected Vector xvredefRES;
        protected Vector xvnormRES;
        protected Vector zeroedxvRES;
        protected Vector xpredefRES;
        protected Vector xposeRES;

        protected MatrixFixed dfv_by_dxvRES, QxRES, dxp_by_dxvRES, dxvredef_by_dxvRES, 
                              dxvredef_by_dxpdefRES, dxvnorm_by_dxvRES, dzeroedxv_by_dxvRES, 
                              dxpredef_by_dxpRES, dxpredef_by_dxpdefRES, RposeRES;

 
        // Getters
        public Vector get_fvRES() {return fvRES;} 
        public Vector get_xpRES() {return xpRES;} 
        public Vector get_fv_noisyRES() {return fv_noisyRES;} 
        public Vector get_xvredefRES() {return xvredefRES;} 
        public Vector get_xvnormRES() {return xvnormRES;} 
        public Vector get_zeroedxvRES() {return zeroedxvRES;} 
        public Vector get_xpredefRES() {return xpredefRES;} 
        public Vector get_xposeRES() {return xposeRES;} 

        public MatrixFixed get_dfv_by_dxvRES() {return dfv_by_dxvRES;} 
        public MatrixFixed get_QxRES() {return QxRES;} 
        public MatrixFixed get_dxp_by_dxvRES() {return dxp_by_dxvRES;} 
        public MatrixFixed get_dxvredef_by_dxvRES() {return dxvredef_by_dxvRES;} 
        public MatrixFixed get_dxvredef_by_dxpdefRES() {return dxvredef_by_dxpdefRES;} 
        public MatrixFixed get_dxvnorm_by_dxvRES() {return dxvnorm_by_dxvRES;} 
        public MatrixFixed get_dzeroedxv_by_dxvRES() {return dzeroedxv_by_dxvRES;} 
        public MatrixFixed get_dxpredef_by_dxpRES() {return dxpredef_by_dxpRES;} 
        public MatrixFixed get_dxpredef_by_dxpdefRES() {return dxpredef_by_dxpdefRES;} 
        public MatrixFixed get_RposeRES() {return RposeRES;} 
    }

    /// <summary>
    /// General 3D motion model.
    /// Position state (standard representation) is: 
    /// {matrix}x & y & z & q_0 & q_1 & q_2 & q_3 {matrix})^T 
    /// </summary>
    public class ThreeD_Motion_Model : Motion_Model
    {
        /// <summary>
        /// Constructor.
        /// </summary>
        /// <param name="state_size">The total number of parameters in the model state</param>
        /// <param name="control_size">The number of parameters in the control state</param>
        /// <param name="m_m_t">A unique string representing this particular motion model type</param>
        public ThreeD_Motion_Model(uint state_size, 
                    uint control_size,
                    String m_m_t)
            : base(7, state_size, control_size, "THREED", m_m_t) 
        {
        }

        public override void func_xpredef_and_dxpredef_by_dxp_and_dxpredef_by_dxpdef  
                    (Vector xp, Vector xpdef)
        {
            // Split position state into cartesian and quaternion

            // r0, q0: position in original frame
            func_r(xp);
            func_q(xp);
            Vector3D r0 = rRES;
            Quaternion q0 = qRES;

            // rn, qn: definition of new frame
            func_r(xpdef);
            func_q(xpdef);
            Vector3D rn = rRES;
            Quaternion qn = qRES;

            Quaternion qnbar = qn.Conjugate();

            // Calculate new cartesian part of xp
            Vector3D Temp31a = new Vector3D(r0 - rn); // Temp31a is r0 - rn
            RotationMatrix Temp33a = qnbar.RotationMatrix();
            Vector3D Temp31b = new Vector3D(Temp33a * Temp31a);

            // Copy into xpredefRES
            xpredefRES[0] = Temp31b.GetX();
            xpredefRES[1] = Temp31b.GetY();
            xpredefRES[2] = Temp31b.GetZ();

            // Calculate new quaternion part of xp
            Quaternion Tempqa = qnbar.Multiply(q0);

            // Copy into xpredefRES
            Vector vecTempqa = Tempqa.GetRXYZ();
            xpredefRES.Update(vecTempqa, 3);

            // Form Jacobian dxpredef_by_dxpRES
            dxpredef_by_dxpRES.Fill(0.0f);
            dxpredef_by_dxpRES.Update(Temp33a, 0, 0);

            MatrixFixed Temp44a = MatrixFixed.dq3_by_dq1(qnbar);
            dxpredef_by_dxpRES.Update(Temp44a, 3, 3);

            // Form Jacobian dxpredef_by_dxpdefRES
            dxpredef_by_dxpdefRES.Fill(0.0f);
            //Temp33a *= -1.0f;
            Temp33a = (RotationMatrix)(Temp33a  * - 1.0f);
            dxpredef_by_dxpdefRES.Update(Temp33a, 0, 0);

            Temp44a = MatrixFixed.dq3_by_dq2(q0);

            MatrixFixed Temp44b = MatrixFixed.dqbar_by_dq();
            MatrixFixed Temp44c = Temp44a * Temp44b;
            dxpredef_by_dxpdefRES.Update(Temp44c, 3, 3);

            // Top right corner of this Jacobian is tricky because we have to
            // differentiate a rotation matrix
            // Uses function that does this in bits.cc
            // Build this corner in Temp34a
            //MatrixFixed temp = new MatrixFixed(3,4);
            MatrixFixed Temp34a = MatrixFixed.dRq_times_a_by_dq(qnbar, Temp31a.GetVNL3());

            // So far we have drN_by_dqnbar; want _by_dqn
            MatrixFixed Temp34b = Temp34a * Temp44b;

            // Finally copy into result matrix
            dxpredef_by_dxpdefRES.Update(Temp34b, 0, 3);

            // cout << "dxpredef_by_dxpdefRES" << dxpredef_by_dxpdefRES;
        }

        public override void func_xpose_and_Rpose(Vector xv)
        {
            func_xp(xv);

            // Turn xp vector (x, y, z, q0, q1, q2, q3) into pose
            xposeRES = xpRES.Extract(3, 0);

            Vector qvec = xpRES.Extract(4, 3);
            Quaternion q = new Quaternion();
            q.SetRXYZ(qvec);

            RposeRES = q.RotationMatrix();
        }

        public override bool bounding_box_test(Vector xp1, Vector xp2, Vector xptest)
        {
            // ThreeD position state xp (x, y, z, q0, q1, q2, q3)
            // Only test x, y, z
            return (xptest[0] >= xp1[0] &&
                    xptest[0] <  xp2[0] &&
                    xptest[1] >= xp1[1] &&
                    xptest[1] <  xp2[1] &&
                    xptest[2] >= xp1[2] &&
                    xptest[2] <  xp2[2]);
        }

        /// <summary>
        /// Extract the cartesian translation part of the state position vector. 
        /// </summary>
        /// <param name="xp"></param>
        public void func_r(Vector xp)
        {
            rRES.Set(xp[0], xp[1], xp[2]);
        }

        public void func_dr_by_dxp(Vector a)
        {
            dr_by_dxpRES.Fill(0.0f);

            dr_by_dxpRES.Put(0, 0, 1.0f);
            dr_by_dxpRES.Put(1, 1, 1.0f);
            dr_by_dxpRES.Put(2, 2, 1.0f);
        }

        /// <summary>
        /// Extract the rotation (quaternion) part of the state position vector. 
        /// </summary>
        /// <param name="?"></param>
        public void func_q(Vector xp)
        {
            Vector qvec = xp.Extract(4, 3);
            qRES.SetRXYZ(qvec);
        }

        public void func_dq_by_dxp(Vector a)
        {
            dq_by_dxpRES.Fill(0.0f);
  
            dq_by_dxpRES.Put(0, 3, 1.0f);
            dq_by_dxpRES.Put(1, 4, 1.0f);
            dq_by_dxpRES.Put(2, 5, 1.0f);
            dq_by_dxpRES.Put(3, 6, 1.0f);
        }


        // Results matrices       
        protected Vector3D rRES = new Vector3D(0,0,0);
        protected MatrixFixed dr_by_dxpRES = new MatrixFixed(3,7);
        protected Quaternion qRES = new Quaternion();
        protected MatrixFixed dq_by_dxpRES = new MatrixFixed(4,7);

        // Getters
        public Vector3D get_rRES() {return rRES;} 
        public Quaternion get_qRES() {return qRES;}
    }

    /// <summary>
    /// A 2D planar model.
    /// The position state (minimal representation) is:(\begin{matrix} z & x & \phi \end{matrix})\f$. For historical reasons, `foward' is the z-axis, and 'left' the x-axis. The angle \phi is anti-clockwise orientation relative to the z-axis, and is restricted to the range-\pi < \phi \leq \pi.
    /// </summary>
    public class TwoD_Motion_Model : Motion_Model
    {
    
        public TwoD_Motion_Model(uint state_size, uint control_size, String m_m_t)
                        : base(3, state_size, control_size, "TWOD", m_m_t) 
        {
        }

        public override void func_xpredef_and_dxpredef_by_dxp_and_dxpredef_by_dxpdef
                    (Vector xp, Vector xpdef)
        {
            float zdef = xpdef[0];
            float xdef = xpdef[1];
            float phidef = xpdef[2];
            float cphidef = (float)Math.Cos(phidef);
            float sphidef = (float)Math.Sin(phidef);
            float z0 = xp[0];
            float x0 = xp[1];
            float phi0 = xp[2];

            Vector a = new Vector(3);
            a[0] = sphidef * (x0 - xdef) + cphidef * (z0 - zdef);
            a[1] = cphidef * (x0 - xdef) - sphidef * (z0 - zdef);
            a[2] = phi0 - phidef;
            
            xpredefRES.CopyIn(a);
            xpredefRES[2] = MatrixFixed.and_pi_range(xpredefRES[2]);

            Vector b = new Vector(9);
            b[0] = cphidef;
            b[1] = sphidef;
            b[2] = 0.0f;
            b[3] = -sphidef;
            b[4] = cphidef;
            b[5] = 0.0f;
            b[6] = 0.0f;
            b[7] = 0.0f;
            b[8] = 1.0f;

            dxpredef_by_dxpRES.CopyIn(b);

            Vector c = new Vector(9);
            c[0] = -cphidef;
            c[1] = -sphidef;
            c[2] = (x0 - xdef) * cphidef - (z0 - zdef) * sphidef;
            c[3] = sphidef;
            c[4] = -cphidef;
            c[5] = -(x0 - xdef) * sphidef - (z0 - zdef) * cphidef;
            c[6] = 0.0f;
            c[7] = 0.0f;
            c[8] = -1.0f;

            dxpredef_by_dxpdefRES.CopyIn(c);
        }

        public override void func_xpose_and_Rpose(Vector xv)
        {
            func_xp(xv);

            // Turn xp vector (z, x, phi) into pose
            xposeRES[0] = xpRES[1]; 
            xposeRES[1] = 0.0f; 
            xposeRES[2] = xpRES[0];

            Vector a = new Vector(9);

            a[0] = (float)Math.Cos(xpRES[2]);
            a[1] = 0.0f;
            a[2] = (float)Math.Sin(xpRES[2]);
            a[3] = 0.0f;
            a[4] = 1.0f;
            a[5] = 0.0f;
            a[6] = -(float)Math.Sin(xpRES[2]);
            a[7] = 0.0f;
            a[8] = (float)Math.Cos(xpRES[2]);
            // Rpose is MLR 
            RposeRES.CopyIn(a);
        }


        public override bool bounding_box_test(Vector xp1, Vector xp2, Vector xptest)
        {
            // TwoD position state xp (z, x, phi)
            // Only test z and x
            return (xptest[0] >= xp1[0] &&
                    xptest[0] <  xp2[0] &&
                    xptest[1] >= xp1[1] &&
                    xptest[1] <  xp2[1]);
        }
    }

    /// <summary>
    /// A 1D model.
    /// The position state (minimal representation) is:( z )\f$. In ID, the position state describes the displacement along a straight line.
    /// </summary>
    public class OneD_Motion_Model : Motion_Model
    {
        public OneD_Motion_Model(uint state_size, uint control_size, String m_m_t)
                : base(1, state_size, control_size, "ONED", m_m_t) 
        {}

        public override void func_xpredef_and_dxpredef_by_dxp_and_dxpredef_by_dxpdef
            (Vector xp, Vector xpdef)
        {
            float zdef = xpdef[0];
            float z0 = xp[0];

            xpredefRES[0] = z0 - zdef;

            dxpredef_by_dxpRES[0, 0] = 1.0f;

            dxpredef_by_dxpdefRES[0, 0] = -1.0f;
        }

        public override void func_xpose_and_Rpose(Vector xv)
        {
            func_xp(xv);

            // Turn xp vector (z) into pose
            xposeRES[0] = 0.0f; 
            xposeRES[1] = 0.0f; 
            xposeRES[2] = xpRES[0];

            RposeRES.SetIdentity();
            // Rpose is MLR 
        }

        public override bool bounding_box_test(Vector xp1, 
                                      Vector xp2,
                                      Vector xptest)
        {
            return(xptest[0] >= xp1[0] &&
                   xptest[0] <  xp2[0]);
        }

    }

    /// <summary>
    /// Class to create an instance of a motion model class given its type string.
    /// This is an abstract base class - writers of specific motion models should 
    /// derive their own version able of providing thir own motion models. 
    /// </summary>
    public class Motion_Model_Creator
    {
        /// Constructor. Does nothing.
        public Motion_Model_Creator() {} 

        /// Returns an instance of the requested motion model (usually by using
        /// the new operator).
        public virtual Motion_Model create_model(String type) {return(null);}
    }

    /// <summary>
    /// this class actually does nothing, but acts as the base class for 
    /// Wide_Camera_Point_Feature_Measurement_Model, and is used to get around a problem
    /// with multiple inherritance in the original code
    /// </summary>
    public class Camera_Feature_Measurement_Model_base
    {
    }


    /// <summary>
    /// Base class for classes that describe the behaviour of feature
    /// measurements.  This base class just contains basic functionality
    /// (graphics, transformation, etc.). The derived classes,
    /// Partially_Initialised_Feature_Measurement_Model and
    /// Fully_Initialised_Feature_Measurement_Model, and classes derived from
    /// these, provide the main behaviour.
    /// 
    /// This class holds no permanent state itself - it just provides functions for
    /// manipulating an externally-held state and its measurements. The main 
    /// state vectors and covariances are:
    /// y_i  - The feature state (e.g. a 3D position vector).
    /// P_{y_iy_i} - The covariance of the feature
    /// y_i  h_i - A measurement of the feature state (e.g.the location of the feature in an image).
    /// R_i - The covariance of the measurement 
    /// h_i - (e.g. uncertainty due to the image resolution).
    /// x_p - The robot position state (e.g. its 3D position and orientation).
    /// S_i - The innovation covariance: the combination of the robot state uncertainty, 
    /// the feature state uncertainty and the measurement uncertainty (e.g. the overall 
    /// uncertainty in the feature's image location).
    /// </summary>
    public class Feature_Measurement_Model
    {
        // used to overcome multiple inherritance problem
        public Camera_Feature_Measurement_Model_base wide_model = null;

        /// <summary>
        /// Constructor.
        /// </summary>
        /// <param name="measurement_size">The number of parameters representing a measurement of the feature</param>
        /// <param name="feature_state_size">The number of parameters to repreent the state of the feature.</param>
        /// <param name="graphics_state_size">The number of parameters to represent an abstraction of the feature for use in a graphical display</param>
        /// <param name="m_m">The robot motion model. Needed to understand and use the robot position statex_pwhen measuring features</param>
        /// <param name="f_t">The name for this feature measurement model (set by the derived class).</param>
        /// <param name="f_g_t">The name for this type of feature, so that it can be drawn (set by the derived class)</param>
        /// <param name="f_i_f">Is this model for fully-initialised features? (if <code>false</code> it is for partially-initialised features.). Set by the derived class.</param>
        public Feature_Measurement_Model(
                    uint measurement_size, 
				    uint feature_state_size,
		       	    uint graphics_state_size,
			        Motion_Model m_m, 
				    String f_t, String f_g_t,
                    bool f_i_f)
        {
            MEASUREMENT_SIZE = measurement_size; 
            FEATURE_STATE_SIZE = feature_state_size;
            GRAPHICS_STATE_SIZE = graphics_state_size;
            feature_type = f_t;
            feature_graphics_type = f_g_t;
            fully_initialised_flag = f_i_f;
            _motion_model = m_m;

            // Allocate matrices for storage of results
            yigraphicsRES = new Vector(GRAPHICS_STATE_SIZE);
            PyiyigraphicsRES = new MatrixFixed(GRAPHICS_STATE_SIZE, GRAPHICS_STATE_SIZE);
            zeroedyigraphicsRES = new Vector(GRAPHICS_STATE_SIZE);
            PzeroedyigraphicsRES = new MatrixFixed(GRAPHICS_STATE_SIZE, GRAPHICS_STATE_SIZE);
            RiRES = new MatrixFixed(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
            SiRES = new MatrixFixed(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
            zeroedyiRES = new Vector(FEATURE_STATE_SIZE);
            dzeroedyi_by_dxpRES = new MatrixFixed(FEATURE_STATE_SIZE, 
			     get_motion_model().POSITION_STATE_SIZE);
            dzeroedyi_by_dyiRES = new MatrixFixed(FEATURE_STATE_SIZE, FEATURE_STATE_SIZE);
        }


        /// <summary>
        /// Calculate the innovation covariance. This is the overall measurement
        /// uncertainty in the feature, a combination of the uncertainty in the robot
        /// state, the feature state, and the measurement uncertainty. This calculation is
        /// generic to all feature measurement models, but could be over-ridden if necessary
        /// (e.g. for an efficient implementation). The innovation covariance is given by
        /// \f[S_i = \frac{\partial h_i}{\partial x_v} P_{xx}
        /// \frac{\partial h_i}{\partial x_v}^T
        /// + \frac{\partial h_i}{\partial x_v} P_{xy_i}
        /// \frac{\partial h_i}{\partial y_i}^T
        /// + \frac{\partial h_i}{\partial y_i} P_{y_i x}
        /// \frac{\partial h_i}{\partial x_v}^T
        /// + \frac{\partial h_i}{\partial y_i} P_{y_i y_i}
        /// \frac{\partial h_i}{\partial y_i}^T
        /// + R_i ]
        /// where R_i is the noise covariance of measurements (usually assumed to
        /// be diagonal with magnitude determined by the image resolution).
        /// </summary>
        /// <param name="Pxx">The covariance P_{xy_i} between the robot state x_v and the feature state y_i .</param>
        /// <param name="Pxyi">The covariance of the feature, P_{y_iy_i} </param>
        /// <param name="Pyiyi">The Jacobian \frac{\partial h_i}{\partial x_v} between the feature measurement h_i and the robot state x_v </param>
        /// <param name="dhi_by_dxv">The Jacobian  \frac{\partial h_i}{\partial y_i} between the feature measurement h_i and the feature state y_i </param>
        /// <param name="dhi_by_dyi">The innovation covariance R_i</param>
        /// <param name="Ri"></param>
        public void func_Si(MatrixFixed Pxx, 
                            MatrixFixed Pxyi,
                            MatrixFixed Pyiyi,
                            MatrixFixed dhi_by_dxv,
                            MatrixFixed dhi_by_dyi,
                            MatrixFixed Ri)
        {
            // Zero SiRES and add bits on
            SiRES.Fill(0.0f);

            SiRES += dhi_by_dxv * Pxx * dhi_by_dxv.Transpose();
  
            MatrixFixed Temp_MM1 = dhi_by_dxv * Pxyi * dhi_by_dyi.Transpose();
            SiRES += Temp_MM1;

            SiRES += Temp_MM1.Transpose();

            SiRES += dhi_by_dyi * Pyiyi * dhi_by_dyi.Transpose();

            SiRES += Ri;
        }    

        // Constants: set in initialisation list for individual models
        /// The number of parameters representing a measurement of the feature.
        public uint MEASUREMENT_SIZE; 

        /// The number of parameters to represent the state of the feature
        public uint FEATURE_STATE_SIZE;

        /// The number of parameters to represent an abstraction of the feature 
        /// for use in a graphical display. (This will often be the same as 
        /// FEATURE_STATE_SIZE, but does not necessarily have to be.)
        public uint GRAPHICS_STATE_SIZE;
  
        /// What is the motion model associated with these feature measurements?
        public Motion_Model get_motion_model() {return _motion_model;}
  
        /// The name for this feature measurement model (set by the derived class).
        public String feature_type;
        /// feature_graphics_type The name for this type of feature, so that it 
        /// can be drawn (set by the derived class)
        public String feature_graphics_type;
        /// Is this model for fully-initialised features? 
        /// (if <code>false</code> it is for partially-initialised features.)
        public bool fully_initialised_flag;
        /// Is this model for fully-initialised features?
        /// (if <code>false</code> it is for partially-initialised features.)
        public bool fully_initialised() {return fully_initialised_flag;}
  
        // Where results will be stored after calls to functions
        protected Vector yigraphicsRES;
        protected Vector zeroedyigraphicsRES;
        protected Vector zeroedyiRES;
        protected MatrixFixed PyiyigraphicsRES, PzeroedyigraphicsRES, 
                              RiRES, SiRES, dzeroedyi_by_dxpRES, dzeroedyi_by_dyiRES;
        protected Motion_Model _motion_model;

        // Getters
        /// Get the value of the feature graphics state y_i^{graphics},
        /// calculated in func_yigraphics_and_Pyiyigraphics().
        public Vector get_yigraphicsRES() {return yigraphicsRES;} 
        /// Get the value of the feature graphics state if the robot were elsewhere,
        /// y_i^{zeroedgraphics}, calculated in
        /// func_zeroedyigraphics_and_Pzeroedyigraphics().
        public Vector get_zeroedyigraphicsRES() {return zeroedyigraphicsRES;} 
        /// Get the value of the feature state if the root were elsewhere,
        /// y_i^{zeroed}, calculated in
        /// func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi().
        public Vector get_zeroedyiRES() {return zeroedyiRES;} 

        /// Get the value of the feature graphics state covariance,
        /// P_{{y_i}{{y_i^{graphics}}}}, calculated in
        /// func_yigraphics_and_Pyiyigraphics().
        public MatrixFixed get_PyiyigraphicsRES() {return PyiyigraphicsRES;} 
        /// Get the value of the feature graphics state covariance, if the robot
        /// were elsewhere, P_{{y_i}{{y_i^{zeroedgraphics}}}},
        /// calculated in func_zeroedyigraphics_and_Pzeroedyigraphics().
        public MatrixFixed get_PzeroedyigraphicsRES() {return PzeroedyigraphicsRES;} 
        /// Get the value of the feature measurement covariance R_i,
        /// calculated earlier (e.g. by func_Ri()).
        public MatrixFixed get_RiRES() {return RiRES;} 
        // Get the value the innovation covariance (the overall image uncertainty
        // for this feature), S_i, calculated by func_Si().
        public MatrixFixed get_SiRES() {return SiRES;} 
        /// Get the value of the Jacobian
        /// \frac{\partial y_i^{zeroed}}{\partial x_p}, calculated
        /// by func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi().
        public MatrixFixed get_dzeroedyi_by_dxpRES() {return dzeroedyi_by_dxpRES;} 
        /// Get the value of the Jacobian
        /// \frac{\partial y_i^{zeroed}}{\partial y_i}, calculated
        /// by func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi().
        public MatrixFixed get_dzeroedyi_by_dyiRES() {return dzeroedyi_by_dyiRES;} 


        /// <summary>
        /// Take the state and covariance of the feature (\f$ y_i \f$ and
        /// \f$ P_{{y_i}{y_i}}\f$) and calculate its graphics state
        /// \f$ y_i^{graphics} \f$ and covariance
        /// \f$ P_{{y_i}{{y_i^{graphics}}}} \f$. (Sometimes actual state
        /// representation will have extra parameters, etc.)
        /// </summary>
        /// <param name="yi">The feature state \f$ y_i \f$</param>
        /// <param name="Pyiyi">The feature covariance \f$ P_{y_iy_i} \f$</param>
        public virtual void func_yigraphics_and_Pyiyigraphics(Vector yi, MatrixFixed Pyiyi) {}

        /// <summary>
        /// Calculate graphics state of feature relative to frame zeroed at
        /// current robot position. This is for drawing from a view centred
        /// at the robot with the right relative uncertainties. If the
        /// graphics state and the feature state are the same, this will do
        /// the same as func_zeroedyi() below, but sometimes they will be different.
        /// </summary>
        /// <param name="yi">The feature state \f$ y_i \f$</param>
        /// <param name="xv">The robot state \f$ x_v \f$</param>
        /// <param name="Pxx">The robot state covariance \f$ P_{xx} \f$</param>
        /// <param name="Pxyi">The covariance \f$ P_{xy_i} \f$ between the robot state \f$ x_v \f$ and the feature state \f$ y_i \f$.</param>
        /// <param name="Pyiyi">The covariance of the feature, \f$ P_{y_iy_i} \f$</param>
        public virtual void func_zeroedyigraphics_and_Pzeroedyigraphics(
                            Vector yi, Vector xv, MatrixFixed Pxx, MatrixFixed Pxyi, MatrixFixed Pyiyi) {}

        /// <summary>
        /// Calculate \f$ \vct{y}_i^{zeroed} \f$, the feature state as it would be
        /// if the origin was at position state \f$ \vct{x}_p \f$, and also
        /// calculates the relevant Jacobians \f$
        /// \partfrac{\vct{y}_i^{zeroed}}{\vct{x}_p} \f$ and \f$
        /// \partfrac{\vct{y}_i^{zeroed}}{\vct{y}_i} \f$.  This function
        /// is used as the first step in predicting measurement values.
        /// </summary>
        /// <param name="yi">The feature state \f$ \vct{y}_i \f$</param>
        /// <param name="xp">The position state \f$ \vct{x}_p \f$</param>
        public virtual void func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(Vector yi, Vector xp) {}

        /// <summary>
        /// Calculates the covariance \f$ R_i \f$ of the measurement noise for a measurement with state \f$ h_i \f$
        /// </summary>
        /// <param name="hi">The feature measurement \f$ h_i \f$</param>
        public virtual void func_Ri(Vector hi) {}

        /// <summary>
        /// Test a feature for visibility, given the robot position \f$ x_p
        /// \f$, the feature state \f$ y_i \f$, the original position state
        /// of the robot when that feature was first observed, \f$
        /// x_p^{orig}\f$, and the current predicted measurement \f$ h_i
        /// \f$. This is used in the active selection of measurements to
        /// decided whether the feature should be attempted to be measured
        /// by the robot at \f$ x_p \f$. A criterion for this should be
        /// defined on the relative positions; for example, success could
        /// only be expected from within a limited range of robot motion
        /// away from the position from which the feature was first seen and
        /// a template stored.
        /// </summary>
        /// <param name="xp">The robot position state \f$ x_p \f$</param>
        /// <param name="yi">The feature state \f$ y_i \f$</param>
        /// <param name="xp_orig">The robot position state when the feature was first observed, \f$ x_p^{orig} \f$</param>
        /// <param name="hi">The current predicted measurement, \f$ h_i \f$</param>
        /// <returns>zero on success, or any other value on failure.</returns>
        public virtual uint visibility_test(Vector xp, Vector yi, Vector xp_orig, Vector hi) {return(0);}

        /// <summary>
        /// Calculate the score for a feature based on its innovation
        /// covariance: how useful would making a measurement of this
        /// feature be? Generally, features with a large innovation
        /// covariance are the most useful, since it makes sense to make a
        /// measurement where the result is uncertain, rather than where it
        /// is possible to accurately predict the result.
        /// </summary>
        /// <param name="Si">The innovation covariance, \f$ S_i \f$</param>
        /// <returns></returns>
        public virtual float selection_score(MatrixFixed Si) {return(0.0f);}

        // Read the parameters of this feature measurement model from the settings.
        public virtual void read_parameters(Settings settings) {}


        /// <summary>
        /// Read the initial state for this feature from the Settings. This reads the
        /// state \vct{y}_i  and the robot position from which it was first
        /// observed, \vct{x}_p^{orig} . These are not stored in the class, so are
        /// returned as parameters.
        /// </summary>
        /// <param name="settings">The Settings class from which to read the data</param>
        /// <param name="yi">To be filled in with the initial feature state,  \vct{y}_i .</param>
        /// <param name="xp_orig">To be filled in with the robot position from which the feature was first observed,  \vct{x}_p^{orig} .</param>
        public void read_initial_state(Settings.Section section, Vector yi, Vector xp_orig)
        {
            ArrayList values;

            yi.Resize(FEATURE_STATE_SIZE);
            values = section.get_entry("yi");
            yi.ReadASCII((String)values[0]);

            uint size = get_motion_model().POSITION_STATE_SIZE;
            xp_orig.Resize(size);
            values = section.get_entry("xp_orig");
            xp_orig.ReadASCII((String)values[0]);
        }
    }

    /// <summary>
    /// Class for a feature that has been fully initialised and is therefore
    /// available for measurements. This class does not store any permanent state - it
    /// just provides helper functions to convert between different state vectors.
    /// The various state vectors and covariances are:
    /// y_i - The feature state (e.g. a 3D position vector).
    /// h_i - The measurement state of the feature (e.g. the projection of feature 
    /// position into the image.)
    /// z_i - An actual measurement of the feature state.
    /// nu_i - The innovation: the difference between the expected measurement 
    ///        h_i and the actual measurement z_i
    /// x_p - The robot position state. (e.g. its 3D position and orientation).
    /// </summary>
    public class Fully_Initialised_Feature_Measurement_Model : Feature_Measurement_Model 
    {
        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="measurement_size">The number of parameters representing a measurement of the feature.</param>
        /// <param name="feature_state_size">The number of parameters to repreent the state of the feature.</param>
        /// <param name="graphics_state_size">The number of parameters to represent an abstraction of the feature for use in a graphical display.</param>
        /// <param name="motion_model">The robot motion model. Needed to understand and use the robot position statex_pwhen measuring features.</param>
        /// <param name="feature_type">The name for this feature measurement model (set by the derived class).</param>
        /// <param name="feature_graphics_type">The name for this type of feature, so that it can be drawn (set by the derived class).</param>
        public Fully_Initialised_Feature_Measurement_Model(
            uint measurement_size,
            uint feature_state_size,
            uint graphics_state_size,
            Motion_Model motion_model,
            String feature_type,
            String feature_graphics_type)
            : base(measurement_size, 
                   feature_state_size,
                   graphics_state_size,
                   motion_model,
                   feature_type,
                   feature_graphics_type, true)
        {
            hiRES = new Vector(MEASUREMENT_SIZE);
            dhi_by_dxpRES = new MatrixFixed(MEASUREMENT_SIZE, 
                       motion_model.POSITION_STATE_SIZE);
            dhi_by_dyiRES = new MatrixFixed(MEASUREMENT_SIZE, FEATURE_STATE_SIZE);
            nuiRES = new Vector(MEASUREMENT_SIZE);
            hi_noisyRES = new Vector(MEASUREMENT_SIZE);
        }

        
        protected Vector hiRES;
        protected Vector nuiRES;
        protected Vector hi_noisyRES;
        protected MatrixFixed dhi_by_dxpRES, dhi_by_dyiRES;

        // Getters
        /// Get the value of the expected feature measurement h_i,
        /// calculated by func_hi_and_dhi_by_dxp_and_dhi_by_dyi().
        public Vector get_hiRES() {return hiRES;} 
        /// Get the value of the innovation \nu_i, calculated by
        /// func_nui().
        public Vector get_nuiRES() {return nuiRES;} 
        /// Get the value of the noisy measurement h_i^noisy, calculated
        /// by func_hi_noisy().
        public Vector get_hi_noisyRES() {return hi_noisyRES;} 

        /// Get the value of the Jacobian \frac{\partial h_i}{\partial x_p},
        /// calculated by func_hi_and_dhi_by_dxp_and_dhi_by_dyi().
        public MatrixFixed get_dhi_by_dxpRES() {return dhi_by_dxpRES;} 
        /// Get the value of \frac{\partial h_i}{\partial y_i},
        /// calculated by func_hi_and_dhi_by_dxp_and_dhi_by_dyi()
        public MatrixFixed get_dhi_by_dyiRES() {return dhi_by_dyiRES;} 


        /// <summary>
        /// The main measurement function. Calculates a measurement \f$ h_i
        /// \f$ from the feature state \f$ y_i \f$ and robot position state \f$
        /// x_p \f$ and also calculates the Jacobians \f$ \frac{\partial
        /// h_i}{\partial x_p} \f$ and \f$ \frac{\partial h_i}{\partial y_i}
        /// </summary>
        /// <param name="yi">The feature state \f$ y_i \f$</param>
        /// <param name="xp">The robot position state \f$ xp \f$</param>
        public virtual void func_hi_and_dhi_by_dxp_and_dhi_by_dyi(Vector yi, Vector xp) {}

        /// <summary>
        /// Calculates the innovation \f$ \nu_i \f$, the difference between
        /// the predicted measurement \f$ h_i \f$ and the actual measurement
        /// \f$ z_i \f$. This function should normally perform a simple
        /// subraction \f$ \nu_i = z_i - h_i \f$, but is left to be defined
        /// in case that is not the case. (For example if the measurement is
        /// an angle which needs normalising to lie between \f$ -\pi
        /// \rightarrow \pi \f$.)
        /// </summary>
        /// <param name="hi">The expected feature measurement \f$ h_i \f$</param>
        /// <param name="zi">The actual feature measurement \f$ z_i \f$</param>
        public virtual void func_nui(Vector hi, Vector zi) {}

        /// <summary>
        /// Noisy measurement for use in simulation, producing a measurement with random noise added.
        /// </summary>
        /// <param name="yi_true">The true feature state \f$ yi \f$</param>
        /// <param name="xp_true">The true robot position state \f$ x_p \f$</param>
        public virtual void func_hi_noisy(Vector yi_true, Vector xp_true) { }

    }


    /// <summary>
    /// Class for a feature that isn't fully initialised after just one
    /// measurement, and so needs further measurements. Typically, a
    /// partially-initalised feature has some known parameters (with Gaussian
    /// uncertainty), and some free parameters represented by other means (such as
    /// particles). This class does not store any permanent state - it just provides
    /// helper functions to convert between different state vectors.
    ///
    /// The various state vectors and covariances are:
    /// y_{pi} - The partial feature state (e.g. the parameters of a 3D line)
    /// h_{pi} - The measurement state of the partially-initialised feature 
    ///          (e.g. the location of the feature in the image.)
    /// h_i - An inital measurement of the feature, used to initialise the 
    ///       partially-initialised feature.
    /// R_i - The covariance of the measurement 
    /// h_i lambda - A vector containing values for the free parameters.
    /// y_{fi} - The fully-initalised feature state (which can be constructed given y_{pi}
    ///          and lambda
    /// x_p - The robot position state. (e.g. its 3D position and orientation).
    /// </summary>
    public class Partially_Initialised_Feature_Measurement_Model : Feature_Measurement_Model 
    {  
        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="measurement_size">The number of parameters representing a measurement of the feature.</param>
        /// <param name="feature_state_size">The number of parameters to repreent the state of the feature.</param>
        /// <param name="graphics_state_size">The number of parameters to represent an abstraction of the feature for use in a graphical display.</param>
        /// <param name="motion_model">The robot motion model. Needed to understand and use the robot position statex_pwhen measuring features.</param>
        /// <param name="feature_type">The name for this feature measurement model (set by the derived class).</param>
        /// <param name="feature_graphics_type">The name for this type of feature, so that it can be drawn (set by the derived class).</param>
        /// <param name="free_parameter_size">The number of parameters that should be left outside the main Gaussian-based representation and represented by other means, such as particles.</param>
        /// <param name="m_i_f_m_m">What feature measurement model will this turn into once it's fully initialised?</param>
        public Partially_Initialised_Feature_Measurement_Model(
            uint measurement_size,
            uint feature_state_size,
            uint graphics_state_size,
            Motion_Model motion_model,
            String feature_type,
            String feature_graphics_type,
            uint free_parameter_size,
            Feature_Measurement_Model m_i_f_m_m)
            : base(measurement_size, 
                   feature_state_size,
                   graphics_state_size,
                   motion_model,
                   feature_type,
                   feature_graphics_type, false)
            
        {
            FREE_PARAMETER_SIZE = free_parameter_size;
            more_initialised_feature_measurement_model = m_i_f_m_m;

            ypiRES = new Vector(FEATURE_STATE_SIZE);
            dypi_by_dxpRES = new MatrixFixed(FEATURE_STATE_SIZE, 
                        motion_model.POSITION_STATE_SIZE);
            dypi_by_dhiRES = new MatrixFixed(FEATURE_STATE_SIZE,
                        MEASUREMENT_SIZE);
            hpiRES = new Vector(MEASUREMENT_SIZE);
            dhpi_by_dxpRES = new MatrixFixed(MEASUREMENT_SIZE, 
                        motion_model.POSITION_STATE_SIZE);
            dhpi_by_dyiRES = new MatrixFixed(MEASUREMENT_SIZE, FEATURE_STATE_SIZE);
            yfiRES = new Vector(more_initialised_feature_measurement_model.FEATURE_STATE_SIZE);
            dyfi_by_dypiRES = new MatrixFixed(more_initialised_feature_measurement_model.FEATURE_STATE_SIZE, FEATURE_STATE_SIZE);
            dyfi_by_dlambdaRES = new MatrixFixed(more_initialised_feature_measurement_model.FEATURE_STATE_SIZE, FREE_PARAMETER_SIZE);
        }


        // The number of parameters that should be left outside the main
        // Gaussian-based representation and represented by other means,
        // such as particles. 
        public uint FREE_PARAMETER_SIZE;

        /// What feature measurement model will this turn into once it's
        /// fully initialised?
        public Feature_Measurement_Model more_initialised_feature_measurement_model;

        // Where results will be stored after calls to functions
        protected Vector ypiRES;
        protected Vector hpiRES;
        protected Vector yfiRES;
        protected MatrixFixed dypi_by_dxpRES, dypi_by_dhiRES, dhpi_by_dxpRES, 
                              dhpi_by_dyiRES, dyfi_by_dypiRES, dyfi_by_dlambdaRES;

        // Getters
        /// Get the value of y_{pi}, calculated earlier.
        public Vector get_ypiRES() {return ypiRES;} 
        /// Get the value of h_{pi}, calculated earlier.
        public Vector get_hpiRES() {return hpiRES;} 
        /// Get the value of y_{fi}, calculated earlier.
        public Vector get_yfiRES() {return yfiRES;} 

        /// Get the value of \frac{\partial y_{pi}}{\partial x_p},
        /// calculated earlier.
        public MatrixFixed get_dypi_by_dxpRES() {return dypi_by_dxpRES;} 
        /// Get the value of \frac{\partial y_{pi}}{\partial h_i},
        /// calculated earlier.
        public MatrixFixed get_dypi_by_dhiRES() {return dypi_by_dhiRES;} 
        /// Get the value of \frac{\partial h_{pi}}{\partial x_p},
        /// calculated earlier.
        public MatrixFixed get_dhpi_by_dxpRES() {return dhpi_by_dxpRES;} 
        /// Get the value of \frac{\partial h_{pi}{\partial y_i},
        /// calculated earlier.
        public MatrixFixed get_dhpi_by_dyiRES() {return dhpi_by_dyiRES;} 
        /// Get the value of \frac{\partial h_{fi}}{\partial y_{pi}}
        ///, calculated earlier.
        public MatrixFixed get_dyfi_by_dypiRES() {return dyfi_by_dypiRES;} 
        /// Get the value of \frac{\partial h_{fi}}{\partial \lambda}
        ///, calculated earlier.
        public MatrixFixed get_dyfi_by_dlambdaRES() {return dyfi_by_dlambdaRES;} 


        /// <summary>
        /// Initialise a new partial feature y_{pi} as a function of
        /// a measurement h_i and the robot position state x_p
        ///. Also calculate its Jacobians \frac{\partial
        /// y_{pi}}{\partial x_p} and \frac{\partial
        /// y_{pi}}{\partial y_i} and the measurement uncertainty Ri.
        /// </summary>
        /// <param name="hi"></param>
        /// <param name="xp"></param>
        public virtual void func_ypi_and_dypi_by_dxp_and_dypi_by_dhi_and_Ri(Vector hi, Vector xp) {}

        /// <summary>
        /// Calculate a measurement h_{pi} from the partial feature
        /// state y_i, robot position state x_p and a set of
        /// values \lambda for the free parameters of the
        /// feature. Also calculates the Jacobians \frac{\partial
        /// h_{pi}}{\partial x_p} and \frac{\partial
        /// h_{pi}}{\partial y_i}. 
        /// </summary>
        /// <param name="yi"></param>
        /// <param name="xp"></param>
        /// <param name="lambda"></param>
        public virtual void func_hpi_and_dhpi_by_dxp_and_dhpi_by_dyi(Vector yi, Vector xp, Vector lambda) {}


        /// <summary>
        /// Convert the partially-initialised feature state y_{pi},
        /// into the fully-initialised feature state y_{fi} using
        /// values \lambda for the free parameters.  Also calculates
        /// the Jacobians \frac{\partial yh_{fi}}{\partial y_{pi}}
        /// and \frac{\partial y_{fi}}{\partial \lambda}.
        /// </summary>
        /// <param name="ypi"></param>
        /// <param name="lambda"></param>
        public virtual void func_yfi_and_dyfi_by_dypi_and_dyfi_by_dlambda(Vector ypi, Vector lambda) {}

    }

    /// <summary>
    /// Class to create an instance of a feature measurement model class given
    /// its type string. This is an abstract base class - writers of specific
    /// feature measurement models should derive their own version able of
    /// providing thir own feature measurement models.
    /// </summary>
    public class Feature_Measurement_Model_Creator
    {
        /// Constructor.
        /// @param m_m The motion model to use when constructing the
        /// Feature_Measurement_Model
        public Feature_Measurement_Model_Creator() {} 

        /// Returns an instance of the requested feature measurement model
        /// (usually by using the new operator. TODO: who
        /// deletes them?
        public virtual Feature_Measurement_Model create_model(String type, Motion_Model motion_model, float MAXIMUM_ANGLE_DIFFERENCE) { return (null); }

    }


/**************************Internal Measurement Model*************************/

    /// <summary>
    /// For making measurements internal to the robot state.
    /// </summary>
    public class Internal_Measurement_Model
    {
        public Internal_Measurement_Model(
            uint measurement_size,
            Motion_Model m_m,
            String i_t)
        {
            MEASUREMENT_SIZE = measurement_size;
            motion_model = m_m;
            internal_type = i_t;

            hvRES = new Vector(MEASUREMENT_SIZE);
            dhv_by_dxvRES = new MatrixFixed(MEASUREMENT_SIZE, motion_model.STATE_SIZE);
            RvRES = new MatrixFixed(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
            nuvRES = new Vector(MEASUREMENT_SIZE);
            SvRES = new MatrixFixed(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
            hv_noisyRES = new Vector(MEASUREMENT_SIZE);
        }

        public void func_Sv(MatrixFixed Pxx,
                            MatrixFixed dhv_by_dxv,
                            MatrixFixed Rv)
        {
            // Zero SiRES and add bits on
            SvRES.Fill(0.0f);

            SvRES += dhv_by_dxv * Pxx * dhv_by_dxv.Transpose();

            SvRES += Rv;
        }

        
        // Constants: set in initialisation list for individual models
        public uint MEASUREMENT_SIZE;
        public Motion_Model motion_model;
        public String internal_type;

        // Where results will be stored after calls to functions
        protected Vector hvRES;
        protected Vector nuvRES;
        protected Vector hv_noisyRES;
        protected MatrixFixed dhv_by_dxvRES, RvRES, SvRES; 

        public Motion_Model get_motion_model() {return motion_model;}

        // Getters
        public Vector get_hvRES() {return hvRES;} 
        public Vector get_nuvRES() {return nuvRES;} 
        public Vector get_hv_noisyRES() {return hv_noisyRES;} 

        public MatrixFixed get_dhv_by_dxvRES() {return dhv_by_dxvRES;} 
        public MatrixFixed get_RvRES() {return RvRES;} 
        public MatrixFixed get_SvRES() {return SvRES;} 


        // Calculation functions which define the model.  Some of these functions 
        // produce several results: this is for efficiency reaons when these 
        // results often need to be used together

        // Measurement function  
        public virtual void func_hv_and_dhv_by_dxv(Vector xv) {}

        // Measurement noise
        public virtual void func_Rv(Vector hv) {}

        // Innovation calculation
        public virtual void func_nuv(Vector hv, Vector zv) {}

        // Noisy measurement for use in simulation
        public virtual void func_hv_noisy(Vector xv_true) {}

        // Test for feasibility of measurement
        public virtual bool feasibility_test(Vector xv, Vector hv) { return (false); }

    }

    /// <summary>
    /// Class to create an instance of a internal measurement model class given
    /// its type string. This is an abstract base class - writers of specific
    /// internal measurement models should derive their own version able of
    /// providing thir own internal measurement models.
    /// </summary>
    public class Internal_Measurement_Model_Creator
    {
        /// Constructor.
        /// @param m_m The motion model to use when constructing the
        /// Internal_Measurement_Model
        public Internal_Measurement_Model_Creator() { }

        /// Returns an instance of the requested internal measurement model
        /// (usually by using the new operator. TODO: who
        /// deletes them?
        public virtual Internal_Measurement_Model create_model(String type,
                                                               Motion_Model motion_model) { return (null); }
    }

}
