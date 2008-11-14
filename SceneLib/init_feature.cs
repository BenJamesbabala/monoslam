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
using System.Collections;
using System.Collections.Generic;
using System.Text;

namespace SceneLibrary
{
    /// <summary>
    /// A class for weighted samples and measurement information.
    /// </summary>
    public class Particle
    {
        //friend class FeatureInitInfo;
        //friend class Scene_Single;

        /// <summary>
        /// Constructor. This is protected since it is only called
        /// from FeatureInitInfo::add_particle()
        /// </summary>
        /// <param name="l">The value(s) for the free parameters \lambda represented by this particle.</param>
        /// <param name="p">The initial probability for this particle</param>
        /// <param name="MEASUREMENT_SIZE">The number of parameters representing a measurement of a feature</param>
        public Particle(Vector l, float p, uint MEASUREMENT_SIZE)
        {
            lambda = new Vector(l); 
            probability = p;

            m_z = new Vector(MEASUREMENT_SIZE);
            m_h = new Vector(MEASUREMENT_SIZE);
            m_SInv = new MatrixFixed(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
        }

        // Getters and setters for use outside Scene_Single
        // Get the value(s) of the free parameters \lambda associated
        // with this particle. 
        public Vector get_lambda() {return lambda;}
        // Get the current expected measurement state h_i for this
        // particle. (e.g. the image location). 
        public Vector get_h() {return m_h;}
        // Get the current measurement z_i for this particle. (e.g. the image
        // location). 
        public Vector get_z() {return m_z;}
        // Get the inverse innovation covariance S_i^{-1} for this
        // particle. The innovation covariance is the total uncertainty in the
        // measurement as a result of uncertainties in the robot, feature and measurement
        // states; the inverse is used to determine a search region in measurement
        // space (e.g. an ellipse in the image).
        public MatrixFixed get_SInv() {return m_SInv;}
        // Get the determinant of the current innovation covariance,
        // \arrowvert S_i \arrowvert. 
        public float get_detS() {return m_detS;}
        // Was the last measurement of this particle successful? 
        public bool get_successful_measurement_flag() {return m_successful_measurement_flag;}
        // Returns the current probability of this particle. 
        public float get_probability() {return probability;}
    
        /// Update the measurement of the feature state. (e.g. the image location)
        public void set_z(Vector new_z) {m_z.Update(new_z);}
        /// Called to update the result of the most recent match attempt
        public void set_successful_measurement_flag(bool newstate) {m_successful_measurement_flag = newstate;}

        // Set the current measurement position h_i for this particle. Called
        // from Scene_Single::predict_partially_initialised_feature_measurements() */
        public void set_h(Vector h) {m_h.Update(h);}
           
        /// The value(s) of the free parameters represented by this particle
        public Vector lambda;
        /// The probability of this particle
        public float probability = 0;
        /// Cumulative probability of all particles up to and including this one
        public float cumulative_probability = 0;

        // Measurement information used for updating particle
        // The measurement of the feature state z_i. (e.g. the image
        // location)
        protected Vector m_z;
        // The current predicted measurement state h_i for this
        // particle. (e.g. image location) 
        protected Vector m_h;
        // The current inverse innovation covariance S_i^{-1} for
        // this particle. 
        protected MatrixFixed m_SInv;
        // The determinant of the current innovation covariance |S_i| 
        protected float m_detS;
        // Was the last measurement of this particle successful? 
        protected bool m_successful_measurement_flag;

        // Set the current innovation covariance S_i^{-1} for this
        // particle. Called from 
        // Scene_Single::predict_partially_initialised_feature_measurements() 
        public void set_S(MatrixFixed Si)
        {
            Cholesky local_Si_cholesky = new Cholesky(Si);
            m_SInv.Update(local_Si_cholesky.Inverse());
            m_detS = SceneLib.Determinant(Si);
        }

    }











    /// <summary>
    /// Class to hold information about a partially-initialised feature. This maintains
    /// a list of particle representing the free parameters \lambda in
    /// the feature.
    /// 
    /// Particles are added by the user of this class by calling add_particle() (no
    /// particles are initially present). Their probabilities are updated externally in
    /// Scene_Single, and then the current estimate for \lambda can be found by
    /// calling normalise_particle_vector_and_calculate_cumulative() followed by
    /// calculate_mean_and_covariance(). Optionally (but recommended),
    /// prune_particle_vector() can also be called to remove particles with small
    /// probabilities.    
    /// </summary>
    public class FeatureInitInfo
    {
        //friend class Scene_Single;

        private Random rnd = new Random();

        //typedef Vector<Particle> as ParticleVector;

        /// <summary>
        /// Constructor. The class is intialised with no particles, so these must be added later using add_particle()
        /// </summary>
        /// <param name="sc">The Scene class.</param>
        /// <param name="f">The feature for which this class represents additional information.</param>
        public FeatureInitInfo(Scene_Single sc, Feature f)
        {
            scene = sc;
            fp = f;
            PARTICLE_DIMENSION = f.get_partially_initialised_feature_measurement_model().FREE_PARAMETER_SIZE;
            number_of_match_attempts = 0;
            nonzerovariance = false;

            mean = new Vector(PARTICLE_DIMENSION);
            covariance = new MatrixFixed(PARTICLE_DIMENSION, PARTICLE_DIMENSION);

            //if (Scene_Single::STATUSDUMP) cout << "PARTICLE_DIMENSION = " 
                                    //<< PARTICLE_DIMENSION << endl;
        }


        /// <summary>
        /// Create a particle and add it to this partially-initialised feature's list of particles
        /// </summary>
        /// <param name="lambda">The value(s) of the free parameters represented by this particle.</param>
        /// <param name="probability">The initial probability to assign to the particle.</param>
        public void add_particle(Vector lambda, float probability)
        {
            //assert(lambda.Size() == PARTICLE_DIMENSION);
            Particle p = new Particle(lambda, probability,
                                      fp.get_partially_initialised_feature_measurement_model().MEASUREMENT_SIZE);
            particle_vector.Add(p);
        }

        /// <summary>
        /// Normalise the probabilities of the particles (divide them by the
        /// total probability so that they sum to one). This function sets each
        /// Particle::probability to its normalised value, and also sets
        /// Particle::cumulative_probability to the total probability of all the particles
        /// up to and including the current one (after normalisation).
        /// </summary>
        /// <returns>false if the total probability is zero, true otherwise.</returns>
        public bool normalise_particle_vector_and_calculate_cumulative()
        {
            float total = 0.0f;
            int i;
            Particle it;
            for (i=0;i<particle_vector.Count;i++)
            {
                it = (Particle)particle_vector[i];
                total += it.get_probability();
            }

            if (total == 0.0)
                return false;

            float cumulative_total = 0.0f;
            for (i = 0; i < particle_vector.Count; i++)
            {
                it = (Particle)particle_vector[i];
                it.probability = it.get_probability() / total;
                it.cumulative_probability = cumulative_total + it.get_probability();
                cumulative_total += it.get_probability();

                //if (DEBUGDUMP) cout << "Particle at " << it.lambda 
                        //<< " normalised probability " << it.probability
                        //<< endl;
            }

            return true;
        }

        /// <summary>
        /// Prune the vector of particles.
        /// It is assumed that the particle probabilities are normalised before we do this.
        /// This function remove particles with probability below
        /// prune_probability_threshold * (1/N) and then normalises again.
        /// </summary>
        /// <param name="?">The threshold to use.</param>
        public void prune_particle_vector(float prune_probability_threshold)
        {
            float prune_threshold = prune_probability_threshold / (float)(particle_vector.Count);

            int i;
            Particle it;
            for (i=0;i<particle_vector.Count;i++)
            {
                it = (Particle)particle_vector[i];
                if (it.probability < prune_threshold)
                {
                    particle_vector.Remove(it);
                }
            }

            normalise_particle_vector_and_calculate_cumulative();
        }

        /// <summary>
        /// Calculate the mean and covariance of \lambda over all the particles,
        /// i.e.
        /// 
        /// \text{mean} &= \mu = \sum_i \lambda_i p(i) \nonumber \\
        /// \text{mean} &= \sum_i \lambda_i\lambda_i^T p(i) - \mu\mu^T \nonumber
        ///
        /// The result is not returned, but is instead stored in the class to be read using
        /// get_mean() and get_covariance().
        /// </summary>
        public void calculate_mean_and_covariance()
        {
            // Vector which will store expected value of lambda * lambda^T
            // (allows us to do this calculation with one pass through particles)
            MatrixFixed ExpectedSquared = new MatrixFixed(PARTICLE_DIMENSION, PARTICLE_DIMENSION);
            ExpectedSquared.Fill(0.0f); 

            // Zero mean vector before filling it up
            mean.Fill(0.0f);

            foreach (Particle it in particle_vector)
            {
                mean += it.lambda * it.probability;
                ExpectedSquared += it.probability * Vector.OuterProduct(it.lambda, it.lambda);
            }

            covariance = ExpectedSquared - Vector.OuterProduct(mean, mean);
        }

        /// <summary>
        /// For each Particle, print its value(s) for \lambda, probability and
        /// cumulative probability to <code>cout</code>.
        /// </summary>
        public void print_probability_distribution()
        {
            int i;
            Particle it;
            for (i = 0; i < particle_vector.Count; i++)
            {
                it = (Particle)particle_vector[i];
                /*
                cout << "Lambda " << it.lambda << " probability " 
                << it.probability << " cumulative " 
	            << it.cumulative_probability << endl;
                */
            }
        }

        /// <summary>
        /// Print the Particle values for \lambda and their probabilities to
        /// <code>cerr</code> in the format of two Matlab vectors, <code>lambda</code> and
        /// <code>prob</code>.
        /// </summary>
        public void print_probability_distribution_matlab()
        {
            Debug.Write("lambda = [");
            Particle it;
            int i;
            for (i = 0; i < particle_vector.Count; i++)
            {
                it = (Particle)particle_vector[i];
                Debug.Write(it.lambda);
            }
             Debug.WriteLine("];");

            Debug.Write("prob = [");
            for (i = 0; i < particle_vector.Count; i++)
            {
                it = (Particle)particle_vector[i];
                Debug.Write(Convert.ToString(it.probability)+ " ");
            }
            Debug.WriteLine("];");
        }


        /*
        /// <summary>
        /// Draw a random sample from the Particle vector, from the pdf given by
        /// the current probabilities.
        /// </summary>
        /// <param name="position_in_vector">Filled in with the index of the chosen Particle.</param>
        /// <returns>The chosen Particle.</returns>
        public Particle sample_from_cumulative_distribution(uint[] position_in_vector)
        {
            float random_number = rnd.Next(10000) / 10000.0f;

            *position_in_vector = 0;
            Particle it;
            int i;
            for (i=0;i<particle_vector.Count;i++)
            {
                it = (Particle)particle_vector[i];
                if (it.cumulative_probability >= random_number)
                {
                    return it;
                }

                (*position_in_vector)++;
            }

            // Should never reach here
            assert(0);
        }
        */


        // Some getters and setters for use outside Scene_Single
        public Feature get_fp() { return fp; }
        public void set_fp(Feature new_fp) { fp = new_fp; }
        public Feature get_fp_noconst() { return fp; }
        public ArrayList get_particle_vector() { return particle_vector; }
        public ArrayList get_particle_vector_noconst() { return particle_vector; }
        public bool get_making_measurement_on_this_step_flag() { return making_measurement_on_this_step_flag; }

        // Returns the most recent value of the mean \lambda calculated with
        // calculate_mean_and_covariance(). 
        public Vector get_mean() { return mean; }
        // Returns the most recent value of the covariance of \lambda
        // calculated with calculate_mean_and_covariance(). 
        public MatrixFixed get_covariance() { return covariance; }

        // Since particles can live in PARTICLE_DIMENSION dimensions, 
        // mean of distribution is a vector and covariance is a matrix
        protected Vector mean;
        protected MatrixFixed covariance;

        // Pointers to related scene and feature objects
        protected Scene_Single scene;
        protected Feature fp;

        //whether this feature has a non-zero variance in its particle probabilities
        public bool nonzerovariance;

        // Vector of particles representing probability distribution
        public ArrayList particle_vector = new ArrayList();

        public uint number_of_match_attempts = 0;
        public bool making_measurement_on_this_step_flag;
        // The dimension the particle distribution will live in
        public uint PARTICLE_DIMENSION;
    }

}
