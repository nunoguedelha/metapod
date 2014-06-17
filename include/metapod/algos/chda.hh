// Copyright 2014
//
// Nuno Guedelha (CNRS)
//
// This file is part of metapod.
// metapod is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// metapod is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with metapod.  If not, see <http://www.gnu.org/licenses/>.

/*
 * Implementation of the Composite Rigid Body Algorithm,
 * based on Featherstone's Rigid Body Dynamics Algorithms.
 */

#ifndef METAPOD_MAIN_HDA_HH
# define METAPOD_MAIN_HDA_HH

# include "metapod/tools/common.hh"
# include "metapod/tools/jcalc.hh"
# include "metapod/tools/depth_first_traversal.hh"
# include "metapod/tools/backward_traversal_prev.hh"
# include "metapod/tools/initnufwddyn.hh"
# include "metapod/tools/qcalc.hh"
# include "metapod/algos/rnea.hh"
# include "metapod/algos/crba.hh"

namespace metapod {
namespace internal {

/// Templated Hybrid Dynamics Algorithm.
/// Takes the multibody tree type as template parameter,
/// and recursively proceeds on the Nodes.



// helper function: 

} // end of namespace metapod::internal


// frontend
template< typename Robot > struct chda
{
  static void run(Robot& robot, 
		  const typename Robot::confVector& q, 
		  const typename Robot::confVector& dq, 
		  typename Robot::confVector& ddq, 
		  typename Robot::confVector& torques
		  )
  {
    /* below matrices and vectors which reordered such that fwd dynamic joints 
       come first, will get the postfix rff (reordered fwd dynamics first).*/
    
    typedef typename Robot::confVector confVector;
    typedef typename Robot::MatrixNBDOFf MatrixNBDOFf;
    
    typedef Eigen::Matrix<typename Robot::RobotFloatType, Robot::nbFdDOF, Robot::NBDOF> MatrixDof10; // first nbFdDOF lines
    typedef Eigen::Matrix<typename Robot::RobotFloatType, Robot::NBDOF, Robot::nbFdDOF> MatrixDof01; // first nbFdDOF columns
    typedef Eigen::Matrix<typename Robot::RobotFloatType, Robot::NBDOF-Robot::nbFdDOF, Robot::NBDOF> MatrixDof20; // last NBDOF-nbFdDOF lines
    typedef Eigen::Matrix<typename Robot::RobotFloatType, Robot::NBDOF, Robot::NBDOF-Robot::nbFdDOF> MatrixDof02; // last NBDOF-nbFdDOF columns
    
    typedef Eigen::Matrix<typename Robot::RobotFloatType, Robot::nbFdDOF, 1> confVectorDof1;
    typedef Eigen::Matrix<typename Robot::RobotFloatType, Robot::NBDOF-Robot::nbFdDOF, 1> confVectorDof2;
    typedef Eigen::Matrix<typename Robot::RobotFloatType, Robot::nbFdDOF, Robot::nbFdDOF> MatrixDof11;
    typedef Eigen::Matrix<typename Robot::RobotFloatType, Robot::nbFdDOF, Robot::NBDOF-Robot::nbFdDOF> MatrixDof12;
    typedef Eigen::Matrix<typename Robot::RobotFloatType, Robot::NBDOF-Robot::nbFdDOF, Robot::nbFdDOF> MatrixDof21;
    typedef Eigen::Matrix<typename Robot::RobotFloatType, Robot::NBDOF-Robot::nbFdDOF, Robot::NBDOF-Robot::nbFdDOF> MatrixDof22;
    
    // we shall use Q and Q sub-matrices Q1left, Q1right, Q2left, Q2right
    qcalc< Robot >::run(); // Apply the permutation matrix Q
    MatrixDof10 Q1left = Robot::Q.template topRows<Robot::nbFdDOF>();
    MatrixDof01 Q1right = Robot::Q.template leftCols<Robot::nbFdDOF>();
    MatrixDof20 Q2left = Robot::Q.template bottomRows<Robot::NBDOF-Robot::nbFdDOF>();
    MatrixDof02 Q2right = Robot::Q.template rightCols<Robot::NBDOF-Robot::nbFdDOF>();
    
    // 1 - compute Cprime = ID(q,q',Qt[0 q2"]) using RNA :
    confVector ddq_rff_1_zeroed = Robot::Q * ddq; // First, reorder ddq
    
    ddq_rff_1_zeroed.template head<Robot::nbFdDOF>().template setZero(); // Then, set unknown accelerations to 0
    
    confVector ddq_1_zeroed = Robot::Qt * ddq_rff_1_zeroed; // roll back to original index order
    
    rnea< Robot, true >::run(robot, q, dq, ddq_1_zeroed); // compute torques => Cprime
    
    confVector CprimeTorques; getTorques(robot, CprimeTorques); // get computed torques
    robot.Cprime = CprimeTorques; // set those torques to robot Cprime parameter
    
    // 2 - compute H11 from Hprime = Q.H.Qt
    crba<Robot, true>::run(robot, q); // First, compute whole H
    MatrixNBDOFf Hrff = Robot::Q * robot.H * Robot::Qt; // H reordered
    MatrixDof11 H11 = Hrff.template topLeftCorner<Robot::nbFdDOF, Robot::nbFdDOF>(); // H11, square matrix of size "nbFdDOF x nbFdDOF"
    
    // 3 - solve H11*q1" = tau1 - C1prime
    confVectorDof1 tau1 = Q1left * torques; // compute tau1: all known torques (nbFdDOF lines)
    confVectorDof1 C1prime = Q1left * CprimeTorques; // compute C1prime (nbFdDOF lines)
    // solve system
    Eigen::LLT<MatrixDof11> lltOfH11(H11);
    confVectorDof1 ddq1 = lltOfH11.solve(tau1 - C1prime);
    
    // 4 - compute tau = Cprime + Qt[H11.q1" H21.q1"]
    //     tau = [tau1, tau2]t
    //     tau2 = C2prime + H21.q1"
    confVectorDof2 C2prime = Q2left * CprimeTorques; // C2prime (NBDOF-nbFdDOF lines)
    MatrixDof21 H21 = Hrff.template bottomLeftCorner<Robot::NBDOF-Robot::nbFdDOF, Robot::nbFdDOF>(); // H21, square matrix of size "NBDOF-nbFdDOF x nbFdDOF"
    confVectorDof2 tau2 = C2prime + H21 * ddq1;
    
    // 5 - complete output vectors ddq and torques
    confVectorDof2 ddq2 = Q2left * ddq;
    confVector ddqRff;
    ddqRff << ddq1,
              ddq2;
    ddq = Robot::Qt * ddqRff;
/*
    confVector torquesRff;
    torquesRff << tau1,
                  tau2;
    torques = Robot::Qt * torquesRff;
*/
    // Here, computation of torques does not use matrix H.
    rnea< Robot, true >::run(robot, q, dq, ddq);
    getTorques(robot, torques); // get final computed torques
  }
};

} // end of namespace metapod

#endif
