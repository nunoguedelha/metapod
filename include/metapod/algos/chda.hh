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
		  const typename Robot::confVector& ddq, 
		  const typename Robot::confVector& torques
		  )
  {
    /* below matrices and vectors reordered such that fwd dynamic joints 
       come first, will get the postfix rff (reordered fwd dynamics first).*/
    
    typedef typename Robot::confVector confVector;
    
    // 1 - compute Cprime = ID(q,q',Qt[0 q2"]) using RNA :
    qcalc< Robot >::run(); // Apply the permutation matrix Q
    std::cout << Robot::Q << std::endl << std::endl; // TEST
    
    confVector ddq_rff_1_zeroed = Robot::Q * ddq; // First, reordered ddq
    std::cout << ddq_rff_1_zeroed << std::endl << std::endl; // TEST
    
    ddq_rff_1_zeroed.head(Robot::fdNodesFirstFillIndex).setZero(); // Then, set unknown accelerations to 0
    std::cout << ddq_rff_1_zeroed << std::endl << std::endl; // TEST
    
    confVector ddq_1_zeroed = Robot::Qt * ddq_rff_1_zeroed; // roll back to original index order
    std::cout << ddq_1_zeroed << std::endl << std::endl; // TEST
    
    rnea< Robot, true >::run(robot, q, dq, ddq_1_zeroed); // compute torques => Cprime
    
    confVector CprimeTorques; getTorques(robot, CprimeTorques); // get computed torques
    robot.Cprime = CprimeTorques; // set those torques to robot Cprime parameter
    std::cout << robot.Cprime << std::endl << std::endl; // TEST
    
    // 2 - compute H11 from Hprime = Q.H.Qt
    typedef typename Robot::MatrixNBDOFf MatrixHrff;
    typedef Eigen::Matrix<typename Robot::RobotFloatType, 1, 1> MatrixH11;
    typedef Eigen::Matrix<typename Robot::RobotFloatType, 1, 2> MatrixH12;
    typedef Eigen::Matrix<typename Robot::RobotFloatType, 2, 1> MatrixH21;
    typedef Eigen::Matrix<typename Robot::RobotFloatType, 2, 2> MatrixH22;
    
    crba<Robot, true>::run(robot, q); // compute whole H
    std::cout << robot.H << std::endl << std::endl; // TEST
    MatrixHrff Hrff = Robot::Q * robot.H * Robot::Qt; // compute H11, square matrix of size "fdNodesFirstFillIndex"
    std::cout << Hrff << std::endl << std::endl; // TEST
    
    MatrixH11 H11 = Hrff.template block(0, 0, Robot::fdNodesFirstFillIndex, Robot::fdNodesFirstFillIndex);
    std::cout << H11 << std::endl; // TEST
    
    // 3 - solve H11 q1" = tau1 - C1prime

    // 4 - compute tau = Cprime + Qt[H11.q1" H21.q1"]
    
  }
};

} // end of namespace metapod

#endif
