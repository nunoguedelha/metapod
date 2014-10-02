// Copyright 2012, 2014
//
// Nuno Guedelha (LAAS, CNRS)
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
 * Implementation of the permutation matrix Q, as described in
 * Featherstone's Rigid Body Dynamics Algorithms.
 * Each line of Q gives the position index of the next joint in the "fd" mode.
 * Explore every joint of the system tree :
 * - for each "fd" joint, add a line in the FD matrix with its respective position
 * - for each "id" joint, add a line in the ID matrix with its respective position.
 * cocatenate matrix Q:
 *   => Q = |FD|
 *          |ID|
 */

#ifndef METAPOD_QCALC_HH
# define METAPOD_QCALC_HH

# include <metapod/tools/depth_first_traversal.hh>
# include <cassert>

namespace metapod {

namespace internal {
  
  typedef enum
  {
    ADD = 0,
    BUILD
  } Qoperation;
  
  template< typename Robot, int maxIndex, int currentIndex = 0 >
  struct OrdrerProofWrapper_internal
  {
    static void run(Robot &robot, typename Robot::MatrixNBDOFf &Hrff)
    {
      // swap column indices "currentIndex" with "fdNodesFirst[currentIndex]"

      OrdrerProofWrapper_internal<Robot, maxIndex, currentIndex+1>::run(robot, Hrff);
    }
  };

  template< typename Robot, int maxIndex >
  struct OrdrerProofWrapper_internal<Robot, maxIndex, maxIndex>
  {
    static void run(Robot &, typename Robot::MatrixNBDOFf &) {}
  };

  template< typename Robot, int q_idx, int nbdof, Qoperation operation, bool fwdDyn >
  struct HandleJointToQmatrix {};
  
  template< typename Robot, int q_idx, int nbdof >
  struct HandleJointToQmatrix<Robot, q_idx, nbdof, ADD, true>
  {
    static void run()
    {
      // Add q_idx index to FD vector in case of a "fd" mode joint
      Robot::fdNodes(0, Robot::fdNodesFillIndex) = q_idx;
      Robot::fdNodesFillIndex++;
      HandleJointToQmatrix<Robot, q_idx+1, nbdof-1, ADD, true>::run();
    }
  };
  
  template< typename Robot, int q_idx, int nbdof >
  struct HandleJointToQmatrix<Robot, q_idx, nbdof, ADD, false>
  {
    static void run()
    {
      // Add q_idx index to ID vector in case of an "id" mode joint
      Robot::idNodes(0, Robot::idNodesFillIndex) = q_idx;
      Robot::idNodesFillIndex++;
      HandleJointToQmatrix<Robot, q_idx+1, nbdof-1, ADD, false>::run();
    }
  };
    
  template< typename Robot, int q_idx >
  struct HandleJointToQmatrix<Robot, q_idx, 0, ADD, false>
  {
    static void run() {}
  };
  
  template< typename Robot, int q_idx >
  struct HandleJointToQmatrix<Robot, q_idx, 0, ADD, true>
  {
    static void run() {}
  };
  
  template< typename Robot, int q_idx, int nbdof, bool fwdDyn >
  struct HandleJointToQmatrix<Robot, q_idx, nbdof, BUILD, fwdDyn>
  {
    static void run()
    {
      // concatenate both lists
      Robot::fdNodesFirst.head(Robot::fdNodesFillIndex) = Robot::fdNodes;
      Robot::fdNodesFirst.tail(Robot::idNodesFillIndex) = Robot::idNodes;
      // build permutation matrix Q from FD & ID nodes reordered list
      Robot::Q  = typename Robot::TranspositionsNBDOFi(Robot::fdNodesFirst);
//      Robot::Qt = Robot::Q.inverse();

      // Permutation matrix restricted to fdNodes => Q.H = [H_11 H_12]
      Robot::Q1  = typename Robot::TranspositionsNbFdDOFi(Robot::fdNodes);
//      Robot::Q1t = Robot::Q1.transpose();

      // Permutation matrix restricted to idNodes => H.Q^t = [H_11; H_21]
/*      Robot::Q2t= Eigen::PermutationMatrix<Robot::NBDOF-Robot::nbFdDOF, Robot::NBDOF, typename Robot::RobotFloatType>(Robot::fdNodesFirst.tail(Robot::NBDOF-Robot::nbFdDOF));
      Robot::Q2 = Robot::Q2t.transpose();*/
    }
  };
  
  
  template< typename Robot, int node_id > struct QcalcVisitor
  {
    typedef typename Nodes<Robot, node_id>::type Node;
    
    static void discover()
    {
      typedef typename Nodes<Robot, node_id>::type Node;
      // add q_idx to FD or ID permutation matrix depending on the joint mode
      HandleJointToQmatrix<Robot, Node::q_idx, Node::Joint::NBDOF, ADD, Node::jointFwdDyn>::run();
    }
    static void finish()
    {}
  };

} // end of namespace metapod::internal

  template< typename Robot > struct qcalc
  {
    static void run()
    {
      Robot::fdNodesFillIndex = 0;
      Robot::idNodesFillIndex = 0;
      depth_first_traversal<internal::QcalcVisitor, Robot>::run();
      internal::HandleJointToQmatrix<Robot, 0, 0, internal::BUILD, true>::run();
    }
  };

  template< typename Robot > struct OrdrerProofWrapper
  {
    static void run(Robot &robot, typename Robot::MatrixNBDOFf &Hrff)
    {
      internal::OrdrerProofWrapper_internal<Robot, Robot::nbFdDOF>::run(robot, Hrff);
    }
  };

} // end of namespace metapod

# endif
