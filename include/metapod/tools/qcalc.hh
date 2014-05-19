// Copyright 2012, 2013
//
// Nuno Guedelha (JRL/LAAS, CNRS/AIST)
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

namespace metapod {

namespace internal {

typedef enum Qoperation
{
  ADD = 0,
  BUILD
};

typedef Eigen::Matrix<FloatType, 1, NBDOF> VectorNBDOFf;

template< typename Robot, int node_id > struct QcalcVisitor
{
  typedef typename Nodes<Robot, node_id>::type Node;
  
  static VectorNBDOFf fdNodesFirst = VectorNBDOFf::Zero(); // permutation indexes for building Q matrix
  static VectorNBDOFf idNodes = VectorNBDOFf::Zero(); // permutation indexes for building Q matrix
  static fdNodesFirstFillIndex = 0;
  static idNodesFillIndex = 0;
  
  template< Qoperation operation, bool fwdDyn >
  struct HandleJointToQmatrix {};
  
  template<>
  struct HandleJointToQmatrix<ADD, true>
  {
    static void run()
    {
      // Add node_id index to FD vector in case of a "fd" mode joint
      fdNodesFirst(1, fdNodesFirstFillIndex) = node_id;
      fdNodesFirstFillIndex++;
    }
  };
  
  template<>
  struct HandleJointToQmatrix<ADD, false>
  {
    static void run()
    {
      // Add node_id index to ID vector in case of a "id" mode joint
      idNodes(1, idNodesFillIndex) = node_id;
      idNodesFillIndex++;
    }
  };
    
  template< bool fwdDyn >
  struct HandleJointToQmatrix<BUILD, fwdDyn>
  {
    static void run()
    {
      // concatenate both lists
      fdNodesFirst.tail(idNodesFillIndex) = idNodes.head(idNodesFillIndex);
      // get permutation matrix Q from node_id list
      Node::Q = Eigen::PermutationMatrix<NBDOF, NBDOF, FloatType>(fdNodesFirst).toDenseMatrix();
    }
  };
  
  static void discover()
  {
    typedef typename Nodes<Robot, node_id>::type Node;
    // add node_id to FD or ID permutation matrix depending on the joint mode
    HandleJointToQmatrix<ADD, Node::jointFwdDyn>::run();
  }
  static void finish()
  {}
};

} // end of namespace metapod::internal

template< typename Robot > struct qcalc
{
  static void run()
  {
    depth_first_traversal<internal::QcalcVisitor, Robot>::run();
    internal::HandleJointToQmatrix<internal::BUILD, true>::run();
  }
};

} // end of namespace metapod

# endif
