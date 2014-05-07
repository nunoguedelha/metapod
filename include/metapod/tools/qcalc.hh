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

enum Qoperation
{
  ADD = 0,
  BUILD
};

namespace internal {

template< typename Robot, int node_id > struct QcalcVisitor
{

  // Add indexVector to FD matrix in case of "fd" mode joint,
  // or add indexVector to FD matrix in case of "id" mode joint.
  
  typedef typename Nodes<Robot, node_id>::type Node;

  static void discover(Robot& robot)
  {
    Node& node = boost::fusion::at_c<node_id>(robot.nodes);
    // set from node_id, the indexVector to add to FD or ID permutation matrix
    Eigen::Matrix< FloatType, 1, NBDOF > indexVector = Eigen::Matrix< FloatType, NBDOF, NBDOF >::Identity.row(node_id);
    HandleJointToQmatrix<ADD, indexVector, node.joint.fwdDyn>::run(robot);
  }
  static void finish(Robot& robot)
  {}
};

} // end of namespace metapod::internal

template< typename Robot > struct qcalc
{
  static void run(Robot& robot)
  {
    depth_first_traversal<internal::QcalcVisitor, Robot>::run(robot);
    HandleJointToQmatrix<BUILD>::run(robot);
  }
};

} // end of namespace metapod

# endif
