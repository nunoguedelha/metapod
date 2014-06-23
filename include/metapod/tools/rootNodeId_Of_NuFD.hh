// Copyright 2012, 2013
//
// Sébastien Barthélémy (Aldebaran Robotics)
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

/// Implementation of two algorithms that check whether a node is NP (no
/// parent) or not and if it has a parent or not.

#ifndef METAPOD_ROOTNODEID_OF_NUFD_HH
# define METAPOD_ROOTNODEID_OF_NUFD_HH

# include <metapod/tools/rootNodeId_Of_NuFD.hh>

namespace metapod
{

  // helper function: browses the nu(FD) subtree until finding the root node.
  template < typename AnyRobot, typename AnyStartNode, bool parent_nuOfFwdDyn, int parent_id > struct rootNodeId_Of_NuFD
  {
    typedef boost::fusion::result_of::value_at_c<typename AnyRobot::NodeVector, parent_id>::type Parent;
    static const int value = rootNodeId_Of_NuFD<AnyRobot, AnyStartNode, Parent::jointNuOfFwdDyn, parent_id>::value;
  }

  template < typename AnyRobot, typename AnyStartNode, bool parent_nuOfFwdDyn >
  struct rootNodeId_Of_NuFD<AnyRobot, AnyStartNode, parent_nuOfFwdDyn, NO_PARENT >
  {
    static const int value = AnyStartNode::id;
  }
  template < typename AnyRobot, typename AnyStartNode, int parent_id >
  struct rootNodeId_Of_NuFD<AnyRobot, AnyStartNode, false, parent_id >
  {
    static const int value = AnyStartNode::id;
  }
  template < typename AnyRobot, typename AnyStartNode, int parent_id >
  struct rootNodeId_Of_NuFD<AnyRobot, AnyStartNode, true, parent_id >
  {
    static const int value = AnyStartNode::id;
  }

  template < typename AnyRobot, typename AnyStartNode, int parent_id >
  struct rootNodeId_Of_NuFD<AnyRobot, AnyStartNode, true, parent_id >
  {
    typedef boost::fusion::result_of::value_at_c<typename AnyRobot::NodeVector, parent_id>::type Parent;
    static const int value = rootNodeId_Of_NuFD<AnyRobot, AnyStartNode, Parent::jointNuOfFwdDyn, parent>::value;
  }
  template < typename AnyRobot, typename AnyStartNode >
  struct rootNodeId_Of_NuFD<AnyRobot, AnyStartNode, false>
  {
    static const int value = AnyStartNode::id;
  }

} // end of namespace metapod.

#endif

