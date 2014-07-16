// Copyright 2014
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

/// Implementation of an algorithm that browses the nu(FD) subtree until finding its root node.

#ifndef METAPOD_ROOTNODE_OF_NUFD_HH
# define METAPOD_ROOTNODE_OF_NUFD_HH


namespace metapod {

  namespace internal {

    typedef enum
    {
      STATE_CHECK_PARENT,
      STATE_CHECK_NUFD
    } state_t;


    template < typename AnyRobot, int any_node_id, bool parent_nuOfFwdDyn, int parent_id, state_t state > struct rootNode_Of_NuFD_internal {};

    template < typename AnyRobot, int any_node_id >
    struct rootNode_Of_NuFD_internal<AnyRobot, any_node_id, false, NO_PARENT, STATE_CHECK_PARENT>
    {
      static const int value = any_node_id;
    };

    template < typename AnyRobot, int any_node_id, int parent_id >
    struct rootNode_Of_NuFD_internal<AnyRobot, any_node_id, false, parent_id, STATE_CHECK_PARENT>
    {
      typedef typename boost::fusion::result_of::value_at_c<typename AnyRobot::NodeVector, parent_id>::type Parent;
      static const int value = rootNode_Of_NuFD_internal<AnyRobot, any_node_id, Parent::jointNuOfFwdDyn, parent_id, STATE_CHECK_NUFD>::value;
    };

    template < typename AnyRobot, int any_node_id, int parent_id >
    struct rootNode_Of_NuFD_internal<AnyRobot, any_node_id, false, parent_id, STATE_CHECK_NUFD >
    {
      static const int value = any_node_id;
    };

    template < typename AnyRobot, int any_node_id, int parent_id >
    struct rootNode_Of_NuFD_internal<AnyRobot, any_node_id, true, parent_id, STATE_CHECK_NUFD >
    {
      typedef typename boost::fusion::result_of::value_at_c<typename AnyRobot::NodeVector, parent_id>::type Parent;
      static const int value = rootNode_Of_NuFD_internal<AnyRobot, parent_id, false, Parent::parent_id, STATE_CHECK_PARENT>::value;
    };

  } // end of namespace internal.

  template < typename Robot, typename StartNode >
  struct rootNode_Of_NuFD
  {
    static const int id = internal::rootNode_Of_NuFD_internal<Robot, StartNode::id, false, StartNode::parent_id, internal::STATE_CHECK_PARENT>::value;
    typedef typename boost::fusion::result_of::value_at_c<typename Robot::NodeVector, id>::type type;
    static const int parent_id = type::parent_id;
  };
} // end of namespace metapod.

#endif

