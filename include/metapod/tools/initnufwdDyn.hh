// Copyright 2014
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


#ifndef METAPOD_INITNUFWDDYN_HH
# define METAPOD_INITNUFWDDYN_HH

# include <metapod/tools/depth_first_traversal.hh>

namespace metapod {
namespace internal {

template <typename Robot, int node_id>
struct InitNuFwdDynVisitor
{
  static void discover(Robot& robot)
  {
    typedef typename Nodes<Robot, node_id>::type Node;
    typedef typename Nodes<Robot, Node::parent_id>::type Parent;
    // if node_id parent is part of nu(fd) set, then node_id is also part of nu(fd)
    Parent& parent = boost::fusion::at_c<Node::parent_id>(robot.nodes);
    Node& node = boost::fusion::at_c<node_id>(robot.nodes);
    node.joint.nuOfFwDyn = parent.joint.nuOfFwDyn;
  }
  static void finish(Robot& robot) {}
};

} // end of namespace metapod::internal

/// init the "nuFwdDyn" parameter for each joint.
template< typename Robot > struct initNuFwdDyn
{
  static void run(Robot& robot)
  {
    depth_first_traversal< internal::InitNuFwdDynVisitor, Robot >::run(robot);
  }
};

} // end of namespace metapod

#endif
