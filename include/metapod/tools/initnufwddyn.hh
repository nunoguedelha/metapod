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

# include <metapod/tools/constants.hh>
# include <metapod/tools/depth_first_traversal.hh>

namespace metapod {
namespace internal {

  // helper function: updates nu(fd) of current node's base joint depending on parent node's base joint. If node_id parent is part of nu(fd) set, then node_id is also part of nu(fd). We first define the generic function...
  template <typename Robot, int node_id, bool isParentNuOfFwdDyn>
  struct inheritNuFwdDyn {};

  // specialisation: nu(fd) of parent node's base joint is true, so child node's joint inherits this property.
  template <typename Robot, int node_id>
  struct inheritNuFwdDyn<Robot, node_id, true>
  {
    static void run()
    {
      typedef typename Nodes<Robot, node_id>::type Node;
      Node::jointNuOfFwdDyn = true;
    }
  };

  // specialisation: if nu(fd) of node's base joint is false, just init nu(fd) = fd. nu(fd) is not impacted by the parent.
  template <typename Robot, int node_id>
  struct inheritNuFwdDyn<Robot, node_id, false>
  {
    static void run()
    {
      typedef typename Nodes<Robot, node_id>::type Node;
      Node::jointNuOfFwdDyn = Node::jointFwdDyn;
    }
  };

  // common template for valid parent_id
  template <typename Robot, int parent_id, int node_id>
  struct updateNuOfFwdDynFromParentOrLocal
  {
    static void run()
    {
      typedef typename Nodes<Robot, parent_id>::type Parent;
      Parent::jointNuOfFwdDyn? 
	inheritNuFwdDyn<Robot, node_id, true>::run() :
	inheritNuFwdDyn<Robot, node_id, false>::run();
    }
  };
  
  // If parent_id is NO_PARENT, just init nu(fd) = fd. nu(fd) is not impacted by the parent.
  template <typename Robot, int node_id>
  struct updateNuOfFwdDynFromParentOrLocal<Robot, NO_PARENT, node_id>
  {
    static void run()
    {
      typedef typename Nodes<Robot, node_id>::type Node;
      Node::jointNuOfFwdDyn = Node::jointFwdDyn;
    }
  };

  // On top of that we define the Visitor
  template <typename Robot, int node_id>
  struct InitNuFwdDynVisitor
  {
    static void discover()
    {
      typedef typename Nodes<Robot, node_id>::type Node;
      updateNuOfFwdDynFromParentOrLocal<Robot, Node::parent_id, node_id>::run();
    }
    static void finish() {}
  };

} // end of namespace metapod::internal

  /// init the "nuFwdDyn" parameter for each joint (use Visitor defined above)
  template< typename Robot > struct initNuFwdDyn
  {
    static void run()
    {
      depth_first_traversal< internal::InitNuFwdDynVisitor, Robot >::run();
    }
  };

} // end of namespace metapod

#endif
