// Copyright 2012, 2013, 2014
//
// Sébastien Barthélémy (Aldebaran Robotics)
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

/// Implementation of two algorithms that check whether a node is NP (no
/// parent) or not and if it has a parent or not.

#ifndef METAPOD_HAS_PARENT_HH
# define METAPOD_HAS_PARENT_HH

# include <metapod/tools/common.hh>

namespace metapod
{

  /// \brief  value member variable is true if Node is NP (no parent), and
  /// false otherwise.
  template < typename Robot, int node_id >
  struct is_NO_PARENT
  {
    static const bool value = false;
  };

  template <typename Robot>
  struct is_NO_PARENT<Robot, NO_PARENT>
  {
    static const bool value = true;
  };

  /// \brief type member variable is the parent node type of Node. If Node has no parent,
  /// type is the Node type itself.
  template < typename Robot, typename Node, bool hasParent >
  struct myParent
  {
    typedef typename boost::fusion::result_of::value_at_c<typename Robot::NodeVector, Node::parent_id>::type type;
  };

  template < typename Robot, typename Node >
  struct myParent<Robot, Node, false>
  {
    typedef Node type;
  };

  /// \brief value member variable is false if Node has no parent. That is, if
  /// it's parent is NP (no parent) and true otherwise.
  template < typename Robot, int node_id >
  struct has_parent
  {
    typedef typename Nodes<Robot, node_id>::type Node;
    static const bool value = !is_NO_PARENT<Robot, Node::parent_id>::value;
    typedef typename myParent<Robot, Node, value>::type type;
  };

} // end of namespace metapod.

#endif

