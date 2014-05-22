// Copyright 2014
//
// Nuno Guedelha (CNRS/LAAS)
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


#ifndef METAPOD_INITNODEIDCONF_HH
# define METAPOD_INITNODEIDCONF_HH

# include <metapod/tools/depth_first_traversal.hh>

namespace metapod {
namespace internal {

template <typename Robot, int node_id>
struct InitNodeIdConfVisitor
{
  static void discover(typename Robot::confVector &v)
  {
    typedef typename boost::fusion::result_of::value_at_c<typename Robot::NodeVector, node_id>::type Node;
    const typename Robot::RobotFloatType nodeid_jointFwdDyn = node_id + (Node::jointFwdDyn? 0.1 : 0);
    for(int i=0; i<Node::Joint::NBDOF; ++i)
      v[Node::q_idx+i] = nodeid_jointFwdDyn;
  }
  static void finish(typename Robot::confVector &) {}
};

} // end of namespace metapod::internal

/// init a configuration vector with values from text file, formatted
/// as printed by the printConf routine.
template< typename Robot > struct initNodeIdConf
{
  static void run(typename Robot::confVector & v)
  {
    depth_first_traversal< internal::InitNodeIdConfVisitor, Robot >::run(v);
  }
};

} // end of namespace metapod

#endif
