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
  static void discover()
  {
    typedef typename Nodes<Robot, node_id>::type Node;
    findString(Node::joint_name, log);
    const int NB_DOF = boost::fusion::result_of::value_at_c<typename Robot::NodeVector, node_id>::type::Joint::NBDOF;
    for(int i=0; i<NB_DOF; ++i)
      log >> v[Node::q_idx+i];
  }
  static void finish() {}
};

} // end of namespace metapod::internal

/// init the "nuFwdDyn" parameter for each joint.
template< typename Robot > struct initConf
{
  static void run()
  {
    depth_first_traversal< internal::InitNuFwdDynVisitor, Robot >::run();
  }
};

} // end of namespace metapod

#endif
