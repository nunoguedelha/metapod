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


#ifndef METAPOD_SPARSEHFROMTRACKNZS_HH
# define METAPOD_SPARSEHFROMTRACKNZS_HH


namespace metapod {
namespace internal {

template <typename Robot>
struct initSparseHfromTrackNZs
{
  typedef typename boost::fusion::result_of::value_at_c<typename Robot::NodeVector, node_id>::type Node;
  static const int NB_DOF = Node::Joint::NBDOF;

  static void discover(std::ifstream &log, typename Robot::confVector &v, const bool &isTorque)
  {
    findString(Node::joint_name, log);
    if( (isTorque && Node::jointFwdDyn) || (!isTorque && !Node::jointFwdDyn) )
    {
      for(int i=0; i<NB_DOF; ++i)
        log >> v[Node::q_idx+i];
    }
    else
    {
      // copy "invalid" configuration to output conf Vector (v[q_idx] to v[q_idx+NB_DOF-1])
      v.template segment<NB_DOF>(Node::q_idx) = Eigen::Matrix<typename Robot::RobotFloatType, NB_DOF, 1>::Constant(-1.1111111);
    }
  }
  static void discover(typename Robot::confVector &log, typename Robot::confVector &v, const bool &isTorque)
  {
    if( (isTorque && Node::jointFwdDyn) || (!isTorque && !Node::jointFwdDyn) )
    {
      // copy log[q_idx..q_idx+NB_DOF-1] to output conf Vector segment v[q_idx..q_idx+NB_DOF-1]
      v.template segment<NB_DOF>(Node::q_idx) = log.template segment<NB_DOF>(Node::q_idx);
    }
    else
    {
      // copy "invalid" configuration to output conf Vector segment v[q_idx..q_idx+NB_DOF-1]
      v.template segment<NB_DOF>(Node::q_idx) = Eigen::Matrix<typename Robot::RobotFloatType, NB_DOF, 1>::Constant(-1.1111111);
    }
  }

  static void finish(std::ifstream &, typename Robot::confVector &, const bool &) {}
  static void finish(typename Robot::confVector &, typename Robot::confVector &, const bool &) {}
};

template <typename Robot, int node_id>
struct InitConfVisitor
{
  static void discover(std::ifstream &log, typename Robot::confVector &v)
  {
    typedef typename Nodes<Robot, node_id>::type Node;
    findString(Node::joint_name, log);
    const int NB_DOF = boost::fusion::result_of::
      value_at_c<typename Robot::NodeVector, node_id>::type::
      Joint::NBDOF;
    for(int i=0; i<NB_DOF; ++i)
      log >> v[Node::q_idx+i];
  }
  static void discover(typename Robot::confVector &log, typename Robot::confVector &v)
  {
    v = log;
  }

  static void finish(std::ifstream &, typename Robot::confVector &) {}
  static void finish(typename Robot::confVector &, typename Robot::confVector &) {}
};

} // end of namespace metapod::internal

template< typename Robot > struct initSparseHfromTrackNZs
{
  static void run(Robot& robot)
  {
    depth_first_traversal< internal::InitConfVisitor, Robot >::run(log, v);
  }
};

template< typename Robot > struct updateSparseHfromTrackNZs
{
  static void run(ConfTypeStreamOrVector & log, typename Robot::confVector & v)
  {
    depth_first_traversal< internal::InitConfVisitor, Robot >::run(log, v);
  }
};


} // end of namespace metapod

#endif
