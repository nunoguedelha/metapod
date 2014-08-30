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

// Implementation of the Differencial Recursive Newton Euler Algorithm,
// based on Featherstone's Rigid Body Dynamics Algorithms.

#ifndef METAPOD_DIFFRNEA_HH
# define METAPOD_DIFFRNEA_HH

# include "metapod/tools/common.hh"
# include "metapod/tools/depth_first_traversal.hh"
# include "metapod/tools/jcalc.hh"

#define GRAVITY_CST 981

namespace metapod {

/// Templated Differencial Recursive Newton-Euler Algorithm.
/// Takes the multibody tree type as template parameter,
/// and recursively proceeds on the Nodes.

template< typename Robot, bool jcalc = true, int gravity =  GRAVITY_CST > struct diffrnea{};

template< typename Robot, int gravity > struct diffrnea< Robot, false, gravity >
{
  typedef typename Robot::RobotFloatType FloatType;
  METAPOD_TYPEDEFS;
  typedef typename Robot::confVector confVector;

  // update body kinematics using data from parent body and joint
  template< int node_id, int parent_id >
  struct update_kinematics
  {
    typedef typename Nodes<Robot, node_id>::type Node;
    typedef typename Nodes<Robot, parent_id>::type Parent;

    static void run(
        Robot & robot,
        const Eigen::Matrix< typename Robot::RobotFloatType, Node::Joint::NBDOF, 1 > & ddqi)
    {
      Node& node = boost::fusion::at_c<node_id>(robot.nodes);
      Parent& parent = boost::fusion::at_c<parent_id>(robot.nodes);
      // iX0 = iXλ(i) * λ(i)X0
      // ai = iXλ(i) * aλ(i) + Si * ddqi
      node.body.ai = sum(node.sXp * parent.body.ai,
                         Motion(node.joint.S.S() * ddqi),
                         Motion::Zero(),
                         Motion::Zero());
    }
  };

  // specialization when parent_id == NO_PARENT
  template< int node_id >
  struct update_kinematics<node_id, NO_PARENT>
  {
    METAPOD_TYPEDEFS;
    typedef typename Nodes<Robot, node_id>::type Node;
    static void run(
        Robot & robot,
        const Eigen::Matrix< typename Robot::RobotFloatType, Node::Joint::NBDOF, 1 > & ddqi)
    {

      Node& node = boost::fusion::at_c<node_id>(robot.nodes);
      // ai = iXλ(i) * aλ(i) + Si * ddqi
      // (with aλ(i) = a0 = 0
      node.body.ai = Motion(node.joint.S.S() * ddqi);
    }

  };
  // update parent force
  template< int node_id, int parent_id >
  struct update_force
  {
    typedef typename Nodes<Robot, node_id>::type Node;
    typedef typename Nodes<Robot, parent_id>::type Parent;

    static void run(Robot & robot)
    {
      Node& node = boost::fusion::at_c<node_id>(robot.nodes);
      Parent& parent = boost::fusion::at_c<parent_id>(robot.nodes);
      // fλ(i) = fλ(i) + λ(i)Xi* * fi
      parent.joint.f = parent.joint.f + node.sXp.applyInv(node.joint.f);
    }
  };

  // specialization when parent_id == NO_PARENT
  template< int node_id >
  struct update_force<node_id, NO_PARENT>
  {
    static void run(Robot&) {}
  };

  template <typename AnyRobot, int node_id> struct DftVisitor
  {
    typedef typename Nodes<AnyRobot, node_id>::type Node;

    static void discover(AnyRobot & robot,
                         const confVector & ,
                         const confVector & ,
                         const confVector & ddq)
    {
      Node& node = boost::fusion::at_c<node_id>(robot.nodes);
      // Extract subvector corresponding to current Node
      const Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > ddqi =
        ddq.template segment<Node::Joint::NBDOF>(Node::q_idx);

      // delegate the actual computation, because the computation is
      // different when the node has a parent and when it does not.
      update_kinematics<node_id, Node::parent_id>::run(robot, ddqi);

      // fi = Ii * ai
      Inertia &I = robot.inertias[node_id];
      node.joint.f = I * node.body.ai;
    }

    METAPOD_HOT
    static void finish(AnyRobot & robot,
                       const confVector & ,
                       const confVector & ,
                       const confVector & )
    {
      Node& node = boost::fusion::at_c<node_id>(robot.nodes);
      // backward computations follow
      // τi = SiT * fi
      node.joint.torque = node.joint.S.S().transpose()
                          * node.joint.f.toVector();
      update_force<node_id, Node::parent_id>::run(robot);
    }

  };

  static void run(Robot & robot,
                  const typename Robot::confVector & q,
                  const typename Robot::confVector & dq,
                  const typename Robot::confVector & ddq)
  {
    depth_first_traversal<DftVisitor, Robot>::run(robot, q, dq, ddq);
  }
};

template< typename Robot, int gravity > struct diffrnea< Robot, true, gravity >
{
  static void run(Robot & robot,
                  const typename Robot::confVector & q,
                  const typename Robot::confVector & dq,
                  const typename Robot::confVector & ddq)
  {
    jcalc< Robot >::run(robot, q, dq);
    diffrnea< Robot, false, gravity >::run(robot, q, dq, ddq);
  }
};

} // end of namespace metapod

#endif
