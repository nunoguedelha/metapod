// Copyright 2011, 2012, 2013, 2014
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

/*
 * Implementation of the Hybrid Composite Rigid Body Algorithm,
 * based on Featherstone's Rigid Body Dynamics Algorithms.
 */

#ifndef METAPOD_HCRBA_HH
# define METAPOD_HCRBA_HH

# include "metapod/tools/common.hh"
# include "metapod/tools/jcalc.hh"
# include "metapod/tools/depth_first_traversal.hh"
# include "metapod/tools/backward_traversal_prev.hh"
# include "metapod/tools/rootNode_Of_NuFD.hh"

namespace metapod {

  typedef enum NuOfFwdDynCheck
  {
    NUFD_NOT_CHECKED,
    NUFD_TRUE,
    NUFD_FALSE
  };

namespace internal {

  // helper function: convert bool to NuOfFwdDynCheck enum. This enum is a "switch" to match one 
  // of the specializations of template "hcrba_update_parent_inertia" defined below.
  template < bool jointNuOfFwdDyn > struct bool2NuOfFwdDynCheck {};
  template <> struct bool2NuOfFwdDynCheck<true> {static const NuOfFwdDynCheck value=NUFD_TRUE;};
  template <> struct bool2NuOfFwdDynCheck<false> {static const NuOfFwdDynCheck value=NUFD_FALSE;};

  // helper function: update Parent inertia with the contribution of child Node
  template < typename Robot, int parent_id, int node_id, NuOfFwdDynCheck nuOfFwdDynCheck=NUFD_NOT_CHECKED > struct hcrba_update_parent_inertia {};
  // Instanciation matches this specialization if parent exists (parent_id is not NP)
  // => then check if parent is part of nu(fd)
  template < typename Robot, int parent_id, int node_id >
  struct hcrba_update_parent_inertia<Robot, parent_id, node_id, NUFD_NOT_CHECKED>
  {
    static void run(Robot& robot)
    {
      typedef typename boost::fusion::result_of::value_at_c<typename Robot::NodeVector, parent_id>::type Parent;
      hcrba_update_parent_inertia<Robot, parent_id, node_id, bool2NuOfFwdDynCheck<Parent::jointNuOfFwdDyn>::value >::run(robot);
    }
  };
  // update parent inertia if node is part of nu(fd)
  template < typename Robot, int parent_id, int node_id >
  struct hcrba_update_parent_inertia<Robot, parent_id, node_id, NUFD_TRUE>
  {
    typedef typename Nodes<Robot, parent_id>::type Parent;
    typedef typename Nodes<Robot, node_id>::type Node;
    static void run(Robot& robot)
    {
      Parent& parent = boost::fusion::at_c<parent_id>(robot.nodes);
      Node& node = boost::fusion::at_c<node_id>(robot.nodes);
      parent.body.Iic = parent.body.Iic + node.sXp.applyInv(node.body.Iic);
    }
  };
  // Do nothing if parent_id is not part of nu(fd)
  template < typename Robot, int parent_id, int node_id >
  struct hcrba_update_parent_inertia<Robot, parent_id, node_id, NUFD_FALSE>
  {
    static void run(Robot&) {}
  };
  // Do nothing if parent_id is NO_PARENT
  template < typename Robot, int node_id >
  struct hcrba_update_parent_inertia<Robot, NO_PARENT, node_id, NUFD_NOT_CHECKED> // we specialized nuOfFwdDynCheck in order to avoid 
  {                                                                               // ambiguous instantiation with first specialization
    static void run(Robot&) {}
  };

  // helper function: initialization of Iic if node is part of a subtree supported by at least 1 FD joint ( nu(fd) )
  template < typename Robot, typename AnyNode, bool jointNuOfFwdDyn > struct init_Iic {};
  // specialization if node is part of nu(fd)
  template < typename Robot, typename AnyNode >
  struct init_Iic<Robot, AnyNode, true>
  {
    static void run(Robot& robot)
    {
      AnyNode& ni = boost::fusion::at_c<AnyNode::id>(robot.nodes);
      ni.body.Iic = robot.inertias[AnyNode::id];
    }
  };
  // specialization if node is not part of nu(fd)
  template < typename Robot, typename AnyNode >
  struct init_Iic<Robot, AnyNode, false>
  {
    static void run(Robot&) {}
  };

  // helper function: set Hii if node i is FD
  template < typename Robot, typename StartNode, bool jointFwdDyn=StartNode::jointFwdDyn > struct setHiiFromFnS {};
  template < typename Robot, typename StartNode >
  struct setHiiFromFnS<Robot, StartNode, true>
  {
    static void run(Robot& robot, StartNode& node)
    {
      // Hii = SiT * F
      robot.H.template block<StartNode::Joint::NBDOF, StartNode::Joint::NBDOF>(StartNode::q_idx, StartNode::q_idx)
        = node.joint.S.transpose() * node.joint_F;
    }
  };
  template < typename Robot, typename StartNode >
  struct setHiiFromFnS<Robot, StartNode, false>
  {
    static void run(Robot& robot, StartNode& node) {}
  };

  template < typename Robot, typename NI, typename NJ, bool jointFwdDyn = NI::jointFwdDyn || NJ::jointFwdDyn > struct setHijHjiFromFnS {};
  template < typename Robot, typename NI, typename NJ >
  struct setHijHjiFromFnS<Robot, NI, NJ, true >
  {
    static void run(Robot& robot, NI& ni)
    {
      NJ& nj = boost::fusion::at_c<NJ::id>(robot.nodes);

      robot.H.template
        block< NI::Joint::NBDOF, NJ::Joint::NBDOF >
             ( NI::q_idx, NJ::q_idx )
        = ni.joint_F.transpose() * nj.joint.S.S();               // Hij = FT * Sj
      robot.H.template
        block< NJ::Joint::NBDOF, NI::Joint::NBDOF >
             ( NJ::q_idx, NI::q_idx )
        = robot.H.template
        block< NI::Joint::NBDOF, NJ::Joint::NBDOF >
             ( NI::q_idx, NJ::q_idx ).transpose();               // Hji = HijT
    }
  };

  template < typename Robot, typename NI, typename NJ >
  struct setHijHjiFromFnS<Robot, NI, NJ, false >
  {
    static void run(Robot&, NI&) {}
  };

  template < typename Robot, typename StartNode, bool jointFwdDyn=StartNode::jointFwdDyn > struct startNodeId_2_endNodeId {};
  template < typename Robot, typename StartNode >
  struct startNodeId_2_endNodeId<Robot, StartNode, true>
  {
    static const int value = NO_PARENT;
  };
  template < typename Robot, typename StartNode >
  struct startNodeId_2_endNodeId<Robot, StartNode, false>
  {
    static const int value = rootNode_Of_NuFD<Robot, StartNode>::parent_id;
  };

  // helper function: if node is in FD, perform all backward computation of Hij values.
  // "i" (StartNode) is the deepest FD node in the nu(fd) subtree to be processed here.
  // "j" is the root FD node in the nu(fd) subtree.
  template < template <typename AnyRobot, int any_node_id, int any_prev_node_id> class BwdtVisitor, 
             typename Robot, typename StartNode, bool jointNuOfFwdDyn=StartNode::jointNuOfFwdDyn > struct backwardHijAcrossNuOfFd {};
  template < template <typename AnyRobot, int any_node_id, int any_prev_node_id> class BwdtVisitor, 
             typename Robot, typename StartNode >
  struct backwardHijAcrossNuOfFd<BwdtVisitor, Robot, StartNode, true>
  {
    static void run(Robot& robot)
    {
      // In below processing, joint_F stands for jFi. joint_F is handled like a buffer,
      // every new value overwrites the previous one (it's a temp parameter).
      // Even if node i is not FD, we will need to compute Hij if node j is FD, so joint_F
      // shall still be needed => compute joint_F wether node i is FD or ID.
      StartNode& node = boost::fusion::at_c<StartNode::id>(robot.nodes);
      node.joint_F = node.body.Iic * node.joint.S;         // F = Iic * Si
      // set Hii if node i is FD
      setHiiFromFnS<Robot, StartNode>::run(robot, node);   // Hii = SiT * F
      
      // Update Hij and Hji traversing the kinematic tree upwards => run a backward_traversal_prev.
      // Start index for j is parent(i) => backward_traversal_prev "start_node_id" param == StartNode::id
      // If node i is FD, run the traversal up to the base node 0 
      //                          => backward_traversal_prev "end_node_id" param = NP
      // If node i is ID, run the traversal up to the root node of nu(FD) 
      //                          => backward_traversal_prev "end_node_id" param = parent of root node of nu(FD)
      // This condition is handled in template "startNodeId_2_endNodeId".
      const int end_node_id = startNodeId_2_endNodeId<Robot, StartNode>::value;
      metapod::backward_traversal_prev< BwdtVisitor, Robot, StartNode::id, end_node_id >::run(robot);
    }
  };
  template < template <typename AnyRobot, int any_node_id, int any_prev_node_id> class BwdtVisitor, 
             typename Robot, typename StartNode >
  struct backwardHijAcrossNuOfFd<BwdtVisitor, Robot, StartNode, false>
  {
    static void run(Robot&) {}
  };

} // end of namespace metapod::internal

// frontend
template< typename Robot, bool jcalc = true > struct hcrba {};
template< typename Robot > struct hcrba<Robot, false>
{
  template <typename AnyRobot, int node_id >
  struct DftVisitor
  {
    typedef typename Nodes<AnyRobot, node_id>::type NI;
    typedef NI Node;
    // Update NJ with data from PrevNJ
    template< typename AnyyRobot, int nj_id, int prev_nj_id >
    struct BwdtVisitor
    {
      typedef typename Nodes<AnyyRobot, nj_id>::type NJ;           // node nj is parent of prev_nj --> "lambda(j)" in Featherstone RBDA table 9.1
      typedef typename Nodes<AnyyRobot, prev_nj_id>::type PrevNJ;  // node prev_nj                 --> "j" in Featherstone RBDA table 9.1
      static void discover(AnyyRobot& robot)
      {
        // get nodes ni, prev_nj. At first iteration (1rst call to BwdtVisitor) : prev_nj = node_id
        NI& ni = boost::fusion::at_c<node_id>(robot.nodes);
        PrevNJ& prev_nj = boost::fusion::at_c<prev_nj_id>(robot.nodes);

        // for below processing, joint_F stands for jFi. joint_F is handled like a buffer,
        // every new value overwrites the previous one (it's a temp parameter).
        ni.joint_F = prev_nj.sXp.mulMatrixTransposeBy(ni.joint_F); // F = lambda(j)Xj* x F
        internal::setHijHjiFromFnS<AnyyRobot, NI, NJ>::run(robot, ni);
      }

      static void finish(AnyyRobot&) {}
    };

    // forward propagation
    static void discover(AnyRobot& robot)
    {
      internal::init_Iic<AnyRobot, Node, Node::jointNuOfFwdDyn>::run(robot);
    }

    static void finish(AnyRobot& robot)
    {
      internal::hcrba_update_parent_inertia<AnyRobot, Node::parent_id, node_id>::run(robot);
      internal::backwardHijAcrossNuOfFd<BwdtVisitor, AnyRobot, Node>::run(robot);
    }
  };

  static void run(Robot& robot, const typename Robot::confVector& )
  {
    depth_first_traversal< DftVisitor, Robot >::run(robot);
  }
  static void run(Robot& robot)
  {
    depth_first_traversal< DftVisitor, Robot >::run(robot);
  }
};

// frontend
template< typename Robot > struct hcrba< Robot, true >
{
  static void run(Robot& robot, const typename Robot::confVector& q)
  {
    jcalc< Robot >::run(robot, q, Robot::confVector::Zero());
    hcrba< Robot, false >::run(robot);
  }
};

} // end of namespace metapod

#endif
