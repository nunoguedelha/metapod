// Copyright 2011, 2012,
//
// Maxime Reis (JRL/LAAS, CNRS/AIST)
// Antonio El Khoury (JRL/LAAS, CNRS/AIST)
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

/*
 * Implementation of the Composite Rigid Body Algorithm,
 * based on Featherstone's Rigid Body Dynamics Algorithms.
 */

#ifndef METAPOD_CRBA_HH
# define METAPOD_CRBA_HH

# include "metapod/tools/common.hh"

namespace metapod
{
  // Entry point of forward crba.
  template< typename Robot, typename Tree > 
  struct crba_forward_propagation;

  // Not be used -- only by crba_forward_propagation
  template< typename Robot, typename Tree, int nbDofInJoint >
  struct crba_forward_propagation_internal;

  // Entry point of forward crba with a parent
  template< typename Robot, typename Tree, typename Parent > 
  struct crba_forward_propagation_parent;

  // Not be used -- only by crba_forward_propagation_parent
  // This template is specialized for specific joint here for fixed joints
  // where nbDofInJoint = 0.
  template< typename Robot, typename Tree, typename Parent,
	    int nbDofInJoint >
  struct crba_forward_propagation_internal_parent;

  template< typename Robot, typename BI, typename BJ, typename Parent >
  struct crba_backward_propagation;

  template< typename Robot, typename BI, typename BJ, typename Parent,
	  int nbDofInParentJoint >
  struct crba_backward_propagation_internal;

  template< typename Robot, bool jcalc = true > struct crba {};

  template< typename Robot > struct crba< Robot, false >
  {
    static void run(const typename Robot::confVector & )
    {
      crba_forward_propagation< Robot, typename Robot::Tree >::run();
    }
  };

  template< typename Robot > struct crba< Robot, true >
  {
    static void run(const typename Robot::confVector & q)
    {
      jcalc< Robot >::run(q, Robot::confVector::Zero());
      crba_forward_propagation< Robot, typename Robot::Tree >::run();
    }
  };

  template < typename Robot, typename Tree >
  struct crba_forward_propagation
  {
    static void run()
    {
      crba_forward_propagation_internal< Robot, Tree, Tree::Joint::NBDOF >::run();
    }
  };

  template< typename Robot >
  struct crba_forward_propagation< Robot, NC >
  {
    static void run() {}
  };

  // Forward crba for general root joint.
  template < typename Robot, typename Tree, int nbDofInJoint >
  struct crba_forward_propagation_internal
  {
    typedef Tree Node;
    typedef typename Node::Body BI;

    static void run()
    {
      Node::Body::Iic = Node::Body::I;

      crba_forward_propagation_parent< Robot, typename Node::Child0, Node >::run();
      crba_forward_propagation_parent< Robot, typename Node::Child1, Node >::run();
      crba_forward_propagation_parent< Robot, typename Node::Child2, Node >::run();
      crba_forward_propagation_parent< Robot, typename Node::Child3, Node >::run();
      crba_forward_propagation_parent< Robot, typename Node::Child4, Node >::run();

      BI::Joint::F = BI::Iic * Node::Joint::S;
       Robot::H.template block<Node::Joint::NBDOF, Node::Joint::NBDOF>
	 (Node::Joint::positionInConf, Node::Joint::positionInConf)
	 = Node::Joint::S.transpose() * BI::Joint::F;

    }
  };

  // Forward crba for general root joint.
  template < typename Robot, typename Tree >
  struct crba_forward_propagation_internal<Robot,Tree, 0>
  {
    typedef Tree Node;
    typedef typename Node::Body BI;

    static void run()
    {
      Node::Body::Iic = Node::Body::I;

      crba_forward_propagation_parent< Robot, typename Node::Child0, Node >::run();
      crba_forward_propagation_parent< Robot, typename Node::Child1, Node >::run();
      crba_forward_propagation_parent< Robot, typename Node::Child2, Node >::run();
      crba_forward_propagation_parent< Robot, typename Node::Child3, Node >::run();
      crba_forward_propagation_parent< Robot, typename Node::Child4, Node >::run();
    }
  };

  template < typename Robot, typename Tree, typename Parent >
  struct crba_forward_propagation_parent
  {
    static void run()
    {
      crba_forward_propagation_internal_parent< Robot, Tree, Parent, Tree::Joint::NBDOF >::run();
    }
  };

  template < typename Robot, typename Parent >
  struct crba_forward_propagation_parent<Robot, NC, Parent>
  {
    static void run(){}
  };


  // Most general implementation.
  template < typename Robot, typename Tree, typename Parent,
	     int nbDofInJoint >
  struct crba_forward_propagation_internal_parent
  {
    typedef Tree Node;
    typedef typename Node::Body BI;
    typedef typename Parent::Body BJ;
    static void run()
    {
      BI::Iic = BI::I;

      crba_forward_propagation_parent< Robot, typename Node::Child0, Node >::run();
      crba_forward_propagation_parent< Robot, typename Node::Child1, Node >::run();
      crba_forward_propagation_parent< Robot, typename Node::Child2, Node >::run();
      crba_forward_propagation_parent< Robot, typename Node::Child3, Node >::run();
      crba_forward_propagation_parent< Robot, typename Node::Child4, Node >::run();

      BJ::Iic = BJ::Iic
	+ Node::Joint::sXp.applyInv(BI::Iic);
      BI::Joint::F = BI::Iic * Node::Joint::S;
      Robot::H.template block<Node::Joint::NBDOF, Node::Joint::NBDOF>
	(Node::Joint::positionInConf, Node::Joint::positionInConf)
	= Node::Joint::S.transpose() * BI::Joint::F;
      
      crba_backward_propagation< Robot, BI, BI, typename BI::Parent >::run();
    }
  };
  
  // Partial specialization for fixed joint.
  template < typename Robot, typename Tree , typename Parent>
  struct crba_forward_propagation_internal_parent< Robot, Tree, Parent, 0 >
   {
     typedef Tree Node;
     typedef typename Node::Body BI;
     typedef typename Parent::Body BJ;

     static void run()
     {
       BI::Iic = BI::I;
       crba_forward_propagation< Robot, typename Node::Child0 >::run();
       crba_forward_propagation< Robot, typename Node::Child1 >::run();
       crba_forward_propagation< Robot, typename Node::Child2 >::run();
       crba_forward_propagation< Robot, typename Node::Child3 >::run();
       crba_forward_propagation< Robot, typename Node::Child4 >::run();
       
       BJ::Iic = BJ::Iic
	 + Node::Joint::Xt.applyInv(BI::Iic);
     }
   };

  // Specialization for fixed root joint.
  template < typename Robot, typename Tree>
  struct crba_forward_propagation_internal_parent< Robot, Tree, NP, 0 >
   {
     typedef Tree Node;
     static void run()
     {
       Node::Body::Iic = Node::Body::I;
       crba_forward_propagation< Robot, typename Node::Child0 >::run();
       crba_forward_propagation< Robot, typename Node::Child1 >::run();
       crba_forward_propagation< Robot, typename Node::Child2 >::run();
       crba_forward_propagation< Robot, typename Node::Child3 >::run();
       crba_forward_propagation< Robot, typename Node::Child4 >::run();
     }
   };

  template< typename Robot, typename BI, typename BJ, typename Parent >
  struct crba_backward_propagation
  {
    static void run()
    {
      crba_backward_propagation_internal<Robot, BI, BJ,
	Parent, Parent::Joint::NBDOF>::run();
    }
  };
  template< typename Robot, typename BI, typename BJ >
  struct crba_backward_propagation< Robot, BI, BJ, NP >
  {
    static void run() {}
  };
  
  template< typename Robot, typename BI, typename BJ, typename Parent,
	    int nbDofInParentJoint >
  struct crba_backward_propagation_internal
  {
    typedef BI Body_i;
    typedef BJ Body_j;
    typedef typename Body_i::Joint Joint_i;
    typedef typename Body_j::Joint Joint_j;

    static void run()
    {
      Joint_i::F = Joint_j::sXp.mulMatrixTransposeBy(Joint_i::F);

      Robot::H.template
        block< Joint_i::NBDOF, Parent::Joint::NBDOF >
             ( Joint_i::positionInConf, Parent::Joint::positionInConf )
        = Joint_i::F.transpose() * Parent::Joint::S.S();
      Robot::H.template
        block< Parent::Joint::NBDOF, Joint_i::NBDOF >
             ( Parent::Joint::positionInConf, Joint_i::positionInConf )
        = Robot::H.block( Joint_i::positionInConf,
                          Parent::Joint::positionInConf,
                          Joint_i::NBDOF,
                          Parent::Joint::NBDOF ).transpose();
      crba_backward_propagation< Robot,
                                 BI,
                                 Parent,
                                 typename Parent::Parent >::run();
    }
  };

  template< typename Robot, typename BI, typename BJ, typename Parent >
  struct crba_backward_propagation_internal< Robot, BI, BJ, Parent, 0 >
  {  
    static void run() {}
  };

} // end of namespace metapod

#endif
