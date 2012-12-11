// Copyright 2011, 2012,
//
// Maxime Reis (JRL/LAAS, CNRS/AIST)
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
 * This file contains print methods.
 */

#ifndef METAPOD_PRINT_HH
# define METAPOD_PRINT_HH

#include "metapod/tools/common.hh"

namespace metapod
{

// Print state of the robot in a stream.
template < typename Node > struct PrintStateVisitor
{
  static void discover(std::ostream & os)
  {
    os << Node::Body::name << " :\n"
       << "sXp :\n" << Node::Joint::sXp << "\n"
       << "Xt :\n" << Node::Joint::Xt << "\n"
       << "Xj :\n" << Node::Joint::Xj << "\n"
       << "S :\n" << Node::Joint::S << "\n"
       << "dotS :\n" << Node::Joint::dotS << "\n"
       << "iX0 :\n" << Node::Body::iX0 << "\n"
       << "vi :\n" << Node::Body::vi << "\n"
       << "ai :\n" << Node::Body::ai << "\n"
       << "I :\n" << Node::Body::I << "\n"
       << "f :\n" << Node::Joint::f << "\n"
       << "τ :\n" << Node::Joint::torque << "\n"
       << std::endl;
  }
  static void finish(std::ostream & ) {}
};

template< typename Robot >
void printState(std::ostream & os)
{
  depth_first_traversal<PrintStateVisitor, Robot>::run(os);
}

/*
 * Print a conf vector in a stream.
 * Can be used to log a configuration that can later be loaded through the
 * initConf method.
 */
template< typename Tree > void printConf(const VectorN & q,
                                         std::ostream & qlog)
{
  typedef Tree Node;

  VectorN qi = q.segment<Node::Joint::NBDOF>(Node::Joint::positionInConf);

  qlog << Node::Joint::name << "\n" << qi << std::endl;

  printConf<typename Node::Child0>(q, qlog);
  printConf<typename Node::Child1>(q, qlog);
  printConf<typename Node::Child2>(q, qlog);
  printConf<typename Node::Child3>(q, qlog);
  printConf<typename Node::Child4>(q, qlog);
}

template<>
inline void printConf<NC>(const VectorN &, std::ostream &){}

// Print Transforms of the robot bodies in a stream.
template < typename Node > struct PrintTransformsVisitor
{
  static void discover(std::ostream & os)
  {
    os << Node::Body::name << "\n"
       << Node::Body::iX0.E() << "\n"
       << Node::Body::iX0.r().transpose() << "\n"
       << std::endl;
  }

  static void finish(std::ostream & ) {}
};

template< typename Robot >
void printTransforms(std::ostream & os)
{
  depth_first_traversal<PrintTransformsVisitor, Robot>::run(os);
}

// Print Torques of the robot in a stream.
template< typename Tree >
void printTorques(std::ostream & os)
{
  typedef Tree Node;

  os << Node::Joint::name << "\n"
     << Node::Joint::torque << "\n"
     << std::endl;

  printTorques<typename Node::Child0>(os);
  printTorques<typename Node::Child1>(os);
  printTorques<typename Node::Child2>(os);
  printTorques<typename Node::Child3>(os);
  printTorques<typename Node::Child4>(os);
}

template<>
inline void printTorques<NC>(std::ostream &){}

// Get Torques of the robot.
template< typename Tree >
void getTorques(VectorN& torques, unsigned& i)
{
  typedef Tree Node;

  unsigned j = 0;
  while (j < Node::Joint::NBDOF)
  {
    torques[i] = Node::Joint::torque[j];
    ++i;
    ++j;
  }

  getTorques<typename Node::Child0>(torques, i);
  getTorques<typename Node::Child1>(torques, i);
  getTorques<typename Node::Child2>(torques, i);
  getTorques<typename Node::Child3>(torques, i);
  getTorques<typename Node::Child4>(torques, i);
}

template<>
inline void getTorques<NC>(VectorN&, unsigned&){}

} // end of namespace metapod.

#endif
