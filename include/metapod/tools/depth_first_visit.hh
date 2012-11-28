// Copyright 2012,
//
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

//
// Implementation of the depth first visit algorithm
//
// To use it, create a visitor
//
//     template<Node> class MyVisitor
//     {
//       static void visit(MyArg & arg)
//       {
//         //... do something
//       };
//     }
//
// Then visit each node with it:
//
//     MyArg myarg;
//     depth_first_visit_internal<MyVisitor, simple_human::Robot>::run(myarg);
//
// This will walks down the tree, and call MyVisitor<Node>::visit(myarg) on
// each node.
//
// There is also a variant with no argument (for both run() and visit())

#ifndef METAPOD_DEPTH_FIRST_VISIT_HH
# define METAPOD_DEPTH_FIRST_VISIT_HH

# include "metapod/tools/common.hh"

namespace metapod
{
  template< template <typename AnyNode> class Visitor,
            typename Node >
  struct depth_first_visit_internal
  {
    template<typename Arg>
    static void run(Arg & arg)
    {
      Visitor<Node>::visit(arg);
      depth_first_visit_internal<Visitor, typename Node::Child0>::run(arg);
      depth_first_visit_internal<Visitor, typename Node::Child1>::run(arg);
      depth_first_visit_internal<Visitor, typename Node::Child2>::run(arg);
      depth_first_visit_internal<Visitor, typename Node::Child3>::run(arg);
      depth_first_visit_internal<Visitor, typename Node::Child4>::run(arg);
    }

    static void run()
    {
      Visitor<Node>::visit();
      depth_first_visit_internal<Visitor, typename Node::Child0>::run();
      depth_first_visit_internal<Visitor, typename Node::Child1>::run();
      depth_first_visit_internal<Visitor, typename Node::Child2>::run();
      depth_first_visit_internal<Visitor, typename Node::Child3>::run();
      depth_first_visit_internal<Visitor, typename Node::Child4>::run();
    }
  };

  template< template <typename AnyNode> class Visitor>
  struct depth_first_visit_internal<Visitor, NC >
  {
    template<typename Arg>
    static void run(Arg & arg) {}

    static void run() {}
  };

  template< template <typename Node> class Visitor,
            typename Robot >
  struct depth_first_visit
  {
    template<typename Arg>
    static void run(Arg & arg)
    {
      depth_first_visit_internal<Visitor, typename Robot::Tree>::run(arg);
    }

    static void run()
    {
      depth_first_visit_internal<Visitor, typename Robot::Tree>::run();
    }
  };
}
#endif
