// Copyright 2012, 2013
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
 * Implementation of Table structure, element setter and getter.
 */

#ifndef METAPOD_TABLE_HH
# define METAPOD_TABLE_HH

# include <metapod/tools/depth_first_traversal.hh>

namespace metapod {
  namespace internal {
    template < int val > class Elem0 {static const int value = val;};
    template < int val > class Elem1 {static const int value = val;};
    template < int val > class Elem2 {static const int value = val;};
    template < int val > class Elem3 {static const int value = val;};

    template < int elem_index, int val > struct GetElem {};
    template < int val > struct GetElem<0, val> {typedef Elem0<val> type;};
    template < int val > struct GetElem<1, val> {typedef Elem1<val> type;};
    template < int val > struct GetElem<2, val> {typedef Elem2<val> type;};
    template < int val > struct GetElem<3, val> {typedef Elem3<val> type;};
    
  } // end of namespace metapod::internal
  
  struct Table
  {
    template < int elem_index, int val > struct Elem
    {
      typedef typename internal::GetElem<elem_index, val>::type type;
    };
  };
  
} // end of namespace metapod

# endif
