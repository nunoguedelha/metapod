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

  } // end of namespace metapod::internal
  /*
  template <> struct Table::Get<0> {typedef internal::Elem0<T> type;};
  template <> struct Table::Get<1> {typedef internal::Elem1<T> type;};
  template <> struct Table::Get<2> {typedef internal::Elem2<T> type;};
  template <> struct Table::Get<3> {typedef internal::Elem3<T> type;};
  */
  template< typename T > struct Table
  {
    class Elem0 {static const T value;};
    class Elem1 {static const T value;};
    class Elem2 {static const T value;};
    class Elem3 {static const T value;};
    
    template < int elem_index > struct Get {typedef Elem0 type;};
    
    template < int elem_index, T value >
    void Set()
    {
      typedef typename Get<elem_index>::type Elem;
      T Elem::value = value;
    };
  };
  
} // end of namespace metapod

# endif
