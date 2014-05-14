// Copyright 2013,
//
// Olivier Stasse (JRL/LAAS, CNRS/AIST)
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
 * This file contains the includes and class definitions necessary for the
 * whole project.
 */

#ifndef METAPOD_MY_STATIC_ASSERT_HH
# define METAPOD_MY_STATIC_ASSERT_HH


template <bool,typename error> struct STATIC_ASSERT_FAILURE;

template<typename error> struct STATIC_ASSERT_FAILURE<true, error>{};

template<int x> struct static_assert_test{};
#define STATIC_ASSERT( B, error)			\
typedef static_assert_test<sizeof(STATIC_ASSERT_FAILURE<(bool)(B),error>)> \
static_assert_typedef_;

struct NODE0_BAD_INIT_AT_COMPILE_TIME {};
struct NODE1_BAD_INIT_AT_COMPILE_TIME {};
struct NODE2_BAD_INIT_AT_COMPILE_TIME {};

#endif
