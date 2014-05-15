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

// This test creates a meta-table, inits its elements and reads them back. This is for validating write/read operations during compile-time.

// Common test tools
#include "common.hh"
#include <metapod/tools/my_static_assert.hh>
#include <metapod/tools/table.hh>

using namespace metapod;

BOOST_AUTO_TEST_CASE (test_initReadTable)
{
  typedef Table<int> MyTable;
  
  MyTable::set<0, 10>();
  static const int value0 = MyTable::Get<0>::type::value;
  
  MyTable::set<1, 20>();
  static const int value1 = MyTable::Get<0>::type::value;
  
  MyTable::set<2, 30>();
  static const int value2 = MyTable::Get<0>::type::value;
  
  MyTable::set<3, 40>();
  static const int value3 = MyTable::Get<0>::type::value;
  
  // test compile-time pocessing //
  struct TABLE0_BAD_INIT_AT_COMPILE_TIME{};
  struct TABLE1_BAD_INIT_AT_COMPILE_TIME{};
  struct TABLE2_BAD_INIT_AT_COMPILE_TIME{};
  struct TABLE3_BAD_INIT_AT_COMPILE_TIME{};

  STATIC_ASSERT(MyTable::Get<0>::type::value == 10, TABLE0_BAD_INIT_AT_COMPILE_TIME);
  STATIC_ASSERT(MyTable::Get<1>::type::value == 20, TABLE1_BAD_INIT_AT_COMPILE_TIME);
  STATIC_ASSERT(MyTable::Get<2>::type::value == 30, TABLE2_BAD_INIT_AT_COMPILE_TIME);
  STATIC_ASSERT(MyTable::Get<3>::type::value == 40, TABLE3_BAD_INIT_AT_COMPILE_TIME);
  
  cout << MyTable::Get<0>::type::value << "\n";
  cout << MyTable::Get<1>::type::value << "\n";
  cout << MyTable::Get<2>::type::value << "\n";
  cout << MyTable::Get<3>::type::value << "\n";
}
