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

// This test loads configuration vectors from disk then write them back and
// check their content is identical. It tests both initConf and printConf.

// Common test tools
#include "common.hh"
#include <metapod/tools/initnufwddyn.hh>

using namespace metapod;

typedef double LocalFloatType;

BOOST_AUTO_TEST_CASE (test_initnufwddyn)
{
  typedef CURRENT_MODEL_ROBOT<LocalFloatType> CURRENT_MODEL_ROBOT_LFT;
  CURRENT_MODEL_ROBOT_LFT robot;
  // Apply the nu(fd) computation to the metapod multibody and print the result in a log file.
  initNuFwdDyn<CURRENT_MODEL_ROBOT_LFT>::run(robot);

  // Write it to a log file. "true" bool value is written as "1", "false" is written as "0", as per reference file format.
  std::ofstream nufwddyn_log("nufwddyn.log", std::ofstream::out);
  PrintNuFwdDynVisitor<CURRENT_MODEL_ROBOT_LFT>(robot, nufwddyn_log);
  nufwddyn_log.close();

  // Compare resulting file with reference file. 
  compareLogs("nufwddyn.log", TEST_DIRECTORY "/nufwddyn.ref", 0);
}
