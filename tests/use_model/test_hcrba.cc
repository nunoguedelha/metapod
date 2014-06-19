// Copyright 2011, 2012, 2013
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

// This test applies the hybrid crba (hcrba) on a test model with a reference configuration,
// then compares the computed inertia matrix with the reference inertia matrix.

// Common test tools
#include "common.hh"
#include <metapod/algos/hcrba.hh>

using namespace metapod;

typedef double LocalFloatType;

BOOST_AUTO_TEST_CASE (test_hcrba)
{
  typedef CURRENT_MODEL_ROBOT<LocalFloatType> CURRENT_MODEL_ROBOT_LFT;
  // set configuration vector q to reference value.
  CURRENT_MODEL_ROBOT_LFT::confVector q;
  std::ifstream qconf(TEST_DIRECTORY "/q.conf");
  initConf<CURRENT_MODEL_ROBOT_LFT>::run(qconf, q);
  qconf.close();

  CURRENT_MODEL_ROBOT_LFT robot;

  // Apply the Hybrid CRBA to the metapod multibody and print the result in a log file
  hcrba< CURRENT_MODEL_ROBOT_LFT, true >::run(robot, q); // Update geometry and run the Hybrid CRBA
  const char result_file[] = "hcrba.log";
  std::ofstream log(result_file, std::ofstream::out);

  log << "generalized_mass_matrix\n" << robot.H << std::endl;
  log.close();

  // Compare results with reference file
  compareLogs(result_file, TEST_DIRECTORY "/hcrba.ref", 1e-3);
}
