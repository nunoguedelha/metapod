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
#include <metapod/algos/crba.hh>
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
  Eigen::Matrix< LocalFloatType, CURRENT_MODEL_ROBOT_LFT::NBDOF, CURRENT_MODEL_ROBOT_LFT::NBDOF > nufdH= robot.H;

  log << "generalized_mass_matrix\n" << robot.H << std::endl;
  log.close();

  // Apply the CRBA to the metapod multibody and print the result in a log file
  crba< CURRENT_MODEL_ROBOT_LFT, true >::run(robot, q); // Update geometry and run the CRBA
  Eigen::Matrix< LocalFloatType, CURRENT_MODEL_ROBOT_LFT::NBDOF, CURRENT_MODEL_ROBOT_LFT::NBDOF > fullH= robot.H;
  
  // Compare results with reference file
  //compareLogs("hcrba.log", "crba.ref", 1e-3);
  
  // compare non zero terms in nufdH with respective terms fullH
  Eigen::Matrix< LocalFloatType, CURRENT_MODEL_ROBOT_LFT::NBDOF, CURRENT_MODEL_ROBOT_LFT::NBDOF > diffH = fullH-nufdH;
  Eigen::Array< LocalFloatType, CURRENT_MODEL_ROBOT_LFT::NBDOF, CURRENT_MODEL_ROBOT_LFT::NBDOF > prodH;
  for(int i=0; i<CURRENT_MODEL_ROBOT_LFT::NBDOF; i++)
  {
    for(int j=0; j<CURRENT_MODEL_ROBOT_LFT::NBDOF; j++)
    {
      prodH(i,j) = nufdH(i,j) * diffH(i,j);
    }
  }
  assert((prodH == 0).all());
}
