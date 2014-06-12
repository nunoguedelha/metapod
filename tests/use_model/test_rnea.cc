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

// This test applies the rnea on a test model with a reference configuration,
// then compares the computed torques with the reference torques

// Common test tools
#include "common.hh"
#include <metapod/algos/rnea.hh>
#include <metapod/algos/crba.hh>

using namespace metapod;

typedef double LocalFloatType;
BOOST_AUTO_TEST_CASE (test_rnea)
{
  typedef CURRENT_MODEL_ROBOT<LocalFloatType> Robot;
  // Set configuration vectors (q, dq, ddq) to reference values.
  Robot::confVector q, qInit, dq, ddq, torques, C, ref_torques;

  std::ifstream qconf(TEST_DIRECTORY "/q.conf");
  std::ifstream qInitconf(TEST_DIRECTORY "/q_init.conf");
  std::ifstream dqconf(TEST_DIRECTORY "/dq.conf");
  std::ifstream ddqconf(TEST_DIRECTORY "/chdaDdq.ref");

  initConf< Robot >::run(qconf, q);
  initConf< Robot >::run(qInitconf, qInit);
  initConf< Robot >::run(dqconf, dq);
  initConf< Robot >::run(ddqconf, ddq);

  qconf.close();
  qInitconf.close();
  dqconf.close();
  ddqconf.close();

  Robot robot;
  
  // Apply the RNEA to the metapod multibody and print the result in a log file.
  rnea< Robot, true >::run(robot, q, dq, ddq);
  const char result_file[] = "rnea.log";
  std::ofstream log(result_file, std::ofstream::out);
  printTorques<Robot>(robot, log);
  log.close();

  // Compare results with reference file
  compareLogs(result_file, TEST_DIRECTORY "/rnea.ref", 1e-3);
  
  // smoke test: torques variable value is not checked
  getTorques(robot, torques);
  std::ifstream torquesconf(TEST_DIRECTORY "/rnea.ref");
  initConf< Robot >::run(torquesconf, ref_torques);
  BOOST_CHECK(ref_torques.isApprox(torques, 1e-3));
  
  /************* check computationof C *************************/
  /*   ( compare torques given by Tau = H.ddq + C )            */
  
  // compute H
  crba<Robot, true>::run(robot, q);
  
  // Compare results with reference file
  std::ofstream log_H("rnea_H.log", std::ofstream::out);
  log_H << "generalized_mass_matrix\n" << robot.H << std::endl;
  log_H.close();
  compareLogs("rnea_H.log", TEST_DIRECTORY "/chdaH.ref", 1e-3);
  
  // compute C
  rnea< Robot, true, false >::run(robot, q, dq, Robot::confVector::Zero());
  getTorques(robot, C);
  
  // compute torques, this time do it from Tau = H*ddq + C
  torques = robot.H * ddq + C;
  
  // Compare results with reference file
  std::ofstream log_torques("rnea_from_DynEquation.log", std::ofstream::out);
  printConf<Robot>(torques, log_torques);
  log_torques.close();
  compareLogs("rnea_from_DynEquation.log", TEST_DIRECTORY "/rnea.ref", 1e-3);
  
  // check torques values
  //BOOST_CHECK(ref_torques.isApprox(torques, 1e-3));
}
