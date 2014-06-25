// Copyright 2014
//
// Nuno Guedelha (CNRS)
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

// This test applies the chda (composite hybrid dynamics algorithm) on a test model with a reference configuration,
// then compares the computed torques and accelerations with the reference 
// torques and accelerations

// Common test tools
#include "common.hh"
#include <metapod/algos/chda.hh>
#include <metapod/algos/rnea.hh>

using namespace metapod;

typedef double LocalFloatType;
BOOST_AUTO_TEST_CASE (test_chda)
{
  typedef CURRENT_MODEL_ROBOT<LocalFloatType> Robot;
  // Set configuration vectors (q, dq, ddq, torques) to reference values.
  Robot::confVector q, dq, ddq, torques, ref_torques;
  Robot robot;
  
  std::ifstream qconf(TEST_DIRECTORY "/q.conf");
  std::ifstream dqconf(TEST_DIRECTORY "/dq.conf");
  std::ifstream ddqconf(TEST_DIRECTORY "/chdaDdq.ref");

  initConf<Robot>::run(qconf, q);
  initConf<Robot>::run(dqconf, dq);
  initConf<Robot>::run(ddqconf, ddq);

  rnea<Robot>::run(robot, q, dq, ddq);
  getTorques(robot, ref_torques);
  std::ofstream refTorquesconf(TEST_DIRECTORY "/chdaTorques.conf", std::ofstream::out);
  printConf<Robot>(ref_torques, refTorquesconf);
  refTorquesconf.close();
  std::ifstream torquesconf(TEST_DIRECTORY "/chdaTorques.ref");

  initConf<Robot, HYBRID_DDQ>::run(ddqconf, ddq);
  initConf<Robot, HYBRID_TORQUES>::run(torquesconf, torques);

  qconf.close();
  dqconf.close();
  ddqconf.close();
  torquesconf.close();
  
  // log the ddq and torques configuration updated with FD/ID joint modes
  std::ofstream logTorquesFdIdInit("chdaTorques.conf", std::ofstream::out);
  printConf<Robot>(torques, logTorquesFdIdInit);
  logTorquesFdIdInit.close();
  std::ofstream logDdqFdIdInit("chdaDdq.conf", std::ofstream::out);
  printConf<Robot>(ddq, logDdqFdIdInit);
  logDdqFdIdInit.close();

  
  // Apply the CHDA (Hybrid Dynamics) to the metapod multibody and print the result in a log file.

  chda<Robot>::run(robot, q, dq, ddq, torques);
  
  // Inertia H results
  const char H_result_file[] = "chdaH.log";
  std::ofstream logH(H_result_file, std::ofstream::out);
  logH << "generalized_mass_matrix\n" << robot.H << std::endl;
  logH.close();
  
  const char torques_result_file[] = "chdaTorques.log";
  std::ofstream logTorques(torques_result_file, std::ofstream::out);
  printConf<Robot>(torques, logTorques);
  logTorques.close();
  
  const char ddq_result_file[] = "chdaDdq.log";
  std::ofstream logDdq(ddq_result_file, std::ofstream::out);
  printConf<Robot>(ddq, logDdq);
  logDdq.close();

  // Compare results with reference files
  compareLogs(H_result_file, TEST_DIRECTORY "/chdaH.ref", 1e-3);
  compareLogs(torques_result_file, TEST_DIRECTORY "/chdaTorques.ref", 1e-3);
  compareLogs(ddq_result_file, TEST_DIRECTORY "/chdaDdq.ref", 1e-3);
  
  /*
  // smoke test: torques variable value is not checked
  getTorques(robot, torques);
  getDdQ(robot, ddq);
  std::ifstream chdaTorquesRef(TEST_DIRECTORY "/chdaTorques.ref");
  initConf< Robot >::run(chdaTorquesRef, ref_torques);
  BOOST_CHECK(ref_torques.isApprox(torques, 1e-3));
  chdaTorquesRef.close();
  std::ifstream chdaDdqRef(TEST_DIRECTORY "/chdaDdQ.ref");
  initConf< Robot >::run(chdaDdqRef, ref_ddq);
  BOOST_CHECK(ref_ddq.isApprox(ddq, 1e-3));
  chdaDdqRef.close();
  */
}
