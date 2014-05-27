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

using namespace metapod;

typedef double LocalFloatType;
BOOST_AUTO_TEST_CASE (test_chda)
{
  typedef CURRENT_MODEL_ROBOT<LocalFloatType> Robot;
  // Set configuration vectors (q, dq, ddq, torques) to reference values.
  Robot::confVector q, dq, ddq, torques, ref_torques;

  std::ifstream qconf(TEST_DIRECTORY "/q.conf");
  std::ifstream dqconf(TEST_DIRECTORY "/dq.conf");
  std::ifstream ddqconf(TEST_DIRECTORY "/ddq.conf");
  std::ifstream torquesconf(TEST_DIRECTORY "/torques.conf");

  initConf<Robot>::run(qconf, q);
  initConf<Robot>::run(dqconf, dq);
  initConf<Robot>::run(ddqconf, ddq);
  initConf<Robot>::run(torquesconf, torques);

  qconf.close();
  dqconf.close();
  ddqconf.close();
  torquesconf.close();

  Robot robot;
  // Apply the CHDA (Hybrid Dynamics) to the metapod multibody and print the result in a log file.
  chda<Robot>::run(robot, q, dq, ddq, torques);
  const char torques_result_file[] = "chdaTorques.log";
  std::ofstream logTorques(torques_result_file, std::ofstream::out);
  printTorques<Robot>(robot, logTorques);
  logTorques.close();

  /*
  const char ddq_result_file[] = "chdaDdq.log";
  std::ofstream logDdq(ddq_result_file, std::ofstream::out);
  logDdq << ddq;
  //printDdq<Robot>(ddq, log);
  logDdq.close();

  // Compare results with reference file
  compareLogs(torques_result_file, TEST_DIRECTORY "/chdaTorques.ref", 1e-3);
  compareLogs(ddq_result_file, TEST_DIRECTORY "/chdaDdQ.ref", 1e-3);
  
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
