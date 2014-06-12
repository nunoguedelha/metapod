// Copyright 2011, 2012, 2013
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

// This test applies the permutation matrix Q computation on a test model with a reference configuration,
// then compares the result with the reference Q matrix.

// Common test tools
#include "common.hh"
#include <metapod/tools/qcalc.hh>
#include <metapod/tools/initnodeidconf.hh>

using namespace metapod;

typedef double LocalFloatType;

BOOST_AUTO_TEST_CASE (test_qcalc)
{
  typedef CURRENT_MODEL_ROBOT<LocalFloatType> CURRENT_MODEL_ROBOT_LFT;

  // Apply the permutation matrix Q computation to the metapod multibody
  qcalc< CURRENT_MODEL_ROBOT_LFT >::run();
  std::cout << "Apply the permutation matrix Q computation to the metapod multibody.\n\n";
  
  // set configuration vector ddq to reference value.
  CURRENT_MODEL_ROBOT_LFT::confVector ddq;
  initNodeIdConf<CURRENT_MODEL_ROBOT_LFT>::run(ddq);
  std::cout << "set configuration vector ddq to initial q_idx values.\n\n";

  // compute re-ordered q_idx vector ddqprime = Q.ddq
  CURRENT_MODEL_ROBOT_LFT::confVector ddqprime;
  ddqprime = CURRENT_MODEL_ROBOT_LFT::Q * ddq;
  std::cout << "compute re-ordered q_idx vector ddqprime = Q.ddq.\n\n";
  
  // Print all results in a log file
  std::cout << "Print all results in a log file\n\n";
  const char result_file[] = "qcalc.log";
  std::ofstream log(result_file, std::ofstream::out);
  log << "permutation matrix Q\n" << CURRENT_MODEL_ROBOT_LFT::Q << std::endl;
  log << "initial vector ddq\n" << ddq << std::endl;
  log << "re-ordered vector ddqprime\n" << ddqprime << std::endl;
  log.close();

  // Compare results with reference file
  compareLogs(result_file, TEST_DIRECTORY "/qcalc.ref", 1e-5);
}
