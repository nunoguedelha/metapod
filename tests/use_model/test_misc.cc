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

// For testing miscellaneous operations or commands

// Common test tools
# include "common.hh"
# include "metapod/tools/common.hh"
# include "metapod/tools/jcalc.hh"
# include "metapod/tools/depth_first_traversal.hh"
# include "metapod/tools/backward_traversal_prev.hh"
# include "metapod/tools/qcalc.hh"
# include "metapod/algos/rnea.hh"
# include "metapod/algos/crba.hh"

using namespace metapod;

using namespace Eigen;

typedef double LocalFloatType;
typedef CURRENT_MODEL_ROBOT<LocalFloatType> Robot;

BOOST_AUTO_TEST_CASE (test_misc)
{
  /*************** 1 - TEST MISC METHODS ************************/

  MatrixXd A(3,3);
  A << 4,-1,2, -1,6,0, 2,0,5;
  std::cout << "The matrix A is" << std::endl << A << std::endl;
  LLT<MatrixXd> lltOfA(A); // compute the Cholesky decomposition of A
  MatrixXd L = lltOfA.matrixL(); // retrieve factor L in the decomposition
  // The previous two lines can also be written as "L = A.llt().matrixL()"
  std::cout << "The Cholesky factor L is" << std::endl << L << std::endl;
  std::cout << "To check this, let us compute L * L.transpose()" << std::endl;
  std::cout << L * L.transpose() << std::endl;
  std::cout << "This should equal the matrix A" << std::endl;

  Eigen::FullPivHouseholderQR<Matrix3d> decA(A);
  std::cout << "is invertible?\n" << (decA.isInvertible()? "true" : "false") << std::endl;
  
  VectorXd B(3,1);
  B << 4, 11, 7;
  std::cout << "The matrix B is" << std::endl << B << std::endl;
  std::cout << "Solution to A.x = B is" << std::endl << lltOfA.solve(B) << std::endl;
  
  /******************* REFERENCE DATA ***************************/
  
  typedef CURRENT_MODEL_ROBOT<LocalFloatType> Robot;
  
  // Set configuration vectors (q, dq, refDdq, refTorques) to reference values.
  Robot::confVector q, dq, refDdq, refTorques;
  
  std::ifstream qconf(TEST_DIRECTORY "/q.conf");
  std::ifstream dqconf(TEST_DIRECTORY "/dq.conf");
  std::ifstream ddqconf(TEST_DIRECTORY "/chdaDdq.ref");
  std::ifstream torquesconf(TEST_DIRECTORY "/chdaTorques.ref");
  
  initConf<Robot>::run(qconf, q);
  initConf<Robot>::run(dqconf, dq);
  initConf<Robot>::run(ddqconf, refDdq);
  initConf<Robot>::run(torquesconf, refTorques);
  
  qconf.close();
  dqconf.close();
  ddqconf.close();
  torquesconf.close();
  
  Robot robot;
  
  /*************** 2 - TEST H11 AND H11 DECOMPOSITION ***********/
  
  std::cout << "2 - TEST H11 AND H11 DECOMPOSITION\n";
  
  typedef Eigen::Matrix<Robot::RobotFloatType, Robot::nbFdDOF, Robot::nbFdDOF> MatrixDof11;
  typedef Eigen::Matrix<Robot::RobotFloatType, Robot::nbFdDOF, 1> confVectorDof1;
  
  // compute H11
  qcalc< Robot >::run(); // Apply the permutation matrix Q
  crba<Robot, true>::run(robot, q);
  Robot::MatrixNBDOFf Hrff = Robot::Q * robot.H * Robot::Qt; // H reordered
  MatrixDof11 Hprime = Hrff.topLeftCorner<Robot::nbFdDOF, Robot::nbFdDOF>(); // H11, square matrix of size "nbFdDOF x nbFdDOF"
  std::cout << std::endl << "Hprime:\n" << Hprime << std::endl;
  std::cout << std::endl << "Hprime symetrique?\n" << Hprime.transpose() - Hprime << std::endl;
  
  // compute refDdq11
  confVectorDof1 refDdq11 = Robot::confVector(Robot::Q * refDdq).head(Robot::nbFdDOF);
  
  // compute Cprime
  confVectorDof1 Cprime;
  Cprime = Hprime * refDdq11;
  std::cout << std::endl << "Cprime:\n" << Cprime << std::endl;
  
  // compute ddq through LLT solving
  confVectorDof1 ddq, diffDdq;
  
  int decomposition = 1;
  switch (decomposition)
    {
    case 1:
      {
	Eigen::LLT<MatrixDof11> lltOfHprime(Hprime); // compute the Cholesky decomposition of Hprime
	ddq = lltOfHprime.solve(Cprime);
	break;
      }
    case 2:
      {
	Eigen::LDLT<MatrixDof11> ldltOfHprime(Hprime); // compute the Cholesky decomposition of Hprime
	ddq = ldltOfHprime.solve(Cprime);
	break;
      }
    case 3:
      {
	Eigen::FullPivHouseholderQR<MatrixDof11> qrOfHprime(Hprime); // compute the Cholesky decomposition of Hprime
	ddq = qrOfHprime.solve(Cprime);
	break;
      }
    default:
      break;
    }
  
  diffDdq = ddq - refDdq11;
  std::cout << diffDdq.squaredNorm();
  
  // ******** TEST compareLogs ***************************/
  
  compareLogs("matrix1.log", "matrix2.log", 1e-3);
}
