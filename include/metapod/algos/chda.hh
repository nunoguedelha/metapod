// Copyright 2014
//
// Nuno Guedelha (LAAS, CNRS)
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
 * Implementation of the Composite Rigid Body Algorithm,
 * based on Featherstone's Rigid Body Dynamics Algorithms.
 */

#ifndef METAPOD_MAIN_HDA_HH
# define METAPOD_MAIN_HDA_HH

# include "metapod/tools/common.hh"
# include "metapod/tools/print.hh"
# include "metapod/tools/jcalc.hh"
# include "metapod/tools/depth_first_traversal.hh"
# include "metapod/tools/backward_traversal_prev.hh"
# include "metapod/tools/initnufwddyn.hh"
# include "metapod/tools/qcalc.hh"
# include "metapod/algos/rnea.hh"
# include "metapod/algos/crba.hh"
# include "metapod/algos/hcrba.hh"
# include "metapod/tools/sparseHfromTrackNZs.hh"


/// Templated Hybrid Dynamics Algorithm.
/// Takes the multibody tree type as template parameter,
/// and recursively proceeds on the Nodes.

#ifndef GRAVITY_CST
#define GRAVITY_CST 981
#endif

namespace metapod {
namespace internal {

  // General use case [0 < nbFdDOF < NBDOF]: we perform the regular Hybrid Dynamics algorithm.

  template< typename Robot, bool jcalc, int gravity, int NBDOF, int nbFdDOF > struct chda_internal
  {
    static void run(Robot& robot,
                    const typename Robot::confVector& q,
                    const typename Robot::confVector& dq,
                    typename Robot::confVector& ddq,
                    typename Robot::confVector& torques,
                    Timer* timer1,
                    Timer* timer2,
                    Timer* timer3,
                    Timer* timer4,
                    Timer* timer5
                    )
    {
      /* below matrices and vectors which reordered such that fwd dynamic joints
       come first, will get the postfix rff (reordered fwd dynamics first).*/

      typedef typename Robot::confVector confVector;
      typedef typename Robot::MatrixNBDOFf MatrixNBDOFf;
      typedef typename Robot::MatrixDof11 MatrixDof11;
      typedef typename Robot::SparseMatrixf SparseMatrixf;

      typedef Eigen::PermutationMatrix<Robot::nbFdDOF, Robot::nbFdDOF, int> PermutationMatrixDof1; // first nbFdDOF lines or columns
      typedef Eigen::PermutationMatrix<Robot::NBDOF-Robot::nbFdDOF, Robot::NBDOF-Robot::nbFdDOF, int> PermutationMatrixDof2; // last NBDOF-nbFdDOF lines or columns

      typedef Eigen::Matrix<typename Robot::RobotFloatType, Robot::nbFdDOF, 1> confVectorDof1;
      typedef Eigen::Matrix<typename Robot::RobotFloatType, Robot::NBDOF-Robot::nbFdDOF, 1> confVectorDof2;
      typedef Eigen::Matrix<typename Robot::RobotFloatType, Robot::NBDOF-Robot::nbFdDOF, Robot::nbFdDOF> MatrixDof21;

      timer1->resume();
      // 1 - compute Cprime = ID(q,q',Qt[0 q2"]) using RNA :
      confVector ddq_rff_1_zeroed = Robot::Q * ddq; // First, reorder ddq

      ddq_rff_1_zeroed.template head<Robot::nbFdDOF>().template setZero(); // Then, set unknown accelerations to 0

      confVector ddq_1_zeroed = Robot::Qt * ddq_rff_1_zeroed; // roll back to original index order

      rnea<Robot, jcalc, gravity>::run(robot, q, dq, ddq_1_zeroed); // compute torques => Cprime

      confVector CprimeTorques; getTorques(robot, CprimeTorques); // get computed torques
      timer1->stop();

      // 2 - compute H11 from Hprime = Q.H.Qt
      timer2->resume();
      hcrba<Robot, false>::run(robot, q); // First, compute whole H
      MatrixNBDOFf Hrff = Robot::Q * robot.H * Robot::Qt; // H reordered
      MatrixDof11 H11 = Hrff.template topLeftCorner<Robot::nbFdDOF, Robot::nbFdDOF>(); // H11, square matrix of size "nbFdDOF x nbFdDOF"
      metapod::updateSparseHfromTrackNZs<Robot>::run(robot, H11); // update sparsity matrix sparseH11 from full H11
      //std::cout << "diff sparsity :\n" << H11 - MatrixDof11(robot.sparseH11) << std::endl;
      timer2->stop();

      timer3->resume();
      // 3 - solve H11*q1" = tau1 - C1prime
      confVectorDof1 tau1 = confVector(Robot::Q * torques).template head<Robot::nbFdDOF>(); // compute tau1: all known torques (nbFdDOF lines)
      confVectorDof1 C1prime = confVector(Robot::Q * CprimeTorques).template head<Robot::nbFdDOF>(); // compute C1prime (nbFdDOF lines)
      // solve system
      //Eigen::LLT<MatrixDof11> lltOfH11(H11);
      robot.lltOfH11.factorize(robot.sparseH11);
      confVectorDof1 ddq1 = robot.lltOfH11.solve(tau1 - C1prime);
      timer3->stop();

      timer4->resume();
      // 4 - compute tau = Cprime + Qt[H11.q1" H21.q1"]
      //     tau = [tau1, tau2]t
      //     tau2 = C2prime + H21.q1"
      confVectorDof2 C2prime = confVector(Robot::Q * CprimeTorques).template tail<Robot::NBDOF-Robot::nbFdDOF>(); // C2prime (NBDOF-nbFdDOF lines)
      MatrixDof21 H21 = Hrff.template bottomLeftCorner<Robot::NBDOF-Robot::nbFdDOF, Robot::nbFdDOF>(); // H21, square matrix of size "NBDOF-nbFdDOF x nbFdDOF"
      confVectorDof2 tau2 = C2prime + H21 * ddq1;
      timer4->stop();

      // 5 - complete output vectors ddq and torques
      timer5->resume();
      confVectorDof2 ddq2 = confVector(Robot::Q * ddq).template tail<Robot::NBDOF-Robot::nbFdDOF>();

      confVector ddqRff;
      ddqRff << ddq1,
                ddq2;
      ddq = Robot::Qt * ddqRff;

      confVector torquesRff;
      torquesRff << tau1,
                    tau2;
      torques = Robot::Qt * torquesRff;
      timer5->stop();
    }
  };

  // Specialization for use case [nbFdDOF = 0]: we perform a full RNEA.
  template< typename Robot, bool jcalc, int gravity, int NBDOF > struct chda_internal<Robot, jcalc, gravity, NBDOF, 0>
  {
    static void run(Robot& robot,
                    const typename Robot::confVector& q,
                    const typename Robot::confVector& dq,
                    typename Robot::confVector& ddq,
                    typename Robot::confVector& torques,
                    Timer* timer1,
                    Timer* timer2,
                    Timer* timer3,
                    Timer* timer4,
                    Timer* timer5
                    )
    {
      timer1->resume();
      rnea<Robot, jcalc, gravity>::run(robot, q, dq, ddq); // compute torques for model robot
      timer1->stop();
      timer4->resume();
      getTorques(robot, torques);                          // get torques from processed model
      timer4->stop();
    }
  };


  // Specialization for use case [nbFdDOF = NBDOF]: we perform a full Forward Dynamics.
  template< typename Robot, bool jcalc, int gravity, int NBDOF > struct chda_internal<Robot, jcalc, gravity, NBDOF, NBDOF>
  {
    static void run(Robot& robot,
                    const typename Robot::confVector& q,
                    const typename Robot::confVector& dq,
                    typename Robot::confVector& ddq,
                    typename Robot::confVector& torques,
                    Timer* timer1,
                    Timer* timer2,
                    Timer* timer3,
                    Timer* timer4,
                    Timer* timer5
                    )
    {
      typedef typename Robot::MatrixNBDOFf MatrixNBDOFf;
      typedef typename Robot::confVector confVector;
      typedef typename Robot::SparseMatrixf SparseMatrixf;

      // we follow the steps of the Hybrid Dynamics algorithm,
      // while ddq1 = ddq, and ddq2 = null vector.
      timer1->resume();
      // 1 - compute C = ID(q,dq,0) using RNA, (all ddq variables are unknown) :
      rnea<Robot, jcalc, gravity>::run(robot, q, dq, confVector::Zero());
      confVector C; getTorques(robot, C);
      timer1->stop();

      timer2->resume();
      // 2 -compute inertia H (H11 = H, H21 = H12 = H22 = null matrixes)
      crba<Robot, false>::run(robot, q);
      MatrixNBDOFf H11 = Robot::Q * robot.H * Robot::Qt; // H reordered
      metapod::updateSparseHfromTrackNZs<Robot>::run(robot, H11); // update sparsity matrix sparseH11 from full H11
      timer2->stop();

      // 3 - solve H*ddq = torques - C
      //Eigen::LLT<typename Robot::MatrixNBDOFf> lltOfH(robot.H);
      //ddq = lltOfH.solve(torques - C);
      confVector tau1 = Robot::Q * torques;
      confVector C1 = Robot::Q * C;
      //SparseMatrixf sparseH11 = robot.perm * SparseMatrixf(robot.sparseH11 * robot.perm.transpose());
      robot.lltOfH11.factorize(robot.sparseH11);
      timer3->resume();
      confVector ddq1 = robot.lltOfH11.solve(tau1 - C1);
      timer3->stop();
      ddq = Robot::Qt * ddq1;
    }
  };

} // end of namespace metapod::internal


// frontend

template< typename Robot, bool jcalc = true, int gravity =  GRAVITY_CST > struct chda
{
  static void run(Robot& robot,
                  const typename Robot::confVector& q,
                  const typename Robot::confVector& dq,
                  typename Robot::confVector& ddq,
                  typename Robot::confVector& torques,
                  Timer* timer1,
                  Timer* timer2,
                  Timer* timer3,
                  Timer* timer4,
                  Timer* timer5
                  )
  {
    internal::chda_internal<Robot, jcalc, gravity, Robot::NBDOF, Robot::nbFdDOF>::run(robot, q, dq, ddq, torques, timer1, timer2, timer3, timer4, timer5);
  }
};

} // end of namespace metapod

#endif
