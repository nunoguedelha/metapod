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

/*
 * Implementation of the Composite Rigid Body Algorithm,
 * based on Featherstone's Rigid Body Dynamics Algorithms.
 */

#ifndef METAPOD_MAIN_HDA_HH
# define METAPOD_MAIN_HDA_HH

# include "metapod/tools/common.hh"
# include "metapod/tools/print.hh";
# include "metapod/tools/jcalc.hh"
# include "metapod/tools/depth_first_traversal.hh"
# include "metapod/tools/backward_traversal_prev.hh"
# include "metapod/tools/initnufwddyn.hh"
# include "metapod/tools/qcalc.hh"
# include "metapod/algos/rnea.hh"
# include "metapod/algos/crba.hh"


/// Templated Hybrid Dynamics Algorithm.
/// Takes the multibody tree type as template parameter,
/// and recursively proceeds on the Nodes.


namespace metapod {
namespace internal {

// helper function: return H11, square matrix of size "nbFdDOF x nbFdDOF", or an empty matrix (null dimension)
// if nbFdDOF = 0.
  template< typename Robot, typename MatrixNBDOFf, typename MatrixDof11, int nbFdDOF >
  struct extractSubMatH11
  {
    static void run(MatrixNBDOFf &Hrff, MatrixDof11 &H11)
    {
       H11 = Hrff.template topLeftCorner<Robot::nbFdDOF, Robot::nbFdDOF>(); // H11, square matrix of size "nbFdDOF x nbFdDOF"
    }
  };

  template< typename Robot, typename MatrixNBDOFf, typename MatrixDof11 >
  struct extractSubMatH11<Robot, MatrixNBDOFf, MatrixDof11, 0>
  {
    static void run(MatrixNBDOFf &, MatrixDof11 &) {} // call to topLeftCorner method with null size would not compile.
                                                      // H11 will not be used anyway in this case.
  };

  // helper function: return tau2, vector of size "NBDOF-nbFdDOF x 1", or an empty matrix (null dimension)
  // if nbFdDOF = NBDOF.
  template< typename Robot, typename MatrixNBDOFf, typename MatrixDof21,
            typename confVector, typename confVectorDof1, typename confVectorDof2,
            int nbIdDOF >
  struct extractSubVectTau2
  {
    static void run(MatrixNBDOFf &Hrff, confVector &CprimeTorques, confVectorDof1 &ddq1, confVectorDof2 &tau2)
    {
      confVectorDof2 C2prime = confVector(Robot::Q * CprimeTorques).template tail<Robot::NBDOF-Robot::nbFdDOF>(); // C2prime (NBDOF-nbFdDOF lines)
      MatrixDof21 H21 = Hrff.template bottomLeftCorner<Robot::NBDOF-Robot::nbFdDOF, Robot::nbFdDOF>(); // H21, square matrix of size "NBDOF-nbFdDOF x nbFdDOF"
      tau2 = C2prime + H21 * ddq1;
    }
  };
  template< typename Robot, typename MatrixNBDOFf, typename MatrixDof21,
            typename confVector, typename confVectorDof1, typename confVectorDof2>
  struct extractSubVectTau2<Robot, MatrixNBDOFf, MatrixDof21, confVector, confVectorDof1, confVectorDof2, 0>
  {
    static void run(MatrixNBDOFf &, confVector &, confVectorDof1 &, confVectorDof2 &) {}
  };

  // helper function: return ddq2, vector of size "NBDOF-nbFdDOF x 1", or an empty matrix (null dimension)
  // if nbFdDOF = NBDOF.
    template< typename Robot, typename confVector, typename confVectorDof2, int nbIdDOF >
    struct extractSubVectDdq2
    {
      static void run(confVector &ddq, confVectorDof2 &ddq2)
      {
         ddq2 = confVector(Robot::Q * ddq).template tail<Robot::NBDOF-Robot::nbFdDOF>();
      }
    };

    template< typename Robot, typename confVector, typename confVectorDof2 >
    struct extractSubVectDdq2<Robot, confVector, confVectorDof2, 0>
    {
      static void run(confVector &, confVectorDof2 &) {} // call to tail method with null size would not compile.
                                                         // ddq2 will not be used anyway in this case.
    };

} // end of namespace metapod::internal


// frontend
template< typename Robot > struct chda
{
  static void run(Robot& robot, 
		  const typename Robot::confVector& q, 
		  const typename Robot::confVector& dq, 
		  typename Robot::confVector& ddq, 
		  typename Robot::confVector& torques
		  )
  {
    /* below matrices and vectors which reordered such that fwd dynamic joints 
       come first, will get the postfix rff (reordered fwd dynamics first).*/
    
    typedef typename Robot::confVector confVector;
    typedef typename Robot::MatrixNBDOFf MatrixNBDOFf;

    typedef Eigen::PermutationMatrix<Robot::nbFdDOF, Robot::nbFdDOF, typename Robot::RobotFloatType> PermutationMatrixDof1; // first nbFdDOF lines or columns
    typedef Eigen::PermutationMatrix<Robot::NBDOF-Robot::nbFdDOF, Robot::NBDOF-Robot::nbFdDOF, typename Robot::RobotFloatType> PermutationMatrixDof2; // last NBDOF-nbFdDOF lines or columns

    typedef Eigen::Matrix<typename Robot::RobotFloatType, Robot::nbFdDOF, 1> confVectorDof1;
    typedef Eigen::Matrix<typename Robot::RobotFloatType, Robot::NBDOF-Robot::nbFdDOF, 1> confVectorDof2;
    typedef Eigen::Matrix<typename Robot::RobotFloatType, Robot::nbFdDOF, Robot::nbFdDOF> MatrixDof11;
    typedef Eigen::Matrix<typename Robot::RobotFloatType, Robot::nbFdDOF, Robot::NBDOF-Robot::nbFdDOF> MatrixDof12;
    typedef Eigen::Matrix<typename Robot::RobotFloatType, Robot::NBDOF-Robot::nbFdDOF, Robot::nbFdDOF> MatrixDof21;


    // we shall use Q and Q sub-matrices Q1, Q2
    qcalc< Robot >::run(); // Apply the permutation matrix Q
    //PermutationMatrixDof1 Q1right = PermutationMatrixDof1(Robot::fdNodesFirst.template topRows<Robot::nbFdDOF>());
    //PermutationMatrixDof1 Q1left = Q1right.transpose();
    //PermutationMatrixDof2 Q2right = PermutationMatrixDof2(Robot::fdNodesFirst.template bottomRows<Robot::NBDOF-Robot::nbFdDOF>());
    //PermutationMatrixDof2 Q2left = Q2right.transpose();
    
    // 1 - compute Cprime = ID(q,q',Qt[0 q2"]) using RNA :
    confVector ddq_rff_1_zeroed = Robot::Q * ddq; // First, reorder ddq
    
    ddq_rff_1_zeroed.template head<Robot::nbFdDOF>().template setZero(); // Then, set unknown accelerations to 0
    
    confVector ddq_1_zeroed = Robot::Qt * ddq_rff_1_zeroed; // roll back to original index order
    
    rnea< Robot, true >::run(robot, q, dq, ddq_1_zeroed); // compute torques => Cprime
    
    confVector CprimeTorques; getTorques(robot, CprimeTorques); // get computed torques
    
    // 2 - compute H11 from Hprime = Q.H.Qt
    crba<Robot, true>::run(robot, q); // First, compute whole H
    MatrixNBDOFf Hrff = Robot::Q * robot.H * Robot::Qt; // H reordered
    MatrixDof11 H11; internal::extractSubMatH11<Robot, MatrixNBDOFf, MatrixDof11, Robot::nbFdDOF>::run(Hrff, H11); // H11, square matrix of size "nbFdDOF x nbFdDOF"
    
    // 3 - solve H11*q1" = tau1 - C1prime
    confVectorDof1 tau1 = confVector(Robot::Q * torques).template head<Robot::nbFdDOF>(); // compute tau1: all known torques (nbFdDOF lines)
    confVectorDof1 C1prime = confVector(Robot::Q * CprimeTorques).template head<Robot::nbFdDOF>(); // compute C1prime (nbFdDOF lines)
    // solve system
    Eigen::LLT<MatrixDof11> lltOfH11(H11);
    confVectorDof1 ddq1 = lltOfH11.solve(tau1 - C1prime);
    
    // 4 - compute tau = Cprime + Qt[H11.q1" H21.q1"]
    //     tau = [tau1, tau2]t
    //     tau2 = C2prime + H21.q1"
    confVectorDof2 tau2;
    internal::extractSubVectTau2<Robot, MatrixNBDOFf, MatrixDof21, confVector, confVectorDof1, confVectorDof2, Robot::NBDOF-Robot::nbFdDOF>
        ::run(Hrff, CprimeTorques, ddq1, tau2); // comuptes intermediate data: C2prime (NBDOF-nbFdDOF lines),
                                                // H21 square matrix of size "NBDOF-nbFdDOF x nbFdDOF".

    // 5 - complete output vectors ddq and torques
    confVectorDof2 ddq2;
    internal::extractSubVectDdq2<Robot, confVector, confVectorDof2, Robot::NBDOF-Robot::nbFdDOF>::run(ddq, ddq2);
    confVector ddqRff;
    ddqRff << ddq1,
              ddq2;
    ddq = Robot::Qt * ddqRff;
    
    confVector torquesRff;
    torquesRff << tau1,
                  tau2;
    torques = Robot::Qt * torquesRff;
    /*
    // Here, computation of torques does not use matrix H.
    rnea< Robot, true >::run(robot, q, dq, ddq);
    getTorques(robot, torques); // get final computed torques
    */
  }
};

} // end of namespace metapod

#endif
