// Copyright 2014
//

// Common test tools
#include "common.hh"
#include <metapod/tools/common.hh>
#include <metapod/timer/timer.hh>
//#include <stdlib.h>     // srand, rand
//#include <time.h>       // time
//#include <random>

using namespace metapod;

typedef double LocalFloatType;

BOOST_AUTO_TEST_CASE (test_transpositions)
{
  static const int dimMat = 35;
  static const int dimTrans = 7;

//  std::default_random_engine generator;
//  std::uniform_int_distribution<int> distribution(1,dimMat);
//  auto dice = std::bind ( distribution, generator );
//  Eigen::Matrix<int, 1, dimMat> transBase; for(int i=0; i<dimMat; i++) {transBase(0,i)=dice();}

  // generate random permutation indices
  Eigen::Array<float, 1, dimMat> randomVectorf =
      (Eigen::Array<float, 1, dimMat>::Random()).abs()*dimMat; // random float array range [0,100]
  Eigen::Array<int, 1, dimMat> randomVectori;
  for(int i=0; i<dimMat; i++) {randomVectori(0,i) = int(randomVectorf(0,i));} // random int array range [0,100]

  Eigen::Transpositions<dimMat, dimMat, int> transBaseTr(randomVectori.matrix());
  Eigen::Matrix<int, 1, dimMat> transBase = Eigen::Matrix<int, 1, dimMat>::LinSpaced(dimMat, 0, dimMat-1);
  transBase = transBase * transBaseTr;

  std::cout << randomVectorf << std::endl << std::endl
            << randomVectori << std::endl << std::endl
            << transBaseTr.indices() << std::endl << std::endl
            << transBase << std::endl;

  Eigen::Matrix<int, 1, dimTrans> trans = transBase.head(dimTrans);

  // convert to transpositions...
  for(int i=0; i<dimTrans-1; i++)
  {
    // for a transposed vector v(), transposition will move v(i) to v( trans(0,i))
    for(int j=i+1; j<dimTrans; j++)
    {
      // replace, in the remaining trans() elements, any occurence of i by trans(0,i)
      if(trans(0,j) == i) {trans(0,j)=trans(0,i);}
    }
  }

  Eigen::Transpositions<dimTrans, dimTrans, int> tr(trans);
  Eigen::Transpose<Eigen::TranspositionsBase<Eigen::Transpositions<dimTrans, dimTrans, int>>> trInv = tr.inverse();

  // create test matrix
  Eigen::Matrix<double, dimMat, dimMat> mat, matTransposed;
  for(int i=0; i<dimMat; i++)
  {
    for(int j=0; j<dimMat; j++)
    {
      mat(i,j) = double(j + 100*i);
    }
  }

  // transpose and print...
  std::cout << mat << std::endl << std::endl
            << transBase << std::endl << std::endl
            << trans << std::endl << std::endl
            << tr.indices() << std::endl << std::endl;
  mat = mat*tr; std::cout << mat << std::endl << std::endl;
  mat = tr*mat; std::cout << mat << std::endl << std::endl;
  mat = trInv*mat; std::cout << mat << std::endl << std::endl;
  mat = mat*trInv; std::cout << mat << std::endl;


  // performance test
  Timer *timer1 = make_timer();
  timer1->start(); timer1->stop();
  double total_time1_us = timer1->elapsed_wall_clock_time_in_us();
  int loop_count = 100000;

  for(int i=0; i<loop_count; i++)
  {
    timer1->resume();
    mat = mat*tr;
    mat = tr*mat;
    timer1->stop();
  }

  total_time1_us = timer1->elapsed_wall_clock_time_in_us() - total_time1_us;
  std::cout << "Transposition mat*tr average execution time is : " << total_time1_us/double(loop_count) << "µs\n";

  /* Aliasing on transpositions and processing time test results:
   * - mat = tr*mat;   --> although "mat" is an operand of the matrix product, Eigen's default
   *   behaviour is not checking for aliasing and not evaluating the product in a temporary buffer.
   * - matTr = tr*mat; --> here the result is stored in a different variable. Still, there is
   *                       aliasing between the series of transpositions in "tr" as if we did:
   *                       matTr = mat;
   *                       matTr = tr*matTr;
   * - tr*mat;         --> does not affect "mat" content. Still there is the usual aliasing in the
   *                       temporary result buffer.
   * - mat = tr*mat;
   *   mat = mat*tr;   --> take the same processing time, and is faster than "mat = tr*mat*tr".
   * - mat.noalias() = tr*mat --> noalias() does not change the behaviour nor the performance.
   */

  typedef CURRENT_MODEL_ROBOT<LocalFloatType> Robot;
  Robot robot;

  std::cout << Robot::Q.indices() << std::endl;
  Robot::InvOfTranspositionsNbFdDOFi Q1inv(Robot::Q1.inverse());

  std::cout << "Q1 :\n" << Robot::Q1.indices() << std::endl;
  std::cout << "Q*mat :\n" << Robot::Q*mat << std::endl << std::endl;
  mat = Robot::Q1 * mat;
  std::cout << "Q1*mat :\n" << mat << std::endl << std::endl;
  mat = Q1inv * mat;
  std::cout << "Q1inv(Q1*mat) :\n" << mat << std::endl << std::endl;

}
