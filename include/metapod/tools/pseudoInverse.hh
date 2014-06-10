// Copyright 2012, 2013
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
 * Imported from calibrationPTZ matrix operation methods.
 */

#ifndef METAPOD_PSEUDOINVERSE_HH
# define METAPOD_PSEUDOINVERSE_HH

extern "C" {

  void dgesvd_(char const* jobu, char const* jobvt,
	       int const* m, int const* n, double* a, int const* lda,
	       double* s, double* u, int const* ldu,
	       double* vt, int const* ldvt,
	       double* work, int const* lwork, int* info);
  void dgegv_(char *, char *, integer *, doublereal *, integer *,
	      doublereal *, integer *, doublereal *, doublereal *,
	      doublereal *, doublereal *, integer *, doublereal *,
	      integer *, doublereal *, integer *, integer *);
}


struct pseudoInverse
{
  static void run(Eigen::MatrixXd A, Eigen::MatrixXd& inv, double threshold)
  {
    bool toTranspose = false;

    MatrixXd mat = A;

    if (mat.rows() < mat.cols())
      {
	mat.transposeInPlace();
	toTranspose = true;
      }

    const unsigned int NR = mat.rows();
    const unsigned int NC = mat.cols();

    MatrixXd U, VT;
    U.resize(NR,NR);
    U.setZero();
    VT.resize(NC,NC);
    VT.setZero();
    VectorXd s;
    s.resize(min(NR,NC));
    s.setZero();
    char Jobu='A';
    char Jobvt='A';
    const int m = NR;
    const int n = NC;
    int linfo;
    int lda = std::max(m,n);
    int lw=-1;
    {
      double vw;
      dgesvd_(&Jobu, &Jobvt, &m, &n,
	      mat.data(), &lda,
	      0, 0, &m, 0, &n, &vw, &lw, &linfo);
      lw = int(vw)+5;
    }
    VectorXd w;
    w.resize(lw);
    w.setZero();
    int lu = U.rows();
    int lvt = VT.rows();
    dgesvd_(&Jobu, &Jobvt, &m, &n,
	    mat.data(), &lda,
	    s.data(),
	    U.data(),
	    &lu,
	    VT.data(), &lvt,
	    w.data(), &lw, &linfo);

    MatrixXd S;
    S.resize(mat.cols(), mat.rows());
    S.setZero();
    for (int i=0;i<mat.cols();i++)
      {
	for (int j=0; j<mat.rows();j++)
	  {
	    if ((i==j) && (fabs(s(i))>threshold))
	      S(i,i) = 1./s(i);
	    else
	      S(i,j) = 0;
	  }
      }
    MatrixXd tmp1;
    tmp1 = S*(U.transpose());
    inv = (VT.transpose())*tmp1;
    if (toTranspose)
      inv.transposeInPlace();
  }
};

# endif
