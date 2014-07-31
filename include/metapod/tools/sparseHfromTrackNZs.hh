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


#ifndef METAPOD_SPARSEHFROMTRACKNZS_HH
# define METAPOD_SPARSEHFROMTRACKNZS_HH

# include <Eigen/Sparse>

namespace metapod {
namespace internal {
} // end of namespace metapod::internal

template< typename Robot > struct initSparseHfromTrackNZs
{
  static void run(Robot& robot) {}
};

template< typename Robot > struct updateSparseHfromTrackNZs
{
  static void run(Robot & robot, typename Robot::MatrixDof11 & h11)
  {
    for(int i=0; i<robot.sparseH11.outerSize(); ++i)
      for(typename Robot::SparseMatrixf::InnerIterator it(robot.sparseH11, i); it; ++it)
      {
        // copy respective value [i,j] from inertia matrix h11[i,j]
        it.valueRef() = h11(it.row(),it.col());
      }
  }
};


} // end of namespace metapod

#endif
