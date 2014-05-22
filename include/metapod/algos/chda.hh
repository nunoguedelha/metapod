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
# include "metapod/tools/jcalc.hh"
# include "metapod/tools/depth_first_traversal.hh"
# include "metapod/tools/backward_traversal_prev.hh"
# include "metapod/tools/initnufwddyn.hh"
# include "metapod/tools/qcalc.hh"
# include "metapod/algos/rnea.hh"
# include "metapod/algos/crba.hh"

namespace metapod {
namespace internal {

/// Templated Hybrid Dynamics Algorithm.
/// Takes the multibody tree type as template parameter,
/// and recursively proceeds on the Nodes.



// helper function: 

} // end of namespace metapod::internal


// frontend
template< typename Robot > struct main_hda()
{
  static void run(Robot& robot, const typename Robot::confVector& q)
  {
    // 1 - compute Cprime = ID(q,q',Qt[0 q2"])
    jcalc< Robot >::run(robot, q, Robot::confVector::Zero());
    crba< Robot, false >::run(robot);

    // 2 - compute H11 from Hprime = Q.H.Qt

    // 3 - solve H11 q1" = tau1 - C1prime

    // 4 - compute tau = Cprime + Qt[H11.q1" H21.q1"]

  }
};

} // end of namespace metapod

#endif
