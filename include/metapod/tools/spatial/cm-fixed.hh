// Copyright 2012,
//
// Olivier STASSE
//
// LAAS, CNRS
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
// GNU General Lesser Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with metapod.  If not, see <http://www.gnu.org/licenses/>.(2);


#ifndef METAPOD_SPATIAL_ALGEBRA_CONSTRAINT_MOTION_FIXED_HH
# define METAPOD_SPATIAL_ALGEBRA_CONSTRAINT_MOTION_FIXED_HH

namespace metapod
{

  namespace Spatial
  {
    class Transform;

    // Class of motion constraint with a fixed frame
    class ConstraintMotionFixed
    {
      public:
        // Constructors
      ConstraintMotionFixed()
      { m_S = vector6_0d::Zero();};

      vector6_0d operator*(const Spatial::Transform &X) const;
      vector6_0d operator*(double d) const;
      vector0d operator*(const Eigen::Matrix< FloatType, 0, 1 > &) const
      { vector0d tmp=vector0d::Zero();
	return tmp; } 

      private:
        vector6_0d m_S;

      public:
      const vector6_0d & S() const {return m_S;}
      vector6_0dt transpose() const {return m_S.transpose();}
    };

    vector6_0d ConstraintMotionFixed::operator*
    (const Spatial::Transform &) const
    {
      vector6_0d tmp = m_S;
      return tmp;                                                   
    }

    vector6_0d ConstraintMotionFixed::operator*
    (double ) const
    {
      vector6_0d tmp = m_S;
      return tmp;                                                   
    }

    template<>
    vector6_0d OperatorMul< vector6_0d, Inertia, ConstraintMotionFixed>::
      mul(const Inertia &,
	  const ConstraintMotionFixed &) const
    {
      vector6_0d r = vector6_0d::Zero();
      return r;
    }
    
    vector6_0d operator*(const Inertia & m,
		       const ConstraintMotionFixed &a) 
    {
      OperatorMul<vector6_0d,Inertia, ConstraintMotionFixed > om;
      return om.mul(m,a);
    }

  }
}

#endif
