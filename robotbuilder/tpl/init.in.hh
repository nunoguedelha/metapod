@LICENSE@
// This file has been generated by the metapod robotbuilder library.

#ifndef @LIBRARY_NAME@_INIT_HH
# define @LIBRARY_NAME@_INIT_HH

# ifdef _MSC_VER
#  pragma warning( push )
// disable warning C4251: need to have DLL interface
// disable warning C4099: struct/class discrepancies
// The following warnings are only needed if the FloatType is float, because
// the code generator uses double anyway.
// disable warning C4305 truncation from 'double' to 'float'
// disable warning C4244 conversion from 'double' to 'float', possible loss of data
#  pragma warning( disable: 4251 4099 4305 4244 )
# endif

# include "config.hh"

# include <metapod/tools/common.hh>
# include <metapod/tools/joint.hh>
# include <metapod/tools/initnufwddyn.hh>

// by default, boost fusion vector only provides constructor for vectors with
// up to 10 elements.
# if !defined(FUSION_MAX_VECTOR_SIZE) && (@ROBOT_NB_BODIES@ > 10)
#  define FUSION_MAX_VECTOR_SIZE @ROBOT_NB_BODIES@
# endif
# if defined(FUSION_MAX_VECTOR_SIZE) && (@ROBOT_NB_BODIES@ > FUSION_MAX_VECTOR_SIZE)
// todo: warn or stop
# endif
# include <boost/fusion/sequence.hpp>
# include <boost/fusion/include/sequence.hpp>
# include <boost/fusion/include/vector.hpp>

namespace metapod {

template <typename FloatType>
class @LIBRARY_NAME@_DLLAPI @ROBOT_CLASS_NAME@ {
  METAPOD_TYPEDEFS;
public:
  // the following new/delete operators are only needed if there is a
  // member variable of fixed-size vectorizable Eigen type (or a member
  // having such a member).
  // It's not easy to tell in advance, so let always use the aligned operators.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Global constants or variable of the robot
  enum { NBDOF = @ROBOT_NB_DOF@ };
  enum { NBBODIES = @ROBOT_NB_BODIES@ };

  typedef FloatType RobotFloatType;
  typedef Eigen::Matrix< FloatType, NBDOF, 1 > confVector;

  enum NodeId
  {
@nodeid_enum_definition@
  };

  // children of the root/NP node
  static const int child0_id = @root_child0_id@;
  static const int child1_id = @root_child1_id@;
  static const int child2_id = @root_child2_id@;
  static const int child3_id = @root_child3_id@;
  static const int child4_id = @root_child4_id@;

  // definition of the node classes (except the root/NP node)
@node_type_definitions@

  // vector of the robot nodes
  typedef boost::fusion::vector@ROBOT_NB_BODIES@<
@nodes_type_list@>
  NodeVector;

  // member variables

  // inertias expressed in body frames
  static Inertia inertias[@ROBOT_NB_BODIES@];
  NodeVector nodes;
  Eigen::Matrix< FloatType, NBDOF, NBDOF > H; // used by crba
  Eigen::Matrix< FloatType, 1, NBDOF> fdNodes; // permutation indexes for building Q matrix
  Eigen::Matrix< FloatType, 1, NBDOF> idNodes; // permutation indexes for building Q matrix
  Eigen::Matrix< FloatType, NBDOF, NBDOF > Q; // used by chda
  Eigen::Matrix< FloatType, NBDOF, NBDOF > Cprime; // used by chda

  @ROBOT_CLASS_NAME@():
    H(Eigen::Matrix< FloatType, NBDOF, NBDOF >::Zero()),
    fdNodes(Eigen::Matrix< FloatType, 1, NBDOF>::Zero()),
    idNodes(Eigen::Matrix< FloatType, 1, NBDOF>::Zero()),
    Q(Eigen::Matrix< FloatType, NBDOF, NBDOF >::Zero()),
    Cprime(Eigen::Matrix< FloatType, NBDOF, NBDOF >::Zero())
  {}
};

// map node id to node type
@map_node_id_to_type@

} // closing namespace metapod

# ifdef _MSC_VER
#  pragma warning( pop )
# endif

#endif
