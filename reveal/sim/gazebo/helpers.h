#ifndef _REVEAL_SIM_GAZEBO_HELPERS_H_
#define _REVEAL_SIM_GAZEBO_HELPERS_H_

#include <string>
#include <vector>

#include "reveal/core/pointers.h"
#include "reveal/core/state.h"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

//-----------------------------------------------------------------------------
namespace Reveal {
//-----------------------------------------------------------------------------
namespace Sim {
//-----------------------------------------------------------------------------
namespace Gazebo {
//-----------------------------------------------------------------------------

class helpers_c {
public:
  /// writes a reveal model into gazebo to update the simulator
  /// @param model the reveal model to write into gazebo
  /// @param world the gazebo world to update
  /// @return true if the operation succeeds OR false if the operation fails
  ///         for any reason
  static bool write_model( Reveal::Core::model_ptr model, gazebo::physics::WorldPtr world );

  /// writes a reveal trial into gazebo to update the simulator
  /// @param trial the reveal trial to write into gazebo
  /// @param experiment the experimental parameters to write into gazebo
  /// @param world the gazebo world to update
  /// @return true if the operation succeeds OR false if the operation fails
  ///         for any reason
  static bool write_trial( Reveal::Core::trial_ptr trial, Reveal::Core::experiment_ptr experiment, gazebo::physics::WorldPtr world );

  /// reads out a gazebo model as a reveal model
  /// @param world the gazebo world to read from
  /// @param model_name the name of the model to read
  /// @param control the map of the controls assigned to the model
  /// @return the reveal model produced by reading from gazebo
  static Reveal::Core::model_ptr read_model( gazebo::physics::WorldPtr world, std::string model_name, std::map<std::string,double> control );

  /// reads out a gazebo state as a reveal model solution
  /// @param world the gazebo world to read from
  /// @param model_list the list of all the models to read into the solution
  /// @param scenario_id the scenario identifier
  /// @return the reveal model solution produced by reading from gazebo
  static Reveal::Core::solution_ptr read_model_solution( gazebo::physics::WorldPtr world, std::vector<std::string> model_list, std::string scenario_id );

  /// read out a gazebo state as a reveal client solution
  /// @param world the gazebo world to read from
  /// @param model_list the list of all the models to read into the solution
  /// @param scenario_id the scenario identifier
  /// @return the reveal client solution produced by reading from gazebo
  static Reveal::Core::solution_ptr read_client_solution( gazebo::physics::WorldPtr world, std::vector<std::string> model_list, std::string scenario_id );

  /// maps a reveal state's angular velocity into a gazebo vector
  /// @param state the reveal state to map from
  /// @return the gazebo compatible position vector produced from state data
  static gazebo::math::Vector3 position( const Reveal::Core::state_c& state );

  /// maps a reveal state's rotation into a gazebo quaternion
  /// @param state the reveal state to map from
  /// @return the gazebo compatible rotation quaternion produced from state data
  static gazebo::math::Quaternion rotation( const Reveal::Core::state_c& state );
  /// maps a reveal state's linear velocity into a gazebo vector
  /// @param state the reveal state to map from
  /// @return the gazebo compatible linear velocity vector produced from state 
  ///         data
  static gazebo::math::Vector3 linear_velocity( const Reveal::Core::state_c& state );

  /// maps a reveal state's angular velocity into a gazebo vector
  /// @param state the reveal state to map from
  /// @return the gazebo compatible angular velocity vector produced from state 
  ///         data
  static gazebo::math::Vector3 angular_velocity( const Reveal::Core::state_c& state );

  /// maps a reveal state into a gazebo pose structure
  /// @param state the reveal state to map from
  /// @return the gazebo compatible pose instance produced from state data
  static gazebo::math::Pose pose( const Reveal::Core::state_c& state );

  /// maps a gazebo position into a reveal state
  /// @param pos the gazebo position vector to map from
  /// @param state the reveal state to map position into
  static void position( const gazebo::math::Vector3 pos, Reveal::Core::state_c& state );

  /// maps a gazebo rotation into a reveal state
  /// @param rot the gazebo rotation quaternion to map from
  /// @param state the reveal state to map rotation into
  static void rotation( const gazebo::math::Quaternion rot, Reveal::Core::state_c& state );

  /// maps a gazebo linear velocity into a reveal state
  /// @param lvel the gazebo linear velocity vector to map from
  /// @param state the reveal state to map linear velocity into
  static void linear_velocity( const gazebo::math::Vector3 lvel, Reveal::Core::state_c& state );

  /// map a gazebo angular velocity into a reveal state
  /// @param avel the gazebo angular velocity vector to map from
  /// @param state the reveal state to map angular velocity into
  static void angular_velocity( const gazebo::math::Vector3 avel, Reveal::Core::state_c& state );

  /// maps a gazebo pose into a reveal state  
  /// @param pose the gazebo pose instance to map from
  /// @param state the reveal state to map pose into
  static void pose( const gazebo::math::Pose pose, Reveal::Core::state_c& state );

  /// gets a gazebo world's simulation time
  /// @param world the gazebo world to read virtual time from
  /// @return the virtual time in seconds as reported by gazebo
  static double sim_time( gazebo::physics::WorldPtr world );

  /// sets a gazebo world's simulation time
  /// @param t the virtual time in seconds to map into gazebo 
  /// @param world the gazebo world to set the virtual time of
  static void sim_time( double t, gazebo::physics::WorldPtr world );

  /// gets a gazebo world's step size
  /// @param world the gazebo world to read the integration step size from
  /// @return the integration step size in seconds as reported by gazebo
  static double step_size( gazebo::physics::WorldPtr world );

  /// gets a gazebo world's real time
  /// @param world the gazebo world to read the real time progress from
  /// @return the real time progress in seconds as reported by gazebo
  static double real_time( gazebo::physics::WorldPtr world );

  /// resets a gazebo world
  /// @param world the gazebo world to reset
  static void reset( gazebo::physics::WorldPtr world );
};

//-----------------------------------------------------------------------------
} // namespace Gazebo
//-----------------------------------------------------------------------------
} // namespace Sim
//-----------------------------------------------------------------------------
} // namespace Reveal
//-----------------------------------------------------------------------------

#endif // _REVEAL_SIM_GAZEBO_HELPERS_H_
