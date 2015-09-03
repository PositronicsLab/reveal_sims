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
  /// @param[in] model the reveal model to write into gazebo
  /// @param[in,out] world the gazebo world to update
  /// @return true if the operation succeeds OR false if the operation fails
  ///         for any reason
  static bool write_model( Reveal::Core::model_ptr model, gazebo::physics::WorldPtr world );

  /// writes a reveal trial into gazebo to update the simulator
  /// @param[in] trial the reveal trial to write into gazebo
  /// @param[in] experiment the experimental parameters to write into gazebo
  /// @param[in,out] world the gazebo world to update
  /// @param[out] models the models read from the trial
  /// @return true if the operation succeeds OR false if the operation fails
  ///         for any reason
  static bool write_trial( Reveal::Core::trial_ptr trial, Reveal::Core::experiment_ptr experiment, gazebo::physics::WorldPtr world, std::vector<Reveal::Core::model_ptr>& models );

  /// reads out a gazebo model as a reveal model
  /// @param[in] world the gazebo world to read from
  /// @param[in] model_name the name of the model to read
  /// @param[in] control the map of the controls assigned to the model
  /// @return the reveal model produced by reading from gazebo
  static Reveal::Core::model_ptr read_model( gazebo::physics::WorldPtr world, std::string model_name, std::map<std::string,double> control, Reveal::Core::model_ptr filter );

  static Reveal::Core::model_ptr read_model( gazebo::physics::WorldPtr world, std::string model_name );

  // TODO: supplant model_list completely with model_filter
  /// reads out a gazebo state as a reveal model solution
  /// @param[in] world the gazebo world to read from
  /// @param[in] model_list the list of all the models to read into the solution
  /// @param[in] scenario_id the scenario identifier
  /// @param[in] model_filters the set of models and the corresponding filtered
  ///            set of coordinates Reveal should maintain
  /// @return the reveal model solution produced by reading from gazebo
  static Reveal::Core::solution_ptr read_model_solution( gazebo::physics::WorldPtr world, std::vector<std::string> model_list, std::string scenario_id, std::vector<Reveal::Core::model_ptr> model_filters );

  // specifically for reading the initial state of the simulation
  static Reveal::Core::solution_ptr read_model_solution( gazebo::physics::WorldPtr world, std::vector<std::string> model_list, std::string scenario_id );

  // TODO: supplant model_list completely with model_filter
  /// read out a gazebo state as a reveal client solution
  /// @param[in] world the gazebo world to read from
  /// @param[in] model_list the list of all the models to read into the solution
  /// @param[in] scenario_id the scenario identifier
  /// @param[in] model_filters the set of models and the corresponding filtered
  ///            set of coordinates Reveal should maintain
  /// @return the reveal client solution produced by reading from gazebo
  static Reveal::Core::solution_ptr read_client_solution( gazebo::physics::WorldPtr world, std::vector<std::string> model_list, std::string scenario_id );

  /// maps a reveal link state's position into a gazebo vector
  /// @param[in] state the reveal link state to map from
  /// @return the gazebo compatible position vector produced from link state 
  ///         data
  static gazebo::math::Vector3 position( const Reveal::Core::link_state_c& state );

  /// maps a reveal link state's rotation into a gazebo quaternion
  /// @param[in] state the reveal link state to map from
  /// @return the gazebo compatible rotation quaternion produced from link state
  ///         data
  static gazebo::math::Quaternion rotation( const Reveal::Core::link_state_c& state );
  /// maps a reveal link state's linear velocity into a gazebo vector
  /// @param[in] state the reveal link state to map from
  /// @return the gazebo compatible linear velocity vector produced from link 
  ///         state data
  static gazebo::math::Vector3 linear_velocity( const Reveal::Core::link_state_c& state );

  /// maps a reveal link state's angular velocity into a gazebo vector
  /// @param[in] state the reveal link state to map from
  /// @return the gazebo compatible angular velocity vector produced from link 
  ///         state data
  static gazebo::math::Vector3 angular_velocity( const Reveal::Core::link_state_c& state );

  /// maps a reveal link state into a gazebo pose structure
  /// @param[in] state the reveal link state to map from
  /// @return the gazebo compatible pose instance produced from link state data
  static gazebo::math::Pose pose( const Reveal::Core::link_state_c& state );

  /// maps a gazebo link position into a reveal link state
  /// @param[in] pos the gazebo position vector to map from
  /// @param[in,out] state the reveal link state to map position into
  static void position( const gazebo::math::Vector3 pos, Reveal::Core::link_state_c& state );

  /// maps a gazebo rotation into a reveal link state
  /// @param[in] rot the gazebo rotation quaternion to map from
  /// @param[in,out] state the reveal link state to map rotation into
  static void rotation( const gazebo::math::Quaternion rot, Reveal::Core::link_state_c& state );

  /// maps a gazebo linear velocity into a reveal link state
  /// @param[in] lvel the gazebo linear velocity vector to map from
  /// @param[in,out] state the reveal link state to map linear velocity into
  static void linear_velocity( const gazebo::math::Vector3 lvel, Reveal::Core::link_state_c& state );

  /// map a gazebo angular velocity into a reveal link state
  /// @param[in] avel the gazebo angular velocity vector to map from
  /// @param[in,out] state the reveal link state to map angular velocity into
  static void angular_velocity( const gazebo::math::Vector3 avel, Reveal::Core::link_state_c& state );

  /// maps a gazebo pose into a reveal link state  
  /// @param[in] pose the gazebo pose instance to map from
  /// @param[in,out] state the reveal link state to map pose into
  static void pose( const gazebo::math::Pose pose, Reveal::Core::link_state_c& state );

  // TODO: begin - update documentation 
  /// maps a reveal joint state's position into a gazebo vector
  /// @param[in] state the reveal joint state to map from
  /// @return the gazebo compatible position vector produced from joint state 
  ///         data
  static double position( const Reveal::Core::joint_state_c& state, unsigned index );

  /// maps a reveal joint state's velocity into a gazebo vector
  /// @param[in] state the reveal joint state to map from
  /// @return the gazebo compatible linear velocity vector produced from joint 
  ///         state data
  static double velocity( const Reveal::Core::joint_state_c& state, unsigned index );

  /// maps a gazebo joint position into a reveal joint state
  /// @param[in,out] state the reveal state to map position into
  /// @param[in] pos the gazebo position vector to map from
  static void position( Reveal::Core::joint_state_c& state, unsigned index, double value );

  /// map a gazebo joint velocity into a reveal joint state
  /// @param[in,out] state the reveal joint state to map velocity into
  /// @param[in] velocity the gazebo velocity vector to map from
  static void velocity( Reveal::Core::joint_state_c& state, unsigned index, double value );
  // TODO: end

  /// gets a gazebo world's simulation time
  /// @param[in] world the gazebo world to read virtual time from
  /// @return the virtual time in seconds as reported by gazebo
  static double sim_time( gazebo::physics::WorldPtr world );

  /// sets a gazebo world's simulation time
  /// @param[in] t the virtual time in seconds to map into gazebo 
  /// @param[in,out] world the gazebo world to set the virtual time of
  static void sim_time( double t, gazebo::physics::WorldPtr world );

  /// gets a gazebo world's step size
  /// @param[in] world the gazebo world to read the integration step size from
  /// @return the integration step size in seconds as reported by gazebo
  static double step_size( gazebo::physics::WorldPtr world );

  /// gets a gazebo world's real time
  /// @param[in] world the gazebo world to read the real time progress from
  /// @return the real time progress in seconds as reported by gazebo
  static double real_time( gazebo::physics::WorldPtr world );

  /// resets a gazebo world
  /// @param[in] world the gazebo world to reset
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
