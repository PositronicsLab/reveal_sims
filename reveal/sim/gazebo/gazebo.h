#ifndef _REVEAL_SIM_GAZEBO_H_
#define _REVEAL_SIM_GAZEBO_H_

#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include "reveal/core/ipc.h"
#include "reveal/core/simulator.h"
#include "reveal/core/package.h"

//-----------------------------------------------------------------------------
namespace Reveal {
//-----------------------------------------------------------------------------
namespace Sim {
//-----------------------------------------------------------------------------

class gazebo_c;
typedef boost::shared_ptr<gazebo_c> gazebo_ptr;

//-----------------------------------------------------------------------------
class gazebo_c : public Reveal::Core::simulator_c {
public:

  /// Enumerated list of possible dynamics engines in gazebo
  enum dynamics_e {
    DYNAMICS_ODE,
    DYNAMICS_BULLET,
    DYNAMICS_DART,
    DYNAMICS_SIMBODY
  };

  /// Default constructor
  gazebo_c( void );

  /// Constructor allowing requisite pointers to be set in construction
  /// @param request_trial pointer to the request trial function
  /// @param submit_solution pointer to the submit solution function
  /// @param context pointer to the parent process' communication context
//  gazebo_c( request_trial_f, submit_solution_f, void* ipc_context );

  /// Destructor
  virtual ~gazebo_c( void );

private:
  Reveal::Core::package_ptr _package; //< the package (scenario) being simulated

  submit_solution_f _submit_solution; //< the submit solution function
  request_trial_f _request_trial;     //< the request trial function
  void* _ipc_context;                 //< the shared communication context

  dynamics_e _dynamics;               //< the dynamics the simulator is using
  std::string _world_path;            //< the path to the gazebo world file
  std::string _model_path;            //< the path to the model files
  std::string _plugin_path;           //< the path to the gazebo plugins
  Reveal::Core::pipe_ptr _ipc;        //< the pipe to the gazebo world plugin

  /// Prompts the user to select the dynamics to use for the current experiment
  /// @return the dynamics the user selected
  dynamics_e prompt_dynamics( void );

  /// Gets the set of gazebo specific system environment variable key
  /// @return the set of environment variable keys relevant to gazebo
  std::vector< std::string > environment_keys( void );

  /// Prints the gazebo tuning menu
  void print_tuning_menu( void ); 

  /// The signal handler to detect the exiting of the gazebo application
  /// @param signum the signal number provided by the posix system
  static void exit_sighandler( int signum );

  // TODO: should add a method to validate what gazebo has loaded for the world
  // with respect to the expectations of a scenario

public:
  // -Reveal::Core::simulator_c interface-

  /// Encapsulates the user interface for all simulator specific configuration
  /// parameters.  Implementation of the Reveal::Core::simulator_c interface
  /// @param scenario the scenario the user has selected previously
  /// @param experiment the experiment that the user is working in
  /// @return returns true if the user interface operations succeeded OR false
  ///         if the operations failed for any reason.
  bool ui_select_configuration( Reveal::Core::scenario_ptr scenario, Reveal::Core::experiment_ptr experiment );

  /// Encapsulates the user interface for all simulator specific tuning
  /// operations.  Implementation of the Reveal::Core::simulator_c interface
  /// @return returns true if the user interface operations succeeded OR false
  ///         if the operations failed for any reason.
  bool ui_select_tuning( void );

  /// Encapsulates the building of the package in a simulator compatible manner.
  /// Implementation of the Reveal::Core::simulator_c interface
  /// @param src_path the path to the package source 
  /// @param build_path the path to the desired package build directory
  /// @return returns true if the build operations succeeded OR false if the 
  ///         operations failed for any reason.
  bool build_package( std::string src_path, std::string build_path );

  /// Encapsulates the execution of the experiment by the simulator.
  /// Implementation of the Reveal::Core::simulator_c interface
  /// @param auth the authorization granted by the server to the client
  /// @param scenario the scenario the simulator is operating on
  /// @param experiment the experiment the simulator is operating on
  /// @return returns true if execution succeeded OR false if execution failed 
  ///         for any reason.
  bool execute( Reveal::Core::authorization_ptr auth, Reveal::Core::scenario_ptr scenario, Reveal::Core::experiment_ptr experiment );

  /// Allows the request trial function to be set by the client.
  /// Implementation of the Reveal::Core::simulator_c interface
  /// @param request_trial the request trial function for the simulator to use
  void set_request_trial( request_trial_f request_trial );

  /// Allows the submit solution function to be set by the client.
  /// Implementation of the Reveal::Core::simulator_c interface
  /// @param submit_solution the submit solution function for the simulator to 
  ///        use
  void set_submit_solution( submit_solution_f submit_solution );

  /// Allows the parent process to set the intercommincation context which is 
  /// shared by all threads under the parent process
  /// @param context the communcation context the simulator will use
  void set_ipc_context( void* context );

};

//-----------------------------------------------------------------------------
} // namespace Sim
//-----------------------------------------------------------------------------
} // namespace Reveal
//-----------------------------------------------------------------------------

#endif // _REVEAL_SIM_GAZEBO_H_
