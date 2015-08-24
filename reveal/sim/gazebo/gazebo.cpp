#include "gazebo.h"

#include <signal.h>

#include "reveal/core/console.h"
#include "reveal/core/error.h"
#include "reveal/core/system.h"

#include "reveal/samples/exchange.h"
#include "reveal/core/scenario.h"
#include "reveal/core/experiment.h"

#include <sstream>
#include <limits>
#include <math.h>

//-----------------------------------------------------------------------------
#define GZENVAR_PLUGIN_PATH "GAZEBO_PLUGIN_PATH"
#define GZENVAR_MODEL_PATH "GAZEBO_MODEL_PATH"
//-----------------------------------------------------------------------------
namespace Reveal {
//-----------------------------------------------------------------------------
namespace Sim {
//-----------------------------------------------------------------------------
gazebo_c::gazebo_c( void ) {
  _request_trial = NULL;
  _submit_solution = NULL;
  _ipc_context = NULL;
}
/*
//-----------------------------------------------------------------------------
gazebo_c::gazebo_c( request_trial_f request_trial, submit_solution_f submit_solution, void* ipc_context ) {
  _request_trial = request_trial;
  _submit_solution = submit_solution;
  _ipc_context = ipc_context;
}
*/
//-----------------------------------------------------------------------------
gazebo_c::~gazebo_c( void ) { 

}

//-----------------------------------------------------------------------------
// Signal Handling : Gazebo process exit detection and 
//-----------------------------------------------------------------------------
bool gz_exited;  // exit condition variable for signal handling
//-----------------------------------------------------------------------------
// Note: only one instance of gzserver can run at a time at present, so having
// a static function tied to a global variable for the signal handler is not
// a major issue as of right now.
void gazebo_c::exit_sighandler( int signum ) {
  // update exit condition variable
  gz_exited = true;
  assert( signum );  // to suppress compiler warning of unused variable
}

//-----------------------------------------------------------------------------
std::vector< std::string > gazebo_c::environment_keys( void ) {
  std::vector< std::string > enkeys;
  enkeys.push_back( "HOME" );
  enkeys.push_back( "LD_LIBRARY_PATH" );
  enkeys.push_back( "OGRE_RESOURCE_PATH" );
  enkeys.push_back( "GAZEBO_MASTER_URI" );
  enkeys.push_back( "GAZEBO_MODEL_DATABASE_URI" );
  enkeys.push_back( "GAZEBO_RESOURCE_PATH" );
  enkeys.push_back( GZENVAR_PLUGIN_PATH );
  enkeys.push_back( GZENVAR_MODEL_PATH );
  return enkeys;
}

//-----------------------------------------------------------------------------
gazebo_c::dynamics_e gazebo_c::prompt_dynamics( void ) {

  std::vector< std::string > list;
  list.push_back( "ODE" );
  list.push_back( "Bullet" );
  list.push_back( "DART" );
  list.push_back( "Simbody" );
  
  unsigned choice = Reveal::Core::console_c::menu( "--Dynamics Menu--", "Select a dynamics engine", list );

  if( choice == 0 ) 
    return DYNAMICS_ODE;
  else if( choice == 1 ) 
    return DYNAMICS_BULLET;
  else if( choice == 2 ) 
    return DYNAMICS_DART;
  else if( choice == 3 ) 
    return DYNAMICS_SIMBODY;

  // return a default if fall through for safety
  return DYNAMICS_ODE;
}

//-----------------------------------------------------------------------------
void gazebo_c::print_tuning_menu( void ) {
  printf( "- Tuning Menu -\n" );
  //printf( "TODO\n" );
  printf( "This is a placeholder for future expansion.  Tuning can be currently accomplished by modifying the relevant package as defined by the $REVEAL_PACKAGE_PATH environment variable.  This menu is provided as a hint that an interactive (and restricted) tuning environment can be supported if desired.\n" );
}
/*
//-----------------------------------------------------------------------------
// simulator plugin interface implementation
//-----------------------------------------------------------------------------
extern "C" {        
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
bool gazebo_c::ui_select_configuration( Reveal::Core::scenario_ptr scenario, Reveal::Core::experiment_ptr experiment ) {
  _dynamics = prompt_dynamics();

  return true; 
}

//-----------------------------------------------------------------------------
bool gazebo_c::ui_select_tuning( void ) {
  if( Reveal::Core::console_c::prompt_yes_no( "Would you like to tune the experiment (Y/N)?" ) )
    print_tuning_menu();

  return true; 
}

//-----------------------------------------------------------------------------
bool gazebo_c::build_package( std::string src_path, std::string build_path ) {

  _plugin_path = combine_path( build_path, "gazebo" );
  _model_path = build_path;

  const bool USE_SDF = true;
  _model_path = combine_path( build_path, "models" );
  if( USE_SDF ) {
    _model_path = combine_path( _model_path, "sdf" );
  } else {
    _model_path = combine_path( _model_path, "urdf" );
  }

  Reveal::Core::console_c::printline( src_path );
  Reveal::Core::console_c::printline( build_path );

  // build package
  _package = Reveal::Core::package_ptr( new Reveal::Core::package_c( src_path, build_path ) );

  Reveal::Core::xml_element_ptr manifest = _package->read( "gazebo", "Gazebo" );
  // parse xml
  std::vector<std::string> build_products;
  Reveal::Core::xml_element_ptr element;
  Reveal::Core::xml_attribute_ptr attribute;
  for( unsigned i = 0; i < manifest->elements(); i++ ) {
    element = manifest->element( i );
    if( element->get_name() == "Product" ) {
      attribute = element->attribute( "file" );
      if( !attribute ) continue;  // malformed xml.  probably a bigger error
      // otherwise
      build_products.push_back( attribute->get_value() );
    } else if( element->get_name() == "World" ) {
      attribute = element->attribute( "file" );
      if( !attribute ) continue;  // malformed xml.  probably a bigger error
      // otherwise
      _world_path = combine_path( build_path, "gazebo" );
      _world_path = combine_path( _world_path, attribute->get_value() );
    }
  }

  // configure package
  bool cmake_result = _package->configure();
  if( !cmake_result ) {
    printf( "ERROR: Failed to configure make for experimental package\nExiting\n" );
    return false;
  } else {
    printf( "Package configuration succeeded\n" );
  }

  // build package
  bool make_result = _package->make( build_products );
  if( !make_result ) {
    printf( "ERROR: Failed to build experimental package\nExiting\n" );
    return false;
  } else {
    printf( "Built package\n" );
  }
  return true;
}

//-----------------------------------------------------------------------------
bool gazebo_c::execute( Reveal::Core::authorization_ptr auth, Reveal::Core::scenario_ptr scenario, Reveal::Core::experiment_ptr experiment ) {

//  scenario->print();
//  printf( "eps[%1.24f]\n", experiment->epsilon );

  Reveal::Core::system_c system( Reveal::Core::system_c::CLIENT );
  if( !system.open() ) return false;

  // sanity check
  if( _request_trial == NULL || _submit_solution == NULL || _ipc_context == NULL ) {
    // the simulator has not been properly set up and execute cannot proceed!
    // TODO: error handling
  }

  Reveal::Samples::exchange_c exchg;
  Reveal::Core::trial_ptr trial;
  Reveal::Core::solution_ptr solution;
  std::string msg;

  printf( "launching gzserver\n" );

  // open ipc channels between this instance and the gazebo process 
  _ipc = Reveal::Core::pipe_ptr( new Reveal::Core::pipe_c( system.monitor_port(), _ipc_context ) );

  if( _ipc->open() != Reveal::Core::pipe_c::ERROR_NONE ) {
    // TODO: Error handling
    Reveal::Core::error_c::printline( "ERROR: failed to open ipc" );
  }
  printf( "ipc opened\n" );

  // initialize exit condition variable
  gz_exited = false;

  // install sighandler to detect when gazebo finishes
  // TODO: make sighandler class more compatible with using a member function
  struct sigaction action;
  memset( &action, 0, sizeof(struct sigaction) );
  action.sa_handler = exit_sighandler;
  sigaction( SIGCHLD, &action, NULL );

  printf( "signal handler installed\n" );

  pid_t pid = fork();
  if( pid < 0 ) {
    // fork failed
    // TODO: error handling
  }

  if( pid == 0 ) {
    // child process
    pid = getpid();

    std::string dynamics_param;
    switch( _dynamics ) {
    case Reveal::Sim::gazebo_c::DYNAMICS_ODE:
      dynamics_param = "ode";         break;
    case Reveal::Sim::gazebo_c::DYNAMICS_BULLET:
      dynamics_param = "bullet";      break;
    case Reveal::Sim::gazebo_c::DYNAMICS_DART:
      dynamics_param = "dart";        break;
    case Reveal::Sim::gazebo_c::DYNAMICS_SIMBODY:
      dynamics_param = "simbody";     break;
    }

    printf( "dynamics: %s\n", dynamics_param.c_str() );

    // build the make command line arguments array
    std::vector<std::string> arg_strings;
    arg_strings.push_back( "reveal-gzserver" );
    //arg_strings.push_back( "-u" );  // paused    TODO: remark after debugging
    arg_strings.push_back( "-e" );
    arg_strings.push_back( dynamics_param );
    arg_strings.push_back( "--verbose" ); 
    arg_strings.push_back( _world_path );
    char* const* exec_argv = param_array( arg_strings );

    // build the enviroment variables array
    std::vector<std::string> env_strings = system_environment_vars();
    bool found_plugin_path = false;
    bool found_model_path = false;
    bool found_package_path = false;

    // search the environment variable array for gazebo environment variables
    // and override them if they exist
    for( std::vector<std::string>::iterator it = env_strings.begin(); it != env_strings.end(); it++ ) {
      std::string search_val;
      std::size_t found;
      search_val = GZENVAR_PLUGIN_PATH;
      found = it->find( search_val );
      if( found != std::string::npos ) { 
        // append the package path to the existing environment path
        //if( found == 0 ) {
          //printf( "l_substr: %s\n", it->substr(0,search_val.size()).c_str() );
          //printf( "r_substr: %s\n", it->substr(search_val.size()+1).c_str() );
          std::string l,r;
          l = it->substr( 0,search_val.size() ).c_str();
          r = it->substr( search_val.size()+1 ).c_str();
          std::stringstream ss;
          ss << search_val << '=';
          ss << _plugin_path << ":";
          ss << r;
          *it = ss.str();
        //}
        found_plugin_path = true;
      }
      search_val = GZENVAR_MODEL_PATH;
      found = it->find( search_val );
      if( found != std::string::npos ) { 
        //if( found == 0 ) {
          std::string l,r;
          l = it->substr( 0, search_val.size() ).c_str();
          r = it->substr( search_val.size() + 1 ).c_str();
          std::stringstream ss;
          ss << search_val << '=';
          ss << _model_path << ":";
          ss << r;
          *it = ss.str();
        //}
        found_model_path = true;
      }
    }
    // if the gazebo environment variables do not exist then create them
    if( !found_plugin_path ) {
      // plugin path envar does not exist, so create it
      std::stringstream ss;
      ss << GZENVAR_PLUGIN_PATH << "=" << _plugin_path;
      env_strings.push_back( ss.str() );
    }
    if( !found_model_path ) {
      // model path envar does not exist, so create it
      std::stringstream ss;
      ss << GZENVAR_MODEL_PATH << "=" << _model_path;
      env_strings.push_back( ss.str() );
    }

    //Reveal::Core::console_c::print( env_strings );

    char* const* exec_envars = param_array( env_strings );

    printf( "gazebo_executable: %s\n", GZSERVER_BIN );

    execve( GZSERVER_BIN, exec_argv, exec_envars );

    perror( "execve" );
    exit( EXIT_FAILURE );

  } else {
    // parent process

    // yield for a short while to give gazebo time to spin up
    printf( "reveal client yielding\n" );
    sleep( 1 );

    // write experiment
    exchg.build_server_experiment( msg, auth, scenario, experiment );

    //printf( "writing experiment to gazebo\n" );
    //printf( "experiment->number_of_trials: %d\n", experiment->number_of_trials );
    if( _ipc->write( msg ) != Reveal::Core::pipe_c::ERROR_NONE ) {
      // TODO: trap and recover
    }

    //unsigned trial_index = 0;
    double sample_rate = scenario->sample_rate;
    double time_step = experiment->time_step;
    double EPSILON = experiment->epsilon;
    double trial_start_time, trial_end_time, trial_duration;
    unsigned current_intermediate_trial;

    scenario->print();

    if( time_step < sample_rate ) 
      trial_duration = sample_rate;
    else
      trial_duration = time_step;

    bool get_next_trial, forward_trial, initial_trial = true, terminate = false;

    printf( "Starting Experiment\n" );
    while( true ) {
      get_next_trial = false;
      forward_trial = false;

      if( initial_trial ) {
        // first pass so request initial trial
        trial_start_time = experiment->start_time;
        trial_end_time = trial_start_time + trial_duration;

        trial = Reveal::Core::trial_ptr( new Reveal::Core::trial_c() );
        trial->scenario_id = scenario->id;
        trial->t = trial_start_time;

        get_next_trial = true;
        current_intermediate_trial = 0;
      } else {
        // a solution now exists (it did not on the first pass)
        assert( solution );
        //double delta = fabs( trial_end_time - solution->t  );
        //printf( "delta(next_trial)[%1.24f], eps[%1.24f]\n", delta, EPSILON );

        // if the solution time approximates the end of the trial time
        if( fabs(trial_end_time - solution->t ) <= EPSILON ) {
          // recompute trial times
          trial_start_time = solution->t;
          trial_end_time = trial_start_time + trial_duration;

          trial = Reveal::Core::trial_ptr( new Reveal::Core::trial_c() );
          trial->scenario_id = scenario->id;
          trial->t = trial_start_time;

          get_next_trial = true;
        }
      }

      // The trial data needs to be refreshed by querying the server
      if( get_next_trial ) {
        // if it another trial is due, send it
        //trial->print();

        if( !_request_trial( auth, experiment, trial ) ) {
          //Reveal::Core::console_c::printline( "request_trial failed" );
          // if no trial can be fetched from the server, then the simulator
          // must be terminated
          terminate = true;
        }

        //trial->print();
      }

      // process the command to send to the simulator
      {
        // if the terminate flag was set, then send the command
        if( terminate ) {
          //printf( "(client) issuing terminate command\n" );
          exchg.build_server_command_exit( msg, auth );
          if( _ipc->write( msg ) != Reveal::Core::pipe_c::ERROR_NONE ) {
            // TODO: trap and recover
          }  
        } else {

          // client is block waiting for either a trial or a step command
          if( initial_trial ) {
            forward_trial = true;
            current_intermediate_trial = 0;
          } else if( get_next_trial && current_intermediate_trial++ > experiment->intermediate_trials_to_ignore ) {
            forward_trial = true;
            current_intermediate_trial = 0;
          }

          if( forward_trial ) {
            //printf( "(client) forwarding trial\n" );

            // write trial to gzipc
            exchg.build_server_trial( msg, auth, experiment, trial );

            if( _ipc->write( msg ) != Reveal::Core::pipe_c::ERROR_NONE ) {
              // TODO: trap and recover
            }
          } else {
            // otherwise send a step command

            //printf( "(client) issuing step command\n" );

            exchg.build_server_command_step( msg, auth );

            if( _ipc->write( msg ) != Reveal::Core::pipe_c::ERROR_NONE ) {
              // TODO: trap and recover
            }
          }
        }
      }  // end process command block

      // switch to block waiting and process the response from the simulator
      // when it reactivates the client
      {
        // TODO : encapsulate in ipc
        zmq_pollitem_t channels[1];
        channels[0].socket = _ipc->socket();
        channels[0].fd = 0;
        channels[0].events = ZMQ_POLLIN;
        channels[0].revents = 0;
        int rc = zmq_poll( channels, 1, -1);
        if( rc == -1 ) {
          if( errno == ETERM ) {
            // At least one member of channels refers to a socket whose context
            // was terminated
            printf( "ERROR: ETERM a socket was terminated\n" );
          } else if( errno == EFAULT ) {
            // The provided channels was NULL
            printf( "ERROR: EFAULT a provided channel was NULL\n" );
          } else if( errno == EINTR ) {
            if( gz_exited ) {
              printf( "Detected Gazebo Exit\n" );
              break;
            }
            // else
            printf( "ERROR: EINTR signal interrupted polling\n" );
          }
        }

        if( channels[0].revents & ZMQ_POLLIN ) {
          // read solution from message
          if( _ipc->read( msg ) != Reveal::Core::pipe_c::ERROR_NONE ) {
            // TODO: trap and recover
          }

          // get the solution
          exchg.parse_client_solution( msg, auth, experiment, solution );

          // publish to revealserver if the end of trial is reached
          //if( solution && solution->t == trial_end_time ) {
          if( solution ) {
            //printf( "(client) received solution\n" );
            //double delta = fabs( trial_end_time - solution->t  );
            //printf( "trial_end_time[%1.24f], solution->t[%1.24f], delta(solution)[%1.24f], eps[%1.24f]\n", trial_end_time, solution->t, delta, EPSILON );
            if( fabs(trial_end_time - solution->t ) <= EPSILON ) {
              //printf( "(client) fowarding solution\n" );

              //solution->print();

              // submit the solution to the server
              bool result = _submit_solution( auth, experiment, solution );
              if( !result ) {
                // TODO: error handling
                Reveal::Core::console_c::printline( "submit_solution failed" );
              }
            }
//          } else {
//            // the solution is effectively ignored until the simulator has reached
//            // the desired trial_end_time
          }
        }
      }  // end process response block

      // Theoretically branch will not be entered; however, detecting EINTR is 
      // not fully reliable and this branch is here to ensure exit if the zmq
      // approach fails to cause exit.
      if( gz_exited ) {
        printf( "Detected Gazebo Exit\n" );
        break;
      }

      if( initial_trial ) initial_trial = false;
    }

    printf( "Experiment Complete\n" );

    // uninstall sighandler
    action.sa_handler = SIG_DFL;
    sigaction( SIGCHLD, &action, NULL );

    //close ipc
    _ipc->close();
  }

  return true;

}
//-----------------------------------------------------------------------------
void gazebo_c::set_request_trial( request_trial_f request_trial ) {
  _request_trial = request_trial;
}

//-----------------------------------------------------------------------------
void gazebo_c::set_submit_solution( submit_solution_f submit_solution ) {
  _submit_solution = submit_solution;
}
//-----------------------------------------------------------------------------
void gazebo_c::set_ipc_context( void* context ) {
  _ipc_context = context;
}

//-----------------------------------------------------------------------------
} // extern "C"
//-----------------------------------------------------------------------------
*/
//-----------------------------------------------------------------------------
} // Sim
//-----------------------------------------------------------------------------
} // Reveal
//-----------------------------------------------------------------------------
