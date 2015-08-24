//#include "reveal/sim/gazebo/helpers.h"
#include "helpers.h"

#include <string>
#include <vector>
#include <sstream>

#include "reveal/core/model.h"
#include "reveal/core/experiment.h"
#include "reveal/core/trial.h"
#include "reveal/core/solution.h"

//-----------------------------------------------------------------------------
namespace Reveal {
//-----------------------------------------------------------------------------
namespace Sim {
//-----------------------------------------------------------------------------
namespace Gazebo {
//-----------------------------------------------------------------------------

bool helpers_c::write_model( Reveal::Core::model_ptr model, gazebo::physics::WorldPtr world ) {

  gazebo::physics::ModelPtr gzmodel = world->GetModel( model->id );
  if( !gzmodel ) return false;

  for( unsigned j = 0; j < model->links.size(); j++ ) {
    Reveal::Core::link_ptr link = model->links[j];

    gazebo::physics::LinkPtr gzlink = gzmodel->GetLink( link->id );
    // TODO : refactor assert into something more flexible and valid
    assert( gzlink );

    // map the reveal link state parameters to the gazebo link
    gzlink->SetWorldPose( pose( link->state ) );
    gzlink->SetLinearVel( linear_velocity( link->state ) );
    gzlink->SetAngularVel( angular_velocity( link->state ) );
  }
  for( unsigned j = 0; j < model->joints.size(); j++ ) {
    Reveal::Core::joint_ptr joint = model->joints[j];

    gazebo::physics::JointPtr gzjoint = gzmodel->GetJoint( joint->id );
    // TODO : refactor assert into something more flexible and valid
    assert( gzjoint );

    // NOTE : GetAngleCount is not implemented for Bullet or ODE
    //unsigned max_k = gzjoint->GetAngleCount();
    //for( unsigned k = 0; k < max_k; k++ )
    //  gzjoint->SetForce( k, joint->control[k] );

    // Bullet and ODE only use 1 DOF?
    gzjoint->SetForce( 0, joint->control[0] ); 
  }
  return true;
}

//-----------------------------------------------------------------------------
bool helpers_c::write_trial( Reveal::Core::trial_ptr trial, Reveal::Core::experiment_ptr experiment, gazebo::physics::WorldPtr world ) {

  world->SetSimTime( trial->t );
  world->GetPhysicsEngine()->SetMaxStepSize( experiment->time_step );

  for( unsigned i = 0; i < trial->models.size(); i++ ) {
    Reveal::Core::model_ptr model = trial->models[i];

    bool result = write_model( model, world );
    if( !result ) {
      // TODO : handle and recover.  This branch implies that the model does
      // not exist in the gazebo world for some reason.
    }
  }
  return true;
}

//-----------------------------------------------------------------------------
// TODO : could be improved by actually returning an error code or writing to
// std::err
// returns an empty pointer if it fails to find the model in the gazebo world
Reveal::Core::model_ptr helpers_c::read_model( gazebo::physics::WorldPtr world, std::string model_name, std::map<std::string,double> control ) {

  //printf( "called me\n" );

  Reveal::Core::model_ptr rvlmodel;
  // attempt to find the model in the gazebo world
  gazebo::physics::ModelPtr gzmodel = world->GetModel( model_name );

  // if gazebo could not find the model, return a reveal model null reference
  if( !gzmodel ) return rvlmodel;

  // otherwise, start mapping the model
  rvlmodel = Reveal::Core::model_ptr( new Reveal::Core::model_c() );
  rvlmodel->id = model_name;

  // get all the links in the gazebo model and iterate through them
  gazebo::physics::Link_V gzlinks = gzmodel->GetLinks();
  for( unsigned i = 0; i < gzlinks.size(); i++ ) {

    // create a new reveal link and assign the correlating properties to it
    Reveal::Core::link_ptr link = Reveal::Core::link_ptr( new Reveal::Core::link_c() );
    link->id = gzlinks[i]->GetName();

    // map the gazebo link state parameters into the reveal link
    pose( gzlinks[i]->GetWorldPose(), link->state );
    linear_velocity( gzlinks[i]->GetWorldLinearVel(), link->state );
    angular_velocity( gzlinks[i]->GetWorldAngularVel(), link->state );

    // insert the link into the reveal model
    rvlmodel->insert( link );
    //rvlmodel->push_back( link );
  }

  // get all the joints in the gazebo model and iterate through them
  gazebo::physics::Joint_V gzjoints = gzmodel->GetJoints();
  for( unsigned i = 0; i < gzjoints.size(); i++ ) {

    // read the joint identifier from gazebo
    std::string id = gzjoints[i]->GetName();
    // assume it is mapped
    bool mapped = true;
    // assume a zero control
    double u = 0;
    try{
      // try to get the value stored in the map
      u = control.at( id );
    } catch( std::exception ex ) {
      // if the map failed to return a reference, then the joint is not mapped
      mapped = false;
    }

    // skip this joint if it is not mapped
    if( !mapped ) continue;

    // otherwise, create a joint and assign it the name and control
    Reveal::Core::joint_ptr joint = Reveal::Core::joint_ptr( new Reveal::Core::joint_c() );
    joint->id = gzjoints[i]->GetName();
    // TODO : Note this only currently supports single axis joints
    joint->control[0] = u;
  
    // insert the joint into the reveal model
    rvlmodel->insert( joint );
  }

  // return the reveal model
  return rvlmodel;
}

//-----------------------------------------------------------------------------
Reveal::Core::solution_ptr helpers_c::read_model_solution( gazebo::physics::WorldPtr world, std::vector<std::string> model_list, std::string scenario_id ) {

  // create a model solution
  Reveal::Core::solution_ptr solution = Reveal::Core::solution_ptr( new Reveal::Core::solution_c( Reveal::Core::solution_c::MODEL ) );

  // map required parameters
  solution->scenario_id = scenario_id;
  solution->t = sim_time( world );

  // map the states of all desired models
  for( unsigned i = 0; i < model_list.size(); i++ ) {
    std::map<std::string,double> null_controls;
    Reveal::Core::model_ptr model = read_model( world, model_list[i], null_controls );

    if( model ) solution->models.push_back( model );
  }
  return solution;
}

//-----------------------------------------------------------------------------
Reveal::Core::solution_ptr helpers_c::read_client_solution( gazebo::physics::WorldPtr world, std::vector<std::string> model_list, std::string scenario_id ) {

  // create a client solution
  Reveal::Core::solution_ptr solution = Reveal::Core::solution_ptr( new Reveal::Core::solution_c( Reveal::Core::solution_c::CLIENT ) );

  // map required parameters
  solution->scenario_id = scenario_id;
  solution->t = sim_time( world );
  solution->real_time = real_time( world );

  // map the states of all desired models
  for( unsigned i = 0; i < model_list.size(); i++ ) {
    std::map<std::string,double> null_controls;
    Reveal::Core::model_ptr model = read_model( world, model_list[i], null_controls );

    if( model ) solution->models.push_back( model );
  }
  return solution;
}

//-----------------------------------------------------------------------------
gazebo::math::Vector3 helpers_c::position( const Reveal::Core::state_c& state ) {
  return gazebo::math::Vector3( state.linear_x(), state.linear_y(), state.linear_z() );
}

//-----------------------------------------------------------------------------
gazebo::math::Quaternion helpers_c::rotation( const Reveal::Core::state_c& state ) {
  return gazebo::math::Quaternion( state.angular_w(), state.angular_x(), state.angular_y(), state.angular_z() );
}

//-----------------------------------------------------------------------------
gazebo::math::Vector3 helpers_c::linear_velocity( const Reveal::Core::state_c& state ) {
  return gazebo::math::Vector3( state.linear_dx(), state.linear_dy(), state.linear_dz() );
}

//-----------------------------------------------------------------------------
gazebo::math::Vector3 helpers_c::angular_velocity( const Reveal::Core::state_c& state ) {
  return gazebo::math::Vector3( state.angular_dx(), state.angular_dy(), state.angular_dz() );
}
  
//-----------------------------------------------------------------------------
gazebo::math::Pose helpers_c::pose( const Reveal::Core::state_c& state ) {
  return gazebo::math::Pose( position( state ), rotation( state ) );
}

//-----------------------------------------------------------------------------
void helpers_c::position( const gazebo::math::Vector3 pos, Reveal::Core::state_c& state ) {
  state.linear_x( pos.x );
  state.linear_y( pos.y );
  state.linear_z( pos.z );
}

//-----------------------------------------------------------------------------
void helpers_c::rotation( const gazebo::math::Quaternion rot, Reveal::Core::state_c& state ) {
  state.angular_x( rot.x );
  state.angular_y( rot.y );
  state.angular_z( rot.z );
  state.angular_w( rot.w );
}

//-----------------------------------------------------------------------------
void helpers_c::linear_velocity( const gazebo::math::Vector3 lvel, Reveal::Core::state_c& state ) {
  state.linear_dx( lvel.x );
  state.linear_dy( lvel.y );
  state.linear_dz( lvel.z );
}

//-----------------------------------------------------------------------------
void helpers_c::angular_velocity( const gazebo::math::Vector3 avel, Reveal::Core::state_c& state ) {
  state.angular_dx( avel.x );
  state.angular_dy( avel.y );
  state.angular_dz( avel.z );
}

//-----------------------------------------------------------------------------
void helpers_c::pose( const gazebo::math::Pose pose, Reveal::Core::state_c& state ) {
  position( pose.pos, state );
  rotation( pose.rot, state );
}

//-----------------------------------------------------------------------------
double helpers_c::sim_time( gazebo::physics::WorldPtr world ) {
  return world->GetSimTime().Double();
}

//-----------------------------------------------------------------------------
void helpers_c::sim_time( double t, gazebo::physics::WorldPtr world ) {
  return world->SetSimTime( gazebo::common::Time( t ) );
}

//-----------------------------------------------------------------------------
double helpers_c::step_size( gazebo::physics::WorldPtr world ) {
  return world->GetPhysicsEngine()->GetMaxStepSize();
}

//-----------------------------------------------------------------------------
double helpers_c::real_time( gazebo::physics::WorldPtr world ) {
  return world->GetRealTime().Double();
}

//-----------------------------------------------------------------------------
void helpers_c::reset( gazebo::physics::WorldPtr world ) {
  world->Reset();
}

//-----------------------------------------------------------------------------
} // Gazebo
//-----------------------------------------------------------------------------
} // Sim
//-----------------------------------------------------------------------------
} // Reveal
//-----------------------------------------------------------------------------
