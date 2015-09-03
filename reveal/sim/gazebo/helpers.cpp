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

  // TODO: only the relevant joints or links indicated in the data should be 
  // manipulated.  The Reveal model does not need a full representation, it only  // needs to have specified the components that will be manipulated.  The model
  // therefore acts as the filter for the components to be adjusted.

  gazebo::physics::ModelPtr gzmodel = world->GetModel( model->id );
  if( !gzmodel ) return false;

  for( unsigned i = 0; i < model->links.size(); i++ ) {
    Reveal::Core::link_ptr rvllink = model->links[i];

    gazebo::physics::LinkPtr gzlink = gzmodel->GetLink( rvllink->id );
    // TODO : refactor assert into something more flexible and valid
    assert( gzlink );

    // map the reveal link state parameters to the gazebo link
    gzlink->SetWorldPose( pose( rvllink->state ) );
    gzlink->SetLinearVel( linear_velocity( rvllink->state ) );
    gzlink->SetAngularVel( angular_velocity( rvllink->state ) );
  }
  for( unsigned i = 0; i < model->joints.size(); i++ ) {
    Reveal::Core::joint_ptr rvljoint = model->joints[i];

    gazebo::physics::JointPtr gzjoint = gzmodel->GetJoint( rvljoint->id );
    // TODO : refactor assert into something more flexible and valid
    //assert( gzjoint );

    // TODO: set the joint state

    // NOTE : GetAngleCount is not implemented for Bullet or ODE
    //unsigned max_k = gzjoint->GetAngleCount();
    //for( unsigned k = 0; k < max_k; k++ )
    //  gzjoint->SetForce( k, joint->control[k] );

    // Bullet and ODE only use 1 DOF?
    //gzjoint->SetForce( 0, joint->control[0] ); 
    //--
    
    if( !gzjoint ) continue;  // identifiers don't align.  big problem

    // TODO: sanity check that reveal joint has same size as gazebo joint
    
    for( unsigned j = 0; j < gzjoint->GetAngleCount(); j++ ) {
      gzjoint->SetPosition( j, rvljoint->state.q(j) ); 
      gzjoint->SetVelocity( j, rvljoint->state.dq(j) ); 
      gzjoint->SetForce( j, rvljoint->control[j] ); 
    }
  }
/*
  for( unsigned j = 0; j < model->joints.size(); j++ ) {
    Reveal::Core::joint_ptr joint = model->joints[j];

    gazebo::physics::JointPtr gzjoint = gzmodel->GetJoint( joint->id );
    // TODO : refactor assert into something more flexible and valid
    assert( gzjoint );

    // TODO: set the joint state

    // NOTE : GetAngleCount is not implemented for Bullet or ODE
    //unsigned max_k = gzjoint->GetAngleCount();
    //for( unsigned k = 0; k < max_k; k++ )
    //  gzjoint->SetForce( k, joint->control[k] );

    // Bullet and ODE only use 1 DOF?
    gzjoint->SetForce( 0, joint->control[0] ); 
  }
*/
  return true;
}

//-----------------------------------------------------------------------------
bool helpers_c::write_trial( Reveal::Core::trial_ptr trial, Reveal::Core::experiment_ptr experiment, gazebo::physics::WorldPtr world, std::vector<Reveal::Core::model_ptr>& models ) {
  models.clear();   // clear is ok since no need to force reallocation

  world->SetSimTime( trial->t );
  world->GetPhysicsEngine()->SetMaxStepSize( experiment->time_step );

  for( unsigned i = 0; i < trial->models.size(); i++ ) {
    Reveal::Core::model_ptr model = trial->models[i];
    models.push_back( model );

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
Reveal::Core::model_ptr helpers_c::read_model( gazebo::physics::WorldPtr world, std::string model_name, std::map<std::string,double> control, Reveal::Core::model_ptr filter ) {
  //printf( "reading model\n" );

  // TODO: similar to the adjustment in write model, read model needs a filter
  // that indicates what components (and only those that) should be recorded.
  // for data generation, the filter is provided by the plugin designer but for
  // the Reveal system, the filter is provided by the model created when the 
  // trial was received.  What we don't want is what we have here and that is
  // arbitrarily reading the whole model.

  Reveal::Core::model_ptr rvlmodel;
  gazebo::physics::ModelPtr gzmodel = world->GetModel( model_name );

  // if gazebo could not find the model, return a reveal model null reference
  if( !gzmodel ) return rvlmodel;
  
  // otherwise, start mapping the model
  rvlmodel = Reveal::Core::model_ptr( new Reveal::Core::model_c() );
  rvlmodel->id = model_name;

  // iterate through the filter and grab only the values defined by the filter 
  for( unsigned i = 0; i < filter->links.size(); i++ ) {
    Reveal::Core::link_ptr flink = filter->links[i];
    Reveal::Core::link_ptr rvllink( new Reveal::Core::link_c() );

    // get the relevant gazebo structure by using the filter's identifier
    gazebo::physics::LinkPtr gzlink = gzmodel->GetLink( flink->id );

    if( !gzlink ) continue;  // identifiers don't align.  big problem

    // get the representative parameters
    rvllink->id = gzlink->GetName();
    pose( gzlink->GetWorldPose(), rvllink->state );
    linear_velocity( gzlink->GetWorldLinearVel(), rvllink->state );
    angular_velocity( gzlink->GetWorldAngularVel(), rvllink->state );

    // insert the link into the reveal model
    rvlmodel->insert( rvllink );
  }
  for( unsigned i = 0; i < filter->joints.size(); i++ ) {
    Reveal::Core::joint_ptr fjoint = filter->joints[i];
    Reveal::Core::joint_ptr rvljoint( new Reveal::Core::joint_c() );

    // get the relevant gazebo structure by using the filter's identifier
    gazebo::physics::JointPtr gzjoint = gzmodel->GetJoint( fjoint->id );
    
    if( !gzjoint ) continue;  // identifiers don't align.  big problem

    rvljoint->id = gzjoint->GetName();
    rvljoint->state.resize( gzjoint->GetAngleCount() );
    rvljoint->control.resize( gzjoint->GetAngleCount() );
    
    for( unsigned j = 0; j < gzjoint->GetAngleCount(); j++ ) {
      // -States-
      rvljoint->state.q( j, gzjoint->GetAngle( j ).Radian() );
      rvljoint->state.dq( j, gzjoint->GetVelocity( j ) );

      // -Controls-
      // assume it is mapped
      //bool mapped = true;
      std::stringstream key;
      key << rvljoint->id << "&" << j;

      // assume a zero control
      double u = 0;
      try{
        // try to get the value stored in the map
        u = control.at( key.str() );
      } catch( std::exception ex ) {
        // if the map failed to return a reference, then the joint is not mapped
        //mapped = false;
      }

      // skip this joint if it is not mapped
      //if( !mapped ) continue;

      rvljoint->control[j] = u;
    }
  
    // insert the joint into the reveal model
    rvlmodel->insert( rvljoint );
  }

  // return the reveal model
  return rvlmodel;
/*
  Reveal::Core::model_ptr rvlmodel;
  // attempt to find the model in the gazebo world
  gazebo::physics::ModelPtr gzmodel = world->GetModel( model_name );

  // if gazebo could not find the model, return a reveal model null reference
  if( !gzmodel ) return rvlmodel;

  // otherwise, start mapping the model
  rvlmodel = Reveal::Core::model_ptr( new Reveal::Core::model_c() );
  rvlmodel->id = model_name;

  //printf( "reading links\n" );
  // get all the links in the gazebo model and iterate through them
  gazebo::physics::Link_V gzlinks = gzmodel->GetLinks();
  for( unsigned i = 0; i < gzlinks.size(); i++ ) {

    // create a new reveal link and assign the correlating properties to it
    Reveal::Core::link_ptr link( new Reveal::Core::link_c() );
    link->id = gzlinks[i]->GetName();

    // map the gazebo link state parameters into the reveal link
    pose( gzlinks[i]->GetWorldPose(), link->state );
    linear_velocity( gzlinks[i]->GetWorldLinearVel(), link->state );
    angular_velocity( gzlinks[i]->GetWorldAngularVel(), link->state );

    // insert the link into the reveal model
    rvlmodel->insert( link );
    //rvlmodel->push_back( link );
  }

  //printf( "reading joints\n" );
  // get all the joints in the gazebo model and iterate through them
  gazebo::physics::Joint_V gzjoints = gzmodel->GetJoints();
  for( unsigned i = 0; i < gzjoints.size(); i++ ) {
    gazebo::physics::JointPtr gzjoint = gzjoints[i];
   
    // read the joint identifier from gazebo
    std::string id = gzjoint->GetName();

    // create a reveal joint and assign it the name and control
    Reveal::Core::joint_ptr rvljoint( new Reveal::Core::joint_c() );
    rvljoint->id = id;

    //printf( "joint[%s] AngleCount: %u\n", id.c_str(), gzjoint->GetAngleCount() );
    assert( gzjoint->GetAngleCount() <= 3 );  // Sanity check number of angles

    for( unsigned j = 0; j < gzjoint->GetAngleCount(); j++ ) {
      // -States-
      rvljoint->state.q( j, gzjoint->GetAngle( j ).Radian() );
      rvljoint->state.dq( j, gzjoint->GetVelocity( j ) );

      // -Controls-
      // assume it is mapped
      //bool mapped = true;
      std::stringstream key;
      key << id << "&" << j;

      // assume a zero control
      double u = 0;
      try{
        // try to get the value stored in the map
        u = control.at( key.str() );
      } catch( std::exception ex ) {
        // if the map failed to return a reference, then the joint is not mapped
        //mapped = false;
      }

      // skip this joint if it is not mapped
      //if( !mapped ) continue;

      rvljoint->control[j] = u;
    }
  
    // insert the joint into the reveal model
    rvlmodel->insert( rvljoint );
  }

  // return the reveal model
  return rvlmodel;
*/
}

//-----------------------------------------------------------------------------
// TODO : could be improved by actually returning an error code or writing to
// std::err
// returns an empty pointer if it fails to find the model in the gazebo world
Reveal::Core::model_ptr helpers_c::read_model( gazebo::physics::WorldPtr world, std::string model_name ) {
/*
  //printf( "reading model\n" );

  // TODO: similar to the adjustment in write model, read model needs a filter
  // that indicates what components (and only those that) should be recorded.
  // for data generation, the filter is provided by the plugin designer but for
  // the Reveal system, the filter is provided by the model created when the 
  // trial was received.  What we don't want is what we have here and that is
  // arbitrarily reading the whole model.

  Reveal::Core::model_ptr rvlmodel;
  gazebo::physics::ModelPtr gzmodel = world->GetModel( model_name );

  // if gazebo could not find the model, return a reveal model null reference
  if( !gzmodel ) return rvlmodel;
  
  // otherwise, start mapping the model
  rvlmodel = Reveal::Core::model_ptr( new Reveal::Core::model_c() );
  rvlmodel->id = model_name;

  // iterate through the filter and grab only the values defined by the filter 
  for( unsigned i = 0; i < filter->links.size(); i++ ) {
    Reveal::Core::link_ptr flink = filter->links[i];
    Reveal::Core::link_ptr rvllink( new Reveal::Core::link_c() );

    // get the relevant gazebo structure by using the filter's identifier
    gazebo::physics::LinkPtr gzlink = gzmodel->GetLink( flink->id );

    if( !gzlink ) continue;  // identifiers don't align.  big problem

    // get the representative parameters
    rvllink->id = gzlink->GetName();
    pose( gzlink->GetWorldPose(), rvllink->state );
    linear_velocity( gzlink->GetWorldLinearVel(), rvllink->state );
    angular_velocity( gzlink->GetWorldAngularVel(), rvllink->state );

    // insert the link into the reveal model
    rvlmodel->insert( rvllink );
  }
  for( unsigned i = 0; i < filter->joints.size(); i++ ) {
    Reveal::Core::joint_ptr fjoint = filter->joints[i];
    Reveal::Core::joint_ptr rvljoint( new Reveal::Core::joint_c() );

    // get the relevant gazebo structure by using the filter's identifier
    gazebo::physics::JointPtr gzjoint = gzmodel->GetJoint( fjoint->id );
    
    if( !gzjoint ) continue;  // identifiers don't align.  big problem

    rvljoint->id = gzjoint->GetName();
    rvljoint->state.resize( gzjoint->GetAngleCount() );
    rvljoint->control.resize( gzjoint->GetAngleCount() );
    
    for( unsigned j = 0; j < gzjoint->GetAngleCount(); j++ ) {
      // -States-
      rvljoint->state.q( j, gzjoint->GetAngle( j ).Radian() );
      rvljoint->state.dq( j, gzjoint->GetVelocity( j ) );

      // -Controls-
      // assume it is mapped
      //bool mapped = true;
      std::stringstream key;
      key << rvljoint->id << "&" << j;

      // assume a zero control
      double u = 0;
      try{
        // try to get the value stored in the map
        u = control.at( key.str() );
      } catch( std::exception ex ) {
        // if the map failed to return a reference, then the joint is not mapped
        //mapped = false;
      }

      // skip this joint if it is not mapped
      //if( !mapped ) continue;

      rvljoint->control[j] = u;
    }
  
    // insert the joint into the reveal model
    rvlmodel->insert( rvljoint );
  }

  // return the reveal model
  return rvlmodel;
*/
  Reveal::Core::model_ptr rvlmodel;
  // attempt to find the model in the gazebo world
  gazebo::physics::ModelPtr gzmodel = world->GetModel( model_name );

  // if gazebo could not find the model, return a reveal model null reference
  if( !gzmodel ) return rvlmodel;

  // otherwise, start mapping the model
  rvlmodel = Reveal::Core::model_ptr( new Reveal::Core::model_c() );
  rvlmodel->id = model_name;

  //printf( "reading links\n" );
  // get all the links in the gazebo model and iterate through them
  gazebo::physics::Link_V gzlinks = gzmodel->GetLinks();
  for( unsigned i = 0; i < gzlinks.size(); i++ ) {
    gazebo::physics::LinkPtr gzlink = gzlinks[i];

    // create a new reveal link and assign the correlating properties to it
    Reveal::Core::link_ptr rvllink( new Reveal::Core::link_c( gzlink->GetName() ) );

    // map the gazebo link state parameters into the reveal link
    pose( gzlink->GetWorldPose(), rvllink->state );
    linear_velocity( gzlink->GetWorldLinearVel(), rvllink->state );
    angular_velocity( gzlink->GetWorldAngularVel(), rvllink->state );

    // insert the link into the reveal model
    rvlmodel->insert( rvllink );
  }

  //printf( "reading joints\n" );
  // get all the joints in the gazebo model and iterate through them
  gazebo::physics::Joint_V gzjoints = gzmodel->GetJoints();
  for( unsigned i = 0; i < gzjoints.size(); i++ ) {
    gazebo::physics::JointPtr gzjoint = gzjoints[i];

    // create a reveal joint and assign it the name and control
    Reveal::Core::joint_ptr rvljoint( new Reveal::Core::joint_c( gzjoint->GetName() ) );

    //printf( "joint[%s] AngleCount: %u\n", rvljoint->id.c_str(), gzjoint->GetAngleCount() );

    rvljoint->state.resize( gzjoint->GetAngleCount() );
    rvljoint->control.resize( 0 );
    
    for( unsigned j = 0; j < gzjoint->GetAngleCount(); j++ ) {
      // -States-
      rvljoint->state.q( j, gzjoint->GetAngle( j ).Radian() );
      rvljoint->state.dq( j, gzjoint->GetVelocity( j ) );
    }
  
    // insert the joint into the reveal model
    rvlmodel->insert( rvljoint );
  }

  // return the reveal model
  return rvlmodel;
}

//-----------------------------------------------------------------------------
Reveal::Core::solution_ptr helpers_c::read_model_solution( gazebo::physics::WorldPtr world, std::vector<std::string> model_list, std::string scenario_id, std::vector<Reveal::Core::model_ptr> model_filters ) {

  // create a model solution
  Reveal::Core::solution_ptr solution( new Reveal::Core::solution_c( Reveal::Core::solution_c::MODEL ) );

  // map required parameters
  solution->scenario_id = scenario_id;
  solution->t = sim_time( world );

  // map the states of all desired models
  for( unsigned i = 0; i < model_list.size(); i++ ) {
    Reveal::Core::model_ptr filter;
    // find the corresponding filter
    for( unsigned j = 0; j < model_filters.size(); j++ ) {
      Reveal::Core::model_ptr f = model_filters[j];
      if( f->id == model_list[i] ) {
        filter = f;
        break;
      }
    }
    // if the model does not have a filter then the model itself is filtered out
    if( !filter ) continue;

    Reveal::Core::model_ptr model = read_model( world, model_list[i] );

    if( model ) solution->models.push_back( model );
  }
  return solution;
}

//-----------------------------------------------------------------------------
Reveal::Core::solution_ptr helpers_c::read_model_solution( gazebo::physics::WorldPtr world, std::vector<std::string> model_list, std::string scenario_id ) {

  // create a model solution
  Reveal::Core::solution_ptr solution( new Reveal::Core::solution_c( Reveal::Core::solution_c::MODEL ) );

  // map required parameters
  solution->scenario_id = scenario_id;
  solution->t = sim_time( world );

  // map the states of all desired models
  for( unsigned i = 0; i < model_list.size(); i++ ) {
    Reveal::Core::model_ptr model = read_model( world, model_list[i] );
    if( model ) solution->models.push_back( model );
  }
  return solution;
}

//-----------------------------------------------------------------------------
Reveal::Core::solution_ptr helpers_c::read_client_solution( gazebo::physics::WorldPtr world, std::vector<std::string> model_list, std::string scenario_id ) {

  // create a client solution
  Reveal::Core::solution_ptr solution( new Reveal::Core::solution_c( Reveal::Core::solution_c::CLIENT ) );

  // map required parameters
  solution->scenario_id = scenario_id;
  solution->t = sim_time( world );
  solution->real_time = real_time( world );

  // map the states of all desired models
  for( unsigned i = 0; i < model_list.size(); i++ ) {
    Reveal::Core::model_ptr model = read_model( world, model_list[i] );
    if( model ) solution->models.push_back( model );
  }
  return solution;
}

//-----------------------------------------------------------------------------
gazebo::math::Vector3 helpers_c::position( const Reveal::Core::link_state_c& state ) {
  return gazebo::math::Vector3( state.linear_x(), state.linear_y(), state.linear_z() );
}

//-----------------------------------------------------------------------------
gazebo::math::Quaternion helpers_c::rotation( const Reveal::Core::link_state_c& state ) {
  return gazebo::math::Quaternion( state.angular_w(), state.angular_x(), state.angular_y(), state.angular_z() );
}

//-----------------------------------------------------------------------------
gazebo::math::Vector3 helpers_c::linear_velocity( const Reveal::Core::link_state_c& state ) {
  return gazebo::math::Vector3( state.linear_dx(), state.linear_dy(), state.linear_dz() );
}

//-----------------------------------------------------------------------------
gazebo::math::Vector3 helpers_c::angular_velocity( const Reveal::Core::link_state_c& state ) {
  return gazebo::math::Vector3( state.angular_dx(), state.angular_dy(), state.angular_dz() );
}
  
//-----------------------------------------------------------------------------
gazebo::math::Pose helpers_c::pose( const Reveal::Core::link_state_c& state ) {
  return gazebo::math::Pose( position( state ), rotation( state ) );
}

//-----------------------------------------------------------------------------
void helpers_c::position( const gazebo::math::Vector3 pos, Reveal::Core::link_state_c& state ) {
  state.linear_x( pos.x );
  state.linear_y( pos.y );
  state.linear_z( pos.z );
}

//-----------------------------------------------------------------------------
void helpers_c::rotation( const gazebo::math::Quaternion rot, Reveal::Core::link_state_c& state ) {
  state.angular_x( rot.x );
  state.angular_y( rot.y );
  state.angular_z( rot.z );
  state.angular_w( rot.w );
}

//-----------------------------------------------------------------------------
void helpers_c::linear_velocity( const gazebo::math::Vector3 lvel, Reveal::Core::link_state_c& state ) {
  state.linear_dx( lvel.x );
  state.linear_dy( lvel.y );
  state.linear_dz( lvel.z );
}

//-----------------------------------------------------------------------------
void helpers_c::angular_velocity( const gazebo::math::Vector3 avel, Reveal::Core::link_state_c& state ) {
  state.angular_dx( avel.x );
  state.angular_dy( avel.y );
  state.angular_dz( avel.z );
}

//-----------------------------------------------------------------------------
void helpers_c::pose( const gazebo::math::Pose pose, Reveal::Core::link_state_c& state ) {
  position( pose.pos, state );
  rotation( pose.rot, state );
}

//-----------------------------------------------------------------------------
double helpers_c::position( const Reveal::Core::joint_state_c& state, unsigned index ) {
  assert( index < state.size_q() );
  return state.q( index );
}

//-----------------------------------------------------------------------------
double helpers_c::velocity( const Reveal::Core::joint_state_c& state, unsigned index ) {
  assert( index < state.size_dq() );
  return state.dq( index );
}

//-----------------------------------------------------------------------------
void helpers_c::position( Reveal::Core::joint_state_c& state, unsigned index, double value ) {
  assert( index < state.size_q() );
  state.q( index, value );
}

//-----------------------------------------------------------------------------
void helpers_c::velocity( Reveal::Core::joint_state_c& state, unsigned index, double value ) {
  assert( index < state.size_dq() );
  state.dq( index, value );
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
