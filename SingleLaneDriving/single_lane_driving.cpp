////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///     @file       single_lane_driving.cpp
///     @author     Adria Serra Moral (adriaserra4@gmail.com)
///     @date       10-30-2019
///
///     @brief      This file includes a simple single lane driving system using ST-graph of obstacle.
///                 For now, the entire code assumes 1D-lane driving. Hence, there are no 2D grids,
///                 no lane-switching, etc. The only way to avoid obstacle(s) is to slow-down (or stop),
///                 or continue making forward progress.
///
///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "single_lane_driving.h"

namespace planning {

/**
 * @brief The STpoint struct
 *
 *          We create a struct with the relevant data for a ST-obstacle point.
 *          This way, it is easy to grow implementation and switch from 1-D to
 *          2-D if necessary.
 */
STpoint::STpoint(const double s, const double t, const double v) {

    // Assign distance
    this->s = s;
    this->s0 = s;

    // Time object was detected
    this->t0 = t;

    // Object speed
    this->v = v;

}

/**
 * @brief getDist
 * @param time  - delta time from current time
 * @return distance from car to obstacle
 *
 * NOTE: For now, we will use this to have descrete points,
 *       So we will keep v as 0.
 */
double STpoint::getDist() const {
    return s0;
}
double STpoint::getDist(const double time) const{
    return (s0 + v * time);
}

/**
 * @brief getSpeed
 * @return return reference to v
 */
const double& STpoint::getSpeed() const {
    return v;
}


/**
 * @brief VehicleState::VehicleState
 * @param time
 * @param x
 * @param vx
 * @param ax
 * @param jx
 */
VehicleState::VehicleState(const double time, const double x, const double vx,
                           const double ax,  const double jx) {
    // Construct state
    this->time = time;
    this->x = x;
    this->vx = vx;
    this->ax = ax;
    this->jx = jx;
}


/**
 * @brief getObstacles
 * @param time
 * @return MinDistancePriorityQue - queue of obstacles with closest obstacles at the top of the queue
 *                                  we care about the closest obstacles
 */
MinDistancePriorityQue STGraphObstacles::getObstacles(const double time) {

    // IF empty, return empty
    if( STMap.empty() ) {
        return MinDistancePriorityQue();
    }

    // If the time is in our discrete implementation, take that
    if( STMap.count(time) > 0 ) {
        return STMap.at(time);
    }

    // We need to interpolate, for now, to go faster since we would
    // need to do obstacle matching or a few complicated things, just return
    // the queue closer to our time, from before or after.

    // get lower and upper queuest to interpolate
    std::map<double, MinDistancePriorityQue>::iterator pqObs_before = STMap.lower_bound(static_cast<double>(time));
    --pqObs_before;
    std::map<double, MinDistancePriorityQue>::iterator pqObs_after = STMap.upper_bound(static_cast<double>(time));

    // check that both exist
    if( pqObs_after == STMap.end() || pqObs_before == STMap.end() ) {
        // we should handle this better, but for now, just return empty queue
        return MinDistancePriorityQue();
    }

    // both exist, continue interpolating
    const double delta_time_before = time - pqObs_before->first;
    const double delta_time_after = pqObs_after->first - time;;

    // Since I am not interpolating, return the closest queue to our desired time
    if( delta_time_after > delta_time_before ) {
        return pqObs_before->second;
    }

    return pqObs_after->second;
}

/**
 * @brief addObstacleDiscrete
 * @param obstacle
 *
 * NOTE, I consider only one obstacle as in 1D having multiple obstacles does not make
 * much sense since we cannot overtake or maneuver much.
 */
void STGraphObstacles::addObstacleDiscrete(const std::map<double, STpoint>& obstacle) {
    // NOTE: THIS CODE DOES NOT CONSIDER THE CASE WHERE WE ADD AN OBSTACLE
    //       AT A TIME THAT WE HAD NOT ADDED BEFORE, BUT IT IS IN BETWEEN
    //       TIMES WHERE WE HAVE OBSTACLES. IN A BETTER CODE, WE WOULD HAVE TO CHECK
    //       EARLIER AND LATER TIME AND INCLUDE ALL OBSTACLES OF INTEREST, MAYBE?

    // add to map
    // for all points:
    for( const auto& obs_it : obstacle ) {
        const double& t = obs_it.first;
        const STpoint& obs = obs_it.second;

        if( STMap.count(t) > 0){
            // we have stuff there, add new
            auto& queue_obs = STMap.at(t);
            queue_obs.push(obs);
        } else {
            // new one, create and add
            MinDistancePriorityQue pq;
            pq.push(obs);

            STMap[t] = pq;
        }
    }

}

void STGraphObstacles::addObstacleDiscrete(const std::vector<double>& times,
                                           const std::vector<STpoint>& obstacle) {
    // NOTE: THIS CODE DOES NOT CONSIDER THE CASE WHERE WE ADD AN OBSTACLE
    //       AT A TIME THAT WE HAD NOT ADDED BEFORE, BUT IT IS IN BETWEEN
    //       TIMES WHERE WE HAVE OBSTACLES. IN A BETTER CODE, WE WOULD HAVE TO CHECK
    //       EARLIER AND LATER TIME AND INCLUDE ALL OBSTACLES OF INTEREST, MAYBE?

    // add to map
    // for all points:
    if( times.size() != obstacle.size() ) {
        // inconsistent information, would need to raise exception error or
        // alert user. This should not happen, return for now
        return;
    }

    for( size_t idx = 0, idx_end = times.size(); idx < idx_end; idx++ ) {

        const double& t = times.at(idx);
        const STpoint& obs = obstacle.at(idx);

        if( STMap.count(t) > 0){
            // we have stuff there, add new
            auto& queue_obs = STMap.at(t);
            queue_obs.push(obs);
        } else {
            // new one, create and add
            MinDistancePriorityQue pq;
            pq.push(obs);

            STMap[t] = pq;
        }
    }
}

void STGraphObstacles::addObstacleDiscrete(const std::vector<std::pair<double, STpoint> >& obstacle) {

    // NOTE: THIS CODE DOES NOT CONSIDER THE CASE WHERE WE ADD AN OBSTACLE
    //       AT A TIME THAT WE HAD NOT ADDED BEFORE, BUT IT IS IN BETWEEN
    //       TIMES WHERE WE HAVE OBSTACLES. IN A BETTER CODE, WE WOULD HAVE TO CHECK
    //       EARLIER AND LATER TIME AND INCLUDE ALL OBSTACLES OF INTEREST, MAYBE?

    // add to map
    // for all points:

    for( const auto& tobs : obstacle ) {
        const double& t = tobs.first;
        const STpoint& obs = tobs.second;

        if( STMap.count(t) > 0){
            // we have stuff there, add new
            auto& queue_obs = STMap.at(t);
            queue_obs.push(obs);
        } else {
            // new one, create and add
            MinDistancePriorityQue pq;
            pq.push(obs);

            STMap[t] = pq;
        }
    }

}

/**
 * @brief STGraphObstacles::reset
 */
void STGraphObstacles::reset() {
    // clear the obstacle map
    STMap.clear();
}

/**
 * @brief getStoppingDistance
 * @param vel
 * @return
 */
double TrajectoryLimits::getStoppingDistance(const double vel) {
    // return the distance traveled in order to stop the vehicle at current state
    // TODO: Include other dynamics such as maximum allowable jerk
    // TODO NOTE: There may be an emergency mode that should be able to override limits if necessary
    return ( vel * vel / ( 2.0 * fabs(acc_min) ) );
}

/**
 * @brief getStoppingTime
 * @param vel
 * @return
 */
double TrajectoryLimits::getStoppingTime(const double vel) {
    // return time that it takes to stop vehicle at current state
    return (vel / fabs(acc_min));
}


/**
 * @brief The SingleAxisTrajectory class
 *
 *        Creates a polynomial trajectory obtained by minimizing snap
 *        From the work of:
 *
 */
SingleAxisTrajectory::SingleAxisTrajectory(TrajectoryLimits* limitsPtr) : pLimits(limitsPtr) {

}

double SingleAxisTrajectory::pos(const double time) const {
    const double t2 = time * time;
    const double t3 = t2 * time;
    const double t4 = t3 * time;
    const double t5 = t4 * time;

    return ( k5 * alpha * t5 + k4 * beta * t4 + k3 * gamma * t3 +
             k2 * acc_init * t2 + k1 * vel_init * time + k0 * pos_init );
}

double SingleAxisTrajectory::vel(const double time) const {
    const double t2 = time * time;
    const double t3 = t2 * time;
    const double t4 = t3 * time;

    return ( k4 * alpha * t4 + k3 * beta * t3 + k2 * gamma * t2 +
             k1 * acc_init * time + k0 * vel_init );
}

double SingleAxisTrajectory::acc(const double time) const {
    const double t2 = time * time;
    const double t3 = t2 * time;

    return ( k3 * alpha * t3 + k2 * beta * t2 + k1 * gamma * time +
             k0 * acc_init );
}

double SingleAxisTrajectory::jerk(const double time) const {
    const double t2 = time * time;

    return ( k2 * alpha * t2 + k1 * beta * time + k0 * gamma );

}

VehicleState SingleAxisTrajectory::state(const double time) const {
    VehicleState st;
    st.time = time;
    st.x = pos(time);
    st.vx = vel(time);
    st.ax = acc(time);
    st.jx = jerk(time);
    return st;
}

/**
 * @brief SingleAxisTrajectory::getFeasibilityBySampling
 * @param time_segment
 * @param number_of_samples
 * @return
 *
 *  Here, since we sample we take advantage and we compute average jerk of trajectory (cost)
 *
 *  NOTE: Normally, we would check for all limits, like curvature, turn rate, etc.
 *        However, since our example is 1D, just check linear acc, vel.
 */
bool SingleAxisTrajectory::getFeasibilityBySampling(const double time_segment, const int number_of_samples) {

    // compute the trajectory sampling dt
    const double dt = time_segment / number_of_samples;

    // initialize time counter
    double t = 0.0;

    // get the trajectory limits
    const double& vel_max = pLimits->vel_max;
    const double& vel_min = pLimits->vel_min;
    const double& acc_max = pLimits->acc_max;
    const double& acc_min = pLimits->acc_min;
    const double& jerk_max = pLimits->jerk_max;

    for( int idx = 0; idx < number_of_samples; idx++ ) {

        // Get velocity
        const double v_idx = vel(t);

        // Get Accel
        const double a_idx = acc(t);

        // Get Jerk
        const double j_idx = jerk(t);

        // If anything is exceeds limits, trajectory is not feasible
        if( v_idx > vel_max || v_idx < vel_min )
            return false;

        if( a_idx > acc_max || a_idx < acc_min )
            return false;

        if( fabs(j_idx) > jerk_max )
            return false;

        // move time forward
        t += dt;

        // add to cost
        cost += j_idx;
    }

    // we passed all checks, return true
    cost /= time_segment;
    return true;


}


/**
 * @brief SingleAxisTrajectory::planTrajectory
 * @param time_segment
 * @param pos_goal
 * @param vel_goal
 * @param acc_goal
 * @return
 */
bool SingleAxisTrajectory::planTrajectory(const double time_segment, const double pos_goal,
                                          const double vel_goal, const double acc_goal) {


    // First, check state is not nan
    if( std::isnan( pos_init ) || std::isnan( vel_init ) || std::isnan( acc_init ) ) {
        is_init = false;
        is_feasible = false;
        return false;
    }

    // Check that we want a goal state
    if( std::isnan( pos_goal ) && std::isnan( vel_goal ) && std::isnan( pos_goal ) ) {
        is_feasible = false;
        return false;
    }

    // get times
    const double T0 = 1.00;
    const double T1 = time_segment;
    const double T2 = time_segment * time_segment;
    const double T3 = T2 * time_segment;
    const double T4 = T2 * T2;
    const double T5 = T4 * time_segment;

    const double T3inv = (1.0 / T3);
    const double T5inv = (1.0 / T5);

    // get delta states
    const double delta_p = pos_goal - pos_init - vel_init * T1 - 0.5 * acc_init * T2;
    const double delta_v = vel_goal - vel_init - acc_init * T1;
    const double delta_a = acc_goal - acc_init;

    // Get polynomial of state we want
    if( !std::isnan( pos_goal ) && !std::isnan( vel_goal ) && !std::isnan( acc_goal ) ) {
        // we have all constraints

        alpha = T5inv * (  720.0 * T0 * delta_p - 360.0 * T1 * delta_v + 60.0 * T2 * delta_a );
        beta =  T5inv * ( -360.0 * T1 * delta_p + 168.0 * T2 * delta_v - 24.0 * T3 * delta_a );
        gamma = T5inv * (   60.0 * T2 * delta_p -  24.0 * T3 * delta_v +  3.0 * T4 * delta_a );

    }
    else if ( std::isnan( pos_goal ) && !std::isnan( vel_goal ) && !std::isnan( acc_goal ) ) {
        // we have vel and acc

        alpha = 0.0;
        beta =  T3inv * ( -12.0 * T0 * delta_v + 6.0 * T1 * delta_a );
        gamma = T3inv * (   6.0 * T1 * delta_v - 2.0 * T2 * delta_a );

    }
    else if ( !std::isnan( pos_goal ) && !std::isnan( vel_goal ) && std::isnan( acc_goal ) ) {
        // we have pos and vel

        alpha = T5inv * (  320.0 * T0 * delta_p - 120.0 * T1 * delta_v );
        beta =  T5inv * ( -200.0 * T1 * delta_p +  72.0 * T2 * delta_v );
        gamma = T5inv * (   40.0 * T2 * delta_p -  12.0 * T3 * delta_v );

    }
    else if ( !std::isnan( pos_goal ) && std::isnan( vel_goal ) && std::isnan( acc_goal ) ) {
        // we have pos only

        alpha = T5inv * (  20.0 * T0 * delta_p );
        beta  = T5inv * ( -20.0 * T1 * delta_p );
        gamma = T5inv * (  10.0 * T2 * delta_p );

    }
    else if ( std::isnan( pos_goal ) && !std::isnan( vel_goal ) && std::isnan( acc_goal ) ) {
        // we have vel only

        alpha = 0.0;
        beta  = T3inv * ( -3.0 * T0 * delta_v );
        gamma = T3inv * (  3.0 * T1 * delta_v );

    }
    else if ( std::isnan( pos_goal ) && std::isnan( vel_goal ) && !std::isnan( acc_goal) ) {
        // we have acc only

        alpha = 0.0;
        beta  = 0.0;
        gamma = delta_a / T1;

    }

    // check if trajectory is feasible
    is_feasible = getFeasibilityBySampling(time_segment, 100*time_segment);
    return is_feasible;
}

bool SingleAxisTrajectory::planTrajectory(const VehicleState& state_init, const double time_segment,
                                          const double pos_goal, const double vel_goal, const double acc_goal) {

    // set initial states
    pos_init = state_init.x;
    vel_init = state_init.vx;
    acc_init = state_init.ax;
    is_init = true;

    // First, check state is not nan
    if( std::isnan( pos_init ) || std::isnan( vel_init ) || std::isnan( acc_init ) ) {
        is_init = false;
        is_feasible = false;
        return false;
    }

    // Check that we want a goal state
    if( std::isnan( pos_goal ) && std::isnan( vel_goal ) && std::isnan( pos_goal ) ) {
        is_feasible = false;
        return false;
    }

    // get times
    const double T0 = 1.00;
    const double T1 = time_segment;
    const double T2 = time_segment * time_segment;
    const double T3 = T2 * time_segment;
    const double T4 = T2 * T2;
    const double T5 = T4 * time_segment;

    const double T3inv = (1.0 / T3);
    const double T5inv = (1.0 / T5);

    // get delta states
    const double delta_p = pos_goal - pos_init - vel_init * T1 - 0.5 * acc_init * T2;
    const double delta_v = vel_goal - vel_init - acc_init * T1;
    const double delta_a = acc_goal - acc_init;

    // Get polynomial of state we want
    if( !std::isnan( pos_goal ) && !std::isnan( vel_goal ) && !std::isnan( acc_goal ) ) {
        // we have all constraints

        alpha = T5inv * (  720.0 * T0 * delta_p - 360.0 * T1 * delta_v + 60.0 * T2 * delta_a );
        beta =  T5inv * ( -360.0 * T1 * delta_p + 168.0 * T2 * delta_v - 24.0 * T3 * delta_a );
        gamma = T5inv * (   60.0 * T2 * delta_p -  24.0 * T3 * delta_v +  3.0 * T4 * delta_a );

    }
    else if ( std::isnan( pos_goal ) && !std::isnan( vel_goal ) && !std::isnan( acc_goal ) ) {
        // we have vel and acc

        alpha = 0.0;
        beta =  T3inv * ( -12.0 * T0 * delta_v + 6.0 * T1 * delta_a );
        gamma = T3inv * (   6.0 * T1 * delta_v - 2.0 * T2 * delta_a );

    }
    else if ( !std::isnan( pos_goal ) && !std::isnan( vel_goal ) && std::isnan( acc_goal ) ) {
        // we have pos and vel

        alpha = T5inv * (  320.0 * T0 * delta_p - 120.0 * T1 * delta_v );
        beta =  T5inv * ( -200.0 * T1 * delta_p +  72.0 * T2 * delta_v );
        gamma = T5inv * (   40.0 * T2 * delta_p -  12.0 * T3 * delta_v );

    }
    else if ( !std::isnan( pos_goal ) && std::isnan( vel_goal ) && std::isnan( acc_goal ) ) {
        // we have pos only

        alpha = T5inv * (  20.0 * T0 * delta_p );
        beta  = T5inv * ( -20.0 * T1 * delta_p );
        gamma = T5inv * (  10.0 * T2 * delta_p );

    }
    else if ( std::isnan( pos_goal ) && !std::isnan( vel_goal ) && std::isnan( acc_goal ) ) {
        // we have vel only

        alpha = 0.0;
        beta  = T3inv * ( -3.0 * T0 * delta_v );
        gamma = T3inv * (  3.0 * T1 * delta_v );

    }
    else if ( std::isnan( pos_goal ) && std::isnan( vel_goal ) && !std::isnan( acc_goal) ) {
        // we have acc only

        alpha = 0.0;
        beta  = 0.0;
        gamma = delta_a / T1;

    }

    // check if trajectory is feasible
    is_feasible = getFeasibilityBySampling(time_segment, 100*time_segment);
    return is_feasible;
}

bool SingleAxisTrajectory::planTrajectory(VehicleState* state_init, const double time_segment,
                                          const double pos_goal, const double vel_goal, const double acc_goal) {

    // set initial states
    pos_init = state_init->x;
    vel_init = state_init->vx;
    acc_init = state_init->ax;
    is_init = true;

    // First, check state is not nan
    if( std::isnan( pos_init ) || std::isnan( vel_init ) || std::isnan( acc_init ) ) {
        is_init = false;
        is_feasible = false;
        return false;
    }

    // Check that we want a goal state
    if( std::isnan( pos_goal ) && std::isnan( vel_goal ) && std::isnan( pos_goal ) ) {
        is_feasible = false;
        return false;
    }

    // get times
    const double T0 = 1.00;
    const double T1 = time_segment;
    const double T2 = time_segment * time_segment;
    const double T3 = T2 * time_segment;
    const double T4 = T2 * T2;
    const double T5 = T4 * time_segment;

    const double T3inv = (1.0 / T3);
    const double T5inv = (1.0 / T5);

    // get delta states
    const double delta_p = pos_goal - pos_init - vel_init * T1 - 0.5 * acc_init * T2;
    const double delta_v = vel_goal - vel_init - acc_init * T1;
    const double delta_a = acc_goal - acc_init;

    // Get polynomial of state we want
    if( !std::isnan( pos_goal ) && !std::isnan( vel_goal ) && !std::isnan( acc_goal ) ) {
        // we have all constraints

        alpha = T5inv * (  720.0 * T0 * delta_p - 360.0 * T1 * delta_v + 60.0 * T2 * delta_a );
        beta =  T5inv * ( -360.0 * T1 * delta_p + 168.0 * T2 * delta_v - 24.0 * T3 * delta_a );
        gamma = T5inv * (   60.0 * T2 * delta_p -  24.0 * T3 * delta_v +  3.0 * T4 * delta_a );

    }
    else if ( std::isnan( pos_goal ) && !std::isnan( vel_goal ) && !std::isnan( acc_goal ) ) {
        // we have vel and acc

        alpha = 0.0;
        beta =  T3inv * ( -12.0 * T0 * delta_v + 6.0 * T1 * delta_a );
        gamma = T3inv * (   6.0 * T1 * delta_v - 2.0 * T2 * delta_a );

    }
    else if ( !std::isnan( pos_goal ) && !std::isnan( vel_goal ) && std::isnan( acc_goal ) ) {
        // we have pos and vel

        alpha = T5inv * (  320.0 * T0 * delta_p - 120.0 * T1 * delta_v );
        beta =  T5inv * ( -200.0 * T1 * delta_p +  72.0 * T2 * delta_v );
        gamma = T5inv * (   40.0 * T2 * delta_p -  12.0 * T3 * delta_v );

    }
    else if ( !std::isnan( pos_goal ) && std::isnan( vel_goal ) && std::isnan( acc_goal ) ) {
        // we have pos only

        alpha = T5inv * (  20.0 * T0 * delta_p );
        beta  = T5inv * ( -20.0 * T1 * delta_p );
        gamma = T5inv * (  10.0 * T2 * delta_p );

    }
    else if ( std::isnan( pos_goal ) && !std::isnan( vel_goal ) && std::isnan( acc_goal ) ) {
        // we have vel only

        alpha = 0.0;
        beta  = T3inv * ( -3.0 * T0 * delta_v );
        gamma = T3inv * (  3.0 * T1 * delta_v );

    }
    else if ( std::isnan( pos_goal ) && std::isnan( vel_goal ) && !std::isnan( acc_goal) ) {
        // we have acc only

        alpha = 0.0;
        beta  = 0.0;
        gamma = delta_a / T1;

    }

    // check if trajectory is feasible
    is_feasible = getFeasibilityBySampling(time_segment, 100*time_segment);
    return is_feasible;
}


/**
 * @brief The SingleLaneDriving class
 */
SingleLaneDriving::SingleLaneDriving(STGraphObstacles *obstacleMapPtr, VehicleState *vehicleStatePtr,
                                     TrajectoryLimits *trajLimitsPtr, const int max_iteration_speed_loop)
    : pObstacleMap(obstacleMapPtr), pState(vehicleStatePtr), pLimits(trajLimitsPtr), polyTrajGenerator(trajLimitsPtr), max_iterations(max_iteration_speed_loop)  {

}

/**
 * @brief SingleLaneDriving::predict
 * @param time
 * @return
 */
VehicleState SingleLaneDriving::predict(const double time) {
    // Given current state, predict car state in the future
    double ax = pState->ax + pState->jx * time;
    double vx = pState->vx + pState->ax * time + 0.5 * pState->jx * time * time;
    double px = pState->x + pState->vx * time + 0.5 * pState->ax * time * time;
    return VehicleState(time, px, vx, ax, pState->jx);
}

/**
 * @brief drive - Main Function, works sort of a state machine or Outer-Loop controller to
 *                generate feasible car trajectories in a single lane.
 */
MinTimePriorityQue SingleLaneDriving::drive(const double time_lookup, const MinTimePriorityQue& curr_plan) {

    // First, get the obstacles at current time
    // NOTE, since the problem stated that ST graph works with relative time,
    // with 0 being current time and t being delta time from now, we assume
    // our map of object has properly being updated to reflect that and that there
    // are negligible delays from the perception sent data to the planner using it.

    // Get obstacle(s) at current time
    MinDistancePriorityQue pqObstacles_now = pObstacleMap->getObstacles(0);

    // Check if we have obstacles

    // --------------- MODE 1: NO OBSTACLES ---------------------------------------
    if( pqObstacles_now.empty() ) {
        // no obstacle(s), deal with this condition (just cruiseee)

        // If we still have a valid trajectory, just continue using it, we do not want to
        // replan too many times to keep smoother rides.
        if( !curr_plan.empty() ) {
            STtraj = curr_plan;
            return STtraj;
        }

        // we executed all maneuvers, re-plan
        // (although with our polynomial, if we do not need a new state we could just
        //  contine calling the same polynomial for an even smoother ride. TODO)
        double alpha = 1.0;
        bool have_feasible_solution { false };

        // generate a feasible solution
        while( !have_feasible_solution ) {

            // Return our polynomial solution
            have_feasible_solution =
                    polyTrajGenerator.planTrajectory(pState, alpha * time_lookup, NAN, 0.95*pLimits->vel_max, 0.0);

            // if solution is not feasible, allow it to move slower by giving it extra time
            // TODO: This is generally fast, but could potentially speed it up with better logic to
            // find a rule for a good next alpha
            if( !have_feasible_solution ) alpha *= 1.1;
        }

        // discretize a little and return map
        // I take dt = 0.01 to have data at 100 Hz-ish (assumed controller rate)
        double dt = 0.01;
        int n_samples = time_lookup * alpha / dt;
        double t_idx {0.0};
        STtraj = MinTimePriorityQue();
        for( size_t idx = 0; idx < n_samples; idx++ ) {

            STtraj.push( polyTrajGenerator.state(t_idx) );

            t_idx += dt;
        }

        return STtraj;
    }

    // We have obstacles, deal with this condition.

    // get the closest object distance at current time
    const STpoint& obs_now = pqObstacles_now.top();
    pqObstacles_now.pop();

    // get distance to closest obstacle
    const double dist_closest_obs = obs_now.getDist();

    // --------------- MODE 2: UNAVOIDABLE COLLISION :( ---------------------------
    if( dist_closest_obs < pLimits->getStoppingDistance( pState->vx ) ) {
        // well, this should not happen but...apparenlty we are in
        // unevitable collision mode... embrace the car
        // actually this would activate super-duper emergency mode

        STtraj = MinTimePriorityQue();
        return STtraj;
    }

    // --------------- MODE 3: AVOIDABLE OBSTACLE (hopefully) ---------------------

    // If we assume that the Obstacle graph we have assumes obstacle position(s)
    // if our state or the world does not change. If we assume that the rate of
    // change of the obstacle distance is the relative speed between obstacle and
    // vehicle, a change in our speed will directly affect the rate of change of
    // distance, and the graph points by (delta_distance = delta_V * time).  Hence:
    //
    //      dist_obs (t) = v_obs_vehicle * time + dist_obs_0
    //
    //      dist_obs (t) = ( v_obs - v_vehicle ) * time + dist_obs_0
    //
    //      We can gain distance by decreasing our speed, or reduce distance by increasing it.
    //
    //      If our guidance strategy is: We want at least d_min from obstacle at
    //      t_lookout time. We get:
    //
    //      dist_obs (t_lookout) = d_min = ( v_obs - v_vehicle ) * t_lookout + dist_obs_0
    //
    //      given a delta speed delta_v.  V_vehicle' = V_vehicle_0 + delta_v
    //
    //      d_min = ( v_obs - v_vehicle_0 - delta_v) * t_lookout + dist_obs_0
    //      d_min = ( v_obs_vehicle - delta_v) * t_lookout + dist_obs_0
    //
    //      delta_v = v_obs_vehicle - ( d_min - dist_obs_0 ) / t_lookout
    //
    //      If we do not have a direct number for v_obs, since we have discrete data,
    //      assume v_obs_vehicle_mean = ( dist_obs(t_lookout) - dist_obs_0 ) / t_lookout.  Hence:
    //
    //      delta_v = ( dist_obs_lookout - dist_obs_0 - d_min + dist_obs_0 ) / t_lookout
    //
    //      delta_v = ( dist_obs_lookout - d_min ) / t_lookout
    //
    //      Yet, our desired velocity at the end is V_desired = saturation(V_0 + delta_V)
    //
    //      where saturation is 0 <= V_desired <= V_max

    // get the distance of obstacle at lookout time
    MinDistancePriorityQue pqObstacles_lookout = pObstacleMap->getObstacles(time_lookup);
    const STpoint& obs_lookup = pqObstacles_lookout.top();
    pqObstacles_lookout.pop();

    // If we have an obstacle approaching the vehicle, this rule will generate a slower velocity to keep
    // the obstacle d_min distance away. Yet, as we slow down, we have extra time to reach that distance,
    // so the vehicle will try to speed up again. Run loop a few iterations to converge to what speed to slow
    // down so that does not happen.
    double delta_v {0.0};
    int iteration {0};
    double delta_progress {1.0};
    double v_new = pState->vx + delta_v;

    while ( iteration < max_iterations && fabs(delta_progress) > 0.01 && v_new < pLimits->vel_max && v_new > pLimits->vel_min ) {

        double delta_v_new = ( obs_lookup.getDist() - pLimits->getStoppingDistance(pState->vx + 0.1 * delta_v) );

        delta_progress = delta_v_new - delta_v;

        delta_v = delta_v_new;

        v_new = pState->vx + delta_v;

        iteration++;
    }

    // Again, saturate our goal to the allowable speeds in the road
    double v_desired = pState->vx + delta_v;
    if( v_desired > pLimits->vel_max ) v_desired = 0.95*pLimits->vel_max;

    // IF there is something blocking road, or a traffic jam, we may want to slow down,
    // however in single-lane drive we do not want to reverse so, minimum speed is 0.
    if( v_desired < 0.0 ) v_desired = 0.0;

    // Generate polyomial to desired state
    double alpha = 1.0;
    bool have_feasible_solution { false };

    // generate a feasible solution
    while( !have_feasible_solution ) {

        have_feasible_solution =
                polyTrajGenerator.planTrajectory(pState, alpha * time_lookup, NAN, v_desired, 0.0);

        // if solution is not feasible, allow it to move slower.
        if( !have_feasible_solution ) alpha *= 1.1;
    }

    // discretize a little and return map
    // I take d * time to have data at 100 Hz-ish
    double dt = 0.01;
    int n_samples = time_lookup * alpha / dt;
    double t_idx {0.0};
    STtraj = MinTimePriorityQue();
    for( size_t idx = 0; idx < n_samples; idx++ ) {

        STtraj.push( polyTrajGenerator.state(t_idx) );

        t_idx += dt;
    }

    return STtraj;
}


/**
 *  @brief  SingleLaneDrivingSystemTest
 *
 */
SingleLaneDrivingSystemTest::SingleLaneDrivingSystemTest() {

    obstacleMap = new STGraphObstacles();

    vehicleStates = new VehicleState();

    trajLimits = new TrajectoryLimits();

    driver = new SingleLaneDriving(obstacleMap, vehicleStates, trajLimits);
}

SingleLaneDrivingSystemTest::~SingleLaneDrivingSystemTest() {
    delete obstacleMap;
    delete vehicleStates;
    delete trajLimits;
    delete driver;
}

/**
 * @brief initializeTest
 */
void SingleLaneDrivingSystemTest::initializeTest() {

    // got some assumed limits of maximum allowable acceleration(s) and jerk
    // In fact, I obtained these numbers from an article talking about autonomous
    // driving ride comfort for seated passengers, so they are pretty legit.

    // First, set limits
    trajLimits->acc_max = 2.0;              ///< m/s^2
//    trajLimits->acc_min = -0.45 * 9.79;   ///< m/s^2 (0.45 gs)
    trajLimits->acc_min = -2.0;             ///< m/s^2 (keep consistant as -2)

    trajLimits->dist_perception = 100;      ///< meters
    trajLimits->vel_max = 16.67;            ///< m/s or 60 km / h
    trajLimits->vel_min = -2.0;             ///< m/s

    trajLimits->jerk_max = 0.9;             ///< jerk limit for discomfort

    // Set starting state
    vehicleStates->x = 0.0;
    vehicleStates->vx = 0.0;
    vehicleStates->ax = 0.0;
    vehicleStates->jx = 0.0;

    // Initial obstacle map
    obstacleMap->reset();

    //

}

/**
 * @brief runTest
 */
void SingleLaneDrivingSystemTest::runTest(std::ofstream& vehicleOutputLog, std::ofstream& obstacleOutputLog) {

    // Open files to store data
    if( !vehicleOutputLog.is_open() ) {
        vehicleOutputLog.open("vehicle_output.csv");
    }
    if( !obstacleOutputLog.is_open() ) {
        obstacleOutputLog.open("obstacle_output.csv");
    }

    // start time of sim
    const double t_start {0.0};

    // end time of sim
    const double t_end {40.0};

    // dt of sim
    const double dt {0.01};

    // Lookup time of simulation
    const double t_lookup = 2.00;

    // Time Interval we call planner
    double dt_planner_counter = 0.0;
    const double dt_planner = 0.2;

    // Time the obstacle first appears
    const double time_obstacle_appears = 8.0;

    // Create an obstacle that may appear
    const double v_obs = 0.60 * trajLimits->vel_max;
    const double t_increment = 0.20000;
    double d_start = 80.0;
    double v_rel = 0.0;

    // RUN SIM
    double time = t_start;

    // Current plane from
    MinTimePriorityQue current_plan;

    // iterations of loop
    int iteration {0};

    while ( time <= t_end ) {

        // Display progress to keep me happy knowing code is running
        if( iteration % 100 == 0 )
            std::cout << "Time is:  " << time << std::endl;

        // if we need to plan, do it
        if( fabs( dt_planner_counter - dt_planner ) < dt  || time == t_start) {

            // If we need to add the obstacle, add it.
            if( time >= time_obstacle_appears ){
                // Update obstacle measurement and add

                // get relative speed, since it is what car sees or measures
                v_rel = v_obs - vehicleStates->vx;

                // Store obstacle prediction (we have a perfect perception)
                std::vector<double> t_obs;
                std::vector<STpoint> p_obs;

                double t_idx = 0.0;
                for ( size_t idx = 0, idx_end = ((t_lookup+1) / t_increment); idx <= idx_end; idx++  ) {

                    t_obs.push_back( t_idx );

                    p_obs.push_back(STpoint( d_start + v_rel * t_idx, t_idx ));

                    t_idx += t_increment;
                }

                obstacleMap->addObstacleDiscrete(t_obs, p_obs);
            }

            // Generate a new Plan (if necessary)
            current_plan = driver->drive(t_lookup, current_plan);
            dt_planner_counter = 0.0;
        }

        // Assume perfect tracking if have a plan,
        // propagate current state if not
        if( current_plan.empty() ) {
            vehicleStates->x += vehicleStates->vx * dt + 0.5 * vehicleStates->ax * dt * dt;
            vehicleStates->vx += vehicleStates->ax * dt + 0.5 * vehicleStates->jx * dt * dt;
            vehicleStates->ax += vehicleStates->jx * dt;
            vehicleStates->time = time;

            if( time >= time_obstacle_appears ) {
                writeObstacle(obstacleOutputLog, time, d_start, v_rel);
                d_start += v_rel * dt;
            }

            writeTrajectory(vehicleOutputLog, *vehicleStates);

            time += dt;
            dt_planner_counter = dt_planner;
            iteration++;
            continue;
        }

        // Again, assume perfect tracking so vehice state is whatever
        // the planner told it to be.
        VehicleState state_idx = current_plan.top();
        current_plan.pop();

        vehicleStates->time = time;
        vehicleStates->x = state_idx.x;
        vehicleStates->vx = state_idx.vx;
        vehicleStates->ax = state_idx.ax;
        vehicleStates->jx = state_idx.jx;


        // Write output to file to plot in python later
        writeTrajectory(vehicleOutputLog, *vehicleStates);

        if( time >= time_obstacle_appears ) {
            writeObstacle(obstacleOutputLog, time, d_start, v_rel);
            d_start += v_rel * dt;
        }

        // Move counters and time forward
        time += dt;
        dt_planner_counter += dt;
        iteration++;
    }

}

// Write to file helper functions
void SingleLaneDrivingSystemTest::writeTrajectory(std::ofstream &vehicleOutputLog, const VehicleState &state) {
    vehicleOutputLog << state.time << "," << state.x << "," << state.vx << "," << state.ax << "," << state.jx << std::endl;
}

void SingleLaneDrivingSystemTest::writeObstacle(std::ofstream &obstacleOutputLog, const double time, const double distance, const double v_rel) {
    obstacleOutputLog << time << "," << distance << "," << v_rel << std::endl;
}

}

using namespace planning;

int main( int argc, char** argv) {

    // Create tester object
    SingleLaneDrivingSystemTest system;

    // Open files where to dump data / results of sim
    std::ofstream vehicleOutputLog("vehicle_output.csv");
    std::ofstream obstacleOutputLog("obstacle_output.csv");

    // initialize test
    system.initializeTest();

    // Run Test
    std::cout << " Running Test ... " << std::endl;
    system.runTest(vehicleOutputLog, obstacleOutputLog);

    // done with test
    vehicleOutputLog.close();
    obstacleOutputLog.close();
    std::cout << " Done With Test ... " << std::endl;

}
