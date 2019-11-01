////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///     @file       single_lane_driving.h
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

#pragma once

#include <map>
#include <algorithm>
#include <queue>
#include <iostream>
#include <fstream>
#include <cmath>

namespace planning {

//============== Forward-Declarations ===================================================================================
struct CompareMinDistance;
struct CompareMinTime;
class STpoint;
class VehicleState;


//==============  Useful Declarations ===================================================================================

/**
 *  MinDistancePriorityQue -> Priority queue ordered from closest obstacle to furthest
 */
using MinDistancePriorityQue = std::priority_queue< STpoint, std::vector<STpoint>, CompareMinDistance >;

/**
 *  MinTimePriorityQue -> Priority queue ordered from earliest state to latest
 */
using MinTimePriorityQue = std::priority_queue< VehicleState, std::vector<VehicleState>, CompareMinTime >;


//==============  Classes / Structs that Hold Raw Data ==================================================================
/**
 * @brief The STpoint struct
 *
 *          We create a struct with the relevant data for a ST-obstacle point.
 *          This way, it is easy to grow implementation and switch from 1-D to
 *          2-D if necessary.
 */
class STpoint {
public:
    STpoint(const double s, const double t, const double v = 0);
    ~STpoint() = default;

    // ===================== Getters ===========================

    /**
     * @brief getDist
     * @param time  - delta time from current time
     * @return distance from car to obstacle
     *
     * NOTE: For now, we will use this to have descrete points,
     *       So time implementation not called ever
     */
    double getDist() const;
    double getDist(const double time) const;

    /**
     * @brief getSpeed
     * @return return reference to v
     */
    const double& getSpeed() const;


protected:
    double s0       {NAN};              ///< distance at which the obstacle was first detected 1-D
                                             ///< in [meters]

    double v        {NAN};              ///< speed of ST-point, In case we have an estimate and we
                                             ///< want to have a motion model for obstacle [m/s]

    double s        {NAN};              ///< distance from vehicle to obstacle 1-D in [meters]
                                             ///< (s = s0 + v * t)

    double t0       {NAN};              ///< time at which the obstacle was first detected [seconds]

};

/**
 * @brief The TrajectoryLimits class
 */
class TrajectoryLimits {
public:

    double      acc_max     {NAN};      ///< Maximum acceleration [m/s^2]

    double      acc_min     {NAN};      ///< Minimum acceleration (stopping) [m/s^2]

    double      vel_max     {NAN};      ///< Maximum velocity [m/s]

    double      vel_min     {NAN};      ///< Minimum velocity (reverse driving) [m/s]

    double      jerk_max    {NAN};      ///< Maximum Jerk [m/s^3]

    double      k_max       {NAN};      ///< Maximum curvature (turning) [m]

    double      r_max       {NAN};      ///< Maximum yaw rate (turning) [m]

    // ---- helper performance limits -----------------------------------------------

    double      min_stopping_distance       {NAN};      ///< Minimum stopping distance [m]

    double      dist_perception             {NAN};     ///< Distance in which we can detect obstacles
                                                            ///< with most confidence [m]

    TrajectoryLimits(){}

    /**
     * @brief getStoppingDistance
     * @param vel
     * @return
     */
    double getStoppingDistance(  const double vel );

    /**
     * @brief getStoppingTime
     * @param vel
     * @return
     */
    double getStoppingTime( const double vel );

};

/**
 * @brief The VehicleState class
 */
class VehicleState {
public:

    ///< -------------- Vehicle States ----------------------------------------------------
    double          time        {0.0};          ///< time of the vehicle state [seconds]
    double          x           {0.0};          ///< position of vehicle [m]
    double          vx          {0.0};          ///< velocity of vehicle [m/s]
    double          ax          {0.0};          ///< acceleration of vehicle [m/s^2]
    double          jx          {0.0};          ///< jerk of vehicle [m/s^3]

    ///< Constructor(s)
    VehicleState() = default;
    VehicleState(const double time, const double x, const double vx, const double ax, const double jx);

    ///< Destructor(s)
    ~VehicleState() = default;

};

//==============  Priority Queue Comparators ============================================================================
struct CompareMinDistance {
    bool operator()(const STpoint& lhs, const STpoint& rhs) {
        return ( lhs.getDist() > rhs.getDist() );
    }
};

struct CompareMinTime {
    bool operator()(const VehicleState& lhs, const VehicleState& rhs) {
        return ( lhs.time > rhs.time );
    }
};


//==============  Single Lane Driving Classes ===========================================================================
/**
 * @brief The SingleAxisTrajectory class
 *
 *        Creates a polynomial trajectory obtained by minimizing snap
 *        From the work of:
 *
 */
class SingleAxisTrajectory {
public:

    SingleAxisTrajectory( TrajectoryLimits* limitsPtr );
    ~SingleAxisTrajectory() = default;

    ///< ------------ Getters ------------------------------------------------------------------------
    double pos(const double time) const;

    double vel(const double time) const;

    double acc(const double time) const;

    double jerk(const double time) const;

    VehicleState state(const double time) const;

//    double getMaxAccInSegment(const double ti, const double tf);
//    double getMinAccInSegment(const double ti, const double tf);
//    double getMaxJerkInSegment(const double ti, const double tf);
//    double getMaxJerkSquaredInSegment( const double ti, const double tf );

    std::pair<double, double> getMinMaxAccInSegment( const double ti, const double tf );

    ///< ------------ Main Function(s) ---------------------------------------------------------------

    /**
     * @brief planTrajectory
     *
     *          Given Initial and End states, gets the values of alpha, beta, and gamma that
     *          define the minimum snap polynomial to get from initial to goal state
     *
     * @param time_segment
     * @param pos_goal
     * @param vel_goal
     * @param acc_goal
     * @return
     */
    bool planTrajectory(const double time_segment,
                        const double pos_goal = NAN, const double vel_goal = NAN,
                        const double acc_goal = NAN);

    bool planTrajectory( const VehicleState& state_init, const double time_segment,
                         const double pos_goal = NAN, const double vel_goal = NAN,
                         const double acc_goal = NAN);

    bool planTrajectory( VehicleState* state_init, const double time_segment,
                         const double pos_goal = NAN, const double vel_goal = NAN,
                         const double acc_goal = NAN);

    /**
     * @brief getFeasibilityBySampling
     *
     *          In theory, it is possible to get the maximum accelerations, speeds, jerk, etc. by
     *          computing the roots of the polynomial. Especially if planning in 1D. However,
     *          for a faster development, sample the entire trajectory at multiple time intervals,
     *          and check that the limits are not violated.
     *
     * @param time_segment
     * @param number_of_samples
     * @return
     */
    bool getFeasibilityBySampling( const double time_segment, const int number_of_samples = 100 );


private:

    bool        is_init         {false};            ///< Flag to know if we initialized trajectory
    bool        is_feasible     {false};            ///< Flag to know if trajectory is feasible

    double      pos_init        {NAN};         ///< Initial Position in [m]
    double      vel_init        {NAN};         ///< Initial Velocity in [m/s]
    double      acc_init        {NAN};         ///< Initial Acceleration [m/s^2]

    double      pos_goal        {NAN};         ///< Goal Position in [m]
    double      vel_goal        {NAN};         ///< Goal Velocity in [m/s]
    double      acc_goal        {NAN};         ///< Goal Acceleration [m/s^2]

    double      alpha           {NAN};
    double      beta            {NAN};
    double      gamma           {NAN};

    double      cost            {NAN};         ///< Cost of Trajectory

    // Trajectory polynomial constant coeffs
    const double        k0              {1.0};              ///< Multiply by t^0
    const double        k1              {1.0};              ///< Multiply by t^1
    const double        k2              {0.5};              ///< Multiply by t^2
    const double        k3              {1.0/6.0};          ///< Multiply by t^3
    const double        k4              {1.0/24.0};         ///< Multiply by t^4
    const double        k5              {1.0/120.0};        ///< Multiply by t^5

    TrajectoryLimits*   pLimits {nullptr};          ///< pointer to trajectory limits


};

/**
 * @brief The STGraphObstacles class
 *
 *          The STGraphObstacles implements the class ( or base class ) of an
 *          STGraph for an obstacle. The ST-graph is represented as a map in order
 *          to have quick access to obstacle data at any time. the map allows us to
 *          use lower_bound and upper_bound to interpolate at points we do not have
 *          discrete data.
 *
 */
class STGraphObstacles
{
public:
    STGraphObstacles() {}
    ~STGraphObstacles() = default;

    /**
     * @brief getObstacles
     * @param time
     * @return MinDistancePriorityQue - queue of obstacles with closest obstacles at the top of the queue
     *                                  we care about the closest obstacles
     */
    MinDistancePriorityQue getObstacles( const double time );

    /**
     * @brief addObstacleDiscrete
     * @param obstacle
     */
    void addObstacleDiscrete( const std::map<double, STpoint>& obstacle );
    void addObstacleDiscrete( const std::vector< std::pair< double, STpoint> >& obstacle );
    void addObstacleDiscrete( const std::vector<double>& times, const std::vector<STpoint>& obstacle);

    /**
     * @brief reset the map
     */
    void reset();

protected:

    // Time graph of STpoints.
    // Key - double - delta time from current instance in [seconds]
    // value - std::vector<STpoint> - vector of obstacle objects.
    std::map<double, MinDistancePriorityQue > STMap;

};

/**
 * @brief The SingleLaneDriving class
 */
class SingleLaneDriving {
public:

    SingleLaneDriving( STGraphObstacles* obstacleMapPtr, VehicleState* vehicleStatePtr,
                       TrajectoryLimits* trajLimitsPtr, const int max_iteration_speed_loop = 20 );

    ~SingleLaneDriving() = default;

    /**
     * @brief drive
     *
     * @return ST-graph of vehicle states from 0 (time it was called) to T (time segment of lookup time)
     */
    MinTimePriorityQue drive(const double time_lookup, const MinTimePriorityQue& curr_plan);


protected:
    /**
     * @brief predict - given current state, predict state in time [s] ahead assuming no controls
     * @param time
     * @return
     */
    VehicleState predict(const double time);

    // Pointers that need to be defined
    STGraphObstacles*       pObstacleMap        {nullptr};      //< pointer to ST-graph of obstacles
    VehicleState*           pState              {nullptr};      //< pointer to Vehicle State
    TrajectoryLimits*       pLimits             {nullptr};      //< pointer to Vehicle Limits

    // Member variables
    SingleAxisTrajectory    polyTrajGenerator;
    MinTimePriorityQue  STtraj;

    const int max_iterations {20};

};


//==============  Single Lane Driving Test Class ========================================================================
/**
 *  @brief  SingleLaneDrivingSystemTest
 *
 */
class SingleLaneDrivingSystemTest {
public:

    /**
     * @brief SingleLaneDrivingSystemTest
     */
    SingleLaneDrivingSystemTest();
    ~SingleLaneDrivingSystemTest();

    STGraphObstacles*       obstacleMap;
    VehicleState*           vehicleStates;
    TrajectoryLimits*       trajLimits;
    SingleLaneDriving*      driver;


    /**
     * @brief initializeTest
     */
    void initializeTest();

    /**
     * @brief runTest
     */
    void runTest(std::ofstream& vehicleOutputLog, std::ofstream& obstacleOutputLog);

    static void writeTrajectory( std::ofstream& vehicleOutputLog, const VehicleState& state );
    static void writeObstacle( std::ofstream& obstacleOutputLog, const double time, const double distance, const double v_rel );

};





}
