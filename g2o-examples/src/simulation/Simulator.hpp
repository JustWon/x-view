//
// Created by carlo on 16.03.17.
//

#ifndef G2O_EXAMPLES_SIMULATOR_HPP
#define G2O_EXAMPLES_SIMULATOR_HPP

#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/vertex_point_xy.h>

#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/edge_se2_pointxy.h>

#include <Eigen/StdVector>
#include <Eigen/Core>

#include <math.h>
#include <vector>
#include <map>

class Simulator {

public:

    enum DIRECTION_CHANGE_TYPE {
        RIGHT = 0,
        LEFT,
        NUM_DIRECTION_CHANGE
    };

    static const std::vector<double> DIRECTION_CHANGE_ANGLES;

    /**
     * @brief: a landmark is a feature seen by the robot while travelling through the word
     */
    struct Landmark {

        int id;
        Eigen::Vector2d truePose;
        Eigen::Vector2d simulatedPose;
        // list of indices referring to robot-poses that see this landmark
        std::vector<int> seenBy;

        Landmark() : id(-1) {}

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark> > LandmarkVector;
    typedef std::vector<Landmark *> LandmarkPtrVector;

    /**
     * @brief: simulated pose of the robot, consisting of a 2D position and an orientation
     */
    struct RobotPose {

        int id;
        g2o::SE2 truePose;
        g2o::SE2 simulatorPose;
        // the landmarks observed by this node
        LandmarkPtrVector landmarks;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        RobotPose() : id(-1) {}

    };

    typedef std::vector<RobotPose, Eigen::aligned_allocator<RobotPose> > RobotPoseVector;

    /**
     * @brief: odometry constraint linking two robot poses (relative position and orientation)
     */
    struct OdometryEdge {
        int from;
        int to;
        g2o::SE2 trueTransf;
        g2o::SE2 simulatorTransf;
        Eigen::Matrix3d information;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        OdometryEdge() : from(-1), to(-1) {}
    };

    typedef std::vector<OdometryEdge, Eigen::aligned_allocator<OdometryEdge> > OdometryEdgeVector;

    /**
     * @brief: landmark constraint linking a robot pose to a landmark
     */
    struct LandmarkEdge {
        int from;
        int to;
        Eigen::Vector2d trueMeas;
        Eigen::Vector2d simulatorMeas;
        Eigen::Matrix2d information;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        LandmarkEdge() : from(-1), to(-1) {}
    };

    typedef std::vector<LandmarkEdge, Eigen::aligned_allocator<LandmarkEdge> > LandmarkEdgeVector;

    // sparse representation of a map, where each cell has a vector of pointers to landmarks
    typedef std::map<int, std::map<int, Simulator::LandmarkPtrVector> > LandmarkGrid;


public:

    Simulator() {}

    /**
     * @brief simulates the motion of o robot
     * @param numPoses number of poses to simulate
     * @param steps steps to take before changing direction
     * @param transNoise transition noise (x is frontal direction, y lateral direction in robot frame)
     * @param rotNoise rotation noise
     * @param bound size of the word
     */
    void simulate(int numPoses, int steps, const Eigen::Vector2d &transNoise, const double rotNoise,
                  const Eigen::Vector2d &bound);

    const RobotPoseVector &poses() const { return m_robotPoses; }

    const LandmarkVector &landmarks() const { return m_landmarks; }

    const OdometryEdgeVector &odometryEdges() const { return m_odometryEdges; }

    const LandmarkEdgeVector &landmarkEdges() const { return m_landmarkEdges; }


private:
    RobotPoseVector m_robotPoses;
    LandmarkVector m_landmarks;
    OdometryEdgeVector m_odometryEdges;
    LandmarkEdgeVector m_landmarkEdges;
    LandmarkGrid m_landmarkGrid;

private:
    /**
     * @see simulate(...)
     */
    void createRobotPoses(int numPoses, int steps, double stepLen, const Eigen::Vector2d &transNoise,
                          const double rotNoise, const Eigen::Vector2d &bound);

    /**
     * @brief generates landmarks along the path followed by the robot
     */
    void createLandmarks();

    /**
     * @brief generates observation constrains between robot poses and landmarks
     * @param maxSensorRange range of the sensor after which no landmark can be seen
     * @param observationProb probability to detect a landmark if it is inside the range
     * @param landmarkNoise noise to be added to the landmark detection
     */
    void createLandmarkObservations(double maxSensorRange, double observationProb, const Eigen::Vector2d& landmarkNoise);

    /**
     * @brief generates a new robot pose (true and simulated data) from the current position given a true motion object and associated noise
     * @param prev current robot pose
     * @param trueMotion true motion object to be applied to current robot pose
     * @param transNoise transition noise
     * @param rotNoise  rotation noise
     * @return new robot pose defined as the previous one after application of motion and noise
     */
    RobotPose generateNewRobotPose(const RobotPose &prev, const g2o::SE2 &trueMotion, const Eigen::Vector2d &transNoise,
                                   double rotNoise);

    /**
     * @brief generates motion object given a step length, and an index referring to the motion type (forward, left, right etc) to be performed
     * @param motionDirectionIndex index referring to Simulator::DIRECTION_CHANGE_TYPE
     * @param stepLen length of the step to be taken
     * @return new motion object representing the motion specified by the step length and the motion type
     */
    g2o::SE2 getMotion(int motionDirectionIndex, double stepLen);

    /**
     * @brief generates a new motion object (g2o::SE2 object)
     * @param trueMotion uncorrupted motion object
     * @param transNoise noise to be applied to the tranlation part of the motion object
     * @param rotNoise noise to be applied to the rotational part of the motion object
     * @return new motion object similar to the trueMotion parameter passed as argument, but corrupted by the specified noise
     */
    g2o::SE2 sampleTransformation(const g2o::SE2 &trueMotion, const Eigen::Vector2d &transNoise, double rotNoise);

};


#endif //G2O_EXAMPLES_SIMULATOR_HPP
