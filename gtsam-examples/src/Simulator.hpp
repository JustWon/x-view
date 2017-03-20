//
// Created by carlo on 17.03.17.
//

#ifndef GTSAM_EXAMPLES_SIMULATOR_HPP
#define GTSAM_EXAMPLES_SIMULATOR_HPP


#include <gtsam/inference/Symbol.h>

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>

#include <gtsam/linear/Sampler.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

namespace gtsam_example {

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
            gtsam::Point2 truePose;
            gtsam::Point2 simulatedPose;
            // list of indices referring to robot-poses that see this landmark
            std::vector<int> seenBy;

            Landmark() : id(-1) {}

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        };

        typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark> > LandmarkVector;
        typedef std::vector<Landmark *> LandmarkPtrVector;

        /**
         * @brief: simulated pose of the robot, consisting of a 2D position and an orientation (angle)
         */
        struct RobotPose {

            int id;
            gtsam::Pose2 truePose;
            gtsam::Pose2 simulatorPose;
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
            gtsam::Pose2 trueTransf;
            gtsam::Pose2 simulatorTransf;
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

        Simulator();

        /**
         * @brief simulates the motion of o robot
         * @param numPoses number of poses to simulate
         * @param steps steps to take before changing direction
         * @param transNoise transition noise (x is frontal direction, y lateral direction in robot frame)
         * @param rotNoise rotation noise
         * @param bound size of the word
         */
        void simulate(int numPoses, int steps);

        void setTranslationNoise(const Eigen::Vector2d &transNoise, const double rotNoise) {
            m_transNoise = transNoise;
            m_rotNoise = rotNoise;
        }

        void setLandmarkDetectionNoise(const double rangeNoise, const double bearingNoise) {
            m_rangeNoise = rangeNoise;
            m_bearingNoise = bearingNoise;
        }

        const RobotPoseVector &poses() const { return m_robotPoses; }

        const LandmarkVector &landmarks() const { return m_landmarks; }

        const OdometryEdgeVector &odometryEdges() const { return m_odometryEdges; }

        const LandmarkEdgeVector &landmarkEdges() const { return m_landmarkEdges; }


    private:
        // odometry parameters
        Eigen::Vector2d m_transNoise;
        double m_rotNoise;
        gtsam::Sampler m_motionSampler;

        // landmark detection parameters
        double m_rangeNoise, m_bearingNoise;
        gtsam::Sampler m_landmarkSampler;

        Eigen::Vector2d m_bound;

        RobotPoseVector m_robotPoses;
        LandmarkVector m_landmarks;
        OdometryEdgeVector m_odometryEdges;
        LandmarkEdgeVector m_landmarkEdges;
        LandmarkGrid m_landmarkGrid;


    public:
        /**
         * @see simulate(...)
         */
        void createRobotPoses(int numPoses, int steps, double stepLen);

        /**
         * @brief generates landmarks along the path followed by the robot
         */
        void createLandmarks();

        /**
         * @brief generates observation constrains between robot poses and landmarks
         * @param maxSensorRange range of the sensor after which no landmark can be seen
         * @param observationProb probability to detect a landmark if it is inside the range
         */
        void createLandmarkObservations(double maxSensorRange, double observationProb);

        /**
         * @brief generates a new robot pose (true and simulated data) from the current position given a true motion object and associated noise
         * @param prev current robot pose
         * @param trueMotion true motion object to be applied to current robot pose
         * @return new robot pose defined as the previous one after application of motion and noise
         */
        RobotPose generateNewRobotPose(const RobotPose &prev, const gtsam::Pose2 &trueMotion);

        /**
         * @brief generates motion object given a step length, and an index referring to the motion type (forward, left, right etc) to be performed
         * @param motionDirectionIndex index referring to Simulator::DIRECTION_CHANGE_TYPE
         * @param stepLen length of the step to be taken
         * @return new motion object representing the motion specified by the step length and the motion type
         */
        gtsam::Pose2 getMotion(int motionDirectionIndex, double stepLen);

        /**
         * @brief generates a new motion object
         * @param trueMotion uncorrupted motion object
         * @return new motion object similar to the trueMotion parameter passed as argument,
         * but corrupted by the noise specified in the simulate(...) function
         */
        gtsam::Pose2 sampleTransformation(const gtsam::Pose2 &trueMotion);

    };

}


#endif //GTSAM_EXAMPLES_SIMULATOR_HPP
