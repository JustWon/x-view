#include "Simulator.hpp"

#include <iostream>
#include <math.h>
#include <cstdlib>

#include <gtsam/linear/Sampler.h>

namespace gtsam_example {


    double uniformRand(double lowerBndr = 0.0, double upperBndr = 1.0) {
        return lowerBndr + ((double) std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
    }

    double gaussRand(double mean = 0.0, double sigma = 1.0) {
        double x, y, r2;
        do {
            x = -1.0 + 2.0 * uniformRand(0.0, 1.0);
            y = -1.0 + 2.0 * uniformRand(0.0, 1.0);
            r2 = x * x + y * y;
        } while (r2 > 1.0 || r2 == 0.0);
        return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
    }


    const std::vector<double> Simulator::DIRECTION_CHANGE_ANGLES = {-M_PI * 0.5, M_PI * 0.5};

    Simulator::Simulator()
            : m_transNoise(Eigen::Vector2d(0.01, 0.01)),
              m_rotNoise(0.02),
              m_rangeNoise(0.05),
              m_bearingNoise(0.02) {}

    Simulator::RobotPose Simulator::generateNewRobotPose(const Simulator::RobotPose &prev,
                                                         const gtsam::Pose2 &trueMotion) {

        Simulator::RobotPose newRobotPose;
        newRobotPose.id = prev.id;

        // concatenation of Pose elements is overloaded by '*' operator

        // True pose computation
        newRobotPose.truePose = prev.truePose * trueMotion;

        // Simulate new pose
        gtsam::Pose2 noisyMotion = sampleTransformation(trueMotion);
        newRobotPose.simulatorPose = prev.simulatorPose * noisyMotion;
        return newRobotPose;
    }

    gtsam::Pose2 Simulator::getMotion(int motionDirectionIndex, double stepLen) {
        if (motionDirectionIndex >= DIRECTION_CHANGE_TYPE::NUM_DIRECTION_CHANGE) {
            std::cerr << "Motion direction index is too large" << std::endl;
            std::cerr << "Simulator only supports " << DIRECTION_CHANGE_TYPE::NUM_DIRECTION_CHANGE << " directions"
                      << std::endl;
            return gtsam::Pose2(stepLen, 0.0, 0.0);
        }
        return gtsam::Pose2(stepLen, 0, DIRECTION_CHANGE_ANGLES[motionDirectionIndex]);
    }


    gtsam::Pose2 Simulator::sampleTransformation(const gtsam::Pose2 &trueMotion) {

        Eigen::Vector2d trueTranslation = trueMotion.translation();
        double trueRotation = trueMotion.theta();


        Eigen::Vector3d sample = m_motionSampler.sample();
        return gtsam::Pose2(trueTranslation(0) + sample(0),
                            trueTranslation(1) + sample(1),
                            trueRotation + sample(2));

    }


    void Simulator::createRobotPoses(int numPoses, int steps, double stepLen) {
        // initialize robot position and orientation
        Simulator::RobotPose firstPose;
        firstPose.id = 0;
        firstPose.truePose = firstPose.simulatorPose = gtsam::Pose2(0, 0, 0);
        m_robotPoses.push_back(firstPose);

        while ((int) m_robotPoses.size() < numPoses) {
            // add straight motion
            for (int i = 0; i < steps && (int) m_robotPoses.size() < numPoses; ++i) {
                Simulator::RobotPose nextRobotPose = generateNewRobotPose(m_robotPoses.back(),
                                                                          gtsam::Pose2(stepLen, 0, 0));
                m_robotPoses.push_back(nextRobotPose);
            }
            if ((int) m_robotPoses.size() == numPoses)
                break;

            // sample a new motion direction
            int motionDirectionIndex = (int) (uniformRand() * DIRECTION_CHANGE_TYPE::NUM_DIRECTION_CHANGE);

            gtsam::Pose2 nextMotionStep = getMotion(motionDirectionIndex, stepLen);
            Simulator::RobotPose nextRobotPose = generateNewRobotPose(m_robotPoses.back(), nextMotionStep);

            // check whether we will walk outside the boundaries in the next iteration
            gtsam::Pose2 maxPoseChange(stepLen * steps, 0, 0);
            gtsam::Pose2 nextStepFinalPose = nextRobotPose.truePose * maxPoseChange;
            if (std::abs(nextStepFinalPose.x()) >= m_bound[0] ||
                std::abs(nextStepFinalPose.y()) >= m_bound[1]) {
                // will be outside boundaries using this, sample new pose using an other direction
                for (int i = 0; i < DIRECTION_CHANGE_TYPE::NUM_DIRECTION_CHANGE; ++i) {
                    nextMotionStep = getMotion(i, stepLen);
                    nextRobotPose = generateNewRobotPose(m_robotPoses.back(), nextMotionStep);
                    nextStepFinalPose = nextRobotPose.truePose * maxPoseChange;
                    if (std::abs(nextStepFinalPose.x()) < m_bound[0] &&
                        std::abs(nextStepFinalPose.y()) < m_bound[1])
                        break;
                }
            }

            m_robotPoses.push_back(nextRobotPose);
        }
    }

    void Simulator::createLandmarks() {
        // creating landmarks along the trajectory

        // how far can the landmarks be from the robot trajectory
        int landmarksRange = 2;
        int landmarksPerUnitArea = 1;

        for (RobotPoseVector::const_iterator it = m_robotPoses.begin(); it != m_robotPoses.end(); ++it) {
            // get the real position of the robot's pose in a discretized grid
            int ccx = (int) round(it->truePose.x());
            int ccy = (int) round(it->truePose.y());
            for (int a = -landmarksRange; a <= landmarksRange; a++)
                for (int b = -landmarksRange; b <= landmarksRange; b++) {
                    int cx = ccx + a;
                    int cy = ccy + b;
                    // get the landmarks already stored in this cell
                    LandmarkPtrVector &landmarksForCell = m_landmarkGrid[cx][cy];
                    // if there are no landmarks, generate them
                    if (landmarksForCell.size() == 0) {
                        for (int i = 0; i < landmarksPerUnitArea; ++i) {
                            Landmark *l = new Landmark();
                            double offx = gaussRand();
                            double offy = gaussRand();
                            l->truePose[0] = cx + offx;
                            l->truePose[1] = cy + offy;
                            landmarksForCell.push_back(l);
                        }
                    }
                }
        }


    }

    void Simulator::createLandmarkObservations(double maxSensorRange, double observationProb) {
        // sensor range squared
        double maxSensorRange2 = maxSensorRange * maxSensorRange;
        int globalId = 0;
        for (RobotPoseVector::iterator it = m_robotPoses.begin(); it != m_robotPoses.end(); ++it) {
            // get the position of the robot discretized to a grid
            Simulator::RobotPose &pv = *it;
            int ccx = (int) round(pv.truePose.x());
            int ccy = (int) round(pv.truePose.y());
            int numGridCells = (int) (maxSensorRange) + 1;

            // set the real ID to the pose node
            pv.id = globalId++;
            // compute the inverse transformation of the pose, used to compute relative measurements
            gtsam::Pose2 trueInv = pv.truePose.inverse();

            for (int a = -numGridCells; a <= numGridCells; a++)
                for (int b = -numGridCells; b <= numGridCells; b++) {
                    int cx = ccx + a;
                    int cy = ccy + b;
                    // get the landmarks stored in current cell
                    LandmarkPtrVector &landmarksForCell = m_landmarkGrid[cx][cy];
                    if (landmarksForCell.size() == 0)
                        continue;
                    for (size_t i = 0; i < landmarksForCell.size(); ++i) {
                        Landmark *l = landmarksForCell[i];
                        // check if this landmark has been seen by current robot pose
                        Eigen::Vector2d distVec = pv.truePose.translation() - l->truePose;
                        double dist2 = distVec.dot(distVec);
                        if (dist2 > maxSensorRange2)
                            continue;
                        double obs = uniformRand();
                        // do we see the landmark?
                        if (obs > observationProb)
                            continue;
                        // if landmark was never seen by any pose, set its id
                        if (l->id < 0)
                            l->id = globalId++;
                        if (l->seenBy.size() == 0) {
                            // compute relative position of the landmark w.r.t real robot pose
                            Eigen::Vector2d trueObservation = trueInv * l->truePose;
                            Eigen::Vector2d simulatedObservation = trueObservation + m_landmarkSampler.sample();
                            // set the pose as the one seen by the simulated robot node
                            l->simulatedPose = pv.simulatorPose * simulatedObservation;
                        }
                        l->seenBy.push_back(pv.id);
                        pv.landmarks.push_back(l);
                    }
                }

        }
    }


    void Simulator::simulate(int numPoses, int steps) {

        // generate the motion sampler used to generate new motions based on the noise
        m_motionSampler = gtsam::Sampler(Eigen::Vector3d(m_transNoise(0), m_transNoise(1), m_rotNoise));
        m_landmarkSampler = gtsam::Sampler(Eigen::Vector2d(m_rangeNoise, m_bearingNoise));

        double stepLen = 1.0;

        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        covariance(0, 0) = m_transNoise(0) * m_transNoise(0);
        covariance(1, 1) = m_transNoise(1) * m_transNoise(1);
        covariance(2, 2) = m_rotNoise * m_rotNoise;
        Eigen::Matrix3d information = covariance.inverse();

        m_robotPoses.clear();

        createRobotPoses(numPoses, steps, stepLen);

        createLandmarks();


        // how far the robot sees landmarks
        double maxSensorRange = 2.5 * stepLen;
        // the robot does not always detect the landmark even if it is inside the sensor range
        double observationProb = 0.8;

        createLandmarkObservations(maxSensorRange, observationProb);


        // add the odometry constraints
        m_odometryEdges.clear();
        for (size_t i = 1; i < m_robotPoses.size(); ++i) {
            const RobotPose &prev = m_robotPoses[i - 1];
            const RobotPose &p = m_robotPoses[i];

            OdometryEdge edge;

            edge.from = prev.id;
            edge.to = p.id;
            edge.trueTransf = prev.truePose.inverse() * p.truePose;
            edge.simulatorTransf = prev.simulatorPose.inverse() * p.simulatorPose;
            edge.information = information;

            m_odometryEdges.push_back(edge);
        }

        m_landmarkEdges.clear();
        m_landmarks.clear();
        // add the landmark observations constraints
        {
            Eigen::Matrix2d landmarkCovariance = Eigen::Matrix2d::Zero();
            landmarkCovariance(0, 0) = m_rangeNoise*m_rangeNoise;
            landmarkCovariance(1, 1) = m_bearingNoise*m_bearingNoise;
            Eigen::Matrix2d landmarkInformation = landmarkCovariance.inverse();

            // add only seen landmarks to the global landmark container
            for (RobotPoseVector::const_iterator it = m_robotPoses.begin(); it != m_robotPoses.end(); ++it) {
                const RobotPose &p = *it;
                // iterate over all landmarks seen by the current pose
                for (size_t j = 0; j < p.landmarks.size(); ++j) {
                    Landmark *l = p.landmarks[j];
                    if (l->seenBy.size() > 0 && l->seenBy[0] == p.id) {
                        m_landmarks.push_back(*l);
                    }
                }
            }

            for (RobotPoseVector::const_iterator it = m_robotPoses.begin(); it != m_robotPoses.end(); ++it) {
                const RobotPose &p = *it;
                gtsam::Pose2 trueInv = p.truePose.inverse();
                for (size_t j = 0; j < p.landmarks.size(); ++j) {
                    Landmark *l = p.landmarks[j];
                    // compute the true observation from robot pose to landmark
                    Eigen::Vector2d trueObservation = trueInv * l->truePose;
                    Eigen::Vector2d observation = trueObservation;
                    if (l->seenBy.size() > 0 && l->seenBy[0] == p.id) { // write the initial position of the landmark
                        observation = p.simulatorPose.inverse() * l->simulatedPose;
                    } else {
                        // create observation for the landmark using the true positions
                        observation += m_landmarkSampler.sample();
                    }

                    LandmarkEdge edge;

                    edge.from = p.id;
                    edge.to = l->id;
                    edge.trueMeas = trueObservation;
                    edge.simulatorMeas = observation;
                    edge.information = landmarkInformation;

                    m_landmarkEdges.push_back(edge);
                }
            }
        }


        // cleaning up
        for (LandmarkGrid::iterator it = m_landmarkGrid.begin(); it != m_landmarkGrid.end(); ++it) {
            for (std::map<int, Simulator::LandmarkPtrVector>::iterator itt = it->second.begin();
                 itt != it->second.end(); ++itt) {
                Simulator::LandmarkPtrVector &landmarks = itt->second;
                for (size_t i = 0; i < landmarks.size(); ++i)
                    delete landmarks[i];
            }
        }

    }

}