//
// Created by carlo on 16.03.17.
//

#include "Simulator.hpp"

#include <iostream>

#include <g2o/stuff/sampler.h>

const std::vector<double> Simulator::DIRECTION_CHANGE_ANGLES = {-M_PI * 0.5, M_PI * 0.5};

Simulator::RobotPose Simulator::generateNewRobotPose(const Simulator::RobotPose &prev, const g2o::SE2 &trueMotion,
                                                     const Eigen::Vector2d &transNoise, double rotNoise) {

    Simulator::RobotPose newRobotPose;
    newRobotPose.id = prev.id;

    // concatenation of SE elements is overloaded by '*' operator

    // True pose computation
    newRobotPose.truePose = prev.truePose * trueMotion;


    // Simulate new pose
    g2o::SE2 noisyMotion = this->sampleTransformation(trueMotion, transNoise, rotNoise);
    newRobotPose.simulatorPose = prev.simulatorPose * noisyMotion;
    return newRobotPose;
}

g2o::SE2 Simulator::getMotion(int motionDirectionIndex, double stepLen) {
    if (motionDirectionIndex >= DIRECTION_CHANGE_TYPE::NUM_DIRECTION_CHANGE) {
        std::cerr << "Motion direction index is too large" << std::endl;
        std::cerr << "Simulator only supports " << DIRECTION_CHANGE_TYPE::NUM_DIRECTION_CHANGE << " directions"
                  << std::endl;
        return g2o::SE2(stepLen, 0, 0);
    }
    return g2o::SE2(stepLen, 0, DIRECTION_CHANGE_ANGLES[motionDirectionIndex]);
}

g2o::SE2 Simulator::sampleTransformation(const g2o::SE2 &trueMotion, const Eigen::Vector2d &transNoise,
                                         double rotNoise) {

    Eigen::Vector2d trueTranslation = trueMotion.translation();
    double rotation = trueMotion.rotation().angle();

    g2o::SE2 noisyMotion(
            trueTranslation(0) + g2o::Sampler::gaussRand(0.0, transNoise(0)),
            trueTranslation(1) + g2o::Sampler::gaussRand(0.0, transNoise(1)),
            rotation + g2o::Sampler::gaussRand(0.0, rotNoise));
    return noisyMotion;
}

void Simulator::createRobotPoses(int numPoses, int steps, double stepLen, const Eigen::Vector2d &transNoise, const double rotNoise,
                                 const Eigen::Vector2d &bound) {
    // initialize robot position and orientation
    Simulator::RobotPose firstPose;
    firstPose.id = 0;
    firstPose.truePose = g2o::SE2(0, 0, 0);
    firstPose.simulatorPose = g2o::SE2(0, 0, 0);
    m_robotPoses.push_back(firstPose);

    while ((int) m_robotPoses.size() < numPoses) {
        // add straight motion
        for (int i = 0; i < steps && (int) m_robotPoses.size() < numPoses; ++i) {
            Simulator::RobotPose nextRobotPose = this->generateNewRobotPose(m_robotPoses.back(),
                                                                            g2o::SE2(stepLen, 0, 0),
                                                                            transNoise, rotNoise);
            m_robotPoses.push_back(nextRobotPose);
        }
        if ((int) m_robotPoses.size() == numPoses)
            break;

        // sample a new motion direction
        int motionDirectionIndex = (int) (g2o::Sampler::uniformRand(0., 1.) *
                                          DIRECTION_CHANGE_TYPE::NUM_DIRECTION_CHANGE);

        g2o::SE2 nextMotionStep = getMotion(motionDirectionIndex, stepLen);
        Simulator::RobotPose nextRobotPose = generateNewRobotPose(m_robotPoses.back(), nextMotionStep, transNoise,
                                                                  rotNoise);

        // check whether we will walk outside the boundaries in the next iteration
        g2o::SE2 maxPoseChange(stepLen *steps,
        0, 0);
        g2o::SE2 nextStepFinalPose = nextRobotPose.truePose * maxPoseChange;
        if (std::abs(nextStepFinalPose.translation().x()) >= bound[0] ||
            std::abs(nextStepFinalPose.translation().y()) >= bound[1]) {
            // will be outside boundaries using this, sample new pose using an other direction
            for (int i = 0; i < DIRECTION_CHANGE_TYPE::NUM_DIRECTION_CHANGE; ++i) {
                nextMotionStep = getMotion(i, stepLen);
                nextRobotPose = generateNewRobotPose(m_robotPoses.back(), nextMotionStep, transNoise, rotNoise);
                nextStepFinalPose = nextRobotPose.truePose * maxPoseChange;
                if (std::abs(nextStepFinalPose.translation().x()) < bound[0] &&
                    std::abs(nextStepFinalPose.translation().y()) < bound[1])
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
        int ccx = (int) round(it->truePose.translation().x());
        int ccy = (int) round(it->truePose.translation().y());
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
                        double offx = g2o::Sampler::gaussRand(0.0, 1.0);
                        double offy = g2o::Sampler::gaussRand(0.0, 0.0);
                        l->truePose[0] = cx + offx;
                        l->truePose[1] = cy + offy;
                        landmarksForCell.push_back(l);
                    }
                }
            }
    }


}

void Simulator::createLandmarkObservations(double maxSensorRange, double observationProb, const Eigen::Vector2d& landmarkNoise) {
    // sensor range squared
    double maxSensorRange2 = maxSensorRange * maxSensorRange;
    int globalId = 0;
    for (RobotPoseVector::iterator it = m_robotPoses.begin(); it != m_robotPoses.end(); ++it) {
        // get the position of the robot discretized to a grid
        Simulator::RobotPose &pv = *it;
        int ccx = (int) round(pv.truePose.translation().x());
        int ccy = (int) round(pv.truePose.translation().y());
        int numGridCells = (int) (maxSensorRange) + 1;

        // set the real ID to the pose node
        pv.id = globalId++;
        // compute the inverse transformation of the pose, used to compute relative measurements
        g2o::SE2 trueInv = pv.truePose.inverse();

        for (int a = - numGridCells; a <= numGridCells; a++)
            for (int b = -numGridCells; b <= numGridCells; b++) {
                int cx = ccx + a;
                int cy = ccy + b;
                // get the landmarks stored in current cell
                LandmarkPtrVector &landmarksForCell = m_landmarkGrid[cx][cy];
                if (landmarksForCell.size() == 0)
                    continue;
                for (size_t i = 0; i < landmarksForCell.size(); ++i) {
                    Landmark *l = landmarksForCell[i];
                    Eigen::Vector2d distVec = pv.truePose.translation() - l->truePose;
                    double dist2 = distVec.dot(distVec);
                    if (dist2 > maxSensorRange2)
                        continue;
                    double obs = g2o::Sampler::uniformRand(0.0, 1.0);
                    // do we see the landmark?
                    if (obs > observationProb)
                        continue;
                    // if we did not see the landmark yet, set its id
                    if (l->id < 0)
                        l->id = globalId++;
                    if (l->seenBy.size() == 0) {
                        Eigen::Vector2d trueObservation = trueInv * l->truePose;
                        Eigen::Vector2d observation = trueObservation;
                        // corrupt the observation by noise
                        observation[0] += g2o::Sampler::gaussRand(0.0, landmarkNoise[0]);
                        observation[1] += g2o::Sampler::gaussRand(0.0, landmarkNoise[1]);
                        // set the pose as the one seen by the simulated robot node
                        l->simulatedPose = pv.simulatorPose * observation;
                    }
                    l->seenBy.push_back(pv.id);
                    pv.landmarks.push_back(l);
                }
            }

    }
}


void Simulator::simulate(int numPoses, int steps, const Eigen::Vector2d &transNoise, const double rotNoise,
                         const Eigen::Vector2d &bound) {

    double stepLen = 1.0;

    Eigen::VectorXd probLimits;
    probLimits.resize(DIRECTION_CHANGE_TYPE::NUM_DIRECTION_CHANGE);
    for (int i = 0; i < probLimits.size(); ++i)
        probLimits(i) = (i + 1) / (double) DIRECTION_CHANGE_TYPE::NUM_DIRECTION_CHANGE;

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    covariance(0, 0) = transNoise(0) * transNoise(0);
    covariance(1, 1) = transNoise(1) * transNoise(1);
    covariance(2, 2) = rotNoise * rotNoise;
    Eigen::Matrix3d information = covariance.inverse();

    m_robotPoses.clear();

    createRobotPoses(numPoses, steps, stepLen, transNoise, rotNoise, bound);

    createLandmarks();

    // how far the robot sees landmarks
    double maxSensorRange = 2.5 * stepLen;
    // the robot does not always detect the landmark even if it is inside the sensor range
    double observationProb = 0.8;
    // define landmark identification noise
    Eigen::Vector2d landmarkNoise(0.05, 0.05);
    createLandmarkObservations(maxSensorRange, observationProb, landmarkNoise);


    // add the odometry measurements
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

    m_landmarks.clear();
    m_landmarkEdges.clear();
    // add the landmark observations
    {
        Eigen::Matrix2d landmarkCovariance = Eigen::Matrix2d::Zero();
        landmarkCovariance(0, 0) = landmarkNoise[0] * landmarkNoise[0];
        landmarkCovariance(1, 1) = landmarkNoise[1] * landmarkNoise[1];
        Eigen::Matrix2d landmarkInformation = landmarkCovariance.inverse();

        for (size_t i = 0; i < m_robotPoses.size(); ++i) {
            const RobotPose &p = m_robotPoses[i];
            // iterate over all landmarks seen by the current pose
            for (size_t j = 0; j < p.landmarks.size(); ++j) {
                Landmark *l = p.landmarks[j];
                if (l->seenBy.size() > 0 && l->seenBy[0] == p.id) {
                    m_landmarks.push_back(*l);
                }
            }
        }

        for (size_t i = 0; i < m_robotPoses.size(); ++i) {
            const RobotPose &p = m_robotPoses[i];
            g2o::SE2 trueInv = p.truePose.inverse();
            for (size_t j = 0; j < p.landmarks.size(); ++j) {
                Landmark *l = p.landmarks[j];
                // compute the true observation from robot pose to landmark
                Eigen::Vector2d trueObservation = trueInv * l->truePose;
                Eigen::Vector2d observation = trueObservation;
                if (l->seenBy.size() > 0 && l->seenBy[0] == p.id) { // write the initial position of the landmark
                    observation = p.simulatorPose .inverse() * l->simulatedPose;
                } else {
                    // create observation for the LANDMARK using the true positions
                    observation[0] += g2o::Sampler::gaussRand(0.0, landmarkNoise[0]);
                    observation[1] += g2o::Sampler::gaussRand(0.0, landmarkNoise[1]);
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