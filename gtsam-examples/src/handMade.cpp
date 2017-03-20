/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file PlanarSLAMExample.cpp
 * @brief Simple robotics example using odometry measurements and bearing-range (laser) measurements
 * @author Alex Cunningham
 */

/**
 * A simple 2D planar slam example with landmarks
 *  - The robot and landmarks are on a 2 meter grid
 *  - Robot poses are facing along the X axis (horizontal, to the right in 2D)
 *  - The robot moves 2 meters each step
 *  - We have full odometry between poses
 *  - We have bearing and range information for measurements
 *  - Landmarks are 2 meters away from the robot trajectory
 */

// As this is a planar SLAM example, we will use Pose2 variables (x, y, theta) to represent
// the robot positions and Point2 variables (x, y) to represent the landmark coordinates.
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use a RangeBearing factor for the range-bearing measurements to identified
// landmarks, and Between factors for the relative motion described by odometry measurements.
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// common Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>


using namespace std;
using namespace gtsam;

int main(int argc, char **argv) {

    // Create a factor graph
    NonlinearFactorGraph graph;

    // Two graphs (g1 and g2) which have odometry constraints (full lines) and landmark observation constraints,
    // only g1_x1 has a prior pose (0,0,0)
    /*
     *            g1_x2....l1.....g2_x1
     *          _/   |              |
     *         /     |              |
     *     g1_x1     |            g2_x2
     *         \_    |              |
     *           \   |              |
     *            g1_x3....l2.....g2_x3
     *
     */

    // Nodes of the two graphs to be merged together by having observed same landmarks
    Symbol g1_x1('x', 1), g1_x2('x', 2), g1_x3('x', 3);
    Symbol g2_x1('x', 4), g2_x2('x', 5), g2_x3('x', 6);

    Pose2 g1_p1, g1_p2, g1_p3;
    Pose2 g2_p1, g2_p2, g2_p3;

    g1_p1 = Pose2(0, 0, 0);
    g1_p2 = Pose2(1, 1, M_PI * 0.25);
    g1_p3 = Pose2(1, -1, -M_PI * 5);

    g2_p1 = Pose2(3, 1, -M_PI * 0.5);
    g2_p2 = Pose2(3, 0, -M_PI * 0.5);
    g2_p3 = Pose2(3, -1, -M_PI * 0.5);

    // landmarks
    Symbol l1('l', 1), l2('l', 2);
    Point2 l1_p;
    l1_p[0] = 2;
    l1_p[1] = 1;
    Point2 l2_p;
    l2_p[0] = 2;
    l2_p[1] = -1;

    // Add a prior on pose x1 at the origin. A prior factor consists of a mean and a noise model (covariance matrix)
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(
            Vector3(0.3, 0.3, 0.1)); // 30cm std on x,y, 0.1 rad on theta
    graph.emplace_shared<PriorFactor<Pose2> >(g1_x1, g1_p1, priorNoise); // add directly to graph

    // Add odometry factors between graph poses
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(
            Vector3(0.2, 0.2, 0.1)); // 20cm std on x,y, 0.1 rad on theta
    graph.emplace_shared<BetweenFactor<Pose2> >(g1_x1, g1_x2, g1_p1.inverse() * g1_p2, odometryNoise);
    graph.emplace_shared<BetweenFactor<Pose2> >(g1_x2, g1_x3, g1_p2.inverse() * g1_p3, odometryNoise);
    graph.emplace_shared<BetweenFactor<Pose2> >(g1_x3, g1_x1, g1_p3.inverse() * g1_p1, odometryNoise);

    graph.emplace_shared<BetweenFactor<Pose2> >(g2_x1, g2_x2, g2_p1.inverse() * g2_p2, odometryNoise);
    graph.emplace_shared<BetweenFactor<Pose2> >(g2_x2, g2_x3, g2_p2.inverse() * g2_p3, odometryNoise);

    // Add Range-Bearing measurements to two different landmarks
    // create a noise model for the landmark measurements
    noiseModel::Diagonal::shared_ptr measurementNoise = noiseModel::Diagonal::Sigmas(
            Vector2(0.1, 0.2)); // 0.1 rad std on bearing, 20cm on range
    // create the measurement values - indices are (pose id, landmark id)

    // Add Bearing-Range factors
    Point2 g1_p2_l1_p = g1_p2.inverse() * l1_p;
    Point2 g1_p3_l2_p = g1_p3.inverse() * l2_p;
    graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(g1_x2, l1, std::atan2(g1_p2_l1_p.y(), g1_p2_l1_p.x()), 1,
                                                             measurementNoise);
    graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(g1_x3, l2, std::atan2(g1_p3_l2_p.y(), g1_p3_l2_p.x()), 1,
                                                             measurementNoise);

    Point2 g2_p1_l1_p = g2_p1.inverse() * l1_p;
    Point2 g2_p3_l2_p = g2_p3.inverse() * l2_p;
    graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(g2_x1, l1, std::atan2(g2_p1_l1_p.y(), g2_p1_l1_p.x()), 1,
                                                             measurementNoise);
    graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(g2_x3, l2, std::atan2(g2_p3_l2_p.y(), g2_p3_l2_p.x()), 1,
                                                             measurementNoise);


    // Print
    graph.print("Factor Graph:\n");

    // Create initial estimate (correct for first graph, shifted and rotated for second graph)
    Values initialEstimate;
    initialEstimate.insert(g1_x1, g1_p1);
    initialEstimate.insert(g1_x2, g1_p2);
    initialEstimate.insert(g1_x3, g1_p3);
    // manually set initial estimate of second graph to be at the origin, ad rotated by 90deg
    initialEstimate.insert(g2_x1, Pose2(0, -1, M_PI*0.5));
    initialEstimate.insert(g2_x2, Pose2(0, 0, M_PI*0.5));
    initialEstimate.insert(g2_x3, Pose2(0, 1, M_PI*0.5));

    initialEstimate.insert(l1, l1_p);
    initialEstimate.insert(l2, l2_p);

    // Print
    initialEstimate.print("Initial Estimate:\n");

    // Optimize using Levenberg-Marquardt optimization. The optimizer
    // accepts an optional set of configuration parameters, controlling
    // things like convergence criteria, the type of linear system solver
    // to use, and the amount of information displayed during optimization.
    // Here we will use the default set of parameters.  See the
    // documentation for the full set of parameters.
    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
    Values result = optimizer.optimize();
    result.print("Final Result:\n");


    return 0;
}

