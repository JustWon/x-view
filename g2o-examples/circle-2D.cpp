// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <g2o/stuff/command_args.h>
#include <g2o/core/factory.h>
#include <g2o/stuff/sampler.h>

int main(int argc, char **argv) {
    // command line parsing
    int numLaps;
    int numNodesPerLap;
    double radius;
    double noiseTranslationX, noiseTranslationY, noiseRotation;
    std::string outFilename;

    g2o::CommandArgs arg;
    arg.param("o", outFilename, "-", "output filename");
    arg.param("laps", numLaps, 50, "how many times the robot travels around the circle");
    arg.param("nodesPerLap", numNodesPerLap, 50, "how many nodes per lap");
    arg.param("radius", radius, 100., "radius of the circle");
    arg.param("noiseTranslationX", noiseTranslationX, 0.01, "set the noise level for the translation in X direction");
    arg.param("noiseTranslationY", noiseTranslationY, 0.01, "set the noise level for the translation in Y direction");
    arg.param("noiseRotation", noiseRotation, 0.01, "set the noise level for the rotation");
    arg.parseArgs(argc, argv);


    // Initialize the optimization problem
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // allocating the optimizer
    g2o::SparseOptimizer optimizer;
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);

    optimizer.setAlgorithm(solver);


    Eigen::Matrix2d translationCovariance = Eigen::Matrix2d::Zero();
    translationCovariance(0, 0) = noiseTranslationX * noiseTranslationX;
    translationCovariance(1, 1) = noiseTranslationY * noiseTranslationY;

    double rotationCovariance = noiseRotation * noiseRotation;

    Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
    information.block(0, 0, 2, 2) = translationCovariance.inverse();
    information(2, 2) = 1.0 / rotationCovariance;

    std::vector<g2o::VertexSE2 *> vertices;
    std::vector<g2o::EdgeSE2 *> edges;
    int id = 0;
    for (int f = 0; f < numLaps; ++f) {
        for (int n = 0; n < numNodesPerLap; ++n) {
            g2o::VertexSE2 *v = new g2o::VertexSE2;
            v->setId(id++);

            double angle = -M_PI + 2.0 * n * M_PI / numNodesPerLap;
            Eigen::Vector2d pos;
            pos(0) = radius * std::cos(angle);
            pos(1) = radius * std::sin(angle);

            g2o::SE2 t(pos(0), pos(1), angle);
            v->setEstimate(t);
            vertices.push_back(v);
        }
    }

    // generate odometry edges
    for (size_t i = 1; i < vertices.size(); ++i) {
        g2o::VertexSE2 *prev = vertices[i - 1];
        g2o::VertexSE2 *cur = vertices[i];
        g2o::SE2 t = prev->estimate().inverse() * cur->estimate();
        g2o::EdgeSE2 *e = new g2o::EdgeSE2;
        e->setVertex(0, prev);
        e->setVertex(1, cur);
        e->setMeasurement(t);
        e->setInformation(information);
        edges.push_back(e);
    }

    g2o::GaussianSampler<Eigen::Vector2d, Eigen::Matrix2d> transSampler;
    transSampler.setDistribution(translationCovariance);

    // noise for all the edges
    for (size_t i = 0; i < edges.size(); ++i) {
        g2o::EdgeSE2 *e = edges[i];
        double relRot = e->measurement().rotation().angle();
        Eigen::Vector2d relTrans = e->measurement().translation();

        double rotNoise = g2o::Sampler::gaussRand(0.0, rotationCovariance);
        Eigen::Vector2d transNoise = transSampler.generateSample();

        relRot = relRot + rotNoise;
        if (relRot > 2 * M_PI)
            relRot -= 2 * M_PI;
        else if (relRot < 0.0)
            relRot += 2 * M_PI;

        relTrans = relTrans + transNoise;
        relTrans *= 1.5;

        g2o::SE2 noisyMeasurement(relTrans(0), relTrans(1), relRot);
        e->setMeasurement(noisyMeasurement);
    }

    // concatenate all the edge constraints to compute the initial state
    for (size_t i = 0; i < edges.size(); ++i) {
        g2o::EdgeSE2 *e = edges[i];
        g2o::VertexSE2 *from = static_cast<g2o::VertexSE2 *>(e->vertex(0));
        g2o::VertexSE2 *to = static_cast<g2o::VertexSE2 *>(e->vertex(1));
        g2o::HyperGraph::VertexSet aux;
        aux.insert(from);
        e->initialEstimate(aux, to);
    }

    // Add data to the optimizer
    for (size_t i = 0; i < vertices.size(); ++i) {
        optimizer.addVertex(vertices[i]);
    }
    for (size_t i = 0; i < edges.size(); ++i) {
        optimizer.addEdge(edges[i]);
    }

    optimizer.vertex(0)->setFixed(true);
    optimizer.setVerbose(true);

    optimizer.save(("before_" + outFilename).c_str());
    std::cerr << "Optimizing" << std::endl;
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    std::cerr << "done." << std::endl;

    optimizer.save(("after_" + outFilename).c_str());

    // write output
    std::ofstream fileOutputStream;
    if (outFilename != "-") {
        std::cout << "Writing into " << outFilename << std::endl;
        fileOutputStream.open(outFilename.c_str());
    } else {
        std::cerr << "writing to stdout" << std::endl;
    }

    std::string vertexTag = g2o::Factory::instance()->tag(vertices[0]);
    std::string edgeTag = g2o::Factory::instance()->tag(edges[0]);

    std::ostream &fout = outFilename != "-" ? fileOutputStream : std::cout;
    for (size_t i = 0; i < vertices.size(); ++i) {
        g2o::VertexSE2 *v = vertices[i];
        fout << vertexTag << " " << v->id() << " ";
        v->write(fout);
        fout << std::endl;
    }

    for (size_t i = 0; i < edges.size(); ++i) {
        g2o::EdgeSE2 *e = edges[i];
        g2o::VertexSE2 *from = static_cast<g2o::VertexSE2 *>(e->vertex(0));
        g2o::VertexSE2 *to = static_cast<g2o::VertexSE2 *>(e->vertex(1));
        fout << edgeTag << " " << from->id() << " " << to->id() << " ";
        e->write(fout);
        fout << std::endl;
    }

    return 0;
}
