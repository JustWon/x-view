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

#include <g2o/types/slam2d/edge_pointxy.h>
#include <g2o/types/slam2d/vertex_point_xy.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <g2o/stuff/command_args.h>
#include <g2o/core/factory.h>
#include <g2o/stuff/sampler.h>

int main(int argc, char **argv) {

    // Initialize the optimization problem
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> > SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // allocating the optimizer
    g2o::SparseOptimizer optimizer;
    SlamLinearSolver *linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);

    optimizer.setAlgorithm(solver);

    const double noise = 0.5;

    Eigen::Matrix2d translationCovariance = Eigen::Matrix2d::Zero();
    translationCovariance(0, 0) = noise * noise;
    translationCovariance(1, 1) = noise * noise;

    Eigen::Matrix2d information = translationCovariance.inverse();

    std::vector<g2o::VertexPointXY *> vertices;
    std::vector<g2o::EdgePointXY *> edges;

    for (int i = 0; i < 4; i++) {
        vertices.push_back(new g2o::VertexPointXY);
        vertices.back()->setId(i);
    }

    Eigen::Vector2d offset(1,1);
    vertices[0]->setEstimate(offset + Eigen::Vector2d(0, 0));
    vertices[1]->setEstimate(offset +Eigen::Vector2d(1, 0));
    vertices[2]->setEstimate(offset + Eigen::Vector2d(1, 1));
    vertices[3]->setEstimate(offset + Eigen::Vector2d(0, 1));


    for (size_t i = 0; i < vertices.size(); ++i) {
        g2o::VertexPointXY *cur = vertices[i];
        g2o::VertexPointXY *next = vertices[(i + 1) % vertices.size()];

        Eigen::Vector2d t = next->estimate() - cur->estimate();
        g2o::EdgePointXY *e = new g2o::EdgePointXY;
        e->setVertex(0, cur);
        e->setVertex(1, next);
        e->setMeasurement(t);
        e->setInformation(information);
        edges.push_back(e);
    }

    g2o::GaussianSampler<Eigen::Vector2d, Eigen::Matrix2d> transSampler;
    transSampler.setDistribution(translationCovariance);

    // noise for all the edges
    for (size_t i = 0; i < edges.size(); ++i) {
        g2o::EdgePointXY *e = edges[i];
        Eigen::Vector2d relTrans = e->measurement();

        relTrans = relTrans + transSampler.generateSample();
        e->setMeasurement(relTrans);
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

    optimizer.save("before.g2o");
    std::cerr << "Optimizing" << std::endl;
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    std::cerr << "done." << std::endl;

    optimizer.save("after.g2o");


    return 0;
}
