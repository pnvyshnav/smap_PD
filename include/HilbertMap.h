#pragma once

#include "Observation.hpp"
#include "BeliefVoxel.h"
#include "Observable.hpp"

/**
 * Ramos, F., & Ott, L. (2016).
 * Hilbert maps: Scalable continuous occupancy mapping with stochastic gradient descent.
 * The International Journal of Robotics Research, 3514, 1717–1730.
 * https://doi.org/10.1177/0278364916684382
 */

/**
 * Inspired by the authors' Python implementation.
 * Each measurement input consists of the points and occupancy information,
 * which is sampled to an m-dimensional space using the Nystroem method.
 * Then, SVM regression is performed using an RBF kernel, SGD and log loss.
 */

enum HilbertMapFeature
{
    FEATURE_NYSTROEM,
    FEATURE_FOURIER, // not implemented
    FEATURE_SPARSE   // not implemented
};

class HilbertMap : public Observable
{
public:
    HilbertMap(HilbertMapFeature feature,
               int components = 1000,
               double rbfGamma = 0.001);

    bool update(const Observation &observation);

    Belief belief(const octomap::point3d& position);

private:
    const HilbertMapFeature _featureType;
    int _components;
    double _rbfGamma;
};
