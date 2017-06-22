
#include "../include/HilbertMap.h"

#include <mlpack/methods/kmeans/kmeans.hpp>
#include <mlpack/core/kernels/gaussian_kernel.hpp>
#include <mlpack/methods/nystroem_method/nystroem_method.hpp>
#include "../rbf/rbf_interp_2d.hpp"
#include "../svm/svm.h"

HilbertMap::HilbertMap(HilbertMapFeature feature, int components, double rbfGamma)
: _featureType(feature), _components(components), _rbfGamma(rbfGamma)
{

}

bool HilbertMap::update(const Observation &observation)
{
    using namespace mlpack::kmeans;

// The dataset we are clustering.
    extern arma::mat data;
// The number of clusters we are getting.
    extern size_t clusters;

// The assignments will be stored in this vector.
    arma::Row<size_t> assignments;
// The centroids will be stored in this matrix.
    arma::mat centroids;

// Initialize with the default arguments.
    KMeans<> k;
//    k.Cluster(data, _components, assignments, centroids);

    return false;
}

Belief HilbertMap::belief(const octomap::point3d &position)
{
    return Belief();
}
