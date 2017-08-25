#pragma once

#include <valarray>
#include <octomap/OcTreeKey.h>

#include "Parameters.h"

/**
 * Class representing the results of computing the inverse sensor cause model.
 */
struct InverseCauseModel
{
    InverseCauseModel() : empty(true)
    {}

    InverseCauseModel(const InverseCauseModel &icm)
            : posteriorOnRay(icm.posteriorOnRay),
              posteriorInfinity(icm.posteriorInfinity),
              rayLength(icm.rayLength),
              empty(icm.empty)
    {
        for (auto &key : icm.ray)
            ray.push_back(octomap::OcTreeKey(key));
    }

    static const InverseCauseModel EMPTY;

    std::valarray<Parameters::NumType> posteriorOnRay;
    Parameters::NumType posteriorInfinity;
    std::vector<octomap::OcTreeKey> ray;
    unsigned int rayLength;
    bool empty;
};
