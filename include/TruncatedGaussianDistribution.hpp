#pragma once

#include <cassert>
#include <limits>
#include <random>

#include <boost/random.hpp>
#include <boost/math/distributions/normal.hpp>

#include "Parameters.h"
#include "UniformDistribution.hpp"


class TruncatedGaussianDistribution
{
public:
    /**
     * Creates a new Truncated Gaussian Distribution object given
     * the distribution's parameters.
     * @param mean The mean.
     * @param std The standard deviation.
     * @param lowerBound The lower boundary.
     * @param upperBound The upper boundary.
     * @param truncated Determines whether Gaussian is truncated by lower/upper bound.
     * @return The Truncated Gaussian Distribution object.
     */
	TruncatedGaussianDistribution(
		Parameters::NumType mean,
		Parameters::NumType std,
		Parameters::NumType lowerBound,
		Parameters::NumType upperBound,
		bool truncated = Parameters::sensorTruncatedGaussianNoise) : _mean(mean), _std(std)
	{
		if (!truncated)
		{
			// we have to be able to express (negative) infinite numbers
			static_assert(std::numeric_limits<Parameters::NumType>::is_iec559, "IEEE 754 required");
			_lowerBound = -std::numeric_limits<Parameters::NumType>::infinity();
			_upperBound = std::numeric_limits<Parameters::NumType>::infinity();
		}
		else
		{
			_lowerBound = lowerBound;
			_upperBound = upperBound;
		}

		_area = boost::math::cdf(_normal, (_upperBound - _mean)/_std)
				- boost::math::cdf(_normal, (_lowerBound - _mean)/_std);
	}
		
	Parameters::NumType pdfValue(Parameters::NumType x) const
	{
		return (Parameters::NumType) ((1. / (_std * _area)) * boost::math::pdf(_normal, (x - _mean) / _std));
	}

	Parameters::NumType cdfValue(Parameters::NumType x) const
	{
		return (Parameters::NumType) ((1. / _area) * (boost::math::cdf(_normal, (x - _mean) / _std)
													  - boost::math::cdf(_normal, (_lowerBound - _mean) / _std)));
	}

	Parameters::NumType sample()
	{
		// Wikipedia says below algorithm is slow and not very accurate... see wiki for better sampling methods
		Parameters::NumType u = UniformDistribution::sample();
		Parameters::NumType cdf_tmp = boost::math::cdf(_normal, (_lowerBound - _mean)/_std);
		Parameters::NumType quantile_tmp = boost::math::quantile(_normal, _area*u + cdf_tmp);
        return _mean + _std * quantile_tmp;
	}

private:
	const Parameters::NumType _mean;
	const Parameters::NumType _std;

	Parameters::NumType _lowerBound;
	Parameters::NumType _upperBound;
	Parameters::NumType _area;

    __attribute__((weak))
	static boost::math::normal_distribution<Parameters::NumType> _normal;
};

boost::math::normal_distribution<Parameters::NumType> TruncatedGaussianDistribution::_normal = boost::math::normal_distribution<Parameters::NumType>(0, 1);
