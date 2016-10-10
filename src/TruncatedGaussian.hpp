#pragma once

#include <cassert>
#include <limits>
#include <random>

#include <boost/random.hpp>
#include <boost/math/distributions/normal.hpp>

#include "parameters.hpp"


class TruncatedGaussian {
public:
	TruncatedGaussian(
		Parameters::NumType mean,
		Parameters::NumType std,
		Parameters::NumType lowerBound,
		Parameters::NumType upperBound) : _mean(mean), _std(std)
	{
		if (!Parameters::sensorTruncatedGaussianNoise)
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

    	_normal = boost::math::normal_distribution<Parameters::NumType>(0, 1);
		_area = boost::math::cdf(_normal, (_upperBound - _mean)/_std) - boost::math::cdf(_normal, (_lowerBound - _mean)/_std);
	
		std::random_device rd;
	    _generator = std::mt19937(rd());
	    _uniform = std::uniform_real_distribution<Parameters::NumType>(0, 1);
	}
		
	Parameters::NumType pdfValue(Parameters::NumType x) const
	{
		return (1. / (_std * _area)) * boost::math::pdf(_normal, (x - _mean)/_std);
	}

	Parameters::NumType cdfValue(Parameters::NumType x) const
	{
		return (1. / _area) * (boost::math::cdf(_normal, (x - _mean)/_std) - boost::math::cdf(_normal, (_lowerBound - _mean)/_std));
	}

	Parameters::NumType sample()
	{
		// Wikipedia says below algorithm is slow and not very accurate... see wiki for better sampling methods
		Parameters::NumType u = _uniform(_generator);
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

	boost::math::normal_distribution<Parameters::NumType> _normal;
	std::uniform_real_distribution<Parameters::NumType> _uniform;
	std::mt19937 _generator;
};
