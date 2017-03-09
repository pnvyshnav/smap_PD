#pragma once

#include <sstream>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>
#include <valarray>
#include <ros/ros.h>

#include "Trajectory.hpp"

/**
 * Implementation of a Minimum Snap Trajectory.
 */
class MinSnapTrajectory : public Trajectory
{
public:
    /**
     * Constructs a Minimum Snap Trajectory given start and end constraints on
     * position, velocity (0 by default), and acceleration (0 by default).
     * @param xStart Start position.
     * @param xEnd End position.
     * @param xdStart Start velocity.
     * @param xdEnd End velocity.
     * @param xddStart Start acceleration.
     * @param xddEnd End acceleration.
     * @param times Number of time step polynomials.
     * @param degree Degree of each time polynomial.
     */
    MinSnapTrajectory(Point xStart, Point xEnd,
                      Point xdStart = Point(), Point xdEnd = Point(),
                      Point xddStart = Point(), Point xddEnd = Point(),
                      unsigned int times = 8, unsigned int degree = 7);

    /**
     * Copy constructor.
     * @param t Trajectory to copy from.
     */
    MinSnapTrajectory(const MinSnapTrajectory &t);

    /**
     * Creates an empty trajectory.
     */
    MinSnapTrajectory();

    // TODO supposed to be evaluated at arc-length u?!
    TrajectoryEvaluationResult evaluate(double u, bool computeTime = false)
    {
        return evaluateAtTime(u);
    }

    TrajectoryEvaluationResult evaluateAtTime(double time)
    {
        TrajectoryEvaluationResult ter;
        ter.time = time;
        ter.splineU = time;
        ter.u = time;
        ter.arcLength = time; // TODO supposed to be current arc-length
        ter.point = _position(time);
        ter.velocity = _velocity(time).norm();
        ter.acceleration = _acceleration(time).norm();
        ter.empty = false;

        double ministep = 1e-3;
        auto next = _position(time + ministep);
        auto prev = _position(time - ministep);
        double yaw = std::atan2(next.x - prev.x, next.y - prev.y);
        ter.yaw = yaw;

        return ter;
    }

    /**
     * Number of definable parameters.
     * @return Degrees of freedom.
     */
    long dof() const
    {
        return _free.cols() * DIMENSIONS;
    }

    /**
     * Parameterize the trajectory by a vector of length given by DOF.
     * @param parameter The parameter row vector.
     */
    void parameterize(const Eigen::VectorXd &parameter)
    {
        assert(parameter.size() == _free.cols() * DIMENSIONS);
//        std::cout << "Parameterizing" << std::endl << parameter.transpose() << std::endl;
        _parameters = parameter;
        _updateCoordinates();
    }

    inline unsigned int degree() const
    {
        return _degree;
    }

    Eigen::VectorXd times() const
    {
        return _times;
    }

    Point start() const
    {
        return _start;
    }

    Point end() const
    {
        return _end;
    }

    Point velocity(double time)
    {
        return _velocity(time);
    }

    double totalTime() const
    {
        return 1; // TODO make Min Snap time-dependent
    }

public: // TODO make private
    //
    // Helper classes and static methods
    //
    class Polynomial
    {
    public:
        Polynomial(unsigned int degree, const Eigen::VectorXd &coefficients)
                : _degree(degree)
        {
            _coefficients = std::valarray<double>(degree + 1);
            for (unsigned int i = 0; i <= degree; ++i)
                _coefficients[i] = coefficients[i];
        }

        Polynomial(unsigned int degree, double sameCoefficient)
                : _degree(degree)
        {
            _coefficients = std::valarray<double>(degree + 1);
            for (unsigned int i = 0; i <= degree; ++i)
                _coefficients[i] = sameCoefficient;
        }

        inline double evaluate(double x) const
        {
            double y = 0;
            for (unsigned int i = 0; i <= _degree; ++i)
            {
//                std::cout << _coefficients[_degree-i] << "*" << x << "^" << i << " + ";
                y += _coefficients[_degree-i] * std::pow(x, (double) i);
            }
//            std::cout << std::endl;
            return y;
        }

        std::valarray<double> monomials(double x) const
        {
            std::valarray<double> ms(_degree+1);
            for (unsigned int i = 0; i <= _degree; ++i)
                ms[i] = _coefficients[i] * std::pow(x, (double)i);
            return ms;
        }

        Polynomial derivative() const
        {
            Eigen::VectorXd v(_degree);
            for (unsigned int i = 1; i <= _degree; ++i)
            {
                v[i-1] = (_degree-i+1) * _coefficients[i-1];
            }
            return Polynomial(_degree-1, v);
        }

        std::valarray<double> coefficients() const
        {
            return _coefficients;
        }

        unsigned int degree() const
        {
            return _degree;
        }

        const std::string str() const
        {
            std::stringstream ss;
            for (unsigned int i = 0; i <= _degree; ++i)
            {
                ss << _coefficients[_degree-i];
                if (i > 0)
                    ss << "x";
                if (i > 1)
                    ss << "^" << i;
                if (i < _degree)
                    ss << " + ";
            }
            return ss.str();
        }

    private:
        std::valarray<double> _coefficients;
        unsigned int _degree;
    };

    class PiecewisePolynomial
    {
    public:
        PiecewisePolynomial()
        {}

        /**
         * Constructs a new piecewise polynomial given all coefficients and polynomial degree.
         * @param degree Degree of the polynomials.
         * @param times Times row vector.
         * @param coefficients Row vector of all polynomial coefficients.
         */
        PiecewisePolynomial(unsigned int degree, const Eigen::VectorXd &times, Eigen::VectorXd &coefficients)
                : _degree(degree), _times(times)
        {
            const unsigned int dd = degree + 1;
            for (unsigned int d = 0; d < coefficients.rows() / dd; ++d)
            {
                Eigen::VectorXd cs = coefficients.segment(d * dd, dd);
                _polynomials.push_back(Polynomial(degree, cs));
            }
        }

        double evaluate(double x) const
        {
            if (x < _times(0))
                return _polynomials.front().evaluate(x);
            if (x > _times(_times.rows()-1))
                return _polynomials.back().evaluate(x - _times(_times.rows()-2));
            for (unsigned int i = 1; i < _times.rows(); ++i)
            {
                if (_times(i-1) <= x && _times(i) >= x)
                    return _polynomials[i-1].evaluate(x - _times(i-1));
            }
            ROS_ERROR("Could not find the polynomial segment for x = %f", x);
            return std::nan("piecewisepoly404");
        }

        PiecewisePolynomial derivative() const
        {
            PiecewisePolynomial der;
            der._degree = _degree - 1;
            der._times = _times;
            for (auto &p : _polynomials)
            {
                der._polynomials.push_back(p.derivative());
            }
            return der;
        }

        unsigned int degree() const
        {
            return _degree;
        }

        Eigen::VectorXd times() const
        {
            return _times;
        }

    private:
        unsigned int _degree;
        Eigen::VectorXd _times;
        std::vector<Polynomial> _polynomials;
    };

    /**
     * Computes the Minimum Snap Trajectory.
     * @param x_s Constraints at start.
     * @param x_f Constraints at end.
     * @param times Times of the generated, piece-wise polynomial.
     * @param degree Degree of polynomial.
     * @return Solution matrix and its null space.
     */
    static std::pair<Eigen::MatrixXd, Eigen::MatrixXd> minimumSnap(
            Eigen::MatrixXd x_s, Eigen::MatrixXd x_f,
            Eigen::VectorXd times, unsigned int degree=4);

    static Eigen::MatrixXd timeMatrix(double time, unsigned int degree, unsigned int continuity);

    template <class MatT>
    static Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
    pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
    {
        typedef typename MatT::Scalar Scalar;
        auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
        const auto &singularValues = svd.singularValues();
        Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
        singularValuesInv.setZero();
        for (unsigned int i = 0; i < singularValues.size(); ++i)
        {
            if (singularValues(i) > tolerance)
            {
                singularValuesInv(i, i) = Scalar{1} / singularValues(i);
            }
            else
            {
                singularValuesInv(i, i) = Scalar{0};
            }
        }
        return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
    }

    inline Point _position(double t)
    {
#if (DIMENSIONS == 3)
        return Point(_positionX.evaluate(t), _positionY.evaluate(t), _positionY.evaluate(t));
#else
        return Point(_positionX.evaluate(t), _positionY.evaluate(t));
#endif
    }

    inline Point _velocity(double t)
    {
#if (DIMENSIONS == 3)
        return Point(_velocityX.evaluate(t), _velocityY.evaluate(t), _velocityY.evaluate(t));
#else
        return Point(_velocityX.evaluate(t), _velocityY.evaluate(t));
#endif
    }

    inline Point _acceleration(double t)
    {
#if (DIMENSIONS == 3)
        return Point(_accelerationX.evaluate(t), _accelerationY.evaluate(t), _accelerationY.evaluate(t));
#else
        return Point(_accelerationX.evaluate(t), _accelerationY.evaluate(t));
#endif
    }

    void _updateCoordinates();

private:
    //
    // Properties
    //
    PiecewisePolynomial _positionX, _velocityX, _accelerationX;
    PiecewisePolynomial _positionY, _velocityY, _accelerationY;
#if (DIMENSIONS == 3)
    PiecewisePolynomial _positionZ, _velocityZ, _accelerationZ;
#endif

    Eigen::MatrixXd _solution, _free; // solution and its null-space
    Eigen::VectorXd _parameters;

    unsigned int _degree;
    Eigen::VectorXd _times;

    Point _start, _end;
};

