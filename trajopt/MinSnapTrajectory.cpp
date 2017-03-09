
#include "MinSnapTrajectory.h"

std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
MinSnapTrajectory::minimumSnap(Eigen::MatrixXd x_s, Eigen::MatrixXd x_f, Eigen::VectorXd times,
                               unsigned int degree)
{
    const unsigned int dimensions = x_s.rows();
    const unsigned int cperdim = x_s.cols();
    const unsigned int continuity = cperdim; // or number of columns in a constraint

    const unsigned int dd = degree + 1;
    const unsigned int cc = continuity + 1;

    const unsigned int n_columns = (times.rows() - 2) * cc + cperdim + cperdim;
    const unsigned int n_rows = (times.rows() - 1) * dd;

    Eigen::MatrixXd constraints = Eigen::MatrixXd::Zero(dimensions, n_columns);
    constraints.leftCols(cperdim) = x_s;
    constraints.rightCols(cperdim) = x_f;
//    ROS_INFO("Constraints: %dx%d", (int)constraints.rows(), (int)constraints.cols());

    const Eigen::MatrixXd zero_time_matrix = -timeMatrix(0, degree, continuity);
//    std::cout << "Zero Time Matrix: " << std::endl << zero_time_matrix << std::endl;

    Eigen::MatrixXd polynomials = Eigen::MatrixXd::Zero(n_rows, n_columns);
//    ROS_INFO("Polynomials: %dx%d", (int)polynomials.rows(), (int)polynomials.cols());
    polynomials.topLeftCorner(dd, cperdim) << timeMatrix(times[0], degree, cperdim - 1);
    for (unsigned int i = 2; i < times.rows(); ++i)
    {
        unsigned int col = (i - 2) * cc + cperdim;
        unsigned int row = (i - 2) * dd;
        polynomials.block(row, col, dd, cc) = timeMatrix(times[i] - times[i-1], degree, continuity);
        polynomials.block(row + dd, col, dd, cc) = zero_time_matrix;
    }
    polynomials.bottomRightCorner(dd, cperdim) = timeMatrix(times(times.rows()-1, 0) - times(times.rows()-2, 0),
                                                            degree, cperdim - 1);
    Eigen::MatrixXd solution = (constraints * pseudoinverse(polynomials));
    Eigen::FullPivLU<Eigen::MatrixXd> lu(polynomials.transpose());
    Eigen::MatrixXd free = lu.kernel();
    ROS_INFO("Solution: %dx%d", (int)solution.rows(), (int)solution.cols());
    ROS_INFO("Null space: %dx%d", (int)free.rows(), (int)free.cols());

    return std::make_pair(std::ref(solution), std::ref(free));
}

Eigen::MatrixXd MinSnapTrajectory::timeMatrix(double time, unsigned int degree, unsigned int continuity)
{
    Eigen::MatrixXd time_matrix = Eigen::MatrixXd::Zero(degree+1, continuity+1);
    auto poly = Polynomial(degree, 1.);
    for (unsigned int derivation = 0; derivation <= continuity; ++derivation)
    {
        auto coeffs = poly.coefficients();
        for (int d = poly.degree(); d >= 0; --d)
        {
            time_matrix(poly.degree() - d, derivation) = coeffs[poly.degree()-d] * std::pow(time, (double)d);
        }
//            time_matrix[:int(len(poly)+1), derivation] = [c*time**d for c, d in zip(poly, range(len(poly),-1,-1))]
        // print "p%s(%i) = %s\n%s" % ("'" * derivation, t, str([c*t**d for c, d in zip(poly, range(len(poly),-1,-1))] + [0]*(continuity-len(poly)-1)), poly)
        poly = poly.derivative();
//            std::cout << "Poly derivative degree: " << poly.degree() << std::endl;
    }

    return time_matrix;
}

MinSnapTrajectory::MinSnapTrajectory(Point xStart, Point xEnd, Point xdStart, Point xdEnd, Point xddStart, Point xddEnd,
                                     unsigned int times, unsigned int degree)
: _degree(degree), _start(xStart), _end(xEnd), Trajectory(false)
{
    Eigen::Matrix<double, DIMENSIONS, 3> x_s, x_f;
    x_s <<  xStart.x, xdStart.x, xddStart.x,
            xStart.y, xdStart.y, xddStart.y
#if (DIMENSIONS == 3)
            ,xStart.z, xdStart.z, xddStart.z
#endif
            ;
    x_f <<  xEnd.x, xdEnd.x, xddEnd.x,
            xEnd.y, xdEnd.y, xddEnd.y
#if (DIMENSIONS == 3)
            ,xEnd.z, xdEnd.z, xddEnd.z
#endif
            ;

//    std::cout << "x_s:" << std::endl << x_s << std::endl;

    _times = Eigen::VectorXd::LinSpaced(times, 0, 1.).transpose();
//    ROS_INFO("times: %dx%d", (int)_times.rows(), (int)_times.cols());
    std::cout << _times << std::endl;

    auto trajectory = MinSnapTrajectory::minimumSnap(x_s, x_f, _times, degree);
    _solution = trajectory.first;
    _free = trajectory.second;
    _parameters = Eigen::VectorXd::Zero(_free.cols() * DIMENSIONS);

    _updateCoordinates();
}

MinSnapTrajectory::MinSnapTrajectory() : Trajectory(true)
{}

void MinSnapTrajectory::_updateCoordinates()
{
    Eigen::Map<Eigen::MatrixXd> params(_parameters.data(), _free.cols(), DIMENSIONS);
    Eigen::MatrixXd fp =  _free * params;
    Eigen::MatrixXd rows = _solution + fp.transpose();
//    std::cout << "X row: " << std::endl << row << std::endl;
    Eigen::VectorXd row = rows.row(0);
    _positionX = PiecewisePolynomial(_degree, _times, row);
    _velocityX = _positionX.derivative();
    _accelerationX = _velocityX.derivative();
    fp =  _free * _parameters.segment(_free.cols(), _free.cols());
//    row = _solution.row(1).transpose() + fp;
    row = rows.row(1);
    _positionY = PiecewisePolynomial(_degree, _times, row);
    _velocityY = _positionY.derivative();
    _accelerationY = _velocityY.derivative();
#if (DIMENSIONS == 3)
    fp =  _free * _parameters.segment(2 * _free.cols(), _free.cols());
    row = _solution.row(2).transpose() + fp;
    _positionZ = PiecewisePolynomial(_degree, _times, row);
    _velocityZ = _positionZ.derivative();
    _accelerationZ = _velocityZ.derivative();
#endif
}

MinSnapTrajectory::MinSnapTrajectory(const MinSnapTrajectory &t)
        : _degree(t._degree), _start(t._start), _end(t._end),
          _times(t._times), _solution(t._solution),
          _free(t._free), _parameters(t._parameters),
          Trajectory(false)
{
    _updateCoordinates();
}