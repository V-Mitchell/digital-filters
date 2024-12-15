#include "FIRFilter.hpp"

FIRFilter::FIRFilter::FIRFilter(const Eigen::MatrixXd &B, const Eigen::MatrixXd &X)
    : _B(B)
    , _X(X)
{
    if (_X.rows() == 0 || _X.cols() == 0)
    {
        throw std::invalid_argument("X rows and columns must be greater than 0");
    }
    if (_B.rows() != _X.cols() || _B.cols() != _X.rows())
    {
        throw std::invalid_argument("Rows of B must equal Columns of X and vice verse");
    }
    if (const double sum_check = _B.sum(); sum_check < 1.0 - EPSILON || sum_check > 1.0 + EPSILON)
    {
        std::stringstream ss;
        ss << "Sum of B must equal 1: B.sum() == " << sum_check;
        throw std::invalid_argument(ss.str().c_str());
    }
}

void FIRFilter::FIRFilter::reset(const Eigen::MatrixXd &X) { _X = X; }

void FIRFilter::FIRFilter::filter(const Eigen::VectorXd &xn, Eigen::VectorXd &yn)
{
    circular_buffer(xn);
    for (int i = 0; i < yn.size(); ++i)
    {
        yn(i) = _B.row(i) * _X.col(i);
    }
}

void FIRFilter::FIRFilter::circular_buffer(const Eigen::VectorXd &xn)
{
    for (int i = _X.rows() - 1; i > 0; --i)
    {
        Eigen::VectorXd temp = _X.row(i - 1);
        _X.row(i) = temp;
    }
    _X.row(0) = xn;
}