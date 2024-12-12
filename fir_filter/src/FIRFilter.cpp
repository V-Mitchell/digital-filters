#include "FIRFilter.hpp"

FIRFilter::FIRFilter::FIRFilter(const size_t N, const Eigen::MatrixXd &B, const Eigen::MatrixXd &X)
    : _N(N)
    , _B(B)
    , _X(X)
{
    if (_N == 0)
    {
        throw std::invalid_argument("N must be greater than 0");
    }
    if (_B.rows() != _X.cols() || _B.cols() != _X.rows())
    {
        throw std::invalid_argument("Rows of B must equal Columns of X and vice verse");
    }
}

void FIRFilter::FIRFilter::reset(const Eigen::MatrixXd &X) { _X = X; }

void FIRFilter::FIRFilter::filter(const Eigen::VectorXd &x0, Eigen::VectorXd &Y)
{
    circular_buffer(x0);
    for (int i = 0; i < Y.size(); ++i)
    {
        Y(i) = _B.row(i) * _X.col(i);
    }
}

void FIRFilter::FIRFilter::circular_buffer(const Eigen::VectorXd &x0)
{
    for (int i = _X.rows() - 1; i > 0; --i)
    {
        Eigen::VectorXd temp = _X.row(i - 1);
        _X.row(i) = temp;
    }
    _X.row(0) = x0;
}