#include "IIRFilter.hpp"

IIRFilter::IIRFilter::IIRFilter(const Eigen::MatrixXd &B, const Eigen::MatrixXd &A,
                                const Eigen::MatrixXd &X, const Eigen::MatrixXd &Y)
    : _B(B)
    , _A(A)
    , _X(X)
    , _Y(Y)
{
    if (_X.rows() == 0 || _Y.rows() == 0 || _X.cols() == 0 || _Y.cols() == 0)
    {
        throw std::invalid_argument("X and Y rows and columns must be greater than 0");
    }
    if (_B.rows() != _X.cols() || _B.cols() != _X.rows() || _A.rows() != _Y.cols() ||
        _A.cols() != _Y.rows())
    {
        throw std::invalid_argument("Rows of B must equal Columns of X and vice verse");
    }
    if (const double sum_check = _B.sum() + _A.sum();
        sum_check < 1.0 - EPSILON || sum_check > 1.0 + EPSILON)
    {
        std::stringstream ss;
        ss << "Sum of B and A must equal 1: B.sum() + A.sum() == " << sum_check;
        throw std::invalid_argument(ss.str().c_str());
    }
}

void IIRFilter::IIRFilter::reset(const Eigen::MatrixXd &X, const Eigen::MatrixXd &Y)
{
    _X = X;
    _Y = Y;
}

void IIRFilter::IIRFilter::filter(const Eigen::VectorXd &xn, Eigen::VectorXd &yn)
{
    circular_buffer(xn, _X);
    for (int i = 0; i < yn.size(); ++i)
    {
        double y = _B.row(i) * _X.col(i);
        y += _A.row(i) * _Y.col(i);
        yn(i) = y;
    }
    circular_buffer(yn, _Y);
}

void IIRFilter::IIRFilter::circular_buffer(const Eigen::VectorXd &v, Eigen::MatrixXd &M)
{
    for (int i = M.rows() - 1; i > 0; --i)
    {
        Eigen::VectorXd temp = M.row(i - 1);
        M.row(i) = temp;
    }
    M.row(0) = v;
}