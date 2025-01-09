#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter::KalmanFilter(const Eigen::MatrixXd &A, const Eigen::MatrixXd &H,
                                         const Eigen::MatrixXd &Q)
{
    _A = A;
    _H = H;
    _Q = Q;
}

void KalmanFilter::KalmanFilter::initialize(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P,
                                            const double &t)
{
    _x = x;
    _P = P;
    _t = t;
}

void KalmanFilter::KalmanFilter::filter(const Eigen::MatrixXd &z, const Eigen::MatrixXd &R,
                                        const double &t, Eigen::MatrixXd &x, Eigen::MatrixXd &P)
{
    const double dt = t - _t;
    _t = t;
    _A(0, 1) = dt;
    const Eigen::MatrixXd xp = _A * _x;
    Eigen::MatrixXd Pp = _A * _P * _A.transpose() + _Q;
    _K = (Pp * _H.transpose()) * (_H * Pp * _H.transpose() + R).inverse();
    _x = xp + _K * (z - _H * xp);
    _P = Pp - _K * _H * Pp;

    x = _x;
    P = _P;
}