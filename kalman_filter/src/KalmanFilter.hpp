#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <Eigen/Dense>

namespace KalmanFilter
{

class KalmanFilter
{
  public:
    KalmanFilter(const Eigen::MatrixXd &A, const Eigen::MatrixXd &H, const Eigen::MatrixXd &Q);
    ~KalmanFilter() = default;

    void initialize(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P, const double &t);

    void filter(const Eigen::MatrixXd &z, const Eigen::MatrixXd &R, const double &t,
                Eigen::MatrixXd &x, Eigen::MatrixXd &P);

  private:
    double _t;
    Eigen::MatrixXd _x;
    Eigen::MatrixXd _P;
    Eigen::MatrixXd _A;
    Eigen::MatrixXd _H;
    Eigen::MatrixXd _R;
    Eigen::MatrixXd _Q;
    Eigen::MatrixXd _K;
};

class ExtendedKalmanFilter
{
  public:
    ExtendedKalmanFilter();
    ~ExtendedKalmanFilter() = default;
};

class UnscentedKalmanFilter
{
  public:
    UnscentedKalmanFilter();
    ~UnscentedKalmanFilter() = default;
};

} // namespace KalmanFilter

#endif