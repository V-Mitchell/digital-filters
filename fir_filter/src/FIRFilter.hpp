#ifndef FIR_FILTER_H_
#define FIR_FILTER_H_

#include <Eigen/Dense>
#include <stdexcept>

namespace FIRFilter
{

class FIRFilter
{
  public:
    FIRFilter(const Eigen::MatrixXd &B, const Eigen::MatrixXd &X);
    ~FIRFilter() = default;

    void reset(const Eigen::MatrixXd &X);

    void filter(const Eigen::VectorXd &xn, Eigen::VectorXd &yn);

  private:
    static constexpr double EPSILON = 0.000001;
    void circular_buffer(const Eigen::VectorXd &xn);

    Eigen::MatrixXd _B;
    Eigen::MatrixXd _X;
};

} // namespace FIRFilter

#endif