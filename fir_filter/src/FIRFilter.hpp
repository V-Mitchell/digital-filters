#ifndef FIR_FILTER_H_
#define FIR_FILTER_H_

#include <Eigen/Dense>
#include <stdexcept>

namespace FIRFilter
{

class FIRFilter
{
  public:
    FIRFilter(const size_t N, const Eigen::MatrixXd &B, const Eigen::MatrixXd &X);
    ~FIRFilter() = default;

    void reset(const Eigen::MatrixXd &X);

    void filter(const Eigen::VectorXd &x0, Eigen::VectorXd &Y);

  private:
    void circular_buffer(const Eigen::VectorXd &x0);

    size_t _N;
    Eigen::MatrixXd _B;
    Eigen::MatrixXd _X;
};

} // namespace FIRFilter

#endif