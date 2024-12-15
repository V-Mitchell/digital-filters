#ifndef IIR_FILTER_H_
#define IIR_FILTER_H_

#include <Eigen/Dense>
#include <stdexcept>

namespace IIRFilter
{

class IIRFilter
{
  public:
    IIRFilter(const Eigen::MatrixXd &B, const Eigen::MatrixXd &A, const Eigen::MatrixXd &X,
              const Eigen::MatrixXd &Y);
    ~IIRFilter() = default;

    void reset(const Eigen::MatrixXd &X, const Eigen::MatrixXd &Y);

    void filter(const Eigen::VectorXd &xn, Eigen::VectorXd &yn);

  private:
    static constexpr double EPSILON = 0.000001;
    void circular_buffer(const Eigen::VectorXd &v, Eigen::MatrixXd &M);

    Eigen::MatrixXd _B, _A;
    Eigen::MatrixXd _X, _Y;
};

} // namespace IIRFilter

#endif