#include <Eigen/Dense>
#include <chrono>
#include <ctime>
#include <iostream>

class KalmanFilter
{
  public:
    KalmanFilter(const Eigen::MatrixXd &A, const Eigen::MatrixXd &H, const Eigen::MatrixXd &Q)
    {
        _A = A;
        _H = H;
        _Q = Q;
    }
    ~KalmanFilter() = default;

    void initialize(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P, const double &t)
    {
        _x = x;
        _P = P;
        _t = t;
    }

    void filter(const Eigen::MatrixXd &z, const Eigen::MatrixXd &R, const double &t,
                Eigen::MatrixXd &x, Eigen::MatrixXd &P)
    {
        const double dt = t - _t;
        _A(0, 1) = dt;
        // _A(1, 3) = dt;
        const Eigen::MatrixXd xp = _A * _x;
        Eigen::MatrixXd Pp = _A * _P * _A.transpose() + _Q;
        _K = Pp * _H.transpose() * (_H * Pp * _H.transpose() + R).inverse();
        _x = xp + _K * (z - _H * xp);
        _P = Pp - _K * _H * Pp;

        x = _x;
        P = _P;
    }

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
    ~ExtendedKalmanFilter();
};

class UnscentedKalmanFilter
{
  public:
    UnscentedKalmanFilter();
    ~UnscentedKalmanFilter();
};

static constexpr double MEASUREMENT_TIME = 15.0;
static constexpr double MEASUREMENT_PERIOD = 0.1;
static constexpr double SIGNAL_AMPLITUDE = 100.0;
static constexpr double BACKGROUND_NOISE = 5.0;

int main(int argc, char **argv)
{
    std::cout << "Kalman Filter Test" << std::endl;
    std::srand(std::time(0));
    Eigen::MatrixXd A{
        {1.0, 0.0}, //
        {0.0, 1.0}, //
    };
    Eigen::MatrixXd H{{1.0, 0.0}};
    Eigen::MatrixXd Q{
        {10.0, 0.0}, //
        {0.0, 25.0}, //
    };
    KalmanFilter KF(A, H, Q);

    const auto start = std::chrono::system_clock::now();
    int num_measurements = 0;
    double previous_measurement = 0.0;
    double t = 0.0;
    double t_previous = 0.0;
    while (t < MEASUREMENT_TIME)
    {
        const auto timestamp = std::chrono::system_clock::now();
        double T = 0;
        while (T < MEASUREMENT_PERIOD)
        {
            T = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                        std::chrono::system_clock::now() - timestamp)
                                        .count()) /
                1000.0;
        }
        t = static_cast<double>(
                std::chrono::duration_cast<std::chrono::milliseconds>(timestamp - start).count()) /
            1000.0;

        const double signal_gt = SIGNAL_AMPLITUDE * t / MEASUREMENT_TIME;
        const double error = static_cast<double>(std::rand() % 1000) * BACKGROUND_NOISE / 1000.0;
        const double measurement = signal_gt + error;

        if (num_measurements == 1)
        {
            const double velocity = (measurement - previous_measurement) / (t - t_previous);
            Eigen::MatrixXd x{
                {measurement}, //
                {velocity},    //
            };
            Eigen::MatrixXd P{
                {10.0, 0.0},    //
                {0.0, 10000.0}, //
            };
            KF.initialize(x, P, t);
            std::cout << "elapsed time " << t << " measurement " << measurement << " filter " << 0.0
                      << std::endl;
        }
        else if (num_measurements > 1)
        {
            Eigen::MatrixXd z{{measurement}};
            Eigen::MatrixXd R{{10.0}};
            Eigen::MatrixXd x, P;
            KF.filter(z, R, t, x, P);
            std::cout << "elapsed time " << t << " measurement " << measurement << " filter "
                      << x(0, 0) << std::endl;
        }
        else
        {
            std::cout << "elapsed time " << t << " measurement " << measurement << " filter " << 0.0
                      << std::endl;
        }
        num_measurements += 1;
        previous_measurement = measurement;
        t_previous = t;
    }

    return 0;
}