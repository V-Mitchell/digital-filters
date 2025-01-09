#include "src/KalmanFilter.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <math.h>
#include <random>

static constexpr double MEASUREMENT_TIME = 15.0;
static constexpr double MEASUREMENT_PERIOD = 0.01;
static constexpr double SIGNAL_AMPLITUDE = 100.0;
static constexpr double SINE_PERIOD_2 = 3.0;
static constexpr double ERROR_MEAN = 0.0;
static constexpr double ERROR_STD = 10.0;

int main(int argc, char **argv)
{
    std::cout << "Kalman Filter Test" << std::endl;
    std::srand(std::time(0));
    std::filesystem::create_directory("log");
    std::ofstream log_file;
    log_file.open("log/output.csv");

    Eigen::MatrixXd A{
        {1.0, 0.0}, //
        {0.0, 1.0}, //
    };
    Eigen::MatrixXd H{{1.0, 0.0}};
    Eigen::MatrixXd Q{
        {1.0, 0.0}, //
        {0.0, 2.5}, //
    };
    KalmanFilter::KalmanFilter KF(A, H, Q);

    std::default_random_engine generator;
    std::normal_distribution<double> dist(ERROR_MEAN, ERROR_STD);
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

        const double signal_gt = SIGNAL_AMPLITUDE * std::sin(M_PI * t / SINE_PERIOD_2);
        const double error = dist(generator);
        const double measurement = signal_gt + error;
        const double velocity = (measurement - previous_measurement) / (t - t_previous);

        if (num_measurements == 1)
        {
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
            Eigen::MatrixXd R{{500.0}};
            Eigen::MatrixXd x, P;
            KF.filter(z, R, t, x, P);
            std::cout << "elapsed time " << t << " measurement " << measurement << " filter "
                      << x(0, 0) << std::endl;
            log_file << t << "," << measurement << "," << x(0, 0) << "\n";
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

    log_file.close();
    return 0;
}