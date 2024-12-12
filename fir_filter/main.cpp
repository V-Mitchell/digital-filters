#include "src/FIRFilter.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <math.h>
#include <random>

static constexpr double MEASUREMENT_TIME = 15.0;
static constexpr double MEASUREMENT_PERIOD = 0.1;
static constexpr double SIGNAL_AMPLITUDE = 100.0;
static constexpr double SINE_PERIOD_2 = 3.0;
static constexpr double ERROR_MEAN = 0.0;
static constexpr double ERROR_STD = 10.0;

int main(int argc, char **argv)
{
    std::cout << "Finite Impulse Response Filter Test" << std::endl;
    std::srand(std::time(0));
    std::filesystem::create_directory("log");
    std::ofstream log_file;
    log_file.open("log/output.csv");

    // 1x5
    Eigen::MatrixXd B{
        {0.2, 0.2, 0.2, 0.2, 0.2},
    };
    // 5x1
    Eigen::MatrixXd X{
        {0.0}, //
        {0.0}, //
        {0.0}, //
        {0.0}, //
        {0.0}, //
    };
    Eigen::VectorXd Y(X.cols());
    FIRFilter::FIRFilter FIR(5, B, X);

    std::default_random_engine generator;
    std::normal_distribution<double> dist(ERROR_MEAN, ERROR_STD);
    const auto start = std::chrono::system_clock::now();
    double t = 0.0;
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

        Eigen::VectorXd x0{{measurement}};
        FIR.filter(x0, Y);

        std::cout << "elapsed time " << t << " measurement " << measurement << " filter " << Y(0)
                  << std::endl;
        log_file << t << "," << measurement << "," << Y(0) << "\n";
    }

    log_file.close();
    return 0;
}