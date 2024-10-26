#include <iostream>
#include <chrono>
#include <ctime>
#include <Eigen/Dense>

int main(int argc, char **argv)
{
    std::cout << "Kalman Filter Test" << std::endl;
    std::srand(std::time(0));
    const auto start = std::chrono::system_clock::now();

    double elapsedS = 0;
    while (elapsedS < 5.0)
    {
        const auto timestamp = std::chrono::system_clock::now();
        elapsedS = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(timestamp - start).count()) / 1000.0;
        std::cout << "elapsed time " << elapsedS << " rand " << std::rand() % 1000 << std::endl;
    }

    return 0;
}