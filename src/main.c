#include "failure.h"

int main(int argc, char **argv) {
    printf(" Quadrotor Single Motor Failure Benchmark Simulation \n");
    printf(" Based on Mueller & D'Andrea (ICRA 2014) Dynamics   \n");

    printf("Running Stock PID Controller Simulation...\n");
    printf("Motor 1 failure injected at t = %.1f s\n", FAILURE_TIME);
    run_benchmark("flight_stock.csv", CONTROLLER_STOCK);

    printf("Running Relaxed Hover Controller Simulation...\n");
    printf("Motor 1 failure injected at t = %.1f s (Yaw surrendered)\n", FAILURE_TIME);
    run_benchmark("flight_relaxed.csv", CONTROLLER_RELAXED);

    printf("Data saved to 'flight_stock.csv' and 'flight_relaxed.csv'.\n");

    return 0;
}
