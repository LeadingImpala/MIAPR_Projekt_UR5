#include <iostream>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <cmath>
#include <unistd.h>
#include <sys/wait.h>

/////////////////////////////////////
// RUN WITH:  g++ test.cpp -o test_monitor -std=c++17 -lm
/////////////////////////////////////

void start_monitor() {
    std::cout << "Starting system load monitor...\n";

    pid_t pid = fork();
    if (pid == 0) {
        // Child process: uruchamiamy skrypt Pythona
        execlp("python3", "python3", "system_load.py", "start", (char *)nullptr);
        perror("Failed to exec Python script");
        exit(1);
    } else if (pid > 0) {
        // Parent process
        std::this_thread::sleep_for(std::chrono::seconds(2)); // daj Pythonowi czas na wystartowanie
    } else {
        std::cerr << "Fork failed!" << std::endl;
        exit(1);
    }
}

void stop_monitor() {
    std::cout << "Stopping system load monitor...\n";
    int result = system("python3 system_load.py stop");
    if (result != 0) {
        std::cerr << "Failed to stop monitor\n";
        exit(1);
    }
}

int main() {
    for(int i=0; i<5; i++){
        start_monitor();
        std::this_thread::sleep_for(std::chrono::seconds(2));

        stop_monitor();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
