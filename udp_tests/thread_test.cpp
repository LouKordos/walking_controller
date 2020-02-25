#include <iostream>
#include<functional>

#include <chrono>
#include <thread>
#include <mutex>

#include <fstream>

#include <string>
#include <random>
#include <ctime>
#include <cmath>

#include <stdlib.h>

std::thread test;

void test_2() {

}

int main() {
    test = std::thread(std::bind(test_2));
}