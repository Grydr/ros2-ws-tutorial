#include "simple_pubsub/fib.hpp"

#include <cassert>

std::vector<int> createFibonacci(int len) { 
    std::vector<int> fibList;
    assert(len > 0);
    fibList.reserve(len);

    int a = 0;
    fibList.push_back(a);
    if (len == 1) {
        return fibList;
    }

    int b = 1;
    fibList.push_back(b);
    if (len == 2) {
        return fibList;
    }
    
    for (int i = 2; i < len; i++) {
        int c = a + b;
        a = b;
        b = c;
        fibList.push_back(c);
    }

    return fibList; 
}