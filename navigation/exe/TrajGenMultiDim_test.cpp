/*
 * Test for TrajGenMultiDim
 */

#include "TrajGenMultiDim.h"

#include <iostream>

using namespace std;

//TrajGen trajGen;

/* Prints a row or column vector. */
#define PRINT_VECTOR(X)                                   \
            printf("(%d x %d) ", (X).rows(), (X).cols()); \
            printf("[");                                  \
            for(int i = 0; i < (X).size(); i++){          \
                printf(" %4.4f", (X)[i]);                 \
            }                                             \
            printf("]\n\r");

void test1() {
    TrajGenMultiDim trajGen;

    Vector6d refStateTraj = trajGen.getReferenceState(1);
    PRINT_VECTOR(refStateTraj)
    refStateTraj = trajGen.getReferenceState(0);
    PRINT_VECTOR(refStateTraj)
}

int main() {

    //Vector6d state;
    //Vector6d refState;

    //trajGen.setTargetState(&state, &refState);

    test1();
    
    cout << "SUCCESS: All Test Passed.\n";
}