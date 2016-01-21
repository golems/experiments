/*
 * Test for TrajGenMultiDim
 */

#include "TrajGen.h"

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
    TrajGen trajGen;

    
    float x_ref, v_ref;
    trajGen.getReferenceState(1, x_ref, v_ref);

    printf("x_Ref: %f v_ref: %f\n", x_ref, v_ref);

    trajGen.getReferenceState(0, x_ref, v_ref);
    printf("x_Ref: %f v_ref: %f\n", x_ref, v_ref);


    float v_i = 1;
    trajGen.initializeTrajectory(v_i, 17.75, 9.5);

    for(int t = 0; t<=10; t++){
       trajGen.getReferenceState(t, x_ref, v_ref);
       printf("t: %d x_Ref: %f v_ref: %f\n", t, x_ref, v_ref);
    }

}

int main() {

    //Vector6d state;
    //Vector6d refState;

    //trajGen.setTargetState(&state, &refState);

    test1();
    
    cout << "SUCCESS: All Test Passed.\n";
}