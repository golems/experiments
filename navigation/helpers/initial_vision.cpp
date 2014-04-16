#include <amino.h>
#include <ach.h>
#include <iostream>
#define NDIM 3

using namespace std; 

ach_channel_t channel;

void print_arr_2d(double A[][NDIM], int n)
{
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < NDIM; j++) {
			printf("%2.3lf ", A[i][j]);
		}
		printf("\n");
	}
}

int main(void){

	// define a 2D trajectory container
	const size_t k = 1;
	double traj[1][3] = {0.8, 3.6, 0};

	// open the channel
	enum ach_status r = ach_open( &channel, "krang_vision", NULL );
	assert(ACH_OK == r);
	r = ach_flush(&channel);
	
	// test send
	cout << "sending traj: " << endl; 
	print_arr_2d(traj, k);
	r = ach_put( &channel, &traj, sizeof(traj) );
	cout << "traj size: " << sizeof(traj) << endl;

}
