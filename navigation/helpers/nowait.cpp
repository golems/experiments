#include <amino.h>
#include <ach.h>
#include <iostream>
#define NDIM 3
#include <unistd.h>

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
	const short k = 2;
	double traj[k][NDIM] = {
			{11,12,13},
			{14,15,16}
		};

	// create the ach channel
	size_t frame_size = 512;
//	size_t frame_cnt = 10;
//	enum ach_status r = ach_create( "krang_state", frame_cnt, frame_size, NULL );
//	if( ACH_OK != r ) {
//		if (ACH_EEXIST == r)
//			fprintf( stderr, "Channel already exists.\n" );
//		else {
//			fprintf( stderr, "Could not create channel: %s\n", ach_result_to_string(r) );
//			exit(EXIT_FAILURE);
//		}
//	}

	// open the channel
	enum ach_status r = ach_open( &channel, "krang_base_waypts", NULL );
	assert(ACH_OK == r);
	r = ach_flush(&channel);
	
	// test send
//	cout << "sending traj: " << endl; print_arr_2d(traj, 2);
//	r = ach_put( &channel, &traj, sizeof(traj) );
	
	// test receive
	while(true) {
		const size_t k = 170;
	 	double rtraj[k][3] = {0};
		size_t frame_size = 0;
		struct timespec abstimeout = aa_tm_future(aa_tm_sec2timespec(.001));
		ach_status_t r = ach_get(&channel, &rtraj, sizeof(rtraj), &frame_size, &abstimeout, ACH_O_LAST);
		if(r == ACH_OK) {
			cout << "received something: " << endl;
			size_t newk = frame_size / 24;
			for(size_t j = 0; j < newk; j++) {
				for(size_t i = 0; i < 3; i++) cout << rtraj[j][i] << " ";
				cout << "\n";
			}
			cout << "size: " << frame_size / 24 << endl;
		}
		if( ACH_MISSED_FRAME == r ) {
				fprintf(stdout, "Missed a/some messages(s)\n");
		} else if( ACH_STALE_FRAMES == r ) {
				fprintf( stdout, "No new data\n" );
		} else if( ACH_OK != r ) {
				fprintf(stdout, "Unable to get a message: %s\n", ach_result_to_string(r) );
		} else if( frame_size != sizeof(rtraj) ) {
				fprintf( stdout, "Unexpected message size\n");
		}
		usleep(1e5);
	}

}
