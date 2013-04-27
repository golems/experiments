
#include <somatic.h>
#include <somatic/daemon.h>
#include "krang.h"



const char *krang_mode_str( krang_mode_t mode) {
	switch(mode) {
	case KRANG_MODE_BAD: return "MODE_BAD";
	case KRANG_MODE_QUIT: return "MODE_QUIT";
	case KRANG_MODE_HALT: return "MODE_HALT";
	case KRANG_MODE_SIT_LO: return "MODE_SIT_LO";
	case KRANG_MODE_SIT_HI: return "MODE_SIT_HI";
	case KRANG_MODE_TOSIT: return "MODE_TOSIT";
	case KRANG_MODE_BALANCE_LO: return "MODE_BALANCE_LO";
	case KRANG_MODE_BALANCE_HI: return "MODE_BALANCE_HI";
	case KRANG_MODE_SIZE: return "MODE_SIZE";
	case KRANG_MODE_BLOCKED: return "MODE_BLOCKED";
	}
	return "unknown_mode";
}

const char *krang_event_str( krang_event_t ev) {
	switch(ev) {
	case KRANG_EVENT_BAD: return "EVENT_BAD";
	case KRANG_EVENT_START: return "EVENT_START";
	case KRANG_EVENT_SIT: return "EVENT_SIT";
	case KRANG_EVENT_STAND: return "EVENT_STAND";
	case KRANG_EVENT_QUIT: return "EVENT_QUIT";
	case KRANG_EVENT_STICK: return "EVENT_STICK";
	case KRANG_EVENT_SIZE: return "EVENT_SIZE";
	case KRANG_EVENT_THRESH_SIT_UP: return "EVENT_THRESH_SIT_UP";
	case KRANG_EVENT_THRESH_SIT_DOWN: return "EVENT_THRESH_SIT_DOWN";
	case KRANG_EVENT_THRESH_TORSO_LO: return "EVENT_THRESH_TORSO_LO";
	case KRANG_EVENT_THRESH_TORSO_HI: return "EVENT_THRESH_TORSO_HI";
	}
	return "unknown";
}

double rft_buffer[3*5];

void krang_init( krang_cx_t *cx ) {
	// ------ daemon init -----------
	somatic_d_opts_t dopt;
	memset(cx, 0, sizeof(*cx)); // zero initialize
	memset(&dopt, 0, sizeof(dopt)); // zero initialize
	dopt.ident = "bal-hack";
	somatic_d_init( &cx->d_cx, &dopt );

	// - Init FT sensor buffers
	aa_fzero(rft_buffer,15);

	// --------- open channels ----------
	somatic_d_channel_open( &cx->d_cx,
							&cx->state_chan, "krang-state",
							NULL );
	somatic_d_channel_open( &cx->d_cx,
							&cx->X.arm[1].ft_chan, "rlwa_ft",
							NULL );
	somatic_d_channel_open( &cx->d_cx,
							&cx->X.arm[0].ft_chan, "llwa_ft",
							NULL );
	somatic_d_channel_open( &cx->d_cx,
							&cx->spnav_chan, "spacenav-data",
							NULL );


	// --------- init parse table ----------
	// set initial mode
	krang_parse_init(cx->parse_table);

	cx->X.mode = KRANG_MODE_HALT;

	// --------- init arm controller ----------------
	for( size_t i = 0; i < 2; i ++ ) {
		rfx_ctrl_ws_init( &cx->X.arm[i].G, 7 );
		rfx_ctrl_ws_lin_k_init( &cx->X.arm[i].K, 7 );
		if(KRANG_I_RIGHT==i){
			aa_fset( cx->X.arm[i].K.f, 0.0, 3 );
			aa_fset( cx->X.arm[i].K.f+3, 0.0, 3 );
			cx->X.arm[i].ftweight = 22.54;
			cx->X.arm[i].ftcm = 0.05035;
		} else {
			aa_fset( cx->X.arm[i].K.f, 0, 3 );
			aa_fset( cx->X.arm[i].K.f+3, 0, 3 );
			cx->X.arm[i].ftweight = 18;
			cx->X.arm[i].ftcm = 0.05035;
		}			
		aa_fset( cx->X.arm[i].K.q, .3, 7 );
		aa_fset( cx->X.arm[i].K.p, 1.0, 3 );
		aa_fset( cx->X.arm[i].K.p+3, 1.0, 3 );
		cx->X.arm[i].K.dls = .005;
		// FIXME: better limits
		aa_fset( cx->X.arm[i].G.q_min, -M_PI, 7 );
		aa_fset( cx->X.arm[i].G.q_max, M_PI, 7 );
	cx->X.arm[i].G.q_min[6] = -2*M_PI; // gripper rotate module has free-rotate
	cx->X.arm[i].G.q_max[6] = 2*M_PI;
		aa_fset( cx->X.arm[i].G.x_min, -4, 3 );
		aa_fset( cx->X.arm[i].G.x_max, 4, 3 );

		double temp_ident[9] = {1,0,0, 0,1,0, 0,0,1};
		aa_fcpy(cx->X.arm[i].R0, temp_ident, 9);
	}
}

void krang_parse_init
(krang_parse_table tab ) {
	// mark all transitions invalid
	for(size_t mode = 0; mode < KRANG_MODE_SIZE; mode++ )
		for(size_t event = 0; event < KRANG_EVENT_SIZE; event++ )
			tab[mode][event] = KRANG_MODE_BAD;


	// This table also generates krang.dot, to make a visual graph from the DFA.
	// Look at krangdfa.png to verify

	// mark valid transitions
	// FORMAT
	// production A := bC  stored as  table[A][b] = C

	// halt
	tab[KRANG_MODE_HALT][KRANG_EVENT_START] = KRANG_MODE_SIT_LO;
	tab[KRANG_MODE_HALT][KRANG_EVENT_QUIT] = KRANG_MODE_QUIT;
	tab[KRANG_MODE_HALT][KRANG_EVENT_THRESH_SIT_UP] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_HALT][KRANG_EVENT_THRESH_SIT_DOWN] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_HALT][KRANG_EVENT_THRESH_TORSO_LO] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_HALT][KRANG_EVENT_THRESH_TORSO_HI] = KRANG_MODE_BLOCKED;

	// sit lo
	tab[KRANG_MODE_SIT_LO][KRANG_EVENT_START] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_SIT_LO][KRANG_EVENT_SIT] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_SIT_LO][KRANG_EVENT_STAND] = KRANG_MODE_BALANCE_LO;
	tab[KRANG_MODE_SIT_LO][KRANG_EVENT_QUIT] = KRANG_MODE_QUIT;
	tab[KRANG_MODE_SIT_LO][KRANG_EVENT_THRESH_SIT_UP] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_SIT_LO][KRANG_EVENT_THRESH_SIT_DOWN] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_SIT_LO][KRANG_EVENT_THRESH_TORSO_LO] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_SIT_LO][KRANG_EVENT_THRESH_TORSO_HI] = KRANG_MODE_SIT_HI;

	// sit hi
	tab[KRANG_MODE_SIT_HI][KRANG_EVENT_START] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_SIT_HI][KRANG_EVENT_SIT] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_SIT_HI][KRANG_EVENT_STAND] = KRANG_MODE_BALANCE_LO; // !bad
	tab[KRANG_MODE_SIT_HI][KRANG_EVENT_QUIT] = KRANG_MODE_QUIT;
	tab[KRANG_MODE_SIT_HI][KRANG_EVENT_THRESH_SIT_UP] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_SIT_HI][KRANG_EVENT_THRESH_SIT_DOWN] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_SIT_HI][KRANG_EVENT_THRESH_TORSO_LO] = KRANG_MODE_SIT_LO;
	tab[KRANG_MODE_SIT_HI][KRANG_EVENT_THRESH_TORSO_HI] = KRANG_MODE_BLOCKED;

	// to sit
	tab[KRANG_MODE_TOSIT][KRANG_EVENT_START] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_TOSIT][KRANG_EVENT_SIT] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_TOSIT][KRANG_EVENT_QUIT] = KRANG_MODE_QUIT;
	tab[KRANG_MODE_TOSIT][KRANG_EVENT_THRESH_SIT_DOWN] = KRANG_MODE_SIT_LO;
	tab[KRANG_MODE_TOSIT][KRANG_EVENT_THRESH_SIT_UP] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_TOSIT][KRANG_EVENT_THRESH_TORSO_LO] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_TOSIT][KRANG_EVENT_THRESH_TORSO_HI] = KRANG_MODE_BLOCKED;

	// balance lo
	tab[KRANG_MODE_BALANCE_LO][KRANG_EVENT_START] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_BALANCE_LO][KRANG_EVENT_STAND] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_BALANCE_LO][KRANG_EVENT_SIT] = KRANG_MODE_TOSIT;
	tab[KRANG_MODE_BALANCE_LO][KRANG_EVENT_QUIT] = KRANG_MODE_QUIT;
	tab[KRANG_MODE_BALANCE_LO][KRANG_EVENT_THRESH_SIT_UP] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_BALANCE_LO][KRANG_EVENT_THRESH_SIT_DOWN] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_BALANCE_LO][KRANG_EVENT_THRESH_TORSO_LO] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_BALANCE_LO][KRANG_EVENT_THRESH_TORSO_HI] = KRANG_MODE_BALANCE_HI;


	// balance hi
	tab[KRANG_MODE_BALANCE_HI][KRANG_EVENT_START] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_BALANCE_HI][KRANG_EVENT_STAND] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_BALANCE_HI][KRANG_EVENT_SIT] = KRANG_MODE_BAD; //! no good
	tab[KRANG_MODE_BALANCE_HI][KRANG_EVENT_QUIT] = KRANG_MODE_QUIT;
	tab[KRANG_MODE_BALANCE_HI][KRANG_EVENT_THRESH_SIT_UP] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_BALANCE_HI][KRANG_EVENT_THRESH_SIT_DOWN] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_BALANCE_HI][KRANG_EVENT_THRESH_TORSO_LO] = KRANG_MODE_BALANCE_LO;
	tab[KRANG_MODE_BALANCE_HI][KRANG_EVENT_THRESH_TORSO_HI] = KRANG_MODE_BLOCKED;

	//quit
	tab[KRANG_MODE_QUIT][KRANG_EVENT_THRESH_SIT_UP] = KRANG_MODE_BLOCKED;
	tab[KRANG_MODE_QUIT][KRANG_EVENT_THRESH_SIT_DOWN] = KRANG_MODE_BLOCKED;
}


void krang_parse_event( krang_cx_t *cx, krang_event_t ev ) {
	// FIXME: validate cx, ev
	if( SOMATIC_D_CHECK_PARM( &cx->d_cx, (ev > 0) && (ev < KRANG_EVENT_SIZE) ) &&
		SOMATIC_D_CHECK_PARM( &cx->d_cx,
							  cx && KRANG_MODE_BAD < cx->X.mode &&
							  KRANG_MODE_SIZE > cx->X.mode )
		) {

		krang_mode_t newmode = cx->parse_table[cx->X.mode][ev];

		if( somatic_d_check( &cx->d_cx, SOMATIC__EVENT__PRIORITIES__ERR,
							 SOMATIC__EVENT__CODES__LOGIC,
							 KRANG_MODE_BAD != newmode,
							 "parse",
							 "invalid event transition, <%s> := [%s]<%s>",
							 krang_mode_str(cx->X.mode),
							 krang_event_str(ev),
							 krang_mode_str(newmode) ) &&
			cx->X.mode != newmode &&
			KRANG_MODE_BLOCKED != newmode)
		{
			printf("parse <%s> := [%s]<%s>\n",
				   krang_mode_str(cx->X.mode),
				   krang_event_str(ev),
				   krang_mode_str(newmode) );
			cx->X.mode = newmode;
		}
	}
}


void krang_destroy( krang_cx_t *cx ) {
	somatic_d_channel_close( &cx->d_cx,
							 &cx->state_chan);
}

void krang_send_state( krang_cx_t *cx) {
	Somatic__Event msg;
	Somatic__Vector vec;
	double data[] = {
		cx->X.q1_0,
		cx->X.dq1_0,
		cx->X.q1_1,
		cx->X.dq1_1,
		cx->X.q2,
		cx->X.dq2,
		cx->X.q3,
		cx->X.dq3,
		cx->X.q1_ref[0],
		cx->X.q1_ref[1],
		cx->X.dq1_ref[0],
		cx->X.dq1_ref[1]
	};
	somatic__event__init(&msg);
	somatic__vector__init(&vec);
	msg.attr = &vec;
	msg.attr->data = data;
	msg.attr->n_data = sizeof(data)/sizeof(double);

	ach_status_t r =
		(ach_status_t)SOMATIC_PACK_SEND( &cx->state_chan,
										 somatic__event, &msg );
	somatic_d_check(&cx->d_cx, SOMATIC__EVENT__PRIORITIES__ERR,
					SOMATIC__EVENT__CODES__COMM_FAILED_TRANSPORT,
					ACH_OK == r, "krang_send_state",
					"sending state: %s\n",
					ach_result_to_string(r));

}


/* ********************************************************************************************** */
void krang_poll( krang_cx_t *cx ) {
	int r;
	// left FT --Sam making horrible guesses
	{
		Somatic__ForceMoment *msg =
			SOMATIC_GET_LAST_UNPACK( r, somatic__force_moment,
									 &cx->d_cx.pballoc, 4096,
									 &cx->X.arm[0].ft_chan);
		if( msg &&
			somatic_d_check_msg(&cx->d_cx,
								msg->force->data && 3 == msg->force->n_data &&
								msg->moment->data && 3 == msg->moment->n_data ,
								"force_moment", "bad data") ) {


			//double rot[4];
			aa_fcpy( cx->X.arm[0].ft, msg->force->data, 3 );
			aa_fcpy( cx->X.arm[0].ft+3, msg->moment->data, 3 );
		}
	}
	// right FT
	{
		Somatic__ForceMoment *msg =
			SOMATIC_GET_LAST_UNPACK( r, somatic__force_moment,
									 &cx->d_cx.pballoc, 4096,
									 &cx->X.arm[1].ft_chan);
		if( msg &&
			somatic_d_check_msg(&cx->d_cx,
								msg->force->data && 3 == msg->force->n_data &&
								msg->moment->data && 3 == msg->moment->n_data ,
								"force_moment", "bad data") ) {
			//double rot[4];

			// Apply a kalman filter to the FT sensor data here.
			// TODO not implemented yet

			// Update FT buffer, so we can ignore outliers
			for (int i=0; i<4; ++i) {
				rft_buffer[3*i] = rft_buffer[3*i+3];
				rft_buffer[3*i+1] = rft_buffer[3*i+4];
				rft_buffer[3*i+2] = rft_buffer[3*i+5];
			}
			// Add most recent to end of buffer
			rft_buffer[12] = msg->force->data[0];
			rft_buffer[13] = msg->force->data[1];
			rft_buffer[14] = msg->force->data[2];

			//aa_dump_vec(stdout,rft_buffer,15);

			// If most recent x is min, don't update
			if ( msg->force->data[0] < rft_buffer[0] && 
				 msg->force->data[0] < rft_buffer[3] && 
				 msg->force->data[0] < rft_buffer[6] &&
				 msg->force->data[0] < rft_buffer[9])  {
				//printf("FT min %g skipping...",msg->force->data[0]);
			}
			// or if most recent x is max, dont update
			else if ( msg->force->data[0] > rft_buffer[0] && 
					  msg->force->data[0] > rft_buffer[3] && 
					  msg->force->data[0] > rft_buffer[6] && 
					  msg->force->data[0] < rft_buffer[9])  {
				//printf("FT max %g skipping...",msg->force->data[0]);
			}
			else {
				aa_fcpy( cx->X.arm[1].ft, msg->force->data, 3 );
				aa_fcpy( cx->X.arm[1].ft+3, msg->moment->data, 3 );
			}
			
		}
	}
	// spacenav
	{
		Somatic__Joystick *msg =
			SOMATIC_GET_LAST_UNPACK( r, somatic__joystick,
									 &cx->d_cx.pballoc, 4096,
									 &cx->spnav_chan);
		if( msg &&
			somatic_d_check_msg(&cx->d_cx,
								msg->axes && msg->axes->data &&
								6 == msg->axes->n_data,
								"spnav", "bad data") ) {
			aa_fcpy( cx->spnav, msg->axes->data, 6 );
		}
	}
}

