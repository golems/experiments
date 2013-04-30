
#include <somatic.h>
#include <somatic/daemon.h>
#include "krang.h"



const char *krang_mode_str( krang_mode_t mode) {
	switch(mode) {
	case KRANG_MODE_BAD: return "MODE_BAD";
	case KRANG_MODE_QUIT: return "MODE_QUIT";
	case KRANG_MODE_HALT: return "MODE_HALT";
/*	case KRANG_MODE_SIT_LO: return "MODE_SIT_LO";
	case KRANG_MODE_SIT_HI: return "MODE_SIT_HI";
	case KRANG_MODE_TOSIT: return "MODE_TOSIT";
	case KRANG_MODE_BALANCE_LO: return "MODE_BALANCE_LO";
	case KRANG_MODE_BALANCE_HI: return "MODE_BALANCE_HI";
	case KRANG_MODE_SIZE: return "MODE_SIZE";
	case KRANG_MODE_BLOCKED: return "MODE_BLOCKED";*/
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

void krang_init( krang_cx_t *cx ) {
}

void krang_parse_init
(krang_parse_table tab ) {
	// mark all transitions invalid
	for(size_t mode = 0; mode < KRANG_MODE_SIZE; mode++ )
		for(size_t event = 0; event < KRANG_EVENT_SIZE; event++ )
			tab[mode][event] = KRANG_MODE_BAD;

/*
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
*/
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
