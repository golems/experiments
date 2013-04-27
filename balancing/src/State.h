#include "krang-io.h"

void krang_init( krang_cx_t *cx );
void krang_poll( krang_cx_t *cx );
void krang_parse_init( krang_parse_table tab );
void krang_parse_event( krang_cx_t *cx, krang_event_t ev );
void krang_destroy( krang_cx_t *cx );
void krang_send_state( krang_cx_t *X);
void krang_threshold( krang_cx_t *X);


