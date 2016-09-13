#ifndef TESTY_PROT
#define TESTY_PROT

#define MAXBUF 10
#define REAL_TERM_ON 1

extern char buf[MAXBUF];
extern uint8_t tab_byte_licznik;
extern uint8_t wybor_kanalu;
extern uint8_t stream_licznik;
extern uint8_t stan;


void send_stream(void);
uint8_t tab_length(char *t);
void make_stream(char *s,uint8_t licznik_data,int16_t *d);


#endif
