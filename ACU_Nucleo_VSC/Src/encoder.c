#include "encoder.h"
#include "global_variables.h"

int polA_cont_up = 0;
int polA_cont_down = 0;
int polB_cont_down = 0;
int polB_cont_up = 0;
int cp;
double resolution = 0.000005;
double mult_fact = 5.238726792;
double mult_fact2 = 5.170416667;
double measurment_per = 0.4;
double enc_speed = 0;
double wheel_speed = 0;
double wheel_speed2 = 0;

int sig_a = RESET;
int sig_b = RESET;