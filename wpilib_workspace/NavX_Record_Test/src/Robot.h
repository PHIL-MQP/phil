#include <sys/time.h>

#include <AHRS.h>

int manditory_init();
int collectSamples(size_t number_of_samples, AHRS *ahrs);

double time_to_sec(struct timeval tv);
