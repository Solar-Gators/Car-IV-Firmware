#ifndef THREADS_H_
#define THREADS_H_

#include "user.hpp"

void ThreadsStart();

/* Defines */

/* Periodic threads */
void readBrakes(void);
void readThrottle(void);


#endif  /* THREADS_H_ */