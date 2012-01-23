#ifndef RP_CONTROLLER_H_
#define RP_CONTROLLER_H_

#include <pthread.h>

struct thread_data {
	int isRunning;  // is calculation finished
	int thread_id; 	// id of the thread
	int stop;       // stop thread
};

void *run(void *ptr_shared_data);

#endif /* RP_CONTROLLER_H_ */
