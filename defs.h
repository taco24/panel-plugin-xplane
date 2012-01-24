#ifndef DEFS_H_
#define DEFS_H_

struct thread_data {
	int isRunning;  // is calculation finished
	int thread_id; 	// id of the thread
	int stop;       // stop thread
};

#endif /* DEFS_H_ */
