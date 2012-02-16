#ifndef DEFS_H_
#define DEFS_H_

// CommandHandler pre-event and post-event designators
#define CMD_HNDLR_PROLOG (1)
#define CMD_HNDLR_EPILOG (0)

#define CMD_EAT_EVENT    (0)
#define CMD_PASS_EVENT   (1)

#if IBM
#define SLEEP_TIME       (5)
#else
#define SLEEP_TIME       (5000)
#endif

#define MAX_DELAY_TIME   (500000)

struct thread_data {
	int isRunning;  // is calculation finished
	int thread_id; 	// id of the thread
	int stop;       // stop thread
};

struct mp_thread_data {
	int isRunning;  // is calculation finished
	int thread_id; 	// id of the thread
	int stop;       // stop thread
};

struct mcp_thread_data {
	int isRunning;  // is calculation finished
	int thread_id; 	// id of the thread
	int stop;       // stop thread
};

struct rp_thread_data {
	int isRunning;  // is calculation finished
	int thread_id; 	// id of the thread
	int stop;       // stop thread
};

struct sp_thread_data {
	int isRunning;  // is calculation finished
	int thread_id; 	// id of the thread
	int stop;       // stop thread
};
#endif /* DEFS_H_ */
