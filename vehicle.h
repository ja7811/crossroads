#ifndef __PROJECTS_PROJECT2_VEHICLE_H__
#define __PROJECTS_PROJECT2_VEHICLE_H__

#include "projects/crossroads/position.h"

#define VEHICLE_STATUS_READY 	0
#define VEHICLE_STATUS_RUNNING	1
#define VEHICLE_STATUS_FINISHED	2

char preemption_table[7][7];

struct vehicle_info {
	char id;
	char state;
	char start;
	char dest;
	struct position position;
	struct lock **map_locks;
};

void vehicle_loop(void *vi);
struct semaphore *vehicle_sema;
struct lock *step_lock;
int step_increased;

int read_count; // temp
int threads_running;
int threads_to_run;
struct semaphore *mutex; // temp
struct semaphore *rw_mutex; //temp

#endif /* __PROJECTS_PROJECT2_VEHICLE_H__ */