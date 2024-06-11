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

int CURRENT_THREADS_CNT;
int step_completed;

struct semaphore *mutex;
struct lock *mutex_lock;
struct lock *thread_count_lock;


#endif /* __PROJECTS_PROJECT2_VEHICLE_H__ */