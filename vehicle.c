
#include <stdio.h>

#include "threads/thread.h"
#include "threads/synch.h"
#include "threads/interrupt.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"

const char PREEMPT_INIT = 127;

/* path. A:0 B:1 C:2 D:3 */
const struct position vehicle_path[4][4][12] = {
	/* from A */ {
		/* to A */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{4,0},{4,1},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from B */ {
		/* to A */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{6,4},{5,4},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from C */ {
		/* to A */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{2,6},{2,5},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from D */ {
		/* to A */
		{{0,2},{1,2},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	}
};

static int is_position_outside(struct position pos)
{
	return (pos.row == -1 || pos.col == -1);
}

static int is_position_center(struct position pos)
{
	return (2 <= pos.row && pos.row <= 4) && (2 <= pos.col && pos.col <= 4);
}

/* return 0:termination, 1:success, -1:fail */
static int try_move(int start, int dest, int step, struct vehicle_info *vi)
{
	struct position pos_cur, pos_next;

	pos_next = vehicle_path[start][dest][step];
	pos_cur = vi->position;

	if (vi->state == VEHICLE_STATUS_RUNNING) {
		/* check termination */
		if (is_position_outside(pos_next)) {
			/* release current preemption */
			preempt_release(vi);
			/* actual move */
			vi->position.row = vi->position.col = -1;
			/* release previous */
			lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
			return 0;
		}
	}

	/* return if next position is preemptied by other thread */
	if (preemption_table[pos_next.row][pos_next.col] != vi->id) 
		return -1;
	/* lock next position */
	lock_acquire(&vi->map_locks[pos_next.row][pos_next.col]);
	if (vi->state == VEHICLE_STATUS_READY) {
		/* start this vehicle */
		vi->state = VEHICLE_STATUS_RUNNING;
	} else {
		/* release current preemption */
		preempt_release(vi);
		/* release current position */
		lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
	}
	/* update position */
	vi->position = pos_next;
	
	return 1;
}

void init_on_mainthread(int thread_cnt){
	/* Called once before spawning threads */
	memset(preemption_table, PREEMPT_INIT, 7 * 7 * sizeof(char));
	/* Initialize variables */
	printf("running threads: %d\n", thread_cnt);
	vehicle_sema = (struct semaphore *) malloc(sizeof(struct semaphore));
	sema_init(vehicle_sema, 1);
	step_lock = (struct lock *) malloc(sizeof(struct lock));
	lock_init(step_lock);
	step_increased = 0;
	
	read_count = 0;
	mutex = (struct semaphore *) malloc(sizeof(struct semaphore));
	sema_init(mutex, 1);
	rw_mutex = (struct semaphore *) malloc(sizeof(struct semaphore));
	sema_init(rw_mutex, 1);

	threads_running = thread_cnt;
	threads_to_run = thread_cnt;
}

/* release current preemption */
void preempt_release(struct vehicle_info *vi){
	enum intr_level old_level;
	old_level = intr_disable();

	int i = vi->position.row, j = vi->position.col;
	if(preemption_table[i][j] == vi->id){
		preemption_table[i][j] = PREEMPT_INIT;
	}

	intr_set_level(old_level);
}

void preempt(int start, int dest, int step, struct vehicle_info *vi){
	enum intr_level old_level;
	old_level = intr_disable ();

	struct position pos = vehicle_path[start][dest][step];
	while(!is_position_outside(pos)){
		char preempted_thread_id = preemption_table[pos.row][pos.col];
		/* preempt position if current vehicle's priority is higher */
		if(preempted_thread_id > vi->id){
			preemption_table[pos.row][pos.col] = vi->id;
		}
		else {
			break;
		}
		pos = vehicle_path[start][dest][++step];
	}
	

	intr_set_level (old_level);
}

void handle_step_increase(){
	sema_down(mutex);
	read_count++;
	if(read_count == 1) sema_down(rw_mutex);
	// printf("reader=%d\n", read_count);
	sema_up(mutex);

	lock_acquire(step_lock);
	if(vehicle_sema->value == 0 && !step_increased) {
		// printf("increasing step, (%d,%d) -> ", vehicle_sema->value, step_increased);
		// crossroads_step++;
	}
	// printf("increased=%d", step_increased);
	step_increased++;
	// printf("->%d\n", step_increased);
	lock_release(step_lock);

	sema_down(mutex);
	read_count--;
	if(read_count == 0) sema_up(rw_mutex);
	sema_up(mutex);
}

void set_step_increased(){
	sema_down(rw_mutex);
	step_increased = 0;
	sema_up(rw_mutex);
}

void vehicle_loop(void *_vi)
{
	int res;
	int start, dest, step;

	struct vehicle_info *vi = _vi;

	start = vi->start - 'A';
	dest = vi->dest - 'A';

	vi->position.row = vi->position.col = -1;
	vi->state = VEHICLE_STATUS_READY;
	step = 0;
	while (1) {
		sema_down(vehicle_sema);
		set_step_increased();
		/* preempt next position */
		preempt(start, dest, step, vi);

		/* vehicle main code */
		res = try_move(start, dest, step, vi);
		if (res == 1) {
			step++;
		}

		/* termination condition. */ 
		if (res == 0) {
			break;
		}

		/* preempt next position */
		preempt(start, dest, step, vi);

		/* unitstep change! */
		// printf("sema=%d\n", vehicle_sema->value);
		handle_step_increase();
		sema_up(vehicle_sema);
		unitstep_changed();
	}	

	/* status transition must happen before sema_up */
	vi->state = VEHICLE_STATUS_FINISHED;
	threads_to_run--;
	sema_up(vehicle_sema);
}
