
#include <stdio.h>

#include "threads/thread.h"
#include "threads/synch.h"
#include "threads/interrupt.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"

const char PREEMPT_INIT = 127;
const char DEBUG = 0;

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
	mutex = (struct semaphore *) malloc(sizeof(struct semaphore));
	sema_init(vehicle_sema, thread_cnt);
	sema_init(mutex, 1);

	mutex_lock = (struct lock *) malloc(sizeof(struct lock));
	lock_init(mutex_lock);

	threads_running = thread_cnt;
	threads_to_run = thread_cnt;

	step_completed = 0;
	cond = (struct condition *) malloc(sizeof(struct condition));
	cond_init(cond);

	finished_thread_cnt = 0;
	TOTAL_THREADS = thread_cnt;
	blocked_threads = (struct list *) malloc(sizeof(struct list));
	list_init(blocked_threads);
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
	lock_acquire(mutex_lock);
	threads_to_run--;
	/* increase step if all vehicles have moved */
	if(threads_to_run == 0){
		threads_to_run = threads_running;
		crossroads_step++;
	}
	lock_release(mutex_lock);
}

void block_thread(){
    enum intr_level old_level;
    old_level = intr_disable ();
    list_push_back(blocked_threads, &thread_current () -> elem);

    thread_block ();
    intr_set_level (old_level);
}

void unblock_threads(){
   enum intr_level old_level;
   old_level = intr_disable();
   while(!list_empty(blocked_threads)){
    struct thread* popped_thread = list_entry( list_pop_front(blocked_threads), struct thread, elem);
    thread_unblock(popped_thread);
   }

   intr_set_level(old_level);
}

void sync_vehicles(){
	lock_acquire(mutex_lock);
	step_completed++;
	lock_release(mutex_lock);
	if(step_completed < TOTAL_THREADS){
		// Some other threads remain incomplete
		block_thread();
	} else {
		// all threads have completed
		lock_acquire(mutex_lock);
		step_completed = finished_thread_cnt;
		unblock_threads();
		lock_release(mutex_lock);
	}
}

void wait_for_other_vehicles(struct vehicle_info *vi){
 	/* Wait for all threads to complete the step */
	lock_acquire(mutex_lock);
	step_completed++;
	lock_release(mutex_lock);
	if(step_completed < TOTAL_THREADS){
		// Some other threads remain incomplete
		if(DEBUG) ("blocking %c (complete=%d)\n", vi->id, step_completed);
		block_thread();
	} else {
		// all threads have completed
		lock_acquire(mutex_lock);
		step_completed = finished_thread_cnt;
		crossroads_step++;
		if(DEBUG) printf("unblocking by %c (complete=%d)\n", vi->id, step_completed);
		if(DEBUG) printf("======step %d======\n", crossroads_step);
		unblock_threads();
		lock_release(mutex_lock);
	}
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
		// printf("%c new step %d\n", vi->id, step);
		sema_down(vehicle_sema);
		// handle_step_increase();
		
		/* preempt next position */
		preempt(start, dest, step, vi);
		if(DEBUG) printf("%c preempted\n", vi->id);
		/* vehicle main code */
		res = try_move(start, dest, step, vi);
		if(DEBUG) printf("%c moved\n", vi->id);
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
		unitstep_changed();
		wait_for_other_vehicles(vi);
		if(DEBUG) printf("%c preempted\n", vi->id);
		sema_up(vehicle_sema);
	}	

	/* status transition must happen before sema_up */
	// printf("%c finished\n", vi->id);
	vi->state = VEHICLE_STATUS_FINISHED;
	lock_acquire(mutex_lock);
	// if(threads_running == threads_to_run) threads_to_run--;
	// threads_running--;
	if(finished_thread_cnt == step_completed) step_completed++;
	finished_thread_cnt++;
	lock_release(mutex_lock);
}
