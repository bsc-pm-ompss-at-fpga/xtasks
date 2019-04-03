#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#include "queue.h"

#define CHECK_COND(cond) if (__sync_bool_compare_and_swap(&cond,1,1)) break;

volatile int cond = 0;

void *consumer(void *_queue){
  queue_t *queue = (queue_t *)_queue;
  static pthread_mutex_t consumer_lock = PTHREAD_MUTEX_INITIALIZER;

  int i=100;
  void *value = NULL;

  for(;;) {
	  usleep(10);
    value = queueTryPop(queue);

    if(value != NULL){
      char * c = (char *)value;
        pthread_mutex_lock(&consumer_lock);
        printf("\n %c, %u\n", *c, (unsigned int)pthread_self());
        fflush(stdout);
        pthread_mutex_unlock(&consumer_lock);
    }

    sched_yield();
    value = NULL;
    CHECK_COND(cond);
  }
}

int main(){
  int i = 0;

  queue_t *queue = queueInit();

  pthread_t _thread;
  pthread_t _thread2;

  pthread_create(&_thread,NULL,consumer,queue);
  pthread_create(&_thread2,NULL,consumer,queue);

  char *value = (char *)malloc(100 * sizeof(char));

  for(i = 0; i < 100; i++){
    value[i] = 'a' + i%20;
    queuePush(queue,&value[i]);
  }

  sleep(1);

	queuePush(queue,&value[4]);
	queuePush(queue,&value[6]);
	queuePush(queue,&value[8]);

	sleep(1);

  __sync_bool_compare_and_swap(&cond,0,1);

  pthread_join(_thread,NULL);
  pthread_join(_thread2,NULL);
  
  free(value);
  queueFini(queue);

  return 0;
}
