#include <iostream>
#include <ctime>

#if 0
#include "queue.h"
int main() {
	queue_t * q = queueInit();
	const clock_t t0 = clock();
	for (int i = 0; i < 10000000; ++i) {
		queuePush(q, &i);
	}
	const clock_t t1 = clock();
	for (int i = 0; i < 10000000; ++i) {
		void * ptr = queueTryPop(q);
	}
	const clock_t t2 = clock();
	queueFini(q);
	const clock_t t5 = clock();
	std::cout << float( t1 - t0 ) / CLOCKS_PER_SEC << " secs - "
	          << float( t2 - t1 ) / CLOCKS_PER_SEC << " secs - "
	          << float( t5 - t2 ) / CLOCKS_PER_SEC << " secs" << std::endl;
}
#else
#include <queue>
int main() {
	std::queue<void *> q;
	const clock_t t0 = clock();
	for (int i = 0; i < 10000000; ++i) {
		q.push(&i);
	}
	const clock_t t1 = clock();
	for (int i = 0; i < 10000000; ++i) {
		q.pop();
	}
	const clock_t t2 = clock();
	std::cout << float( t1 - t0 ) / CLOCKS_PER_SEC << " secs - "
	          << float( t2 - t1 ) / CLOCKS_PER_SEC << " secs" << std::endl;
}
#endif
