/*--------------------------------------------------------------------
  (C) Copyright 2017-2019 Barcelona Supercomputing Center
                          Centro Nacional de Supercomputacion

  This file is part of OmpSs@FPGA toolchain.

  This code is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation; either version 3 of
  the License, or (at your option) any later version.

  OmpSs@FPGA toolchain is distributed in the hope that it will be
  useful, but WITHOUT ANY WARRANTY; without even the implied
  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this code. If not, see <www.gnu.org/licenses/>.
--------------------------------------------------------------------*/

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
