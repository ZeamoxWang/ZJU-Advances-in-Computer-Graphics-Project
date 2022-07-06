#pragma once

#include <iostream>
#include <vector>
#include <cstdio> 
#include <ctime>
#include <map>
#include <list>
#include <omp.h>
#include <queue>
#include <set>
#include <algorithm>
#include <string>
#include <time.h>

#define TEST1 1
#define TEST2 2
#define TEST3 3
#define TEST4 4
#define TEST5 5
#define TEST6 6
#define TEST7 7
#define TEST8 8
#define TEST9 9

#define THREADS_NUM 3
#define SMALL_NODES_THREADS_NUM 6

enum SearchMode {
	Default, RANGESTATION, RANGEROAD, RANGETAXI, NNSTATION, NNROAD, NNTAXI
};

enum TreeMode {
	QUADTREE, SERIAL, PARALLEL
};
