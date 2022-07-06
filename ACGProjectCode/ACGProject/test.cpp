#include "common.h"
#include "Geometry.h"
#include "QuadTree.h"
#include "SerialKDTree.h"
#include "ParallelKDTree.h"
#include<windows.h>
extern vector<MapGeom::Feature> features;
extern vector<MapGeom::Geometry*> readGeom(const char* filename);
extern vector<string> readName(const char* filename);

extern SearchMode mode;
extern TreeMode treeState;

extern bool showTreeNodes;
extern bool taxiTree;
extern bool roadTree;
extern bool atStation;
extern bool atTaxi;
extern bool rangeSearch;
extern bool nnSearch;

extern vector<MapGeom::Feature> roads;

extern MapGeom::QuadTree stationQTree;
extern MapGeom::QuadTree roadQTree;
extern MapGeom::QuadTree taxiQTree;

extern MapGeom::SerialKDTree stationSerialKDTree;
extern MapGeom::SerialKDTree roadSerialKDTree;
extern MapGeom::SerialKDTree taxiSerialKDTree;

extern MapGeom::ParallelKDTree stationParallelKDTree;
extern MapGeom::ParallelKDTree roadParallelKDTree;
extern MapGeom::ParallelKDTree taxiParallelKDTree;

extern void loadTaxiData();
extern void buildTaxiTree();
extern void updateMode();

using namespace MapGeom;

void transformValue(double& res, const char* format = "%.2lf") {

	char buf[20];
	sprintf(buf, format, res);
	sscanf(buf, "%lf", &res);
}

void SerialKDTreeNNWorstAnalysis()
{
	if (!taxiTree) {
		loadTaxiData();
		buildTaxiTree();
	}
	taxiTree = true;
	atTaxi = true;
	atStation = false;
	updateMode();

	LARGE_INTEGER freq_;
	QueryPerformanceFrequency(&freq_);

	LARGE_INTEGER begin_time;
	LARGE_INTEGER end_time;

	MapGeom::SerialKDTree* tree = new MapGeom::SerialKDTree(0.35);
	tree->setCts(1.31593);
	tree->buildKDTree(features);

	srand(time(NULL));
	double x, y, max = 0;
	vector<MapGeom::Feature> candidateFeatures;
	vector<MapGeom::Feature> selectedFeatures;
	for (int j = 0; j < 10; j++) {
		max = 0;
		for (int i = 0; i <= 20000; i++) {
			candidateFeatures.clear();
			x = -((rand() % 225) / 10000.0 + 73.9812);
			y = (rand() % 239) / 10000.0 + 40.7247;

			QueryPerformanceCounter(&begin_time);

			tree->NNQuery(x, y, candidateFeatures);
			// refine step
			int NNIndex = 0;
			double minDist = candidateFeatures[0].distance(x, y);
			double dist;
			for (int j = 1; j < candidateFeatures.size(); j++) {
				dist = candidateFeatures[j].distance(x, y);
				if (dist < minDist) {
					minDist = dist;
					NNIndex = j;
				}
			}
			QueryPerformanceCounter(&end_time);
			double ns_time = (end_time.QuadPart - begin_time.QuadPart) * 1000000.0 / freq_.QuadPart;
			if (ns_time > max)max = ns_time;
		}
		cout << "Serial-KDTree NN Query Worst Time: " << max << "ns" << endl;
	}

	delete tree;
}

void QuadTreeNNWorstAnalysis()
{
	if (!taxiTree) {
		loadTaxiData();
		buildTaxiTree();
	}
	taxiTree = true;
	atTaxi = true;
	atStation = false;
	updateMode();

	LARGE_INTEGER freq_;
	QueryPerformanceFrequency(&freq_);

	LARGE_INTEGER begin_time;
	LARGE_INTEGER end_time;

	MapGeom::QuadTree* tree = new MapGeom::QuadTree();
	tree->setCapacity(25);
	tree->constructQuadTree(features);

	srand(time(NULL));
	double x, y, max = 0;
	vector<MapGeom::Feature> candidateFeatures;
	vector<MapGeom::Feature> selectedFeatures;
	for (int j = 0; j < 10; j++) {
		max = 0;
		for (int i = 0; i <= 20000; i++) {
			candidateFeatures.clear();
			x = -((rand() % 225) / 10000.0 + 73.9812);
			y = (rand() % 239) / 10000.0 + 40.7247;

			QueryPerformanceCounter(&begin_time);

			tree->NNQuery(x, y, candidateFeatures);
			// refine step
			int NNIndex = 0;
			double minDist = candidateFeatures[0].distance(x, y);
			double dist;
			for (int j = 1; j < candidateFeatures.size(); j++) {
				dist = candidateFeatures[j].distance(x, y);
				if (dist < minDist) {
					minDist = dist;
					NNIndex = j;
				}
			}
			QueryPerformanceCounter(&end_time);
			double ns_time = (end_time.QuadPart - begin_time.QuadPart) * 1000000.0 / freq_.QuadPart;
			if (ns_time > max)max = ns_time;
		}
		cout << "QuadTree NN Query Worst Time: " << max << "ns" << endl;
	}

	delete tree;
}

void ParallelKDTreeRangeQueryAnalysis()
{
	if (!taxiTree) {
		loadTaxiData();
		buildTaxiTree();
	}
	taxiTree = true;
	atTaxi = true;
	atStation = false;
	updateMode();

	MapGeom::ParallelKDTree* kdTree = new MapGeom::ParallelKDTree(0.45);
	kdTree->setCts(1.53945);
	kdTree->buildKDTree(features);

	srand(time(NULL));
	double x1, y1, x2, y2;
	vector<MapGeom::Feature> candidateFeatures;
	vector<MapGeom::Feature> selectedFeatures;
	clock_t start_time = clock();
	clock_t end_time = clock();
	for (int i = 0; i <= 10000; i++) {
		candidateFeatures.clear();
		x1 = -((rand() % 225) / 10000.0 + 73.9812);
		y1 = (rand() % 239) / 10000.0 + 40.7247;
		x2 = -((rand() % 225) / 10000.0 + 73.9812);
		y2 = (rand() % 239) / 10000.0 + 40.7247;
		if (x1 > x2) {
			swap(x1, x2);
		}
		if (y1 > y2) {
			swap(y1, y2);
		}
		Envelope selectedRect(x1, x2, y1, y2);
		kdTree->rangeQuery(selectedRect, candidateFeatures);

		// refine step;
		selectedFeatures.swap(vector<MapGeom::Feature>());
		int total = candidateFeatures.size();
		// use reserve function to get avoid the growth of vector length when doing parallel insertion;
		selectedFeatures.reserve(total);

		// use parallel algorithm to accelarate the speed;

#pragma omp parallel for private(selectedRect)
		for (int j = 0; j < total; j++) {
			if (candidateFeatures[j].getGeom()->intersects(selectedRect)) {
				selectedFeatures.push_back(candidateFeatures[j]);
			}
		}
	}

	end_time = clock();
	cout << "Range Query time: " << (end_time - start_time) / 1000.0 << "s" << endl << endl;
	delete kdTree;
}

void SerialKDTreeRangeQueryAnalysis()
{
	if (!taxiTree) {
		loadTaxiData();
		buildTaxiTree();
	}
	taxiTree = true;
	atTaxi = true;
	atStation = false;
	updateMode();

	MapGeom::SerialKDTree* kdTree = new MapGeom::SerialKDTree(0.35);
	kdTree->setCts(1.31593);
	kdTree->buildKDTree(features);

	srand(time(NULL));
	double x1, y1, x2, y2;
	vector<MapGeom::Feature> candidateFeatures;
	vector<MapGeom::Feature> selectedFeatures;
	clock_t start_time = clock();
	clock_t end_time = clock();
	for (int i = 0; i <= 10000; i++) {
		candidateFeatures.clear();
		x1 = -((rand() % 225) / 10000.0 + 73.9812);
		y1 = (rand() % 239) / 10000.0 + 40.7247;
		x2 = -((rand() % 225) / 10000.0 + 73.9812);
		y2 = (rand() % 239) / 10000.0 + 40.7247;
		if (x1 > x2) {
			swap(x1, x2);
		}
		if (y1 > y2) {
			swap(y1, y2);
		}
		Envelope selectedRect(x1, x2, y1, y2);
		kdTree->rangeQuery(selectedRect, candidateFeatures);
		// refine step
		selectedFeatures.clear();
		for (MapGeom::Feature x : candidateFeatures) {
			if (x.getGeom()->intersects(selectedRect) == true) {
				selectedFeatures.push_back(x);
			}
		}
	}
	end_time = clock();
	cout << "Range Query time: " << (end_time - start_time) / 1000.0 << "s" << endl << endl;
	delete kdTree;
}

void QuadTreeRangeQueryAnalysis()
{
	if (!taxiTree) {
		loadTaxiData();
		buildTaxiTree();
	}
	taxiTree = true;
	atTaxi = true;
	atStation = false;
	updateMode();

	MapGeom::QuadTree* qTree = new MapGeom::QuadTree();
	qTree->setCapacity(25);
	qTree->constructQuadTree(features);

	srand(time(NULL));
	double x1, y1, x2, y2;
	vector<MapGeom::Feature> candidateFeatures;
	vector<MapGeom::Feature> selectedFeatures;
	clock_t start_time = clock();
	clock_t end_time = clock();
	for (int i = 0; i <= 10000; i++) {
		candidateFeatures.clear();
		x1 = -((rand() % 225) / 10000.0 + 73.9812);
		y1 = (rand() % 239) / 10000.0 + 40.7247;
		x2 = -((rand() % 225) / 10000.0 + 73.9812);
		y2 = (rand() % 239) / 10000.0 + 40.7247;
		if (x1 > x2) {
			swap(x1, x2);
		}
		if (y1 > y2) {
			swap(y1, y2);
		}
		Envelope selectedRect(x1, x2, y1, y2);
		qTree->rangeQuery(selectedRect, candidateFeatures);
		// refine step
		selectedFeatures.clear();
		for (MapGeom::Feature x : candidateFeatures) {
			if (x.getGeom()->intersects(selectedRect) == true) {
				selectedFeatures.push_back(x);
			}
		}
	}
	end_time = clock();
	cout << "Range Query time: " << (end_time - start_time) / 1000.0 << "s" << endl << endl;
	delete qTree;
}

void ParallelKDTreeNNQueryAnalysis()
{
	if (!taxiTree) {
		loadTaxiData();
		buildTaxiTree();
	}
	taxiTree = true;
	atTaxi = true;
	atStation = false;
	updateMode();

	for (double Ce = 0.05; Ce <= 0.95; Ce += 0.1) {
		for (double Cts = 1; Cts <= 2; Cts *= 1.04) {
			MapGeom::ParallelKDTree* kdTree = new MapGeom::ParallelKDTree(Ce);
			kdTree->setCts(Cts);
			kdTree->buildKDTree(features);

			srand(time(NULL));
			double x, y;
			vector<MapGeom::Feature> candidateFeatures;

			clock_t start_time = clock();
			clock_t end_time = clock();
			for (int i = 0; i <= 100000; i++) {
				candidateFeatures.clear();
				x = -((rand() % 225) / 10000.0 + 73.9812);
				y = (rand() % 239) / 10000.0 + 40.7247;
				kdTree->NNQuery(x, y, candidateFeatures);
				// refine step
				int NNIndex = 0;
				double minDist = candidateFeatures[0].distance(x, y);
				double dist;
				for (int j = 1; j < candidateFeatures.size(); j++) {
					dist = candidateFeatures[j].distance(x, y);
					if (dist < minDist) {
						minDist = dist;
						NNIndex = j;
					}
				}
			}
			end_time = clock();
			cout << "Parallel-KDTree Cts: " << Cts << ", Ce:" << Ce << ";  NNQuery time: " << (end_time - start_time) / 1000.0 << "s" << endl;
			delete kdTree;
		}
	}

}

void QuadTreeNNQueryAnalysis()
{
	if (!taxiTree) {
		loadTaxiData();
		buildTaxiTree();
	}
	taxiTree = true;
	atTaxi = true;
	atStation = false;
	updateMode();

	for (int capacity = 20; capacity <= 200; capacity++) {
		MapGeom::QuadTree* qTree = new MapGeom::QuadTree();
		qTree->setCapacity(capacity);
		qTree->constructQuadTree(features);

		srand(time(NULL));
		double x, y;
		vector<MapGeom::Feature> candidateFeatures;

		clock_t start_time = clock();
		clock_t end_time = clock();
		for (int i = 0; i <= 100000; i++) {
			candidateFeatures.clear();
			x = -((rand() % 225) / 10000.0 + 73.9812);
			y = (rand() % 239) / 10000.0 + 40.7247;
			qTree->NNQuery(x, y, candidateFeatures);
			// refine step
			int NNIndex = 0;
			double minDist = candidateFeatures[0].distance(x, y);
			double dist;
			for (int j = 1; j < candidateFeatures.size(); j++) {
				dist = candidateFeatures[j].distance(x, y);
				if (dist < minDist) {
					minDist = dist;
					NNIndex = j;
				}
			}
		}
		end_time = clock();
		cout << "Quad Tree Capacity: " << capacity << ";  NNQuery time: " << (end_time - start_time) / 1000.0 << "s" << endl;
		delete qTree;
	}

}

void SerialKDTreeNNQueryAnalysis()
{
	if (!taxiTree) {
		loadTaxiData();
		buildTaxiTree();
	}
	taxiTree = true;
	atTaxi = true;
	atStation = false;
	updateMode();
	for (double Ce = 0.05; Ce <= 0.95; Ce += 0.1) {
		for (double Cts = 1; Cts <= 2; Cts *= 1.04) {
			MapGeom::SerialKDTree* kdTree = new MapGeom::SerialKDTree(Ce);
			kdTree->setCts(Cts);
			kdTree->buildKDTree(features);

			srand(time(NULL));
			double x, y;
			vector<MapGeom::Feature> candidateFeatures;

			clock_t start_time = clock();
			clock_t end_time = clock();
			for (int i = 0; i <= 100000; i++) {
				candidateFeatures.clear();
				x = -((rand() % 225) / 10000.0 + 73.9812);
				y = (rand() % 239) / 10000.0 + 40.7247;
				kdTree->NNQuery(x, y, candidateFeatures);
				// refine step
				int NNIndex = 0;
				double minDist = candidateFeatures[0].distance(x, y);
				double dist;
				for (int j = 1; j < candidateFeatures.size(); j++) {
					dist = candidateFeatures[j].distance(x, y);
					if (dist < minDist) {
						minDist = dist;
						NNIndex = j;
					}
				}
			}
			end_time = clock();
			cout << "Serial-KDTree Cts: " << Cts << ", Ce:" << Ce << ";  NNQuery time: " << (end_time - start_time) / 1000.0 << "s" << endl;
			delete kdTree;
		}
	}
	
}

void BuildingTimeAnalysis()
{
	if (!taxiTree) {
		loadTaxiData();
		buildTaxiTree();
	}
	taxiTree = true;
	atTaxi = true;
	atStation = false;
	updateMode();
	int maxThreads = omp_get_max_threads();
	for (double Cts = 0.1; Cts <= 20; Cts *= 2) {
		for (double Ce = 0.05; Ce <= 0.95; Ce += 0.1) {
			MapGeom::ParallelKDTree* kdPTree = new MapGeom::ParallelKDTree(Ce);
			kdPTree->setCts(Cts);

			clock_t start_time = clock();
			kdPTree->buildKDTree(features);
			clock_t end_time = clock();
			clock_t ptime = end_time - start_time;

			MapGeom::SerialKDTree* kdTree = new MapGeom::SerialKDTree(Ce);
			kdTree->setCts(Cts);

			start_time = clock();
			kdTree->buildKDTree(features);
			end_time = clock();
			clock_t time = end_time - start_time;

			cout << "The parallel algorithm saved " << 100.0 * (1 - (double)ptime / time) << "% time cost "
				<< "when Ce = " << Ce << " and Cts = " << Cts << endl;
			delete kdPTree;
			delete kdTree;
			// make sure all the threads have been recycled;
			while (omp_get_max_threads() < maxThreads) {
				cout << "Sleep 1s to get threads back;" << endl;
				_sleep(1000);
			}
		}
	}


}

void test(int t)
{
	cout << "*********************Start*********************" << endl;
	if (t == TEST1) {
		cout << "TEST1: Comparison of Building Time between Parallel-KDTree and Serial-KDTree" << endl;
		BuildingTimeAnalysis();
	}
	else if (t == TEST2) {
		cout << "TEST2: QuadTree NN Query Analysis" << endl;
		QuadTreeNNQueryAnalysis();
	}
	else if (t == TEST3) {
		cout << "TEST3: Serial-KDTree NN Query Analysis" << endl;
		SerialKDTreeNNQueryAnalysis();
	}
	else if (t == TEST4) {
		cout << "TEST4: Parallel-KDTree NN Query Analysis" << endl;
		ParallelKDTreeNNQueryAnalysis();
	}
	else if (t == TEST5) {
		cout << "TEST5: QuadTree Range Query Analysis for the Best Parameter" << endl;
		QuadTreeRangeQueryAnalysis();
	}
	else if (t == TEST6) {
		cout << "TEST6: Serial-KDTree Range Query Analysis for the Best Parameter" << endl;
		SerialKDTreeRangeQueryAnalysis();
	}
	else if (t == TEST7) {
		cout << "TEST7: Parallel-KDTree Range Query Analysis for the Best Parameter" << endl;
		ParallelKDTreeRangeQueryAnalysis();
	}
	else if (t == TEST8) {
		cout << "TEST8: QuadTree NN Query Worst Performance Analysis" << endl;
		QuadTreeNNWorstAnalysis();
	}
	else if (t == TEST9) {
		cout << "TEST9: Serial-KDTree NN Query Worst Performance Analysis" << endl;
		SerialKDTreeNNWorstAnalysis();
	}

	cout << "**********************End**********************" << endl;
}
