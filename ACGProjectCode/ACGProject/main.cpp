#include "common.h"
#include "Geometry.h"
#include "QuadTree.h"
#include "SerialKDTree.h"
#include "ParallelKDTree.h"
#include "shapelib/shapefil.h"
#include <gl/freeglut.h>       // Glut¿âÍ·ÎÄ¼þ


using namespace std;

extern void test(int t);

int screenWidth = 650;
int screenHeight = 650;

double pointSize = 2.0;

SearchMode mode = Default;
TreeMode treeState = QUADTREE;

bool showTreeNodes = false;
bool taxiTree = false;
bool roadTree = false;
bool atStation = true;
bool atTaxi = false;
bool rangeSearch = false;
bool nnSearch = false;

vector<MapGeom::Feature> features;
vector<MapGeom::Feature> roads;

MapGeom::QuadTree stationQTree;
MapGeom::QuadTree roadQTree;
MapGeom::QuadTree taxiQTree;

MapGeom::SerialKDTree stationSerialKDTree;
MapGeom::SerialKDTree roadSerialKDTree;
MapGeom::SerialKDTree taxiSerialKDTree;


MapGeom::ParallelKDTree stationParallelKDTree;
MapGeom::ParallelKDTree roadParallelKDTree;
MapGeom::ParallelKDTree taxiParallelKDTree;

MapGeom::Feature nearestFeature;

bool firstPoint = true;
MapGeom::Point corner[2];
MapGeom::Envelope selectedRect;
vector<MapGeom::Feature> selectedFeatures;

// read shapefile document;
/* NOT MY WORK */
vector<string> readName(const char* filename)
{
	DBFHandle file = DBFOpen(filename, "r");

	vector<string> res;
	int cct = DBFGetRecordCount(file);
	res.reserve(cct);
	for (int i = 0; i < cct; ++i) {
		string a = DBFReadStringAttribute(file, i, 0);
		res.push_back(a);
	}

	DBFClose(file);

	return res;
}

// read geometry from shapefile document;
/* NOT MY WORK */
vector<MapGeom::Geometry*> readGeom(const char* filename)
{
	SHPHandle file = SHPOpen(filename, "r");

	int pnEntities, pnShapeType;
	double padfMinBound[4], padfMaxBound[4];
	SHPGetInfo(file, &pnEntities, &pnShapeType, padfMinBound, padfMaxBound);

	vector<MapGeom::Point> points;
	vector<MapGeom::Geometry*> geoms;
	geoms.reserve(pnEntities);
	switch (pnShapeType) {
	case SHPT_POINT:
		for (int i = 0; i < pnEntities; ++i) {
			SHPObject* pt = SHPReadObject(file, i);
			geoms.push_back(new MapGeom::Point(pt->padfY[0], pt->padfX[0]));
			SHPDestroyObject(pt);
		}
		break;

	case SHPT_ARC:
		for (int i = 0; i < pnEntities; ++i) {
			points.clear();
			SHPObject* pt = SHPReadObject(file, i);
			for (int j = 0; j < pt->nVertices; ++j) {
				points.push_back(MapGeom::Point(pt->padfY[j], pt->padfX[j]));
			}
			SHPDestroyObject(pt);
			geoms.push_back(new MapGeom::LineString(points));
		}
		break;

	}

	SHPClose(file);
	return geoms;
}

// print geometry information;
/* NOT MY WORK */
void printGeom(vector<MapGeom::Geometry*>& geom)
{
	cout << "Geometry:" << endl;
	for (vector<MapGeom::Geometry*>::iterator it = geom.begin(); it != geom.end(); ++it) {
		(*it)->print();
	}
}

// delete geometry information;
/* NOT MY WORK */
void deleteGeom(vector<MapGeom::Geometry*>& geom)
{
	for (vector<MapGeom::Geometry*>::iterator it = geom.begin(); it != geom.end(); ++it) {
		delete* it;
		*it = NULL;
	}
	geom.clear();
}

void updateMode() {
	if (nnSearch && atTaxi) {
		mode = NNTAXI;
	}
	else if (rangeSearch && atTaxi) {
		mode = RANGETAXI;
	}
	else if (nnSearch && atStation) {
		mode = NNSTATION;
	}
	else if (rangeSearch && atStation) {
		mode = RANGESTATION;
	}
	else {
		if (nnSearch) mode = NNROAD;
		else if (rangeSearch) mode = RANGEROAD;
	}
}

void loadRoadData()
{
	vector<MapGeom::Geometry*> geom = readGeom(".//data/highway");

	roads.clear();
	for (size_t i = 0; i < geom.size(); ++i)
		roads.push_back(MapGeom::Feature(to_string(i), geom[i]));

	cout << "road number: " << geom.size() << endl;
}

void buildRoadTree() {
	cout << "-------START BUILDING FOR ROAD-------" << endl;
	roadQTree.setCapacity(20);
	roadQTree.constructQuadTree(roads);

	roadSerialKDTree.buildKDTree(roads);

	roadParallelKDTree.buildKDTree(roads);
	cout << "-----------------END-----------------" << endl;

	atTaxi = false;
	atStation = false;
	updateMode();
}

void loadStationData()
{
	vector<MapGeom::Geometry*> geom = readGeom(".//data/station");
	vector<string> name = readName(".//data/station");

	features.clear();
	for (size_t i = 0; i < geom.size(); ++i)
		features.push_back(MapGeom::Feature(name[i], geom[i]));

	cout << "station number: " << geom.size() << endl;
}

void buildStationTree() {
	cout << "-----START BUILDING FOR STATIONS-----" << endl;
	stationQTree.setCapacity(5);
	stationQTree.constructQuadTree(features);

	stationSerialKDTree.buildKDTree(features);

	stationParallelKDTree.buildKDTree(features);
	cout << "-----------------END-----------------" << endl;
}

void loadTaxiData()
{
	vector<MapGeom::Geometry*> geom = readGeom(".//data/taxi");
	vector<string> name = readName(".//data/taxi");

	features.clear();
	for (size_t i = 0; i < geom.size(); ++i)
		features.push_back(MapGeom::Feature(name[i], geom[i]));

	cout << "taxi number: " << geom.size() << endl;

}

void buildTaxiTree() {
	cout << "-------START BUILDING FOR TAXI-------" << endl;
	cout << "It will take about 2 minutes in debug mode..." << endl;
	taxiQTree.setCapacity(32);
	taxiQTree.constructQuadTree(features);

	taxiSerialKDTree.buildKDTree(features);

	taxiParallelKDTree.buildKDTree(features);
	cout << "-----------------END-----------------" << endl;
	atTaxi = true;
	atStation = false;
	updateMode();
}

void rangeQuery()
{
	vector<MapGeom::Feature> candidateFeatures;

	// filter step:
	switch (treeState) {
	case QUADTREE:
		if (mode == RANGESTATION)
			stationQTree.rangeQuery(selectedRect, candidateFeatures);
		else if (mode == RANGEROAD)
			roadQTree.rangeQuery(selectedRect, candidateFeatures);
		else if (mode == RANGETAXI)
			taxiQTree.rangeQuery(selectedRect, candidateFeatures);

		break;
	case SERIAL:
		if (mode == RANGESTATION)
			stationSerialKDTree.rangeQuery(selectedRect, candidateFeatures);
		else if (mode == RANGEROAD)
			roadSerialKDTree.rangeQuery(selectedRect, candidateFeatures);
		else if (mode == RANGETAXI)
			taxiSerialKDTree.rangeQuery(selectedRect, candidateFeatures);
		break;
	case PARALLEL:
		if (mode == RANGESTATION)
			stationParallelKDTree.rangeQuery(selectedRect, candidateFeatures);
		else if (mode == RANGEROAD)
			roadParallelKDTree.rangeQuery(selectedRect, candidateFeatures);
		else if (mode == RANGETAXI)
			taxiParallelKDTree.rangeQuery(selectedRect, candidateFeatures);
		break;
	}

	// refine step:
	selectedFeatures.clear();
	for (MapGeom::Feature x : candidateFeatures) {
		if (x.getGeom()->intersects(selectedRect) == true) {
			selectedFeatures.push_back(x);
		}
	}

}

void NNQuery(MapGeom::Point p)
{
	vector<MapGeom::Feature> candidateFeatures;

	// filter step:
	switch (treeState) {
	case QUADTREE:
		if (mode == NNSTATION)
			stationQTree.NNQuery(p.getX(), p.getY(), candidateFeatures);
		else if (mode == NNROAD)
			roadQTree.NNQuery(p.getX(), p.getY(), candidateFeatures);
		else if (mode == NNTAXI)
			taxiQTree.NNQuery(p.getX(), p.getY(), candidateFeatures);
		break;
	case SERIAL:
		if (mode == NNSTATION)
			stationSerialKDTree.NNQuery(p.getX(), p.getY(), candidateFeatures);
		else if (mode == NNROAD)
			roadSerialKDTree.NNQuery(p.getX(), p.getY(), candidateFeatures);
		else if (mode == NNTAXI)
			taxiSerialKDTree.NNQuery(p.getX(), p.getY(), candidateFeatures);
		break;
	case PARALLEL:
		if (mode == NNSTATION)
			stationParallelKDTree.NNQuery(p.getX(), p.getY(), candidateFeatures);
		else if (mode == NNROAD)
			roadParallelKDTree.NNQuery(p.getX(), p.getY(), candidateFeatures);
		else if (mode == NNTAXI)
			taxiParallelKDTree.NNQuery(p.getX(), p.getY(), candidateFeatures);
		break;

	}

	if (candidateFeatures.empty()) {
		cout << "NN query fails!" << endl;
		return;
	}


	// refine step:
	int NNIndex = 0;
	double minDist = candidateFeatures[0].distance(p.getX(), p.getY());
	double dist;
	for (int i = 1; i < candidateFeatures.size(); i++) {
		dist = candidateFeatures[i].distance(p.getX(), p.getY());
		if (dist < minDist) {
			minDist = dist;
			NNIndex = i;
		}
	}
	nearestFeature = candidateFeatures[NNIndex];

}

// transform coordination from screen to geographical;
/* NOT MY WORK */
void transfromPt(MapGeom::Point& pt)
{
	const MapGeom::Envelope bbox = stationQTree.getEnvelope();
	double width = bbox.getMaxX() - bbox.getMinX() + 0.002;
	double height = bbox.getMaxY() - bbox.getMinY() + 0.002;

	double x = pt.getX() * width / screenWidth + bbox.getMinX() - 0.001;
	double y = pt.getY() * height / screenHeight + bbox.getMinY() - 0.001;

	x = max(bbox.getMinX(), x);
	x = min(bbox.getMaxX(), x);
	y = max(bbox.getMinY(), y);
	y = min(bbox.getMaxY(), y);
	pt = MapGeom::Point(x, y);
}

void display()
{
	//glClearColor(241 / 255.0, 238 / 255.0, 232 / 255.0, 0.0); 
	glClearColor(1.0, 1.0, 1.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	const MapGeom::Envelope bbox = stationQTree.getEnvelope();
	gluOrtho2D(bbox.getMinX() - 0.001, bbox.getMaxX() + 0.001, bbox.getMinY() - 0.001, bbox.getMaxY() + 0.001);

	// render road;
	glColor3d(252 / 255.0, 214 / 255.0, 164 / 255.0);
	for (size_t i = 0; i < roads.size(); ++i) {
		roads[i].draw();
	}

	// render points;
	if (atStation || atTaxi) {
		glPointSize((float)pointSize);
		glColor3d(0.0, 146 / 255.0, 247 / 255.0);
		for (size_t i = 0; i < features.size(); ++i)
			features[i].draw();
	}

	// render tree nodes;
	if (showTreeNodes) {
		glColor3d(0.0, 146 / 255.0, 247 / 255.0);
		switch (treeState) {
		case QUADTREE:
			if (mode == RANGESTATION || mode == NNSTATION || atStation == true)
				stationQTree.draw();
			else if (mode == RANGETAXI || mode == NNTAXI || atTaxi == true)
				taxiQTree.draw();
			else roadQTree.draw();
			break;
		case SERIAL:
			if (mode == RANGESTATION || mode == NNSTATION || atStation == true)
				stationSerialKDTree.draw();
			else if (mode == RANGETAXI || mode == NNTAXI || atTaxi == true)
				taxiSerialKDTree.draw();
			else roadSerialKDTree.draw();
			break;
		case PARALLEL:
			if (mode == RANGESTATION || mode == NNSTATION || atStation == true)
				stationParallelKDTree.draw();
			else if (mode == RANGETAXI || mode == NNTAXI || atTaxi == true)
				taxiParallelKDTree.draw();
			else roadParallelKDTree.draw();
			break;
		}

	}

	// render the closest point;
	if (mode == NNSTATION || mode == NNTAXI) {
		glPointSize(5.0);
		glColor3d(0.9, 0.0, 0.0);
		nearestFeature.draw();
	}

	// render the closest line;
	if (mode == NNROAD) {
		glLineWidth(3.0);
		glColor3d(0.9, 0.0, 0.0);
		nearestFeature.draw();
		glLineWidth(1.0);
	}

	// highlight a region;
	if (mode == RANGESTATION || mode == RANGEROAD || mode == RANGETAXI) {
		glColor3d(0.0, 0.0, 0.0);
		selectedRect.draw();
		glColor3d(1.0, 0.0, 0.0);
		for (size_t i = 0; i < selectedFeatures.size(); ++i)
			selectedFeatures[i].draw();
	}

	glFlush();
	glutSwapBuffers();
}

/* NOT MY WORK */
void mouse(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
		if (mode == RANGESTATION || mode == RANGEROAD || mode == RANGETAXI) {
			if (firstPoint) {
				selectedFeatures.clear();
				corner[0] = MapGeom::Point(x, screenHeight - y);
				transfromPt(corner[0]);
			}
			else {
				corner[1] = MapGeom::Point(x, screenHeight - y);
				transfromPt(corner[1]);
				selectedRect = MapGeom::Envelope(min(corner[0].getX(), corner[1].getX()), max(corner[0].getX(), corner[1].getX()),
					min(corner[0].getY(), corner[1].getY()), max(corner[0].getY(), corner[1].getY()));
				rangeQuery();
			}
			firstPoint = !firstPoint;
			glutPostRedisplay();
		}
	}
}

/* NOT MY WORK */
void passiveMotion(int x, int y)
{
	corner[1] = MapGeom::Point(x, screenHeight - y);

	if ((mode == RANGESTATION || mode == RANGEROAD || mode == RANGETAXI) && !firstPoint) {
		corner[1] = MapGeom::Point(x, screenHeight - y);
		transfromPt(corner[1]);
		selectedRect = MapGeom::Envelope(min(corner[0].getX(), corner[1].getX()), max(corner[0].getX(), corner[1].getX()),
			min(corner[0].getY(), corner[1].getY()), max(corner[0].getY(), corner[1].getY()));
		rangeQuery();

		glutPostRedisplay();
	}
	else if (mode == NNSTATION || mode == NNROAD || mode == NNTAXI) {
		MapGeom::Point p(x, screenHeight - y);
		transfromPt(p);
		NNQuery(p);

		glutPostRedisplay();
	}
}

/* NOT MY WORK */
void changeSize(int w, int h)
{
	screenWidth = w;
	screenHeight = h;
	glViewport(0, 0, w, h);
	glutPostRedisplay();
}

void printMode() {
	cout << "This mode is ";
	switch (mode)
	{
	case Default:
		cout << "only display ";
		break;
	case RANGESTATION:
		cout << "range query for stations ";
		break;
	case RANGEROAD:
		cout << "range query for roads ";
		break;
	case RANGETAXI:
		cout << "range query for taxis ";
		break;
	case NNSTATION:
		cout << "knn query for stations ";
		break;
	case NNROAD:
		cout << "knn query for roads ";
		break;
	case NNTAXI:
		cout << "knn query for taxis ";
		break;
	default:
		break;
	}
	cout << "via ";
	switch (treeState) {
	case QUADTREE:
		cout << "QuadTree;" << endl;
		break;
	case SERIAL:
		cout << "Serial-KdTree; " << endl;
		break;
	case PARALLEL:
		cout << "Parallel-KdTree;" << endl;
		break;
	}
}

void processNormalKeys(unsigned char key, int x, int y)
{
	switch (key) {
	case 27:exit(0); break;
	case 'Q':
	case 'q':
		treeState = QUADTREE;
		break;
	case 'C':
	case 'c':
		treeState = SERIAL;
		break;
	case 'P':
	case 'p':
		treeState = PARALLEL;
		break;
	case 'S':
	case 's':
		rangeSearch = true;
		nnSearch = false;
		firstPoint = true;
		break;
	case 'N':
	case 'n':
		rangeSearch = false;
		nnSearch = true;
		break;
	case 'A':
	case 'a':
		if (!roadTree) {
			loadRoadData();
			buildRoadTree();
			roadTree = true;
		}
		cout << "The road tree has been built successfully!" << endl;
		break;
	case 'B':
	case 'b':
		if (!taxiTree) {
			loadTaxiData();
			buildTaxiTree();
			taxiTree = true;
		}
		cout << "The taxi tree has been built successfully!" << endl;
		break;
	case 'W':
	case 'w':
		showTreeNodes = false;
		loadStationData();
		atStation = true;
		atTaxi = false;
		break;
	case 'R':
	case 'r':
		showTreeNodes = false;
		if (!roadTree) {
			cout << "Please press A/a to build road tree first!" << endl;
			return;
		}
		atStation = false;
		atTaxi = false;
		break;
	case 'T':
	case 't':
		showTreeNodes = false;
		loadTaxiData();
		if (!taxiTree) {
			cout << "Please press B/b to build taxi tree first!" << endl;
			return;
		}
		atStation = false;
		atTaxi = true;
		break;
	case 'X':
	case 'x':
		if ((mode == NNTAXI || mode == RANGETAXI) && !taxiTree) {
			cout << "Please press B/b to build taxi tree first!" << endl;
			return;
		}
		else if ((mode == NNROAD || mode == RANGEROAD) && !roadTree) {
			cout << "Please press A/a to build road tree first!" << endl;
			return;
		}
		showTreeNodes = !showTreeNodes;
		break;
	case '+':
		pointSize *= 1.1;
		break;
	case '-':
		pointSize /= 1.1;
		break;
	case '1':
		test(TEST1); break;
	case '2':
		test(TEST2); break;
	case '3':
		test(TEST3); break;
	case '4':
		test(TEST4); break;
	case '5':
		test(TEST5); break;
	case '6':
		test(TEST6); break;
	case '7':
		test(TEST7); break;
	case '8':
		test(TEST8); break;
	case '9':
		test(TEST9); break;
	default:
		//mode = Default;
		break;
	}
	updateMode();
	printMode();
	glutPostRedisplay();
}

int main(int argc, char* argv[])
{
	cout << "Key Usage:\n"
		<< "  Q/q: switch to QuadTree mode (default)\n"
		<< "  C/c: switch to Serial-KdTree mode\n"
		<< "  P/p: switch to Parallel-KdTree mode\n"
		<< "  S/s: range search\n"
		<< "  N/n: nearest search\n"
		<< "  A/a: load road data\n"
		<< "  B/b: load taxi data\n"
		<< "  W/w: switch to station mode\n"
		<< "  R/r: switch to road mode\n"
		<< "  T/t: switch to taxi mode\n"
		<< "  X/x: show tree nodes\n"
		<< "  +  : increase point size\n"
		<< "  -  : decrease point size\n"
		<< "  1  : TEST1: Comparison of Building Time between Parallel-KDTree and Serial-KDTree\n"
		<< "  2  : TEST2: QuadTree NN Query Analysis\n"
		<< "  3  : TEST3: Serial-KDTree NN Query Analysis\n"
		<< "  4  : TEST4: Parallel-KDTree NN Query Analysis\n"
		<< "  5  : TEST5: QuadTree Range Query Analysis for the Best Parameter\n"
		<< "  6  : TEST6: Serial-KDTree Range Query Analysis for the Best Parameter\n"
		<< "  7  : TEST7: Parallel-KDTree Range Query Analysis for the Best Parameter\n"
		<< "  8  : TEST8: QuadTree NN Query Worst Performance Analysis\n"
		<< "  9  : TEST9: Serial-KDTree NN Query Worst Performance Analysis\n"
		<< "  ESC: quit\n"
		<< endl;

	loadRoadData();
	loadStationData();
	buildStationTree();

	glutInit(&argc, argv);
	glutInitWindowSize(screenWidth, screenHeight);
	glutInitWindowPosition(0, 0);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutCreateWindow("New York");

	glutMouseFunc(mouse);
	glutDisplayFunc(display);
	glutPassiveMotionFunc(passiveMotion);
	glutReshapeFunc(changeSize);
	glutKeyboardFunc(processNormalKeys);

	glutMainLoop();

	return 0;
}
