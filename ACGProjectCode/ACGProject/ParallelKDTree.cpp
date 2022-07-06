#include "ParallelKDTree.h"

namespace MapGeom {

	splitCandidate::splitCandidate(ParallelKDSmallNode& smallRoot, Axis axis, int splitNum, int totalBin)
	{
		// splitNum is confined from 1 to totalBin - 1;

		Envelope envelope = smallRoot.getEnvelope();
		double xmin = envelope.getMinX();
		double xmax = envelope.getMaxX();
		double ymin = envelope.getMinY();
		double ymax = envelope.getMaxY();

		double split;
		switch (axis) {
		case Axis::X:
			split = xmin + (xmax - xmin) * splitNum / totalBin;
			leftChild = Envelope(xmin, split, ymin, ymax);
			rightChild = Envelope(split, xmax, ymin, ymax);
			break;
		case Axis::Y:
			split = ymin + (ymax - ymin) * splitNum / totalBin;
			leftChild = Envelope(xmin, xmax, ymin, split);
			rightChild = Envelope(xmin, xmax, split, ymax);
			break;
		}

		// Generate the bitmask;
		left = right = 0;
		Envelope tempE;
		for (int i = smallRoot.getFeatureNum() - 1; i >= 0; i--) {
			tempE = smallRoot.getFeature(i).getEnvelope();
			left = left << 1;
			right = right << 1;
			if (tempE.intersect(leftChild)) {
				left += 1;
			}
			if (tempE.intersect(rightChild)) {
				right += 1;
			}
		}

	}

	double splitCandidate::getLeftPerimeter(const ParallelKDSmallNode& node) const
	{
		return leftChild.intersectEnvelope(node.getEnvelope()).getPerimeter();
	}

	double splitCandidate::getRightPerimeter(const ParallelKDSmallNode& node) const
	{
		return rightChild.intersectEnvelope(node.getEnvelope()).getPerimeter();
	}

	void specificAssignment(vector<ParallelKDNode*>& To, vector<ParallelKDSmallNode*>& From)
	{
		To.clear();
		for (vector<ParallelKDSmallNode*>::iterator it = From.begin(); it != From.end(); it++) {
			To.push_back(*it);
		}
	}

	void ParallelKDNode::add(const Feature& f)
	{
		if (!f.getEnvelope().intersect(this->bbox)) return;

		vector <Geometry*> clipAns = f.getGeom()->clip(bbox);
		for (int i = 0; i < clipAns.size(); i++) {
			features.push_back(Feature(f.getName() + "_" + to_string(i), clipAns[i]));
			delete clipAns[i];
		}
	}

	void ParallelKDNode::add(const vector<Feature>& fs)
	{
		for (int i = 0; i < fs.size(); i++) {
			add(fs[i]);
		}
		return;

	}

	// This function has been abandoned because of its bad performance;
	void ParallelKDNode::parallelAdd(const vector<Feature>& fs)
	{
#pragma omp parallel for num_threads(THREADS_NUM)
		for (int i = 0; i < fs.size(); i++) {
			if (!fs[i].getEnvelope().intersect(this->bbox)) continue;
			vector <Geometry*> clipAns = fs[i].getGeom()->clip(bbox);
			for (int j = 0; j < clipAns.size(); j++) {
				Feature f(fs[i].getName() + "_" + to_string(j), clipAns[j]);
#pragma omp critical
				features.push_back(f);
			}
		}
		return;
	}

	void ParallelKDNode::countNode(int& interiorNum, int& leafNum)
	{
		if (isLeafNode()) {
			leafNum++;
		}
		else {
			interiorNum++;
			for (int i = 0; i < 2; i++)
				nodes[i]->countNode(interiorNum, leafNum);
		}
	}

	int ParallelKDNode::countHeight(int height)
	{
		height++;
		if (!isLeafNode()) {
			int cur = height;
			for (int i = 0; i < 2; i++) {
				height = max(height, nodes[i]->countHeight(cur));
			}
		}
		return height;
	}

	void ParallelKDNode::draw()
	{
		if (isLeafNode()) {
			bbox.draw();
		}
		else {
			for (int i = 0; i < 2; i++) nodes[i]->draw();
		}
	}

	void ParallelKDNode::rangeQuery(Envelope& rect, vector<Feature>& features)
	{

		if (this->isLeafNode()) {
			((ParallelKDSmallNode*)this)->getAllFeaturesTo(features);
		}
		else {
			if (!this->nodes[0]->bbox.intersect(rect)) {
				this->nodes[1]->rangeQuery(rect, features);
			}
			else if (!this->nodes[1]->bbox.intersect(rect)) {
				this->nodes[0]->rangeQuery(rect, features);
			}
			else {
				this->nodes[0]->rangeQuery(rect, features);
				this->nodes[1]->rangeQuery(rect, features);
			}
		}
	}

	// This function search a leaf node which has AT LEAST ONE feature with BACKTRACKING!
	ParallelKDNode* ParallelKDNode::pointInLeafNode(double x, double y)
	{
		if (this->isLeafNode() && ((ParallelKDSmallNode*)this)->bitmask == 0) {
			return NULL;
		}
		else if (this->isLeafNode() && ((ParallelKDSmallNode*)this)->bitmask != 0) {
			return this;
		}
		else {
			ParallelKDNode* ans = NULL;

			if (this->nodes[0]->bbox.contain(x, y)) {
				ans = this->nodes[0]->pointInLeafNode(x, y);
				if (ans != NULL) {
					return ans;
				}
				ans = this->nodes[1]->pointInLeafNode(x, y);
				return ans;

			}
			else {
				ans = this->nodes[1]->pointInLeafNode(x, y);
				if (ans != NULL) {
					return ans;
				}
				ans = this->nodes[0]->pointInLeafNode(x, y);
				return ans;
			}

		}
	}

	ParallelKDSmallNode::ParallelKDSmallNode(const ParallelKDSmallNode& smallNode, const splitCandidate& sc, Orientation o)
	{
		smallRoot = smallNode.smallRoot;
		Envelope myEnvelope = smallNode.getEnvelope();
		switch (o)
		{
		case Orientation::Left:
			bbox = myEnvelope.intersectEnvelope(sc.getLeftEnvelope());
			bitmask = sc.getLeftCode(smallNode.bitmask);
			break;
		case Orientation::Right:
			bbox = myEnvelope.intersectEnvelope(sc.getRightEnvelope());
			bitmask = sc.getRightCode(smallNode.bitmask);
			break;
		}
		nodes[0] = nodes[1] = NULL;
	}

	double ParallelKDSmallNode::getPerimeter() const
	{
		return bbox.getPerimeter();
	}

	void ParallelKDSmallNode::getAllFeaturesTo(vector <Feature>& fs) const
	{
		if (features.size() != 0) {
			fs.insert(fs.end(), features.begin(), features.end());
		}
		else {
			unsigned long code = bitmask;
			int index = 0;
			while (code != 0) {
				if ((code & 1) == 1) fs.push_back(smallRoot->features[index]);
				index++;
				code = code >> 1;
			}
		}

	}

	bool ParallelKDTree::buildKDTree(vector<Feature>& features)
	{
		clock_t basic_time = clock();
		clock_t start_time = basic_time;
		clock_t end_time = clock();

		if (features.empty())
			return false;
		vector<ParallelKDNode*> nodelist, activelist, nextlist;
		vector<ParallelKDSmallNode*> smalllist;
		bbox = getFeaturesEnvelopeWithReduction(features);
		this->root = new ParallelKDNode(bbox);
		this->root->add(features);
		activelist.push_back(root);

		end_time = clock();
		cout << "Parallel preparation time: " << (end_time - start_time) / 1000.0 << "s" << endl;
		start_time = clock();

		// Large node stage;
		while (!activelist.empty()) {
			nodelist.insert(nodelist.end(), activelist.begin(), activelist.end());
			nextlist.clear();
			processLargeNodesWithoutSynchronize(activelist, smalllist, nextlist);
			nextlist.swap(activelist);
		}

		end_time = clock();
		cout << "Parallel large node stage time: " << (end_time - start_time) / 1000.0 << "s" << endl;
		start_time = clock();

		// Small node stage;
		preprocessSmallNodes(smalllist);
		specificAssignment(activelist, smalllist);

		while (!activelist.empty()) {
			nodelist.insert(nodelist.end(), activelist.begin(), activelist.end());
			nextlist.clear();
			processSmallNodes(activelist, nextlist);
			nextlist.swap(activelist);
		}

		end_time = clock();
		cout << "Parallel small node stage time: " << (end_time - start_time) / 1000.0 << "s" << endl;
		start_time = clock();

		cout << "Parallel building time: " << (end_time - basic_time) / 1000.0 << "s" << endl;
	}

	void ParallelKDTree::processLargeNodesWithoutSynchronize(vector<ParallelKDNode*>& activelist, vector<ParallelKDSmallNode*>& smalllist, vector<ParallelKDNode*>& nextlist)
	{
		// Get the total envelope of each node in activelist;
		vector<Envelope> envelopes;
		for (ParallelKDNode* x : activelist) {
			envelopes.push_back(getFeaturesEnvelopeWithReduction(x->getFeatures()));
		}


		ParallelKDSmallNode** tmpSmall = new ParallelKDSmallNode * [2 * activelist.size()];
		ParallelKDNode** tmpNext = new ParallelKDNode * [2 * activelist.size()];

#pragma omp parallel for num_threads(THREADS_NUM)
		for (int i = 0; i < 2 * activelist.size(); i++) {
			tmpSmall[i] = NULL;
			tmpNext[i] = NULL;
		}

#pragma omp parallel for num_threads(THREADS_NUM)
		for (int i = 0; i < activelist.size(); i++) {
			Envelope ch1, ch2;
			double xmin, xmax, ymin, ymax, xminE, xmaxE, yminE, ymaxE, xrange, yrange;

			xmin = activelist[i]->getEnvelope().getMinX();
			xmax = activelist[i]->getEnvelope().getMaxX();
			ymin = activelist[i]->getEnvelope().getMinY();
			ymax = activelist[i]->getEnvelope().getMaxY();

			xminE = envelopes[i].getMinX();
			xmaxE = envelopes[i].getMaxX();
			yminE = envelopes[i].getMinY();
			ymaxE = envelopes[i].getMaxY();

			xrange = xmax - xmin;
			yrange = ymax - ymin;

			// Compare each side of node i;

			if ((xmax - xmaxE) / xrange >= Ce) {
				ch1 = Envelope(xmaxE, xmax, ymin, ymax);
				ch2 = Envelope(xmin, xmaxE, ymin, ymax);
			}
			else if ((xminE - xmin) / xrange >= Ce) {
				ch1 = Envelope(xmin, xminE, ymin, ymax);
				ch2 = Envelope(xminE, xmax, ymin, ymax);
			}
			else if ((ymax - ymaxE) / yrange >= Ce) {
				ch1 = Envelope(xmin, xmax, ymaxE, ymax);
				ch2 = Envelope(xmin, xmax, ymin, ymaxE);
			}
			else if ((yminE - ymin) / yrange >= Ce) {
				ch1 = Envelope(xmin, xmax, ymin, yminE);
				ch2 = Envelope(xmin, xmax, yminE, ymax);
			}
			else {
				if (xrange > yrange) {
					ch1 = Envelope(xmin, 0.5 * (xmin + xmax), ymin, ymax);
					ch2 = Envelope(0.5 * (xmin + xmax), xmax, ymin, ymax);
				}
				else if (xrange <= yrange) {
					ch1 = Envelope(xmin, xmax, ymin, 0.5 * (ymin + ymax));
					ch2 = Envelope(xmin, xmax, 0.5 * (ymin + ymax), ymax);
				}
			}

			activelist[i]->nodes[0] = new ParallelKDNode(ch1);
			activelist[i]->nodes[1] = new ParallelKDNode(ch2);
			activelist[i]->nodes[0]->add(activelist[i]->features);
			activelist[i]->nodes[1]->add(activelist[i]->features);
			activelist[i]->features.clear();

#pragma omp parallel for
			for (int j = 0; j < 2; j++) {
				if (activelist[i]->nodes[j]->features.size() > threshold) {
					tmpNext[2 * i + j] = activelist[i]->nodes[j];
					//nextlist.push_back(activelist[i]->nodes[j]);
				}
				else {
					ParallelKDSmallNode* smallRoot = new ParallelKDSmallNode(*activelist[i]->nodes[j]);
					//smalllist.push_back(smallRoot);
					tmpSmall[2 * i + j] = smallRoot;
					delete activelist[i]->nodes[j];
					activelist[i]->nodes[j] = smallRoot;
				}
			}
		}

		for (int i = 0; i < 2 * activelist.size(); i++) {
			if (tmpSmall[i] != NULL) {
				smalllist.push_back(tmpSmall[i]);
			}
			else {
				nextlist.push_back(tmpNext[i]);
			}
		}

		delete[] tmpSmall;
		delete[] tmpNext;
	}

	void ParallelKDTree::processLargeNodes(vector<ParallelKDNode*>& activelist, vector<ParallelKDSmallNode*>& smalllist, vector<ParallelKDNode*>& nextlist)
	{
		// Get the total envelope of each node in activelist;
		vector<Envelope> envelopes;
		for (ParallelKDNode* x : activelist) {
			envelopes.push_back(getFeaturesEnvelopeWithReduction(x->getFeatures()));
		}


#pragma omp parallel for num_threads(THREADS_NUM)
		for (int i = 0; i < activelist.size(); i++) {
			Envelope ch1, ch2;
			double xmin, xmax, ymin, ymax, xminE, xmaxE, yminE, ymaxE, xrange, yrange;

			xmin = activelist[i]->getEnvelope().getMinX();
			xmax = activelist[i]->getEnvelope().getMaxX();
			ymin = activelist[i]->getEnvelope().getMinY();
			ymax = activelist[i]->getEnvelope().getMaxY();

			xminE = envelopes[i].getMinX();
			xmaxE = envelopes[i].getMaxX();
			yminE = envelopes[i].getMinY();
			ymaxE = envelopes[i].getMaxY();

			xrange = xmax - xmin;
			yrange = ymax - ymin;

			// Compare each side of node i;

			if ((xmax - xmaxE) / xrange >= Ce) {
				ch1 = Envelope(xmaxE, xmax, ymin, ymax);
				ch2 = Envelope(xmin, xmaxE, ymin, ymax);
			}
			else if ((xminE - xmin) / xrange >= Ce) {
				ch1 = Envelope(xmin, xminE, ymin, ymax);
				ch2 = Envelope(xminE, xmax, ymin, ymax);
			}
			else if ((ymax - ymaxE) / yrange >= Ce) {
				ch1 = Envelope(xmin, xmax, ymaxE, ymax);
				ch2 = Envelope(xmin, xmax, ymin, ymaxE);
			}
			else if ((yminE - ymin) / yrange >= Ce) {
				ch1 = Envelope(xmin, xmax, ymin, yminE);
				ch2 = Envelope(xmin, xmax, yminE, ymax);
			}
			else {
				if (xrange > yrange) {
					ch1 = Envelope(xmin, 0.5 * (xmin + xmax), ymin, ymax);
					ch2 = Envelope(0.5 * (xmin + xmax), xmax, ymin, ymax);
				}
				else if (xrange <= yrange) {
					ch1 = Envelope(xmin, xmax, ymin, 0.5 * (ymin + ymax));
					ch2 = Envelope(xmin, xmax, 0.5 * (ymin + ymax), ymax);
				}
			}

			activelist[i]->nodes[0] = new ParallelKDNode(ch1);
			activelist[i]->nodes[1] = new ParallelKDNode(ch2);
			activelist[i]->nodes[0]->add(activelist[i]->features);
			activelist[i]->nodes[1]->add(activelist[i]->features);
			activelist[i]->features.clear();

			//#pragma omp parallel for
			for (int j = 0; j < 2; j++) {
				if (activelist[i]->nodes[j]->features.size() > threshold) {
#pragma omp critical
					nextlist.push_back(activelist[i]->nodes[j]);
				}
				else {
					ParallelKDSmallNode* smallRoot = new ParallelKDSmallNode(*activelist[i]->nodes[j]);
#pragma omp critical
					smalllist.push_back(smallRoot);
					delete activelist[i]->nodes[j];
					activelist[i]->nodes[j] = smallRoot;
				}
			}
		}


	}

	void ParallelKDTree::preprocessSmallNodes(vector<ParallelKDSmallNode*>& smalllist)
	{
		for (ParallelKDSmallNode* smallRoot : smalllist) {
			for (int i = 1; i < splitCandidateNum; i++) {
				smallRoot->splitList.push_back(splitCandidate(*smallRoot, Axis::X, i, splitCandidateNum));
				smallRoot->splitList.push_back(splitCandidate(*smallRoot, Axis::Y, i, splitCandidateNum));
			}
		}
		return;
	}

	void ParallelKDTree::processSmallNodes(vector<ParallelKDNode*>& activelist, vector<ParallelKDNode*>& nextlist)
	{
#pragma omp parallel for num_threads(SMALL_NODES_THREADS_NUM)
		for (int k = 0; k < activelist.size(); k++) {
			ParallelKDSmallNode* smallNode = (ParallelKDSmallNode*)activelist[k];
			unsigned long s = smallNode->bitmask;
			ParallelKDSmallNode* r = smallNode->smallRoot;
			double A0 = smallNode->getPerimeter();
			double SAH0 = count1(s);

			// Choose the best split plan;
			int minIndex = -1;
			int CL, CR;
			double AL, AR, SAH;
			if (SAH0 != 0) {
				for (int i = 0; i < r->splitList.size(); i++) {
					// Rule out plans cannot cut this bbox;
					Envelope splitLine = r->splitList[i].getLeftEnvelope().intersectEnvelope(r->splitList[i].getRightEnvelope());
					if (!activelist[k]->getEnvelope().intersect(splitLine))
						continue;

					CL = r->splitList[i].getLeftNum(s);
					CR = r->splitList[i].getRightNum(s);
					AL = r->splitList[i].getLeftPerimeter(*smallNode);
					AR = r->splitList[i].getRightPerimeter(*smallNode);
					SAH = (CL * AL + CR * AR) / A0 + Cts;
					if (SAH < SAH0) {
						SAH0 = SAH;
						minIndex = i;
					}
				}
			}

			if (minIndex == -1) {
				// This small node is a leaf node;
			}
			else {
				// Split this node with plan splitList[minIndex];
				smallNode->nodes[0] = new ParallelKDSmallNode(*smallNode, r->splitList[minIndex], Orientation::Left);
				smallNode->nodes[1] = new ParallelKDSmallNode(*smallNode, r->splitList[minIndex], Orientation::Right);
#pragma omp critical
				nextlist.push_back(smallNode->nodes[0]);
#pragma omp critical
				nextlist.push_back(smallNode->nodes[1]);
			}
		}
	}

	void ParallelKDTree::rangeQuery(Envelope& rect, vector<Feature>& features)
	{
		features.clear();
		if (!bbox.intersect(rect))
			return;
		this->root->rangeQuery(rect, features);

	}

	// This function has been abandoned because of its bad performance;
	void ParallelKDTree::rangeQueryParallel2(Envelope& rect, vector<Feature>& features)
	{
		features.clear();
		if (!bbox.intersect(rect))
			return;
		vector<Feature> f[4];
		Envelope e[4];
		double minx = rect.getMinX();
		double maxx = rect.getMaxX();
		double midx = 0.5 * (minx + maxx);
		double miny = rect.getMinY();
		double maxy = rect.getMaxY();
		double midy = 0.5 * (miny + maxy);
		e[0] = Envelope(minx, midx, miny, midy);
		e[1] = Envelope(midx, maxx, miny, midy);
		e[2] = Envelope(minx, midx, midy, maxy);
		e[3] = Envelope(midx, maxx, midy, maxy);
#pragma omp parallel for num_threads(4)
		for (int i = 0; i < 4; i++) {
			this->root->rangeQuery(e[i], f[i]);
		}
		int end[5];
		end[0] = 0;
		for (int i = 1; i <= 4; i++) {
			end[i] = end[i - 1] + f[i - 1].size();
		}
		features.reserve(end[4]);
		int pos;
#pragma omp parallel for num_threads(4) private(pos)
		for (int i = 0; i < 4; i++) {
			for (pos = end[i]; pos < end[i + 1]; pos++) {
				features[pos] = f[i][pos - end[i]];
			}
		}


	}

	// This function has been abandoned because of its bad performance;
	void ParallelKDTree::rangeQueryParallel1(Envelope& rect, vector<Feature>& features)
	{
		features.clear();
		if (!bbox.intersect(rect))
			return;

		queue<ParallelKDNode*> nodeQueue;
		nodeQueue.push(root);
		ParallelKDNode* currentNode;
		//!this->nodes[0]->bbox.intersect(rect)
		while (nodeQueue.size() != 4) {
			currentNode = nodeQueue.front();
			if (currentNode->nodes[0] != NULL && currentNode->nodes[0]->bbox.intersect(rect)) {
				nodeQueue.push(currentNode->nodes[0]);
			}
			if (currentNode->nodes[1] != NULL && currentNode->nodes[1]->bbox.intersect(rect)) {
				nodeQueue.push(currentNode->nodes[1]);
			}
			if (currentNode->nodes[0] == NULL && currentNode->nodes[1] == NULL) {
				break;
			}

			nodeQueue.pop();
		}


		if (nodeQueue.size() == 1) {
			currentNode->rangeQuery(rect, features);
			return;
		}
		else {
			vector<Feature>f[8];
			ParallelKDNode** tmp = new ParallelKDNode * [8];
			tmp[0] = tmp[1] = tmp[2] = tmp[3] = NULL;
			tmp[4] = tmp[5] = tmp[6] = tmp[7] = NULL;
			int count = nodeQueue.size();
			for (int i = 0; i < count; i++) {
				tmp[i] = nodeQueue.front();
				nodeQueue.pop();
			}

#pragma omp parallel for num_threads(count)
			for (int i = 0; i < count; i++) {
				tmp[i]->rangeQuery(rect, f[i]);
			}

			for (int i = 0; i < count; i++) {
				features.insert(features.end(), f[i].begin(), f[i].end());
			}

			delete[] tmp;

		}

	}

	bool ParallelKDTree::NNQuery(double x, double y, vector<Feature>& features)
	{
		if (!root || !(root->getEnvelope().contain(x, y)))
			return false;

		// filter step:
		ParallelKDSmallNode* samples = (ParallelKDSmallNode*)this->root->pointInLeafNode(x, y);
		if (samples->getFeatureNum() == 0) return false;

		vector <Feature> samplesFeature;
		samples->getAllFeaturesTo(samplesFeature);
		double minDist = samplesFeature[0].maxDistance2Envelope(x, y);;
		double tempMin;
		for (int i = 1; i < samplesFeature.size(); i++) {
			tempMin = samplesFeature[i].maxDistance2Envelope(x, y);
			if (tempMin < minDist) {
				minDist = tempMin;
			}
		}

		Envelope NNQueryRange(x - minDist, x + minDist, y - minDist, y + minDist);
		rangeQuery(NNQueryRange, features);

		return true;
	}

}

