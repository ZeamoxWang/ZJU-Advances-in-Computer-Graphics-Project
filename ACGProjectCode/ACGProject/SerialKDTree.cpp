#include "SerialKDTree.h"

namespace MapGeom {

	splitCandidate::splitCandidate(SerialKDSmallNode& smallRoot, Axis axis, int splitNum, int totalBin)
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

	double splitCandidate::getLeftPerimeter(const SerialKDSmallNode& node) const
	{
		return leftChild.intersectEnvelope(node.getEnvelope()).getPerimeter();
	}

	double splitCandidate::getRightPerimeter(const SerialKDSmallNode& node) const
	{
		return rightChild.intersectEnvelope(node.getEnvelope()).getPerimeter();
	}

	void specificAssignment(list<SerialKDNode*>& To, list<SerialKDSmallNode*>& From)
	{
		To.clear();
		for (list<SerialKDSmallNode*>::iterator it = From.begin(); it != From.end(); it++) {
			To.push_back(*it);
		}
	}

	void SerialKDNode::add(const Feature& f)
	{
		if (!f.getEnvelope().intersect(this->bbox)) return;

		vector <Geometry*> clipAns = f.getGeom()->clip(bbox);
		for (int i = 0; i < clipAns.size(); i++) {
			features.push_back(Feature(f.getName() + "_" + to_string(i), clipAns[i]));
			delete clipAns[i];
		}

	}

	void SerialKDNode::add(const vector<Feature>& fs)
	{
		for (int i = 0; i < fs.size(); i++) {
			add(fs[i]);
		}
		return;
	}

	void SerialKDNode::countNode(int& interiorNum, int& leafNum)
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

	int SerialKDNode::countHeight(int height)
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

	void SerialKDNode::draw()
	{
		if (isLeafNode()) {
			bbox.draw();
		}
		else {
			for (int i = 0; i < 2; i++) nodes[i]->draw();
		}
	}

	void SerialKDNode::rangeQuery(Envelope& rect, vector<Feature>& features)
	{
		if (this->isLeafNode()) {
			((SerialKDSmallNode*)this)->getAllFeaturesTo(features);
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
	SerialKDNode* SerialKDNode::pointInLeafNode(double x, double y)
	{
		if (this->isLeafNode() && ((SerialKDSmallNode*)this)->bitmask == 0) {
			return NULL;
		}
		else if (this->isLeafNode() && ((SerialKDSmallNode*)this)->bitmask != 0) {
			return this;
		}
		else {
			SerialKDNode* ans = NULL;

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

	SerialKDSmallNode::SerialKDSmallNode(const SerialKDSmallNode& smallNode, const splitCandidate& sc, Orientation o)
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

	double SerialKDSmallNode::getPerimeter() const
	{
		return bbox.getPerimeter();
	}

	void SerialKDSmallNode::getAllFeaturesTo(vector <Feature>& fs) const
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

	bool SerialKDTree::buildKDTree(vector<Feature>& features)
	{
		clock_t basic_time = clock();
		clock_t start_time = basic_time;
		clock_t end_time = clock();

		if (features.empty())
			return false;
		list<SerialKDNode*> activelist, nextlist;
		vector<SerialKDNode*> nodelist;
		list<SerialKDSmallNode*> smalllist;
		bbox = getFeaturesEnvelope(features);
		this->root = new SerialKDNode(bbox);
		this->root->add(features);
		activelist.push_back(root);

		end_time = clock();
		cout << "Serial preparation time: " << (end_time - start_time) / 1000.0 << "s" << endl;
		start_time = clock();

		// Large node stage;
		while (!activelist.empty()) {
			nodelist.insert(nodelist.end(), activelist.begin(), activelist.end());
			nextlist.clear();
			processLargeNodes(activelist, smalllist, nextlist);
			nextlist.swap(activelist);
		}

		end_time = clock();
		cout << "Serial large node stage time: " << (end_time - start_time) / 1000.0 << "s" << endl;
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
		cout << "Serial small node stage time: " << (end_time - start_time) / 1000.0 << "s" << endl;
		start_time = clock();

		cout << "Serial building time: " << (end_time - basic_time) / 1000.0 << "s" << endl;

	}

	void SerialKDTree::processLargeNodes(list<SerialKDNode*>& activelist, list<SerialKDSmallNode*>& smalllist, list<SerialKDNode*>& nextlist)
	{
		// Get the total envelope of each node in activelist;
		vector<Envelope> envelopes;
		for (SerialKDNode* x : activelist) {
			envelopes.push_back(getFeaturesEnvelope(x->getFeatures()));
		}

		// Split large nodes;
		Envelope ch1, ch2;
		double xmin, xmax, ymin, ymax, xminE, xmaxE, yminE, ymaxE, xrange, yrange;
		int i = 0;
		for (list<SerialKDNode*>::iterator it = activelist.begin(); it != activelist.end(); it++, i++) {
			xmin = (*it)->getEnvelope().getMinX();
			xmax = (*it)->getEnvelope().getMaxX();
			ymin = (*it)->getEnvelope().getMinY();
			ymax = (*it)->getEnvelope().getMaxY();

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

			(*it)->nodes[0] = new SerialKDNode(ch1);
			(*it)->nodes[1] = new SerialKDNode(ch2);
			(*it)->nodes[0]->add((*it)->features);
			(*it)->nodes[1]->add((*it)->features);
			(*it)->features.clear();
			for (int j = 0; j < 2; j++) {
				if ((*it)->nodes[j]->features.size() > threshold) {
					nextlist.push_back((*it)->nodes[j]);
				}
				else {
					SerialKDSmallNode* smallRoot = new SerialKDSmallNode(*(*it)->nodes[j]);
					smalllist.push_back(smallRoot);
					delete (*it)->nodes[j];
					(*it)->nodes[j] = smallRoot;
				}
			}

		}

	}

	void SerialKDTree::preprocessSmallNodes(list<SerialKDSmallNode*>& smalllist)
	{
		for (SerialKDSmallNode* smallRoot : smalllist) {
			for (int i = 1; i < splitCandidateNum; i++) {
				smallRoot->splitList.push_back(splitCandidate(*smallRoot, Axis::X, i, splitCandidateNum));
				smallRoot->splitList.push_back(splitCandidate(*smallRoot, Axis::Y, i, splitCandidateNum));
			}
		}
		return;
	}

	void SerialKDTree::processSmallNodes(list<SerialKDNode*>& activelist, list<SerialKDNode*>& nextlist)
	{
		for (SerialKDNode* x : activelist) {
			SerialKDSmallNode* smallNode = (SerialKDSmallNode*)x;
			unsigned long s = smallNode->bitmask;
			SerialKDSmallNode* r = smallNode->smallRoot;
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
					if (!x->getEnvelope().intersect(splitLine))
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
				smallNode->nodes[0] = new SerialKDSmallNode(*smallNode, r->splitList[minIndex], Orientation::Left);
				smallNode->nodes[1] = new SerialKDSmallNode(*smallNode, r->splitList[minIndex], Orientation::Right);
				nextlist.push_back(smallNode->nodes[0]);
				nextlist.push_back(smallNode->nodes[1]);
			}


		}
	}

	void SerialKDTree::test() {

	}

	void SerialKDTree::rangeQuery(Envelope& rect, vector<Feature>& features)
	{
		features.clear();
		if (!bbox.intersect(rect))
			return;
		this->root->rangeQuery(rect, features);

	}

	bool SerialKDTree::NNQuery(double x, double y, vector<Feature>& features)
	{
		if (!root || !(root->getEnvelope().contain(x, y)))
			return false;

		// filter step:
		SerialKDSmallNode* samples = (SerialKDSmallNode*)this->root->pointInLeafNode(x, y);
		if (samples->getFeatureNum() == 0) return false;

		vector <Feature> samplesFeature;
		samples->getAllFeaturesTo(samplesFeature);
		double minDist = samplesFeature[0].maxDistance2Envelope(x, y);
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

