#include "QuadTree.h"

namespace MapGeom {

	void QuadNode::split(size_t capacity)
	{
		for (int i = 0; i < 4; i++) {
			delete nodes[i];
			nodes[i] = NULL;
		}

		if (features.size() <= capacity) {
			return;
		}
		double minX = this->getEnvelope().getMinX();
		double maxX = this->getEnvelope().getMaxX();
		double midX = 0.5 * (minX + maxX);
		double minY = this->getEnvelope().getMinY();
		double maxY = this->getEnvelope().getMaxY();
		double midY = 0.5 * (minY + maxY);

		// [0]: NW (Left Up), [1]: SW (Left Down), [2]: SE (Right Down), [3]: NE (Right Up);
		nodes[0] = new QuadNode(Envelope(midX, maxX, midY, maxY));
		nodes[1] = new QuadNode(Envelope(midX, maxX, minY, midY));
		nodes[2] = new QuadNode(Envelope(minX, midX, minY, midY));
		nodes[3] = new QuadNode(Envelope(minX, midX, midY, maxY));

		for (int i = 0; i < 4; i++) {
			for (Feature x : features) {
				if (nodes[i]->bbox.intersect(x.getEnvelope())) nodes[i]->features.push_back(x);
			}
		}

		bool fewerSize = false;
		for (int i = 0; i < 4; i++) {
			if (nodes[i]->features.size() < features.size()) {
				fewerSize = true;
				break;
			}
		}
		if (!fewerSize) {
			cout << "There are " << features.size() << " points sharing the same coordination! ";
			cout << "Please increase the capacity!" << endl;
			for (int i = 0; i < 4; i++) {
				delete nodes[i];
				nodes[i] = NULL;
			}
			return;
		}

		// Delete features information in this node;
		features.swap(vector<Feature>());
		for (int i = 0; i < 4; i++) {
			nodes[i]->split(capacity);
		}
	}

	void QuadNode::countNode(int& interiorNum, int& leafNum)
	{
		if (isLeafNode()) {
			++leafNum;
		}
		else {
			++interiorNum;
			for (int i = 0; i < 4; ++i)
				nodes[i]->countNode(interiorNum, leafNum);
		}
	}

	int QuadNode::countHeight(int height)
	{
		++height;
		if (!isLeafNode()) {
			int cur = height;
			for (int i = 0; i < 4; ++i) {
				height = max(height, nodes[i]->countHeight(cur));
			}
		}
		return height;
	}

	void QuadNode::rangeQuery(Envelope& rect, vector<Feature>& features)
	{
		if (!bbox.intersect(rect))
			return;

		if (this->isLeafNode()) {
			features.insert(features.end(), this->features.begin(), this->features.end());
		}
		else {
			for (int i = 0; i < 4; i++) {
				this->nodes[i]->rangeQuery(rect, features);
			}
		}

	}

	// This function search a leaf node which has AT LEAST ONE feature with BACKTRACKING!
	QuadNode* QuadNode::pointInLeafNode(double x, double y)
	{
		if (this->isLeafNode() && this->features.empty()) {
			return NULL;
		}
		else if (this->isLeafNode() && !this->features.empty()) {
			return this;
		}
		else {
			QuadNode* ans = NULL;

			for (int i = 0; i < 4; i++) {
				if (this->nodes[i]->bbox.contain(x, y)) {
					ans = this->nodes[i]->pointInLeafNode(x, y);
					if (ans != NULL) {
						return ans;
					}
				}
			}

			for (int i = 0; i < 4; i++) {
				if (!this->nodes[i]->bbox.contain(x, y)) {
					ans = this->nodes[i]->pointInLeafNode(x, y);
					if (ans != NULL) {
						return ans;
					}
				}
			}

			return NULL;
		}
	}

	void QuadNode::draw()
	{
		if (isLeafNode()) {
			bbox.draw();
		}
		else {
			for (int i = 0; i < 4; ++i)
				nodes[i]->draw();
		}
	}

	bool QuadTree::constructQuadTree(vector<Feature>& features)
	{
		clock_t start_time = clock();
		clock_t end_time = clock();

		if (features.empty())
			return false;

		bbox = getFeaturesEnvelope(features);
		this->root = new QuadNode(bbox);
		this->root->add(features);
		this->root->split(capacity);

		cout << "Quad Tree building time: " << (end_time - start_time) / 1000.0 << "s" << endl;

		return true;
	}

	void QuadTree::countQuadNode(int& interiorNum, int& leafNum)
	{
		interiorNum = 0;
		leafNum = 0;
		if (root)
			root->countNode(interiorNum, leafNum);
	}

	void QuadTree::countHeight(int& height)
	{
		height = 0;
		if (root)
			height = root->countHeight(0);
	}

	// This function only return the candidate features stored into the second parameter;
	void QuadTree::rangeQuery(Envelope& rect, vector<Feature>& features)
	{
		features.clear();
		this->root->rangeQuery(rect, features);

	}


	bool QuadTree::NNQuery(double x, double y, vector<Feature>& features)
	{
		if (!root || !(root->getEnvelope().contain(x, y)))
			return false;

		// filter step:
		QuadNode* samples = this->root->pointInLeafNode(x, y);
		if (samples->getFeatureNum() == 0) return false;

		double minDist = samples->getFeature(0).maxDistance2Envelope(x, y);
		double tempMin;
		for (int i = 1; i < samples->getFeatureNum(); i++) {
			tempMin = samples->getFeature(i).maxDistance2Envelope(x, y);
			if (tempMin < minDist) {
				minDist = tempMin;
			}
		}

		Envelope NNQueryRange(x - minDist, x + minDist, y - minDist, y + minDist);
		rangeQuery(NNQueryRange, features);

		return true;
	}

	void QuadTree::draw()
	{
		if (root)
			root->draw();
	}

}
