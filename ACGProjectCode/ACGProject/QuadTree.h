#pragma once

#include "Geometry.h"
#include "common.h"

namespace MapGeom {

	class QuadNode {
	private:
		Envelope bbox;
		QuadNode* nodes[4]; // [0]: NW (Left Up), [1]: SW (Left Down), [2]: SE (Right Down), [3]: NE (Right Up);
		vector<Feature> features;

		QuadNode() {};

	public:
		QuadNode(Envelope& box) {
			bbox = box;
			nodes[0] = nodes[1] = nodes[2] = nodes[3] = NULL;
		}

		~QuadNode() {
			for (int i = 0; i < 4; ++i) {
				delete nodes[i];
			}
		}

		bool isLeafNode() { return nodes[0] == NULL; }

		const Envelope& getEnvelope() { return bbox; }

		QuadNode* getChildNode(size_t i) { return i < 4 ? nodes[i] : NULL; }

		size_t    getFeatureNum() { return features.size(); }

		Feature& getFeature(size_t i) { return features[i]; }

		void add(Feature& f) { features.push_back(f); }

		void add(vector<Feature>& fs) { features.insert(features.begin(), fs.begin(), fs.end()); }

		void countNode(int& interiorNum, int& leafNum);

		int countHeight(int height);

		void draw();

		// split the node into four child nodes, assign each feature to its overlaped child node(s),
		// clear feature vector, and split child node(s) if its number of features is larger than capacity;
		void split(size_t capacity);

		void rangeQuery(Envelope& rect, vector<Feature>& features);

		QuadNode* pointInLeafNode(double x, double y);
	};


	class QuadTree {
	private:
		QuadNode* root;
		size_t capacity;
		Envelope bbox;

	public:
		QuadTree() : root(NULL), capacity(5) {}
		QuadTree(size_t cap) : root(NULL), capacity(cap) {}
		~QuadTree() {
			delete root;
			root = NULL;
		}

		void setCapacity(int capacity) { this->capacity = capacity; }
		int  getCapacity() const { return capacity; }

		const Envelope& getEnvelope() const { return bbox; }

		bool constructQuadTree(vector<Feature>& features);

		void countQuadNode(int& interiorNum, int& leafNum);

		void countHeight(int& height);

		void rangeQuery(Envelope& rect, vector<Feature>& features);

		bool NNQuery(double x, double y, vector<Feature>& features);

		QuadNode* pointInLeafNode(double x, double y) { return root->pointInLeafNode(x, y); }

		void draw();
	};

}