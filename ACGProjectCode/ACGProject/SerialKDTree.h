#pragma once

#include "Geometry.h"
#include "splitCandidate.h"
#include "common.h"

namespace MapGeom {

	class SerialKDNode
	{
	protected:
		Envelope bbox;
		SerialKDNode* nodes[2];
		vector<Feature> features;
		SerialKDNode() {};

	public:
		friend class SerialKDTree;

		SerialKDNode(Envelope& box) {
			bbox = box;
			nodes[0] = nodes[1] = NULL;
		}

		~SerialKDNode() {
			for (int i = 0; i < 2; ++i) {
				if (nodes[i] != NULL) {
					delete nodes[i];
					nodes[i] = NULL;
				}
			}
		}

		bool isLeafNode() { return nodes[0] == NULL; }

		const Envelope& getEnvelope() const { return bbox; }

		SerialKDNode* getChildNode(size_t i) { return i < 2 ? nodes[i] : NULL; }

		virtual size_t    getFeatureNum() const { return features.size(); }

		virtual Feature& getFeature(size_t i) { return features[i]; }

		vector<Feature>& getFeatures() { return features; }

		void add(const Feature& f);

		void add(const vector<Feature>& fs);

		void countNode(int& interiorNum, int& leafNum);

		int countHeight(int height);

		void draw();

		void rangeQuery(Envelope& rect, vector<Feature>& features);

		SerialKDNode* pointInLeafNode(double x, double y);

	};

	class SerialKDSmallNode : public SerialKDNode {

	private:
		SerialKDSmallNode* smallRoot;
		vector <splitCandidate> splitList;
		unsigned long bitmask;
	public:
		friend SerialKDTree;
		friend SerialKDNode;

		SerialKDSmallNode(const SerialKDNode& origin) :SerialKDNode(origin) {
			// Move twice in order to get avoid of the auto optimization of compiler when feature num = 32;
			bitmask = 1 << (origin.getFeatureNum() >> 1);
			bitmask = bitmask << ((origin.getFeatureNum() + 1) >> 1);

			bitmask -= 1;
			smallRoot = this;
			nodes[0] = nodes[1] = NULL;
		}

		SerialKDSmallNode(const SerialKDSmallNode& smallNode, const splitCandidate& sc, Orientation o);

		~SerialKDSmallNode() {
			vector <splitCandidate>().swap(splitList);
			for (int i = 0; i < 2; ++i) {
				if (nodes[i] != NULL) {
					delete nodes[i];
					nodes[i] = NULL;
				}
			}
		}

		double getPerimeter() const;

		virtual size_t    getFeatureNum() const { return count1(smallRoot->bitmask & bitmask); };

		void getAllFeaturesTo(vector <Feature>& features) const;

	};

	void specificAssignment(list<SerialKDNode*>& To, list<SerialKDSmallNode*>& From);

	class SerialKDTree {
	private:
		SerialKDNode* root;
		Envelope bbox;
		double Ce;
		const int threshold = 32;
		const int splitCandidateNum = 32;
		double Cts = 5;

	public:
		SerialKDTree(double Ce = 0.45) : root(NULL), Ce(Ce) {};
		~SerialKDTree() {
			delete root;
		}

		void setCts(double c) {
			Cts = c;
		}

		void setCe(double c) {
			Ce = c;
		}

		int getSplitCandidateNum() const { return splitCandidateNum; }

		const Envelope& getEnvelope() const { return bbox; }

		bool buildKDTree(vector<Feature>& features);

		void processLargeNodes(list<SerialKDNode*>& activelist, list<SerialKDSmallNode*>& smalllist, list<SerialKDNode*>& nextlist);

		void preprocessSmallNodes(list<SerialKDSmallNode*>& smalllist);

		void processSmallNodes(list<SerialKDNode*>& activelist, list<SerialKDNode*>& nextlist);

		void countQuadNode(int& interiorNum, int& leafNum);

		void countHeight(int& height);

		void rangeQuery(Envelope& rect, vector<Feature>& features);

		bool NNQuery(double x, double y, vector<Feature>& features);

		SerialKDNode* pointInLeafNode(double x, double y) { return root->pointInLeafNode(x, y); }

		void draw() { root->draw(); };

		void test();
	};
}