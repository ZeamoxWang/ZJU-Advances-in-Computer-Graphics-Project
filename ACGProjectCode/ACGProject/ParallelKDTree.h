#pragma once

#include "Geometry.h"
#include "splitCandidate.h"
#include "common.h"

namespace MapGeom {

	class ParallelKDNode
	{
	protected:
		Envelope bbox;
		ParallelKDNode* nodes[2];
		vector<Feature> features;
		ParallelKDNode() {};

	public:
		friend class ParallelKDTree;

		ParallelKDNode(Envelope& box) {
			bbox = box;
			nodes[0] = nodes[1] = NULL;
		}

		~ParallelKDNode() {
			for (int i = 0; i < 2; ++i) {
				if (nodes[i] != NULL) {
					delete nodes[i];
					nodes[i] = NULL;
				}
			}
		}

		bool isLeafNode() { return nodes[0] == NULL; }

		const Envelope& getEnvelope() const { return bbox; }

		ParallelKDNode* getChildNode(size_t i) { return i < 2 ? nodes[i] : NULL; }

		virtual size_t    getFeatureNum() const { return features.size(); }

		virtual Feature& getFeature(size_t i) { return features[i]; }

		vector<Feature>& getFeatures() { return features; }

		void add(const Feature& f);

		void add(const vector<Feature>& fs);

		void parallelAdd(const vector<Feature>& fs);

		void countNode(int& interiorNum, int& leafNum);

		int countHeight(int height);

		void draw();

		void rangeQuery(Envelope& rect, vector<Feature>& features);

		ParallelKDNode* pointInLeafNode(double x, double y);

	};

	class ParallelKDSmallNode : public ParallelKDNode {

	private:
		ParallelKDSmallNode* smallRoot;
		vector <splitCandidate> splitList;
		unsigned long bitmask;
	public:
		friend ParallelKDTree;
		friend ParallelKDNode;

		ParallelKDSmallNode(const ParallelKDNode& origin) :ParallelKDNode(origin) {
			// Move twice in order to get avoid of the auto optimization of compiler when feature num = 32;
			bitmask = 1 << (origin.getFeatureNum() >> 1);
			bitmask = bitmask << ((origin.getFeatureNum() + 1) >> 1);
			bitmask -= 1;
			smallRoot = this;
			nodes[0] = nodes[1] = NULL;
		}

		ParallelKDSmallNode(const ParallelKDSmallNode& smallNode, const splitCandidate& sc, Orientation o);

		~ParallelKDSmallNode() {
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

	void specificAssignment(vector<ParallelKDNode*>& To, vector<ParallelKDSmallNode*>& From);

	class ParallelKDTree {
	private:
		ParallelKDNode* root;
		Envelope bbox;
		double Ce;
		const int threshold = 32;
		const int splitCandidateNum = 32;
		double Cts = 5;

	public:
		ParallelKDTree(double Ce = 0.45) : root(NULL), Ce(Ce) {};

		~ParallelKDTree() {
			delete root;
		}

		void setCts(double c) {
			Cts = c;
		}

		void setCe(double c) {
			Ce = c;
		}

		int getsplitCandidateNum() const { return splitCandidateNum; }

		const Envelope& getEnvelope() const { return bbox; }

		bool buildKDTree(vector<Feature>& features);

		void processLargeNodes(vector<ParallelKDNode*>& activelist, vector<ParallelKDSmallNode*>& smalllist, vector<ParallelKDNode*>& nextlist);

		void processLargeNodesWithoutSynchronize(vector<ParallelKDNode*>& activelist, vector<ParallelKDSmallNode*>& smalllist, vector<ParallelKDNode*>& nextlist);

		void preprocessSmallNodes(vector<ParallelKDSmallNode*>& smalllist);

		void processSmallNodes(vector<ParallelKDNode*>& activelist, vector<ParallelKDNode*>& nextlist);

		void rangeQueryParallel1(Envelope& rect, vector<Feature>& features);

		void rangeQueryParallel2(Envelope& rect, vector<Feature>& features);

		void rangeQuery(Envelope& rect, vector<Feature>& features);

		bool NNQuery(double x, double y, vector<Feature>& features);

		ParallelKDNode* pointInLeafNode(double x, double y) { return root->pointInLeafNode(x, y); }

		void draw() { root->draw(); };

		void test();
	};


}




