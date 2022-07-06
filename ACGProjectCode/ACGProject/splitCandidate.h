#pragma once
#include "Geometry.h"

namespace MapGeom {

	class SerialKDSmallNode;
	class ParallelKDSmallNode;

	int inline count1(unsigned long code)
	{
		int count = 0;
		while (code != 0) {
			code = code & (code - 1);
			count++;
		}
		return count;
	}

	class splitCandidate {
	private:
		Envelope leftChild, rightChild;
		unsigned long left, right;
	public:
		splitCandidate() { };
		splitCandidate(SerialKDSmallNode& smallRoot, Axis axis, int splitNum, int totalBin);
		splitCandidate(ParallelKDSmallNode& smallRoot, Axis axis, int splitNum, int totalBin);

		double getLeftPerimeter(const SerialKDSmallNode& node) const;
		double getRightPerimeter(const SerialKDSmallNode& node) const;

		double getLeftPerimeter(const ParallelKDSmallNode& node) const;
		double getRightPerimeter(const ParallelKDSmallNode& node) const;

		int getLeftNum(const unsigned long& input) const;
		int getRightNum(const unsigned long& input) const;

		unsigned long getLeftCode(const unsigned long& input) const { return input & left; };
		unsigned long getRightCode(const unsigned long& input) const { return input & right; };

		Envelope getLeftEnvelope() const { return leftChild; };
		Envelope getRightEnvelope() const { return rightChild; };
	};
}