#include "splitCandidate.h"

namespace MapGeom {

	int splitCandidate::getLeftNum(const unsigned long& input) const
	{
		unsigned long code = input & left;
		return count1(code);
	}

	int splitCandidate::getRightNum(const unsigned long& input) const
	{
		unsigned long code = input & right;
		return count1(code);
	}

}