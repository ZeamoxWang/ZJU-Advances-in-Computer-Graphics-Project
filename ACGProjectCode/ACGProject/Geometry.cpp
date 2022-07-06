#include "Geometry.h"
#include "common.h"
#include <cmath>
#include <gl/freeglut.h> 

// Any LineString which has an envelope smaller than this value will be output as a Point;
#define MIN_DIFFERENCE 0.00000001

namespace MapGeom {

	typedef int OutCode;

	const int INSIDE = 0; // 0000
	const int LEFT = 1;   // 0001
	const int RIGHT = 2;  // 0010
	const int BOTTOM = 4; // 0100
	const int TOP = 8;    // 1000

	// Compute the bit code for a point (x, y) using the clip rectangle
	// bounded diagonally by (xmin, ymin), and (xmax, ymax)
	// ASSUME THAT xmax, xmin, ymax and ymin are global constants.
	/* NOT MY WORK */
	OutCode ComputeOutCode(double x, double y, double xmin, double xmax, double ymin, double ymax)
	{
		OutCode code;

		code = INSIDE;          // initialised as being inside of [[clip window]]

		if (x < xmin)           // to the left of clip window
			code |= LEFT;
		else if (x > xmax)      // to the right of clip window
			code |= RIGHT;
		if (y < ymin)           // below the clip window
			code |= BOTTOM;
		else if (y > ymax)      // above the clip window
			code |= TOP;

		return code;
	}

	// Envelope functions
	bool Envelope::contain(double x, double y) const
	{
		return x >= minX && x <= maxX && y >= minY && y <= maxY;
	}

	bool Envelope::contain(const Envelope& envelope) const
	{
		return this->contain(envelope.minX, envelope.minY) && this->contain(envelope.maxX, envelope.maxY);
	}

	bool Envelope::intersect(const Envelope& envelope) const
	{
		bool overlapX = (maxX >= envelope.minX) && (minX <= envelope.maxX);
		bool overlapY = (maxY >= envelope.minY) && (minY <= envelope.maxY);
		return overlapX && overlapY;
	}

	/* NOT MY WORK */
	Envelope Envelope::unionEnvelope(const Envelope& envelope) const
	{
		double xmin = min(minX, envelope.minX);
		double xmax = max(maxX, envelope.maxX);
		double ymin = min(minY, envelope.minY);
		double ymax = max(maxY, envelope.maxY);
		return Envelope(xmin, xmax, ymin, ymax);
	}

	/* NOT MY WORK */
	Envelope Envelope::intersectEnvelope(const Envelope& envelope) const
	{
		if (!this->intersect(envelope)) {
			cout << "Error when intercecting envelope!" << endl;
			return Envelope();
		}

		double xmin = max(minX, envelope.minX);
		double xmax = min(maxX, envelope.maxX);
		double ymin = max(minY, envelope.minY);
		double ymax = min(maxY, envelope.maxY);
		return Envelope(xmin, xmax, ymin, ymax);
	}

	/* NOT MY WORK */
	void Envelope::draw() const
	{
		glBegin(GL_LINE_STRIP);

		glVertex2d(minX, minY);
		glVertex2d(minX, maxY);
		glVertex2d(maxX, maxY);
		glVertex2d(maxX, minY);
		glVertex2d(minX, minY);

		glEnd();
	}
	
	// Point functions
	// Calculate the distance between point to point;
	double Point::distance(const Point* point) const
	{
		return sqrt((x - point->x) * (x - point->x) + (y - point->y) * (y - point->y));
	}

	// Calculate the distance between point to line;
	double Point::distance(const LineString* line) const
	{
		double mindist = line->getPointN(0).distance(this);
		for (size_t i = 0; i < line->numPoints() - 1; i++) {
			double dist = 0;
			double x1 = line->getPointN(i).getX();
			double y1 = line->getPointN(i).getY();
			double x2 = line->getPointN(i + 1).getX();
			double y2 = line->getPointN(i + 1).getY();
			// Calculate the distance between Point P(x, y) and Line [P1(x1, y1), P2(x2, y2)]
			double dotProduction = (x - x1) * (x2 - x1) + (y - y1) * (y2 - y1);
			double p1p2Length = line->getPointN(i + 1).distance(&line->getPointN(i));
			if (0 <= dotProduction && dotProduction < p1p2Length * p1p2Length) {
				// This point could be cast on the line segment;
				double pp1Length = line->getPointN(i).distance(this);
				dist = sqrt(pp1Length * pp1Length - dotProduction * dotProduction / p1p2Length / p1p2Length);
			}
			else {
				// This point cannot be cast on the line segment;
				dist = line->getPointN(i + 1).distance(this);
			}

			if (dist < mindist)
				mindist = dist;
		}
		return mindist;
	}

	bool Point::intersects(const Envelope& rect)  const
	{
		return (x >= rect.getMinX()) && (x <= rect.getMaxX()) && (y >= rect.getMinY()) && (y <= rect.getMaxY());
	}

	/* NOT MY WORK */
	void Point::draw()  const
	{
		glBegin(GL_POINTS);
		glVertex2d(x, y);
		glEnd();
	}

	vector<Geometry*> Point::clip(const Envelope& rect) const
	{
		vector <Geometry*> clipAns;
		if (rect.contain(this->x, this->y)) {
			Point* ans = new Point(*this);
			clipAns.push_back(ans);
		}
		return clipAns;
	}

	// Cohen¨CSutherland clipping algorithm clips a line from
	// P0 = (x0, y0) to P1 = (x1, y1) against a rectangle with 
	// diagonal from (xmin, ymin) to (xmax, ymax).
	Point* Point::cutPointWithEnvelope(const Point& anothor, const Envelope& rect) const
	{
		double x0 = this->getX(), y0 = this->getY();
		double x1 = anothor.getX(), y1 = anothor.getY();
		double xmin = rect.getMinX(), xmax = rect.getMaxX(), ymin = rect.getMinY(), ymax = rect.getMaxY();

		Point* ans = NULL;

		// compute outcodes for P0, P1, and whatever point lies outside the clip rectangle
		OutCode outcode0 = ComputeOutCode(x0, y0, xmin, xmax, ymin, ymax);
		OutCode outcode1 = ComputeOutCode(x1, y1, xmin, xmax, ymin, ymax);


		if (!(outcode0 | outcode1) || (outcode0 & outcode1)) {
			// bitwise OR is 0: both points inside window;
			// Or bitwise AND is not 0: both points share an outside zone (LEFT, RIGHT, TOP,
			// or BOTTOM), so both must be outside window;
			return ans;
		}
		else {
			// failed both tests, so calculate the line segment to clip
			// from an outside point to an intersection with clip edge
			double x, y;

			// At least one endpoint is outside the clip rectangle; pick it.
			OutCode outcodeOut = outcode0 ? outcode0 : outcode1;

			// Now find the intersection point;
			// use formulas:
			//   slope = (y1 - y0) / (x1 - x0)
			//   x = x0 + (1 / slope) * (ym - y0), where ym is ymin or ymax
			//   y = y0 + slope * (xm - x0), where xm is xmin or xmax
			// No need to worry about divide-by-zero because, in each case, the
			// outcode bit being tested guarantees the denominator is non-zero
			if (outcodeOut & TOP) {           // point is above the clip window
				x = x0 + (x1 - x0) * (ymax - y0) / (y1 - y0);
				y = ymax;
			}
			else if (outcodeOut & BOTTOM) { // point is below the clip window
				x = x0 + (x1 - x0) * (ymin - y0) / (y1 - y0);
				y = ymin;
			}
			else if (outcodeOut & RIGHT) {  // point is to the right of clip window
				y = y0 + (y1 - y0) * (xmax - x0) / (x1 - x0);
				x = xmax;
			}
			else if (outcodeOut & LEFT) {   // point is to the left of clip window
				y = y0 + (y1 - y0) * (xmin - x0) / (x1 - x0);
				x = xmin;
			}

			ans = new Point(x, y);
			return ans;
		}
	}

	
	// LineString functions
	/* NOT MY WORK */
	void LineString::constructEnvelope()
	{
		double minX, minY, maxX, maxY;
		maxX = minX = points[0].getX();
		maxY = minY = points[0].getY();
		for (size_t i = 1; i < points.size(); ++i) {
			maxX = max(maxX, points[i].getX());
			maxY = max(maxY, points[i].getY());
			minX = min(minX, points[i].getX());
			minY = min(minY, points[i].getY());
		}
		envelope = Envelope(minX, maxX, minY, maxY);
	}

	bool intersectTest(double x0, double y0, double x1, double y1, double xmin, double xmax, double ymin, double ymax)
	{
		// compute outcodes for P0, P1, and whatever point lies outside the clip rectangle
		OutCode outcode0 = ComputeOutCode(x0, y0, xmin, xmax, ymin, ymax);
		OutCode outcode1 = ComputeOutCode(x1, y1, xmin, xmax, ymin, ymax);
		bool accept = false;

		while (true) {
			if (!(outcode0 | outcode1)) {
				// bitwise OR is 0: both points inside window; trivially accept and exit loop
				accept = true;
				break;
			}
			else if (outcode0 & outcode1) {
				// bitwise AND is not 0: both points share an outside zone (LEFT, RIGHT, TOP,
				// or BOTTOM), so both must be outside window; exit loop (accept is false)
				break;
			}
			else {
				// failed both tests, so calculate the line segment to clip
				// from an outside point to an intersection with clip edge
				double x, y;

				// At least one endpoint is outside the clip rectangle; pick it.
				OutCode outcodeOut = outcode0 ? outcode0 : outcode1;

				// Now find the intersection point;
				// use formulas:
				//   slope = (y1 - y0) / (x1 - x0)
				//   x = x0 + (1 / slope) * (ym - y0), where ym is ymin or ymax
				//   y = y0 + slope * (xm - x0), where xm is xmin or xmax
				// No need to worry about divide-by-zero because, in each case, the
				// outcode bit being tested guarantees the denominator is non-zero
				if (outcodeOut & TOP) {           // point is above the clip window
					x = x0 + (x1 - x0) * (ymax - y0) / (y1 - y0);
					y = ymax;
				}
				else if (outcodeOut & BOTTOM) { // point is below the clip window
					x = x0 + (x1 - x0) * (ymin - y0) / (y1 - y0);
					y = ymin;
				}
				else if (outcodeOut & RIGHT) {  // point is to the right of clip window
					y = y0 + (y1 - y0) * (xmax - x0) / (x1 - x0);
					x = xmax;
				}
				else if (outcodeOut & LEFT) {   // point is to the left of clip window
					y = y0 + (y1 - y0) * (xmin - x0) / (x1 - x0);
					x = xmin;
				}

				// Now we move outside point to intersection point to clip
				// and get ready for next pass.
				if (outcodeOut == outcode0) {
					x0 = x;
					y0 = y;
					outcode0 = ComputeOutCode(x0, y0, xmin, xmax, ymin, ymax);
				}
				else {
					x1 = x;
					y1 = y;
					outcode1 = ComputeOutCode(x1, y1, xmin, xmax, ymin, ymax);
				}
			}
		}
		return accept;
	}

	/* NOT MY WORK */
	bool LineString::intersects(const Envelope& rect)  const
	{
		double xmin = rect.getMinX();
		double xmax = rect.getMaxX();
		double ymin = rect.getMinY();
		double ymax = rect.getMaxY();

		for (size_t i = 1; i < points.size(); ++i)
			if (intersectTest(points[i - 1].getX(), points[i - 1].getY(), points[i].getX(), points[i].getY(), xmin, xmax, ymin, ymax))
				return true;
		return false;
	}

	/* NOT MY WORK */
	void LineString::draw()  const
	{
		if (envelope.getWidth() < MIN_DIFFERENCE && envelope.getHeight() < MIN_DIFFERENCE) {
			points[0].draw();
		}
		else {
			glBegin(GL_LINE_STRIP);
			for (size_t i = 0; i < points.size(); ++i)
				glVertex2d(points[i].getX(), points[i].getY());
			glEnd();
		}

	}

	/* NOT MY WORK */
	void LineString::print() const
	{
		cout << "LineString(";
		for (size_t i = 0; i < points.size(); ++i) {
			if (i != 0)
				cout << ", ";
			cout << points[i].getX() << " " << points[i].getY();
		}
		cout << ")";
	}

	vector<Geometry*> LineString::clip(const Envelope& rect) const
	{
		vector <Geometry*> clipAns;
		vector <Point> tempPoints;
		bool thisPointIn, lastPointIn = true;
		for (int i = 0; i < points.size(); i++) {
			thisPointIn = rect.contain(points[i].getX(), points[i].getY());
			if (thisPointIn && lastPointIn) {
				tempPoints.push_back(points[i]);
			}
			else if (thisPointIn && !lastPointIn) {
				Point lastP = *points[i].cutPointWithEnvelope(points[i - 1], rect);
				tempPoints.push_back(lastP);
				tempPoints.push_back(points[i]);

			}
			else if (!thisPointIn && !tempPoints.empty()) {
				tempPoints.push_back(*points[i].cutPointWithEnvelope(tempPoints[tempPoints.size() - 1], rect));
				LineString* newLineString = new LineString(tempPoints);
				clipAns.push_back(newLineString);
				tempPoints.clear();
			}
			lastPointIn = thisPointIn;
		}

		if (tempPoints.size() != 0) {
			LineString* newLineString = new LineString(tempPoints);
			clipAns.push_back(newLineString);
		}

		return clipAns;
	}

	// Feature functions
	Envelope getFeaturesEnvelope(const vector<Feature> f)
	{
		Envelope envelope = f[0].getEnvelope();
		for (Feature x : f) {
			envelope = envelope.unionEnvelope(x.getEnvelope());
		}
		return envelope;
	}

	Envelope getFeaturesEnvelopeWithReduction(const vector<Feature> f)
	{
		Envelope envelope = f[0].getEnvelope();
		int n = f.size();
#pragma omp parallel num_threads(THREADS_NUM)
		{
			Envelope envelopePrivate = f[0].getEnvelope();
#pragma omp for nowait
			for (int i = 1; i < n; i++) {
				envelopePrivate = envelopePrivate.unionEnvelope(f[i].getEnvelope());
			}
#pragma omp critical
			envelope = envelope.unionEnvelope(envelopePrivate);
		}

		return envelope;
	}


}
