#pragma once

#include "common.h"
using namespace std;

namespace MapGeom {

	enum class Axis { X, Y };

	enum class Orientation { Left, Right };

	class Point;
	class LineString;

	class Envelope {
	private:
		double minX;
		double minY;
		double maxX;
		double maxY;

	public:
		Envelope() : minX(0), minY(0), maxX(0), maxY(0) {}
		Envelope(double minX, double maxX, double minY, double maxY) : minX(minX), minY(minY), maxX(maxX), maxY(maxY) {}

		double getMinX() const { return minX; }
		double getMinY() const { return minY; }
		double getMaxX() const { return maxX; }
		double getMaxY() const { return maxY; }

		double getWidth()  const { return maxX - minX; }
		double getHeight() const { return maxY - minY; }
		double getPerimeter() const { return 2 * (getWidth() + getHeight()); }

		bool contain(double x, double y) const;

		void draw() const;

		void print() const { cout << "Envelope( " << minX << " " << maxX << " " << minY << " " << maxY << ") "; }

		bool operator == (const Envelope& t1) const { return (minX == t1.minX && minY == t1.minY && maxX == t1.maxX && maxY == t1.maxY); }
		bool operator != (const Envelope& t1) const { return !(*this == t1); }

		bool contain(const Envelope& envelope) const;
		bool intersect(const Envelope& envelope) const;
		Envelope unionEnvelope(const Envelope& envelope) const;
		Envelope intersectEnvelope(const Envelope& envelope) const;

	};


	/*
	 * Geometry hierarchy
	 */
	class Geometry {
	protected:
		Envelope envelope;

	public:
		Geometry() {}
		virtual ~Geometry() {}

		const Envelope& getEnvelope() const { return envelope; }

		virtual void     constructEnvelope() = 0;
		virtual double   distance(const Geometry* geom)   const { return geom->distance(this); } // Euclidean distance
		virtual double   distance(const Point* point)     const = 0;
		virtual double   distance(const LineString* line) const = 0;
		virtual bool     intersects(const Envelope& rect) const = 0;
		virtual void	 draw()                           const = 0;
		virtual void     print()                          const = 0;

		virtual bool isPoint() const = 0;
		virtual vector<Geometry*> clip(const Envelope& rect) const = 0;

	};

	class Point : public Geometry {
	private:
		double x;
		double y;

	public:
		Point() : x(0), y(0) {}
		Point(double x, double y) : x(x), y(y) { constructEnvelope(); }
		virtual ~Point() {}

		double getX() const { return x; }
		double getY() const { return y; }

		virtual void constructEnvelope() { envelope = Envelope(x, x, y, y); }

		// Euclidean distance
		virtual double distance(const Point* point) const;
		virtual double distance(const LineString* line) const;

		// intersection test with the envelope for range query
		virtual bool intersects(const Envelope& rect)  const;

		virtual void draw()  const;

		virtual void print() const { cout << "Point(" << x << " " << y << ")"; }

		virtual vector<Geometry*> clip(const Envelope& rect) const;

		Point* cutPointWithEnvelope(const Point& anothor, const Envelope& rect) const;

		virtual bool isPoint() const { return true; };
	};

	class LineString :public Geometry {
	private:
		vector<Point> points;

	public:
		LineString() {}
		LineString(vector<Point>& pts) : points(pts) { constructEnvelope(); }
		virtual ~LineString() {}

		size_t numPoints()         const { return points.size(); }
		Point  getStartPoint()     const { return points.front(); }
		Point  getEndPoint()       const { return points.back(); }
		Point  getPointN(size_t n) const { return points[n]; }

		virtual void constructEnvelope();

		// Euclidean distance
		virtual double distance(const Point* point) const { return point->distance(this); }
		virtual double distance(const LineString* line) const { return 0; }

		// intersection test with the envelope for range query
		virtual bool intersects(const Envelope& rect)  const;

		virtual void draw()  const;

		virtual void print() const;

		virtual vector<Geometry*> clip(const Envelope& rect) const;

		virtual bool isPoint() const { return false; }
	};

	class Feature {
	private:
		string name;
		Geometry* geom;
		Envelope envelope;

	public:
		Feature() : geom(NULL) {}
		Feature(string name, Geometry* g) : name(name) {
			// deep copy;
			if (g->isPoint()) {
				geom = new Point(*(Point*)g);
			}
			else {
				geom = new LineString(*(LineString*)g);
			}
			envelope = geom->getEnvelope();
		}

		Feature(const Feature& f) {
			name = f.name;
			envelope = f.envelope;
			// deep copy;
			if (f.geom->isPoint()) {
				geom = new Point(*(Point*)f.geom);
			}
			else {
				geom = new LineString(*(LineString*)f.geom);
			}
		}

		~Feature() {
			if (geom != NULL) {
				delete geom;
				geom = NULL;
			}
		}

		Feature& operator=(const Feature& rhs) {
			if (this != &rhs) {
				name = rhs.name;
				envelope = rhs.envelope;
				// deep copy;
				if (rhs.geom->isPoint()) {
					geom = new Point(*(Point*)rhs.geom);
				}
				else {
					geom = new LineString(*(LineString*)rhs.geom);
				}
			}
			return *this;
		}

		const string& getName() const { return name; }

		const Geometry* getGeom() const { return geom; };

		const Envelope& getEnvelope() const { return envelope; }

		double maxDistance2Envelope(double x, double y) const {
			const Envelope& e = geom->getEnvelope();
			double x1 = e.getMinX();
			double y1 = e.getMinY();
			double x2 = e.getMaxX();
			double y2 = e.getMaxY();

			double d1 = sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
			double d2 = sqrt((x - x1) * (x - x1) + (y - y2) * (y - y2));
			double d3 = sqrt((x - x2) * (x - x2) + (y - y1) * (y - y1));
			double d4 = sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2));

			return max(max(d1, d2), max(d3, d4));
		}

		double distance(double x, double y) const {
			Point point(x, y);
			return geom->distance(&point);
		}

		void print() const {
			cout << "Feature: " << name << " ";
			geom->print();
		}

		void draw() const {
			if (geom)
				geom->draw();
		}

	};

	Envelope getFeaturesEnvelope(const vector<Feature> f);

	Envelope getFeaturesEnvelopeWithReduction(const vector<Feature> f);

}
