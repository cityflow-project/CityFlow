#ifndef CITYFLOW_TYPEDEF_H
#define CITYFLOW_TYPEDEF_H

#include "milo/dtoa_milo.h"
#include <vector>
#include <cmath>
#include <random>

namespace CityFlow {

	constexpr double eps = 1e-8;

	class Road;
 	class Lane;
	class Point {
	public:
		double x = 0.0;
		double y = 0.0;

		static constexpr double eps = 1e-8;
		static int sign(double x);

		Point() = default;
		Point(double x, double y);
		double len();
		Point normal();
		Point unit();
		double ang();
	};
	typedef Point Vector;

	Point operator*(const Point &A, double k);
	Point operator-(const Point &A, const Point &B);
	Point operator+(const Point &A, const Point &B);
	Point operator-(const Point &A);
	double crossMultiply(const Point &A, const Point &B);
	double dotMultiply(const Point &A, const Point &B);
	double calcAng(Point A, Point B);

	Point calcIntersectPoint(Point A, Point B, Point C, Point D);
	bool onSegment(Point A, Point B, Point P);

	struct ControlInfo {
		double speed = 0.0;
		double changingSpeed = 0.0;
		Lane * nextLane = nullptr;
		bool waitingForChangingLane = false;
		bool collision = false;
	};

	struct LaneInfo {
		Road *road;
		int LaneNumber;
		double pos;
	};

	struct FinalInfo {
		//TODO
	};

	inline double max2double(double x, double y) {
		return x > y ? x : y;
	}

	inline double min2double(double x, double y) {
		return x < y ? x : y;
	}
    
    inline std::string double2string(double x) {
		char ret[20];
		dtoa_milo(x, ret);
		std::string str(ret);
        return str;
    }

	std::vector<int> generateRandomIndices(size_t n, std::mt19937 *rnd); // size_t compile error


}

#endif //CITYFLOW_TYPEDEF_H
