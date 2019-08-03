#ifndef CITYFLOW_TYPEDEF_H
#define CITYFLOW_TYPEDEF_H
#include "rapidjson/document.h"

#include "dtoa_milo.h"
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

    class jsonFormatError: public std::runtime_error {
    public:
        jsonFormatError(const std::string &info) : std::runtime_error(info){}
    };

    class jsonMemberMiss: public jsonFormatError  {
    private:
        std::string info;
    public:
        explicit jsonMemberMiss(const std::string &name) :
            jsonFormatError(name + " is required but missing in json file"){}
    };

    class jsonTypeError: public jsonFormatError  {
    private:
        std::string info;
    public:
        jsonTypeError(const std::string &name, const std::string &type) :
            jsonFormatError(name + ": expected type " + type){}
    };
	int getJsonMemberInt(const std::string &name, const rapidjson::Value &object);
	double getJsonMemberDouble(const std::string &name, const rapidjson::Value &object);
    bool getJsonMemberBool(const std::string &name, const rapidjson::Value &object);
    std::string getJsonMemberString(const std::string &name, const rapidjson::Value &object);

    int getJsonMemberInt(const std::string &name, const rapidjson::Value &object, int default_value);
    double getJsonMemberDouble(const std::string &name, const rapidjson::Value &object, double default_value);
    bool getJsonMemberBool(const std::string &name, const rapidjson::Value &object, bool default_value);

    const rapidjson::Value &getJsonMemberArray(const std::string &name, const rapidjson::Value &object);
    const rapidjson::Value &getJsonMemberObject(const std::string &name, const rapidjson::Value &object);

}

#endif //CITYFLOW_TYPEDEF_H
