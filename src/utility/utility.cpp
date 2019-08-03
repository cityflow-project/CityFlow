#include "utility/utility.h"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace CityFlow {
	
	Point operator*(const Point &A, double k) {
		return {A.x * k, A.y * k};
	}

	Point operator-(const Point &A, const Point &B) {
		return {A.x - B.x, A.y - B.y};
	}

	Point operator+(const Point &A, const Point &B) {
		return {A.x + B.x, A.y + B.y};
	}
    Point operator-(const Point &A) {
	    return {-A.x, -A.y};
	}

    Point calcIntersectPoint(Point A, Point B, Point C, Point D) {
	    Point P = A;
	    Point Q = C;
	    Vector u = B - A;
	    Vector v = D - C;
	    return P + u * (crossMultiply(Q - P, v) / crossMultiply(u, v));
    }


    double crossMultiply(const Point &A, const Point &B) {
	    return A.x * B.y - A.y * B.x;
    }

    double dotMultiply(const Point &A, const Point &B) {
	    return A.x * B.x + A.y * B.y;
    }

	double calcAng(Point A, Point B) {
		double ang = A.ang() - B.ang();
		double pi = acos(-1);
		while(ang >= pi / 2)
			ang -= pi / 2;
		while(ang < 0)
			ang += pi / 2;
		return std::min(ang, pi - ang);
	}

    bool onSegment(Point A, Point B, Point P) {
	    double v1 = crossMultiply(B-A, P-A);
	    double v2 = dotMultiply(P-A, P-B);
	    return Point::sign(v1) == 0 && Point::sign(v2) <= 0;
    }

    Point Point::unit() {
	    double l = len();
	    return {x / l, y / l};
    }

    Point Point::normal() {
	    return {-y, x};
    }

    Point::Point(double x, double y):x(x),y(y) { }

    double Point::len() {
	    return sqrt(x * x + y * y);
    }

	double Point::ang() {
		return atan2(y, x);
	}

    int Point::sign(double x) {
        return (x + Point::eps > 0) - (x < Point::eps);
    }

	std::vector<int> generateRandomIndices(size_t n, std::mt19937 *rnd) {
		std::vector<int> randoms;
		randoms.reserve(n);
		for (size_t i = 0; i < n; ++i) {
			randoms.emplace_back(i);
		}
		std::shuffle(randoms.begin(), randoms.end(), *rnd);
		return randoms;
	}

    int getJsonMemberInt(const std::string &name, const rapidjson::Value &object) {
        assert(object.IsObject());
	    auto iter = object.FindMember(name.c_str());
	    if (iter == object.MemberEnd())
	        throw jsonMemberMiss(name);
        if (!iter->value.IsInt())
	        throw jsonTypeError(name, "int");
        return iter->value.GetInt();
    }

    double getJsonMemberDouble(const std::string &name, const rapidjson::Value &object) {
        assert(object.IsObject());
        auto iter = object.FindMember(name.c_str());
        if (iter == object.MemberEnd())
            throw jsonMemberMiss(name);
        if (!iter->value.IsDouble() && !iter->value.IsInt()) {
            throw jsonTypeError(name, "dobule");
        }
        return iter->value.GetDouble();
    }

    bool getJsonMemberBool(const std::string &name, const rapidjson::Value &object) {
        assert(object.IsObject());
        auto iter = object.FindMember(name.c_str());
        if (iter == object.MemberEnd())
            throw jsonMemberMiss(name);
        if (!iter->value.IsBool())
            throw jsonTypeError(name, "bool");
        return iter->value.GetBool();
    }

    std::string getJsonMemberString(const std::string &name, const rapidjson::Value &object) {
        assert(object.IsObject());
        auto iter = object.FindMember(name.c_str());
        if (iter == object.MemberEnd())
            throw jsonMemberMiss(name);
        if (!iter->value.IsString())
            throw jsonTypeError(name, "string");
        return iter->value.GetString();
    }

    const rapidjson::Value &getJsonMemberArray(const std::string &name, const rapidjson::Value &object) {
        assert(object.IsObject());
        auto iter = object.FindMember(name.c_str());
        if (iter == object.MemberEnd())
            throw jsonMemberMiss(name);
        if (!iter->value.IsArray())
            throw jsonTypeError(name, "array");
        return iter->value;
    }

    const rapidjson::Value &getJsonMemberObject(const std::string &name, const rapidjson::Value &object) {
        assert(object.IsObject());
        auto iter = object.FindMember(name.c_str());
        if (iter == object.MemberEnd())
            throw jsonMemberMiss(name);
        if (!iter->value.IsObject())
            throw jsonTypeError(name, "object");
        return iter->value;
    }

    int getJsonMemberInt(const std::string &name, const rapidjson::Value &object, int default_value) {
        assert(object.IsObject());
        auto iter = object.FindMember(name.c_str());
        if (iter == object.MemberEnd() || !iter->value.IsInt())
            return default_value;
        return iter->value.GetInt();
    }

    double getJsonMemberDouble(const std::string &name, const rapidjson::Value &object, double default_value) {
        assert(object.IsObject());
        auto iter = object.FindMember(name.c_str());
        if (iter == object.MemberEnd() || (!iter->value.IsDouble() && !iter->value.IsInt()))
            return default_value;
        return iter->value.GetDouble();
    }

    bool getJsonMemberBool(const std::string &name, const rapidjson::Value &object, bool default_value) {
        assert(object.IsObject());
        auto iter = object.FindMember(name.c_str());
        if (iter == object.MemberEnd() || !iter->value.IsBool())
            return default_value;
        return iter->value.GetBool();
    }
}