#include "utility/utility.h"
#include <cmath>
#include <algorithm>
#include <iostream>

#include "rapidjson/filereadstream.h"
#include "rapidjson/filewritestream.h"
#include "rapidjson/cursorstreamwrapper.h"
#include "rapidjson/writer.h"
#include "rapidjson/document.h"
#include "rapidjson/error/en.h"

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

    bool readJsonFromFile(const std::string &filename, rapidjson::Document &document) {
        FILE *fp = fopen(filename.c_str(), "r");
        if (!fp) {
            return false;
        }
        char readBuffer[JSON_BUFFER_SIZE];
        rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
        rapidjson::CursorStreamWrapper<rapidjson::FileReadStream> csw(is);
        document.ParseStream(csw);
        if (document.HasParseError()) {
            std::cerr << "Json parsing error at line " << csw.GetLine() << std::endl;
            std::cerr << rapidjson::GetParseError_En(document.GetParseError());
            std::cerr << std::endl;
            throw JsonFormatError("Json parsing error");
            return false;
        }
        fclose(fp);
        return true;
    }

    bool writeJsonToFile(const std::string &filename, const rapidjson::Document &document) {
        FILE *fp = fopen(filename.c_str(), "w");
        if (!fp) {
            return false;
        }
        char writeBuffer[JSON_BUFFER_SIZE];
        rapidjson::FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
        rapidjson::Writer<rapidjson::FileWriteStream> writer(os);
        document.Accept(writer);
        fclose(fp);
        return true;
    }

    const rapidjson::Value &getJsonMemberValue(const std::string &name, const rapidjson::Value &object) {
        assert(object.IsObject());
        auto iter = object.FindMember(name.c_str());
        if (iter == object.MemberEnd())
            throw JsonMemberMiss(name);
        return iter->value;
    }


    const rapidjson::Value &
    getJsonMemberObject(const std::string &name, const rapidjson::Value &object) {
        const auto &value = getJsonMemberValue(name, object);
        if (!value.IsObject())
            throw JsonTypeError(name, "object");
        return value;
    }

    const rapidjson::Value &
    getJsonMemberArray(const std::string &name, const rapidjson::Value &object) {
        const auto &value = getJsonMemberValue(name, object);
        if (!value.IsArray())
            throw JsonTypeError(name, "array");
        return value;
    }

    template<>
    bool jsonConvertableTo<double>(const rapidjson::Value &value) {
        //We do not differentiate 123.0 and 123
        return value.IsNumber();
    }

}