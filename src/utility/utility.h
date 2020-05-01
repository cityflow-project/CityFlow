#ifndef CITYFLOW_TYPEDEF_H
#define CITYFLOW_TYPEDEF_H

#include "rapidjson/document.h"
#include "dtoa_milo.h"

#include <vector>
#include <cmath>
#include <random>
#include <typeinfo>
#include <stdexcept>

namespace CityFlow {

    constexpr double eps = 1e-8;
    constexpr size_t JSON_BUFFER_SIZE = 65536;

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
        char ret[30];
        dtoa_milo(x, ret);
        std::string str(ret);
        return str;
    }

    std::vector<int> generateRandomIndices(size_t n, std::mt19937 *rnd); // size_t compile error

    bool readJsonFromFile(const std::string &filename, rapidjson::Document &document);
    bool writeJsonToFile(const std::string &filename, const rapidjson::Document &document);

    class JsonFormatError: public std::runtime_error {
    public:
        explicit JsonFormatError(const std::string &info) : std::runtime_error(info){}
    };

    class JsonMemberMiss: public JsonFormatError  {
    private:
        std::string info;
    public:
        explicit JsonMemberMiss(const std::string &name) :
                JsonFormatError(name + " is required but missing in json file"){}
    };

    class JsonTypeError: public JsonFormatError  {
    private:
        std::string info;
    public:
        JsonTypeError(const std::string &name, const char* type) :
                JsonFormatError(name + ": expected type " + type){}

        JsonTypeError(const std::string &name, const std::string &type) :
                JsonTypeError(name, type.c_str()){}
    };

    const rapidjson::Value &getJsonMemberValue(const std::string &name, const rapidjson::Value &object);

    template<typename T>
    bool jsonConvertableTo(const rapidjson::Value &value) {
        return value.Is<T>();
    };

    template<>
    bool jsonConvertableTo<double>(const rapidjson::Value &value);

    template<typename T>
    T getJsonMember(const std::string &name, const rapidjson::Value &object) {
        assert(object.IsObject());
        const auto &value = getJsonMemberValue(name, object);
        if (!jsonConvertableTo<T>(value))
            throw JsonTypeError(name, typeid(T).name());
        return value.Get<T>();
    }

    template<typename T>
    T getJsonMember(const std::string &name, const rapidjson::Value &object, const T &default_value) {
        assert(object.IsObject());
        auto iter = object.FindMember(name.c_str());
        if (iter == object.MemberEnd() || !jsonConvertableTo<T>(iter->value))
            return default_value;
        return iter->value.Get<T>();
    }

    const rapidjson::Value &
    getJsonMemberObject(const std::string &name, const rapidjson::Value &object);

    const rapidjson::Value &
    getJsonMemberArray(const std::string &name, const rapidjson::Value &object);
}

#endif //CITYFLOW_TYPEDEF_H
