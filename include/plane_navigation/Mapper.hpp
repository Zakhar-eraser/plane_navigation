#ifndef MAPPER_TOOL
#define MAPPER_TOOL

#include <yaml-cpp/yaml.h>
#include <map>
#include "Segment.hpp"
#include <thread>

class Contour;

class LinkedPoint
{
 private:
    float x;
    float y;
    LinkedPoint *pnextPoint;
    LinkedPoint *psymmetricXPoint;
    LinkedPoint *psymmetricYPoint;
    std::vector<LinkedPoint*> psameX;
    std::vector<LinkedPoint*> psameY;
    std::vector<LinkedPoint*> psymmetricXSegment;
    std::vector<LinkedPoint*> psymmetricYSegment;
    std::vector<LinkedPoint*> psameAngle;
 public:
    LinkedPoint(float x, float y);
    LinkedPoint(Position pos);
    void Set(Position pos);
    void SetX(float x);
    void SetY(float y);
    Position Get();
    float GetX();
    float GetY();
 friend class Contour;
};

class Contour
{
 private:
   LinkedPoint *firstPoint;
   LinkedPoint *lastPoint;
   void UpdateReferences();
 public:
   Contour();
   Contour(Position firstPoint);
   ~Contour();

   void AddPoint(Position pos);
   void RemovePoint(Position pos);
   LinkedPoint *forEach(LinkedPoint *(*op)(LinkedPoint*), bool (*exitCondition)(LinkedPoint*));
};

class Mapper
{
 private:
    Contour *contour;
    float measureStep;
    std::vector<Position> measuredPoints;
    std::thread calibrationThread;
    void MeasurementLoop();
    void Calibrate();
 public:
    Mapper();
    void ReadSegmentNotation();
    void StartMeasurement();
    void StopMeasurement();
};
#endif