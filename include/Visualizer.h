#pragma once

#include "TrueMap.h"
#include "Sensor.h"
#include "FakeRobot.hpp"

typedef BeliefMap MapType;

class Visualizer
{
public:
    Visualizer(TrueMap *trueMap, MapType *map, FakeRobot<> *robot,
               bool gymMode = false, int skipFrame = 1, bool egoCentric = true);
    virtual ~Visualizer();

    void render();
    void update();

    void setEpisode(unsigned int episode);
    void setVelocity(float velocity);
    void setAngularVelocity(float angularVelocity);
    void setObservation(std::vector<float> observation);
};
