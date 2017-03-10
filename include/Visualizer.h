#pragma once

#include "TrueMap.h"
#include "Sensor.h"
#include "FakeRobot.hpp"


class Visualizer
{
    friend class BeliefMapDrawer;
public:
    Visualizer(TrueMap *trueMap, BeliefMap *beliefMap, FakeRobot<> *robot);

    virtual ~Visualizer();

    void render();

    void update();
};
