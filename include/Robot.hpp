#pragma once

#include <vector>
#include <functional>

#include "Observation.hpp"

class Robot
{
public:
    typedef void (*ObservationHandler)(const Observation &);

    virtual void run() = 0;
    virtual void stop() = 0;

    void registerObserver(ObservationHandler handler)
    {
        _observers.push_back(handler);
    }

protected:
    void publishObservation(const Observation &observation)
    {
        for (auto &handler : _observers)
            (*handler)(observation);
    }

private:
    std::vector<ObservationHandler> _observers;
};
