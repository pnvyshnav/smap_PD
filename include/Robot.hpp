#pragma once

#include <vector>
#include <functional>

#include "Observation.hpp"

/**
 * Represents a robot that constantly generates observations
 * while running.
 */
class Robot
{
public:
    typedef std::function<void(const Observation &)> ObservationHandler;

    virtual void run() = 0;
    virtual void stop() = 0;

    void registerObserver(ObservationHandler handler)
    {
        _observers.push_back(handler);
    }

protected:
    /**
     * Updates all subscribed ObservationHandlers with the
     * provided observation.
     * @param observation The observation.
     */
    void publishObservation(const Observation &observation)
    {
        for (auto &handler : _observers)
            handler(observation);
    }

private:
    std::vector<ObservationHandler> _observers;
};
