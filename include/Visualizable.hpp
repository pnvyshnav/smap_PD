#pragma once

class Visualizer;
class Visualizable
{
public:
    typedef std::function<void(const Visualizable*)> Subscriber;

    Visualizable() : _id(newId++)
    {}

    void subscribe(Subscriber subscriber)
    {
        _subscribers.push_back(subscriber);
    }

    void publish() const
    {
        updateVisualization();
    }

    const unsigned int visualizationId() const
    {
        return _id;
    }

protected:
    void updateVisualization() const
    {
        for (auto &subscriber : _subscribers)
            subscriber(this);
    }

private:
    std::vector<Subscriber> _subscribers;
    __attribute__((weak))
    static unsigned int newId;
    const unsigned int _id;

};

unsigned int Visualizable::newId = 0;
