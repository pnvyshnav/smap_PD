#pragma once

class Observable
{
public:
    typedef std::function<void(const Observable*)> Subscriber;

    Observable() : _id(newId++)
    {}

    void subscribe(Subscriber subscriber)
    {
        _subscribers.push_back(subscriber);
    }

    void publish() const
    {
        updateSubscribers();
    }

    const unsigned int observableId() const
    {
        return _id;
    }

protected:
    void updateSubscribers() const
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

unsigned int Observable::newId = 0;
