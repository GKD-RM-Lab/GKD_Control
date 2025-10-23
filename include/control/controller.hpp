#pragma once

#include <list>

class Controller {
public:
    float out = 0.f;
    virtual void set(float x) = 0;
    virtual ~Controller() = default;
};

template<typename T>
concept ControllerBase = std::is_base_of_v<Controller, T>;

class ControllerList : public Controller {
private:
    template<typename T>
    static Controller* RealCopy(Controller* v) {
        return static_cast<Controller *>(new T(*static_cast<T*>(v)));
    }
    std::list<Controller *> list;
    std::list<Controller *(*)(Controller *)> copy_list;

    void selfCopy();
public:
    ~ControllerList() override;

    ControllerList() = default;

    ControllerList(const ControllerList &ctrl);

    ControllerList& operator=(const ControllerList &ctrl);

    ControllerList(ControllerList &&ctrl) noexcept :
            list(std::move(ctrl.list)), copy_list(std::move(ctrl.copy_list)) {
        out = ctrl.out;
    }

    template<ControllerBase T> requires (!std::is_same_v<T, ControllerList>)
    ControllerList(const T &ctrl) {
        list.emplace_back(new T(ctrl));
        copy_list.push_back(RealCopy<T>);
    }

    ControllerList(const ControllerList &left, const ControllerList &right);

    ControllerList(ControllerList &&left, ControllerList &&right);

    void set(float v) override;
};

ControllerList operator>>(const ControllerList &c1, const ControllerList &c2);

ControllerList operator>>(ControllerList &&c1, ControllerList &&c2);

ControllerList operator>>(ControllerList &&c1, const ControllerList &c2);

ControllerList operator>>(const ControllerList &c1, ControllerList &&c2);

float operator>>(float v, Controller& c);
