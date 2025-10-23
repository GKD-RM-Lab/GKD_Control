# Hardware_manager
*Hardware_manager*的实现基于*ListBase*和*Callback*，用于管理机器人的*IO*设备
## 规范
*Hardware_manager*应当管理所有与外界交互的*IO*(~~应当更名为*IO_Manager*~~)，例如*can*、*serial*、*socket*，而不应该是挂载在*can*上的motor或者挂载在*serial*上的imu

## 构造器
```cpp
Hardware::Hardware_manager<IO1, IO2, ...> hardware(io1, io2, ...);
```

*IO1, IO2, ...* 是你要管理的*IO*的**类名**，而 *io1, io2, ...* 是你要管理的*IO*的**实例或指针**

特别的，在DEBUG模式下，传入*nullptr*会让manager放弃对该*IO*的启动和管理，并自动跳过所有操作，而不会报错。你可以使用此功能方便的关闭某些*IO*进行debug。

而在RELEASE模式下，你不应该这么做，这会导致程序发生段错误
## *task*(必要)
对于一个需要被*Hardware_manager*管理的*IO*来说，你必须实现一个名为***task***的***public***函数
```cpp
void task() {
    ...
}
```
该函数会在*Hardware_manager*构造的同时，作为一个线程被启动起来

### 用例
```cpp
#include "hardware_manager.hpp"

class Can
{
   public:
    void task() {
        std::cout << "can is start!" << std::endl;
    }
};

int main() {
    Can can0;
    Hardware::Hardware_manager<Can> hardware(can0);
    hardware.join();
}
```
## send

如果你需要实现发送信息的功能，你可以在类中实现名为***send***的***public***函数（你可以实现多种不同的*send*函数）
```cpp
void send(Args...) {
    ...
}
```
在外部，你可以对*hardware_manager*的实例调用*send*函数，并标明设备
```cpp
hardware.send<idx>(args...)
```
*hardware_manager*则会直接调用第***idx***个被管理对象的*send*函数，并将参数***args***传入

### 用例
```cpp
#include "hardware_manager.hpp"

class Can
{
   public:
    Can(const std::string &name) : name_(name) {}

    void task() {
        std::cout << "can is start!" << std::endl;
    }

    void send(const std::string &msg) {
        std::cout << name_ << " send " << msg << std::endl;
    }

    void send(int x) {
        std::cout << name_ << " send " << x << std::endl;
    }

   private:
    std::string name_;
};

int main() {
    Can can0("can0");
    Hardware::Hardware_manager<Can> hardware(can0);
    hardware.join();
    hardware.send<0>("msg1");
    hardware.send<0>(123);
}
```
## join
对*hardware_manager*对象调用*join*函数会自动join它启动的所有线程(也就是所有被管理对象的*task*函数)

### 用例
```cpp
#include "hardware_manager.hpp"

class Can
{
   public:
    Can(const std::string &name) : name_(name) {}

    void task() {
        for(int i = 0; i < 10; i++) {
            std::cout << name_ << " task at " << i << std::endl;
        }
        std::cout << name_ << " task exit!" << std::endl;
    }

   private:
    std::string name_;
};

int main() {
    Can can0("can0");
    Can can1("can1");
    Hardware::Hardware_manager<Can, Can> hardware(can0, can1);
    hardware.join();
}
```

## callback
在io_callback.hpp中，提供了*Callback*类。

如果你的*IO*类想要实现设置回调函数的功能，需要*public*继承*Callback*类，并传入你想要进行callback的类型。

如果你的一个*IO*上挂载了多个设备，并且他们所需要的回调函数类型不同，那么你可以向*Callback*类传入多种类型，
你的*IO*类就会存在多个不同类型的回调函数，并且可以被分别注册。

```cpp
class IoName : public Callback<Type1, Type2, ...> {
    ...
};
```

继承*Callback*后，你可以直接在类中(一般是在*task*函数中)使用*callback*函数，其参数是你选定的其中一种

```cpp
class IoName : public Callback<Type1, Type2, ...> {
    void task() {
        ...
        Type1 val1 = ...;
        ...
        callback(val1);
        ...
        Type2 val2 = ...;
        ...
        callback(val2);
        ...
    }
};
```

## register_callback(不含Key、设备ID)
你可以对任意一个继承了*Callback*的*IO*注册回调函数，你需要对*hardware_manager*调用*register_callback*函数，指定*IO*编号和回调类型。

你所注册的回调函数需要是一个lambada函数或者std::function，其返回值为空，并且传入的参数是你所规定的回调类型。

```cpp
hardware.register_callback<idx, Type>([&](const Type &data) {
    ...
});
```
或
```cpp
hardware.register_callback<idx, Type>(function);
```

特别的，如果第*idx*指向的*IO*设备只存在一种回调类型，那么我们可以省略*Type*。

```cpp
hardware.register_callback<idx>([&](const Type &data) {
    ...
});
```
或
```cpp
hardware.register_callback<idx>(function);
```

在注册后，*IO*调用的该类型的回调函数，会变成你注册的回调函数

### 用例
```cpp
#include "io_callback.hpp"
#include "hardware_manager.hpp"

class Can : public Hardware::Callback<int, std::string>
{
   public:
    void task() {
        while (true) {
            int type;
            std::cin >> type;
            if (type == 0) {
                int msg1;
                std::cin >> msg1;
                callback(msg1);
            } else {
                std::string msg2;
                std::cin >> msg2;
                callback(msg2);
            }
        }
    }
};

int main() {
    Can can0;
    Hardware::Hardware_manager<Can> hardware(can0);

    hardware.register_callback<0, int>(
        [&](int x) { std::cout << "get callback with int, value is " << x << std::endl; });

    hardware.register_callback<0, std::string>(
        [&](const std::string &x) { std::cout << "get callback with string, value is " << x << std::endl; });

    hardware.join();
}
```

## Callback_key
在io_callback.hpp中，提供了*Callback_key*类。

如果你的一个*IO*设备挂载了多个相同的设备，但有着不同的设备ID，那么你可以选择继承*Callback_key*类。

其参数列表中的第一个参数是作为Key（设备ID）的类型，之后的参数则是回调函数的参数类型（一般是一个，也可以传入多个）。

```cpp
class IoName : public Callback_key<KeyType, Arg1, ...> {
    ...
};
```

继承Callback_key后，你可以直接在类中(一般是在task函数中)使用*callback_key*函数，其第一个参数是*KeyType*类型的，表示设备id，
剩下的则是传入回调函数的参数

```cpp
class IoName : public Callback<KeyType, Args1, ...> {
    void task() {
        ...
        callback_key(key, args..);
        ...
    }
};
```

## register_callback(含Key、设备ID)
同样的，你可以对任意一个继承了*Callback_key*的*IO*注册回调函数，你需要对*hardware_manager*调用*register_callback*函数，但只需要指定*IO*编号

你所注册的回调函数需要是一个lambada函数或者std::function，其返回值为空，并且传入的参数是你所规定的回调类型。

不同的是，在函数前你还需要传入本次注册的Key，也就是设备ID。

```cpp
hardware.register_callback<idx>(key, [&](const Type &data) {
    ...
});
```
或
```cpp
hardware.register_callback<idx>(key, function);
```

在注册后，如果*IO*调用了*callback_key*，并且传入的key和你本次注册的key匹配，则manager会调用你注册的回调函数

### 用例
```cpp
#include "io_callback.hpp"
#include "hardware_manager.hpp"

class Can : public Hardware::Callback_key<std::string, double>
{
   public:
    void task() {
        while (true) {
            std::string name;
            double val;
            std::cin >> name >> val;
            callback_key(name, val);
        }
    }
};

int main() {
    Can can0;
    Hardware::Hardware_manager<Can> hardware(can0);

    hardware.register_callback<0>(
        "imu", [&](double x) { std::cout << "imu get callback, with value " << x << std::endl; });

    hardware.register_callback<0>(
        "rc", [&](double x) { std::cout << "rc get callback, with value " << x << std::endl; });

    hardware.join();
}
```

## get
如果以上的功能还是不能满足你，那么你可以直接向hardware_manager所要某个*IO*对象的指针，并直接对其操作。
```cpp
auto p = hardware.get<idx>();
```
该函数会返回第***idx***个*IO*设备的指针

### 用例
```cpp
#include "hardware_manager.hpp"

class Can
{
   public:
    Can(const std::string &name) : name_(name) {}

    void task() {
        std::cout << "can is start!" << std::endl;
    }

    void send(int x) {
        std::cout << name_ << " send " << x << std::endl;
    }

    void rename(const std::string &name) {
        name_ = name;
    }

   private:
    std::string name_;
};

int main() {
    Can can0("can0");
    Hardware::Hardware_manager<Can> hardware(can0);
    hardware.join();

    hardware.send<0>(123);
    hardware.get<0>()->rename("can1");
    hardware.send<0>(321);
}
```

## 注意
1. 你可以同时继承*Callback*和*Callback_key*，但是不能同时继承多个*Callback_key*（~~可能之后会改进~~）。
2. 如果你发现了bug，你可以联系@Rhine解决。
3. 请尽量不要修改本库的代码，如果你想了解该管理器的实现并修改，请你仔细阅读ListBase档和Callback的文档，
掌握模板元编程，并清晰其中的继承关系与转化。