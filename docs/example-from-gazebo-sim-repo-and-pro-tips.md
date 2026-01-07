# Real Gazebo Sim Repo Examples + Pro Tips and the Pimpl Pattern

<iframe
  width="960"
  height="540"
  src="https://www.youtube.com/embed/Q-MYXJwnlDM?si=CChrpBR0WcfuyHCE"
  title="Gazebo Sim Structure Plugin Perspective"
  frameborder="0"
  allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
  allowfullscreen>
</iframe>


The intention of these entire tutorial series was to make you capable enough to deal with actual [gazebo sim repo codes](https://github.com/gazebosim/gz-sim/tree/gz-sim10/src/systems) & build good understanding of [gazebo sim api](https://gazebosim.org/api/sim/9/namespacegz_1_1sim.html). <br>
Will see ***some tips***, ***actual repo examples*** & ***Piml Pattern used in gazebo sim plugin***.


## Pro Tips

### 1. Best Place To Start Check Gazebo Sim Available Api


important part in api doc:<br>
    - [gz::sim](https://gazebosim.org/api/sim/9/namespacegz_1_1sim.html){target=_blank}<br>
    - [gz::sim::EntityComponentManager](https://gazebosim.org/api/sim/9/classgz_1_1sim_1_1EntityComponentManager.html){target=_blank}<br>
    - [gz::sim::components](https://gazebosim.org/api/sim/9/namespacegz_1_1sim_1_1components.html){target=_blank}<br>

most **important** gazebo sim availabe [components list](https://gazebosim.org/api/sim/9/namespacegz_1_1sim_1_1components.html){target=_blank}


![components list](assets/images/components_lists.png)

<br>
<br>

### 2. Use [Common Codes In System Plugins](common-code-system-plugin.md) from this tutorial

[Common Codes In System Plugins](common-code-system-plugin) is basically cheat sheet of all the code that we ever wrote on this series. It has some different types of function api which can be used to **find entity**, **read/write on the components**

<br>
<br>

### 3. Use Large language models (LLMs) like ChatGPT, Claude,Gemini

Use [ChatGPT](https://chatgpt.com/), [Claude](https://claude.ai/login?returnTo=%2Fnew%3F),[Gemini](https://gemini.google.com/app?hl=en-IN),[Perplexity](https://www.perplexity.ai/) etc LLMs **Free Version** can be used in following way to write better code:

1. Undertanding a gazebo sim repo code
2. Find Some Good Examples
3. Ask LLMS to give some code examples on gazebo sim.
4. Find Issues in building the code.

etc...

**Note:** <br>
Here you should not misunderstand & do pure vibe coding ***(copy/paste) no its very bad to maintain code later***.<br> 
Its more like googles search on steroids for find examples easily & resolving silly mistakes to ***improve your overall efficiency & save time***

<br>
<br>

### 4. Read Gazebo Sim Repo Codes

[gazebo sim systems repo codes](https://github.com/gazebosim/gz-sim/tree/gz-sim10/src/systems)

- Code on gazebo sim repo are more clean & follows better coding standing than the codes we learn on the series but good thing is we touch upon the important parts which you will also find a lot in gazebo sim codes.

- ***Piml Pattern*** used in gazebo sim plugin


### How Pimpl (Pointer to Implementation) Works

In a standard C++ class, if you add a private member variable or a private helper method, any file that #includes that header must be recompiled. With Pimpl, you move those private details into a separate "implementation" class.


example:


.hh file
```c++
// MyClass.h
#include <memory>

class MyClassPrivate;           // Forward declaration


class MyClass {
public:
    MyClass();
    ~MyClass(); // Destructor must be defined in the .cpp
    void doSomething();

private:
    std::unique_ptr<MyClassPrivate> dataPtr; // The pointer to implementation
};
```


.cc file
```c++
// MyClass.cpp
#include "MyClass.h"
#include <iostream>
#include <vector> // This dependency is hidden from the header!

class MyClassPrivate;           // Forward declaration
{
    int secretValue;
    std::vector<double> heavyData; // Changes here don't affect MyClass.h

    void internalLogic() {
        std::cout << "Logic hidden in Pimpl\n";
    }
};

MyClass::MyClass() : pImpl(std::make_unique<dataPtr>()) {
    dataPtr->secretValue = 42;
}

MyClass::~MyClass() = default; // Defined here where Impl is complete

void MyClass::doSomething() {
    dataPtr->internalLogic();
}
```


## Why Use Pimpl?

1. **Faster Build Times (The "Compilation Firewall")**<br>
In large projects, header files are often included by hundreds of other files. If you change a private member in a header, every one of those files must recompile. By using Pimpl, you move those members to the .cpp file. You can change the private logic as much as you want, and only the .cpp file needs to be recompiled.


2. **Binary Compatibility (ABI Stability)**<br>
If you are developing a library (like a DLL or .so file), changing the private members of a class usually changes the size of the object. This breaks binary compatibility with applications already using the library. With Pimpl, the size of MyClass remains constant (the size of one pointer), allowing you to update the library without forcing users to recompile their apps.


3. **Cleaner Headers**<br>
The header file becomes a clean interface. It doesn't leak implementation details or require users to include "messy" internal dependencies.


## VelocityControl Plugin Example From Gazebo Sim Repo

![VelocityControl](assets/images/velocity_control_gz_repo.png)



[VelocityControl from gazebo sim github](https://github.com/gazebosim/gz-sim/tree/gz-sim10/src/systems/velocity_control)

These code logic is very similar to our [MovelMovelTopicWay Plugin](movemodel_using_topic.md) 

so lets make some changes in the code to have Pimp pattern in it.

[MovelMovelTopicWayPimp Plugin github](https://github.com/Hrithik-verma/gazebo-sim-plugins-tutorial/tree/main/standalone_gz_sim_plugins/move_model_topicway_pimp_plugin)



<br>
<br>
<br>

#### Velocity Control System — Exmplained By ChatGPT

<iframe
  src="../assets/ChatGPT_GazeboSim_VelocityControl_Explained.pdf"
  width="100%"
  height="800px"
  style="border:0;"
></iframe>
