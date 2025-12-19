# Some C++ Concepts


## 1.[Namespace in C++](https://www.w3schools.com/cpp/cpp_namespaces.asp){target=_blank}


```c++
#include <iostream>

namespace name_1{
    namespace name_2{
        int x;
        void my_fn(){
            std::cout<<"hi"<<std::endl;
        }
    }
}


int main() {
    
    //access
    name_1::name_2::x = 1;
    name_1::name_2::my_fn();

    return 0;

}

```

similar you will a lot find in gazebo sim codes

```c++
namespace gz
{
namespace sim
{
namespace systems
{

    
.....

}
}
}

//access outside
gz::sim::systems xyz; //means its inside gz , sim , systems namespace
```

so line like these ```gz::transport::Node node;``` means node in gz , transport namespace<br>

also means in the doc its under  `gz -> transport` in the [gazebo sim transport api doc](https://gazebosim.org/api/transport/14/namespacegz_1_1transport.html){target=_blank}
![gz_transport](assets/images/gz_transport.png)

<br>
<br>
<br>
<br>

## 2.[std::optional](https://builtin.com/articles/stdoptional){target=_blank}

The term std::option refers to the std::optional class template in C++, which is used to represent a value that ```may or may not be present.```

```c++
std::optional<std::string> maybe_name = "Alice";

if (maybe_name.has_value()) {  //has_value()
    std::cout << "Name is: " << maybe_name.value() << std::endl;  //value()
}

```


so similarly in gazebo sim api also [api example](https://gazebosim.org/api/sim/9/classgz_1_1sim_1_1EntityComponentManager.html#a2c134e7116a28967f3f276d0ebdced16:~:text=a%20model%20component%3A-,std%3A%3Aoptional%3C%20Entity%20%3E%C2%A0,-EntityByName%20(const%20std)){target=_blank}

![optional](assets/images/optional.png)

```c++
    auto entityOpt = _ecm.EntityByName(this->modelName);
    if (!entityOpt.has_value())
    {
      gzdbg << "Model [" << this->modelName
            << "] not found yet. Skipping velocity application." << std::endl;
      return;
    }

    this->targetEntity = entityOpt.value();
    gzmsg << "Found target model entity: " << this->targetEntity << std::endl;
```

<br>
<br>
<br>
<br>

## 3.[this pointer](https://www.geeksforgeeks.org/cpp/this-pointer-in-c/){target=_blank}

```c++

#include <iostream>

class person{
  private:
    int id;
  
  public:
   person(int id){
     this->id = id;  // access data member
                     // to avoid confusion id = id  
   }
   
   int show_id(){
      return this->id; 
   }
   
   void print_data(){
       std::cout<< this->show_id() << std::endl;  // accessing member funtion
   }
};


int main() {
    person a = person(5);
    a.print_data();
    return 0;
}
```

similary, on plugin code many time we use this pointer<br>
![this_pointer](assets/images/this_pointer.png)