# Common Used System Plugin API

important part in api doc:<br>
    - [gz::sim](https://gazebosim.org/api/sim/9/namespacegz_1_1sim.html){target=_blank}<br>
    - [gz::sim::EntityComponentManager](https://gazebosim.org/api/sim/9/classgz_1_1sim_1_1EntityComponentManager.html){target=_blank}<br>
    - [gz::sim::components](https://gazebosim.org/api/sim/9/namespacegz_1_1sim_1_1components.html){target=_blank}<br>



### Understand Component 

```c++
using X = Component<DataType, class Tag>;

//        or 

using X = Component<DataType, class Tag, Serializer>;
```

to access component: ```components::X``` , ```X``` is the component Name <br>
Datatype: data type of that component <br>
Serializer: use for data transfer (not so important) <br>


![name](assets/images/name_component.png)


![vel](assets/images/linvel_component.png)


![model](assets/images/model_component.png)

access:<br> 
- ```components::Name```<br>
- ```components::LinearVelocity```<br>
- ```components::Model```


<br>
<br>
<br>

## System Plugin Steps
  ![find entity](assets/images/find_entity.png)


### 1. Find The Entity


We will take help of components to find entity integer value like:

  - has `Name` component of value `xyz`
  - has both `Entity` and `Name` components



  <details>
   <summary>a. plugin is directly attached to entity</summary>
  
  
    in world file .sdf
    ```xml
     <model>
        ...

        <plugin>..... </plugin>   
     </model>
    ```
   
    code

    ```c++
     void MoveModel::Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                        EntityComponentManager &_ecm,
                        EventManager &/*_eventMgr*/){
            // _entity --> is the entity of model
            entity = _entity;
    }
    ```
  </details>

  <details>
  <summary>b. search the entity by its name</summary>
  
   ```c++
      // find the entity (int value) which has Name Component with value as "xyz"
      targetEntity = _ecm.EntityByComponents(components::Name("xyz"));
   ```

   ```c++
   auto entityOpt = _ecm.EntityByName("abc");
   entity = entityOpt.value()
   ```
  </details>
  


  <details>
  <summary>c. find multiple entity</summary>
  

  ```c++

  //check for such components which has Light, Name components
  // each
  _ecm.Each<components::Light, components::Name>(
    [&](const Entity &_entity,
        const components::Light *,
        const components::Name *_name) -> bool
    {
      this->lightEntites.push_back(_entity);
      // gzmsg << "Found light: " << _name->Data()
      //       << " (entity " << _entity << ")\n";
      return true;
    });
  ```

  </details>


<details>
  <summary>d. find EntityByComponents</summary>

uing ```EntityByComponents()```

find such a joint entity which has all of the followings components:
        parent entity is ***model component*** to which plugin is attached,
        has a ***name component*** of value ```joint_1``` 
        ***joint compenent*** is attach on it

[reference code](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/Model.cc#L132C1-L133C1)

```c++
  this->jointEntity = _ecm.EntityByComponents(
      components::ParentEntity(_entity),
      components::Name(jointName),
      components::Joint());
```

### 2. Read/Write on the Component value
![command](assets/images/cmd_component.png)

...``Cmd`` means to command to change that container value like ```LinearVelocityCmd```, ```WorldPoseCmd```, 	```VisualCmd```

### Read
<details>
   <summary>a. Component</summary>

  ```c++
  //find Name component value of the entity(any entity_type eg model,link,world etc)
  auto Name = _ecm.Component<components::Name>(this->targetEntity);
  this->modelName = Name->Data();
  ```

</details>


#### Write
 <details>
   <summary>a. SetComponentData</summary>
   
   ```c++
   const gz::math::Vector3d vel(0.0, 0.0, this->zVelocity);
   //set the LinearVelocityCmd component value to be vel
   _ecm.SetComponentData<components::LinearVelocityCmd>(this->targetEntity,{vel});
   ```
 </details>

  <details>
   <summary>b. mutable component</summary>
   
   ```c++
   //set the LinearVelocityCmd component value to be vel
   auto velComp = _ecm.Component<components::LinearVelocityCmd>(this->targetEntity);
   velComp->Data() = vel;
   ```
 </details>


