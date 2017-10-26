# Information model

###What is an Information model

An information model is typically understood as an abstraction and a representation of the entities in an environment including their properties, attributes and operations. An information model also describes the way that entities (modules in this particular case) relate to each other.  An information model is typically independent of any specific repository, software usage, protocol, or platform.  As such it can be used to define the semantics for the interactions between modules, becoming all of these interoperable.

####Our objectives

* Standardize logical middleware interfaces between robot modules.

* Create a relevant and simple guide which details the interface that each module have to contain.

* Complete the Information Model with the most relevant robotics devices, for now we have few of them.

* Build it as modular as possible to be used by the majority of devices: Instead of creatinng heavy and large messages with which you can control many things, they are designed to perform specific actions.

* Analyze all types of devices, from the most basic to the most complicated to reach the majority of the robot modules.

* Take into account that it will be used in differents implementations (midlewares).


####Collaborative

The best way to create a standardization project is to build it collaboratively. For us it is very important to count with the opinions of the [ROS](http://www.ros.org/) community (considering that the Information Model is written using ROS messages syntax), robotics experts (being the information model an implementation for differents midleware) and device manufacturers (since there are the ones who know the most all the characerristics and the possibilities of each component), among others (all constructive opinions are welcome).

The more collaborative the more solid will be the Model Information and more useful will be for all who want to benefit from the work done. That is why we wanted to share it in the **state of development**. For our part, we will continue adding more components, giving priority to the most useful robotics component, and also improve the existent documentation with received feedback.

####Building process

We are working on creating a modular interface assigning at least a basic message that covers the main objective of the device. For the other characteristics we use optional messages, so the user will be able to work with the 100% of possibilities that the component offers.

Although the information model has required analyzing each device separately to create custom messages, all are based on a generic structure created by a strategist designed for the standardization of robotic modules.

To create a easily understable interface, the Information model follows a naming convention.

###H-ROS

The Information Model is funded as part of our work in [H-ROS](www.h-ros.com).

[H-ROS](www.h-ros.com), The Hardware Robot Operatin System, delivers a solution for companies manufacturing sensors, actuators etc. to create a modular robot components that interoperate and can be easily reused; even from different manufacturers, you will only have to connect them together, that is to say, with [H-ROS](www.h-ros.com) you can create a _plug and play_ devices **simplifying the robot building process** and making **interoperable modules**.

Nowadays the incompatibility in terms of communication between components is one of the biggest problem in robotics. An expert is required to spend almost a 70% time over the total the incompatibility in terms of communication between components.  For this to happen, all the modules have to follow the Information Model (among many other things) making each of them speak the same language at all levels.

H-ROS is an infrastructure that works on top of [ROS 2](http://design.ros2.org/articles/why_ros2.html), what it has leaded us to create an Information Model which standarized modules using ROS messages syntax. But we want to do it more generic adapting the Information Model for different implementations. That is to say, the Information Model is created specifically for [H-ROS](www.h-ros.com), but I have to say, that we are taking care of making it as modular as posible, so, all of you can take advantage of this work.

Solve one of the biggest problems of robotics: the incompatibility in terms of communication between components. The Information Model is focused on create a software standard content that each device has contain to be sure that all of them speak the same language all levels simplifying the **ROBOT BUILDING PROCESS**. As mentioned before, [H-ROS](www.h-ros.com) uses [ROS 2](http://design.ros2.org/articles/why_ros2.html) infrastructure, that is why we have worked focused on it.


###Standadization organizations

To push ROS to the next level of acceptance, we are working actively with organizations like [ISO 299](https://www.iso.org/committee/5915511.html) or [OMG](http://www.omg.org/). That will help us to create a solid [H-ROS](www.h-ros.com) infrastructure and also make the Information Model a reference document to create a standard robot modules. For the moment, as well as we are totally involved in the [ISO](https://www.iso.org/home.html), International Organization for Standardization, with the OMG organization we are just on the first steps.

