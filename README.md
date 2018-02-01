# Hardware Robot Information Model (HRIM), what is it?

HRIM, is an standard interface that offers to the robotics community thefacilitate  manufacturing of reusable and interoperable
robot hardware modules for the construction of modular robots. HRIM focuses on the standardization of the logical interfaces between robot modules, or said otherwise HRIM focuses on hardware, designing a set of rules that each device has to meet in order to achieve interoperability.

An information model for modular robots built upon the principles and abstractions of the popular Robot Operating System (ROS) framework.

**Information Model definition**

An information model is typically understood as an abstraction and a representation of the entities in an environment including their properties, attributes and operations. An information model also describes the way that entities (modules in this particular case) relate to each other.  An information model is typically independent of any specific repository, software usage, protocol, or platform.  As such it can be used to define the semantics for the interactions between modules, becoming all of these interoperable.

**Motivation**

HRIM is created due to the need for a standard model in terms of communication between components.

Nowadays there is no leading candidate for a solid and standard interface that remove the integration effort for bulding robots and much less for a global plug and play standard. This is a topic that has been demangding for years by the industry to to be able to obtain the flexibility in their robots to create small customized series for clients or to be able to use the same machinery to do different tasks, and at the same time reduce costs.

ROS supports the reuse of code, however, although it offers some messages for different components, users end up creating their own messages (adapting them to their needs) or use existing ones as best suits them (without following any rules). Consequently, when a new device is added, the user has to invest time in understanding the content of the package to later be able to implement it.

HRIM wants to simplify this whole process, creating a series of rules and steps to follow when creating the packet of the devices. In this way all the components will use the same pattern to communicate, for example all the cameras will send the images in the same format at least.

Many sectors will benefit thanks to HRIM: developers, industrial roboticists, robot modules manufacturers etc., the robotics industry in general.

**Our objectives**

* Standardize logical middleware interfaces between robot modules.
* Make HRIM a common interface for building any robot, for that we have taking into account all [module types](/hrim/standard%20modules) that make up a robot.
* Create a relevant and simple guide which details the interface that each module have to contain.
* Complete HRIM with the most relevant robotics devices, for now we have few of them.
* Build it as modular as possible to be used by the majority of devices: Instead of creating heavy and large messages with which you can control many things, they are designed to perform specific actions.
* Analyze all types of devices, from the most basic to the most complicated in order to reach the majority of the robot modules.
* Take into account that it will be used in differents [implementations](/hrim/implementations) (midlewares).
* Dynamic platform that changes as the technology advances.
* In the future the intention is the implemention of MDE techniques to make HRIM usable among robotics frameworks, not only with ROSe


**Collaborative**

The best way to create a standardization project is to build it collaboratively. For us it is very important to count with the opinions of the [ROS](http://www.ros.org/) community (considering that the first interaction of the information model is written using ROS messages syntax), robotics experts (being the information model an implementation for differents midlewares) and device manufacturers (since there are the ones who know the most about all the characteristics and the possibilities of each component), among others (all constructive opinions are welcome).

The more collaborative the more solid will be the information model and more useful will be for all who want to benefit from the work done. That is why we wanted to share it in the **state of development**. For our part, we will continue adding more components (giving priority to the most used in robotics), while improving the existing with received feedback.

**Building process**

We are working on creating a modular interface assigning at least a basic message that covers the main objective of the device. For the other characteristics we use optional messages, so the user will be able to work with the 100% of possibilities that the component offers.

Although the information model has required analyzing each device separately to create custom messages, all are based on a [generic structure](/hrim/introduction/general%20structure) created by a strategist designed for the standardization of robotic modules: [standardization process](/hrim/introduction/standardization%20process).

To create an easily understable interface, the information model follows a [naming convention](/hrim/introduction/naming%20convention). It is recommended to check it before jumping to particular cases.

**Standardization organizations**

To push the information model to the next level of acceptance, we are working actively with organizations like [ISO 299](https://www.iso.org/committee/5915511.html) or [OMG](http://www.omg.org/). Especially we have to mention the Hardware Abstraction Layer for Robotics Technology (HAL4RT) created by the OMG organization.

**HAL4RT**

HAL4RT is an Application Program Interface (API) for the layer between an application software of a middleware and the drivers for devices (such as sensor inputs, motor control commands) that increases the portability and reusability of the device drivers. This specification aims to enable device makers, device users and software users to build robotic software without any concern about the differences among the targeted devices. HRIM shares some correlation with different standardization
projects, specifically with [HAL4RT 2.0](http://www.omg.org/spec/HAL4RT/) which although the objective is different, is the existing standardis closest to our vision. However, we must highlight some differences and enhancements:

* HRIM considers all [types of devices](/hrim/standard%20modules) necessary for the construction of a robot including but not limited to
components that provide: sensing, actuation, communication, cognition, user interfaces or power.
* HRIM is based on the ROS component model, that is, it adopts most of its abstractions, while HAL4RT defines a Platform-Independet Model (PIM).
* HRIM aims to include relevant and related hardware standardization efforts such as HAL4RT. The ROS component model is, in some aspects, already compatible with HAL4RT. Therefore, HRIM is HAL4RT compliant.
* HRIM defines the units used for all the devices, taking into account the official repository of [ROS](http://www.ros.org/reps/rep-0103.html) and the [International System of Units](https://www.bipm.org/en/publications/si-brochure/section2-2.html), enabling a well defined and correct communication between devices and systems.
* HRIM infrastructure is based on a thorough market analysis of each type of device. These interactions with manufacturers allow the model to take into account all the possibilities (features) that the robotic hardware market offers.
* HRIM contemplates all the robotics hardware, so that users can build any type of robot. For example, in the Actuator category, besides servomotors, different types and subtypes of actuation devices are being defined. Such as a gripper, a vacuum gripper, etc.
* Compared with HAL4RT, HRIM takes into account devices with complex data streams. For instance, a camera can send an image or a video.

Comparing to other related work and standards, HRIM aims for generality, which is essential for acceptance of the model and for achieving global hardware standardization of robotic components.

**Conformance with HAL4RT**

Conformance with HAL4RT version 2.0 is described below:

* HRIM conforms with section 7.4.1. with the exception of the field property, which has been discarded due to the lack of significance given our particular assumptions (the ROS component model). It is planned that the HALObserver interface will be included in future releases of HRIM.
* HRIM conforms with section 7.4.2 of HAL4RT. Furthermore, our team identified the most common actuators and classified them depending on their specific purpose, thus all the device aspects can be customized in detail. For example, unlike HAL4RT, HRIM divides the servos in two different device-types: linear servomotors and rotary servomotors.
* HRIM conforms with Section 7.4.3 of HAL4RT, creating a model for each one of the most common sensor devices. In addition, motivated by components that could send more than an individual value, HRIM extends the defined abstraction of sensors defined by HAL4RT. For example A camera, besides sending traditional images, could also be capable of sending a compressed image.


**H-ROS**

HRIM was created through a need found during the development of [H-ROS](https://www.h-ros.com),  where  the need of reusability and interoperability in robotics is solved by a plug-and-play hardware components. For that, a standard communication between components is needed. So, HRIM
is the part in charge of providing a common interface.

However we wanted to make HRIM an independent standard interface for robot modules which contains rules/specifications that standardize interactions between different robot components. In this way, we are apporting to robotics community a solution that everyone can benefit from, instead of spending resources for its development.

[H-ROS](https://www.h-ros.com), The Hardware Robot Operating System, delivers a solution for companies manufacturing sensors, actuators etc. to create a modular robot components that **interoperate** and can be easily **reused**; even from different manufacturers, you will only have to connect them together. With [H-ROS](https://www.h-ros.com) you can create **plug and play** devices **simplifying the robot building process**.

Nowadays the incompatibility in terms of communication between components is one of the biggest problem in robotics. An expert is required to spend almost a 70% time over the total (building robot process) working to achieve communication between components. H-ROS eliminates this problem thanks to all its infrastructure, and the information model is part of it, standardizing the content that all the modules must include to be sure that all of them speak the same language at all levels.


H-ROS is an infrastructure that works on top of [ROS 2](http://design.ros2.org/articles/why_ros2.html), what it has leaded us to create an Information Model which standarized modules using ROS messages syntax. But we want to do it more generic adapting the information model for different [implementations](/hrim/implementations). That is to say, the information model is created specifically for [H-ROS](https://www.h-ros.com), but I have to say, that we are taking care of making it as modular as posible, so, all of you can take advantage of this work.

The information model will solve one of the biggest problem of robotics: the incompatibility in terms of communication between components. The information model is focused on create a software standard content that each device has to contain. As mentioned before, [H-ROS](https://www.h-ros.com) uses [ROS 2](http://design.ros2.org/articles/why_ros2.html) infrastructure, that is why we have worked focused on it.
