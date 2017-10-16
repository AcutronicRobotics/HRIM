# Information model

## What is?
An information model is typically understood as an abstraction and a representation of the entities in an environment including their properties, attributes and operations. An information model also describes the way that entities (modules in this particular case) relate to each other. An information model is typically independent of any specific repository, software usage, protocol, or platform. As such it can be used to define the semantics for the interactions between modules, becoming all of these interoperable.

## Building process
Although the information model has required analyzing each device separately to create custom messages, all are based on a generic structure created by a strategist designed for the standardization of robotic modules: standardization process.

To create a easily understable implementation, nodes, topics, messages, services etc. the Information model follow the same naming convention. It is a base of the information model, so it is recommended to check it before jumping to particular cases.

## Solid infraestructure
We are working actively with standardization international organizations like OMG and ISO - Modularization of services robots (ISO TC 299).

## H-ROS
It is a project founded by H-ROS. H-ROS, The Hardware Robot Operatin System, delivers a solution for companies manufacturing sensors, actuators etc. to create a modular robot components that interoperate and can be easily reused; even from different manufacturers, you will only have to connect them together, that is to say, with H-ROS you can create a plug and play devices. Accordingly, H-ROS simplify the robot building process and makes the modules interoperable.

Nowadays the incompatibility in terms of communication between components is one of the biggest problem in robotics. An expert is required to spend almost a 70% time over the total the incompatibility in terms of communication between components For this to happen, all the modules have to follow the Information Model (among many other things) making each of them speak the same language at all levels.

H-ROS is an infrastructure that works on top of ROS 2, what it has leaded us to create an Information Model which standarized modules using ROS messages syntax. But we want to do it more generic adapting the Information Model for different implementations. That is to say, the Information Model is created specifically for H-ROS, but I have to say, that we are taking care of making it as modular as posible, so, all of you can take advantage of this work.

Solve one of the biggest problems of robotics: the incompatibility in terms of communication between components. The Information Model is focused on create a software standard content that each device has contain to be sure that all of them speak the same language all levels simplifying the ROBOT BUILDING PROCESS. As mentioned before, H-ROS uses ROS 2 infrastructure, that is why we have worked focused on it. .
