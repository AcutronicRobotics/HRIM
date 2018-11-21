[![Build Status](https://travis-ci.org/erlerobot/HRIM.svg?branch=master)](https://travis-ci.org/erlerobot/HRIM)

# Hardware Robot Information Model (HRIM)

![](https://docs.h-ros.com/user/pages/04.HRIM/StandardLogicalInterface.jpg)

The Hardware Robot Information Model or **HRIM** for short, is a common interface that facilitates interoperability among different vendors of robot hardware components with the purpose of building modular robots. HRIM focuses on the standardization of the logical interfaces between robot modules, designing a set of rules that each device has to meet in order to achieve interoperability.

- [Documentation](https://docs.h-ros.com/hrim)
- [Web page](http://therobotmodel.com)
- [Discussion and issues](https://github.com/erlerobot/HRIM/issues)
- [Contributions (through pull requests)](https://github.com/erlerobot/HRIM/pulls)

Real world implementations requires taking into account the common hardware modules used in robotics. The robot modules have been classified in 7 **types** of devices: sensor, actuator, cognition, communication, power, ui and composite. Each type is composed by **sub-types or devices** related to the functionality of the module. *For example, a camera is a sub-type of the sensor type* and represented with the following structure:

```
sensor/
...
├── camera
│   ├── hrim_sensor_camera_msgs
│   │   ├── CMakeLists.txt
│   │   ├── msg
│   │   │   ├── CameraInfo.msg
│   │   │   ├── CompressedImage.msg
│   │   │   ├── Image.msg
│   │   │   ├── PTZ.msg
│   │   │   └── SpecsCamera.msg
│   │   └── package.xml
│   └── hrim_sensor_camera_srvs
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── srv
│           └── SetCameraInfo.srv
...
```

###


A preliminary whitepaper about this work is available at https://arxiv.org/abs/1802.01459. Please use the following BibTex entry to cite our work:

```
@article{hrim,
  title={An information model for modular robots: the Hardware Robot Information Model (HRIM)},
  author={Zamalloa, Irati and Muguruza, Iñigo and Hernández, Alejandro and Kojcev, Risto and Mayoral, Víctor},
  journal={arXiv preprint arXiv:1802.01459},
  year={2018}
}
```

### Script usage

To command structure would be the following:
```
python hrim.py {show|generate} {path-to-component.xml}
```
Where:

`{show|generate}` is mandatory and signifies the action to take:
* **show**: parses the passed module file and prints it's processed contents.
* **generate**: parses the passed module file and generates the corresponding files to use with ROS2. The generated files will be located inside  a `generated` folder on the root of the repository.

`{path-to-component.xml}` is mandatory and signifies the path (either absolute or relative from the repository's root) to one of the following files:
* models/actuator/electricmotor/electricmotor.xml
* models/actuator/electricmotor/electricmotor_clean.xml
* models/actuator/gripper/gripper.xml
* models/actuator/gripper/gripper_clean.xml
* models/actuator/servo/servo.xml
* models/actuator/servo/servo_clean.xml
* models/composite/conveyor/conveyor.xml
* models/composite/conveyor/conveyor_clean.xml
* models/composite/mobilebase/mobilebase.xml
* models/composite/mobilebase/mobilebase_clean.xml
* models/power/battery/battery.xml
* models/power/battery/battery_clean.xml
* models/sensor/encoder/encoder.xml
* models/sensor/encoder/encoder_clean.xml
* models/sensor/gasdetector/gasdetector.xml
* models/sensor/gasdetector/gasdetector_clean.xml
* models/sensor/gps/gps.xml
* models/sensor/gps/gps_clean.xml
* models/sensor/lidar/lidar.xml
* models/sensor/lidar/lidar_clean.xml
* models/sensor/rangefinder/rangefinder.xml
* models/sensor/rangefinder/rangefinder_clean.xml
* models/sensor/torque/torque.xml
* models/sensor/torque/torque_clean.xml

All of the following would be valid paths:
* `$(pwd)/models/sensor/torque/torque.xml`
* `./models/sensor/torque/torque.xml`
* `/models/sensor/torque/torque.xml`
* `models/sensor/torque/torque.xml`


Alternatively, shorthands are provided to **generate** the files for every existing module:
* **all**: will generate the files for every existing (non-development) module file.
* **allClean**: will generate the files for every existing development module file (module xml files ending with *_clean*).

### Examples

While positioned at the repository's root all of the following are valid command executions:

* `python hrim.py show $(pwd)/models/sensor/torque/torque.xml`
* `python hrim.py show ./models/sensor/torque/torque.xml`
* `python hrim.py show /models/sensor/torque/torque.xml`
* `python hrim.py show models/sensor/torque/torque.xml`
* `python hrim.py generate $(pwd)/models/sensor/torque/torque.xml`
* `python hrim.py generate ./models/sensor/torque/torque.xml`
* `python hrim.py generate /models/sensor/torque/torque.xml`
* `python hrim.py generate models/sensor/torque/torque.xml`
* `python hrim.py generate all`
* `python hrim.py generate allClean`
