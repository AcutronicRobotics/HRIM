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

A preliminary whitepaper about this work is available at https://arxiv.org/abs/1802.01459. Please use the following BibTex entry to cite our work:

```
@article{hrim,
  title={An information model for modular robots: the Hardware Robot Information Model (HRIM)},
  author={Zamalloa, Irati and Muguruza, Iñigo and Hernández, Alejandro and Kojcev, Risto and Mayoral, Víctor},
  journal={arXiv preprint arXiv:1802.01459},
  year={2018}
}
```

### Install dependencies
```
sudo apt-get install python-lxml
```

Alternatively, the OS-independent `pip` command would be:
```bash
pip install lxml
```

### Script usage

The command structure would be the following:
```
python hrim.py [-h] [-p {ros2}] [-e] {show,generate,list,clear} filePath
```
Where:

`-h` shows script's help.

`-p/--platform` should be followed by which platform (e.g. ROS 2) to generate the implementation in. Current valid values are:
* ros2

`-e/--extend` indicates whether already shown definitions should be shown again or not when printing the module's model.

`{show,generate,list,clear}` is mandatory and signifies the action to take:
* **show**: parses the passed module file and prints it's processed contents.
* **generate**: parses the passed module file and generates the corresponding packages/files to use with the selected platform (ROS 2 by default). The generated files will be located inside  a *generated* directory on the root of the repository.
* **list**: provides a list of existing models or implementations.
* **clear**: deletes generated implementations.
<!-- * **clear**: deletes the passed module's generated implementation. Looks for the passed path inside the *generated* directory, i.e. `force` would look for *generated/force* for deletion. -->

`filePath` is mandatory and is dependent on the action to take.

If the chosen action is **show** it signifies the path (either absolute or relative from the repository's root) to one of the following files:
* models/actuator/gripper/gripper.xml
* models/actuator/gripper/gripper_clean.xml
* models/actuator/electricmotor/electricmotor.xml
* models/actuator/electricmotor/electricmotor_clean.xml
* models/actuator/servo/servo.xml
* models/actuator/servo/servo_clean.xml
* models/composite/mobilebase/mobilebase.xml
* models/composite/mobilebase/mobilebase_clean.xml
* models/composite/conveyor/conveyor.xml
* models/composite/conveyor/conveyor_clean.xml
* models/composite/arm/arm.xml
* models/composite/arm/arm_clean.xml
* models/power/battery/battery.xml
* models/power/battery/battery_clean.xml
* models/sensor/lidar/lidar.xml
* models/sensor/lidar/lidar_clean.xml
* models/sensor/rangefinder/rangefinder.xml
* models/sensor/rangefinder/rangefinder_clean.xml
* models/sensor/thermometer/thermometer.xml
* models/sensor/thermometer/thermometer_clean.xml
* models/sensor/3dcamera/3dcamera_stereo.xml
* models/sensor/3dcamera/3dcamera_stereo_clean.xml
* models/sensor/3dcamera/3dcamera_tof.xml
* models/sensor/3dcamera/3dcamera_tof_clean.xml
* models/sensor/3dcamera/3dcamera_depth.xml
* models/sensor/3dcamera/3dcamera_depth_clean.xml
* models/sensor/imu/imu.xml
* models/sensor/imu/imu_clean.xml
* models/sensor/hygrometer/hygrometer.xml
* models/sensor/hygrometer/hygrometer_clean.xml
* models/sensor/forcetorque/forcetorque.xml
* models/sensor/forcetorque/forcetorque_clean.xml
* models/sensor/gps/gps.xml
* models/sensor/gps/gps_clean.xml
* models/sensor/gasdetector/gasdetector.xml
* models/sensor/gasdetector/gasdetector_clean.xml
* models/sensor/torque/torque.xml
* models/sensor/torque/torque_clean.xml
* models/sensor/force/force.xml
* models/sensor/force/force_clean.xml
* models/sensor/microphone/microphone.xml
* models/sensor/microphone/microphone_clean.xml
* models/sensor/camera/camera.xml
* models/sensor/camera/camera_clean.xml
* models/sensor/encoder/encoder.xml
* models/sensor/encoder/encoder_clean.xml

All of the following would be valid paths:
* `$(pwd)/models/sensor/torque/torque.xml`
* `./models/sensor/torque/torque.xml`
* `/models/sensor/torque/torque.xml`
* `models/sensor/torque/torque.xml`

If the chosen action is **generate** it can either represent one of the models listed above or one of the existing shorthands to generate the files for every existing module:
* **all**: will generate the files for every existing (non-development) model file.
* **allClean**: will generate the files for every existing development module file (model xml files ending with *_clean*).

If the chosen action is **list** it signifies what is meant to get listed:
* **models**: will list all existing models.
* **implementations**: will list all existing generated implementations.

If the chosen action is **clear** it can either represent a directory name or a shorthand:
* **{directory}**: will delete that directory inside the generation folder (i.e. *force* would delete `generated/force`)
* **all**: will delete every implementation of the chosen platform.

### Examples

While positioned at the repository's root all of the following are valid command executions:

* `python hrim.py show $(pwd)/models/sensor/torque/torque.xml`
* `python hrim.py show ./models/sensor/torque/torque.xml`
* `python hrim.py -e show /models/sensor/torque/torque.xml`
* `python hrim.py show --extend models/sensor/torque/torque.xml`
* `python hrim.py show models/sensor/torque/torque.xml`
* `python hrim.py generate $(pwd)/models/sensor/torque/torque.xml`
* `python hrim.py generate ./models/sensor/torque/torque.xml`
* `python hrim.py -p ros2 generate /models/sensor/torque/torque.xml`
* `python hrim.py generate models/sensor/torque/torque.xml`
* `python hrim.py generate --platform ros2 all`
* `python hrim.py generate allClean`
* `python hrim.py clear force`
* `python hrim.py clear all`
