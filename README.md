[![Build Status](https://travis-ci.org/AcutronicRobotics/HRIM.svg?branch=master)](https://travis-ci.org/AcutronicRobotics/HRIM)

# Hardware Robot Information Model (HRIM)

![](https://acutronicrobotics.com/docs/technology/hrim/StandardLogicalInterface.jpg)

The Hardware Robot Information Model or **HRIM** for short, is a common interface that facilitates interoperability among different vendors of robot hardware components with the purpose of building modular robots. HRIM focuses on the standardization of the logical interfaces between robot modules, designing a set of rules that each device has to meet in order to achieve interoperability.

- [Documentation](https://docs.h-ros.com/hrim)
- [Web page](http://therobotmodel.com)
- [Discussion and issues](https://github.com/erlerobot/HRIM/issues)
- [Contributions (through pull requests)](https://github.com/erlerobot/HRIM/pulls)

Real world implementations requires taking into account the common hardware modules used in robotics. The robot modules have been classified in 7 **types** of devices: sensor, actuator, cognition, communication, power, ui and composite. Each type is composed by **sub-types or devices** related to the functionality of the module. *For example, a camera is a sub-type of the sensor type* and represented with the following structure:

```bash
generated/
...
├── camera
│   ├── hrim_sensor_camera_msgs
│   │   ├── msg
│   │   │   ├── Audio.msg
│   │   │   ├── CameraInfo.msg
│   │   │   ├── CompressedImage.msg
│   │   │   ├── Image.msg
│   │   │   ├── PTZ.msg
│   │   │   ├── Reconfiguration.msg
│   │   │   └── SpecsCamera.msg
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── hrim_sensor_camera_srvs
│   │   ├── srv
│   │   │   └── SetCameraInfo.srv
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── mandatory_parameters.yaml
│   └── optional_parameters.yaml
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
```bash
pip3 install lxml
```

### Script usage

The command structure would be the following:
```
python3 hrim.py [-h] [-p {ros2}] [-e] {show,generate,compose,compile,list,clear} filePath [filePath ...]
```
Where:

`-h/--help` shows script's help.

`-p/--platform` should be followed by which platform (e.g. ROS 2) to generate the implementation in. Current valid values are:
* ros2

`-e/--extend` indicates whether already shown definitions should be shown again or not when printing the module's model (only shown once by default).

`{show,generate,compose,compile,list,clear}` is mandatory and signifies the action to take:
* **show**: parses the passed module file and prints it's processed contents.
* **generate**: parses the passed module file and generates the corresponding packages/files to use with the selected platform (ROS 2 by default). The generated files will be located inside  a `generated` directory on the root of the repository.
* **compose**: meta-model composition. Generates a composition of models from the passed models in the repository's root, named `model.xml`.
* **compile**: meta-model package generation. Generates the corresponding packages/files based on the passed model composition file.
* **list**: lists either generated implementations or existing models.
* **clear**: deletes generated implementations.

`filePath` is mandatory and what it signifies depends on the action to take:
* **show**: path to the xml model file to show a definition of.
* **generate**: path to the xml model to generate an implementation from. Alternatively, one of the following shorthands:
  * ***all***: generates the implementation of all models.
  * ***actuators***: generates the implementation of every existent actuator models.
  * ***composites***: generates the implementation of every existent composite models.
  * ***powers***: generates the implementation of every existent power models.
  * ***sensors***: generates the implementation of every existent sensor models.
* **compose**: one or more models to make a composition from, in a `type/subtype` or `type/subtype/model` (without the file extension) format. You can get a list of all valid models making use of the **list** function.
* **compile**: the path to the model composition xml file (by default **model.xml**)
* **list**: what you want to list:
  * ***models***: lists all existing models.
  * ***implementations***: lists all existing generated implementations.
* **clear**: name of the implementation to delete from the `generated` directory (i.e. `imu` will delete `generated/imu`). Alternatively, use **all** to delete all generated implementations (doesn't include compositions).

### Examples

While positioned at the repository's root all of the following are valid command executions:

* `python3 hrim.py show $(pwd)/models/sensor/torque/torque.xml`
* `python3 hrim.py show ./models/sensor/torque/torque.xml`
* `python3 hrim.py -e show /models/sensor/torque/torque.xml`
* `python3 hrim.py show --extend models/sensor/torque/torque.xml`
* `python3 hrim.py show models/sensor/torque/torque.xml`
* `python3 hrim.py generate $(pwd)/models/sensor/torque/torque.xml`
* `python3 hrim.py generate ./models/sensor/torque/torque.xml`
* `python3 hrim.py -p ros2 generate /models/sensor/torque/torque.xml`
* `python3 hrim.py generate models/sensor/torque/torque.xml`
* `python3 hrim.py generate --platform ros2 all`
* `python3 hrim.py list models`
* `python3 hrim.py clear all`
* `python3 hrim.py clear imu`
* `python3 hrim.py compose composite/arm sensor/imu actuator/gripper/gripper sensor/3dcamera/3dcamera_stereo`
* `python3 hrim.py compile model.xml`
* `python3 hrim.py compile validComposition.xml`
