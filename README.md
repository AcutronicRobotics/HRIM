[![Build Status](https://travis-ci.org/AcutronicRobotics/HRIM.svg?branch=master)](https://travis-ci.org/AcutronicRobotics/HRIM)

# Hardware Robot Information Model (HRIM)

![](https://acutronicrobotics.com/docs/technology/hrim/StandardLogicalInterface.jpg)

The Hardware Robot Information Model or **HRIM** for short, is a common interface that facilitates interoperability among different vendors of robot hardware components with the purpose of building modular robots. HRIM focuses on the standardization of the logical interfaces between robot modules, designing a set of rules that each device has to meet in order to achieve interoperability.

- [Documentation](https://acutronicrobotics.com/docs/technology/hrim)
- [Web page](https://acutronicrobotics.com/technology/hrim/)
- [Discussion and issues](https://github.com/AcutronicRobotics/HRIM/issues)
- [Contributions (through pull requests)](https://github.com/AcutronicRobotics/HRIM/pulls)

Real world implementations requires taking into account the common hardware modules used in robotics. The robot modules have been classified in 7 **types** of devices: sensor, actuator, cognition, communication, power, ui and composite. Each type is composed by **sub-types or devices** related to the functionality of the module. *For example, a camera is a sub-type of the sensor type* and represented with the following structure:

```bash
generated/
...
├── sensor
│   ├── camera
│   │   ├── hrim_sensor_camera_msgs
│   │   │   ├── CMakeLists.txt
│   │   │   ├── msg
│   │   │   │   ├── Audio.msg
│   │   │   │   ├── CameraInfo.msg
│   │   │   │   ├── CompressedImage.msg
│   │   │   │   ├── Image.msg
│   │   │   │   ├── PTZ.msg
│   │   │   │   └── Reconfiguration.msg
│   │   │   └── package.xml
│   │   ├── hrim_sensor_camera_srvs
│   │   │   ├── CMakeLists.txt
│   │   │   ├── package.xml
│   │   │   └── srv
│   │   │       ├── SetCameraInfo.srv
│   │   │       └── SpecsCamera.srv
│   │   ├── mandatory_parameters.yaml
│   │   └── optional_parameters.yaml
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

### Installation
Through pip:
```bash
pip3 install hrim
```
Local install through pip:
```bash
# while positioned at the repository's root
pip3 install -e installator
```
Manually running the installator:
```bash
# while positioned at the repository's root
cd installator
sudo python3 setup.py install
```

### Script usage

#### Commands should be executed from the repository's root

The command structure would be the following:
```
hrim [-h] [-p {ros2}] {generate,compose,compile,list,clear} filePath [filePath ...]
```
Where:

`-h/--help` shows script's help.

`-p/--platform` should be followed by which platform (e.g. ROS 2) to generate the implementation in. Current valid values are:
* ros2

`{generate,compose,compile,list,clear}` is mandatory and signifies the action to take:
* **generate**: generates the corresponding packages/files to use with the selected platform (ROS 2 by default). The generated files will be located inside  a `generated` directory on the root of the repository. It expects either the path to a valid XML model or one of the following shorthands:
  * ***all***: generates the implementation of all models.
  * ***actuators***: generates the implementation of every existent actuator models.
  * ***composites***: generates the implementation of every existent composite models.
  * ***powers***: generates the implementation of every existent power models.
  * ***sensors***: generates the implementation of every existent sensor models.
* **compose**: meta-model composition. Generates a composition of models from the passed models in the repository's root, named `model.xml`. It expects one or more models to make a composition from, in a `type/subtype` or `type/subtype/model` (without the file extension) format. You can get a list of all valid models making use of the **list models** function.
* **compile**: meta-model package generation. Generates the corresponding packages/files based on the passed model composition file.
* **list**: lists either:
  * ***models***: all existing component models.
  * ***implementations***: all generated implementations on the `generated` directory.
* **clear**: deletes implementations from the `generated` directory. It expects either the relative path from said folder to delete (i.e. `sensor/imu` will delete `generated/sensor/imu`, `sensor` will delete all generated sensor implementations) or **all** to delete all generated implementations (doesn't include compositions).

For further information on the model composition process, refer to [our documentation](https://acutronicrobotics.com/docs/technology/hrim/model/meta-model#model-composition).

### Examples

While positioned at the repository's root all of the following are valid command executions:

* `hrim generate $(pwd)/models/sensor/torque/torque.xml`
* `hrim generate ./models/sensor/torque/torque.xml`
* `hrim -p ros2 generate /models/sensor/torque/torque.xml`
* `hrim generate models/sensor/torque/torque.xml`
* `hrim generate --platform ros2 all`
* `hrim list models`
* `hrim list implementations`
* `hrim clear all`
* `hrim clear sensor/imu`
* `hrim clear sensor`
* `hrim compose composite/arm sensor/imu actuator/gripper/gripper sensor/3dcamera/3dcamera_stereo`
* `hrim compile model.xml`
* `hrim compile validComposition.xml`
