# Datasheet generator

This tool allows to create a datasheet based on the XML description.


## Script usage

### Script requirements

```bash
pip3 install -r requirements.txt
sudo apt install evince texlive-latex-extra -y
```

The command structure would be the following:

```bash
usage: datasheet_generator.py [-h] [--filename FILENAME] [--logo LOGO]
                              [--component_type {arm,battery,camera,converyor,depthsensor,encoder,force,forcetoque,gasdetector,gps,gripper,hygrometer,lidar,microphone,mobile,motor,rangefinder,rotaryservo,thermometer,torque}]
```

Where:

 - `-h/--help` shows script's help.
 - `--filename` selects which XML file you want to convert to pdf
 - `--logo` selects the logo that the document will show
 - `--component_type` select the component type. Possible options are : `arm`, `battery`,` camera`, `converyor`, `depthsensor`, `encoder`, `force`, `forcetoque`, `gasdetector`, `gps`, `gripper`, `hygrometer`, `lidar`, `microphone`, `mobile` , `motor`, `rangefinder`, `rotaryservo`, `thermometer`, `torque`

## Examples

 * `python3 datasheet_generator.py --filename specific_components_datasheet/actuator/robotiq_gripper_finger_datasheet.xml --logo robotiq --component_type gripper`
 * `python3 datasheet_generator.py --filename specific_components_datasheet/actuator/hans_rotaryservo_datasheet.xml --logo hans --component_type rotaryservo`
