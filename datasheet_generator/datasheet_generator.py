import argparse
import subprocess
import xml.etree.ElementTree as et

component_type_array = ['arm', 'battery', "camera", "converyor", "depthsensor",
                        "encoder", "force", "forcetoque", "gasdetector",
                        "gps", "gripper", "hygrometer", "lidar", "microphone",
                        "mobile", "motor", "rangefinder", "rotaryservo",
                        "thermometer", "torque"]

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--filename', help='file name')
    parser.add_argument('--logo', help='logo file')
    parser.add_argument('--component_type', help='component type',
                        choices=component_type_array)
    args = parser.parse_args()

    tree = et.parse(args.filename)
    root = tree.getroot()

    company_name = args.logo
    component_type = args.component_type

    with open('latex/template.tex', 'r') as content_file:
        content = content_file.read()

    string_concepts = ''
    string_characteristics = ''
    string_features = ''
    index = 0

    for property in root:
        tag = property.tag
        if str(tag) == "generic_specs":
            for x in property:
                if index % 2 == 0:
                    string_concepts = string_concepts + \
                        '\\rowcolor[HTML]{C0C0C0} \n'

                name = x.attrib["name"]
                if "unit" in x.attrib:
                    unit = x.attrib["unit"]
                else:
                    unit = ""
                v = ""
                for value in x:
                    v = value.text
                name = name.replace("_", '\_')
                string_concepts += str(name) + ' & ' + str(v) + " " + str(
                    unit) + ' \\\\ \hline\n'
                index = index + 1

        if str(tag) == "component_features":
            for x in property:
                name = x.attrib["name"]

                unit = ""
                if "unit" in x.attrib:
                    unit = x.attrib["unit"]

                value_min = 0
                value_max = 0
                for value in x:
                    if value.tag == "value_min":
                        value_min = value.text
                    if value.tag == "value_max" :
                        value_max = value.text
                name = name.replace("_", '\_')
                string_features += str(name) + ' & ' + str(
                    value_min) + "/" + str(value_max) + "  " + str(
                    unit) + " & " + x.attrib["description"] + ' \\\\ \hline\n'

        elif str(tag) == "component_specs":
            for x in property:
                name = x.attrib["name"]

                unit = ""
                if "unit" in x.attrib:
                    unit = x.attrib["unit"]
                v = ""
                for value in x:
                    v = value.text
                name = name.replace("_", '\_')
                string_characteristics += str(name) + ' & ' + str(
                    v) + " " + str(unit) + ' \\\\ \hline\n'

    content = content.replace('{%CONCEPT%}', string_concepts)
    content = content.replace('{%CHARACTERISTICS%}', string_characteristics)
    content = content.replace('{%FEATURES%}', string_features)
    content = content.replace('{%COMPANY_LOGO%}', company_name)
    content = content.replace('{%DEVICE_TYPE%}', component_type)

    f = open("latex/output.tex", "w")
    f.write(content)
    f.close()

    subprocess.run(['/bin/pdflatex', '-no-file-line-error' 
                                      'latex/output.tex'], shell=False)
    subprocess.run(['/bin/evince output.pdf', 'output.pdf'], shell=False)
