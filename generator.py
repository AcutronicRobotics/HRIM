import sys
import os
import xml.etree.ElementTree as ET

def addNewElement(name, value, unit, last_line):
    if last_line is False:
        print(str(name) + " & " + str(value) + " & " + str(unit) + " \\\\")
    else:
        print(str(name) + " & " + str(value) + " & " + str(unit))

def main():
    tree = ET.parse("/home/erle/Desktop/anas/hrim_milestone3/specific_components_datasheet/actuator/hans_rotaryservo_datasheet.xml");
    root = tree.getroot()

    for property in root:
        tag = property.tag
        if str(tag) == "generic_specs":
            count = 1
            for x in property:
                name = x.attrib["name"]
                #print(x.attrib["name"])

                if "unit" in x.attrib:
                    unit = x.attrib["unit"]
                    #print(x.attrib["unit"])
                else:
                    unit = "N/A"

                for value in x:
                    v = value.text
                    #print(value.text)
                count += 1
                if count <= len(property):
                    addNewElement(name, v, unit, False)
                else:
                    addNewElement(name, v, unit, True)
                #print("\n")

        else:
            print("\n")
            count = 1
            #print("\n")
            #print(property.tag)
            #print("\n")
            for x in property:
                #print(x.attrib["name"])

                if "unit" in x.attrib:
                    unit = x.attrib["unit"]
                    #print(x.attrib["unit"])
                else:
                    unit = "N/A"

                for value in x:
                    v = value.text
                    #print(value.text)
                count += 1
                if count <= len(property):
                    addNewElement(name, v, unit, False)
                else:
                    addNewElement(name, v, unit, True)
                #print("\n")

main()
