import sys
import os
import xml.etree.ElementTree as ET

def addNewElement(name, value, unit):
    print(str(name) + " & " + str(value) + " & " + str(unit))

def main():
    tree = ET.parse("/home/erle/Desktop/anas/hrim_milestone3/specific_components_datasheet/actuator/hans_rotaryservo_datasheet.xml");
    root = tree.getroot()

    for property in root:
        tag = property.tag
        if str(tag) == "generic_specs":
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
                addNewElement(name, v, unit)
                #print("\n")

        else:
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
                addNewElement(name, v, unit)
                #print("\n")

main()
