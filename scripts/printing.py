import sys
import os
from utils import getTabs
from parsing import ModuleParser
import platform
try:
	import lxml.etree as ET
except ImportError:
	platform_type = platform.system()
	if platform_type in ["Darwin", "Windows"]:
		print("Install lxml using:")
		print("\tpip install lxml")
		print("\tor")
		print("\tpip3 install lxml")
		exit()
	elif platform_type == "Linux":
		print("Install lxml using:")
		print("\tsudo apt install python-lxml")
		print("\tor")
		print("\tsudo apt install python3-lxml")
		exit()


class ModulePrinter:
    # shorthand for printing single and list values
    def printValue(self, tabs, title, value, type=None):
        msg = getTabs(tabs)
        msg+=title
        msg+=": "
        if value is None or len(value)==0:
            msg+="-"
        else:
            if isinstance(value, list) or isinstance(value, dict):
                msg+="["
                for val in value:
                    if not type is None and type == "string":
                        msg+=("\'"+val +"\'")if val is not None else "None, "
                    elif val is None:
                        msg+="None, "
                    else:
                        msg+=str(val)+", "
                msg=msg[:-2]+"]"
            else:
                msg+=value
        print(msg)

    # recursively formats and prints properties
    def printProperty(self, property, baseTabs):
        print(getTabs(baseTabs+1)+property.name+" {")
        # format array values and it's length (if set)
        if property.type in self.dataTypes:
            type=self.dataTypes[property.type]+(("[{}]".format(property.length if property.length is not None else "")) if property.array else "")
        else:
            # if the type doesn't appear on the dataMapping file we assume it's a generated type, we capitalize it to differentiate them
            type=property.type[0:1].upper()+property.type[1:]+(("[{}]".format(property.length if property.length is not None else "")) if property.array else "")
        self.printValue(baseTabs+2, "type", type)
        self.printValue(baseTabs+2, "unit", property.unit)
        if property.unit == "enum" and len(property.enumeration) > 0:
            print(getTabs(baseTabs+2)+"possible values [")
            for val in list(property.enumeration.keys()):
                print(getTabs(baseTabs+3)+val)
            print(getTabs(baseTabs+2)+"]")
        self.printValue(baseTabs+2, "description", property.desc)
        if len(property.properties) > 0:
            self.printValue(baseTabs+2, "fileName", property.fileName)
            if property.fileName is not None and (property.fileName not in self.shownFiles or self.extend):
                self.shownFiles.append(property.fileName)
                print(getTabs(baseTabs+2)+"properties {\n")
                for subProp in property.properties:
                    self.printProperty(subProp, baseTabs+2)
                print(getTabs(baseTabs+2)+"}")
        elif len(property.enumeration) > 0 and property.value is not None:
            self.printValue(baseTabs+2, "value", list(property.enumeration.keys())[list(property.enumeration.values()).index(int(property.value))])
        else:
            self.printValue(baseTabs+2, "value", property.value, property.type)
        print(getTabs(baseTabs+1)+"}\n")

    # formats and prints topics
    def printTopic(self, topic, baseTabs):
        print(getTabs(baseTabs)+topic.name+" {")
        self.printValue(baseTabs+1, "description", topic.desc)
        self.printValue(baseTabs+1, "type", topic.type)
        self.printValue(baseTabs+1, "fileName", topic.fileName)
        print(getTabs(baseTabs+1)+"mandatory: "+("yes" if topic.mandatory else "no"))

        if topic.fileName is not None and (topic.fileName not in self.shownFiles or self.extend):
            self.shownFiles.append(topic.fileName)
            if len(topic.properties)>0:
                print(getTabs(baseTabs+1)+"properties {\n")
                for prop in topic.properties:
                    self.printProperty(prop, baseTabs+1)
                print(getTabs(baseTabs+1)+"}\n")
            if len(topic.response)>0:
                print(getTabs(baseTabs+1)+"response {\n")
                for res in topic.response:
                    self.printProperty(res, baseTabs+1)
                print(getTabs(baseTabs+1)+"}\n")
        print("\t}\n")

    # main print method
    def printModule(self, module, plat, full):
        self.extend = full

        cwd = os.getcwd()

        self.dataTypes = ModuleParser().getDataTypes(plat)

        # print parsing
        print("\n\n{0} MODULE:\n\n".format(module.name.upper()))
        # list topics
        print("TOPICS")
        for topic in module.topics:
            self.printTopic(topic, 1)

        # check for parameters
        if len(module.params)>0:
            print("\n------------------------------------------------------------\n")

            print("PARAMETERS\n")
            for param in module.params:
                print("\t"+param.name+":")
                print(getTabs(2)+"type: "+param.type)
                self.printValue(2, "unit", param.unit)
                self.printValue(2, "description", param.desc)
                print(getTabs(2)+"mandatory: "+("yes" if param.mandatory else "no"))
                self.printValue(2, "value", param.value)
                print("\n")

    def __init__(self):
        self.extend = False
        self.dataTypes = {}
        self.shownFiles = []
