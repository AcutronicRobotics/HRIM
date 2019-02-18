import sys
import json
import re
import os
from .classes import *
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

class ModuleParser:

    # calling-path independent method to get a platforms data type mapping
    def getDataTypes(self, plat):
        try:

            dataTypes = {}

            # save the current path
            basePath = os.path.abspath(os.getcwd())

            # look for the main script to assure the repository root path
            #while not os.path.exists(os.path.join(os.getcwd(), "__main__.py")):
            #    os.chdir("..")

            # parse the mapping file
            dataTree = ET.parse(os.path.join(os.getcwd(), "models", "dataMapping.xml"))
            dataRoot = dataTree.getroot()

            # return to the previous path
            os.chdir(basePath)

            # map the data types
            if any(platform.attrib.get("name") == plat for platform in dataRoot):
                for platform in dataRoot.iter("platform"):
                    if platform.attrib.get("name") == plat:
                        for dataType in platform.iter("type"):
                            dataTypes[dataType.attrib.get("name")] = dataType.attrib.get("value")
            else:
                print("Chosen platform doesn't exist")
                sys.exit(1)

        except Exception as e:
            print("Error parsing dataType mapping file")
            raise

        return dataTypes

    # check for value type, else all values'll be strings
    def processValue(self, value, type):
        if value is None or len(value)==0:
            return None
        # if numeric with no decimals (bool would either be 0 or 1)
        if type in ["int","uint","bool"]:
            return int(value)
        # if numeric with decimals
        elif type in ["float","double"]:
            return float(value)
        # else return the value without modifying, it's treated as a string
        else:
            return value

    # topic parsing and processing
    def processTopic(self, topic, mandatory):
        top = Topic(topic.attrib.get("name"), topic.attrib.get("type"), mandatory)
        if "description" in topic.attrib:
            top.desc = topic.attrib.get("description")
        top.fileName = topic.attrib.get("fileName")
        for prop in topic:
            if prop.tag == "property":
                top.addProp(self.processProperty(prop))
            if prop.tag == "response":
                for res in prop:
                    top.addRes(self.processProperty(res))
            if prop.tag == "feedback":
                for res in prop:
                    top.addFeed(self.processProperty(res))
        return top

    # property and subproperty recursive parsing and processing
    def processProperty(self, property):
        if "type" in property.attrib:
            type = property.attrib.get("type")

            # check for array declaration in type
            if re.search(r"\[.*\]", type):
                prop = Property(property.attrib.get("name"), re.sub(r"\[.*\]", "", type))
                prop.array = True

                # check for array length if specified
                matches = re.finditer(r"\[\D*(\d+)\]", type)
                for i, match in enumerate(matches):
                    prop.length = int(match.group(1))
            else:
                prop = Property(property.attrib.get("name"), type)
        else:
            prop = Property(property.attrib.get("name"))
        if "unit" in property.attrib:
            prop.unit = property.attrib.get("unit")

            # check for enumeration values
            if prop.unit=="enum" and "enum" in property.attrib:
                prop.enumeration = json.loads(property.attrib.get("enum"))
        if "description" in property.attrib:
            prop.desc = property.attrib.get("description")

        if "fileName" in property.attrib:
            prop.fileName = property.attrib.get("fileName")

        if "package" in property.attrib:
            prop.package = property.attrib.get("package")

        # check for subproperties
        if any(x.tag == "property" for x in property):

            # process subproperties
            for subProp in property:
                if subProp.tag == "property":
                    prop.addProp(self.processProperty(subProp))

        # else process the value
        else:

            # if array assume multiple values
            if prop.array:

                # if length is specified fill every entry before appending value
                if prop.length is not None:
                    value=[None]*prop.length
                    i = 0
                    for x in property:
                        if x.tag == "value" and i<prop.length:
                            value[i] = x.text
                            i+=1

                # else simply append value to the array
                else:
                    value=[]
                    for x in property:
                        if x.tag == "value":
                            value.append(x.text)

            # else process a single value
            else:
                value = property[0].text
            prop.value = value
        return prop

    # parameter parsing and processing
    def processParam(self, parameter, mandatory):
        # check for array declaration in type
        if re.search(r"\[.*\]", parameter.attrib.get("type")):
            param = Parameter(parameter.attrib.get("name"), re.sub(r"\[.*\]", "", parameter.attrib.get("type")), mandatory, parameter.attrib.get("unit"), parameter.attrib.get("description"))
            param.array = True

            # check for array length if specified
            matches = re.finditer(r"\[\D*(\d+)\]", parameter.attrib.get("type"))
            for i, match in enumerate(matches):
                param.length = int(match.group(1))

            if param.length is not None:
                value=[None]*param.length
                i = 0
                for x in parameter:
                    if x.tag == "value" and i<param.length:
                        value[i] = self.processValue(x.text,param.type)
                        i+=1
            else:
                value=[]
                for x in parameter:
                    if x.tag == "value":
                        value.append(self.processValue(x.text,param.type))
            param.value = value
            return param
        else:
            return Parameter(parameter.attrib.get("name"), parameter.attrib.get("type"), mandatory, parameter.attrib.get("unit"), parameter.attrib.get("description"), self.processValue(parameter[0].text,parameter.attrib.get("type")))

    def parseBase(self, path):
        genericTopics = []

        try:
            genTree = ET.parse(path)
            genTree.xinclude()
            genRoot=genTree.getroot()
            for topic in genRoot:
                genericTopics.append(self.processTopic(topic, True))
        except:
            print("Problem parsing generic topics")
            sys.exit(1)
        return genericTopics

    def parseComposition(self, path):
        compTree = ET.parse(path)
        compRoot=compTree.getroot()
        composition = Composition(compRoot.attrib.get("name"))
        for model in compRoot:
            topicList = []
            for topic in model.findall("topic"):
                topicList.append(topic.attrib.get("name"))
            paramList = []
            for param in model.findall("param"):
                paramList.append(param.attrib.get("name"))
            modelPath = os.path.join(os.getcwd(), model.attrib.get("path"))
            module = self.parseFile(modelPath)
            for topic in list(module.topics):
                if not topic.mandatory and topic.name not in topicList: module.topics.remove(topic)
            for param in list(module.params):
                if not param.mandatory and param.name not in paramList: module.params.remove(param)
            composition.modules.append(module)
        return composition

    # main parse method
    def parseFile(self, filePath):

        # check for parsability
        try:
            if os.path.exists(filePath):
                path = filePath
            else:
                path = os.getcwd()+filePath
            tree = ET.parse(path)
        except Exception as e:
            print("Problem parsing "+filePath)
            print(e)
            sys.exit(1)

        tree.xinclude()
        root=tree.getroot()

        # instantiate module
        module = Module(root.attrib.get("name"), root.attrib.get("type"), root.attrib.get("description"))

        # loop through mandatory topics and parameters
        for tag in root[0]:
            if tag.tag == "topic":
                module.addTopic(self.processTopic(tag, True))
            if tag.tag == "param":
                module.addParam(self.processParam(tag, True))

        # check for optional topics/parameters
        if len(root)>1:
            # loop through optional topics and parameters
            for tag in root[1]:
                if tag.tag == "topic":
                    module.addTopic(self.processTopic(tag, False))
                if tag.tag == "param":
                    module.addParam(self.processParam(tag, False))

        return module

    def __init__(self):
        pass
