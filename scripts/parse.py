import lxml.etree as ET
import sys
import json
import re
from classes import *

# if topic/properties not defined on the parsed file should show their full content or not when printing
extend = False
if len(sys.argv) == 3:
	if sys.argv[2] == "1":
		extend = True

# topic parsing and processing
def processTopic(topic, mandatory):
	top = Topic(topic.attrib.get("name"), topic.attrib.get("type"), mandatory)
	if "description" in topic.attrib:
		top.desc = topic.attrib.get("description")
	for prop in topic:
		if prop.tag == "property":
			top.addProp(processProperty(prop))
	if "origin" in topic.attrib:
		top.origin = topic.attrib.get("origin")
	return top

# property and subproperty recursive parsing and processing
def processProperty(property):
	if "type" in property.attrib:
		type = property.attrib.get("type")
		if re.search(r"\[.*\]", type):
			prop = Property(property.attrib.get("name"), re.sub(r"\[.*\]", "", type))
			prop.array = True
			matches = re.finditer(r"\[\D*(\d+)\]", type)
			for i, match in enumerate(matches):
				prop.length = int(match.group(1))
		else:
			prop = Property(property.attrib.get("name"), type)
	else:
		prop = Property(property.attrib.get("name"))
	if "unit" in property.attrib:
		prop.unit = property.attrib.get("unit")
		if prop.unit=="enum" and "enum" in property.attrib:
			prop.enumeration = json.loads(property.attrib.get("enum"))
	if "description" in property.attrib:
		prop.desc = property.attrib.get("description")

	if any(x.tag == "property" for x in property):

		if "origin" in property.attrib:
			prop.origin = property.attrib.get("origin")

		for subProp in property:
			if subProp.tag == "property":
				prop.addProp(processProperty(subProp))
	else:
		if prop.array:
			if prop.length is not None:
				value=[None]*prop.length
				i = 0
				for x in property:
					if x.tag == "value" and i<prop.length:
						value[i] = x.text
						i+=1
			else:
				value=[]
				for x in property:
					if x.tag == "value":
						value.append(x.text)
		else:
			value = property[0].text
		prop.value = value
	return prop

# shorthand for tabulation
def getTabs(n):
	str=""
	for x in range(n):
		str+="\t"
	return str

# shorthand for printing single and list values
def mostrar(tabs, title, value, type=None):
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
					msg+=val+", "
			msg=msg[:-2]+"]"
		else:
			msg+=value
	print msg

# recursively formats and prints properties
def printProperty(property, baseTabs):
	print getTabs(baseTabs+1)+property.name+" {"
	# format array values and it's length (if set)
	type=dataTypes[property.type]+(("[{}]".format(property.length if property.length is not None else "")) if property.array else "")
	mostrar(baseTabs+2, "type", type)
	mostrar(baseTabs+2, "unit", property.unit)
	if property.unit == "enum":
		print getTabs(baseTabs+2)+"possible values ["
		for val in property.enumeration.keys():
			print getTabs(baseTabs+3)+val
		print getTabs(baseTabs+2)+"]"
		# mostrar(baseTabs+2, "possible values", property.enumeration)
	mostrar(baseTabs+2, "description", property.desc)
	if len(property.properties) > 0:
		print getTabs(baseTabs+2)+"origin: "+(property.origin if len(property.origin)>0 else "own")
		if extend or not len(property.origin)>0:
			print getTabs(baseTabs+2)+"properties {\n"
			for subProp in property.properties:
				printProperty(subProp, baseTabs+2)
			print getTabs(baseTabs+2)+"}"
	elif len(property.enumeration) > 0 and property.value is not None:
		mostrar(baseTabs+2, "value", property.enumeration.keys()[property.enumeration.values().index(int(property.value))])
	else:
		mostrar(baseTabs+2, "value", property.value, property.type)
	print getTabs(baseTabs+1)+"}\n"

# formats and prints topics
def printTopic(topic, baseTabs):
	print getTabs(baseTabs)+topic.name+" {"
	mostrar(baseTabs+1, "description", topic.desc)
	mostrar(baseTabs+1, "type", topic.type)
	print getTabs(baseTabs+1)+"origin: "+(topic.origin if len(topic.origin)>0 else "own")
	# print getTabs(baseTabs+1)+"origin: "+(topic.origin if len(topic.origin)>0 else "own")
	print getTabs(baseTabs+1)+"mandatory: "+("yes" if topic.mandatory else "no")

	if extend or not len(topic.origin)>0:
		print getTabs(baseTabs+1)+"properties {\n"
		for prop in topic.properties:
			printProperty(prop, baseTabs+1)
		print getTabs(baseTabs+1)+"}\n"
	print "\t}\n"

# check for file input
if len(sys.argv) == 1:
	print "Please input file to parse"
	sys.exit()

# check for parsability
try:
	tree = ET.parse(sys.argv[1])
except Exception as e:
	print "Problem parsing "+sys.argv[1]
	print e
	sys.exit(1)

tree.xinclude()
root=tree.getroot()

plat = "ros2"

# map the datatypes
dataTypes = {}
dataTree = ET.parse("/home/erle/workspace/xmlHRIM/models/dataMapping.xml")
dataRoot = dataTree.getroot()

if any(platform.attrib.get("name") == plat for platform in dataRoot):
	for platform in dataRoot.iter("platform"):
		if platform.attrib.get("name") == plat:
			for type in platform.iter("type"):
				dataTypes[type.attrib.get("name")] = type.attrib.get("value")
else:
	print "Chosen platform doesn't exist"
	sys.exit(1)

# instantiate module
module = Module(root.attrib.get("name"))

# loop through mandatory topics and parameters
for tag in root[0]:
	if tag.tag == "topic":
		module.addTopic(processTopic(tag, True))
	if tag.tag == "param":
		module.addParam(Parameter(tag.attrib.get("name"), tag.attrib.get("type"), True, tag.attrib.get("unit"), tag.attrib.get("description"), tag[0].text))

# check for optional topics/parameters
if len(root)>1:
	# loop through optional topics and parameters
	for tag in root[1]:
		if tag.tag == "topic":
			module.addTopic(processTopic(tag, False))
		if tag.tag == "param":
			module.addParam(Parameter(tag.attrib.get("name"), tag.attrib.get("type"), False, tag.attrib.get("unit"), tag.attrib.get("description"), tag[0].text))

# print parsing results
print "\n\n{0} MODULE:\n\n".format(module.name.upper())

# list topics
print "TOPICS"
for topic in module.topics:
	printTopic(topic, 1)

# check for parameters
if len(module.params)>0:
	print "\n------------------------------------------------------------\n"

	print "PARAMETERS\n"
	for param in module.params:
		print "\t"+param.name+":"
		print getTabs(2)+"type: "+param.type
		mostrar(2, "unit", param.unit)
		mostrar(2, "description", param.desc)
		print getTabs(2)+"mandatory: "+("yes" if param.mandatory else "no")
		mostrar(2, "value", param.value)
		print "\n"
