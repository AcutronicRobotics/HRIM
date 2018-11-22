import lxml.etree as ET
import sys
import json
import re
import os
from classes import *

# check for value type, else all values'll be strings
def processValue(value, type):
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
def processTopic(topic, mandatory):
	top = Topic(topic.attrib.get("name"), topic.attrib.get("type"), mandatory)
	if "description" in topic.attrib:
		top.desc = topic.attrib.get("description")
	top.fileName = topic.attrib.get("fileName")
	for prop in topic:
		if prop.tag == "property":
			top.addProp(processProperty(prop))
		if prop.tag == "response":
			for res in prop:
				top.addRes(processProperty(res))
	return top

# property and subproperty recursive parsing and processing
def processProperty(property):
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

	# check for subproperties
	if any(x.tag == "property" for x in property):

		# process subproperties
		for subProp in property:
			if subProp.tag == "property":
				prop.addProp(processProperty(subProp))

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
def processParam(parameter, mandatory):
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
					value[i] = processValue(x.text,param.type)
					i+=1
		else:
			value=[]
			for x in parameter:
				if x.tag == "value":
					value.append(processValue(x.text,param.type))
		param.value = value
		return param
	else:
		return Parameter(parameter.attrib.get("name"), parameter.attrib.get("type"), mandatory, parameter.attrib.get("unit"), parameter.attrib.get("description"), processValue(parameter[0].text,parameter.attrib.get("type")))

def main(filePath):

	# check for parsability
	try:
		if os.path.exists(filePath):
			path = filePath
		else:
			path = os.getcwd()+filePath
		tree = ET.parse(path)
	except Exception as e:
		print "Problem parsing "+filePath
		print e
		sys.exit(1)

	tree.xinclude()
	root=tree.getroot()

	# instantiate module
	module = Module(root.attrib.get("name"), root.attrib.get("type"), root.attrib.get("description"))

	# loop through mandatory topics and parameters
	for tag in root[0]:
		if tag.tag == "topic":
			module.addTopic(processTopic(tag, True))
		if tag.tag == "param":
			module.addParam(processParam(tag, True))

	# check for optional topics/parameters
	if len(root)>1:
		# loop through optional topics and parameters
		for tag in root[1]:
			if tag.tag == "topic":
				module.addTopic(processTopic(tag, False))
			if tag.tag == "param":
				module.addParam(processParam(tag, False))

	return module
