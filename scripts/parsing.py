import lxml.etree as ET
import sys
import json
import re
from classes import *

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

def main(filePath):

	# check for parsability
	try:
		tree = ET.parse(filePath)
	except Exception as e:
		print "Problem parsing "+filePath
		print e
		sys.exit(1)

	tree.xinclude()
	root=tree.getroot()

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

	return module
