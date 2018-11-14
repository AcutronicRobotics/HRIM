import lxml.etree as ET
import sys
import os
import operator

def main(module):
	# default platform
	plat = "ros2"

	# map the datatypes
	global dataTypes
	dataTypes = {}

	cwd = os.getcwd()

	dataTree = ET.parse(cwd+"/models/dataMapping.xml")
	dataRoot = dataTree.getroot()

	if any(platform.attrib.get("name") == plat for platform in dataRoot):
		for platform in dataRoot.iter("platform"):
			if platform.attrib.get("name") == plat:
				for type in platform.iter("type"):
					dataTypes[type.attrib.get("name")] = type.attrib.get("value")
	else:
		print "Chosen platform doesn't exist"
		sys.exit(1)

	if not os.path.exists(cwd+"/generated"):
			os.mkdir("generated")
	os.chdir("generated")
	cwd = os.getcwd()

	if not os.path.exists(cwd+"/"+module.name):
		os.mkdir(module.name)

	os.chdir(module.name)
	cwd = os.getcwd()

	for topic in module.topics:
		os.chdir(cwd)
		if len(topic.origin)==0:
			folderPath = os.getcwd()+"/hrim_"+module.type+"_"+module.name+"_msgs/msg"
			if not os.path.exists(folderPath):
				os.makedirs(folderPath)
			os.chdir(folderPath)
			msg = ""
			if topic.desc is not None and len(topic.desc)>0:
				msg+="# "+topic.desc+"\n\n"
			for prop in topic.properties:
				if prop.unit is not None and prop.unit == "enum":
					for value in sorted( ((v,k) for k,v in prop.enumeration.iteritems())):
						msg+=prop.type+" "+value[1]+"="+str(value[0])+"\n"
				msg+=dataTypes[prop.type]+("[{}] ".format(prop.length) if prop.array else " ")+prop.name+((" # "+prop.desc) if prop.desc is not None else "")+"\n\n"
			filepath = topic.name+".msg"
			text_file = open(filepath, "w")
			text_file.write(msg)
			text_file.close()
