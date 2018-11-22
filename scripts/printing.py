import lxml.etree as ET
import sys
import os

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
					msg+=str(val)+", "
			msg=msg[:-2]+"]"
		else:
			msg+=value
	print msg

# recursively formats and prints properties
def printProperty(property, baseTabs):
	print getTabs(baseTabs+1)+property.name+" {"
	# format array values and it's length (if set)
	if property.type in dataTypes:
		type=dataTypes[property.type]+(("[{}]".format(property.length if property.length is not None else "")) if property.array else "")
	else:
		# if the type doesn't appear on the dataMapping file we assume it's a generated type, we capitalize it to differentiate them
		type=property.type[0:1].upper()+property.type[1:]+(("[{}]".format(property.length if property.length is not None else "")) if property.array else "")
	mostrar(baseTabs+2, "type", type)
	mostrar(baseTabs+2, "unit", property.unit)
	if property.unit == "enum":
		print getTabs(baseTabs+2)+"possible values ["
		for val in property.enumeration.keys():
			print getTabs(baseTabs+3)+val
		print getTabs(baseTabs+2)+"]"
	mostrar(baseTabs+2, "description", property.desc)
	if len(property.properties) > 0:
		mostrar(baseTabs+2, "fileName", property.fileName)
		if extend or property.fileName is not None:
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
	mostrar(baseTabs+1, "fileName", topic.fileName)
	print getTabs(baseTabs+1)+"mandatory: "+("yes" if topic.mandatory else "no")

	if extend or topic.fileName is not None:
		if len(topic.properties)>0:
			print getTabs(baseTabs+1)+"properties {\n"
			for prop in topic.properties:
				printProperty(prop, baseTabs+1)
			print getTabs(baseTabs+1)+"}\n"
		if len(topic.response)>0:
			print getTabs(baseTabs+1)+"response {\n"
			for res in topic.response:
				printProperty(res, baseTabs+1)
			print getTabs(baseTabs+1)+"}\n"
	print "\t}\n"

def main(module, full):
    global extend
    extend = full

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

    # print parsing
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
