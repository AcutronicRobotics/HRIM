import lxml.etree as ET
import sys
import os

# shorthand for yaml tabulation
def getTabs(n):
	str=""
	for x in range(n):
		str+="  "
	return str

def formatProperty(prop):
	return dataTypes[prop.type]+("[{}] ".format(prop.length if prop.length is not None else "") if prop.array else " ")+prop.name+((" # "+prop.desc) if prop.desc is not None else "")+"\n\n"

def main(module):

	messages = False
	services = False

	# default platform
	plat = "ros2"

	# map the datatypes
	global dataTypes
	dataTypes = {}
	cwd = os.getcwd()
	dataTree = ET.parse(cwd+"/models/dataMapping.xml")
	dataRoot = dataTree.getroot()

	# check for the platform
	if any(platform.attrib.get("name") == plat for platform in dataRoot):
		for platform in dataRoot.iter("platform"):
			if platform.attrib.get("name") == plat:
				for type in platform.iter("type"):
					dataTypes[type.attrib.get("name")] = type.attrib.get("value")
	else:
		print "Chosen platform doesn't exist"
		sys.exit(1)

	# parse the templates necessary for the package
	os.chdir("templates")
	with open('package.txt', 'r') as myfile:
		pkg=myfile.read()

	# insert the package's name and description in package.xml's content
	msgPkg = pkg.replace("%PKGNAME%", "hrim_"+module.type+"_"+module.name+"_msgs")
	msgPkg = msgPkg.replace("%PKGDESC%", module.desc)

	# insert the package's name and description in package.xml's content
	srvPkg = pkg.replace("%PKGNAME%", "hrim_"+module.type+"_"+module.name+"_srvs")
	srvPkg = srvPkg.replace("%PKGDESC%", module.desc)

	with open('cmake.txt', 'r') as myfile:
		makeFile=myfile.read()

	# insert the package's name and description in CMakeLists.txt's content
	msgMakeFile = makeFile.replace("%PKGNAME%", "hrim_"+module.type+"_"+module.name+"_msgs")

	# insert the package's name and description in CMakeLists.txt's content
	srvMakeFile = makeFile.replace("%PKGNAME%", "hrim_"+module.type+"_"+module.name+"_srvs")

	os.chdir(cwd)

	# if the container directory for the generated module doesn't exist, create it
	if not os.path.exists(cwd+"/generated"):
			os.mkdir("generated")
	os.chdir("generated")
	cwd = os.getcwd()

	# if the module directory for the generated files doesn't exist, create it
	if not os.path.exists(cwd+"/"+module.name):
		os.mkdir(module.name)

	os.chdir(module.name)
	cwd = os.getcwd()

	# list of files for CMakeLists.txt
	msgFiles = []
	srvFiles = []

	# for each topic of the module
	for topic in module.topics:

		# reposition ourselves for each topic
		os.chdir(cwd)

		if topic.type == "publish" or topic.type == "subscribe":

			# check if file has already been generated
			if topic.fileName not in msgFiles:

				messages = True

				# package folder naming
				msgFolderPath = os.getcwd()+"/hrim_"+module.type+"_"+module.name+"_msgs/msg"

				# if the package directories don't exist, create them
				if not os.path.exists(msgFolderPath):
					os.makedirs(msgFolderPath)

				# position ourselves on the package's msg folder
				os.chdir(msgFolderPath)
				msg = ""

				# check for an overall message description
				if topic.desc is not None and len(topic.desc)>0:
					msg+="# "+topic.desc+"\n\n"

				for prop in topic.properties:

					# check for enumeration types
					if prop.unit is not None and prop.unit == "enum":

						# sort enumeration values for readability
						for value in sorted( ((v,k) for k,v in prop.enumeration.iteritems())):
							msg+=prop.type+" "+value[1]+"="+str(value[0])+"\n"

					# process each property, checking if it's value is an array and if it has any description
					# msg+=dataTypes[prop.type]+("[{}] ".format(prop.length if prop.length is not None else "") if prop.array else " ")+prop.name+((" # "+prop.desc) if prop.desc is not None else "")+"\n\n"
					msg+=formatProperty(prop)

				# generate each .msg file and add it to the list
				if topic.fileName is None:
					fileName = topic.name.title()+".msg"
					msgFiles.append(topic.name.title())
				else:
					fileName = topic.fileName+".msg"
					msgFiles.append(topic.fileName)

				text_file = open(fileName, "w")
				text_file.write(msg)
				text_file.close()

		elif topic.type == "service":

			# check if file has already been generated
			if topic.fileName not in srvFiles:

				services = True

				# package folder naming
				srvFolderPath = os.getcwd()+"/hrim_"+module.type+"_"+module.name+"_srvs/srv"

				# if the package directories don't exist, create them
				if not os.path.exists(srvFolderPath):
					os.makedirs(srvFolderPath)

				# position ourselves on the package's srv folder
				os.chdir(srvFolderPath)
				srv = ""

				# check for an overall service description
				if topic.desc is not None and len(topic.desc)>0:
					srv+="# "+topic.desc+"\n\n"

				for prop in topic.properties:

					# check for enumeration types
					if prop.unit is not None and prop.unit == "enum":

						# sort enumeration values for readability
						for value in sorted( ((v,k) for k,v in prop.enumeration.iteritems())):
							srv+=prop.type+" "+value[1]+"="+str(value[0])+"\n"

					# process each property, checking if it's value is an array and if it has any description
					# srv+=dataTypes[prop.type]+("[{}] ".format(prop.length if prop.length is not None else "") if prop.array else " ")+prop.name+((" # "+prop.desc) if prop.desc is not None else "")+"\n\n"
					srv+=formatProperty(prop)

				srv+="---\n"

				for prop in topic.response:
					# check for enumeration types
					if prop.unit is not None and prop.unit == "enum":

						# sort enumeration values for readability
						for value in sorted( ((v,k) for k,v in prop.enumeration.iteritems())):
							srv+=prop.type+" "+value[1]+"="+str(value[0])+"\n"

					# process each property, checking if it's value is an array and if it has any description
					# srv+=dataTypes[prop.type]+("[{}] ".format(prop.length if prop.length is not None else "") if prop.array else " ")+prop.name+((" # "+prop.desc) if prop.desc is not None else "")+"\n\n"
					srv+=formatProperty(prop)

				# generate each .srv file and add it to the list
				if topic.fileName is None:
					fileName = topic.name.title()+".srv"
					srvFiles.append(topic.name.title())
				else:
					fileName = topic.fileName+".srv"
					srvFiles.append(topic.fileName)

				text_file = open(fileName, "w")
				text_file.write(srv)
				text_file.close()

    # if the package has messages
	if messages:
		# reposition ourselves on the package's root
		os.chdir(msgFolderPath[:-4])

		# generate the package.xml file
		package = open("package.xml", "w")
		package.write(msgPkg)
		package.close()

		# insert the .msg list in the CMakeLists.txt
		msgList = ""
		for tmp in msgFiles:
			msgList+="\t\"msg/"+tmp+".msg\"\n"

		msgMakeFile = msgMakeFile.replace("%PKGFILES%", msgList[:-1])

		# generate the CMakeLists.txt file
		cmake = open("CMakeLists.txt", "w")
		cmake.write(msgMakeFile)
		cmake.close()

    # if the package has services
	if services:
		# reposition ourselves on the package's root
		os.chdir(srvFolderPath[:-4])

		# generate the package.xml file
		package = open("package.xml", "w")
		package.write(srvPkg)
		package.close()

		# insert the .srv list in the CMakeLists.txt
		srvList = ""
		for tmp in srvFiles:
			srvList+="\t\"srv/"+tmp+".srv\"\n"

		srvMakeFile = srvMakeFile.replace("%PKGFILES%", srvList[:-1])

		# generate the CMakeLists.txt file
		cmake = open("CMakeLists.txt", "w")
		cmake.write(srvMakeFile)
		cmake.close()

	# reposition ourselves on the generated module's root folder
	os.chdir(cwd)
	manParams = ""
	optParams = ""

	for param in module.params:

		# mandatory parameters parsing
		if param.mandatory:
			if param.desc is not None:
				manParams+=getTabs(1)+"# "+param.desc+"\n"
			manParams+=getTabs(1)+param.name+": "+(param.value if param.value is not None else "")+"\n\n"

		# optional parameters parsing
		else:
			optParams+=getTabs(1)+"# "+param.desc+"\n"
			optParams+=getTabs(1)+param.name+": "+(param.value if param.value is not None else "")+"\n\n"

	if len(manParams)>0:
		params = open("mandatory_parameters.yaml", "w")
		params.write(module.name+":\n"+manParams)
		params.close()

	if len(optParams)>0:
		params = open("optional_parameters.yaml", "w")
		params.write(module.name+":\n"+optParams)
		params.close()
