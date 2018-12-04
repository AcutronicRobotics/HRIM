import lxml.etree as ET
import sys
import os

class ModuleCompiler:
	# shorthand for yaml tabulation
	def getTabs(self, n):
		str=""
		for x in range(n):
			str+="  "
		return str

	def processSubProperty(self, prop):
		if prop.fileName not in self.msgFiles:
			subMsg = ""
			for subProp in prop.properties:
				if subProp.fileName is not None:
					self.processSubProperty(subProp)
				else:
					if subProp.unit is not None and subProp.unit == "enum":

						# sort enumeration values for readability
						for value in sorted( ((v,k) for k,v in subProp.enumeration.iteritems())):
							subMsg+=subProp.type+" "+value[1]+"="+str(value[0])+"\n"
				subMsg+=self.formatProperty(subProp)
			subFileName = prop.fileName+".msg"
			self.msgFiles.append(prop.fileName)

			text_file = open(subFileName, "w")
			text_file.write(subMsg)
			text_file.close()

	# separate the message generation for messages declared inside services
	def processMessage(self, module, topic):

		# check if file has already been generated
		if topic.fileName not in self.msgFiles:

			# package folder naming
			self.msgFolderPath = os.getcwd()+"/hrim_"+module.type+"_"+module.name+"_msgs/msg"

			# if the package directories don't exist, create them
			if not os.path.exists(self.msgFolderPath):
				os.makedirs(self.msgFolderPath)

			# position ourselves on the package's msg folder
			os.chdir(self.msgFolderPath)
			msg = ""

			# check for an overall message description
			if topic.desc is not None and len(topic.desc)>0:
				msg+="# "+topic.desc+"\n\n"

			for prop in topic.properties:

				if prop.fileName is not None:
					self.processSubProperty(prop)
				else:

					# check for enumeration types
					if prop.unit is not None and prop.unit == "enum":

						# sort enumeration values for readability
						for value in sorted( ((v,k) for k,v in prop.enumeration.iteritems())):
							msg+=prop.type+" "+value[1]+"="+str(value[0])+"\n"

				# process each property, checking if it's value is an array and if it has any description
				msg+=self.formatProperty(prop)

			# generate each .msg file and add it to the list
			if topic.fileName is None:
				fileName = topic.name.title()+".msg"
				self.msgFiles.append(topic.name.title())
			else:
				fileName = topic.fileName+".msg"
				self.msgFiles.append(topic.fileName)

			text_file = open(fileName, "w")
			text_file.write(msg)
			text_file.close()

	def formatProperty(self, prop):
		if prop.fileName is None:
			type = self.dataTypes[prop.type]
		else:
			type = self.msgPkgName+"/"+prop.fileName
		return type+("[{}] ".format(prop.length if prop.length is not None else "") if prop.array else " ")+prop.name+((" # "+prop.desc) if prop.desc is not None else "")+"\n\n"

	def compileModule(self, module):

		messages = False
		services = False
		dependency = False

		# default platform
		plat = "ros2"

		# map the datatypes
		self.dataTypes = {}
		cwd = os.getcwd()
		dataTree = ET.parse(cwd+"/models/dataMapping.xml")
		dataRoot = dataTree.getroot()

		# check for the platform
		if any(platform.attrib.get("name") == plat for platform in dataRoot):
			for platform in dataRoot.iter("platform"):
				if platform.attrib.get("name") == plat:
					for type in platform.iter("type"):
						self.dataTypes[type.attrib.get("name")] = type.attrib.get("value")
		else:
			print "Chosen platform doesn't exist"
			sys.exit(1)

		# parse the templates necessary for the package
		os.chdir("templates")
		with open('package.txt', 'r') as myfile:
			pkg=myfile.read()

		# insert the package's name and description in package.xml's content
		self.msgPkgName = "hrim_"+module.type+"_"+module.name+"_msgs"
		msgPkg = pkg.replace("%PKGNAME%", self.msgPkgName)
		msgPkg = msgPkg.replace("%PKGDESC%", module.desc)
		msgPkg = msgPkg.replace("%PKGBUILD%", "")
		msgPkg = msgPkg.replace("%PKGEXEC%", "")

		# insert the package's name and description in package.xml's content
		srvPkg = pkg.replace("%PKGNAME%", "hrim_"+module.type+"_"+module.name+"_srvs")
		srvPkg = srvPkg.replace("%PKGDESC%", module.desc)

		with open('cmake.txt', 'r') as myfile:
			makeFile=myfile.read()

		# insert the package's name and description in CMakeLists.txt's content
		msgMakeFile = makeFile.replace("%PKGNAME%", self.msgPkgName)
		msgMakeFile = msgMakeFile.replace("%PKGFIND%", "")
		msgMakeFile = msgMakeFile.replace("%PKGDEP%", "")

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
		self.msgFolderPath = ""
		self.msgFiles = []
		srvFiles = []

		# for each topic of the module
		for topic in module.topics:

			# reposition ourselves for each topic
			os.chdir(cwd)

			if topic.type == "publish" or topic.type == "subscribe":
				messages = True
				self.processMessage(module, topic)

			elif topic.type == "service":

				services = True

				# check if file has already been generated
				if topic.fileName not in srvFiles:

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

						if prop.fileName is not None:
							dependency = True
							os.chdir(cwd)
							self.processMessage(module, prop)
							os.chdir(srvFolderPath)
						# check for enumeration types
						if prop.unit is not None and prop.unit == "enum":

							# sort enumeration values for readability
							for value in sorted( ((v,k) for k,v in prop.enumeration.iteritems())):
								srv+=prop.type+" "+value[1]+"="+str(value[0])+"\n"

						# process each property, checking if it's value is an array and if it has any description
						srv+=self.formatProperty(prop)

					srv+="---\n"

					for prop in topic.response:
						# check for enumeration types
						if prop.unit is not None and prop.unit == "enum":

							# sort enumeration values for readability
							for value in sorted( ((v,k) for k,v in prop.enumeration.iteritems())):
								srv+=prop.type+" "+value[1]+"="+str(value[0])+"\n"

						# process each property, checking if it's value is an array and if it has any description
						srv+=self.formatProperty(prop)

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
			os.chdir(self.msgFolderPath[:-4])

			# generate the package.xml file
			package = open("package.xml", "w")
			package.write(msgPkg)
			package.close()

			# insert the .msg list in the CMakeLists.txt
			msgList = ""
			for tmp in self.msgFiles:
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

			if dependency:
				srvMakeFile = srvMakeFile.replace("%PKGFIND%", "\nfind_package({} REQUIRED)".format(self.msgPkgName))
				srvMakeFile = srvMakeFile.replace("%PKGDEP%", "\n\t"+self.msgPkgName)
				srvPkg = srvPkg.replace("%PKGBUILD%", "\n\t<build_depend>{}</build_depend>".format(self.msgPkgName))
				srvPkg = srvPkg.replace("%PKGEXEC%", "\n\t<exec_depend>{}</exec_depend>".format(self.msgPkgName))
			else:
				srvMakeFile = srvMakeFile.replace("%PKGFIND%", "")
				srvMakeFile = srvMakeFile.replace("%PKGDEP%", "")
				srvPkg = srvPkg.replace("%PKGBUILD%", "")
				srvPkg = srvPkg.replace("%PKGEXEC%", "")

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
					manParams+=self.getTabs(1)+"# "+param.desc+"\n"
				manParams+=self.getTabs(1)+param.name+": "+(str(param.value) if param.value is not None else "")+"\n\n"

			# optional parameters parsing
			else:
				if param.desc is not None:
					optParams+=self.getTabs(1)+"# "+param.desc+"\n"
				optParams+=self.getTabs(1)+param.name+": "+(str(param.value) if param.value is not None else "")+"\n\n"

		if len(manParams)>0:
			params = open("mandatory_parameters.yaml", "w")
			params.write(module.name+":\n"+manParams)
			params.close()

		if len(optParams)>0:
			params = open("optional_parameters.yaml", "w")
			params.write(module.name+":\n"+optParams)
			params.close()

	def __init__(self):
		self.dataTypes = None
		self.msgPkgName = None
		self.msgFiles = None
		self.msgFolderPath = None
