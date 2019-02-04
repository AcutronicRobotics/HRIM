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

class ModuleCompiler:
    def processSubProperty(self, prop, generic):
        try:
            if prop.fileName not in self.msgFiles and prop.fileName not in self.ownFiles and self.checkGenerated(prop.fileName) == False:
                subMsg = ""
                for subProp in prop.properties:
                    if subProp.fileName is not None:
                        try:
                            if prop.fileName in ["Header", "Time"]:
                                self.processSubProperty(subProp, True)
                            else:
                                self.processSubProperty(subProp, False)
                        except:
                            raise
                    else:
                        if subProp.unit is not None and subProp.unit == "enum" and len(subProp.enumeration) > 0:
                            # sort enumeration values for readability
                            for value in sorted( ((v,k) for k,v in subProp.enumeration.items())):
                                subMsg+=subProp.type+" "+value[1]+"="+str(value[0])+"\n"
                    subMsg+=self.formatProperty(subProp)
                subFileName = prop.fileName+".msg"
                if generic:
                    self.msgFiles.append(prop.fileName)
                else:
                    self.ownFiles.append(prop.fileName)
                    
                text_file = open(subFileName, "w")
                text_file.write(subMsg)
                text_file.close()
        except:
            print("Error while processing subproperty message: "+prop.fileName)
            raise

    # separate the message generation for messages declared inside services
    def processMessage(self, module, topic):
        try:
            # check if file has already been generated
            if topic.fileName not in self.msgFiles and topic.fileName not in self.ownFiles:

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
                        self.processSubProperty(prop, False)
                    else:

                        # check for enumeration types
                        if prop.unit is not None and prop.unit == "enum" and len(prop.enumeration) > 0:

                            # sort enumeration values for readability
                            for value in sorted( ((v,k) for k,v in prop.enumeration.items())):
                                msg+=prop.type+" "+value[1]+"="+str(value[0])+"\n"

                    # process each property, checking if it's value is an array and if it has any description
                    msg+=self.formatProperty(prop)

                # generate each .msg file and add it to the list
                if topic.fileName is None:
                    fileName = topic.name.title()+".msg"
                    self.ownFiles.append(topic.name.title())
                else:
                    fileName = topic.fileName+".msg"
                    self.ownFiles.append(topic.fileName)

                text_file = open(fileName, "w")
                text_file.write(msg)
                text_file.close()
        except:
            print("Error while processing topic message: "+topic.fileName)
            raise

    def formatProperty(self, prop):
        if prop.fileName is None:
            type = self.dataTypes[prop.type]
        else:
            found = self.checkGenerated(prop.fileName)
            if found != False:
                type = found+"/"+prop.fileName
            else:
                type = self.msgPkgName+"/"+prop.fileName
        return type+("[{}] ".format(prop.length if prop.length is not None else "") if prop.array else " ")+prop.name+((" # "+prop.desc) if prop.desc is not None else "")+"\n\n"

    def compileGeneric(self, topics, plat, package="generic"):
        try:
            cwd = os.getcwd()

            self.msgPkgName = "hrim_"+package+"_msgs"

            self.dataTypes = ModuleParser().getDataTypes(plat)

            os.chdir("templates")
            with open('package.txt', 'r') as myfile:
                pkg=myfile.read()

            with open('cmake.txt', 'r') as myfile:
                makeFile=myfile.read()

            os.chdir(cwd)

            genPath = os.path.join(cwd, "generated", package, self.msgPkgName, "msg")

            # if the generic package directory doesn't exist, create it
            if not os.path.exists(genPath):
                os.makedirs(genPath)

            os.chdir(genPath)

            self.ownFiles = []

            for topic in topics:

                # reposition ourselves for each topic
                os.chdir(genPath)

                msg = ""

                # check for an overall message description
                if topic.desc is not None and len(topic.desc)>0:
                    msg+="# "+topic.desc+"\n\n"

                for prop in topic.properties:

                    if prop.fileName is not None:
                        self.processSubProperty(prop, True)
                    else:

                        # check for enumeration types
                        if prop.unit is not None and prop.unit == "enum":

                            # sort enumeration values for readability
                            for value in sorted( ((v,k) for k,v in prop.enumeration.items())):
                                msg+=prop.type+" "+value[1]+"="+str(value[0])+"\n"

                    # process each property, checking if it's value is an array and if it has any description
                    msg+=self.formatProperty(prop)

                genName = topic.fileName if topic.fileName is not None else topic.name.title()
                checkFile = self.checkGenerated(genName)

                if checkFile == False:
                    fileName = genName+".msg"
                    self.msgFiles.append(genName)
                    text_file = open(fileName, "w")
                    text_file.write(msg)
                    text_file.close()
                else:
                    print(checkFile)

            self.pkgDeps.remove(self.msgPkgName)

            # update the list of generated files at self.generatedFiles 
            self.listGenerated()

            # insert the .msg list in the CMakeLists.txt
            msgList = ""
            for tmp in sorted(self.generatedFiles[self.msgPkgName]):
                msgList+="\t\"msg/"+tmp+".msg\"\n"

            buildDeps = ""
            execDeps = ""
            pkgFind = ""
            pkgDep = ""
            for pkgName in self.pkgDeps:
                buildDeps = buildDeps+("\n\t<build_depend>{}</build_depend>").format(pkgName)
                execDeps = execDeps+("\n\t<exec_depend>{}</exec_depend>").format(pkgName)
                pkgFind = pkgFind+("\nfind_package({} REQUIRED)").format(pkgName)
                pkgDep = pkgDep+("\n\t{}").format(pkgName)

            # insert the package's name and description in CMakeLists.txt's content
            msgMakeFile = makeFile.replace("%PKGNAME%", self.msgPkgName)
            msgMakeFile = msgMakeFile.replace("%PKGFIND%", pkgFind)
            msgMakeFile = msgMakeFile.replace("%PKGDEP%", pkgDep)

            msgMakeFile = msgMakeFile.replace("%PKGFILES%", msgList[:-1])


            genPkg = pkg.replace("%PKGNAME%", self.msgPkgName)
            genPkg = genPkg.replace("%PKGDESC%", "defines the "+package+" HRIM messages used by modules")
            genPkg = genPkg.replace("%PKGBUILD%", buildDeps)
            genPkg = genPkg.replace("%PKGEXEC%", execDeps)

            os.chdir("..")

            # generate the CMakeLists.txt file
            cmake = open("CMakeLists.txt", "w")
            cmake.write(msgMakeFile)
            cmake.close()

            # generate the package.xml file
            package = open("package.xml", "w")
            package.write(genPkg)
            package.close()

        except Exception as e:
            print("Error while compiling generic package")
            raise

    def listGenerated(self):
        cwd = os.getcwd()
        os.chdir(self.initPath)
        genList = {}
        if os.path.exists("generated"):
            pkgList = os.listdir("generated")
            for package in pkgList:
                pkgName = os.path.split(package)[-1]
                genList["hrim_"+pkgName+"_msgs"] = []
                genPath = os.path.join(self.initPath,"generated", pkgName, "hrim_"+pkgName+"_msgs", "msg")
                if os.path.exists(genPath):
                    msgList = os.listdir(genPath)
                    for message in msgList:
                        genList["hrim_"+pkgName+"_msgs"].append(message.split(".")[0])
        os.chdir(cwd)
        self.generatedFiles = genList
        return genList

    def checkGenerated(self, message):
        res = False
        genList = self.listGenerated()
        for key, list in genList.items():
            if message in list:
                res = key

        if res != False and res not in self.pkgDeps:
            self.pkgDeps.append(res)

        return res

    def compileModule(self, module, plat):
        try:
            messages = False
            services = False
            dependency = False
            self.pkgDeps = []

            cwd = os.getcwd()

            self.dataTypes = ModuleParser().getDataTypes(plat)
            self.msgPkgName = "hrim_"+module.type+"_"+module.name+"_msgs"

            # parse the templates necessary for the package
            os.chdir("templates")
            with open('package.txt', 'r') as myfile:
                pkg=myfile.read()

            # insert the package's name and description in package.xml's content
            srvPkg = pkg.replace("%PKGNAME%", "hrim_"+module.type+"_"+module.name+"_srvs")
            srvPkg = srvPkg.replace("%PKGDESC%", module.desc)

            with open('cmake.txt', 'r') as myfile:
                makeFile=myfile.read()

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
            self.ownFiles = []
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
                            if prop.unit is not None and prop.unit == "enum" and len(prop.enumeration) > 0:

                                # sort enumeration values for readability
                                for value in sorted( ((v,k) for k,v in prop.enumeration.items())):
                                    srv+=prop.type+" "+value[1]+"="+str(value[0])+"\n"

                            # process each property, checking if it's value is an array and if it has any description
                            srv+=self.formatProperty(prop)

                        srv+="---\n"

                        for prop in topic.response:
                            # check for enumeration types
                            if prop.unit is not None and prop.unit == "enum":

                                # sort enumeration values for readability
                                for value in sorted( ((v,k) for k,v in prop.enumeration.items())):
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

            buildDeps = ""
            execDeps = ""
            pkgFind = ""
            pkgDep = ""
            for pkgName in self.pkgDeps:
                buildDeps = buildDeps+("\n\t<build_depend>{}</build_depend>").format(pkgName)
                execDeps = execDeps+("\n\t<exec_depend>{}</exec_depend>").format(pkgName)
                pkgFind = pkgFind+("\nfind_package({} REQUIRED)").format(pkgName)
                pkgDep = pkgDep+("\n\t{}").format(pkgName)

            # insert the package's name and description in package.xml's content
            msgPkg = pkg.replace("%PKGNAME%", self.msgPkgName)
            msgPkg = msgPkg.replace("%PKGDESC%", module.desc)
            msgPkg = msgPkg.replace("%PKGBUILD%", buildDeps)
            msgPkg = msgPkg.replace("%PKGEXEC%", execDeps)

            # insert the package's name and description in CMakeLists.txt's content
            msgMakeFile = makeFile.replace("%PKGNAME%", self.msgPkgName)
            msgMakeFile = msgMakeFile.replace("%PKGFIND%", pkgFind)
            msgMakeFile = msgMakeFile.replace("%PKGDEP%", pkgDep)

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
                for tmp in self.ownFiles:
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
                        manParams+=getTabs(1)+"# "+param.desc+"\n"
                    manParams+=getTabs(1)+param.name+": "+(str(param.value) if param.value is not None else "")+"\n\n"

                # optional parameters parsing
                else:
                    if param.desc is not None:
                        optParams+=getTabs(1)+"# "+param.desc+"\n"
                    optParams+=getTabs(1)+param.name+": "+(str(param.value) if param.value is not None else "")+"\n\n"

            if len(manParams)>0:
                params = open("mandatory_parameters.yaml", "w")
                params.write(module.name+":\n"+manParams)
                params.close()

            if len(optParams)>0:
                params = open("optional_parameters.yaml", "w")
                params.write(module.name+":\n"+optParams)
                params.close()
        except:
            print("Error while processing module: "+module.name)
            raise

    def __init__(self):
        self.dataTypes = {}
        self.msgPkgName = None
        self.msgFiles = []
        self.ownFiles = None
        self.msgFolderPath = None
        self.generatedFiles = {}
        self.initPath = os.getcwd()
        self.pkgDeps = []
