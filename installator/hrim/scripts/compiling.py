import os
from .utils import getTabs

class ModuleCompiler:
    def processSubProperty(self, prop, type):
        try:
            if self.checkGenerated(prop.fileName, type) is False:
                subMsg = ""
                for subProp in prop.properties:
                    if(subProp.fileName is not None
                       and (subProp.package is None or subProp.package == self.msgPkgName)
                      ):
                        try:
                            self.processSubProperty(subProp, type)
                        except:
                            raise
                    else:
                        if(subProp.unit is not None and subProp.unit == "enum"
                           and len(subProp.enumeration) > 0
                          ):
                            # sort enumeration values for readability
                            for value in sorted(((v, k) for k, v in subProp.enumeration.items())):
                                subMsg += subProp.type+" "+value[1]+"="+str(value[0])+"\n"
                    subMsg += self.formatProperty(subProp, type)
                subFileName = prop.fileName+".msg"

                if len(subMsg) > 0:
                    if self.base:
                        self.baseFiles[self.msgPkgName].append(prop.fileName)

                    self.ownFiles.append(prop.fileName)

                    textFile = open(subFileName, "w")
                    textFile.write(subMsg)
                    textFile.close()
        except:
            print("Error while processing subproperty message: "+prop.fileName)
            raise

    # separate the message generation for messages declared inside services
    def processMessage(self, topic):
        try:
            # check if file has already been generated
            res = self.checkGenerated(topic.fileName, topic.type)
            if res is False:

                # package folder naming
                self.msgFolderPath = os.path.join(os.getcwd(), self.msgPkgName, "msg")

                # if the package directories don't exist, create them
                if not os.path.exists(self.msgFolderPath):
                    os.makedirs(self.msgFolderPath)

                # position ourselves on the package's msg folder
                os.chdir(self.msgFolderPath)
                msg = ""

                # check for an overall message description
                if topic.desc is not None and len(topic.desc) > 0:
                    msg += "# "+topic.desc+"\n\n"

                for prop in topic.properties:

                    if prop.fileName is not None and (
                            prop.package is None or prop.package == self.msgPkgName
                    ):
                        self.processSubProperty(prop, topic.type)
                    else:

                        # check for enumeration types
                        if(prop.unit is not None and prop.unit == "enum"
                           and len(prop.enumeration) > 0
                          ):

                            # sort enumeration values for readability
                            for value in sorted(((v, k) for k, v in prop.enumeration.items())):
                                msg += prop.type+" "+value[1]+"="+str(value[0])+"\n"

                    # process each property, checking if it's value is an array
                    # and if it has any description
                    msg += self.formatProperty(prop, topic.type)

                    if prop.package is not None:
                        if prop.package not in self.msgDeps:
                            self.msgDeps.append(prop.package)

                # generate each .msg file and add it to the list
                if topic.fileName is None:
                    fileName = topic.name.title()+".msg"
                    self.ownFiles.append(topic.name.title())
                else:
                    fileName = topic.fileName+".msg"
                    self.ownFiles.append(topic.fileName)

                if len(msg) > 0:
                    textFile = open(fileName, "w")
                    textFile.write(msg)
                    textFile.close()
        except:
            print("Error while processing topic message: "+topic.fileName)
            raise

    def formatProperty(self, prop, type):
        if prop.fileName is None:
            type = self.dataTypes[prop.type]
        else:
            if prop.package is not None:
                type = prop.package+"/"+prop.fileName
            else:
                found = self.checkGenerated(prop.fileName, type)
                if found != False:
                    type = found+"/"+prop.fileName
                else:
                    type = self.msgPkgName+"/"+prop.fileName
        length = prop.length if prop.length is not None else ""
        ret = type
        ret += ("[{}]".format(length) if prop.array else "")
        ret += " "+prop.name+((" # "+prop.desc) if prop.desc is not None else "")+"\n\n"
        return ret

    def checkGenerated(self, message, type):
        res = False
        for key, list in self.baseFiles.items():
            if message in list:
                res = key
        if res is False:
            if message in self.ownFiles:
                res = self.msgPkgName

        if(res != False and res != self.msgPkgName
           and type in ["publish", "subscribe"] and res not in self.msgDeps
          ):
            self.msgDeps.append(res)

        return res

    def compileModule(self, module, base=False):
        self.base = base
        messages = False
        services = False
        actions = False
        self.msgDeps = []
        self.srvDeps = []
        self.actionDeps = []
        self.ownFiles = []
        srvFiles = []
        actionFiles = []

        # Actions will always depend on action_msgs
        self.actionDeps.append("action_msgs")

        cwd = os.getcwd()

        if self.base:
            self.msgPkgName = "hrim_"+module.name+"_msgs"
            self.baseFiles[self.msgPkgName] = []
        else:
            if(module.type in
               ["actuator", "sensor", "communication", "cognition", "ui", "power", "composite"]
              ):
                self.msgPkgName = "hrim_"+module.type+"_"+module.name+"_msgs"
            else:
                self.msgPkgName = "hrim_"+module.name+"_msgs"

        os.chdir("templates")
        with open('package.txt', 'r') as myfile:
            pkgFile = myfile.read()
        with open('cmake.txt', 'r') as myfile:
            makeFile = myfile.read()

        if self.composition or self.base:
            cwd = os.path.join(cwd, self.genPath, module.type)
        else:
            cwd = os.path.join(cwd, self.genPath, module.type, module.name)

        pkgPath = os.path.join(cwd, self.msgPkgName)

        # if the directory doesn't exist, create it
        if not os.path.exists(pkgPath):
            os.makedirs(pkgPath)

        for topic in module.topics:
            # reposition ourselves for each topic
            os.chdir(cwd)

            if topic.type == "publish" or topic.type == "subscribe":
                messages = True
                self.processMessage(topic)
                if self.base:
                    self.baseFiles[self.msgPkgName].append(topic.fileName)
            else:
                if topic.type == "service":
                    myDep = self.srvDeps
                    myFiles = srvFiles
                    shortType = "srv"
                    if(self.base or module.type not in
                       ["actuator", "sensor", "communication",
                        "cognition", "ui", "power", "composite"
                       ]
                      ):
                        srvPkgName = "hrim_"+module.name+"_"+shortType+"s"
                    else:
                        srvPkgName = "hrim_"+module.type+"_"+module.name+"_"+shortType+"s"
                    pkgName = srvPkgName
                    srvFolderPath = os.path.join(os.getcwd(), srvPkgName, shortType)
                    folderPath = srvFolderPath
                if topic.type == "action":
                    myDep = self.actionDeps
                    myFiles = actionFiles
                    shortType = "action"
                    if(self.base or module.type not in
                       ["actuator", "sensor", "communication",
                        "cognition", "ui", "power", "composite"
                       ]
                      ):
                        actionPkgName = "hrim_"+module.name+"_"+shortType+"s"
                    else:
                        actionPkgName = "hrim_"+module.type+"_"+module.name+"_"+shortType+"s"
                    pkgName = actionPkgName
                    actionFolderPath = os.path.join(os.getcwd(), actionPkgName, shortType)
                    folderPath = actionFolderPath
                # check if file has already been generated
                if(topic.fileName not in myFiles
                   and (topic.package is None or topic.package == pkgName)
                  ):

                    # if the package directories don't exist, create them
                    if not os.path.exists(folderPath):
                        os.makedirs(folderPath)

                    # position ourselves on the package's action folder
                    os.chdir(folderPath)
                    fileContent = ""

                    # check for an overall file description
                    if topic.desc is not None and len(topic.desc) > 0:
                        fileContent += "# "+topic.desc+"\n\n"

                    for prop in topic.properties:

                        if(prop.fileName is not None and
                           (prop.package is None or prop.package == self.msgPkgName)
                          ):
                            os.chdir(cwd)
                            self.processMessage(prop)
                            os.chdir(folderPath)
                        # check for enumeration types
                        if(prop.unit is not None and prop.unit == "enum"
                           and len(prop.enumeration) > 0
                          ):

                            # sort enumeration values for readability
                            for value in sorted(((v, k) for k, v in prop.enumeration.items())):
                                fileContent += prop.type+" "+value[1]+"="+str(value[0])+"\n"

                        # process each property, checking if it's value is an array
                        # and if it has any description
                        fileContent += self.formatProperty(prop, topic.type)

                        if prop.package is not None:
                            if prop.package not in myDep:
                                myDep.append(prop.package)

                    fileContent += "---\n"

                    for prop in topic.response:
                        if(prop.fileName is not None
                           and (prop.package is None or prop.package == self.msgPkgName)
                          ):
                            os.chdir(cwd)
                            self.processMessage(prop)
                            os.chdir(folderPath)

                        # check for enumeration types
                        if(prop.unit is not None and prop.unit == "enum"
                           and len(prop.enumeration) > 0
                          ):

                            # sort enumeration values for readability
                            for value in sorted(((v, k) for k, v in prop.enumeration.items())):
                                fileContent += prop.type+" "+value[1]+"="+str(value[0])+"\n"

                        # process each property, checking if it's value is an array
                        # and if it has any description
                        fileContent += self.formatProperty(prop, topic.type)

                        if prop.package is not None:
                            if prop.package not in myDep:
                                myDep.append(prop.package)

                    if topic.type == "action":
                        fileContent += "---\n"

                        for prop in topic.feedback:
                            if(prop.fileName is not None
                               and (prop.package is None or prop.package == self.msgPkgName)
                              ):
                                os.chdir(cwd)
                                self.processMessage(prop)
                                os.chdir(folderPath)

                            # check for enumeration types
                            if(prop.unit is not None and prop.unit == "enum"
                               and len(prop.enumeration) > 0
                              ):

                                # sort enumeration values for readability
                                for value in sorted(((v, k) for k, v in prop.enumeration.items())):
                                    fileContent += prop.type+" "+value[1]+"="+str(value[0])+"\n"

                            # process each property, checking if it's value is an array
                            # and if it has any description
                            fileContent += self.formatProperty(prop, topic.type)

                            if prop.package is not None:
                                if prop.package not in myDep:
                                    myDep.append(prop.package)

                    # generate each .action file and add it to the list
                    if topic.fileName is None:
                        fileName = topic.name.title()+"."+shortType
                        myFiles.append(topic.name.title())
                    else:
                        fileName = topic.fileName+"."+shortType
                        myFiles.append(topic.fileName)

                    textFile = open(fileName, "w")
                    textFile.write(fileContent)
                    textFile.close()

                    if topic.type == "service":
                        services = True
                    if topic.type == "action":
                        actions = True

        if self.msgPkgName in self.msgDeps:
            self.msgDeps.remove(self.msgPkgName)
        if "hrim_generic_msgs" in self.msgDeps:
            self.msgDeps.remove("hrim_generic_msgs")

        buildDeps = ""
        execDeps = ""
        pkgFind = ""
        pkgDep = ""
        for pkgName in self.msgDeps:
            buildDeps = buildDeps+("\n\t<build_depend>{}</build_depend>").format(pkgName)
            execDeps = execDeps+("\n\t<exec_depend>{}</exec_depend>").format(pkgName)
            pkgFind = pkgFind+("\nfind_package({} REQUIRED)").format(pkgName)
            pkgDep = pkgDep+("\n\t\t{}").format(pkgName)

        # insert the package's name and description in package.xml's content
        msgPkg = pkgFile.replace("%PKGNAME%", self.msgPkgName)
        msgPkg = msgPkg.replace("%PKGDESC%", module.desc)
        msgPkg = msgPkg.replace("%PKGBUILD%", buildDeps)
        msgPkg = msgPkg.replace("%PKGEXEC%", execDeps)

        # insert the package's name and description in CMakeLists.txt's content
        msgMakeFile = makeFile.replace("%PKGNAME%", self.msgPkgName)
        msgMakeFile = msgMakeFile.replace("%PKGFIND%", pkgFind)

        if len(pkgDep) > 0:
            pkgDep = "\n\tDEPENDENCIES"+pkgDep

        msgMakeFile = msgMakeFile.replace("%PKGDEP%", pkgDep)

        # if the package has messages
        if messages:
            self.ownFiles = os.listdir(self.msgFolderPath)
            # reposition ourselves on the package's root
            os.chdir(self.msgFolderPath[:-4])

            # generate the package.xml file
            package = open("package.xml", "w")
            package.write(msgPkg)
            package.close()

            # insert the .msg list in the CMakeLists.txt
            msgList = ""
            for tmp in sorted(self.ownFiles):
                msgList += "\t\"msg/"+tmp+"\"\n"

            msgMakeFile = msgMakeFile.replace("%PKGFILES%", msgList[:-1])

            # generate the CMakeLists.txt file
            cmake = open("CMakeLists.txt", "w")
            cmake.write(msgMakeFile)
            cmake.close()

        # if the package has services
        if services:
            srvFiles = os.listdir(srvFolderPath)
            # reposition ourselves on the package's root
            os.chdir(srvFolderPath[:-4])

            srvMakeFile = makeFile.replace("%PKGNAME%", srvPkgName)
            srvPkg = pkgFile.replace("%PKGNAME%", srvPkgName)

            srvPkg = srvPkg.replace("%PKGDESC%", module.desc)

            if len(self.srvDeps) > 0:
                buildDeps = ""
                execDeps = ""
                pkgFind = ""
                pkgDep = ""
                for dependency in self.srvDeps:
                    buildDeps = buildDeps+("\n\t<build_depend>{}</build_depend>").format(dependency)
                    execDeps = execDeps+("\n\t<exec_depend>{}</exec_depend>").format(dependency)
                    pkgFind = pkgFind+("\nfind_package({} REQUIRED)").format(dependency)
                    pkgDep = pkgDep+("\n\t\t{}").format(dependency)
                srvMakeFile = srvMakeFile.replace("%PKGFIND%", pkgFind)
                srvMakeFile = srvMakeFile.replace("%PKGDEP%", "\n  DEPENDENCIES"+pkgDep)
                srvPkg = srvPkg.replace("%PKGBUILD%", buildDeps)
                srvPkg = srvPkg.replace("%PKGEXEC%", execDeps)
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
            for tmp in sorted(srvFiles):
                srvList += "\t\"srv/"+tmp+"\"\n"

            srvMakeFile = srvMakeFile.replace("%PKGFILES%", srvList[:-1])

            # generate the CMakeLists.txt file
            cmake = open("CMakeLists.txt", "w")
            cmake.write(srvMakeFile)
            cmake.close()

        # if the package has actions
        if actions:
            actionFiles = os.listdir(actionFolderPath)
            # reposition ourselves on the package's root
            os.chdir(actionFolderPath[:-7])

            actionMakeFile = makeFile.replace("%PKGNAME%", actionPkgName)
            actionPkg = pkgFile.replace("%PKGNAME%", actionPkgName)

            actionPkg = actionPkg.replace("%PKGDESC%", module.desc)

            if len(self.actionDeps) > 0:
                buildDeps = ""
                execDeps = ""
                pkgFind = ""
                pkgDep = ""
                for dependency in self.actionDeps:
                    buildDeps = buildDeps+("\n\t<build_depend>{}</build_depend>").format(dependency)
                    execDeps = execDeps+("\n\t<exec_depend>{}</exec_depend>").format(dependency)
                    pkgFind = pkgFind+("\nfind_package({} REQUIRED)").format(dependency)
                    pkgDep = pkgDep+("\n\t\t{}").format(dependency)
                actionMakeFile = actionMakeFile.replace("%PKGFIND%", pkgFind)
                actionMakeFile = actionMakeFile.replace("%PKGDEP%", "\n  DEPENDENCIES"+pkgDep)
                actionPkg = actionPkg.replace("%PKGBUILD%", buildDeps)
                actionPkg = actionPkg.replace("%PKGEXEC%", execDeps)
            else:
                actionMakeFile = actionMakeFile.replace("%PKGFIND%", "")
                actionMakeFile = actionMakeFile.replace("%PKGDEP%", "")
                actionPkg = actionPkg.replace("%PKGBUILD%", "")
                actionPkg = actionPkg.replace("%PKGEXEC%", "")

            # generate the package.xml file
            package = open("package.xml", "w")
            package.write(actionPkg)
            package.close()

            # insert the .action list in the CMakeLists.txt
            actionList = ""
            for tmp in sorted(actionFiles):
                actionList += "\t\"action/"+tmp+"\"\n"

            actionMakeFile = actionMakeFile.replace("%PKGFILES%", actionList[:-1])

            # generate the CMakeLists.txt file
            cmake = open("CMakeLists.txt", "w")
            cmake.write(actionMakeFile)
            cmake.close()

        manParams = ""
        optParams = ""

        for param in module.params:

            # mandatory parameters parsing
            if param.mandatory:
                if param.desc is not None:
                    manParams += getTabs(1)+"# "+param.desc+"\n"
                manParams += getTabs(1)+param.name+": "
                manParams += (str(param.value) if param.value is not None else "")+"\n\n"

            # optional parameters parsing
            else:
                if param.desc is not None:
                    optParams += getTabs(1)+"# "+param.desc+"\n"
                optParams += getTabs(1)+param.name+": "
                optParams += (str(param.value) if param.value is not None else "")+"\n\n"

        if len(manParams) > 0:
            self.manParams = self.manParams+module.name+":\n"+manParams

        if len(optParams) > 0:
            self.optParams = self.optParams+module.name+":\n"+optParams

        os.chdir(cwd)

    def generateParameters(self):
        if len(self.manParams) > 0:
            params = open("mandatory_parameters.yaml", "w")
            params.write(self.manParams)
            params.close()
            self.manParams = ""
        if len(self.optParams) > 0:
            params = open("optional_parameters.yaml", "w")
            params.write(self.optParams)
            params.close()
            self.optParams = ""

    def composeModule(self, modules, paths):
        model = open("model.xml", "w")
        composition = "<?xml version=\"1.0\"?>\n<composition name=\"defaultName\">\n"
        strContent = ""
        index = 0
        for module in modules:
            ownTopics = []
            ownParams = []
            strTopics = getTabs(1)+"<model type=\"{}\" subtype=\"{}\" path=\"{}\">\n".format(
                module.type, module.name, paths[index]
            )
            strParams = ""
            for topic in module.topics:
                if not topic.mandatory:
                    ownTopics.append(topic)
            for param in module.params:
                if not param.mandatory:
                    ownParams.append(param)
            ownTopics.sort(key=lambda topic: topic.name)
            ownParams.sort(key=lambda param: param.name)
            for topic in ownTopics:
                strTopics = strTopics+(getTabs(2)+"<topic name=\"{}\"/>\n").format(topic.name)
            for param in ownParams:
                strParams = strParams+(getTabs(2)+"<param name=\"{}\"/>\n").format(param.name)
            strTopics = strTopics+strParams+getTabs(1)+"</model>"
            strContent = strContent+strTopics+"\n"
            index = index+1
        composition = composition+strContent[:-1]+"\n</composition>"
        model.write(composition)
        model.close()

    def __init__(self):
        self.dataTypes = {}
        self.msgPkgName = None
        self.baseFiles = {}
        self.composition = False
        self.ownFiles = None
        self.msgFolderPath = None
        self.generatedFiles = {}
        self.initPath = os.getcwd()
        self.genPath = None
        self.msgDeps = []
        self.srvDeps = []
        self.actionDeps = []
        self.manParams = ""
        self.optParams = ""
        self.base = False
