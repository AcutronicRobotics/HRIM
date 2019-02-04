import sys
import os
from utils import getTabs

class ModuleCompiler:
    def processSubProperty(self, prop):
        try:
            if self.checkGenerated(prop.fileName) == False:
                subMsg = ""
                for subProp in prop.properties:
                    if subProp.fileName is not None:
                        try:
                            self.processSubProperty(subProp)
                        except:
                            raise
                    else:
                        if subProp.unit is not None and subProp.unit == "enum" and len(subProp.enumeration) > 0:
                            # sort enumeration values for readability
                            for value in sorted( ((v,k) for k,v in subProp.enumeration.items())):
                                subMsg+=subProp.type+" "+value[1]+"="+str(value[0])+"\n"
                    subMsg+=self.formatProperty(subProp)
                subFileName = prop.fileName+".msg"

                if len(subMsg) > 0:
                    if self.base:
                        self.baseFiles[self.msgPkgName].append(prop.fileName)

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
            res = self.checkGenerated(topic.fileName)
            if res == False:

                # package folder naming
                self.msgFolderPath = os.path.join(os.getcwd(),self.msgPkgName,"msg")

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

                if len(msg) > 0:
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

    def checkGenerated(self, message):
        res = False
        for key, list in self.baseFiles.items():
            if message in list:
                    res = key
        if res == False:
            if message in self.ownFiles:
                res=self.msgPkgName

        if res != False and res != self.msgPkgName and res not in self.pkgDeps:
            self.pkgDeps.append(res)

        return res

    def compileModule(self, module, base):
        self.base = base
        messages = False
        services = False
        dependency = False
        self.pkgDeps = []
        self.ownFiles = []
        srvFiles = []

        cwd = os.getcwd()

        if self.base:
            self.msgPkgName = "hrim_"+module.name+"_msgs"
            self.baseFiles[self.msgPkgName] = []
        else:
            self.msgPkgName = "hrim_"+module.type+"_"+module.name+"_msgs"

        os.chdir("templates")
        with open('package.txt', 'r') as myfile:
            pkgFile=myfile.read()
        with open('cmake.txt', 'r') as myfile:
            makeFile=myfile.read()

        if self.composition:
            cwd = os.path.join(cwd, self.genPath)
        else:
            cwd = os.path.join(cwd, self.genPath, module.name)

        pkgPath = os.path.join(cwd, self.msgPkgName)

        # if the directory doesn't exist, create it
        if not os.path.exists(pkgPath):
            os.makedirs(pkgPath)

        for topic in module.topics:
            # reposition ourselves for each topic
            os.chdir(cwd)

            if topic.type == "publish" or topic.type == "subscribe":
                messages = True
                self.processMessage(module, topic)
                if self.base:
                    self.baseFiles[self.msgPkgName].append(topic.fileName)
            elif topic.type == "service":
                services = True
                # check if file has already been generated
                if topic.fileName not in srvFiles:

                    # package folder naming
                    srvFolderPath = os.path.join(os.getcwd(),("hrim_"+module.type+"_"+module.name+"_srvs"), "srv")

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
                        if prop.unit is not None and prop.unit == "enum" and len(prop.enumeration) > 0:

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

        if self.msgPkgName in self.pkgDeps:
            self.pkgDeps.remove(self.msgPkgName)

        buildDeps = ""
        execDeps = ""
        pkgFind = ""
        pkgDep = ""
        for pkgName in self.pkgDeps:
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
                msgList+="\t\"msg/"+tmp+"\"\n"

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

            srvMakeFile = makeFile.replace("%PKGNAME%", "hrim_"+module.type+"_"+module.name+"_srvs")

            srvPkg = pkgFile.replace("%PKGNAME%", "hrim_"+module.type+"_"+module.name+"_srvs")
            srvPkg = srvPkg.replace("%PKGDESC%", module.desc)

            if dependency:
                srvMakeFile = srvMakeFile.replace("%PKGFIND%", "\nfind_package({} REQUIRED)".format(self.msgPkgName))
                srvMakeFile = srvMakeFile.replace("%PKGDEP%", "\n  DEPENDENCIES\n\t\t"+self.msgPkgName)
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
            for tmp in sorted(srvFiles):
                srvList+="\t\"srv/"+tmp+"\"\n"

            srvMakeFile = srvMakeFile.replace("%PKGFILES%", srvList[:-1])

            # generate the CMakeLists.txt file
            cmake = open("CMakeLists.txt", "w")
            cmake.write(srvMakeFile)
            cmake.close()

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
            self.manParams =  self.manParams+module.name+":\n"+manParams

        if len(optParams)>0:
            self.optParams =  self.optParams+module.name+":\n"+optParams

        os.chdir(cwd)

    def generateParameters(self):
        if len(self.manParams)>0:
            params = open("mandatory_parameters.yaml", "w")
            params.write(self.manParams)
            params.close()
            self.manParams = ""
        if len(self.optParams)>0:
            params = open("optional_parameters.yaml", "w")
            params.write(self.optParams)
            params.close()
            self.optParams = ""

    def composeModule(self, modules, paths):
        model = open("model.xml", "w")
        composition = "<?xml version=\"1.0\"?>\n<composition name=\"defaultName\">\n"
        strContent=""
        index = 0
        for module in modules:
            ownTopics = []
            ownParams = []
            strTopics = getTabs(1)+"<model type=\"{}\" subtype=\"{}\" path=\"{}\">\n".format(module.type, module.name, paths[index])
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
        self.pkgDeps = []
        self.manParams = ""
        self.optParams = ""
        self.base = False
