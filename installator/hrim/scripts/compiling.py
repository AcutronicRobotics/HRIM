import os
from .utils import get_tabs


class ModuleCompiler:
    def process_sub_property(self, prop, sub_type):
        try:
            if self.check_generated(prop.fileName, sub_type) is False:
                sub_msg = ""
                for subProp in prop.properties:
                    if (subProp.fileName is not None and
                            (subProp.package is None or
                             subProp.package == self.msgPkgName)):
                        try:
                            self.process_sub_property(subProp, sub_type)
                        except Exception as e:
                            raise e
                    else:
                        if (subProp.unit is not None and
                                subProp.unit == "enum" and
                                len(subProp.enumeration) > 0):
                            # sort enumeration values for readability
                            for value in sorted(((v, k) for k, v in
                                                 subProp.enumeration.items())):
                                sub_msg += subProp.type + " " + value[1] + \
                                           "=" + str(value[0]) + "\n"
                    sub_msg += self.format_property(subProp, sub_type)
                sub_file_name = prop.fileName + ".msg"

                if len(sub_msg) > 0:
                    if self.base:
                        self.baseFiles[self.msgPkgName].append(prop.fileName)

                    self.ownFiles.append(prop.fileName)

                    text_file = open(sub_file_name, "w")
                    text_file.write(sub_msg)
                    text_file.close()
        except Exception as e:
            print("Error while processing subproperty message: " +
                  prop.fileName)
            raise e

    # separate the message generation for messages declared inside services
    def process_message(self, topic):
        try:
            # check if file has already been generated
            res = self.check_generated(topic.fileName, topic.type)
            if res is False:

                # package folder naming
                self.msgFolderPath = os.path.join(os.getcwd(),
                                                  self.msgPkgName, "msg")

                # if the package directories don't exist, create them
                if not os.path.exists(self.msgFolderPath):
                    os.makedirs(self.msgFolderPath)

                # position ourselves on the package's msg folder
                os.chdir(self.msgFolderPath)
                msg = ""

                # check for an overall message description
                if topic.desc is not None and len(topic.desc) > 0:
                    msg += "# " + topic.desc + "\n\n"

                for prop in topic.properties:

                    if prop.fileName is not None and (
                            prop.package is None or
                            prop.package == self.msgPkgName
                    ):
                        self.process_sub_property(prop, topic.type)
                    else:

                        # check for enumeration types
                        if (prop.unit is not None and prop.unit == "enum" and
                                len(prop.enumeration) > 0):

                            # sort enumeration values for readability
                            for value in sorted(((v, k) for k, v
                                                 in prop.enumeration.items())):
                                msg += prop.type + " " + value[1] + "=" \
                                       + str(value[0]) + "\n"

                    # process each property, checking if it's value is an array
                    # and if it has any description
                    msg += self.format_property(prop, topic.type)

                    if prop.package is not None:
                        if prop.package not in self.msgDeps:
                            self.msgDeps.append(prop.package)

                # generate each .msg file and add it to the list
                if topic.fileName is None:
                    file_name = topic.name.title() + ".msg"
                    self.ownFiles.append(topic.name.title())
                else:
                    file_name = topic.fileName + ".msg"
                    self.ownFiles.append(topic.fileName)

                if len(msg) > 0:
                    text_file = open(file_name, "w")
                    text_file.write(msg)
                    text_file.close()
        except Exception as e:
            print("Error while processing topic message: " + topic.fileName)
            raise e

    def format_property(self, prop, format_type):
        if prop.fileName is None:
            format_type = self.dataTypes[prop.type]
        else:
            if prop.package is not None:
                format_type = prop.package + "/" + prop.fileName
            else:
                found = self.check_generated(prop.fileName, format_type)
                if found:
                    format_type = str(found) + "/" + prop.fileName
                else:
                    format_type = self.msgPkgName + "/" + prop.fileName
        length = prop.length if prop.length is not None else ""
        ret = format_type
        ret += ("[{}]".format(length) if prop.array else "")
        ret += " " + prop.name + ((" # " + prop.desc) if prop.desc is not None
                                  else "") + "\n\n"
        return ret

    def check_generated(self, message, check_type):
        res = False
        for key, items_list in self.baseFiles.items():
            if message in items_list:
                res = key
        if res is False:
            if message in self.ownFiles:
                res = self.msgPkgName

        if (res and res != self.msgPkgName and
                check_type in ["publish", "subscribe"] and
                res not in self.msgDeps):
            self.msgDeps.append(res)

        return res

    def compile_module(self, module, base=False):
        global actionPkgName, actionFolderPath, srvPkgName, srvFolderPath, \
            shortType, myDep, folderPath, pkgName, myFiles
        self.base = base
        messages = False
        services = False
        actions = False
        self.msgDeps = []
        self.srvDeps = []
        self.actionDeps = []
        self.ownFiles = []
        srv_files = []
        action_files = []

        # Actions will always depend on action_msgs
        self.actionDeps.append("action_msgs")

        cwd = os.getcwd()

        if self.base:
            self.msgPkgName = "hrim_" + module.name + "_msgs"
            self.baseFiles[self.msgPkgName] = []
        else:
            if (module.type in
                    ["actuator", "sensor", "communication", "cognition", "ui",
                     "power", "composite"]):
                self.msgPkgName = "hrim_" + module.type + "_" + module.name + \
                                  "_msgs"
            else:
                self.msgPkgName = "hrim_" + module.name + "_msgs"

        os.chdir("templates")
        with open('package.txt', 'r') as myfile:
            pkg_file = myfile.read()
        with open('cmake.txt', 'r') as myfile:
            make_file = myfile.read()

        if self.composition or self.base:
            cwd = os.path.join(cwd, self.genPath, module.type)
        else:
            cwd = os.path.join(cwd, self.genPath, module.type, module.name)

        pkg_path = os.path.join(cwd, self.msgPkgName)

        # if the directory doesn't exist, create it
        if not os.path.exists(pkg_path):
            os.makedirs(pkg_path)

        for topic in module.topics:
            # reposition ourselves for each topic
            os.chdir(cwd)

            if topic.type == "publish" or topic.type == "subscribe":
                messages = True
                self.process_message(topic)
                if self.base:
                    self.baseFiles[self.msgPkgName].append(topic.fileName)
            else:
                if topic.type == "service":
                    myDep = self.srvDeps
                    myFiles = srv_files
                    shortType = "srv"
                    if (self.base or module.type not in
                            ["actuator", "sensor", "communication",
                             "cognition", "ui", "power", "composite"
                             ]):
                        srvPkgName = "hrim_" + module.name + "_" + shortType \
                                     + "s"
                    else:
                        srvPkgName = "hrim_" + module.type + "_" \
                                     + module.name + "_" + shortType + "s"
                    pkgName = srvPkgName
                    srvFolderPath = os.path.join(os.getcwd(), srvPkgName,
                                                 shortType)
                    folderPath = srvFolderPath
                if topic.type == "action":
                    myDep = self.actionDeps
                    myFiles = action_files
                    shortType = "action"
                    if (self.base or module.type not in
                            ["actuator", "sensor", "communication",
                             "cognition", "ui", "power", "composite"
                             ]):
                        actionPkgName = "hrim_" + module.name + "_" \
                                        + shortType + "s"
                    else:
                        actionPkgName = "hrim_" + module.type + "_" \
                                        + module.name + "_" + shortType + "s"
                    pkgName = actionPkgName
                    actionFolderPath = os.path.join(os.getcwd(), actionPkgName,
                                                    shortType)
                    folderPath = actionFolderPath
                # check if file has already been generated
                if (topic.fileName not in myFiles and
                        (topic.package is None or topic.package == pkgName)):

                    # if the package directories don't exist, create them
                    if not os.path.exists(folderPath):
                        os.makedirs(folderPath)

                    # position ourselves on the package's action folder
                    os.chdir(folderPath)
                    file_content = ""

                    # check for an overall file description
                    if topic.desc is not None and len(topic.desc) > 0:
                        file_content += "# " + topic.desc + "\n\n"

                    for prop in topic.properties:

                        if (prop.fileName is not None and
                                (prop.package is None or
                                 prop.package == self.msgPkgName)):
                            os.chdir(cwd)
                            self.process_message(prop)
                            os.chdir(folderPath)
                        # check for enumeration types
                        if (prop.unit is not None and prop.unit == "enum" and
                                len(prop.enumeration) > 0):

                            # sort enumeration values for readability
                            for value in sorted(((v, k) for k, v
                                                 in prop.enumeration.items())):
                                file_content += prop.type + " " + value[1] \
                                               + "=" + str(value[0]) + "\n"

                        # process each property, checking if it's value is
                        # an array and if it has any description
                        file_content += self.format_property(prop, topic.type)

                        if prop.package is not None:
                            if prop.package not in myDep:
                                myDep.append(prop.package)

                    file_content += "---\n"

                    for prop in topic.response:
                        if (prop.fileName is not None and
                                (prop.package is None or prop.package ==
                                 self.msgPkgName)):
                            os.chdir(cwd)
                            self.process_message(prop)
                            os.chdir(folderPath)

                        # check for enumeration types
                        if (prop.unit is not None and prop.unit == "enum" and
                                len(prop.enumeration) > 0):

                            # sort enumeration values for readability
                            for value in sorted(((v, k) for k, v
                                                 in prop.enumeration.items())):
                                file_content += prop.type + " " + value[1] \
                                               + "=" + str(value[0]) + "\n"

                        # process each property, checking if it's
                        # value is an array and if it has any description
                        file_content += self.format_property(prop, topic.type)

                        if prop.package is not None:
                            if prop.package not in myDep:
                                myDep.append(prop.package)

                    if topic.type == "action":
                        file_content += "---\n"

                        for prop in topic.feedback:
                            if (prop.fileName is not None and
                                    (prop.package is None or prop.package ==
                                     self.msgPkgName)):
                                os.chdir(cwd)
                                self.process_message(prop)
                                os.chdir(folderPath)

                            # check for enumeration types
                            if (prop.unit is not None and
                                    prop.unit == "enum" and
                                    len(prop.enumeration) > 0):

                                # sort enumeration values for readability
                                for value in sorted(((v, k) for k, v
                                                     in prop.enumeration.items(
                                ))):
                                    file_content += prop.type + " " + \
                                                    value[1] + "=" + \
                                                    str(value[0]) + "\n"

                            # process each property, checking if it's value
                            # is an array and if it has any description
                            file_content += self.format_property(prop,
                                                                 topic.type)

                            if prop.package is not None:
                                if prop.package not in myDep:
                                    myDep.append(prop.package)

                    # generate each .action file and add it to the list
                    if topic.fileName is None:
                        file_name = topic.name.title() + "." + shortType
                        myFiles.append(topic.name.title())
                    else:
                        file_name = topic.fileName + "." + shortType
                        myFiles.append(topic.fileName)

                    text_file = open(file_name, "w")
                    text_file.write(file_content)
                    text_file.close()

                    if topic.type == "service":
                        services = True
                    if topic.type == "action":
                        actions = True

        if self.msgPkgName in self.msgDeps:
            self.msgDeps.remove(self.msgPkgName)
        if "hrim_generic_msgs" in self.msgDeps:
            self.msgDeps.remove("hrim_generic_msgs")

        build_deps = ""
        exec_deps = ""
        pkg_find = ""
        pkg_dep = ""
        for pkgName in self.msgDeps:
            build_deps = build_deps + "\n\t<build_depend>{}</build_depend>" \
                .format(pkgName)
            exec_deps = exec_deps + "\n\t<exec_depend>{}</exec_depend>" \
                .format(pkgName)
            pkg_find = pkg_find + "\nfind_package({} REQUIRED)" \
                .format(pkgName)
            pkg_dep = pkg_dep + "\n\t\t{}" \
                .format(pkgName)

        # insert the package's name and description in package.xml's content
        msg_pkg = pkg_file.replace("%PKGNAME%", self.msgPkgName)
        msg_pkg = msg_pkg.replace("%PKGDESC%", module.desc)
        msg_pkg = msg_pkg.replace("%PKGBUILD%", build_deps)
        msg_pkg = msg_pkg.replace("%PKGEXEC%", exec_deps)

        # insert the package's name and description in CMakeLists.txt's content
        msg_make_file = make_file.replace("%PKGNAME%", self.msgPkgName)
        msg_make_file = msg_make_file.replace("%PKGFIND%", pkg_find)

        if len(pkg_dep) > 0:
            pkg_dep = "\n\tDEPENDENCIES" + pkg_dep

        msg_make_file = msg_make_file.replace("%PKGDEP%", pkg_dep)

        # if the package has messages
        if messages:
            self.ownFiles = os.listdir(self.msgFolderPath)
            # reposition ourselves on the package's root
            os.chdir(self.msgFolderPath[:-4])

            # generate the package.xml file
            package = open("package.xml", "w")
            package.write(msg_pkg)
            package.close()

            # insert the .msg list in the CMakeLists.txt
            msg_list = ""
            for tmp in sorted(self.ownFiles):
                msg_list += "\t\"msg/" + tmp + "\"\n"

            msg_make_file = msg_make_file.replace("%PKGFILES%", msg_list[:-1])

            # generate the CMakeLists.txt file
            cmake = open("CMakeLists.txt", "w")
            cmake.write(msg_make_file)
            cmake.close()

        # if the package has services
        if services:
            srv_files = os.listdir(srvFolderPath)
            # reposition ourselves on the package's root
            os.chdir(srvFolderPath[:-4])

            srv_make_file = make_file.replace("%PKGNAME%", srvPkgName)
            srv_pkg = pkg_file.replace("%PKGNAME%", srvPkgName)

            srv_pkg = srv_pkg.replace("%PKGDESC%", module.desc)

            if len(self.srvDeps) > 0:
                build_deps = ""
                exec_deps = ""
                pkg_find = ""
                pkg_dep = ""
                for dependency in self.srvDeps:
                    build_deps = build_deps + \
                                 "\n\t<build_depend>{}</build_depend>"\
                                 .format(dependency)
                    exec_deps = exec_deps + "\n\t<exec_depend>{}" \
                                            "</exec_depend>".format(dependency)
                    pkg_find = pkg_find + "\nfind_package({} REQUIRED)" \
                        .format(dependency)
                    pkg_dep = pkg_dep + "\n\t\t{}".format(dependency)
                srv_make_file = srv_make_file.replace("%PKGFIND%", pkg_find)
                srv_make_file = srv_make_file.replace("%PKGDEP%",
                                                      "\n  DEPENDENCIES" +
                                                      pkg_dep)
                srv_pkg = srv_pkg.replace("%PKGBUILD%", build_deps)
                srv_pkg = srv_pkg.replace("%PKGEXEC%", exec_deps)
            else:
                srv_make_file = srv_make_file.replace("%PKGFIND%", "")
                srv_make_file = srv_make_file.replace("%PKGDEP%", "")
                srv_pkg = srv_pkg.replace("%PKGBUILD%", "")
                srv_pkg = srv_pkg.replace("%PKGEXEC%", "")

            # generate the package.xml file
            package = open("package.xml", "w")
            package.write(srv_pkg)
            package.close()

            # insert the .srv list in the CMakeLists.txt
            srv_list = ""
            for tmp in sorted(srv_files):
                srv_list += "\t\"srv/" + tmp + "\"\n"

            srv_make_file = srv_make_file.replace("%PKGFILES%", srv_list[:-1])

            # generate the CMakeLists.txt file
            cmake = open("CMakeLists.txt", "w")
            cmake.write(srv_make_file)
            cmake.close()

        # if the package has actions
        if actions:
            action_files = os.listdir(actionFolderPath)
            # reposition ourselves on the package's root
            os.chdir(actionFolderPath[:-7])

            action_make_file = make_file.replace("%PKGNAME%", actionPkgName)
            action_pkg = pkg_file.replace("%PKGNAME%", actionPkgName)

            action_pkg = action_pkg.replace("%PKGDESC%", module.desc)

            if len(self.actionDeps) > 0:
                build_deps = ""
                exec_deps = ""
                pkg_find = ""
                pkg_dep = ""
                for dependency in self.actionDeps:
                    build_deps = build_deps \
                        + "\n\t<build_depend>{}</build_depend>" \
                        .format(dependency)
                    exec_deps = exec_deps \
                        + "\n\t<exec_depend>{}</exec_depend>" \
                        .format(dependency)
                    pkg_find = pkg_find \
                        + "\nfind_package({} REQUIRED)" \
                        .format(dependency)
                    pkg_dep = pkg_dep + "\n\t\t{}".format(dependency)
                action_make_file = action_make_file.replace("%PKGFIND%",
                                                            pkg_find)
                action_make_file = action_make_file.replace("%PKGDEP%",
                                                            "\n  DEPENDENCIES"
                                                            + pkg_dep)
                action_pkg = action_pkg.replace("%PKGBUILD%", build_deps)
                action_pkg = action_pkg.replace("%PKGEXEC%", exec_deps)
            else:
                action_make_file = action_make_file.replace("%PKGFIND%", "")
                action_make_file = action_make_file.replace("%PKGDEP%", "")
                action_pkg = action_pkg.replace("%PKGBUILD%", "")
                action_pkg = action_pkg.replace("%PKGEXEC%", "")

            # generate the package.xml file
            package = open("package.xml", "w")
            package.write(action_pkg)
            package.close()

            # insert the .action list in the CMakeLists.txt
            action_list = ""
            for tmp in sorted(action_files):
                action_list += "\t\"action/" + tmp + "\"\n"

            action_make_file = action_make_file.replace("%PKGFILES%",
                                                        action_list[:-1])

            # generate the CMakeLists.txt file
            cmake = open("CMakeLists.txt", "w")
            cmake.write(action_make_file)
            cmake.close()

        man_params = ""
        opt_params = ""

        for param in module.params:

            # mandatory parameters parsing
            if param.mandatory:
                if param.desc is not None:
                    man_params += get_tabs(1) + "# " + param.desc + "\n"
                man_params += get_tabs(1) + param.name + ": "
                man_params += (str(param.value) if param.value is not None else
                               "") + "\n\n"

            # optional parameters parsing
            else:
                if param.desc is not None:
                    opt_params += get_tabs(1) + "# " + param.desc + "\n"
                opt_params += get_tabs(1) + param.name + ": "
                opt_params += (str(param.value) if param.value is not None else
                               "") + "\n\n"

        if len(man_params) > 0:
            self.manParams = self.manParams + module.name + ":\n" + man_params

        if len(opt_params) > 0:
            self.optParams = self.optParams + module.name + ":\n" + opt_params

        os.chdir(cwd)

    def generate_parameters(self):
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

    @staticmethod
    def compose_module(modules, paths):
        model = open("model.xml", "w")
        composition = \
            "<?xml version=\"1.0\"?>\n<composition name=\"defaultName\">\n"
        str_content = ""
        index = 0
        for module in modules:
            own_topics = []
            own_params = []
            str_topics = get_tabs(1) + "<model type=\"{}\" subtype=\"{}\" " \
                                       "path=\"{}\">\n".format(
                module.type, module.name, paths[index]
            )
            str_params = ""
            for topic in module.topics:
                if not topic.mandatory:
                    own_topics.append(topic)
            for param in module.params:
                if not param.mandatory:
                    own_params.append(param)
            own_topics.sort(key=lambda topic_name: topic_name.name)
            own_params.sort(key=lambda param_name: param_name.name)
            for topic in own_topics:
                str_topics = str_topics + (get_tabs(2) +
                                           "<topic name=\"{}\"/>\n") \
                    .format(topic.name)
            for param in own_params:
                str_params = str_params + (get_tabs(2) +
                                           "<param name=\"{}\"/>\n") \
                    .format(param.name)
            str_topics = str_topics + str_params + get_tabs(1) + "</model>"
            str_content = str_content + str_topics + "\n"
            index = index + 1
        composition = composition + str_content[:-1] + "\n</composition>"
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
