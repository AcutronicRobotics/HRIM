import os
from .classes import Module

# shorthand for tabulation
def getTabs(num):
    ret = ""
    for _ in range(num):
        ret += "\t"
    return ret

def checkModule(parser, module, moduleList):
    depList = []
    hasDeps = False
    ownPkg = "hrim_"+module.type+"_"+module.name

    tmpList = []
    for entry in moduleList:
        tmpList.append(entry.name)

    if module.name in tmpList:
        moduleList.insert(0, moduleList.pop(tmpList.index(module.name)))
        return moduleList
    moduleList.insert(0, module)
    checkDependencies(module, ownPkg, depList)
    for entry in depList:
        path = os.path.join("models", entry.split("_")[1], entry.split("_")[2], entry.split("_")[2]+".xml")
        subList = checkModuleFile(parser, path, moduleList)
        try:
            for module in subList:
                if module not in moduleList:
                    moduleList.insert(0, module)
        except Exception as e:
            if subList not in moduleList:
                moduleList.insert(0, subList)
            # print(e)

            # print(path)
        # subModules = checkModuleFile(parser)
        # print(ownPkg+" has deps from "+entry)
    return moduleList

def checkModuleFile(parser, path, moduleList):
    depList = []
    hasDeps = False
    module = parser.parseFile(path)
    ownPkg = "hrim_"+module.type+"_"+module.name

    tmpList = []
    for entry in moduleList:
        tmpList.append(entry.name)

    if module.name in tmpList:
        moduleList.insert(0, moduleList.pop(tmpList.index(module.name)))
        return moduleList
    moduleList.insert(0, module)
    checkDependencies(module, ownPkg, depList)
    for entry in depList:
        path = os.path.join("models", entry.split("_")[1], entry.split("_")[2], entry.split("_")[2]+".xml")
        subList = checkModuleFile(parser, path, moduleList)
        try:
            for module in subList:
                if module not in moduleList:
                    moduleList.insert(0, module)
        except Exception as e:
            if subList not in moduleList:
                moduleList.insert(0, subList)
            # print(e)

            # print(path)
        # subModules = checkModuleFile(parser)
        # print(ownPkg+" has deps from "+entry)
    return moduleList

def checkDependencies(entry, pkgName, depList):
    if isinstance(entry, Module):
        hasDeps = False
        for topic in entry.topics:
            if(checkDependencies(topic, pkgName, depList)):
                hasDeps = True
        return hasDeps
    else:
        if entry.package is not None and entry.package.startswith("hrim_"):
            if not entry.package.startswith("hrim_generic") and not entry.package.startswith(pkgName):
                depList.append(entry.package)
                return True
        hasDeps = False
        for entry in entry.properties:
            if(checkDependencies(entry, pkgName, depList)):
                hasDeps = True
        if hasattr(entry, "response"):
            for entry in entry.response:
                if(checkDependencies(entry, pkgName, depList)):
                    hasDeps = True
        if hasattr(entry, "feedback"):
            for entry in entry.feedback:
                if(checkDependencies(entry, pkgName, depList)):
                    hasDeps = True
        return hasDeps
