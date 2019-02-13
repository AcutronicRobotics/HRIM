import sys
import os
#os.chdir(os.path.abspath(os.path.dirname(__file__)))
# #sys.path.insert(0, 'scripts')
from hrim.scripts import ModulePrinter
from hrim.scripts import ModuleParser
from hrim.scripts import ModuleCompiler
from hrim.scripts import *
import argparse
import re
import shutil

# locate all module models of the repository and return a list with their full path
def findModels(dirName):
    # list given directory's files and subdirectories
    dirContents = os.listdir(dirName)
    modelFiles = list()
    # iterate over directory contents
    for entry in dirContents:
        # create full path
        fullPath = os.path.join(dirName, entry)
        # if item is a directory, iterate through it's contents
        if os.path.isdir(fullPath):
            modelFiles = modelFiles + findModels(fullPath)
        else:
            # else check the file is a model: look for xml files inside model
            # categories and ignore files inside "topics" directories
            if (
                bool(re.search('models.(actuator|sensor|communication|cognition|ui|power|composite).*\.xml$', fullPath)) and
                not bool(re.search('.*models.*topics', fullPath))
            ):
                modelFiles.append(fullPath)
    return modelFiles

def genBase(parser, compiler, platform):
    path = os.getcwd()

    generic = Module("generic", "generic", "defines the generic HRIM messages used by modules")
    generic.topics = parser.parseBase(os.path.join(path, "models", "generic", "base.xml"))

    compiler.compileModule(generic, True)
    print("Succesfully generated "+platform+" implementation of HRIM's generic package.")
    os.chdir(path)

    geometry = Module("geometry", "geometry", "defines the geometry HRIM messages used by modules")
    geometry.topics = parser.parseBase(os.path.join(path, "models", "geometry", "geometry.xml"))

    compiler.compileModule(geometry, True)
    print("Succesfully generated "+platform+" implementation of HRIM's geometry package.")
    os.chdir(path)

def main(args=None):
    # Add usage messages

    parser=argparse.ArgumentParser(
        description='''Hardware Robot Information Model (HRIM) implementation generation tool.''',
        formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('action', choices=['show','generate', 'compose', 'compile', 'list', 'clear'], help='''Action to take:
show:
    print a representation of the passed valid XML module's model structure and values.
generate:
    generate the platform-specific implementation of the passed valid XML model.
compose:
    generate a composition of modules.
compile:
    generate the platform-specific implementation of the passed composition.
list:
    list available models or generated implementations.
clear:
    delete the passed generated implementation.
        ''')
    parser.add_argument('filePath', nargs="+" , help='''The path to a valid xml model file.
Alternatively, either a shorthand for the generate command:
    all:
        generates the implementation of every existent model
What the list command will look for:
    models:
        will list all available models.
    implementations:
        will list all generated module implementations.
Or the implementation to be deleted by the clear command:
    all:
        all implementations on the generation folder.
    {module name}:
        deletes the files related to said module (i.e. "force" would delete generated/force).
                     ''')
    parser.add_argument('-p', '--platform', default='ros2', choices=['ros2'], help='The platform for the generated model, ros2 by default.')
    parser.add_argument('-e', '--extend', action='store_true', default=False, help='Whether to expand topic definitions when "show"-ing.')
    # If no argument is provided, show usage
    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(0)

    args=parser.parse_args()


    try:
        parser = ModuleParser()
        uniquePath = args.filePath[0]
        if args.action == "show":
            extend = args.extend
            module = parser.parseFile(uniquePath)
            ModulePrinter().printModule(module, args.platform, extend)
        elif args.action == "generate":
            path = os.getcwd()

            compiler = ModuleCompiler()
            compiler.dataTypes = parser.getDataTypes(args.platform)

            compiler.genPath="generated"
            genBase(parser, compiler, args.platform)

            if uniquePath == "moveit":

                module = parser.parseFile(os.path.join(path, "models", "sensor", "3dcamera", "3dcamera_tof.xml"))
                compiler.compileModule(module)
                compiler.generateParameters()
                print("Succesfully generated "+args.platform+" implementation of "+module.name+" module.")
                os.chdir(path)

                module = parser.parseFile(os.path.join(path, "models", "composite", "arm", "arm.xml"))
                compiler.compileModule(module)
                compiler.generateParameters()
                print("Succesfully generated "+args.platform+" implementation of "+module.name+" module.")
                os.chdir(path)

                shape = Module("shape", "moveit", "defines the shape HRIM messages used by modules")
                shape.topics = parser.parseBase(os.path.join(path, "models", "shape", "shape.xml"))
                compiler.compileModule(shape)
                print("Succesfully generated "+args.platform+" implementation of HRIM's shape package.")
                os.chdir(path)

                object_recognition = Module("object_recognition", "moveit", "defines the object_recognition HRIM messages used by modules")
                object_recognition.topics = parser.parseBase(os.path.join(path, "models", "object_recognition", "object_recognition.xml"))
                compiler.compileModule(object_recognition)
                print("Succesfully generated "+args.platform+" implementation of HRIM's object_recognition package.")
                os.chdir(path)

                octomap = Module("octomap", "moveit", "defines the octomap HRIM messages used by modules")
                octomap.topics = parser.parseBase(os.path.join(path, "models", "octomap", "octomap.xml"))
                compiler.compileModule(octomap)
                print("Succesfully generated "+args.platform+" implementation of HRIM's octomap package.")
                os.chdir(path)

                moveit = Module("moveit", "moveit", "defines the moveit HRIM messages used by modules")
                moveit.topics = parser.parseBase(os.path.join(path, "models", "moveit", "moveit.xml"))
                compiler.compileModule(moveit)
                print("Succesfully generated "+args.platform+" implementation of HRIM's moveit package.")
                os.chdir(path)

            elif uniquePath == "control":

                module = parser.parseFile(os.path.join(path, "models", "composite", "arm", "arm.xml"))
                compiler.compileModule(module)
                compiler.generateParameters()
                print("Succesfully generated "+args.platform+" implementation of "+module.name+" module.")
                os.chdir(path)

                control = Module("control", "control", "defines the control HRIM messages used by modules")
                control.topics = parser.parseBase(os.path.join(path, "models", "control", "control.xml"))
                compiler.compileModule(control, True)
                print("Succesfully generated "+args.platform+" implementation of HRIM's control package.")
                os.chdir(path)

            else:

                # check for file generation shorthands
                if uniquePath == "all":
                    fileList = findModels(os.path.join(path, "models"))
                    for item in fileList:
                        module = parser.parseFile(item)
                        compiler.compileModule(module)
                        compiler.generateParameters()
                        print("Succesfully generated "+args.platform+" implementation of "+module.name+" module.")
                        os.chdir(path)

                    module = Module("shape", "moveit", "defines the shape HRIM messages used by modules")
                    module.topics = parser.parseBase(os.path.join(path, "models", "shape", "shape.xml"))
                    compiler.compileModule(module)
                    print("Succesfully generated "+args.platform+" implementation of HRIM's shape package.")
                    os.chdir(path)

                    module = Module("object_recognition", "moveit", "defines the object_recognition HRIM messages used by modules")
                    module.topics = parser.parseBase(os.path.join(path, "models", "object_recognition", "object_recognition.xml"))
                    compiler.compileModule(module)
                    print("Succesfully generated "+args.platform+" implementation of HRIM's object_recognition package.")
                    os.chdir(path)

                    module = Module("octomap", "moveit", "defines the octomap HRIM messages used by modules")
                    module.topics = parser.parseBase(os.path.join(path, "models", "octomap", "octomap.xml"))
                    compiler.compileModule(module)
                    print("Succesfully generated "+args.platform+" implementation of HRIM's octomap package.")
                    os.chdir(path)

                    module = Module("moveit", "moveit", "defines the moveit HRIM messages used by modules")
                    module.topics = parser.parseBase(os.path.join(path, "models", "moveit", "moveit.xml"))
                    compiler.compileModule(module)
                    print("Succesfully generated "+args.platform+" implementation of HRIM's moveit package.")
                    os.chdir(path)

                    module = Module("control", "control", "defines the control HRIM messages used by modules")
                    module.topics = parser.parseBase(os.path.join(path, "models", "control", "control.xml"))
                    compiler.compileModule(module, True)
                    print("Succesfully generated "+args.platform+" implementation of HRIM's control package.")

                elif uniquePath == "actuators":
                    fileList = findModels(os.path.join(path, "models", "actuator"))
                    for item in fileList:
                        module = parser.parseFile(item)
                        compiler.compileModule(module)
                        compiler.generateParameters()
                        print("Succesfully generated "+args.platform+" implementation of "+module.name+" module.")
                        os.chdir(path)

                elif uniquePath == "composites":
                    fileList = findModels(os.path.join(path, "models", "composite"))
                    for item in fileList:
                        module = parser.parseFile(item)
                        compiler.compileModule(module)
                        compiler.generateParameters()
                        print("Succesfully generated "+args.platform+" implementation of "+module.name+" module.")
                        os.chdir(path)

                elif uniquePath == "powers":
                    fileList = findModels(os.path.join(path, "models", "power"))
                    for item in fileList:
                        module = parser.parseFile(item)
                        compiler.compileModule(module)
                        compiler.generateParameters()
                        print("Succesfully generated "+args.platform+" implementation of "+module.name+" module.")
                        os.chdir(path)

                elif uniquePath == "sensors":
                    fileList = findModels(os.path.join(path, "models", "sensor"))
                    for item in fileList:
                        module = parser.parseFile(item)
                        compiler.compileModule(module)
                        compiler.generateParameters()
                        print("Succesfully generated "+args.platform+" implementation of "+module.name+" module.")
                        os.chdir(path)

                # else try to generate the implementation based on the passed file
                else:
                    module = parser.parseFile(uniquePath)
                    compiler.compileModule(module)
                    compiler.generateParameters()
                    print("Succesfully generated "+args.platform+" implementation of "+module.name+" module.")

        elif args.action == "compose":
            modules=[]
            paths=[]
            for eachPath in args.filePath:
                fullPath = os.path.join(os.getcwd(), "models", eachPath)
                if os.path.exists(fullPath):
                    modelList = findModels(fullPath)
                    properList = []
                    for model in modelList:
                        properList.append(model)
                    if len(properList)==1:
                        module = parser.parseFile(properList[0])
                        modules.append(module)
                        path = properList[0].replace(os.getcwd()+"/","")
                        paths.append(path)
                        print("Adding {} defined at {} to the composition.".format(module.name, path))
                    else:
                        print("More than one model found by handle '{}':".format(eachPath))
                        for model in properList:
                            print("\t"+model)
                        exit()
                else:
                    fullPath = os.path.join(os.getcwd(), "models", eachPath+".xml")
                    if os.path.exists(fullPath):
                        module = parser.parseFile(fullPath)
                        modules.append(module)
                        path = fullPath.replace(os.getcwd()+"/","")
                        paths.append(path)
                        print("Adding {} defined at {} to the composition.".format(module.name, path))
                    else:
                        print("No model found by handle '{}'.".format(eachPath))
                        print("Module composition cancelled.")
                        exit()
            compiler = ModuleCompiler()
            compiler.composeModule(modules, paths)
            print("Composition generated: model.xml")
        elif args.action == "compile":
            path = os.getcwd()
            composition = parser.parseComposition(os.path.join(os.getcwd(),uniquePath))
            if os.path.exists(os.path.join(path, "composition", composition.name)):
                shutil.rmtree(os.path.join(path, "composition", composition.name))
            compiler = ModuleCompiler()
            compiler.genPath = "composition/"+composition.name
            compiler.dataTypes = ModuleParser().getDataTypes(args.platform)
            compiler.composition = True
            genBase(parser, compiler, args.platform)
            # for module in modules:
            for module in composition.modules:
                os.chdir(path)
                compiler.compileModule(module)

            os.chdir(os.path.join(path, compiler.genPath))
            compiler.generateParameters()

            shutil.copyfile(os.path.join(path, uniquePath), uniquePath)

        elif args.action == "list":
            if uniquePath == "models":
                modelList = findModels(os.path.join(os.getcwd(), "models"))
                for model in sorted(modelList):
                    pathList = model.split(os.sep)
                    print(pathList[-3]+"/"+pathList[-2]+"/"+pathList[-1].replace('.xml',""))
            elif uniquePath == "implementations":
                impList = os.listdir("generated")
                if len(impList) > 0:
                    for implementation in sorted(impList):
                        print(implementation)
                else:
                    print("There's no generated implementations.")
        elif args.action == "clear":
            if len(os.listdir("generated")) > 0:
                if uniquePath == "all":
                    delDirs = os.listdir("generated")
                    for delPath in sorted(delDirs):
                        fullPath = os.path.join(os.getcwd(), "generated", delPath)
                        shutil.rmtree(fullPath)
                        print("Deleted "+fullPath)
                else:
                    fullPath = os.path.join(os.getcwd(), "generated", uniquePath)
                    if os.path.exists(fullPath):
                        shutil.rmtree(fullPath)
                        print("Deleted "+fullPath)
                    else:
                        print("Couldn't find passed directory for deletion.")
            else:
                print("There is no implementation to delete (generated directory is empty).")
        else:
            print("Unknown command")
    except UnicodeDecodeError:
         print("Unicode exception, check your locales\nUseful command: export LC_ALL=C.UTF-8")
         sys.exit(1)
    except:
         print("An error occurred during command execution")
         sys.exit(1)

if __name__ == '__main__':

    main()
