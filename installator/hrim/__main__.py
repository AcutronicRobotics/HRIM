import sys
import os
import argparse
import re
import shutil
import pkg_resources
from hrim.scripts import ModuleCompiler, ModuleParser
from hrim.scripts.classes import Module


# locate all module models of the repository and return a list with their full
# path
def find_models(dir_name):
    # list given directory's files and subdirectories
    dir_contents = os.listdir(dir_name)
    model_files = list()
    # iterate over directory contents
    for entry in dir_contents:
        # create full path
        full_path = os.path.join(dir_name, entry)
        # if item is a directory, iterate through it's contents
        if os.path.isdir(full_path):
            model_files = model_files + find_models(full_path)
        elif (
                re.search(
                    r'models.(actuator|sensor|communication|cognition|ui|'
                    r'power|composite).*\.xml$',
                    full_path
                ) and
                not bool(re.search('.*models.*topics', full_path))
        ):
            model_files.append(full_path)
    return model_files


def gen_base(parser, compiler, args):
    path = os.getcwd()

    generic = Module("generic", "generic", "defines the generic HRIM messages "
                                           "used by modules")
    generic.topics = parser.parse_base(os.path.join(path, "models", "generic",
                                                    "base.xml"))

    compiler.compile_module(generic, True)
    print("Successfully generated " + args.platform + " implementation of "
                                                      "HRIM's generic package."
          )
    os.chdir(path)


def main():
    # Add usage messages

    arg_parser = argparse.ArgumentParser(
        description='''Hardware Robot Information Model (HRIM) implementation
         generation tool.''',
        formatter_class=argparse.RawTextHelpFormatter)
    arg_parser.add_argument(
        'action', choices=['generate', 'compose', 'compile', 'list', 'clear'],
        help='''Action to take:
generate:
    generate the platform-specific implementation of the passed valid XML
    model.
compose:
    generate a composition of modules.
compile:
    generate the platform-specific implementation of the passed composition.
list:
    list available models or generated implementations.
clear:
    delete the passed generated implementation.'''
    )

    arg_parser.add_argument(
        'filePath', nargs="+",
        help='''The path to a valid xml model file.
Alternatively, either a shorthand for the generate command:
    all:
        generates the implementation of every existent model
    actuators:
        generates the implementation of every existent actuator models
    composites:
        generates the implementation of every existent composite models
    powers:
        generates the implementation of every existent power models
    sensors:
        generates the implementation of every existent sensor models
What the list command will look for:
    models:
        will list all available models.
    implementations:
        will list all generated module implementations.
Or the implementation to be deleted by the clear command:
    all:
        all implementations on the generation folder.
    {path}:
        deletes the files related to said module (i.e. `sensor/imu` will delete
        `generated/sensor/imu`, `sensor` will delete all generated sensor
         implementations).'''
    )

    arg_parser.add_argument(
        '-v', '--version', action='store_true', default=False,
        help='print the tooling version'
    )
    arg_parser.add_argument(
        '-p', '--platform', default='ros2', choices=['ros2'],
        help='The platform for the generated model, ros2 by default.'
    )

    # If no argument is provided, show usage
    if len(sys.argv) == 1:
        arg_parser.print_help()
        sys.exit(0)
    elif any(arg in sys.argv for arg in ["-v", "--version"]):
        print(pkg_resources.require("hrim")[0].version)
        sys.exit(0)

    args = arg_parser.parse_args()

    try:
        parser = ModuleParser()
        unique_path = args.filePath[0]
        if args.action == "generate":
            path = os.getcwd()

            compiler = ModuleCompiler()
            compiler.dataTypes = parser.get_data_types(args.platform)

            compiler.genPath = "generated"
            gen_base(parser, compiler, args)

            # check for file generation shorthands
            if unique_path == "all":
                file_list = find_models(os.path.join(path, "models"))
                for item in file_list:
                    module = parser.parse_file(item)
                    compiler.compile_module(module)
                    compiler.generate_parameters()
                    print(
                        "Successfully generated " + args.platform +
                        " implementation of " + module.name + " module."
                    )
                    os.chdir(path)

            elif unique_path == "actuators":
                file_list = find_models(os.path.join(path, "models",
                                                     "actuator"))
                for item in file_list:
                    module = parser.parse_file(item)
                    compiler.compile_module(module)
                    compiler.generate_parameters()
                    print(
                        "Successfully generated " + args.platform +
                        " implementation of " + module.name + " module."
                    )
                    os.chdir(path)

            elif unique_path == "composites":
                file_list = find_models(os.path.join(path, "models",
                                                     "composite"))
                for item in file_list:
                    module = parser.parse_file(item)
                    compiler.compile_module(module)
                    compiler.generate_parameters()
                    print(
                        "Successfully generated " + args.platform +
                        " implementation of " + module.name + " module."
                    )
                    os.chdir(path)

            elif unique_path == "powers":
                file_list = find_models(os.path.join(path, "models", "power"))
                for item in file_list:
                    module = parser.parse_file(item)
                    compiler.compile_module(module)
                    compiler.generate_parameters()
                    print(
                        "Successfully generated " + args.platform +
                        " implementation of " + module.name + " module."
                    )
                    os.chdir(path)

            elif unique_path == "sensors":
                file_list = find_models(os.path.join(path, "models", "sensor"))
                for item in file_list:
                    module = parser.parse_file(item)
                    compiler.compile_module(module)
                    compiler.generate_parameters()
                    print(
                        "Successfully generated " + args.platform +
                        " implementation of " + module.name + " module."
                    )
                    os.chdir(path)

            # else try to generate the implementation based on the passed file
            else:
                module = parser.parse_file(unique_path)
                compiler.compile_module(module)
                compiler.generate_parameters()
                print(
                    "Successfully generated " + args.platform +
                    " implementation of " + module.name + " module."
                )

        elif args.action == "compose":
            modules = []
            paths = []
            for eachPath in args.filePath:
                full_path = os.path.join(os.getcwd(), "models", eachPath)
                if os.path.exists(full_path):
                    model_list = find_models(full_path)
                    proper_list = []
                    for model in model_list:
                        proper_list.append(model)
                    if len(proper_list) == 1:
                        module = parser.parse_file(proper_list[0])
                        modules.append(module)
                        path = proper_list[0].replace(os.getcwd() + "/", "")
                        paths.append(path)
                        print("Adding {} defined at {} to the composition."
                              .format(module.name, path))
                    else:
                        print("More than one model found by handle '{}':"
                              .format(eachPath))
                        for model in proper_list:
                            print("\t" + model)
                        exit()
                else:
                    full_path = os.path.join(os.getcwd(), "models", eachPath +
                                             ".xml")
                    if os.path.exists(full_path):
                        module = parser.parse_file(full_path)
                        modules.append(module)
                        path = full_path.replace(os.getcwd() + "/", "")
                        paths.append(path)
                        print("Adding {} defined at {} to the composition.".
                              format(module.name, path))
                    else:
                        print("No model found by handle '{}'.".
                              format(eachPath))
                        print("Module composition cancelled.")
                        exit()
            compiler = ModuleCompiler()
            compiler.compile_module(modules, paths)
            print("Composition generated: model.xml")
        elif args.action == "compile":
            path = os.getcwd()
            composition = parser.parse_composition(os.path.join(os.getcwd(),
                                                                unique_path))
            if os.path.exists(os.path.join(path, "composition",
                                           composition.name)):
                shutil.rmtree(os.path.join(path, "composition",
                                           composition.name))
            compiler = ModuleCompiler()
            compiler.genPath = "composition/" + composition.name
            compiler.dataTypes = ModuleParser().get_data_types(args.platform)
            compiler.composition = True
            gen_base(parser, compiler, args)
            # for module in modules:
            for module in composition.modules:
                os.chdir(path)
                compiler.compile_module(module)

            os.chdir(os.path.join(path, compiler.genPath))
            compiler.generate_parameters()

            shutil.copyfile(os.path.join(path, unique_path), unique_path)

        elif args.action == "list":
            if unique_path == "models":
                model_list = find_models(os.path.join(os.getcwd(), "models"))
                for model in sorted(model_list):
                    path_list = model.split(os.sep)
                    print(path_list[-3] + "/" + path_list[-2] + "/" +
                          path_list[-1].replace('.xml', ""))
            elif unique_path == "implementations":
                path_list = os.listdir("generated")
                if len(path_list) > 0:
                    for path in sorted(path_list):
                        if (path in
                                ["actuator", "sensor", "communication",
                                 "cognition", "ui", "power", "composite"]):
                            sub_path_list = os.listdir(os.path.join(
                                "generated", path))
                            for subPath in sorted(sub_path_list):
                                print(path + "/" + subPath)
                        else:
                            print(path)
                else:
                    print("There's no generated implementations.")
        elif args.action == "clear":
            if len(os.listdir("generated")) > 0:
                if unique_path == "all":
                    del_dirs = os.listdir("generated")
                    for delPath in sorted(del_dirs):
                        full_path = os.path.join(os.getcwd(), "generated",
                                                 delPath)
                        shutil.rmtree(full_path)
                        print("Deleted " + full_path)
                else:
                    full_path = os.path.join(os.getcwd(), "generated",
                                             unique_path)
                    if os.path.exists(full_path):
                        shutil.rmtree(full_path)
                        print("Deleted " + full_path)
                    else:
                        print("Couldn't find passed directory for deletion.")
            else:
                print("There is no implementation to delete (generated "
                      "directory is empty).")
        else:
            print("Unknown command")
    except UnicodeDecodeError as ex:
        print("Unicode exception, check your locales\nUseful command: "
              "export LC_ALL=C.UTF-8")
        raise ex
    except Exception as ex:
        print("An error occurred during command execution")
        raise ex


if __name__ == '__main__':
    main()
