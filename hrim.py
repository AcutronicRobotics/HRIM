import sys
import os
os.chdir(os.path.abspath(os.path.dirname(__file__)))
sys.path.insert(0, 'scripts')
from printing import ModulePrinter
from parsing import ModuleParser
from compiling import ModuleCompiler
from classes import *
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

def main(args):
    try:
    	parser = ModuleParser()
    	if args.action == "show":
    		extend = args.extend
    		module = parser.parseFile(args.filePath)
    		ModulePrinter().printModule(module, args.platform, extend)
    	elif args.action == "generate":
    		path = os.getcwd()
    		generic = parser.parseGeneric()
    		compiler = ModuleCompiler()
    		compiler.compileGeneric(generic, args.platform)
    		print "Succesfully generated "+args.platform+" implementation of HRIM's generic package."
    		os.chdir(path)
    		# check for file generation shorthands
    		if args.filePath == "all":
    			fileList = findModels(os.path.join(path, "models"))
    			for item in fileList:
    				# if the model isn't a development file process it
    				if not bool(re.search('.*_clean.xml$', item)):
    					os.chdir(path)
    					module = parser.parseFile(item)
    					compiler.compileModule(module, args.platform)
    					print "Succesfully generated "+args.platform+" implementation of "+module.name+" module."
    		elif args.filePath == "allClean":
    			fileList = findModels(os.path.join(path, "models"))
    			for item in fileList:
    				# if the model is a development file process it
    				if bool(re.search('.*_clean.xml$', item)):
    					os.chdir(path)
    					module = parser.parseFile(item)
    					compiler.compileModule(module, args.platform)
    					print "Succesfully generated "+args.platform+" implementation of "+module.name+" module."
    		# else try to generate the implementation based on the passed file
    		else:
    			module = parser.parseFile(args.filePath)
    			compiler.compileModule(module, args.platform)
                print "Succesfully generated "+args.platform+" implementation of "+module.name+" module."
        elif args.action == "clear":
    		if len(os.listdir("generated")) > 0:
        		if args.filePath == "all":
        			delDirs = os.listdir("generated")
        			for delPath in os.listdir("generated"):
        				fullPath = os.path.join(os.getcwd(), "generated", delPath)
        				shutil.rmtree(fullPath)
        				print "Deleted "+fullPath
        		else:
        			fullPath = os.path.join(os.getcwd(), "generated", args.filePath)
        			if os.path.exists(fullPath):
        				shutil.rmtree(fullPath)
        				print "Deleted "+fullPath
        			else:
        				print "Couldn't find passed directory for deletion."
    		else:
        		print "There is no implementation to delete (generated directory is empty)."
    	else:
    		print "Unknown command"
    except:
        print "An error occurred during command execution"
        sys.exit(1)


if __name__ == '__main__':

	# Add usage messages

	parser=argparse.ArgumentParser(
	    description='''Hardware Robot Information Model (HRIM) implementation generation tool.''',
		formatter_class=argparse.RawTextHelpFormatter)
	parser.add_argument('action', choices=['show','generate', 'clear'], help='''Action to take:
show:
	print a representation of the passed valid XML module's model structure and values.
generate:
	generate the platform-specific implementation of the passed valid XML model.
clear:
	delete the passed generated implementation.
		''')
	parser.add_argument('filePath', help='''The path to a valid xml model file.
Alternatively, either a shorthand for the generate command:
    all:
        generates the implementation of every existent model
    allClean:
        same as all, but taking into account the development models (file name ends in _clean)
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

	main(args)
