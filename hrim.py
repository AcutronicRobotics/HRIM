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
    		elif args.filePath == "allClean":
    			fileList = findModels(os.path.join(path, "models"))
    			for item in fileList:
    				# if the model is a development file process it
    				if bool(re.search('.*_clean.xml$', item)):
    					os.chdir(path)
    					module = parser.parseFile(item)
    					compiler.compileModule(module, args.platform)
    		# else try to generate the implementation based on the passed file
    		else:
    			module = parser.parseFile(args.filePath)
    			compiler.compileModule(module, args.platform)
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
	parser.add_argument('action', choices=['show','generate'], help='''Action to take:
show:
	print a representation of the passed valid XML module's model structure and values.
generate:
	generate the platform-specific implementation of the passed valid XML model.
		''')
	parser.add_argument('filePath', help='The path to a valid xml model file.')
	parser.add_argument('-p', '--platform', default='ros2', choices=['ros2'], help='The platform for the generated model, ros2 by default.')
	parser.add_argument('-e', '--extend', action='store_true', default=False, help='Whether to expand topic definitions when "show"-ing.')

	# If no argument is provided, show usage
	if len(sys.argv) == 1:
		parser.print_help()
		sys.exit(0)

	args=parser.parse_args()

	main(args)
