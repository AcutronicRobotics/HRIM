import sys
sys.path.insert(0, 'scripts')
import printing
import parsing
import compiling
from classes import *

def main(args):
	if args[1] == "show":
		extend = False
		# if topic/properties not defined on the parsed file should show their full content or not when printing
		if len(args) == 4:
			if args[3] == "1":
				extend = True
		# check for file input
		if len(args) < 3:
			print "Please input file to parse"
			sys.exit()
		module = parsing.main(args[2])
		printing.main(module, extend)
	elif args[1] == "generate":
		extend = False
		# if topic/properties not defined on the parsed file should show their full content or not when printing
		if len(args) == 4:
			if args[3] == "1":
				extend = True
		# check for file input
		if len(args) < 3:
			print "Please input file to parse"
			sys.exit()
		module = parsing.main(args[2])
		compiling.main(module)
	else:
		print "Unknown command"



if __name__ == '__main__':
	main(sys.argv)
