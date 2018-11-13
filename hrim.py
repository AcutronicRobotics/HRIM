import sys
sys.path.insert(0, 'scripts')
import printing
import parsing
from classes import *

def main(args):
	extend = False

	# if topic/properties not defined on the parsed file should show their full content or not when printing
	if len(args) == 3:
		if args[2] == "1":
			extend = True

	# check for file input
	if len(args) == 1:
		print "Please input file to parse"
		sys.exit()

	module = parsing.main(args[1])

	printing.main(module, extend)

if __name__ == '__main__':
	main(sys.argv)
