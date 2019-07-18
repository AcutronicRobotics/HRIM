import argparse
import sys
import os
import xml.etree.ElementTree as ET


if __name__== "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--filename', help='file name')
    args = parser.parse_args()

    tree = ET.parse(args.filename);
    root = tree.getroot()

    with open('latex/template.tex', 'r') as content_file:
        content = content_file.read()

    string_concepts = ''
    string_characteristics = ''
    string_features = ''

    index = 0

    for property in root:
        tag = property.tag
        if str(tag) == "generic_specs":
            for x in property:

                if (index % 2 == 0):
                    string_concepts = string_concepts + '\\rowcolor[HTML]{C0C0C0} \n'

                name = x.attrib["name"]

                if "unit" in x.attrib:
                    unit = x.attrib["unit"]
                else:
                    unit = ""

                for value in x:
                    v = value.text
                name = name.replace("_", '\_')
                print(str(name) + " & " + str(v) + " & " + str(unit))
                string_concepts += str(name) + ' & ' + str(v) + " " + str(unit) + ' \\\\ \hline\n'
                index = index + 1
        else:
            for x in property:
                name = x.attrib["name"]

                if "unit" in x.attrib:
                    unit = x.attrib["unit"]
                else:
                    unit = ""

                for value in x:
                    v = value.text
                name = name.replace("_", '\_')
                string_characteristics += str(name) + ' & ' + str(v) + " " + str(unit) + ' \\\\ \hline\n'

    content = content.replace('{%CONCEPT%}', string_concepts)
    content = content.replace('{%CHARACTERISTICS%}', string_characteristics)
    content = content.replace('{%FEATURES%}', string_features)

    f = open("latex/output.tex", "w")
    f.write(content)
    f.close()

    cmd = 'pdflatex -no-file-line-error latex/output.tex'
    os.system(cmd)
    os.system('evince output.pdf')
