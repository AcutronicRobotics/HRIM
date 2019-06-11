import sys
import json
import re
import os
import lxml.etree as et
from .classes import Composition, Module, Topic, Property, Parameter


class ModuleParser:

    # calling-path independent method to get a platforms data type mapping
    @staticmethod
    def get_data_types(plat):
        try:

            data_types = {}

            # save the current path
            base_path = os.path.abspath(os.getcwd())

            # parse the mapping file
            data_tree = et.parse(os.path.join(os.getcwd(), "models",
                                              "dataMapping.xml"))
            data_root = data_tree.getroot()

            # return to the previous path
            os.chdir(base_path)

            # map the data types
            if any(platform.attrib.get("name") == plat for platform
                   in data_root):
                for platform in data_root.iter("platform"):
                    if platform.attrib.get("name") == plat:
                        for dataType in platform.iter("type"):
                            data_types[dataType.attrib.get("name")] = \
                                dataType.attrib.get("value")
            else:
                print("Chosen platform doesn't exist")
                sys.exit(1)

        except Exception as ex:
            print("Error parsing dataType mapping file")
            raise ex

        return data_types

    # check for value type, else all values'll be strings
    @staticmethod
    def process_value(value, process_value_type):
        if value is None or len(value) == 0:
            return None
        # if numeric with no decimals (bool would either be 0 or 1)
        if process_value_type in ["int", "uint", "bool"]:
            return int(value)
        # if numeric with decimals
        if process_value_type in ["float", "double"]:
            return float(value)
        # else return the value without modifying, it's treated as a string
        return value

    # topic parsing and processing
    def process_topic(self, topic, mandatory):
        top = Topic(topic.attrib.get("name"), topic.attrib.get("type"),
                    mandatory)
        if "description" in topic.attrib:
            top.desc = topic.attrib.get("description")
        if "package" in topic.attrib:
            top.package = topic.attrib.get("package")
        top.fileName = topic.attrib.get("fileName")
        for prop in topic:
            if prop.tag == "property":
                top.add_prop(self.process_property(prop))
            if prop.tag == "response":
                for res in prop:
                    top.add_res(self.process_property(res))
            if prop.tag == "feedback":
                for res in prop:
                    top.add_feed(self.process_property(res))
        return top

    # property and sub property recursive parsing and processing
    def process_property(self, process_property):
        if "type" in process_property.attrib:
            type_list = process_property.attrib.get("type")

            # check for array declaration in type
            if re.search(r"\[.*\]", type_list):
                prop = Property(process_property.attrib.get("name"),
                                re.sub(r"\[.*\]", "", type_list))
                prop.array = True

                # check for array length if specified
                matches = re.finditer(r"\[\D*(\d+)\]", type_list)
                for i, match in enumerate(matches):
                    prop.length = int(match.group(1))
            else:
                prop = Property(process_property.attrib.get("name"), type_list)
        else:
            prop = Property(process_property.attrib.get("name"))
        if "unit" in process_property.attrib:
            prop.unit = process_property.attrib.get("unit")

            # check for enumeration values
            if prop.unit == "enum" and "enum" in process_property.attrib:
                prop.enumeration = json.loads(process_property.attrib
                                              .get("enum"))
        if "description" in process_property.attrib:
            prop.desc = process_property.attrib.get("description")

        if "fileName" in process_property.attrib:
            prop.fileName = process_property.attrib.get("fileName")

        if "package" in process_property.attrib:
            prop.package = process_property.attrib.get("package")

        # check for sub properties
        if any(x.tag == "property" for x in process_property):

            # process sub properties
            for subProp in process_property:
                if subProp.tag == "property":
                    prop.add_prop(self.process_property(subProp))

        # else process the value
        else:

            # if array assume multiple values
            if prop.array:

                # if length is specified fill every entry before appending
                # value
                if prop.length is not None:
                    value = [None] * prop.length
                    i = 0
                    for tag in process_property:
                        if tag.tag == "value" and i < prop.length:
                            value[i] = tag.text
                            i += 1

                # else simply append value to the array
                else:
                    value = []
                    for tag in process_property:
                        if tag.tag == "value":
                            value.append(tag.text)

            # else process a single value
            else:
                value = process_property[0].text
            prop.value = value
        return prop

    # parameter parsing and processing
    def process_param(self, parameter, mandatory):
        # check for array declaration in type
        if re.search(r"\[.*\]", parameter.attrib.get("type")):

            param = Parameter(
                parameter.attrib.get("name"), re.sub(r"\[.*\]",
                                                     "",
                                                     parameter.attrib.get(
                                                         "type")),
                mandatory, parameter.attrib.get("unit")
            )
            param.desc = parameter.attrib.get("description")

            param.array = True

            # check for array length if specified
            matches = re.finditer(r"\[\D*(\d+)\]",
                                  parameter.attrib.get("type"))
            for i, match in enumerate(matches):
                param.length = int(match.group(1))

            if param.length is not None:
                value = [None] * param.length
                i = 0
                for tag in parameter:
                    if tag.tag == "value" and i < param.length:
                        value[i] = self.process_value(tag.text, param.type)
                        i += 1
            else:
                value = []
                for tag in parameter:
                    if tag.tag == "value":
                        value.append(self.process_value(tag.text, param.type))
            param.value = value
            return param
        param = Parameter(
            parameter.attrib.get("name"), parameter.attrib.get("type"),
            mandatory, parameter.attrib.get("unit")
        )
        param.desc = parameter.attrib.get("description")
        param.value = self.process_value(parameter[0].text, param.type)
        return param

    def parse_base(self, path):
        generic_topics = []

        try:
            gen_tree = et.parse(path)
            gen_tree.xinclude()
            gen_root = gen_tree.getroot()
            for topic in gen_root:
                generic_topics.append(self.process_topic(topic, True))
        except Exception as ex:
            print("Problem parsing generic topics")
            raise ex
        return generic_topics

    def parse_composition(self, path):
        comp_tree = et.parse(path)
        comp_root = comp_tree.getroot()
        composition = Composition(comp_root.attrib.get("name"))
        for model in comp_root:
            topic_list = []
            for topic in model.findall("topic"):
                topic_list.append(topic.attrib.get("name"))
            param_list = []
            for param in model.findall("param"):
                param_list.append(param.attrib.get("name"))
            model_path = os.path.join(os.getcwd(), model.attrib.get("path"))
            module = self.parse_file(model_path)
            for topic in list(module.topics):
                if not topic.mandatory and topic.name not in topic_list:
                    module.topics.remove(topic)
            for param in list(module.params):
                if not param.mandatory and param.name not in param_list:
                    module.params.remove(param)
            composition.modules.append(module)
        return composition

    # main parse method
    def parse_file(self, file_path):

        # check for parsability
        try:
            if os.path.exists(file_path):
                path = file_path
            else:
                path = os.getcwd() + file_path
            tree = et.parse(path)
        except Exception as ex:
            print("Problem parsing " + file_path)
            raise ex

        tree.xinclude()
        root = tree.getroot()

        # instantiate module
        module = Module(
            root.attrib.get("name"), root.attrib.get("type"),
            root.attrib.get("description")
        )

        # loop through mandatory topics and parameters
        for tag in root[0]:
            if tag.tag == "topic":
                module.add_topic(self.process_topic(tag, True))
            if tag.tag == "param":
                module.add_param(self.process_param(tag, True))

        # check for optional topics/parameters
        if len(root) > 1:
            # loop through optional topics and parameters
            for tag in root[1]:
                if tag.tag == "topic":
                    module.add_topic(self.process_topic(tag, False))
                if tag.tag == "param":
                    module.add_param(self.process_param(tag, False))

        return module

    def __init__(self):
        pass
