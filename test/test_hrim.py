import unittest
from hrim.scripts.utils import getTabs
from hrim.scripts.parsing import *
from hrim.scripts.classes import *

class HRIMTESTS(unittest.TestCase):
    def test_utils(self):
        print("Running test_utils")
        self.assertEqual("\t\t", getTabs(2))
        self.assertEqual("\t\t\t\t\t\t", getTabs(6))
        self.assertEqual("\t\t\t\t", getTabs(4))
        self.assertEqual("", getTabs(0))

    def test_ModularParser(self):
        print("Running test_ModularParser")
        module_parser = ModuleParser()

        self.assertEqual(None, module_parser.processValue(None, "int"))
        self.assertEqual(1, module_parser.processValue("1", "int"))
        self.assertEqual(1.5, module_parser.processValue("1.5", "double"))
        self.assertNotEqual(1.5, module_parser.processValue("1.55", "float"))

        datatype_str = {'boolean': 'bool', 'byte': 'byte', 'double': 'float64', 'enum': 'enum', 'float32': 'float32', 'float64': 'float64', 'int8': 'int8', 'int16': 'int16', 'int32': 'int32', 'string': 'string', 'uint8': 'uint8', 'uint16': 'uint16', 'uint32': 'uint32', 'uint64': 'uint64'}
        self.assertEqual(datatype_str, module_parser.getDataTypes("ros2"))
        self.assertNotEqual(datatype_str, module_parser.getDataTypes("none"))

    def test_Classes(self):
        module = Module("module1", "uint8", "description")
        self.assertEqual("module1", module.name)
        self.assertEqual("uint8", module.type)
        self.assertEqual("description", module.desc)
        module.addTopic("topic1")
        module.addTopic("topic2")
        module.addParam("param1")
        module.addParam("param2")
        module.addParam("param3")

        self.assertEqual(2, len(module.topics))
        self.assertEqual(3, len(module.params))

        topics = Topic("param1", "uint8", True)
        self.assertEqual("param1", topics.name)
        self.assertEqual("uint8", topics.type)
        self.assertEqual(True, topics.mandatory)
        topics.addProp("property1")
        topics.addRes("response1")
        topics.addRes("response2")
        topics.addFeed("feedback1")
        topics.addFeed("feedback2")
        topics.addFeed("feedback3")
        self.assertEqual(1, len(topics.properties))
        self.assertEqual(2, len(topics.response))
        self.assertEqual(3, len(topics.feedback))

        properties = Property("prop1", "uint8")
        self.assertEqual("prop1", properties.name)
        self.assertEqual("uint8", properties.type)

        properties.addProp("property1")
        properties.addProp("property2")
        self.assertEqual(2, len(properties.properties))

        parameters = Parameter("param1", "uint8", False, "m")
        self.assertEqual("param1", parameters.name)
        self.assertEqual("uint8", parameters.type)
        self.assertEqual(False, parameters.mandatory)
        self.assertEqual("m", parameters.unit)

if __name__ == '__main__':
    print("Running HRIMTESTS")
    unittest.main()
