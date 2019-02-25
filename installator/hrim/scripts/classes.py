class Composition:
    def __init__(self, name):
        self.name = name
        self.modules = []
# Main data structure, contains both topics and parameters
# Defines a node's structure
class Module:
    def __init__(self, name, type, description):
        # module name (i.e. camera)
        self.name = name
        # module categorization (i.e. sensor)
        self.type = type
        # module short description
        self.desc = description
        # list of the module's topics
        self.topics = []
        # list of the module's parameters
        self.params = []
    def addTopic(self, topic):
        self.topics.append(topic)
    def addParam(self, param):
        self.params.append(param)

# Data structure for node topics, contains properties
# Defines the first-level communication artifacts of a module
class Topic:
    def __init__(self, name="", type="", mandatory=""):
        # topic name (i.e. status)
        self.name = name
        # topic type (publish, subscribe, service...)
        self.type = type
        # whether the topic must exist on the module
        self.mandatory = mandatory
        # a list of each of the topic's values
        self.properties = []
        # a list of each of the topic's response values (requires type to be a service)
        self.response = []
        # a list of each of the topics's feedback values (requires type to be an action)
        self.feedback = []
        # topic short description, optional
        self.desc = None
        # the name of the artifact to be generated from the topic
        self.fileName = None
        # the name of the interface's package of origin
        self.package = None
    def addProp(self, property):
        self.properties.append(property)
    def addRes(self, response):
        self.response.append(response)
    def addFeed(self, feedback):
        self.feedback.append(feedback)

# Data structure for topic properties, can contain subproperties (property referencing an artifact)
# Defines each value inside a topic/artifact property (property containing other properties)
class Property:
    def __init__(self, name="", type=None):
        # property name (i.e. device_name)
        self.name = name
        # property data type (i.e. float64)
        self.type = type
        # an artifact property's own property list
        # (no use if the property's type is a base data type)
        self.properties = []
        # possible values in case of an enumeration property
        # (no use if the property's type isn't enum)
        self.enumeration = []
        # whether the property holds a list of values
        self.array = False
        # the length of said list (no use if the property isn't a list), optional
        self.length = None
        # the property's value, can be a list, optional
        self.value = None
        # the unit the property represents (i.e. hertz), optional
        self.unit = None
        # short description of the property, optional
        self.desc = None
        # name of the artifact to be generted from the property
        # (no use if the property's type is a base data type)
        self.fileName = None
        self.package = None
    def addProp(self, property):
        self.properties.append(property)

# Data structure for node parameters
class Parameter:
    def __init__(self, name, type, mandatory, unit=None):
        # parameter name (i.e. frame_id)
        self.name = name
        # parameter data type (i.e. string)
        self.type = type
        # whether the parameter must exist on the module
        self.mandatory = mandatory
        # the unit the parameter represents (i.e. radian), optional
        self.unit = unit
        # short description of the parameter, optional
        self.desc = None
        # the parameter's value, optional
        self.value = None
        # whether the parameter holds a list of values
        self.array = False
        #the length of said list (no use if the parameter isn't a list), optional
        self.length = None
