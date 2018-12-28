# Main data structure, contains both topics and parameters
# Defines a node's structure
class Module:
    def __init__(self, name, type, description):
        self.name = name    # module name (i.e. camera)
        self.type = type    # module categorization (i.e. sensor)
        self.desc = description # module short description
        self.topics = []    # list of the module's topics
        self.params = []    # list of the module's parameters
    def addTopic(self, topic):
        self.topics.append(topic)
    def addParam(self, param):
        self.params.append(param)

# Data structure for node topics, contains properties
# Defines the first-level communication artifacts of a module
class Topic:
    def __init__(self, name="", type="", mandatory=""):
        self.name = name    # topic name (i.e. status)
        self.type = type    # topic type (publish, subscribe, service...)
        self.mandatory = mandatory  # whether the topic must exist on the module
        self.properties = []    # a list of each of the topic's values
        self.response = []  # a list of each of the topic's response values (requires type to be a service)
        self.desc=None  # topic short description, optional
        self.fileName=None  # the name of the artifact to be generated from the topic
    def addProp(self, property):
        self.properties.append(property)
    def addRes(self, response):
        self.response.append(response)

# Data structure for topic properties, can contain subproperties (property referencing an artifact)
# Defines each value inside a topic/artifact property (property containing other properties)
class Property:
    def __init__(self, name="", type=None):
        self.name = name    # property name (i.e. device_name)
        self.type = type    # property data type (i.e. float64)
        self.properties = []    # an artifact property's own property list (no use if the property's type is a base data type)
        self.enumeration=[] # possible values in case of an enumeration property (no use if the property's type isn't enum)
        self.array = False  # whether the property holds a list of values
        self.length = None  # the length of said list (no use if the property isn't a list), optional
        self.value = None   # the property's value, can be a list, optional
        self.unit=None  # the unit the property represents (i.e. hertz), optional
        self.desc=None  # short description of the property, optional
        self.fileName=None  # name of the artifact to be generted from the property (no use if the property's type is a base data type)
    def addProp(self, property):
        self.properties.append(property)

# Data structure for node parameters
class Parameter:
    def __init__(self, name, type, mandatory, unit=None, description=None, value=None):
        self.name = name    # parameter name (i.e. frame_id)
        self.type = type    # parameter data type (i.e. string)
        self.mandatory = mandatory  # whether the parameter must exist on the module
        self.unit=unit  # the unit the parameter represents (i.e. radian), optional
        self.desc=description   # short description of the parameter, optional
        self.value=value    # the parameter's value, optional
        self.array = False  # whether the parameter holds a list of values
        self.length = None  #the length of said list (no use if the parameter isn't a list), optional
