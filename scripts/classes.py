class Module:
    def __init__(self, name, type, description):
        self.name = name
        self.type = type
        self.desc = description
        self.topics = []
        self.params = []
    def addTopic(self, topic):
        self.topics.append(topic)
    def addParam(self, param):
        self.params.append(param)

class Topic:
    def __init__(self, name="", type="", mandatory=""):
        self.name = name
        self.type = type
        self.mandatory = mandatory
        self.properties = []
        self.response = []
        self.desc=None
        self.fileName=None
        self.origin = ""
    def addProp(self, property):
        self.properties.append(property)
    def addRes(self, response):
        self.response.append(response)

class Property:
    def __init__(self, name="", type=None):
        self.name = name
        self.type = type
        self.properties = []
        self.enumeration=[]
        self.array = False
        self.length = None
        self.value = None
        self.unit=None
        self.desc=None
        self.origin = ""
    def addProp(self, property):
        self.properties.append(property)

class Parameter:
    def __init__(self, name, type, mandatory, unit=None, description=None, value=None):
        self.name = name
        self.type = type
        self.mandatory = mandatory
        self.unit=unit
        self.desc=description
        self.value=value
