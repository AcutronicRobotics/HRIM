function generateIDForm(){

  var textID = "<?xml version=\"1.0\"?>\n" +
  "<topic name=\"id\" type=\"service\" fileName=\"ID\" package=\"hrim_generic_srvs\">\n"+
  "  <response>\n"+
  "    <property name=\"device_kind_id\" type=\"uint8\" unit=\"enum\" enum='{\"HRIM_COMM\": 0, \"HRIM_SENSOR\": 1, \"HRIM_ACTUATOR\": 2, \"HRIM_COGNITION\": 3, \"HRIM_UI\": 4, \"HRIM_POWER\": 5, \"HRIM_COMPOSITE\": 6}' description=\"device classification\">\n"+
  "        <value/>\n"+
  "    </property>\n"+
  "    <property name=\"device_name\" type=\"string\" description=\"sub-type\">\n"+
  "        <value/>\n"+
  "    </property>\n"+
  "    <property name=\"vendor_id\" type=\"uint32\" description=\"vendor identifier\">\n"+
  "        <value/>\n"+
  "    </property>\n"+
  "    <property name=\"product_id\" type=\"uint32\" description=\"part number\">\n"+
  "        <value/>\n"+
  "    </property>\n"+
  "    <property name=\"instance_id\" type=\"uint32\" description=\"module unique identifier\">\n"+
  "        <value/>\n"+
  "    </property>\n"+
  "    <property name=\"hrim_version\" type=\"string\" description=\"used hrim version\">\n"+
  "        <value/>\n"+
  "    </property>\n"+
  "    <property name=\"hros_version\" type=\"string\" description=\"used h-ros version\">\n"+
  "        <value/>\n"+
  "    </property>\n"+
  "  </response>\n"+
  "</topic>\n"

  var parser = new DOMParser();
  var xmlDoc = parser.parseFromString(textID, "text/xml");

  var myCollection = xmlDoc.getElementsByTagName("topic")[0].getElementsByTagName("response")[0].getElementsByTagName("property")

  // console.log(myCollection)

  elements_of_the_form = ""
  elements_of_the_table = ""

  for (i = 0; i < myCollection.length; i++) {
    // console.log(myCollection[i])
    // console.log(myCollection[i].getAttribute("name"))

    elements_of_the_form += myCollection[i].getAttribute("name") + ":" +
      "<input type=\"text\" name=\"" + myCollection[i].getAttribute("name") + "\"><br>"

    elements_of_the_table += "<tr><td>"+ myCollection[i].getAttribute("name") +"</td><td>"+ myCollection[i].getAttribute("description") +" </td></tr>"
  }

  // document.getElementById("demo").innerHTML = "<table border=\"1\"><tr><th>Concept</th><th>Description</th></tr>" + elements_of_the_table + "</table>"
  // console.log("<form> " + elements_of_the_form + "</form>")
  document.getElementById("formhrim").innerHTML = "<form> " + elements_of_the_form + "</form>"


  return xmlDoc;

}
