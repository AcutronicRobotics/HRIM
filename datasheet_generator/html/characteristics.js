function generateCharacteristicsFormRotaryServo(){

text = "<component_specs>\n" +
"  <property name=\"control_type\" type=\"uint8\" unit=\"enum\" enum='{\"CONTROL_TYPE_NONE\": 0, \"CONTROL_TYPE_POSITION\": 1, \"CONTROL_TYPE_EFFORT\": 2, \"CONTROL_TYPE_VELOCITY\": 3, \"CONTROL_TYPE_POSITION_VELOCITY\": 4, \"CONTROL_TYPE_POSITION_EFFORT\": 5, \"CONTROL_TYPE_VELOCITY_EFFORT\": 6, \"CONTROL_TYPE_POSITION_VELOCITY_EFFORT\": 7}' description=\"rotary servomotor control type\"> \n" +
"      <value>4</value>\n" +
"  </property>\n" +
"  <property name=\"precision\" type=\"float64\" unit=\"rad\" description=\"angular precision\">\n" +
"      <value>0.00008722222</value>\n" +
"  </property>\n" +
"  <property name=\"rated_speed\" type=\"float64\" unit=\"rad/s\" description=\"servomotor speed\">\n" +
"      <value>1.46607657</value>\n" +
"  </property>\n" +
"  <property name=\"reachable_speed\" type=\"float64\" unit=\"rad/s\" description=\"maximum speed\">\n" +
"      <value>1.46607657</value>\n" +
"  </property>\n" +
"  <property name=\"rated_torque\" type=\"float64\" unit=\"N/m\" description=\"servomotor torque\">\n" +
"      <value>9</value>\n" +
"  </property>\n" +
"  <property name=\"reachable_torque\" type=\"float64\" unit=\"N/m\" description=\"peak torque\">\n" +
"      <value>13</value>\n" +
"  </property>\n" +
"</component_specs>"

  var parser = new DOMParser();
  var xmlDoc = parser.parseFromString(text, "text/xml");

  var myCollection = xmlDoc.getElementsByTagName("component_specs")[0].getElementsByTagName("property")
  elements_of_the_form = ""

  for (i = 0; i < myCollection.length; i++) {
    elements_of_the_form += "<tr><td>" + myCollection[i].getAttribute("name") + "</td><td>" +
      "<input type=\"text\" name=\"" + myCollection[i].getAttribute("name") + "\"></td></tr>"

    elements_of_the_table += "<tr><td>"+ myCollection[i].getAttribute("name") +"</td><td>"+ myCollection[i].getAttribute("description") +" </td></tr>"
  }
  document.getElementById("formcharacteristics").innerHTML = "<form> <table>" + elements_of_the_form + "</table></form>"
}


function generateCharacteristicsFormCamera(){

text = "<component_specs>\n" +
"	<property name=\"resolution\" type=\"string\" unit=\"pixel\" description=\"example: 752x480, 1280x1024\">\n" +
"      <value>640x480,320x240</value>\n" +
"  </property>\n" +
"  <property name=\"horizontal_fov\" type=\"float32\" unit=\"radian\" description=\"horizontal field of view\">\n" +
"      <value>58</value>\n" +
"  </property>\n" +
"  <property name=\"vertical_fov\" type=\"float32\" unit=\"radian\" description=\"vertical field of view\">\n" +
"      <value>45</value>\n" +
"  </property>\n" +
"  <property name=\"frame_rate\" type=\"uint16\" unit=\"frames per second\" description=\"maximum possible frame rate\">\n" +
"      <value>60</value>\n" +
"  </property>\n" +
"  <property name=\"accuracy\" type=\"float32\" unit=\"metre\" description=\"minimum possible accuracy value\">\n" +
"      <value>0.1</value>\n" +
"  </property>\n" +
"  <property name=\"operation_range\" type=\"uint16[]\" unit=\"metre\" description=\"working distance (min-max)\">\n" +
"      <value>6.0</value>\n" +
"  </property>\n" +
"  <property name=\"image_formats\" type=\"string\" description=\"example: mono8, rgb1, bayergrbg8\">\n" +
"      <value>mono8,rgb8</value>\n" +
"  </property>\n" +
"  <property name=\"rgb\" type=\"boolean\" description=\"true: image in RGB (color); false: image in grayscale\">\n" +
"      <value>true</value>\n" +
"  </property>\n" +
"</component_specs>"

var parser = new DOMParser();
var xmlDoc = parser.parseFromString(text, "text/xml");

var myCollection = xmlDoc.getElementsByTagName("component_specs")[0].getElementsByTagName("property")
elements_of_the_form = ""

for (i = 0; i < myCollection.length; i++) {
  elements_of_the_form += "<tr><td>" + myCollection[i].getAttribute("name") + "</td><td>" +
    "<input type=\"text\" name=\"" + myCollection[i].getAttribute("name") + "\"></td></tr>"

  elements_of_the_table += "<tr><td>"+ myCollection[i].getAttribute("name") +"</td><td>"+ myCollection[i].getAttribute("description") +" </td></tr>"
}
document.getElementById("formcharacteristicscamera").innerHTML = "<form><table>" + elements_of_the_form + "</table></form>"
}
