import xml.dom.minidom

dom = xml.dom.minidom.parse("actuator0.XML") # or xml.dom.minidom.parseString(xml_string)
pretty_xml_as_string = dom.toprettyxml()
print(pretty_xml_as_string)