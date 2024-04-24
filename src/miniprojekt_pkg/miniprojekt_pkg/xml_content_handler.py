import xml.sax

# Create a custom ContentHandler class
class MyContentHandler(xml.sax.ContentHandler):
    def __init__(self, list):
        self.att_list_ = list

    def startElement(self, name, attrs):
        self.current_list = []
        self.current_data = ""
        #print("Start element:", name)
        self.current_data = self.current_data + str("element: " + str(name))
        self.current_list.append(str(name))

    def endElement(self, name):
        #print("End element:", name)
        if name != "Pallet":
            self.att_list_.append(self.current_list)

    def characters(self, content):
        if content.strip():
            #print("Content:", content)
            self.current_data = str(self.current_data + content)
            self.current_list.append(str(content))