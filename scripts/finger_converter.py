#!/usr/bin/env python


import xml.etree.ElementTree as ET
import os, sys

#file = open("Design.xacro", "r")

path = os.path.dirname(os.path.dirname(__file__))




def generateYAML(name, jointNames):
    string = """type: "effort_controllers/JointGroupPositionController"\njoints:"""
    for j in jointNames:
        string += "\n - " + j 

    for j in jointNames:
        string += "\n"+j+":\n pid: {p: 0.20, i: 0.10, d: 0.0}"

    file = open(path+"/config/"+name+".yaml", "w")
    file.write(string)
    file.close()



def convertFingers(root):

    addedLines = []
    toRemove = []
    jointNames = [] 

    for child in root:
        if(child.tag=="finger"):

            name = child.attrib["name"]
            segments = None
            base = None
            length=None
            origin=None
            rotation=None


            try:
                for param in child:
                    if(param.tag=="segments"):
                        segments = int(param.attrib["value"])
                    elif(param.tag=="base"):
                        base = param.attrib["name"]
                    elif(param.tag=="length"):
                        length = float(param.attrib["value"])
                    elif(param.tag=="origin"):
                        try:
                            origin = param.attrib["xyz"]
                        except:
                            origin = "0 0 0"
                        try:
                            rotation=param.attrib["rpy"]
                        except:
                            rotation="0 0 0"
            except:
                print("MISSING PARAMETERS FOR FINGER")
                break

            toRemove.append(child)
            

            leng = length/segments
            lengthString = '"%s"' % (leng)

            addedLines.append("\n <!--  MACROS FOR FINGER: " + name + "-->\n")
            addedLines.append("\n<xacro:seg name=" + ('"%s"' % (name+"seg1")) + " length="+lengthString+"/>\n")
            addedLines.append("<xacro:seg_joint_offset seg1=" + ('"%s"' % (base)) + " seg2=" + ('"%s"' % (name+"seg1")) + " length="+ lengthString + " offset=" + ('"%s"' % (origin)) +  " rotation=" +  ('"%s"' % (rotation)) + " />\n")
            addedLines.append("<xacro:seg_trans seg1=" + ('"%s"' % (base)) + " seg2="+ ('"%s"' % (name+"seg1")) + "/>\n")
            
            
            jointNames.append(base+"_"+name+"seg1_joint")
            for n in range(1,segments):
                segName = '"%s"' % (name+"seg" +str(n+1))
                parentName = '"%s"' % (name+"seg" +str(n))
                addedLines.append("<xacro:segment name=" +segName+" parent="+parentName + " length="+lengthString+"/>\n")
                jointNames.append(name+"seg"+str(n)+"_"+name+"seg"+str(n+1)+"_joint")

    generateYAML("fingers", jointNames)

    for c in toRemove:
        root.remove(c)

    addedLines.append("</robot>")
    return addedLines

#CONVERT xacro file

tree = ET.parse(path+"/urdf/mainDesign.xacro") #Open design
root = tree.getroot()
addedLines = convertFingers(root) #Remove instances of fingers, get list of replacement commands
tree.write(path+"/scripts/output.xacro") #Write to intermediate file


file = open(path+"/scripts/output.xacro","r") #open intermediate file
lines = file.readlines()
for l in range(len(lines)):
    lines[l] = lines[l].replace("ns0", "xacro") #Replace XML namespace stuff with xacro
file.close()

file=open(path+"/urdf/output.xacro","w")
file.writelines(lines[:-1])  #Write file with xacro replacement - no </robot> line
file.writelines(addedLines) #Add macro calls to generate fingers
file.close()

print("CONVERTED")

#Create 
