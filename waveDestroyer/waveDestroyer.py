from maya import cmds as mc
import pymel.core as pm



def generate() :
    exists = pm.ls("container")
    if(len(exists) != 0):
        return
    pm.group( em=True, name='container' )
    for i in range(-10, 10):
        for x in range(-10, 10):
            pm.polyCube()
            pm.move(i + (i * .1), 1, x + (x*.1))
            pm.group(parent = "container")
    
generate()