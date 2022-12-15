# commandPort -n "localhost:7001" -stp "mel" -echoOutput;

from re import S
from select import select
from this import d
import maya.cmds as cmds
[driver, driven] = cmds.ls( selection=True )
print(driver, driven)

def loopArrays(edge):
    ret = []
    masterList = cmds.pickWalk(edge, d = "down", typ = "edgering")
    cmds.select(cl=True)
    masterList.reverse()
    for edge in masterList:
        currentLoop = cmds.pickWalk(masterList[1], d = "right", typ = "edgeloop")
        verts = cmds.polyListComponentConversion(currentLoop, fe= True,  tv = True)
        ret.append(verts)
    return(ret)

# def clusterManager(driver):
#     for verts in driver:
#         cmds.select(verts, add = True)


driverVerts = loopArrays(driver)
drivenVerts = loopArrays(driven)

# print(driverVerts)
for verts in driverVerts:
    cmds.select(verts, add = True)
for verts in drivenVerts:
    cmds.select(verts, add = True)

# # drivenVerts = loopArrays(driven)

# clusterManager(driver)
