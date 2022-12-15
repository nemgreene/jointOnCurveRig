from unicodedata import name
import pymel.core as pmc

def Mouth():
    pmc.delete('cleanup')
    cleanup = pmc.group(empty = True, name = "cleanup")
    # grab ccs by name
    top, bottom, pivot = pmc.ls('driver_top', 'driver_bottom', 'driver_pivot')
    pivotJoint = pmc.joint(n="jtAlign_mouthPivot_01")
    pmc.parent(pivotJoint, cleanup)

    #it over curves
    #make a new joint
    # return {corners:[]} 
    def makeJoints(curve, name):
        curvesCvs = pmc.ls(str(curve) + '.cv[*]', flatten=1)
        for index, cv in enumerate(curvesCvs[1:-1]):
            # print(pmc.pointPosition(cv))
            locAlign = pmc.spaceLocator(n = 'locAlign_' + name + "_"+ str(index + 1))
            anim = pmc.pathAnimation(locAlign, c = curve, f = 1)
            pmc.disconnectAttr(str(anim) + '.uValue')
            pmc.setAttr(str(anim) + '.uValue', index + 1)
            pmc.parent(locAlign, cleanup)

    makeJoints(top, "top")
    # makeJoints(bottom.slice(1, -1))

Mouth()