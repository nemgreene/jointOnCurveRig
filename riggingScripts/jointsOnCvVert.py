from maya import cmds as mc
import pymel.core as pm


# user should select first the curve, then the root joint

# make sure the curve is linear!!

[curve, locator]  = pm.ls(selection=True)
# print(curve, rootLocator)
# curve = 'curve1'
# locator = "locator1"
name = "mouth_lower"


def clearCache():
    pm.delete("mouth_lower*")


# the script should start by attaching one locator to every vert on the curve
def locatorOnVert() :
    locatorGroup = pm.group(empty=1, n=name + "_jOCVC_locators")
    pm.select(clear=1)
    rootJoint = pm.joint( n=name + "_jointOnCVRoot", p= pm.getAttr(str(locator) + '.t'))
    curveVerts = pm.ls(curve + '.cv[*]', fl =1)
    for i in range(len(curveVerts)):
        pm.select(clear=1)
        # create locator
        # A locator will be pointing down the curve
        itLocatorA = pm.spaceLocator(n=name + "_jCOCV_driverLocatorA_" + str(i))
        pm.setAttr(itLocatorA + ".localScale", (.1, .1, .1))
        iteratorPath = pm.pathAnimation(itLocatorA, stu=0, etu=30, c =curve, f = 1,  n = name + "_driverMotionPath" + str(i) )
        # B locator will be pointing up the
        # break driven key
        pm.disconnectAttr(str(iteratorPath) + ".uValue")
        # move to corresponding vert
        pm.setAttr(str(iteratorPath) + ".uValue", i)
        # once the locator is set up
        pm.select(clear=1)
        # create root joint
        itJoint = pm.joint( n=name + '_jOCVC_parent_' + str(i) , p= pm.getAttr(str(locator) + '.t'))
        pm.setAttr(itJoint + ".radius", .3)
        # aim root joint at locator
        pm.aimConstraint(itLocatorA, itJoint)
        # create child joint
        childJoint = pm.duplicate(itJoint, ic=1)[0]
        pm.rename(childJoint, name + '_bind_jOCVC_child_' + str(i))
        # delete childs AimCOnstriant
        pm.delete(pm.pickWalk(childJoint, d='down'))
        # disconnect child rotate
        pm.disconnectAttr(childJoint)
        # move child to locator
        pm.move(childJoint , pm.getAttr(str(itLocatorA) + ".t"))
        pm.setAttr(childJoint + ".radius", .3)
        if(i == 0 or i == len(curveVerts) -1 ):
            pm.orientConstraint(itLocatorA, childJoint)
            # corner
        else:
            itLocatorB = pm.duplicate(itLocatorA, ic=1)[0]
            pm.rename(itLocatorB, name + "_jCOCV_driverLocatorB_" + str(i))
            pm.disconnectAttr(str(itLocatorB) + ".rx")
            pm.disconnectAttr(str(itLocatorB) + ".ry")
            pm.disconnectAttr(str(itLocatorB) + ".rz")
            prevLocator = pm.ls(name + "_jCOCV_driverLocatorA_" + str(i-1))
            pm.aimConstraint(prevLocator, itLocatorB, aim = [0, -1, 0], u = [0,0,1])
            childOrient = pm.orientConstraint(itLocatorA, itLocatorB, childJoint)
            pm.setAttr(str(childOrient)+".interpType", 2)
            pm.parent(itLocatorB, locatorGroup)



        # add child joint to arr for later reOrienting
        # child root to master
        # parent child to root
        pm.parent(childJoint, itJoint)
        pm.parent(itJoint, rootJoint)
        pm.parent(itLocatorA, locatorGroup)
    pm.rename(locator, name + "jOCVC_root_loc")
    pm.parent(curve, rootJoint )
    pm.rename(curve, name + "jOCVC_root_curve")
    pm.parent(name + "jOCVC_root_loc", rootJoint )

def createControlRig():
    # create groups
    clusterGroup = pm.group(em= 1, name=name + "_clusters01")
    ccGroup = pm.group(em= 1, name=name + "_CC01")
    # select CVS to iterate over
    curveVerts = pm.ls(curve + '.cv[*]', fl =1)
    for i in range(len(curveVerts)):
        # select vert
        pm.select(curveVerts[i])
        # make cluster
        itCLuster = pm.cluster(name=name + 'cluster_' + str(i))[1]
        pm.select(cl=1)
        # parent cluster into groups
        pm.parent(itCLuster, clusterGroup)
        # make control
        ctrl = pm.circle(name = name + "_cc_" + str(i))[0]
        # position and orient cc
        i = pm.pointConstraint(itCLuster, ctrl)
        x = pm.aimConstraint(locator, ctrl, aim = [0, 0, 1])
        pm.delete(i)
        pm.delete(x)
        # move cc out slightly
        pm.move(0, 0, -.5, ctrl,  relative=True, objectSpace=True, worldSpaceDistance=True)
        pm.scale(ctrl, .7, .7, .7)
        # freezeTransformation and delete history
        pm.makeIdentity(a = 1)
        pm.delete(ch = 1)

        # connect cluster to cc
        pm.pointConstraint(ctrl, itCLuster, mo=1)
        pm.parent(ctrl, ccGroup)

        

# clearCache()
locatorOnVert()
createControlRig()





