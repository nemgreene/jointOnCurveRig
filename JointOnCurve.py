from os import stat
import pymel.core as pmc
import re
import math
import time

# Grab Currrent Time Before Running the Code
start = time.time()


# The intention of this script is to automate the generation of a Joint on curve rig
# Still in alpha stages

# Needs in the world scene:
# joint: "driver_jawPivot"
#   aligns with the jaw pivot on the rig
# nurbsCvs x2: "driver_top, driver_bottom", scene left to right
#   this will align with the the verts on the edge loop for the mouth/eyelid openning
# spaceLocator: "driver_pivot" 
#   the premise of the rig rotates around a single pivot to replicate the volume of the muzzle/eyeball
#   this pivot sits at the center of that


# ------------------------------------------------------------------------------------------------------------------------------
# TODO
# Handle z translation mouthwide/individual
# flatten is set up on the ccs side, need to teach the parent joints how to scale 
# axis agnostic
# ------------------------------------------------------------------------------------------------------------------------------

# Utility function, extracts component number from name
# input str("driver_top.cv[1]")
# output int(1)
def xtractNumber(str):
    return int(re.findall("\[(\d+)\]", str)[0])

# Utility function calculates 2d euclidean distance
def euclidean_distance(x, y):
    return math.sqrt(sum((px - py) ** 2 for px, py in zip(x, y)))

def pivotToComponent(driver, driven):
            [x,y,z] = pmc.xform(str(driver), ws=1, q=1, t=1)
            pmc.move(x,y,z,str(driven) + ".scalePivot",str(driven) + ".rotatePivot",rpr=1,a=1,)


# Generate mouth 
def Mouth():
    #region
    
    # look for a cleanup folder to delete
    try:
        pmc.delete("cleanup")
    except:
        print("No cleanup pal")
    # initialize cleanup folder to store everything into
    cleanup = pmc.group(empty=True, name="cleanup")

    #Creating empty groups to assign the rig hierarchy into 
    # hidden/static
    static = pmc.group(empty=True, name="grpStatic_mouthRig01")
    locators = pmc.group(empty=True, name="locators_mouth01")
    clusters = pmc.group(empty=True, name="clusters_mouth01")

    # moveable
    translate = pmc.group(empty=True, name="grpTranslate_mouthRig01")
    jointsOffset = pmc.group(empty=True, name="joints_offset_mouth01")
    joints = pmc.group(empty=True, name="joints_mouth01")

    ccs = pmc.group(empty=True, name="ccs_mouth01")
    ccsLeft = pmc.group(empty=True, name="ccs_mouthL01")
    ccsRight = pmc.group(empty=True, name="ccs_mouthR01")

    # Handle scaling for the rightCC group, potentially superfluous
    pmc.scale(ccsRight, [1, 1, -1])
    pmc.makeIdentity(s=1, apply=1)
    pmc.scale(ccsRight, [-1, 1, 1])

    pmc.hide(clusters)

    pmc.parent([translate, static], cleanup)
    pmc.parent([locators, clusters], static)
    pmc.parent([ccsLeft, ccsRight], ccs)
    pmc.parent(joints, jointsOffset)
    pmc.parent([ccs, jointsOffset], translate)
    # end group hierarchy setup


    # grab drivers by name
    top, bottom, pivot, jaw = pmc.ls("driver_top", "driver_bottom", "driver_pivot", "driver_jawPivot")
    pmc.select(cl=1)
    pivotJoint = pmc.joint(
        n="jtAlign_mouthPivot_01", p=pmc.pointPosition(pivot), radius=0.5
    )
    pmc.parent(pivotJoint, joints)

    #Simulate mouth joints being half moved down when mouth opens 
    pivotToComponent(jaw, joints)
    pivotToComponent(jaw, jointsOffset)

    # pmc.orientConstraint(joints)

    # generate driver curves that deform ccs and main curve
    # weve crrently got bshpDrivers --> drivers --> loc --> ccs --> curve --> joints
    # this function makes the bshpDrivers, drivers
    # cuts bshpDrivers and reorients them
    # makes spaceLocators that will be used to drive the ccs down the road

    #endregion
    
    def makeDrivers(top, bottom):
        driverLocs = pmc.group(em=1, n="grpLoc_driverCurve_mouth01")
        pmc.parent(driverLocs, locators)
        # for the top and bottom curves that exist
        for index, curve in enumerate([top, bottom]):
            label = "_top01" if index == 0 else "_bottom01"
            # make a duplicate that will drive its ccs
            driver = pmc.duplicate(
                curve, n="ccDriver_mouth" + label
            )
            # make a secondary curve. 
            # this will be split up, then have the open/smile/frown clusters attached to them 
            # bshpDriver -> driver -> ccs -> curve -> joints
            bshpDriver = pmc.duplicate(
                curve, n="bshpDriver_mouth" + label
            )
            # select cvs from top/bottom
            curvesCvs = pmc.ls(str(curve) + ".cv[*]", flatten=1)
            # cut curves in half. Not sure this is actually necessary since ive moved away from a bshp model over to clusters, 
            # but will need refactoring down the stream if not 
            target = str(bshpDriver[0]) + ".u[" + str((len(curvesCvs) - 1) / 2) + "]"
            left, right = pmc.detachCurve(target, rpo=1)

            pmc.rename(
                right,
                "bshpDriver_mouth" + ("_topLeft01" if index == 0 else "_bottomLeft01"),
            )
            pmc.rename(
                left,
                "bshpDriver_mouth"
                + ("_topRight01" if index == 0 else "_bottomRight01"),
            )
            # set up curves to have a mirrored steup, cv[0] at corners
            pmc.reverseCurve(left)

            # left/right -> drives -> driver
            wire = pmc.wire(driver[0], w=[left, right], gw=0, en=1, ce=0, li=0)
            pmc.setAttr(str(wire[0]) + ".dropoffDistance[0]", .001)
            pmc.setAttr(str(wire[0]) + ".dropoffDistance[1]", .001)
            # pmc.hide(driver)
            baseWires = pmc.ls(
                str(wire[1]) + "BaseWire*",
                str(wire[2]) + "BaseWire*",
                type="transform",
            )
            pmc.parent(driver, bshpDriver, translate)
            pmc.showHidden(left, right)
            pmc.parent(left, baseWires, translate)

            # generate locators on driverWire that will be used to control ccs
            for idx, cvs in enumerate(curvesCvs):
                y = "top_" if index == 0 else "bottom_"
                nameIndex = ""
                if idx == 0:
                    nameIndex = "L_corner01"
                elif idx == (len(curvesCvs) - 1):
                    nameIndex = "R_corner01"
                else:
                    nameIndex = str(idx)
                locAlign = pmc.spaceLocator(n="locAlign_mouthDriver_" + y + nameIndex)
                pmc.setAttr(str(locAlign) + "Shape.visibility", 0)
                animPos = pmc.pathAnimation(locAlign, c=driver[0], f=1)
                pmc.disconnectAttr(str(animPos) + ".uValue")
                pmc.setAttr(str(animPos) + ".uValue", idx)
                pmc.setAttr(str(locAlign) + ".localScale", (0.4, 0.4, 0.4))
                pmc.parent(locAlign, locators)

    
    def makeShapeClusters():
        # access 4 bshp curves
        bl, br, tl, tr = pmc.ls("bshpDriver*01", flatten=1, type="transform")
        # cluster/offset all for the mouth open
        grpOpen, clusterOpen = clusterAndOffset(
            [bl, br, tl, tr],
            ("clusterDriver_mouth_open01"),
        )

        # subroutine to set the cluster weights for opening the mouth using clusterOPen
        def makeOpen(curve, name, offset):
            cvs = pmc.ls(str(curve) + ".cv[*]", flatten=1)
            # currently using pythag derivative to graph a circle for the weights in [0-1] space
            # index 0 == weight 0
            # index len(cvs) == 1
            # essentially plots a quarter circle
            def falloff(num, offset):
                a = round(
                    (
                        (
                            math.sqrt((len(cvs) ** 2) - ((len(cvs) - num) ** 2))
                            / len(cvs)
                        )
                        / 2
                    )
                    * offset
                    + 0.5,
                    4,
                )
                return a

            for index, cv in enumerate(cvs):
                pmc.percent(
                    clusterOpen[0],
                    cv,
                    v=falloff(index, offset),
                )
            pmc.parent(grpOpen, clusters)

        # subroutine that centers the pivot of the curves to the highest index see also furthest forward
        # to allow scaling to flatten to front 
        def makePurse(curve, name):
            cvs = pmc.ls(str(curve) + ".cv[*]", flatten=1)
            highest = cvs[-1]
            x, y, z = pmc.pointPosition(highest)
            pmc.move(
                x,
                y,
                z,
                str(curve) + ".scalePivot",
                str(curve) + ".rotatePivot",
                rpr=1,
                a=1,
            )
            return

        # subroutine to create the smileCLuster
        def makeSmile(curve, name):
            cvs = pmc.ls(str(curve) + ".cv[*]", flatten=1)
            grp, cluster = clusterAndOffset(
                cvs,
                ("clusterDriver_mouth" + name + "_smile01"),
            )
            if name[1] == "R":
                pmc.transformLimits(cluster[1], tx = (0, 0), etx =(1,0))
            else :
                pmc.transformLimits(cluster[1], tx = (0, 0), etx =(0,1))
            # loosely exponential increase in weight transformed to [0-1] space
            for index, cv in enumerate(cvs):
                pmc.percent(
                    cluster[0],
                    cv,
                    v=(round(((float(len(cvs)) - index) ** 3) / (len(cvs) ** 3), 4)),
                )

            # for index, cv in enumerate(cvs):
            pmc.parent(grp, clusters)


        # for all 4 curves, create clusters
        for xy in zip([tr, tl, br, bl], ["TR", "TL", "BR", "BL"], [-1, -1, 1, 1]):
            makeSmile(xy[0], xy[1])
            makeOpen(xy[0], xy[1], xy[2])
            makePurse(xy[0], xy[1])

    # core of the rig happens here
    # iterate over curves top/bottom
    # make a new joint at pivot, and at each cv on the curve
    # make 2 spaceLocators to drive its orientation, and its parent aim
    # returns [cornerLocator, cornerCv] why cornerCV?
    def makeJoints(curve, name):
        curvesCvs = pmc.ls(str(curve) + ".cv[*]", flatten=1)
        ret = []
        for index, cv in enumerate(curvesCvs):
            # handle corners
            # this needs to be handled seperately, as we dont have a second axis to consider
            # and only need one locator
            if index == 0 or index == len(curvesCvs) - 1:
                cornerName = (
                    "locAlignPos_" + name + "_" + str(index) + "_l_mouthCorner"
                    if index == 0
                    else "locAlignPos_" + name + "_" + str(index) + "_r_mouthCorner"
                )
                # create spaceLocator
                locAlignPos = pmc.spaceLocator(n=cornerName)
                pmc.hide(locAlignPos)
                # attach to motion path
                animPos = pmc.pathAnimation(locAlignPos, c=curve, f=1)
                # break anim connection
                pmc.disconnectAttr(str(animPos) + ".uValue")
                # move it to where it needs to be
                pmc.setAttr(str(animPos) + ".uValue", index)
                pmc.setAttr(str(locAlignPos) + ".localScale", (0.4, 0.4, 0.4))
                ret.append(locAlignPos)
                ret.append(cv)
                pmc.parent(locAlignPos, locators)
            else:
                # make 2 space locators
                # one will go up the chain, the other will go down for bidirectional aiming/orienting
                locAlignPos = pmc.spaceLocator(
                    n="locAlignPos_" + name + "_" + str(index)
                )
                locAlignNeg = pmc.spaceLocator(
                    n="locAlignNeg_" + name + "_" + str(index)
                )
                pmc.hide(locAlignPos, locAlignNeg)
                # attach to patht
                animPos = pmc.pathAnimation(locAlignPos, c=curve, f=1)
                animNeg = pmc.pathAnimation(locAlignNeg, c=curve, f=0)
                # break anim connection 
                pmc.disconnectAttr(str(animPos) + ".uValue")
                pmc.disconnectAttr(str(animNeg) + ".uValue")
                # move into place
                pmc.setAttr(str(animPos) + ".uValue", index)
                pmc.setAttr(str(animNeg) + ".uValue", index)
                # make sure the orientation of both is identical
                orient = pmc.orientConstraint(locAlignPos, locAlignNeg)
                # then remove
                pmc.delete(orient)
                # aim locAlignNeg at its predecessor
                # potentially reevaluate how this is done, surely we want it to be aimed with negY?
                pmc.aimConstraint(
                    pmc.ls("locAlignPos_*%s*" % str(index - 1))[0],
                    locAlignNeg,
                    weight=1,
                    maintainOffset=1,
                )
                # scale down
                pmc.setAttr(str(locAlignNeg) + ".localScale", (0.1, 0.1, 0.1))
                pmc.setAttr(str(locAlignPos) + ".localScale", (0.1, 0.1, 0.1))
                # make bone at pivot and bone at locator
                jtOri = pmc.joint(
                    p=pmc.pointPosition(pivot),
                    n="jtOri_mouth" + name + "_" + str(index),
                    radius=0.3,
                )
                bindJt = pmc.joint(
                    p=pmc.pointPosition(locAlignPos),
                    n="bind_mouth" + name + "_" + str(index),
                    radius=0.2,
                )
                # allow for scaling of parent to not interfere with scaling of bindJt
                pmc.setAttr(str(bindJt) + ".segmentScaleCompensate", 1)
                pmc.parent(bindJt, cleanup)
                # orient the bind joint between the 2 locators
                orient = pmc.orientConstraint(
                    locAlignNeg, locAlignPos, bindJt, w=1, mo=1
                )
                pmc.setAttr(str(orient) + ".interpType", 2)
                # aim parent joint at the locator on the curve meant to drive if
                pmc.aimConstraint(locAlignPos, jtOri)
                pmc.parent(bindJt, jtOri)
                pmc.parent(jtOri, pivotJoint)
                pmc.parent(locAlignNeg, locAlignPos, locators)

        return ret

    # utility function make a cc based off a vertex and a an orientation joint
    # match oriJt orientation, and move [.3, 0, 0]
    # centerPivot, freezeTransformations
    def makeCC(cc, oriJt):
        pmc.xform(cc, cp=1)
        orient = pmc.orientConstraint(oriJt, cc)
        pmc.delete(orient)
        pmc.makeIdentity(cc, s=1, t=1, apply=1)
        pmc.move(
            cc, [0.3, 0, 0], relative=True, objectSpace=True, worldSpaceDistance=True
        )
        pmc.makeIdentity(cc, s=1, r=1, t=1, apply=1)
        pmc.rotate(cc, [0, 90, 0])
        pmc.makeIdentity(cc, s=1, r=1, t=1, apply=1)

    #utility function that makes a cluster with a cv/cvs
    # and stores it in an offset group to remove double translations
    def clusterAndOffset(cv, name):
        cluster = pmc.cluster(cv, n=name)
        grp = pmc.group(em=1, n="grpClsOffset_" + name)
        pmc.xform(grp, cp=1)
        pmc.makeIdentity(grp, t=1, s=1, r=1)

        pmc.parent(cluster[1], grp)
        return [grp, cluster]

    # for each curve
    # make a cluster at each cv
    # make a cc that drives it, sanitize its rotation
    # parent it to the locator meant to drive it
    # handle mirroring for right side
    def ccGenerator(curve, name):
        curvesCvs = pmc.ls(str(curve) + ".cv[*]", flatten=1)
        # for all cvs in the curve, excludint the corners
        for index, cv in enumerate(curvesCvs[1:-1]):
            # make a cc
            cc = pmc.circle(
                n="anim_cc_mouth" + str(name) + "_" + str(index),
                c=pmc.pointPosition(cv),
                r=0.15,
            )[0]

            # define joint whose orienatation it should match
            oriJt = pmc.ls("jtOri_mouth" + name + "_" + str(index + 1))
            # make cc
            makeCC(cc, oriJt)
            # make cluster it will drive
            grp, cluster = clusterAndOffset(
                cv, "clusterCC_mouth_" + name + "_" + str(index + 1)
            )
            # parent cluster offset group to cluster
            pmc.parent(grp, "clusters_mouth01")
            # connect cc to cluster
            pmc.pointConstraint(cc, grp, mo=1)
            grpOri = pmc.group(em=1, n="grpAlign_mouthDriver_" + str(index))
            # finally, once the cc --> cluster --> curve
            # we need to have driver --> cc
            # select the spaceLocator that will drive the cc
            target = pmc.ls("locAlign_mouthDriver" + name + "_" + str(index + 1))[0]
            if index < len(curvesCvs) / 2:
                # consider refactoring to parent constraint instead of parent, to keep the ccs in hierarchy instead of in locators
                # pmc.parent(cc, pmc.ls("ccs_mouthL01")[0])
                pmc.parent(grpOri, target)
            else:
                # this looks like its going to mirror the ccs, but not 100% sure how
                pmc.scale(grpOri, [0, 0, -1])
                pmc.makeIdentity(s=1, t=1, r=1)
                pmc.parent(grpOri, target)
                # pmc.scale(grpOri, [-1, 0, 0])
                pmc.setAttr(str(grpOri) + ".scaleX", -1)
                pmc.parent(grpOri, pmc.ls(target)[0])
            pmc.parent(cc, grpOri)
            pmc.makeIdentity(cc, s=1, t=1, r=1, apply=1)

    # accept top:[l, r], bottom:[l, r]
    # Corners on the mouth rig need special care, so ive put in a custom function to handle them
    def handleCorners(top, bottom):
       
        # utility that will run for each corner with a new name
        def abstract(corner):
            # naming dictionary
            absObj = (
                {
                    "name": "l",
                    "loc": 0,
                    "cv": 1,
                    "target": "ccs_mouthL01",
                }
                if corner == "l"
                else {
                    "name": "r",
                    "loc": 2,
                    "cv": 3,
                    "target": "ccs_mouthR01",
                }
            )
            name, loc, cv, target, rest = (
                lambda name, loc, cv, target, **rest: (
                    name,
                    loc,
                    cv,
                    target,
                    rest,
                )
            )(**absObj)
            pmc.select(cl=1)
            # make joint at cc[0]
            cornerJoint = pmc.joint(
                n="bind_" + name + "_mouthCorner01",
                p=pmc.pointPosition(top[loc]),
                radius=0.4,
            )
            # orient to match locator
            orientL = pmc.orientConstraint(
                top[loc], bottom[loc], cornerJoint, maintainOffset=1
            )
            pmc.setAttr(str(orientL) + ".interpType", 2)
            pmc.select(clear=1)
            # make parent joint
            jtOriL = pmc.joint(
                p=pmc.pointPosition(pivot), n="jtOri_" + name + "_corner01", radius=0.4
            )
            # aim parent at locators
            pmc.aimConstraint(top[loc], bottom[loc], jtOriL)
            pmc.parent(cornerJoint, jtOriL)

            pmc.parent(jtOriL, pivotJoint)
            # make cc
            ccL = pmc.circle(
                n="anim_" + name + "_mouthCorner01",
                c=pmc.pointPosition(top[loc]),
                r=0.3,
            )[0]
            oriJtL = pmc.ls("jtOri_" + name + "_corner01")
            # for some reason, makeCC didnt work with one of these, so ive had to include it longhand
            # this block does the same, make a cc, orient to joing, move [.3, 0, 0]
            pmc.xform(ccL, cp=1)
            orient = pmc.orientConstraint(oriJtL, ccL)
            pmc.delete(orient)

            pmc.makeIdentity(s=1, t=1, apply=1)
            pmc.delete(ccL, ch=1)
            pmc.move(
                ccL,
                [0.3, 0, 0],
                relative=True,
                objectSpace=True,
                worldSpaceDistance=True,
            )
            pmc.makeIdentity(s=1, r=1, t=1, apply=1)
            pmc.rotate(ccL, [0, 90, 0])
            pmc.makeIdentity(ccL, s=1, r=1, t=1, apply=1)
            # ---------------------------------------------------------------------------------------------------------------------------------------------------
            # make cluster for both the corner actual verts, potentially deprecated? Deleting this may solve out double transformation issue
            grp, cluster = clusterAndOffset(
                [top[cv], bottom[cv]], "cluster_mouth_" + name + "_01"
            )
            pmc.parent(grp, clusters)
            # find locators meant to drive this cluster
            locDrivers = pmc.ls('locAlign_mouthDriver_*_'+name.upper()+'_corner01')
            # attach
            pmc.parentConstraint(locDrivers, cluster)

            # find smile clusters
            smileClusters = pmc.ls("*" + corner.upper() + "_smile*Handle", flatten=1)
            # for each of the 2 smileClusters
            for index, clus in enumerate(smileClusters):
                # if corner == r, mirror 
                if corner == "r":
                    # create a multiply divide node to flip tX
                    mult = pmc.shadingNode(
                        "multiplyDivide",
                        asUtility=True,
                        name="rClusterOffset" + str(index),
                    )
                    pmc.setAttr(str(mult) + ".input1", [-1, 1, 1])
                    pmc.connectAttr(str(ccL) + ".t", str(mult) + ".input2", f=1)
                    pmc.parent(ccL, ccsRight)
                    pmc.makeIdentity(ccL, apply=1, t=1, r=1, s=1)
                    pmc.connectAttr(str(mult) + ".output", str(clus) + ".t", f=1)
                else:
                    pmc.connectAttr(str(ccL) + ".translate", str(clus) + ".translate")
                    pmc.parent(ccL, ccsLeft)

            # pin the cc parent group's pivot to the jaw pivot
            pivotToComponent(jaw, target)

            # orient the cc's parent to the jaw
            # this mimics the corner CC being driven by the opening
            pmc.orientConstraint(pmc.ls("ccs_mouth01"),pmc.ls("locAlign_jaw_openDriver01"), pmc.ls(target), mo=1  )

            # Next we need to bind translate z on the ccs to flatten the bshp curves
            [bottomBshp, topBshp] = pmc.ls("bshpDriver_mouth_*"+ name.upper() + "*01", type = "transform")


            # Set key on anim_corner tz to contorl flatten of bshp
            locTemp = pmc.spaceLocator(n = "temp01")
            pmc.pointConstraint(bottomBshp, locTemp)
            pmc.parent(locTemp, cleanup)

            [cx, cy, cz]  = pmc.xform(ccL, ws=1, q=1, rp=1)
            [lx, ly, lz] = pmc.xform(locTemp, ws=1, q=1, rp=1)
            pmc.delete(locTemp)
            # set Keys to control Purse
            pmc.select(ccL)
            pmc.addAttr( longName='Purse', defaultValue=1.0, minValue=0, maxValue=1, h=0, k=1, r=1, w=1 )
            ccL.attr('Purse').set(0)
            pmc.transformLimits(ccL, tz = (lz, 0), etz =(1,1))

            # Make keys
            pmc.setDrivenKeyframe( str(topBshp) + ".scaleZ",cd = str(ccL) + ".translateZ" )
            pmc.setDrivenKeyframe( str(bottomBshp) + ".scaleZ",cd = str(ccL) + ".translateZ" )
            pmc.move(0, 0, lz-cz, ccL, ws = 1)
            pmc.scale([str(topBshp), str(bottomBshp)],[1,1,0], r=1)
            pmc.setDrivenKeyframe( str(topBshp) + ".scaleZ",cd = str(ccL) + ".translateZ" )
            pmc.setDrivenKeyframe( str(bottomBshp) + ".scaleZ",cd = str(ccL) + ".translateZ" )
            # set interp to linear, and set infinity to linear
            
            
            pmc.select(bottomBshp)
            pmc.selectKey( str(bottomBshp), time=(lz-cz), attribute='scaleZ') 
            pmc.setInfinity(poi="linear")
            pmc.keyTangent(outAngle="3.5")
            pmc.selectKey( str(bottomBshp), time=(0), attribute='scaleZ') 
            pmc.keyTangent(itt="flat")
            pmc.setInfinity(poi="linear")


            pmc.select(topBshp)
            pmc.selectKey( str(topBshp), time=(lz-cz), attribute='scaleZ') 
            pmc.setInfinity(poi="linear")
            pmc.keyTangent(outAngle="3.5")
            pmc.selectKey( str(topBshp), time=(0), attribute='scaleZ') 
            pmc.keyTangent(itt="flat")
            pmc.setInfinity(poi="linear")
            
            pmc.move(0, 0, 0, ccL, ws = 1)
            # set Keys to control corner Inward
            # Make keys
            pmc.setDrivenKeyframe( str(topBshp) + ".scaleX",cd = str(ccL) + ".translateX" )
            pmc.setDrivenKeyframe( str(bottomBshp) + ".scaleX",cd = str(ccL) + ".translateX" )
            pmc.xform(ccL, t=(-cx, 0,0),ws=1)
            pmc.scale([str(topBshp), str(bottomBshp)],[0,1,1], r=1)
            pmc.setDrivenKeyframe( str(topBshp) + ".scaleX",cd = str(ccL) + ".translateX" )
            pmc.setDrivenKeyframe( str(bottomBshp) + ".scaleX",cd = str(ccL) + ".translateX" )
            # set interp to linear, and set infinity to linear
            pmc.select(cl=1)
            pmc.selectKey(str(bottomBshp), attribute = 
            'scaleX',  k=1 )
            pmc.selectKey(str(topBshp), add=1, attribute = 
            'scaleX',  k=1 )
            pmc.keyTangent(itt="linear", ott="linear")
            # pmc.setInfinity(poi="linear")
            # pmc.select(cl=1)
            # pmc.keyTangent(itt="linear", ott="linear")
            # pmc.setInfinity(poi="linear")
            
            pmc.move(0, 0, 0, ccL, ws = 1)

            pmc.select(topBshp, bottomBshp)





        abstract("r")
        abstract("l")


    #handles the opening of the mouth
    #creates a public locator that will be left to tie into the rig
    # this locator drives the  root of the joints
    # and the open mouth cluster
    def bindOpening():
        # the corner ccs currently live on their own
        # they deform the rig, but are not driven by the open cluster
        [x,y,z] = pmc.xform(jaw, ws=1, q=1, t=1)
        # this creates the locator that will be left client facing to integrate into the rig
        publicJaw = pmc.spaceLocator(n = "locAlign_jaw_openDriver01", p=[x,y,z])
        pmc.xform(publicJaw, cp=1)
        openCluster = pmc.ls('clusterDriver_mouth_open01Handle')[0]
        pmc.parent(publicJaw, translate)
        # orient the locator to the cluster
        temp = pmc.orientConstraint(openCluster, publicJaw)
        pmc.delete(temp)
        # center pivots
        pmc.move(
            x,y,z,
            str(openCluster) + ".scalePivot",
            str(openCluster) + ".rotatePivot",
            rpr=1,
            a=1,
        )
        # finally, connect the roation of the locator to the rotation of the cluster
        pmc.connectAttr(str(publicJaw) + '.r' , str(openCluster) + '.r')
        # next bing the root joint of the mouth to follow the opening cluster
        pmc.orientConstraint(jointsOffset, publicJaw, joints)

    def connectZScale(child, parent, target, purse):
        target = pmc.ls(target)[0]

        # dist between and joint and locator
        def abs(joint, locator,name,  subNode = False, parentPmm = False) :
            pMM = pmc.createNode("pointMatrixMult", n = "pmm_" + name + str(locator))
            pmc.connectAttr(str(joint)+".translate", pMM + ".inPoint",)
            pmc.connectAttr(str(joint)+".parentMatrix", pMM + ".inMatrix",)
            distNode = pmc.shadingNode('distanceBetween', au = 1, n = "dist_" + name+ str(locator))
            pmc.connectAttr(str(pMM)+".output", str(distNode) + ".point2", )
            if(subNode == False):
                subNode = pmc.shadingNode('plusMinusAverage', au = 1, n = "sub_" + name+ str(locator))
                pmc.connectAttr(str(distNode)+".distance", str(subNode) + (".input1D[0]" if name == "parent_" else ".input1D[1]"), f=1)
            if(parentPmm == False):
                pmc.connectAttr(str(locator)+".worldPosition[0]", str(distNode) + ".point1")
            else:
                pmc.connectAttr(str(parentPmm) + '.output', str(distNode) + '.point1')

            pmc.setAttr(str(subNode) + ".operation", 2)
            return subNode, pMM
        # connect the parent->target output on 1X
        [subNodePassover, pMM] = abs(parent, target, "parent_")
        abs(child, target,'child_', subNodePassover, pMM)

        [childD, parentD] = pmc.ls("dist*" + str(target))
        divA = pmc.shadingNode('multiplyDivide', au = 1, n = "divA_" + str(target))
        divA.attr('operation').set(2)
        divA.attr("input2X").set(float(childD.getAttr("distance")))
        subNodePassover.attr("input1D[1]").set(float(childD.getAttr("distance")))

        multB = pmc.shadingNode('multiplyDivide', au = 1, n = "multB_" + str(target))
        addA = pmc.shadingNode('plusMinusAverage', au = 1, n = "a_" + str(target))
        pmc.connectAttr(str(subNodePassover)+'.output1D' ,str(divA) + ".input1X")
        pmc.connectAttr(str(divA)+'.outputX' ,str(multB) + ".input1X")
        pmc.connectAttr(str(purse)+'.Purse' ,str(multB) + ".input2X")
        pmc.connectAttr(str(multB) + '.outputX', str(addA)+ ".input1D[0]")
        addA.attr('input1D[1]').set(1)
        pmc.delete(childD)

        pmc.connectAttr( str(addA) + ".output1D" ,str(parent)+".scaleX")
        
    
    # Alpha, this runs the calculations to figure out how far a parent joint needs to scale to be at the same point as a cv
    # This will have to be strung into a node chain to dynamically scale down the road 
    def handleZ():
        bindJts = pmc.ls("bind*_mouth*", flatten=1, type="joint")
        # bindHighest = sorted(
        #     bindJts, reverse=1, key=lambda x: pmc.xform(x, ws=1, q=1, t=1)[2]
        # )
        for index, joint in enumerate(bindJts):
            # locAlign_mouthDriver_top_1
            # locAlign_mouthDriver_top_L_corner01
            target = ''
            if(index == 0 or index == len(bindJts)-1):
                side = re.findall(r"bind_(\w)_mouthCorner", str(joint))[0]
                target = "locAlign_mouthDriver_top_"+side.upper()+"_corner01"
            else:
                target = "locAlign_mouthDriver_"+ re.findall(r'bind_mouth_(\w+_\d+)', str(joint))[0]

            pmc.select(joint)
            parent = pmc.pickWalk(direction="up")[0]
            # anim_l_mouthCorner01
            # anim_r_mouthCorner01
            [lx, ly, lz] = pmc.xform(joint, q=1,t=1,  ws=1)
            purse = 'anim_r_mouthCorner01' if lx >= 0 else "anim_l_mouthCorner01"
            connectZScale(joint, parent, target, purse)
            



    makeDrivers(top, bottom)
    makeShapeClusters()

    topCorners = makeJoints(top, "_top")
    bottomCorners = makeJoints(bottom, "_bottom")

    ccGenerator(top, "_top")
    ccGenerator(bottom, "_bottom")

    bindOpening()
    handleCorners(topCorners, bottomCorners)

    handleZ()


    print("Mouth Complete")
    



Mouth()
# Grab Currrent Time After Running the Code
end = time.time()

#Subtract Start Time from The End Time
total_time = end - start
print("\n"+ str(total_time) + "total run time")
