from os import stat
import pymel.core as pmc
import re
import math


def xtractNumber(str):
    return int(re.findall("\[(\d+)\]", str)[0])


def euclidean_distance(x, y):
    return math.sqrt(sum((px - py) ** 2 for px, py in zip(x, y)))


def Mouth():
    try:
        pmc.delete("cleanup")
    except:
        print("No cleanup pal")
    cleanup = pmc.group(empty=True, name="cleanup")

    # hidden/static
    static = pmc.group(empty=True, name="grpStatic_mouthRig01")
    locators = pmc.group(empty=True, name="locators_mouth01")
    clusters = pmc.group(empty=True, name="clusters_mouth01")

    # moveable
    translate = pmc.group(empty=True, name="grpTranslate_mouthRig01")

    joints = pmc.group(empty=True, name="joints_mouth01")

    ccs = pmc.group(empty=True, name="ccs_mouth01")
    ccsLeft = pmc.group(empty=True, name="ccs_mouthL01")
    ccsRight = pmc.group(empty=True, name="ccs_mouthR01")

    pmc.scale(ccsRight, [1, 1, -1])
    pmc.makeIdentity(s=1, apply=1)
    pmc.scale(ccsRight, [-1, 1, 1])

    pmc.hide(clusters)

    pmc.parent([translate, static], cleanup)
    pmc.parent([locators, clusters], static)
    pmc.parent([ccsLeft, ccsRight], ccs)
    pmc.parent([ccs, joints], translate)
    # grab ccs by name
    top, bottom, pivot = pmc.ls("driver_top", "driver_bottom", "driver_pivot")
    pmc.select(cl=1)
    pivotJoint = pmc.joint(
        n="jtAlign_mouthPivot_01", p=pmc.pointPosition(pivot), radius=0.5
    )
    pmc.parent(pivotJoint, joints)

    # generate driver curves that deform ccs and main curve
    # returns the driver clusters to be bound later
    def makeDrivers(top, bottom):
        driverLocs = pmc.group(em=1, n="grpLoc_driverCurve_mouth01")
        pmc.parent(driverLocs, locators)
        # makes duplicate driver curve
        # parents curve into translate
        # it over drivre curve, create space locator, attach to motion path,
        for index, curve in enumerate([top, bottom]):
            driver = pmc.duplicate(
                curve, n="ccDriver_mouth" + ("_top01" if index == 0 else "_bottom01")
            )
            pmc.hide(driver)
            pmc.parent(driver, translate)
            curvesCvs = pmc.ls(str(curve) + ".cv[*]", flatten=1)
            for idx, cvs in enumerate(curvesCvs[1:-1]):
                name = "top_" if index == 0 else "bottom_"
                locAlign = pmc.spaceLocator(n="locAlign_mouthDriver_" + name + str(idx))
                pmc.setAttr(str(locAlign) + "Shape.visibility", 0)
                animPos = pmc.pathAnimation(locAlign, c=driver[0], f=1)
                pmc.disconnectAttr(str(animPos) + ".uValue")
                pmc.setAttr(str(animPos) + ".uValue", idx + 1)
                pmc.setAttr(str(locAlign) + ".localScale", (0.4, 0.4, 0.4))
                pmc.parent(locAlign, locators)

        # select all ccs from top and bottom driver curves
        topCCs = pmc.ls("ccDriver_mouth_top01.cv[*]", flatten=1)
        bottomCCs = pmc.ls("ccDriver_mouth_bottom01.cv[*]", flatten=1)

        topLeft = filter(
            lambda x: int(re.findall("\[(\d+)\]", str(x))[0]) < int(len(topCCs) / 2),
            topCCs,
        )
        bottomLeft = filter(
            lambda x: int(re.findall("\[(\d+)\]", str(x))[0]) < int(len(bottomCCs) / 2),
            bottomCCs,
        )

        topRight = filter(
            lambda x: int(re.findall("\[(\d+)\]", str(x))[0]) > int(len(topCCs) / 2),
            topCCs,
        )
        bottomRight = filter(
            lambda x: int(re.findall("\[(\d+)\]", str(x))[0]) > int(len(bottomCCs) / 2),
            bottomCCs,
        )

        # leftCluster = pmc.cluster(topLeft, bottomLeft, n="clsDriver_mouthCorner_l_01")
        # rightCluster = pmc.cluster(
        #     topRight, bottomRight, n="clsDriver_mouthCorner_r_01"
        # )

        grpL, clusterL = clusterAndOffset(
            [topLeft, bottomLeft], "clsDriver_mouthCorner_l_01"
        )
        grpR, clusterR = clusterAndOffset(
            [topRight, bottomRight], "clsDriver_mouthCorner_r_01"
        )
        pmc.parent(grpR, grpL, "clusters_mouth01")

        def falloff(curve, cluster):
            for index, cv in enumerate(curve):
                number = float(re.findall("\[(\d+)\]", str(cv))[0])
                number = abs(len(curve) - number)
                pmc.percent(cluster[0], cv, v=((number**2) / float(len(curve) ** 2)))

        falloff(topLeft, clusterL)
        falloff(topRight, clusterR)
        falloff(bottomLeft, clusterL)
        falloff(bottomRight, clusterR)
        return [clusterL, clusterR]

    # it over curves
    # make a new joint
    # return cornerLocators : []
    def makeJoints(curve, name):
        curvesCvs = pmc.ls(str(curve) + ".cv[*]", flatten=1)
        ret = []
        for index, cv in enumerate(curvesCvs):
            if index == 0 or index == len(curvesCvs) - 1:
                cornerName = (
                    "locAlignPos_" + name + "_" + str(index) + "_l_mouthCorner"
                    if index == 0
                    else "locAlignPos_" + name + "_" + str(index) + "_r_mouthCorner"
                )
                locAlignPos = pmc.spaceLocator(n=cornerName)
                pmc.hide(locAlignPos)
                animPos = pmc.pathAnimation(locAlignPos, c=curve, f=1)
                pmc.disconnectAttr(str(animPos) + ".uValue")
                pmc.setAttr(str(animPos) + ".uValue", index)
                pmc.setAttr(str(locAlignPos) + ".localScale", (0.4, 0.4, 0.4))
                ret.append(locAlignPos)
                ret.append(cv)
                pmc.parent(locAlignPos, locators)
            else:
                locAlignPos = pmc.spaceLocator(
                    n="locAlignPos_" + name + "_" + str(index)
                )
                locAlignNeg = pmc.spaceLocator(
                    n="locAlignNeg_" + name + "_" + str(index)
                )
                pmc.hide(locAlignPos, locAlignNeg)
                animPos = pmc.pathAnimation(locAlignPos, c=curve, f=1)
                animNeg = pmc.pathAnimation(locAlignNeg, c=curve, f=0)
                pmc.disconnectAttr(str(animPos) + ".uValue")
                pmc.disconnectAttr(str(animNeg) + ".uValue")
                pmc.setAttr(str(animPos) + ".uValue", index)
                pmc.setAttr(str(animNeg) + ".uValue", index)
                orient = pmc.orientConstraint(locAlignPos, locAlignNeg)
                pmc.delete(orient)
                pmc.aimConstraint(
                    pmc.ls("locAlignPos_*%s*" % str(index - 1))[0],
                    locAlignNeg,
                    weight=1,
                    maintainOffset=1,
                )
                pmc.setAttr(str(locAlignNeg) + ".localScale", (0.1, 0.1, 0.1))
                pmc.setAttr(str(locAlignPos) + ".localScale", (0.1, 0.1, 0.1))
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
                pmc.setAttr(str(bindJt) + ".segmentScaleCompensate", 1)
                pmc.parent(bindJt, cleanup)
                orient = pmc.orientConstraint(
                    locAlignNeg, locAlignPos, bindJt, w=1, mo=1
                )
                pmc.setAttr(str(orient) + ".interpType", 2)
                pmc.aimConstraint(locAlignPos, jtOri)
                pmc.parent(bindJt, jtOri)
                pmc.parent(jtOri, joints)
                pmc.parent(locAlignNeg, locAlignPos, locators)

        return ret

    # make a cc based off a vertex and a an orientation joint
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

    # function to iterate over a curve
    # excluding corners
    # and generate a cc for each cv
    def clusterAndOffset(cv, name):
        cluster = pmc.cluster(cv, n=name)
        grp = pmc.group(em=1, n="grpClsOffset_" + name)
        pmc.xform(grp, cp=1)
        pmc.makeIdentity(grp, t=1, s=1, r=1)

        pmc.parent(cluster[1], grp)
        return [grp, cluster]

    # create a cluster
    # bind that cluster to the cc
    def ccGenerator(curve, name):
        curvesCvs = pmc.ls(str(curve) + ".cv[*]", flatten=1)
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
            target = pmc.ls("locAlign_mouthDriver" + name + "_" + str(index))[0]

            if index < len(curvesCvs) / 2:
                # pmc.parent(cc, pmc.ls("ccs_mouthL01")[0])
                pmc.parent(grpOri, target)

            else:
                pmc.scale(grpOri, [0, 0, -1])
                pmc.makeIdentity(s=1, t=1, r=1)
                pmc.parent(grpOri, target)
                # pmc.scale(grpOri, [-1, 0, 0])
                pmc.setAttr(str(grpOri) + ".scaleX", -1)
                pmc.parent(grpOri, pmc.ls(target)[0])
            pmc.parent(cc, grpOri)
            pmc.makeIdentity(cc, s=1, t=1, r=1, apply=1)

    # accept top:[l, r], bottom:[l, r]
    # makes bind/jtOri in each corner
    def handleCorners(top, bottom, leftCluster, rightCluster):
        def abstract(corner):
            absObj = (
                {
                    "name": "l",
                    "loc": 0,
                    "cv": 1,
                    "target": "ccs_mouthL01",
                    "driverCluster": leftCluster,
                }
                if corner == "l"
                else {
                    "name": "r",
                    "loc": 2,
                    "cv": 3,
                    "target": "ccs_mouthR01",
                    "driverCluster": rightCluster,
                }
            )
            name, loc, cv, target, driverCluster, rest = (
                lambda name, loc, cv, target, driverCluster, **rest: (
                    name,
                    loc,
                    cv,
                    target,
                    driverCluster,
                    rest,
                )
            )(**absObj)
            pmc.select(cl=1)

            cornerJoint = pmc.joint(
                n="bind_" + name + "_mouthCorner01",
                p=pmc.pointPosition(top[loc]),
                radius=0.4,
            )
            orientL = pmc.orientConstraint(
                top[loc], bottom[loc], cornerJoint, maintainOffset=1
            )
            pmc.setAttr(str(orientL) + ".interpType", 2)

            pmc.select(clear=1)
            jtOriL = pmc.joint(
                p=pmc.pointPosition(pivot), n="jtOri_" + name + "_corner01", radius=0.4
            )

            pmc.aimConstraint(top[loc], bottom[loc], jtOriL)
            pmc.parent(cornerJoint, jtOriL)

            pmc.parent(jtOriL, joints)

            ccL = pmc.circle(
                n="anim_" + name + "_mouthCorner01",
                c=pmc.pointPosition(top[loc]),
                r=0.3,
            )[0]

            oriJtL = pmc.ls("jtOri_" + name + "_corner01")

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

            pmc.parent(ccL, target)
            pmc.makeIdentity(ccL, s=1, r=1, t=1, apply=1)

            grp, cluster = clusterAndOffset(
                [top[cv], bottom[cv]], "cluster_mouth_" + name + "_01"
            )

            pmc.parent(grp, "clusters_mouth01")
            pmc.pointConstraint(ccL, grp, mo=1)
            pmc.connectAttr(str(ccL) + ".t", str(driverCluster[1]) + ".t", f=1)
            # if right side we need to mult x * -1
            if corner == "r":
                mult = pmc.shadingNode(
                    "multiplyDivide", asUtility=True, name="multiplyDivideTest"
                )
                pmc.setAttr(str(mult) + ".input1", [-1, 1, 1])
                pmc.connectAttr(str(ccL) + ".t", str(mult) + ".input2", f=1)
                pmc.connectAttr(
                    str(mult) + ".output", str(driverCluster[1]) + ".t", f=1
                )

        abstract("r")
        abstract("l")

    def handleZ():
        bindJts = pmc.ls("bind*_mouth*", flatten=1, type="joint")
        bindHighest = sorted(
            bindJts, reverse=1, key=lambda x: pmc.xform(x, ws=1, q=1, t=1)[2]
        )
        tZ = pmc.xform(bindHighest[0], ws=1, q=1, t=1)[2]
        for index, joint in enumerate(bindJts[-1:]):
            tX = pmc.xform(joint, ws=1, q=1, t=1)[1]
            parent = []
            pmc.select(joint)
            parent = pmc.pickWalk(direction="up")
            [pX, pY, pZ] = pmc.xform(parent[0], ws=1, q=1, t=1)
            [cX, cY, cZ] = pmc.xform(joint, ws=1, q=1, t=1)
            parent = [round(pX, 5), round(pZ, 5)]
            child = [round(cX, 5), round(cZ, 5)]
            target = [round(cX, 5), round(tZ, 5)]
            cP = euclidean_distance(child, parent)
            tP = euclidean_distance(target, parent)
            ratio = (1 / cP) * tP
            print(ratio)
            # distB = math.dist(target, [])

    # TODO:
    # Handle z translation mouthwide/individual
    # axis agnostic

    leftCluster, rightCluster = makeDrivers(top, bottom)

    topCorners = makeJoints(top, "_top")
    bottomCorners = makeJoints(bottom, "_bottom")

    ccGenerator(top, "_top")
    ccGenerator(bottom, "_bottom")

    handleCorners(topCorners, bottomCorners, leftCluster, rightCluster)

    handleZ()

    # pmc.pointConstraint("anim_l_mouthCorner01", leftCluster, mo=1)
    # pmc.pointConstraint("anim_r_mouthCorner01", rightCluster, mo=1)
    print("Mouth Complete")


Mouth()
