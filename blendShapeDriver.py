# from asyncore import loop
# from gettext import translation
# from select import select

# from numpy import outer
import ast
import pymel.core as pm
from maya import cmds as mc
import re
from collections import Counter
import operator

def exTract(name):
    return int(re.search(r'\[(\d+)\]', str(name)).group(1))
    

def tSub(x, y):
    return(x[0]- y[0], x[1] - y[1], x[2] - y[2])

def tRound(t):
    return[round(t[0], 4), round(t[1], 4), round(t[2], 4)]

def rindex(lst, value):
    return len(lst) - operator.indexOf(reversed(lst), value) - 1

# function expects vert/edge checking if it is a range or a single edge
# in : pCylinder1.e[140] or pCylinder1.e[140:141]
# out: ["pCylinder1.e[140]"] or ['pCylinder1.e[140]', 'pCylinder1.e[141]']
def pointRange(inRange):
    ret = []
    for i in inRange:
        temp = []
        if ":" not in i:
            ret.append(i)
            continue
        # print(re.search(r'\[(\d+):(\d+)\]', str(i)).group(1))
        retRange = ([int(re.search(r'\[(\d+):(\d+)\]', str(i)).group(1)), int(re.search(r'\[(\d+):(\d+)\]', str(i)).group(2))])
        name = re.search(r'(.+)\[', i).group(1)

        retRange = list(retRange)
        for i in range(retRange[0], retRange[1] + 1):
            # print(str(name) + ".vtx[" + str(i) + "]")
            temp.append(str(name)  + "[" + str(i) + "]")
        ret = ret + temp
    return ret

def uniqueVerts(driver, driven):
    driver = re.sub("\|", "", str(driver))
    driven = re.sub("\|", "", str(driven))
    globalOffset = mc.getAttr(str(driver) + ".translate")[0]
    driverVerts  = mc.ls(driver+ '.vtx[*]', fl=1)
    drivenVerts  = mc.ls(driven+ '.vtx[*]', fl=1)


    def customFilter(i):
        operationDriver = driverVerts[i]
        operationDriven = drivenVerts[i]
        drivenVertPosition = mc.pointPosition(operationDriven)
        driverVertPosition = mc.pointPosition(operationDriver)
        result =  list(tSub(driverVertPosition, globalOffset))
        return(str(tRound(result)) != str(tRound(drivenVertPosition)))

    uniqueVerts = list(filter(customFilter, range(len(drivenVerts))))
    uniqueVerts = (map(lambda x : driven + '.vtx[%s]' %x,uniqueVerts))

    # mc.select(uniqueVerts)
    return(uniqueVerts)

def detectBone(verts):
    ret = []
    for obj in verts:
    #     objHist = mc.listHistory(obj, pdo=True)
    #     skinCluster = mc.ls(objHist, type="skinCluster") or [None]
    #     cluster = skinCluster[0]
    #     # print( obj, cluster)
    #     clusterHist = mc.listHistory(cluster, pdo=True)
    #     clusterParts = mc.listHistory(clusterHist, pdo=True)
    #     print(clusterParts)
        pm.select(obj)
        print("break")
        vert = pm.selected()[0]
        mesh = vert.node()
        sc = mesh.listHistory(type='skinCluster')[0] # Note this is not a great way to get skinClusters in general, but it works in simple situations.
        infs = sc.influenceObjects()
        vId = vert.currentItemIndex()
        top = sorted(infs,reverse=True, key = lambda x : pm.getAttr(sc.weightList[vId].weights[infs.index(x)]))
        # for jId in xrange(sc.numInfluenceObjects()):
            # print('\t', infs[jId], pm.getAttr(sc.weightList[vId].weights[jId]))
        ret.append(top[0])
    print(Counter(ret))

def vertsToLoops(verts):
    vertList = verts
    blacklist = []
    loops = []
    tally = []

    counter = 0
    # Inital iterator guessing which is the optimal edge loop orientation
    for v in vertList:
        if v in blacklist:
            continue
        cross = pointRange(mc.polyListComponentConversion(v, fv=1, te=1))
        
        mc.select(v)
        # print(cross)
        [e1, e2, e3, e4] = cross
        mc.polySelect(e1, el=(exTract(e1)))
        cross1 = mc.ls(sl=1)

        mc.polySelect(e3, el=(exTract(e3)))
        cross2 = mc.ls(sl=1)

        cross1Verts = pointRange(mc.polyListComponentConversion(cross1, fe=1, tv=1))
        cross2Verts = pointRange(mc.polyListComponentConversion(cross2, fe=1, tv=1))

        cross1Filtered = filter(lambda x : x in vertList,cross1Verts)
        cross2Filtered = filter(lambda x : x in vertList,cross2Verts)

        # This proximity calculator needs to be updated to a percentile
        # Number of verts may not always be enough
    #     # instead do totalVerts/effectedVerts
        c1Perc = float(len(cross1Filtered) / float(len(cross1Verts))) * 100
        c2Perc = float(len(cross2Filtered) / float(len(cross2Verts))) * 100

        selectedVerts = cross1Filtered if c1Perc > c2Perc else cross2Filtered
        
        counter += 1
        blacklist = blacklist + selectedVerts
        loops.append({
            "vertex" : v,
            "cross1" : cross1Filtered,
            "cross2" : cross2Filtered,
            "selected" : 'cross1' if c1Perc > c2Perc else 'cross2'
        })
        tally.append('cross1' if c1Perc > c2Perc else 'cross2')

  
    # find most common orientation
    c = Counter(tally)
    common, count = c.most_common()[0]
    # seperate arrays of loops that conform/not to common orientation
    loops = filter(lambda x : x['selected'] == common, loops)
    outliers = filter(lambda x : x['selected'] != common, loops)

    # iterate over the outliers, correct their orientation with the most common
    outliers = map(lambda x : str(x[common]), outliers)
    # ensure that only unique vert lists are present
    outliersUnique = list(set(outliers))
    outliersUnique = map(lambda x : ast.literal_eval(x), outliersUnique)
    # prepare to stack vert loop ranges for return
    loops = map(lambda x : x[common], loops)
    loops = loops + outliersUnique

    
    return loops
        
def sortLoops(loops):
    # initial sort to determine which is the vertex highest in the poleDirection
    ret = sorted(loops, reverse = True, key = lambda x : mc.pointPosition(x[0])[pD])

    # store highest vert number
    point = exTract(ret[0][0])

    # resort list to see which vert number is closest to point var
    ret = sorted(ret, key = lambda x : point - exTract(x[0]))
    

    
    # for r in ret:    
    #     mc.select(r)
    #     mc.refresh()
        
    #     result = mc.promptDialog(
    #     title='Rename Object',
    #     message='Enter Name:',
    #     button=['OK', 'Cancel'],
    #     defaultButton='OK',
    #     cancelButton='Cancel',
    #     dismissString='Cancel')

    #     if result == 'OK':
    #         continue
    #     else:
    #         break

    return ret
    
def clusterPairs(verts):
    # iterate over vert ranges
    ret = []
    for vertLoop in verts:
        # for each, make a driver/driven cluster
        # driver
        driverLoop = ast.literal_eval(re.sub(str(driven), str(driver), str(vertLoop)))
        
        # blend = mc.blendShape(driverLoop, vertLoop)
        # mc.select(blend)
        # ret.append(blend[0])
        # mc.select(blend)

    # print(ret)
        


# -------------------------------------------------------------------------------------------
# [driver, driven] = mc.ls(sl=True,long=True)
pD = 1

[driver, driven] = mc.ls('pCylinder2', 'pCylinder1')
# print(driver, d)


#Returns array of all verts displaced by deformation
targetVerts = uniqueVerts(driver, driven)

# determins which is the driver bone
driverBone = detectBone(targetVerts)

# # returns array of arrays containing oriented edge loops
# vertLoops = vertsToLoops(targetVerts)
# mc.select(cl=1)

# # Sort array using some up criteria
# sortedLoops = sortLoops(vertLoops)
# # print(sortedLoops)

# # Taking the sorted loops, bind them to their driver loop using blendShapes
# blendShapes = clusterPairs(sortedLoops)










