from maya import cmds
import maya.api.OpenMaya as om
import operator
import pymel.core as pmc

def getClosestVertex(mesh, pos=(0,0,0)):
    """Return closest vertex and distance from mesh to world-space position [x, y, z].
        
    Uses om.MfnMesh.getClosestPoint() returned face ID and iterates through face's vertices.
    
    Example:
        >>> getClosestVertex("pCube1", pos=[0.5, 0.5, 0.5])
        # (3, 0.0)
        >>> getClosestVertex("pCube1", pos=[0.5, 0.9, 0.5])
        # (3, 0.4)

    Args:
        mesh (str): Mesh node name.
        pos (list): Position vector XYZ
        
    Returns:
        tuple: (vertex index, distance)
    
    """
    
    print("eh")
    pos = om.MPoint(pos)
    sel = om.MSelectionList()
    sel.add(mesh)

    fn_mesh = om.MFnMesh(sel.getDagPath(0))
    
    index = fn_mesh.getClosestPoint(pos, space=om.MSpace.kWorld)[1]  # closest polygon index    
    face_vertices = fn_mesh.getPolygonVertices(index)  # get polygon vertices
    
    vertex_distances = ((vertex, fn_mesh.getPoint(vertex, om.MSpace.kWorld).distanceTo(pos)) 
                         for vertex in face_vertices)
    return min(vertex_distances, key=operator.itemgetter(1))

x, y, z = pmc.xform('locator1', q=1, t=1)
print(x, y, z)
vert = getClosestVertex("pSphere1", pos=[x, y, z])
print(vert)

pmc.select('pSphere1.vtx[' + str(vert[0]) + ']')
