from maya import cmds


cmds.file(new=True, f=True)

if cmds.pluginInfo("parallelFrameTransportNode", q=True, loaded=True):
    cmds.unloadPlugin("parallelFrameTransportNode")
cmds.loadPlugin("parallelFrameTransportNode")


pft = cmds.createNode("parallelFrameTransport", n="pft")
curve = cmds.curve(p=[[0, 0, 0], [0, 2, 0], [0, 4, 0], [0, 6, 0]], d=3)
start_matrix = cmds.createNode("transform", n="startMatrix")

# The pft node needs an input curve and a start matrix to determine the
# initial tangent and normal vector
#
cmds.connectAttr(
    "{0}.worldSpace[0]".format(curve), "{0}.inputCurve".format(pft))
cmds.connectAttr(
    "{0}.worldMatrix[0]".format(start_matrix), "{0}.startMatrix".format(pft))

# Create a number of joints and distribute them along the curve via pft node
#
num_joints = 10
skin_jnts = []
for i in range(num_joints):
    cmds.setAttr("pft.params[{0}]".format(i), i / float(num_joints - 1))
    cmds.select(d=True)
    joint = cmds.joint(p=[0, 0, 0])
    cmds.setAttr("{0}.displayLocalAxis".format(joint), 1)
    cmds.connectAttr(
        "{0}.outTranslate[{1}]".format(pft, i), "{0}.translate".format(joint))
    cmds.connectAttr(
        "{0}.outRotate[{1}]".format(pft, i), "{0}.rotate".format(joint))
    cmds.connectAttr(
        "{0}.outScale[{1}]".format(pft, i), "{0}.scaleX".format(joint))
    cmds.connectAttr(
        "{0}.outScale[{1}]".format(pft, i), "{0}.scaleZ".format(joint))
    if i > 0 and i < num_joints - 1:
        skin_jnts.append(joint)

# Create demo rig
#
start_ctl = cmds.circle(n="Start_CTL", nr=[0, 1, 0])
start_jnt = cmds.joint(n="Start_JNT")
end_ctl = cmds.circle(n="End_CTL", nr=[0, 1, 0])
end_jnt = cmds.joint(n="End_JNT")
cmds.setAttr("{0}.ty".format(end_ctl[0]), 6)
cmds.setAttr("{0}.rotateOrder".format(end_ctl[0]), 1)
cmds.makeIdentity(end_ctl, apply=True, t=True)

# Skin the curve to the two control joints that also act as skin joints for the geo
#
curve_skin = cmds.skinCluster(
    [start_jnt, end_jnt], curve, n='curve_SKN', tsb=True)[0]

# Twist setup, controlled by the start and end controller Y rotation
#
cmds.setAttr("{0}.twist[1].twist_Position".format(pft), 1)
cmds.connectAttr("{0}.rotateY".format(start_ctl[0]),
                 "{0}.twist[0].twist_FloatValue".format(pft))
cmds.connectAttr("{0}.rotateY".format(end_ctl[0]),
                 "{0}.twist[1].twist_FloatValue".format(pft))

# Volume Preservation
#
cmds.setAttr("{0}.restLength".format(pft), 6) # hardcoded length of the curve
cmds.setAttr("{0}.scaleRamp[0].scaleRamp_FloatValue".format(pft), 0)
cmds.setAttr("{0}.scaleRamp[1].scaleRamp_Position".format(pft), 0.5)
cmds.setAttr("{0}.scaleRamp[2].scaleRamp_Position".format(pft), 1)
cmds.setAttr("{0}.scaleRamp[1].scaleRamp_FloatValue".format(pft), 2)

# Create and skin test geo
#
cube = cmds.polyCube(h=6, sy=num_joints)
cmds.setAttr("{0}.ty".format(cube[0]), 3)
cmds.skinCluster(
    [start_jnt] + skin_jnts + [end_jnt], cube, tsb=True)[0]
