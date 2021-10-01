from math import sqrt
from hpp import Transform
from hpp.corbaserver.manipulation import ConstraintGraph,Constraints
from hpp.corbaserver import Client
Client ().problem.resetProblem ()
from manipulation import robot, vf, ps, Ground, Box, Pokeball, PathPlayer, gripperName, ballName

vf.loadEnvironmentModel (Ground, 'ground')
vf.loadEnvironmentModel (Box, 'box')
vf.moveObstacle ('box/base_link_0', [0.3+0.04, 0, 0.04, 0, 0, 0, 1])
vf.moveObstacle ('box/base_link_1', [0.3-0.04, 0, 0.04, 0, 0, 0, 1])
vf.moveObstacle ('box/base_link_2', [0.3, 0.04, 0.04, 0, 0, 0, 1])
vf.moveObstacle ('box/base_link_3', [0.3, -0.04, 0.04, 0, 0, 0, 1])

vf.loadObjectModel (Pokeball, 'pokeball')
robot.setJointBounds ('pokeball/root_joint', [-.4,.4,-.4,.4,-.1,1.,
                                              -1.0001, 1.0001,-1.0001, 1.0001,
                                              -1.0001, 1.0001,-1.0001, 1.0001,])


q1 = [0, -1.57, 1.57, 0, 0, 0, .3, 0, 0.025, 0, 0, 0, 1]

## Create graph
graph = ConstraintGraph (robot, 'graph')

## Create constraint of relative position of the ball in the gripper when ball
## is grasped
ballInGripper = [0, .137, 0, 0.5, 0.5, -0.5, 0.5]
ps.createTransformationConstraint ('grasp', gripperName, ballName,
                                   ballInGripper, 6*[True,])
## Create nodes and edges
#  Warning the order of the nodes is important. When checking in which node
#  a configuration lies, node constraints will be checked in the order of node
#  creation.

graph.createNode (['grasp_placement', 'gripper_above_ball','ball_above_ground','placement' ,'grasp' ])

graph.createEdge ('grasp_placement', 'gripper_above_ball', 'move_gripper_up', 1, 'gripper_above_ball')
graph.createEdge ('gripper_above_ball', 'grasp_placement', 'grasp_ball', 1, 'grasp_placement')

graph.createEdge ('grasp_placement', 'ball_above_ground', 'take_ball_up', 1, 'grasp_placement')
graph.createEdge ('ball_above_ground', 'grasp_placement', 'pull_ball_down', 1, 'ball_above_ground')

graph.createEdge ('ball_above_ground', 'grasp', 'take_ball_away', 1, 'grasp')
graph.createEdge ('grasp', 'ball_above_ground', 'approach_ground', 1, 'ball_above_ground')

graph.createEdge ('gripper_above_ball', 'placement', 'move_gripper_away', 1, 'placement')
graph.createEdge ('placement', 'gripper_above_ball','approach_ball' , 1, 'gripper_above_ball')

graph.createEdge ('placement', 'placement', 'transit', 1, 'placement')
graph.createEdge ('grasp', 'grasp', 'transfer', 1, 'grasp')


## Create transformation constraint : ball is in horizontal plane with free
## rotation around z
ps.createTransformationConstraint ('placement', '', ballName,
                                   [0,0,0.025,0, 0, 0, 1],
                                   [False, False, True, True, True, False,])
#  Create complement constraint
ps.createTransformationConstraint ('placement/complement', '', ballName,
                                   [0,0,0.025,0, 0, 0, 1],
                                   [True, True, False, False, False, True,])

ps.setConstantRightHandSide ('placement', True)
ps.setConstantRightHandSide ('placement/complement', False)



## Set constraints of nodes and edges
graph.addConstraints (node='placement', constraints = \
                      Constraints (numConstraints = ['placement'],))
graph.addConstraints (node='grasp',
                      constraints = Constraints (numConstraints = ['grasp']))
graph.addConstraints (edge='transit', constraints = \
                      Constraints (numConstraints = ['placement/complement']))
graph.addConstraints (edge='grasp_ball', constraints = \
                      Constraints (numConstraints = ['placement/complement']))


# These edges are in node 'grasp'
graph.addConstraints (edge='transfer',     constraints = Constraints ())
graph.addConstraints (edge='release-ball', constraints = Constraints ())


ps.selectPathValidation ("Discretized", 0.01)
ps.selectPathProjector ("Progressive", 0.1)
graph.initialize ()

res, q_init, error = graph.applyNodeConstraints ('placement', q1)
q2 = q1 [::]
q2 [7] = .2

res, q_goal, error = graph.applyNodeConstraints ('placement', q2)

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

# v = vf.createViewer ()
# pp = PathPlayer (v)
# v (q1)

