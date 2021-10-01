class MotionPlanner:
  def __init__ (self, robot, ps):
    self.robot = robot
    self.ps = ps

  def solveBiRRT (self, maxIter = float("inf")):
    self.ps.prepareSolveStepByStep ()
    finished = False

    # In the framework of the course, we restrict ourselves to 2 connected components.
    nbCC = self.ps.numberConnectedComponents ()
    if nbCC != 2:
      raise Exception ("There should be 2 connected components.")

    iter = 0
    while True:
      #### RRT begin      
      q_rand=self.robot.shootRandomConfig()
      for index in range(nbCC):
        q_near,_=self.ps.getNearestConfig (q_rand, connectedComponentId=index)
        _,i,_= self.ps.directPath(q_near,q_rand,True)
        length = self.ps.pathLength(i) 	
        config = self.ps.configAtParam(i,length) 
        self.ps.addConfigToRoadmap(config) 
        self.ps.addEdgeToRoadmap(q_near,config,i,True)	

      ## Try connecting the new nodes together
      ##for i in range (len(newConfigs)):
      ##  pass
      #### RRT end
      ## Check if the problem is solved.
      nbCC = self.ps.numberConnectedComponents ()
      if nbCC == 1:
        # Problem solved
        finished = True
        break
      iter = iter + 1
      if iter > maxIter:
        break
    if finished:
        self.ps.finishSolveStepByStep ()
        return self.ps.numberPaths () - 1

  def solvePRM (self):
    self.ps.prepareSolveStepByStep ()
    #### PRM begin
    #### PRM end
    self.ps.finishSolveStepByStep ()
