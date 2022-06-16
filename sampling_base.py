import random 
import math 
import pygame


class SamplingMap:
  # Class to draw tbe map environment, where our robot will move. Includes 
  # functions that draw the path of the robot and generate the obstacles.

  def __init__(self, start, goal, mapDim, obsDim, obsNum):
    self.start=start  # start config
    self.goal=goal  # goal config
    self.mapDim=mapDim   # dimensions of map grid
    self.m_h, self.m_w = self.mapDim   #height and width 

    # window settings
    self.mapWindowName='RRT implementation'
    pygame.display.set_caption(self.mapWindowName)
    self.map=pygame.display.set_mode((self.m_w, self.m_h))
    self.map.fill((255,255,255))     # setting background of map to white.
    self.nodeRadius=2    
    self.nodeThickness=0
    self.edgeThickness=1

    # obstacle settings
    self.obstacles=[]
    self.obsDim=obsDim
    self.obsNum=obsNum

    # Colors
    self.grey = (70,70,70)
    self.blue = (0,0,255)
    self.green = (0,255,0)
    self.red = (255,0,0)
    self.white = (255,255,255)


  def drawMap(self, obstacles):
    pygame.draw.circle(self.map,self.green,self.start,self.nodeRadius+5,0)
    pygame.draw.circle(self.map,self.red,self.goal,self.nodeRadius+20,1)
    self.drawObs(obstacles)

  def drawPath(self, path):
    for node in path:
      pygame.draw.circle(self.map,self.red,node,self.nodeRadius+3,0)

  def drawObs(self, obstacles):
    obstaclesList=obstacles.copy()
    while (len(obstaclesList)>0):
      obstacle=obstaclesList.pop(0)
      pygame.draw.rect(self.map, self.grey, obstacle)


class SamplingGraph:
  def __init__(self, start, goal, mapDim, obsDim, obsNum):
    (x,y)=start
    self.start=start
    self.goal=goal
    self.goalFlag=False       #flag raised when robot reaches goal config    
    self.m_h, self.m_w = mapDim
    self.x=[]           # array to save and keep track of node x coordinates
    self.y=[]           # same for y coordinates
    self.parent=[]      # parent nodes

    # tree initialization
    self.x.append(x)
    self.y.append(y)
    self.parent.append(0)  # the initial parent node is called "0"

    # obstacle settings
    self.obstacles=[]
    self.obsDim=obsDim
    self.obsNum=obsNum

    # path
    self.goalState=None   # has the tree reached goal region?
    self.path=[]


  def makeRandomRect(self):
    # obstacles are rectangular in shape and we define the coordinates of
    # the upper left corner of the rectangle.
    x_rect = int(random.uniform(0, self.m_w-self.obsDim))
    y_rect = int(random.uniform(0, self.m_h-self.obsDim))
    
    return (x_rect, y_rect)

  def makeObs(self):
    obs=[]

    for i in range(0, self.obsNum):
      temp=None # temporary variable to hold rectangle before it gets stored
      containsStartGoal = True   # flag to see whether or not start and goal 
                                 # configs are inside an obstacle
      while containsStartGoal:
        ul_corner = self.makeRandomRect()
        temp = pygame.Rect(ul_corner, (self.obsDim, self.obsDim))
        if temp.collidepoint(self.start) or temp.collidepoint(self.goal):
          containsStartGoal = True       # verifies if start or goal is inside
                                         # an obstacle rectangle
        else:
          containsStartGoal = False
      obs.append(temp)
    self.obstacles=obs.copy()
    return obs 
 
  def addNode(self,n,x,y):   #n is node ID, x,y are its coordinates
    self.x.insert(n,x)
    self.y.insert(n,y)
  
  def removeNode(self,n):
    self.x.pop(n)
    self.y.pop(n)

  def addEdge(self,parent,child):
    self.parent.insert(child,parent)    # child serves as index in parent list and parent 

  def removeEdge(self,n):
    self.parent.pop(n)

  def numberOfNodes(self):
    return len(self.x)
  
  def distance(self,n1,n2):       #compute the euclidean distance between two nodes 
    (x1,y1)=(self.x[n1],self.y[n1])
    (x2,y2)=(self.x[n2],self.y[n2])
    px=(float(x1)-float(x2))**2
    py=(float(y1)-float(y2))**2
    return (px+py)**(0.5)

  def distance_to_goal(self,n,ngoal):
    (x1,y1)=(self.x[n],self.y[n])
    (x2,y2)=(ngoal[0],ngoal[1])
    px=(float(x1)-float(x2))**2
    py=(float(y1)-float(y2))**2
    return (px+py)**(0.5)


  def sample_envir(self):          # randomly sample point in map   
    x=int(random.uniform(0,self.m_w))
    y=int(random.uniform(0,self.m_h))
    return x,y
  
  def nearest(self,n):      # find the closest to node in the tree to a given node
    dmin=self.distance(0,n)
    nnear=0
    for i in range(0,n):
      if self.distance(i,n)<dmin:
        dmin=self.distance(i,n)
        nnear=i
    return nnear

  def isFree(self):            # check if newly created node collides with an obstacle. If it does, it removes it.
    n=self.numberOfNodes()-1
    (x,y)=(self.x[n],self.y[n])
    obs=self.obstacles.copy()
    while len(obs)>0:
      temp=obs.pop(0)
      if temp.collidepoint(x,y):
        self.removeNode(n)
        return False
    return True

  def crossObstacle(self,x1,x2,y1,y2):     # check to see if an edge between two nodes crosses an obstacle
    obs=self.obstacles.copy()
    while (len(obs)>0):
      temp=obs.pop(0)
      for i in range(0,101):    # here we are doing interpolation to check if intermediate points on the line collide with any given obstacle
        u=i/100
        x=x1*u + x2*(1-u)
        y=y1*u + y2*(1-u)
        if temp.collidepoint(x,y):
          return True
    return False

  def connect(self,n1,n2):
    (x1,y1) = (self.x[n1],self.y[n1])
    (x2,y2) = (self.x[n2],self.y[n2])
    if self.crossObstacle(x1,x2,y1,y2):
      self.removeNode(n2)
      return False
    else:
      self.addEdge(n1,n2)
      return True

  def step(self,n1,n2,dmax=35):      #dmax defines the step size
    d=self.distance(n1,n2)
    if d>dmax:
      (x1,y1)=(self.x[n1],self.y[n1])
      (x2,y2)=(self.x[n2],self.y[n2])
      (px,py)=(x2-x1,y2-y1)
      theta=math.atan2(py,px)
      (x,y)=(int(x1+dmax*math.cos(theta)),
             int(y1+dmax*math.sin(theta)))
      self.removeNode(n2)
      if abs(x-self.goal[0])<dmax and abs(y-self.goal[1])<dmax:
        self.addNode(n2,self.goal[0],self.goal[1])
        self.goalState = n2
        self.goalFlag=True
      else:
        self.addNode(n2,x,y)

  def extend(self,n1,n2):              # note here that I'm doing a 1-step extension. Due to lack of time, I didn't do a multi step linear polation.
    self.step(n1,n2)
    self.connect(n1,n2)


  def pathToGoal(self):         # gets the path (node numbers) in the tree that goes from start to goal
    if self.goalFlag:
      self.path=[]
      self.path.append(self.goalState)
      newpos=self.parent[self.goalState]
      while (newpos!=0):
        self.path.append(newpos)
        newpos=self.parent[newpos]
      self.path.append(0)
    return self.goalFlag


  def getPathCoords(self):        # gets the path (coordinates) in the tree that goes from start to goal
    pathCoords=[]
    for node in self.path:
      x,y=(self.x[node],self.y[node])
      pathCoords.append((x,y))
    return pathCoords

  def bias_RRT(self,ngoal):      # bias inspired by 1.2 that I added for RRT
    n=self.numberOfNodes()
    self.addNode(n,ngoal[0],ngoal[1])
    nnear=self.nearest(n)
    self.step(nnear,n)
    self.connect(nnear,n)
    return self.x,self.y,self.parent

  def expand(self):
    n=self.numberOfNodes()
    x,y=self.sample_envir()
    self.addNode(n,x,y)
    if self.isFree():
      nnearest=self.nearest(n)
      self.extend(nnearest,n)
    return self.x,self.y,self.parent
  
  def random_expand(self):      # used for 1.1
    n=self.numberOfNodes()
    x,y=self.sample_envir()
    self.addNode(n,x,y)
    if self.isFree():
      nrand=random.randint(0,n-1)
      self.extend(nrand,n)
    return self.x,self.y,self.parent

  def knowledge_expand(self,ngoal):     # used for 1.2
    n=self.numberOfNodes()
    x,y=self.sample_envir()
    self.addNode(n,x,y)
    if self.isFree():
      dmin=self.distance(0,n)
      nnear=0
      for i in range(0,n):
        if self.distance_to_goal(i,ngoal)<dmin:
          dmin=self.distance(i,n)
          nnear=i
      self.extend(nnear,n)
    return self.x,self.y,self.parent
  
