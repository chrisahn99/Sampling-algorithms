import pygame
from RRTbase import RRTGraph
from RRTbase import RRTMap


def main():
  dimensions=(800,1200)
  start=(50,50)
  goal=(900,700)
  obsdim=60
  obsnum=50
  iteration=0

  pygame.init()
  map=RRTMap(start,goal,dimensions,obsdim,obsnum)
  graph=RRTGraph(start,goal,dimensions,obsdim,obsnum)

  obstacles=graph.makeObs()
  map.drawMap(obstacles)

  while(not graph.pathToGoal()):
    X,Y,Parent=graph.knowledge_expand(goal)
    pygame.draw.circle(map.map,map.grey,(X[-1],Y[-1]),map.nodeRadius+2,0)
    pygame.draw.line(map.map,map.blue,(X[-1],Y[-1]),(X[Parent[-1]],Y[Parent[-1]]),
                    map.edgeThickness)
      
    pygame.display.update()
    iteration+=1

  map.drawPath(graph.getPathCoords())
  pygame.display.update()
  running = True

  # game loop
  while running:
    
  # for loop through the event queue  
    for event in pygame.event.get():
    
        # Check for QUIT event      
        if event.type == pygame.QUIT:
            running = False


if __name__=="__main__":
    main()