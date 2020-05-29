/*
* Mustafa Bay
* Maximilian Törnqvist
* Fredrik Hammar
*/

  public class Tank4 extends Tank {
  
     boolean started;
     boolean done;
     
     SensorDistance front_sensor; 
     
     Tank4(int id, Team team, PVector startpos, float diameter, CannonBall ball, PVector goal) {
       super(id, team, startpos, diameter, ball);
       this.started = false;
       this.done = false;
       this.goal = goal;
       formulateProblem();
       
       //Sensors
       front_sensor = new SensorDistance(this, 0f); 
       addSensor(front_sensor);
       
    }
    
    //*******************************************************
    // Tanken meddelas om kollision med trädet.
    public boolean message_collision(Tree other) {
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].collision(Tree)");
      return true;
    }
    /*
    * Contains the different conditions for different goal formulations
    * Different goals can be added by adding if statements 
    *
    */
    void formulateProblem() {
      Predicate<PVector> gPred = (s) -> s.equals(goal);
      generalProblem = new GeneralProblem(position, aFunc, rFunc, gPred);
      println("position:" + position + "id: " + id + "goal: " + goal);
    }
    /*
    * Action "up"
    *
    * Moves the agents position y-50 (North on the grid)
    *
    */
    void moveUp() {
      if(!isOutOfBounds(position.y-50)) {
        moveTo(grid.getNearestNode(new PVector(position.x, position.y-50)).position);
      }
    }
    
    /*
    * Action "down"
    *
    * Moves the agents position y+50 (South on the grid)
    *
    */
    void moveDown() {
      if(!isOutOfBounds(position.y+50)) {
      moveTo(grid.getNearestNode(new PVector(position.x, position.y+50)).position);
      }
    }
    
    /*
    * Action "right"
    *
    * Moves the agents position x+50 (West on the grid)
    *
    */
    void moveRight() {
      if(!isOutOfBounds(position.x+50)) {
      moveTo(grid.getNearestNode(new PVector(position.x+50, position.y)).position);
      }
    }
    
    /*
    * Action "left"
    *
    * Moves the agents position x-50 (East on the grid)
    *
    */
    void moveLeft() {
      if(!isOutOfBounds(position.x-50)) {
        moveTo(grid.getNearestNode(new PVector(position.x-50, position.y)).position);
      }
    }
    
    /*
    * Action "wait"
    *
    * The agent doesnt move
    *
    */
    void moveWait() {
        moveTo(grid.getNearestNode(this.position).position);
    }
    
    // Initial wandering helper func
    // Not used
    ArrayList<String> getPossibleActions() {
      ArrayList<String> actions = new ArrayList<>();
      if(!isOutOfBounds(position.y-50)) {
        actions.add("up");
      }
      if(!isOutOfBounds(position.y+50)) {
        actions.add("down");
      }
      if(!isOutOfBounds(position.x+50)) {
        actions.add("right");
      }
      if(!isOutOfBounds(position.x-50)) {
        actions.add("left");
      }
      return actions;
    }

    /*
    * Returns the next node in the path queue towards a goal
    *
    * return Nod the next node in path
    */
    Nod getNextNodeInPath() {
      Nod current = (Nod) pathQueue.poll();
      println(pathQueue.size());
      return current;
    }
      
    
    /*
    * Function used for path following when a path has been constructed
    * by an offline search algorithm
    *
    * Gives a move to the agent by getting the next node in paths parent action.
    */
    void go() {
      if(!started)
        return;
      if(done)
        return;
      String action = (String) getNextNodeInPath().getAction();
      if(action.equalsIgnoreCase("up")) {
        moveUp();
      } else if (action.equalsIgnoreCase("down")) {
        moveDown();
      } else if (action.equalsIgnoreCase("left")) {
        moveLeft();
      } else if (action.equalsIgnoreCase("right")) {
        moveRight();
      } else if (action.equalsIgnoreCase("wait")) {
        moveWait();
      }
    }

  /**
  * Creates and returns a new path object with
  * a constrained path.
  *
  * @param constraints  a set of constraints that the tank must follow
  * @return             a Path object with a constrained path
  *
  */
  Path getPath(HashSet<Constraint> constraints) {
      // Call the low-level which is A*
      Queue<Nod> tempQueue = search.constrainedAStar(generalProblem, goal, constraints, this).path();
      HashMap<Integer, PVector> pathMap = new HashMap<Integer, PVector>();
      int i = 1;
      while (!tempQueue.isEmpty()) {
        pathMap.put(i, (PVector) tempQueue.poll().state);
        i++;
      }
      Queue<Nod> xQueue = search.constrainedAStar(this.generalProblem, this.goal, constraints, this).path();
      Path pathObj = new Path(this, pathMap, xQueue, xQueue.size());
      
      return pathObj;
    }
      
    //*******************************************************
    public void updateLogic() {
      super.updateLogic();
      
      if (!done) {
        if (!this.started) {
          if (team.started)
             started = true;
        }
        if (started) {
          if (pathQueue.size() == 0) {
          print("done -------------------------------------" + this.id);
          done = true;
          }
        }
        
        if (this.stop_state) {
          go();
        }       
    }
   }
 }
