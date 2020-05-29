/*
* Mustafa Bay
* Maximilian Törnqvist
* Fredrik Hammar
*/

import java.util.Random;
import java.util.ArrayList;
import java.util.HashSet;
class Team2 extends Team {
  

  Team2(int team_id, int tank_size, color c, 
    PVector tank0_startpos, int tank0_id, CannonBall ball0, 
    PVector tank1_startpos, int tank1_id, CannonBall ball1, 
    PVector tank2_startpos, int tank2_id, CannonBall ball2) {
    super(team_id, tank_size, c, tank0_startpos, tank0_id, ball0, tank1_startpos, tank1_id, ball1, tank2_startpos, tank2_id, ball2);  

    tanks[0] = new Tank4(tank0_id, this, this.tank0_startpos, this.tank_size, ball0, new PVector(750, 750));
    tanks[1] = new Tank4(tank1_id, this, this.tank1_startpos, this.tank_size, ball1, new PVector(750, 450));
    tanks[2] = new Tank4(tank2_id, this, this.tank2_startpos, this.tank_size, ball2, new PVector(550, 750));
    
    search = new Search();
    started = false;
    solutions = new ArrayList<>();

    //this.homebase_x = width - 151;
    //this.homebase_y = height - 351;
  }

  void updateLogic() {
    if (!started) {
      solutions = search.csb();
      tanks[0].pathQueue = solutions.get(0).pathQueue;
      tanks[1].pathQueue = solutions.get(1).pathQueue;
      tanks[2].pathQueue = solutions.get(2).pathQueue;
      
      started = true;
    }
  }
  
  Path getPath(HashSet<Constraint> constraints, Tank4 t) {
      
      Queue<Nod> tempQueue = search.constrainedAStar(t.generalProblem, t.goal, constraints, t).path();
      HashMap<Integer, PVector> pathMap = new HashMap<Integer, PVector>();
      int i = 1;
      while (!tempQueue.isEmpty()) {
        pathMap.put(i, (PVector) tempQueue.poll().state);
        i++;
      }
      Queue<Nod> xQueue = search.constrainedAStar(t.generalProblem, t.goal, constraints, t).path();
      Path pathObj = new Path(t, pathMap, xQueue, xQueue.size());
      
      return pathObj;
    }

  //==================================================
  public class Tank1 extends Tank {

    boolean started;
    Sensor locator;
    Sensor us_front; //ultrasonic_sensor front

    Tank1(int id, Team team, PVector startpos, float diameter, CannonBall ball) {
      super(id, team, startpos, diameter, ball);

      us_front = getSensor("ULTRASONIC_FRONT");
      addSensor(us_front);

      started = false;
    }

    public void initialize() {
    }

    // Tanken meddelas om kollision med tree.
    //public void message_collision(Tree other) {
    //  println("*** Team"+this.team_id+".Tank["+ this.getId() + "].collision(Tree)");

    //  chooseAction();
    //}

    // Tanken meddelas om kollision med tanken.
    public void message_collision(Tank other) {
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].collision(Tank)");

      chooseAction();
    }

    public void arrived() {
      super.arrived(); // Tank
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].arrived()");

      chooseAction();
    }

    public void arrivedRotation() {
      super.arrivedRotation();

      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].arrivedRotation()");
      //moveTo(new PVector(int(random(width)),int(random(height))));
      //moveTo(grid.getRandomNodePosition()); // Slumpmässigt mål.
      moveForward_state(); // Tank
    }

    public void chooseAction() {
      //moveTo(grid.getRandomNodePosition());
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].chooseAction()");
      //resetTargetStates(); // Tank
      //resetAllMovingStates(); // Tank

      float r = random(1, 360);
      rotateTo(radians(r));
    }

    public void readSensorDistance() {
      SensorReading sr = readSensor_distance(us_front);
      //println("1sr.distance(): "+ sr.distance());
      if ((sr.distance() < this.radius) && this.isMoving) {
        if (!this.stop_state) {
          println("Team"+this.team_id+".Tank["+ this.getId() + "] Har registrerat ett hinder. (Tank.readSensorDistance())");
          //stopMoving();
          //stopTurning_state()
          //this.stop_state = true;
          stopMoving_state(); //Tank
          //chooseAction();
        }
      }
    }

    public void updateLogic() {
      //super.updateLogic();


      // Avoid contact with other objects and tanks.
      float threshold = .1f;
      //println("========================================");
      //println("Team"+this.team_id+".Tank["+ this.getId() + "] : " + us_front.readValue(0));
      //if (us_front.readValue(0) < threshold) {
      //  println("*** Team"+this.team_id+".Tank["+ this.getId() + "]: (us_front.readValue(0) < threshold)");
      //}

      // println("Team"+this.team_id+".Tank["+ this.getId() + "] : " + us_front.readValue1());



      if (!started) {
        started = true;
        initialize();

        moveForward_state();
        //moveForward();
      }

      if (!this.userControlled) {
        readSensorDistance();

        //moveForward_state();
        if (this.idle_state) {
          //rotateTo()
          chooseAction();
        }
      }
    }
  }

  //==================================================
  public class Tank2 extends Tank {

    boolean started;

    //*******************************************************
    Tank2(int id, Team team, PVector startpos, float diameter, CannonBall ball) {
      super(id, team, startpos, diameter, ball);

      this.started = false; 

      //this.isMoving = true;
      //moveTo(grid.getRandomNodePosition());
    }

    //*******************************************************
    // Reterera, fly!
    public void retreat() {
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].retreat()");
      moveTo(grid.getRandomNodePosition()); // Slumpmässigt mål.
    }

    //*******************************************************
    // Reterera i motsatt riktning (ej implementerad!)
    public void retreat(Tank other) {
      //println("*** Team"+this.team_id+".Tank["+ this.getId() + "].retreat()");
      //moveTo(grid.getRandomNodePosition());
      retreat();
    }

    //*******************************************************
    // Fortsätt att vandra runt.
    public void wander() {
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].wander()");
      //rotateTo(grid.getRandomNodePosition());  // Rotera mot ett slumpmässigt mål.
      moveTo(grid.getRandomNodePosition()); // Slumpmässigt mål.
    }


    ////*******************************************************
    //// Tanken meddelas om kollision med trädet.
    //public void message_collision(Tree other) {
    //  println("*** Team"+this.team_id+".Tank["+ this.getId() + "].collision(Tree)");
    //  wander();
    //}

    //*******************************************************
    // Tanken meddelas om kollision med tanken.
    public void message_collision(Tank other) {
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].collision(Tank)");

      //moveTo(new PVector(int(random(width)),int(random(height))));
      //println("this.getName());" + this.getName()+ ", this.team_id: "+ this.team_id);
      //println("other.getName());" + other.getName()+ ", other.team_id: "+ other.team_id);

      if ((other.getName() == "tank") && (other.team_id != this.team_id)) {
        if (this.hasShot && (!other.isDestroyed)) {
          println("["+this.team_id+":"+ this.getId() + "] SKJUTER PÅ ["+ other.team_id +":"+other.getId()+"]");
          fire();
        } else {
          retreat(other);
        }

        rotateTo(other.position);
        //wander();
      } else {
        wander();
      }
    }
    
    //*******************************************************  
    // Tanken meddelas om den har kommit hem.
    public void message_arrivedAtHomebase() {
      //println("*** Team"+this.team_id+".Tank["+ this.getId() + "].message_isAtHomebase()");
      println("! Hemma!!! Team"+this.team_id+".Tank["+ this.getId() + "]");
    }

    //*******************************************************
    // används inte.
    public void readyAfterHit() {
      super.readyAfterHit();
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].readyAfterHit()");

      //moveTo(grid.getRandomNodePosition());
      wander();
    }

    //*******************************************************
    public void arrivedRotation() {
      super.arrivedRotation();
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].arrivedRotation()");
      //moveTo(new PVector(int(random(width)),int(random(height))));
      arrived();
    }

    //*******************************************************
    public void arrived() {
      super.arrived();
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].arrived()");

      //moveTo(new PVector(int(random(width)),int(random(height))));
      //moveTo(grid.getRandomNodePosition());
      wander();
    }

    //*******************************************************
    public void updateLogic() {
      super.updateLogic();

      if (!started) {
        started = true;
        moveTo(grid.getRandomNodePosition());
      }

      if (!this.userControlled) {

        //moveForward_state();
        if (this.stop_state) {
          //rotateTo()
          wander();
        }

        if (this.idle_state) {
          wander();
        }
      }
    }
  }

  //==================================================
  public class Tank3 extends Tank {

    PVector cr = new PVector();
    float wandertheta;
    float maxforce;

    Tank3(int id, Team team, PVector startpos, float diameter, CannonBall ball) {
      super(id, team, startpos, diameter, ball);

      this.wandertheta = 0;
      this.maxforce = 0.05;
      this.stop_state = false;
    }

    //--------------------
    // A method that calculates and applies a steering force towards a target
    // STEER = DESIRED MINUS VELOCITY
    void seek(PVector target) {
      PVector desired = PVector.sub(target, position);  // A vector pointing from the position to the target

      // Normalize desired and scale to maximum speed
      desired.normalize();
      desired.mult(maxspeed);


      // Steering = Desired minus Velocity
      PVector steer = PVector.sub(desired, velocity);

      steer.limit(maxforce);  // Limit to maximum steering force
      //println("steer: " + steer.getHeading());

      if (steer.heading() < 0) {
        this.turning_right_state = false;
        this.turning_left_state = true;
      } else
        if (steer.heading() > 0) {
          this.turning_right_state = true;
          this.turning_left_state = false;
        }

      //applyForce(steer);
    }

    void wander() {
      float wanderR = 25;         // Radius for our "wander circle"
      float wanderD = 200;//80         // Distance for our "wander circle"
      float change = 0.3;
      wandertheta += random(-change, change);     // Randomly change wander theta

      // Now we have to calculate the new position to steer towards on the wander circle
      PVector circlepos = new PVector();
      circlepos.set(velocity);    // Start with velocity
      circlepos.normalize();            // Normalize to get heading
      circlepos.mult(wanderD);          // Multiply by distance
      circlepos.add(position);               // Make it relative to boid's position

      float h = velocity.heading();        // We need to know the heading to offset wandertheta
      //float h = this.heading;        // We need to know the heading to offset wandertheta

      PVector circleOffSet = new PVector(wanderR*cos(wandertheta+h), wanderR*sin(wandertheta+h));
      PVector target = PVector.add(circlepos, circleOffSet);

      seek(target);

      // Render wandering circle, etc.
      if (debugOn) drawWanderStuff(position, circlepos, target, wanderR);
    }

    // A method just to draw the circle associated with wandering
    void drawWanderStuff(PVector position, PVector circle, PVector target, float rad) {
      stroke(0);
      noFill();
      ellipseMode(CENTER);
      ellipse(circle.x, circle.y, rad*2, rad*2);

      ellipse(circle.x, circle.y, 10, 10);
      //ellipse(position.x,position.y,10,10);

      ellipse(target.x, target.y, 4, 4);
      line(position.x, position.y, circle.x, circle.y);
      line(circle.x, circle.y, target.x, target.y);
    }
    //--------------------

    void checkFront_sensor() {
    }

    void checkEnvironment_sensor() {
      float tempx = 0;
      PVector w = new PVector(0, 0);
    }

    public void updateLogic() {
      super.updateLogic();

      if (!this.userControlled) {
        checkEnvironment_sensor();


        if (!this.stop_state) {
          moveForward_state();
          wander();

          //println("heading1: " + this.heading);
          //println("velocity1: " + this.velocity); 
          //println("velocityheading1: " + this.velocity.getHeading()); 
          //this.heading = this.velocity.getHeading();
        }
      }
    }
  }
  

  
  public class Tank5 extends Tank {
  
     boolean started;
     boolean reporting; // true when enemyspotted inside enemybase
     
     SensorDistance front_sensor; 

     GeneralProblem<PVector, PVector> problemx; // Structure to formulate problem
     PVector goal; // current goal
     Queue<Nod> path; // path towards found goal, used by offline search algorithms
     Search search; // Structure for different search methods
     HashSet<PVector> visited;
     Tank5(int id, Team team, PVector startpos, float diameter, CannonBall ball) {
       super(id, team, startpos, diameter, ball);
       started = false;
       reporting = false;
       
       //Sensors
       front_sensor = new SensorDistance(this, 0f); 
       addSensor(front_sensor);
       
       visited = new HashSet<PVector>();
       search = new Search();
       PVector goal = null;
       
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
    void formulateGoal() {
      if(!started) {
        goal = new PVector(100, 50);
      }
      if(isAtEnemybase) {
        goal = startpos;
      }
      Predicate<PVector> gPred = (s) -> s.equals(goal);
      BiFunction<PVector, PVector, PVector> rFuncTest = (state, nextState) -> {
      if (problemx.getActions(state).contains(nextState)){
        return nextState;
      }
     return state;
    };
      problemx = new GeneralProblem(position, senso, rFuncTest, gPred);
      println("goal" + goal);
    }
    
    /*
    * Returns the next node in the path queue towards a goal
    *
    * return Nod the next node in path
    */
    Nod getNextNodeInPath() {
      Nod current = (Nod) path.poll();
      println(path.size());
      return current;
    }
    
    /*
    * Wandering function only used when agent is an LRTA* Agent.
    *
    * Gives a move to the agent by calling LRTA from its search object
    */
    void lrta() {
      if (!started)
        return;
      PVector action = search.LRTAp(position, problemx, visited);
      visited.add(action);
      grid.getNearestNode(position).removeContent();
      moveTo(action);
    }
  /*
  * Returns the closeby available nodes
  * Checks if sensors detect nearby object
  *
  * @param state the current state of the agent
  * @return actions a list of possible actions given by the "close" sensors
  */
  List<PVector> readCloseSensors(PVector state) {
    ArrayList<PVector> actions = new ArrayList<>();
    if(grid.getNearestNode(new PVector(state.x, state.y-50)).empty() && !visited.contains(new PVector(state.x, state.y-50))) {
        actions.add(grid.getNearestNode(new PVector(state.x, state.y-50)).position);
      }
      if(grid.getNearestNode(new PVector(state.x, state.y+50)).empty() && !visited.contains(new PVector(state.x, state.y+50))) {
        actions.add(grid.getNearestNode(new PVector(state.x, state.y+50)).position);
      }
      if(grid.getNearestNode(new PVector(state.x+50, state.y)).empty() && !visited.contains(new PVector(state.x+50, state.y))) {
        actions.add(grid.getNearestNode(new PVector(state.x+50, state.y)).position);
      }
      if(grid.getNearestNode(new PVector(state.x-50, state.y)).empty() && !visited.contains(new PVector(state.x-50, state.y))) {
        actions.add(grid.getNearestNode(new PVector(state.x-50, state.y)).position);
      }
      return actions;
  }
  /*
  * Returns the first sprite object in sight
  */
  
  Sprite readSensor() {
    PVector wall = front_sensor.readValue2();
    float wallX = Math.round(wall.x);
    float wallY = Math.round(wall.y);
    float posX = Math.round(position.x);
    float posY = Math.round(position.y);
    
    // Left
    if (wallX == 0) {
      for(float i = posX-50; i > wallX; i-=50) {
        if(!isEmpty(new PVector(i, posY))) {
          print(grid.getNearestNode(new PVector(i, posY)).content().getName());
          return grid.getNearestNode(new PVector(i, posY)).content();
        } 
      }
    }  
    // Right
    else if (wallX == 800) {
      for(float i = posX+50; i < wallX; i+=50) {
        if(!isEmpty(new PVector(i, posY)) && !visited.contains(new PVector(i, posY))) {
          print(grid.getNearestNode(new PVector(i, posY)).content().getName());
          return grid.getNearestNode(new PVector(i, posY)).content();
        }
      }
    }  
    // Up
    else if (wallY == 0) {
      for(float i = posY-50; i > wallY; i-=50) {
        if(!isEmpty(new PVector(posX, i)) && !visited.contains(new PVector(posX, i))) {
          print(grid.getNearestNode(new PVector(posX, i)).content().getName());
          return grid.getNearestNode(new PVector(posX, i)).content();
        }
      }
    }  
    // Down
    else if (wallY == 800) {
      for(float i = posY+50; i < wallY; i+=50) {
        if(!isEmpty(new PVector(posX, i)) && !visited.contains(new PVector(posX, i))) {
          print(grid.getNearestNode(new PVector(posX, i)).content().getName());
          return grid.getNearestNode(new PVector(posX, i)).content();
        }
      }
    }
    return null;
  }
    
    

  /*
  * Returns a list of possible actions
  * Action function that stimulates percepts by asking the environment about the content of closeby nodes
  * If a node is empty it is then considered as a possible move and the corresponding action to reach that state/node
  *
  * @return List<String> the list of possible actions
  */
  
  Function<PVector, List<PVector>> senso = (state) -> {
    List<PVector> actions = new ArrayList<PVector>(readCloseSensors(position));
    PVector wall = front_sensor.readValue2();
    float wallX = Math.round(wall.x);
    float wallY = Math.round(wall.y);
    float posX = Math.round(position.x);
    float posY = Math.round(position.y);
    
    if (wallX == 0) {
      for(float i = posX-50; i > wallX; i-=50) {
        if(isEmpty(new PVector(i, posY)) && !visited.contains(new PVector(i, posY))) {
          actions.add(grid.getNearestNodePosition(new PVector(i, posY)));
        } else 
            break;
      }
      if (!actions.isEmpty()) {
        return actions;
      }
    }  
    if (wallX == 800) {
      for(float i = posX+50; i < wallX; i+=50) {
        if(isEmpty(new PVector(i, posY)) && !visited.contains(new PVector(i, posY))) {
          actions.add(grid.getNearestNodePosition(new PVector(i, posY)));
        } else 
            break;
      }
      if (!actions.isEmpty()) {
        return actions;
      }
    }  
     if (wallY == 0) {
      for(float i = posY-50; i > wallY; i-=50) {
        if(isEmpty(new PVector(posX, i)) && !visited.contains(new PVector(posX, i))) {
          actions.add(grid.getNearestNodePosition(new PVector(posX, i)));
        } else 
            break;
      }
      if (!actions.isEmpty()) {
        return actions;
      }
    }  
    if (wallY == 800) {
      for(float i = posY+50; i < wallY; i+=50) {
        if(isEmpty(new PVector(posX, i)) && !visited.contains(new PVector(posX, i))) {
          actions.add(grid.getNearestNodePosition(new PVector(posX, i)));
        } else 
            break;
      }
      if (!actions.isEmpty()) {
        return actions;
      }
    }
      return actions;
};

BiFunction<PVector, PVector, PVector> rFunctTest = (state, nextState) -> {
  if (problemx.getActions(state).contains(nextState)){
    return nextState;
  }
   return state;
};


  boolean isEmpty(PVector location) {
    if(grid.getNearestNode(location).empty()) 
      return true;
    return false;
  }
      
    //*******************************************************
    public void updateLogic() {
      super.updateLogic();

      if (!started) {
        formulateGoal();
        problemx.setGoal(goal);
        println(problemx.getGoal());
        if(!problemx.testGoal(goal)) {
          println("wohoo");
        }
        // ASTAR (Uncomment when trying a star)
        //path = search.aStarSearch(problem, goal).path();
        //println(path.size());
        
        started = true;
      }
      
      if (!reporting && isAtEnemybase) {
        formulateGoal();
        problemx.setGoal(goal);
        reporting = true;
      }

      if (!this.userControlled) {
        
        if (this.stop_state) {
          //ASTAR (Uncomment when trying a star)
          //go();
          
          //LRTAStar (Uncomment when trying LRTAStar)
          lrta();
        }

        if (this.idle_state) {
          //go();
        }        
      }
    }
  }
}
