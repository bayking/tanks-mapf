/*
* Mustafa Bay
* Maximilian Törnqvist
* Fredrik Hammar
*/



Function<ArrayList<Path>, Integer> totalCostFn = (x) -> {
          int s = 0;
            for (Path p : x) s += p.totalCost;
            return s;
        };

  /*
  * Returns a list of possible actions
  * Action function only checks if next action will be outside the map/grid
  *
  * @return List<String> the list of possible actions
  */
Function<PVector, List<String>> aFunc = (state) -> {
  ArrayList<String> actions = new ArrayList<>();
      if(!isOutOfBounds(state.y-50)) {
        actions.add("up");
      }
      if(!isOutOfBounds(state.y+50)) {
        actions.add("down");
      }
      if(!isOutOfBounds(state.x+50)) {
        actions.add("right");
      }
      if(!isOutOfBounds(state.x-50)) {
        actions.add("left");
      }
      actions.add("wait");
      return actions;
};

  /*
  * Returns a PVector(state) that is reached by applying String (action) on PVector(another state)
  *
  * @return PVector state reached by applying a on state
  */
BiFunction<PVector, String, PVector> rFunc = (state, a) -> {
  if(a.equalsIgnoreCase("up") && !isOutOfBounds(state.y-50)) {
        return new PVector(state.x, state.y-50);
      }
  if(a.equalsIgnoreCase("down") && !isOutOfBounds(state.y+50)) {
        return new PVector(state.x, state.y+50);
      }
  if(a.equalsIgnoreCase("right") && !isOutOfBounds(state.x+50)) {
        return new PVector(state.x+50, state.y);
      }
  if(a.equalsIgnoreCase("left") && !isOutOfBounds(state.x-50)) {
        return new PVector(state.x-50, state.y);
      }
  if(a.equalsIgnoreCase("wait")) {
    return state;
  }
   return state;
};

  /*
  * Helper function which checks if a coordinate is out of bounds
  */
boolean isOutOfBounds(float cord) {
      if(cord < 751 && cord > 49)
        return false;
      return true;
    }
    

// call to Team updateLogic()
void updateTeamsLogic() {
  teams[0].updateLogic();
  teams[1].updateLogic();
}

// call to Tank updateLogic()
void updateTanksLogic() {
  
  for (Tank tank : allTanks) {
    if (tank.isReady) {
      tank.updateLogic();
    }
  }
  
  //for (int i = 0; i < tanks.length; i++) {
  //  this.tanks[i].updateLogic();
  //}
}

// call to Tank update()
void updateTanks() {
  
  for (Tank tank : allTanks) {
    //if (tank.health > 0)
    tank.update();
  }
  
  //for (int i = 0; i < tanks.length; i++) {
  //  this.tanks[i].updateLogic();
  //}
}

void updateShots() {
  for (int i = 0; i < allShots.length; i++) {
    if ((allShots[i].passedTime > wait) && (!allTanks[i].hasShot)) {
      allShots[i].resetTimer();
      allTanks[i].loadShot();
    }
    allShots[i].update();
  }
}

void checkForCollisionsShots() {
  for (int i = 0; i < allShots.length; i++) {
    if (allShots[i].isInMotion) {
      for (int j = 0; j<allTrees.length; j++) {
        allShots[i].checkCollision(allTrees[j]);
      }
     
      for (int j = 0; j < allTanks.length; j++) {
        if (j != allTanks[i].getId()) {
          allShots[i].checkCollision(allTanks[j]);
        }
      }
    }
  }
}

void checkForCollisionsTanks() {
  // Check for collisions with Canvas Boundaries
  for (int i = 0; i < allTanks.length; i++) {
    allTanks[i].checkEnvironment();
    
    // Check for collisions with "no Smart Objects", Obstacles (trees, etc.)
    for (int j = 0; j < allTrees.length; j++) {
      allTanks[i].checkCollision(allTrees[j]);
    }
    
    // Check for collisions with "Smart Objects", other Tanks.
    for (int j = 0; j < allTanks.length; j++) {
      //if ((allTanks[i].getId() != j) && (allTanks[i].health > 0)) {
      if (allTanks[i].getId() != j) {
        allTanks[i].checkCollision(allTanks[j]);
      }
    }
  }
}

void loadShots() {
  for (int i = 0; i < allTanks.length-1; i++) {
    allTanks[i].loadShot();
  }
}

//void shoot(Tank id, PVector pvec) {
void shoot(int id) {
  println("App.shoot()");
 // println(id +": "+ pvec);
  //allShots.get(id).setStartPosition(pvec);
  
  //myAudio.blast();
  
  allShots[id].isInMotion = true;
  allShots[id].startTimer();
}

void setTargetPosition(PVector pvec) {
  PVector nodevec = grid.getNearestNodePosition(pvec);
  //allTanks[tankInFocus].moveTo(pvec);
  allTanks[tankInFocus].moveTo(nodevec);
}

//******************************************************

/**
 * Find the point of intersection between two lines.
 * This method uses PVector objects to represent the line end points.
 * @param v0 start of line 1
 * @param v1 end of line 1
 * @param v2 start of line 2
 * @param v3 end of line 2
 * @return a PVector object holding the intersection coordinates else null if no intersection 
 */
public PVector line_line_p(PVector v0, PVector v1, PVector v2, PVector v3){
  final double ACCY   = 1E-30;
  PVector intercept = null;

  double f1 = (v1.x - v0.x);
  double g1 = (v1.y - v0.y);
  double f2 = (v3.x - v2.x);
  double g2 = (v3.y - v2.y);
  double f1g2 = f1 * g2;
  double f2g1 = f2 * g1;
  double det = f2g1 - f1g2;

  if(abs((float)det) > (float)ACCY){
    double s = (f2*(v2.y - v0.y) - g2*(v2.x - v0.x))/ det;
    double t = (f1*(v2.y - v0.y) - g1*(v2.x - v0.x))/ det;
    if(s >= 0 && s <= 1 && t >= 0 && t <= 1)
      intercept = new PVector((float)(v0.x + f1 * s), (float)(v0.y + g1 * s));
  }
  return intercept;
}
  
//******************************************************
//Används inte.
/**
   * Put angle in degrees into [0, 360) range
   */
//  public static float fixAngle(float angle) {
float fixAngle(float angle) {
    while (angle < 0f)
      angle += 360f;
    while (angle >= 360f)
      angle -= 360f;
    return angle;
}

//Används inte.
//public static float relativeAngle(float delta){
float relativeAngle(float delta){
    while (delta < 180f)
      delta += 360f;
    while (delta >= 180f)
      delta -= 360f;
    return delta;
}
