/*
* Mustafa Bay
* Maximilian Törnqvist
* Fredrik Hammar
*/

import java.util.HashSet;
/**
* A conflict is a tuple(a1, a2, v, t) where agent a1 and agent a2
* occupy vertex v at a time point t. 
*
* A set holds the tanks for iterability in the high-level CBS
* 
*
*/
public class Conflict {
    ArrayList<Tank4> tanks;
    PVector pos;
    int time;
     
    /**
    * Constructs a Constraint with the specified components
    *
    * @param tank1  The first agent.
    * @param tank2  The second agent.
    * @param pos    PVector that represents vertex v
    * @param time   Time point t
    */
    public Conflict(Tank4 tank1, Tank4 tank2, PVector pos, int time) {
        this.tanks = new ArrayList<Tank4>();
        tanks.add(tank1);
        tanks.add(tank2);
        this.pos = pos;
        this.time = time;
    }
    
    @Override
    public String toString() {
      return "tank1 "+tanks.get(0).id+" "+"tank2 "+ tanks.get(1).id+" "+"pos "+pos+" "+ "time " +time;
    }
}
