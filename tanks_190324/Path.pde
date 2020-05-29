/*
* Mustafa Bay
* Maximilian Törnqvist
* Fredrik Hammar
*/
import java.util.HashMap;

/**
* A path container class that is used in validation by the high-level CSB
* and by a single agent to hold its path.
*
* @author Mustafa Bay
*
*/
public class Path {
    Tank4 tank;
    HashMap<Integer, PVector> path;
    Queue<Nod> pathQueue;
    int totalCost;
    
    /**
    * Constructs a Path with the specified components
    *
    * @param tank       The given agent.
    * @param path       Maps time points to PVectors. Used in validation to make sure
                        agents are not in conflict with eachother at a given time point
    * @param pathQueue  A queue that is sent to the agent when there are no conflicts
                        in any paths. Polled by the agent to get ordered actions towards goal.
    * @param totalCost  Total amount of time/steps to reach goal from agents initial state.
    */
    public Path(Tank4 tank, HashMap<Integer, PVector> path, Queue<Nod> pathQueue, int totalCost) {
        this.tank = tank;
        this.path = path;
        this.pathQueue = pathQueue;
        this.totalCost = totalCost;
    }
    /**
    * Default constructor.
    */
    public Path() {

    }
}
