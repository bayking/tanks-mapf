/*
* Mustafa Bay
* Maximilian Törnqvist
* Fredrik Hammar
*/

/*
* A datastructure that contains different search algorithms.
* A tank holds one object of this class to simplify testing between different solutions to search problems
*
* @author Mustafa Bay
*/
import java.util.HashSet;
import java.util.LinkedList;
import java.util.PriorityQueue;

public class Search {
    // Default constructor
    public Search() {
    }
    
    public ArrayList<Path> pp(){
      ArrayList<Path> pp = new ArrayList<Path>();
      // No constraints at start
      HashSet<Constraint> constraints = new HashSet<Constraint>();
      //Find individual paths using low-level
      for (int i = 0; i < teams[1].tanks.length; i++) {
          Tank4 current = (Tank4) teams[1].tanks[i];
          Path path = current.getPath(constraints);
          pp.add(path);
          HashMap<Integer, PVector> pathMap = path.path;
          for(Map.Entry<Integer, PVector> entry : pathMap.entrySet()){
            for(Tank t : teams[1].tanks) {
              if(current.id != t.id) {
              constraints.add(new Constraint((Tank4) t, entry.getValue(), entry.getKey()));
            }
          }
        }  
      }
      return pp;
    }

    
    /**
    * A constrained implementation of A* Search
    * Starts traversing from the initial state formulated in the problem formulation
    * and aims to find a path to the given goal node having the smallest cost while
    * avoiding nodes in constraints. It does this by maintaining a tree of paths 
    * originating at the start node and extending those paths one edge at a time
    * until its termination criterion is satisfied.
    *
    * f(n) = pathcost + manhattanDistance()
    *
    * Returns the Nod correspoding to the goal state of the problem.
    *
    * @param problem     the problem formulation of the agent that is calling this method
    * @param goal        the state to reach
    * @param constraints nodes/PVectors/states that may not be visisted in path at a given time point
    * @return Nod        corresponding to the goal state of the problem
    */
    public Nod<PVector, String> constrainedAStar(GeneralProblem<PVector, String> problem, PVector goal, HashSet<Constraint> constraints, Tank4 t) {
        problem.setGoal(goal);
        Nod<PVector, String> node = new Nod(problem.getInitialState());
        float childDist = 1;

        HashMap<Nod<PVector, String>, Nod<PVector, String>> parentMap = new HashMap<Nod<PVector, String>, Nod<PVector, String>>();
        HashMap<Nod<PVector, String>, Float> distances = new HashMap<Nod<PVector, String>, Float>();
        HashSet<Nod<PVector, String>> explored = new HashSet<Nod<PVector, String> >();
        PriorityQueue<Nod> frontier = initQueue();

        distances.put(node, node.getPathCost());
        frontier.add(node);
        while(!frontier.isEmpty()) {
            node = frontier.poll();

            if (!explored.contains(node)) {
                explored.add(node);
                if (problem.testGoal(node.getState())) {
                  return node;
                }
                HashSet<Nod<PVector, String>> children = node.expand(problem);
                for (Nod<PVector, String> n : children) {
                    Constraint constraint = new Constraint(t, n.getState(), n.step);
                    Constraint c = new Constraint(t, n.getState(), n.step);
                    if (!explored.contains(n) && !frontier.contains(n) && !grid.getNearestNode(n.state).hasTree() && !constraints.contains(constraint)) {
                        frontier.add(n);
                    }
                    else if (frontier.contains(n) && !grid.getNearestNode(n.state).hasTree() && !constraints.equals(constraint)) {
                        float manDist = this.manhattanDistance(n.getState(), problem.getGoal());
                        n.pathCost = n.getPathCost() + manDist;
                        for (Nod open_node : frontier) {
                          if (n.getState().equals(open_node.getState()) && n.pathCost < open_node.pathCost){
                            open_node = n;
                          }
                        }
                    }
                }
            }
        }
        return node;
    }
    
    /*
    * Helper method to initialize PriorityQueue with path cost as the comparing field.
    */
    private PriorityQueue<ConstraintTreeNode> initConstraintQueue() {
        return new PriorityQueue<>(10, new Comparator<ConstraintTreeNode>() {
            public int compare(ConstraintTreeNode x, ConstraintTreeNode y) {
                if (x.getCost() < y.getCost()) {
                return -1;
                }     
                if (x.getCost() > y.getCost()){
                    return 1;
                }
                return 0;
            };
        });
      }
      
      
      /*
      * Once a consistent path has been found for each agent
      * with respect to its constraints, these paths are then validated
      * with respect to the other agents. The validation is performed
      * by simulating the set of k paths. If all agents reach their goal
      * without any conflict, this CT node N is declared as the goal
      * node and a null Conflict is returned.
      * If, however, while performing
      * the validation a conflict C = (a1, a2 , v, t) is found for two
      * or more agents a1 and a2, the validation halts and the node
      * is declared as a non-goal node and the conflict is returned.
      *
      * @param solutions  a list that contains all paths in a CT-node.
      * 
      * @return           a conflict if found, else a null Conflict.  
      */
     private Conflict validate(ArrayList<Path> solutions) {
        // Random high number that no cost will reach
        int minPathCost = 2131212;
        
        // Get highest path cost
        for (Path p : solutions) {
            if (p.totalCost < minPathCost)
                minPathCost = p.totalCost;
        }

        Path path1 = solutions.get(0);
        Path path2 = solutions.get(1);
        Path path3 = solutions.get(2);
        for (Path p : solutions) {
                //print("Tank: " + p.tank.id + "Path: " + p.path);
              }
        
        // Compare each PVector at every time point up to minPathCost.
        // If Conflict is found, return it, else return null.
        for (int i = 1; i <= minPathCost; i++) {
            if (path1.path.get(i).equals(path2.path.get(i))) {
              Conflict conflict = new Conflict(path1.tank, path2.tank, path1.path.get(i), i);
              //print(" pos: " + conflict.pos + " time " + conflict.time + " ");
              return conflict;
            }
            else if (path2.path.get(i).equals(path3.path.get(i))) {
              Conflict conflict = new Conflict(path2.tank, path3.tank, path2.path.get(i), i);
              //print(" pos: " + conflict.pos + " time " + conflict.time + " ");
              return conflict;
            }
            else if (path1.path.get(i).equals(path3.path.get(i))) {
              Conflict conflict = new Conflict(path1.tank, path3.tank, path1.path.get(i), i);
              //print(" pos: " + conflict.pos + " time " + conflict.time + " ");
              return conflict;
            }
        }
        return null;
    }

    /**
    * Helper function that returns a constrained path for every
    * agent in a team.
    *
    * @param constraints  a set of constraints
    * @param team         the team of agents to get paths from
    * @return             a list of paths for every agent in team
    */
    private ArrayList<Path> getPaths(HashSet<Constraint> constraints, Team2 team) {
        ArrayList<Path> solutions = new ArrayList<>();
        for (int i = 0; i < team.tanks.length; i++) {
            solutions.add(team.getPath(constraints, (Tank4) team.tanks[i]));
        }
        return solutions;
    }
    
    /**
    * A two-level algorithm that returns optimal paths for every
    * agent in a team that calls it. At the high-level, a search
    * is performed on a tree based on conflicts between agents.
    * At the low-level, a* search is performed only for a single agent
    * at a time.
    *
    * @return  a set of optimal paths for every agent in the team
    *          that calls the method.
    *
    */
    public ArrayList<Path> csb() {
        // No constraints at start
        Constraint emptyConstraint = new Constraint();
        HashSet<Constraint> constraints = new HashSet<Constraint>(); 
        // Find individual paths using low-level
        ArrayList<Path> solutions = getPaths(constraints, (Team2) teams[1]);
        // Define root node
        ConstraintTreeNode<Constraint, Path> root = new ConstraintTreeNode<Constraint, Path>(constraints, solutions, totalCostFn);
        // Declare node and conflict pointers
        ConstraintTreeNode<Constraint, Path> current;
        Conflict conflict;
        // init queue and add root to open set/frontier
        PriorityQueue<ConstraintTreeNode> frontier = initConstraintQueue();
        frontier.add(root);

        while (!frontier.isEmpty()) {
            // get best node from open set (lowest solution cost)
            current = frontier.poll();
            // if current has no conflicts, return solution
            if (validate(current.getSolutions()) == null) {
                return current.solutions;
            }
            // else get first conflict in node
            conflict = validate(current.getSolutions());
            //print(" pos: " + conflict.pos + " time " + conflict.time + " ");
            // create two children from current. Both children inherit the constraint and create new solutions.
            for(Tank4 t : conflict.tanks) {
                Constraint constraint = new Constraint(t, conflict.pos, conflict.time);
                HashSet<Constraint> holder = current.constraints;
                holder.add(constraint);
                ConstraintTreeNode<Constraint, Path> A = new ConstraintTreeNode(holder, current.solutions, totalCostFn);
                ArrayList<Path> paths = new ArrayList<>();
                for (Path p : A.solutions){
                  if (p.tank == t) {
                    p = t.getPath(A.constraints);
                    }
                    paths.add(p);
                }
                A.solutions = paths;
                A.updateCost();
                frontier.add(A);
            }
            //print(" size " + frontier.size());
        }
        return solutions;
    }
    
    
    
     public Nod<PVector, String> dfs(GeneralProblem problem) {
      Stack<Nod> frontier = new Stack<Nod>();
        HashSet<Nod> explored = new HashSet<Nod>();
        
        Nod<PVector, String> node = new Nod(problem.getInitialState());
        if (problem.testGoal(node.getState())) {
            return node;
        }
        frontier.add(new Nod(problem.getInitialState()));

        while(!frontier.isEmpty()) {
          Nod parent = frontier.pop();
           if (problem.testGoal(parent.getState())) {
                 return parent;
             }
           explored.add(parent);
           for (Object o : problem.getActions(parent.getState())) {
                String a = (String) o;
                Nod<PVector, String> child = parent.childNode(problem, o);

              if(!explored.contains(child)) {

               frontier.add(child);
              }
            
           }

        }
      
      return node;
    }
     /**
    * An implementation of Breadth-First Search
    * Starts traversing from the initial state formulated in the problem formulation
    * and "explores all of neighbors nodes at the present depth prior to moving on to
    * the nodes at the next depth level" (Wikipedia)
    *
    * Returns the Nod correspoding to the goal state of the problem.
    *
    * @param problem the problem formulation of the agent that is calling this method
    * @return Nod corresponding to the goal state of the problem
    */
    public Nod<PVector, String> bfs(GeneralProblem<PVector, String> problem) {
        Queue<Nod> frontier = new LinkedList<Nod>();
        HashSet<Nod> explored = new HashSet<Nod>();
        Nod<PVector, String> node = new Nod(problem.getInitialState());
        if (problem.testGoal(node.getState())) {
            return node;
        }
        frontier.add(node);
        
        while(!frontier.isEmpty()) {
            node = frontier.poll();
            explored.add(node);
            for (Object o : problem.getActions(node.getState())) {
                String a = (String) o;
                Nod<PVector, String> child = node.childNode(problem, a);
                if (!frontier.contains(child) || !explored.contains(child)) {
                    if (problem.testGoal(child.getState())) {
                        return child;
                    }
                    frontier.add(child);
                }
            }
        }
        return node;
    }
    
 
    /**
    * An implementation of A* Search
    * Starts traversing from the initial state formulated in the problem formulation
    * and "aims to find a path to the given goal node having the smallest cost.
    * It does this by maintaining a tree of paths originating at the start node and
    * extending those paths one edge at a time until its termination criterion is satisfied." (Wikipedia)
    *
    * f(n) = pathcost + manhattanDistance()
    *
    * Returns the Nod correspoding to the goal state of the problem.
    *
    * @param problem the problem formulation of the agent that is calling this method
    * @return Nod corresponding to the goal state of the problem
    */
    public Nod<PVector, String> aStarSearch(GeneralProblem<PVector, String> problem, PVector goal) {
        Nod<PVector, String> node = new Nod(problem.getInitialState());
        float childDist = 1;

        HashMap<Nod<PVector, String>, Nod<PVector, String>> parentMap = new HashMap<Nod<PVector, String>, Nod<PVector, String>>();
        HashMap<Nod<PVector, String>, Float> distances = new HashMap<Nod<PVector, String>, Float>();
        HashSet<Nod<PVector, String>> explored = new HashSet<Nod<PVector, String> >();
        PriorityQueue<Nod> frontier = initQueue();

        distances.put(node, node.getPathCost());
        frontier.add(node);

        while(!frontier.isEmpty()) {
            node = frontier.poll();

            if (!explored.contains(node)) {
                explored.add(node);
                if (problem.testGoal(node.getState())) {
                  return node;
                }
                HashSet<Nod<PVector, String>> children = node.expand(problem);
                for (Nod<PVector, String> n : children) {
                    if (!explored.contains(n) && !frontier.contains(n) && !grid.getNearestNode(n.state).hasTree())
                        frontier.add(n);
                    else if (frontier.contains(n)) {
                        float manDist = this.manhattanDistance(n.getState(), problem.getGoal());
                        float totalDist = n.getPathCost() + manDist;
                        if (totalDist < (node.getPathCost()+this.manhattanDistance(node.getState(),  problem.getGoal()))) {
                            frontier.remove(node);
                            frontier.add(n);
                        }
                    }
                }
            }
        }
        return node;
    }
    
    /*
    * Helper method to initialize PriorityQueue with path cost as the comparing field.
    */
    private PriorityQueue<Nod> initQueue() {
        return new PriorityQueue<>(10, new Comparator<Nod>() {
            public int compare(Nod x, Nod y) {
                if (x.getPathCost() < y.getPathCost()) {
                return -1;
                }     
                if (x.getPathCost() > y.getPathCost()){
                    return 1;
                }
                return 0;
            };
        });
    }
    
    /* 
    * Heuristic function that returns the manhattan distance between two vectors
    *
    * @param x the first vector
    * @param y the second vector
    * @return  the manhattan distance between x and y
    */
    private float manhattanDistance(PVector x, PVector y) {
        float distance = Math.abs(x.x - y.x)
                + Math.abs(x.y - y.y);
        return distance;
    }
    
    /*
    * Learning Real-Time A* Search
    * Builds a map of the environment with problem.getActions(state) and problem.getResult()
    * then chooses the "apparently best" move according to its current cost estimates
    * (Russel & Norvig, 2016)
    *
    * Returns the "apparently best" action given its current state
    *
    * @param state the current state of the agent calling this method
    * @param problem the problem formulation of the agent that is calling this method
    * @return String the "apparently best" action from state
    */
    
    public String LRTA(PVector state, GeneralProblem<PVector, String> problem) {
        float minCost = 1000000000;
        Nod<PVector, String> initialNod = new Nod(state);
        String a = null;

        if (problem.testGoal(initialNod.getState()))
            return "stop"; // stop
        for (String b : problem.getActions(state)) {
            if (LRTACost(state, b, problem.getResult(state, b), problem) < minCost) {
                minCost = LRTACost(state, b, problem.getResult(state, b), problem);
                a = b;
            }
        }
        return a;
    }
    
    /*
    * Helper function to calculate the estimated cost to reach the goal through a neighbor.
    * "The estimated cost to reach the goal through a neighbor state is the cost to get to state plus the estimated
    * cost to get to a goal from there. c(s, a, state) + H(state) (Russel & Norvig, 2016)
    *
    * @param s current state
    * @param a an action which gets the agent from s to state
    * @param state the neighboring state
    * @param problem the problem formulation of the agent that is calling this method
    * @return r the estimated cost to reach the goal through state from s
    */ 
    public float LRTACost(PVector s, String a, PVector state, GeneralProblem<PVector, String> problem){
            float r = manhattanDistance(s, state) + manhattanDistance(state, problem.getGoal());
            return r;
        }
    
    // not used anymore

    class ParentWrapper<S, A> {
        private final S state;
        private final A action;
        private final int hash;

        private ParentWrapper(S state, A action) {
            this.state = state;
            this.action = action;
            this.hash = Objects.hash(this.state, this.action);
        }
        
        public ParentWrapper<S, A> of(S state, A action) {
            return new ParentWrapper(state, action);
        }
        
        @Override
        public boolean equals(Object o) {
            if (o == this) return true;
            if (o instanceof ParentWrapper) {
                ParentWrapper other = (ParentWrapper) o;
                return Objects.equals(this.state, other.state) && 
                        Objects.equals(this.action, other.action);
            }
            return false;
        }
        
        @Override
        public int hashCode() {
            return hash;
        }

        public S getState() {
            return state;
        }

        public A getAction() {
            return action;
        }
    }
    
    /*
    * Learning Real-Time A* Search
    * Builds a map of the environment with problem.getActions(state) and problem.getResult()
    * then chooses the "apparently best" move according to its current cost estimates
    * (Russel & Norvig, 2016)
    *
    * Returns the "apparently best" action given its current state
    *
    * @param state the current state of the agent calling this method
    * @param problem the problem formulation of the agent that is calling this method
    * @return String the "apparently best" action from state
    */
    
    public PVector LRTAp(PVector state, GeneralProblem<PVector, PVector> problem, HashSet<PVector> visited) {
        float minCost = 1000000000;
        Nod<PVector, String> initialNod = new Nod(state);
        PVector a = null;

        if (problem.testGoal(initialNod.getState()))
            return null; // stop
        for (PVector b : problem.getActions(state)) {
            if (LRTACostp(state, b, problem.getResult(state, b), problem) < minCost && !visited.contains(b)) {
                minCost = LRTACostp(state, b, problem.getResult(state, b), problem);
                a = b;
            }
        }
        return a;
    }
    
    /*
    * Helper function to calculate the estimated cost to reach the goal through a neighbor.
    * "The estimated cost to reach the goal through a neighbor state is the cost to get to state plus the estimated
    * cost to get to a goal from there. c(s, a, state) + H(state) (Russel & Norvig, 2016)
    *
    * @param s current state
    * @param a an action which gets the agent from s to state
    * @param state the neighboring state
    * @param problem the problem formulation of the agent that is calling this method
    * @return r the estimated cost to reach the goal through state from s
    *
    * Since the movement is gridbased and the cost between each gridspace is the same
    * there is no need to to calculate the cost from s to state
    */ 
    public float LRTACostp(PVector s, PVector a, PVector state, GeneralProblem<PVector, PVector> problem){
            float r = manhattanDistance(state, problem.getGoal());
            return r;
        }
}
