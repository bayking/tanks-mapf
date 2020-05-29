/*
* Mustafa Bay
* Maximilian Törnqvist
* Fredrik Hammar
*/

import java.util.ArrayList;
import java.util.HashSet;
import java.util.function.Function;

/**
 * This part of the program is heavily based on:
 * Sharon G, Stern R, Felner A, Sturtevant N.
 * Conflict-Based Search For Optimal Multi-Agent Path Finding 2015.
 *
 * Many of the comments and explanations are segments from the article.
 *
 * Represents nodes in a ConstraintTree
 * At the high-level, CBS searches a constraint tree (CT). A
 * CT is a binary tree. Each node N in the CT contains
 * the following fields of data: (1) A set of constraints
 * (N.constraints). The root of the CT contains an empty
 * set of constraints. The child of a node in the CT inherits the
 * constraints of the parent and adds one new constraint for one
 * agent. (2) A solution (N.solution). A set of k paths, one
 * path for each agent. The path for agent a1 must be consistent
 * with the constraints of a1.
 * Such paths are found by the lowlevel (3) The total cost (N.cost) of the current solution
 * (summation over all the single-agent path costs). We denote
 * this cost the f-value of the node.
 * Node N in the CT is a goal node when N.solution is
 * valid, i.e., the set of paths for all agents have no conflicts.
 * The high-level performs a best-first search on the CT where
 * nodes are ordered by their costs
 *
 * @param <C> The type used to represent constraints
 * @param <P> The type used to represent paths
 *
 * @author Mustafa Bay
 */
public class ConstraintTreeNode<C, P> {

    HashSet<C> constraints;
    ArrayList<P> solutions;
    Function<ArrayList<P>, Integer> totalCostFn;
    int cost;


    /**
     * Constructs a node with the specified components, which includes a
     * total cost function.
     *
     * @param constraints   A set of constraints.
     * @param solutions     A set of k paths, one for each agent.
     * @param totalCostFn   A total cost function that sums up all single-agent path costs in solutions.
     */
    public ConstraintTreeNode(HashSet<C> constraints, ArrayList<P> solutions, Function<ArrayList<P>, Integer> totalCostFn) {
        this.constraints = constraints;
        this.solutions = solutions;
        this.totalCostFn = totalCostFn;
        this.cost = totalCostFn.apply(solutions);
    }
    
    /**
     * Constructs a node with a total cost function.
     *
     * @param totalCostFn   A total cost function that sums up all single-agent path costs in solutions.
     */
    public ConstraintTreeNode(Function<ArrayList<P>, Integer> totalCostFn) {
      this.totalCostFn = totalCostFn;
    }

    /**
     * Constructs a node with null values.
     */
    public ConstraintTreeNode() {
        this.constraints = null;
        this.solutions = null;
        this.cost = 0;
    }


    public HashSet<C> getConstraints() {
        return constraints;
    }

    public ArrayList<P> getSolutions() {
        return solutions;
    }

    public int getCost() {
        return cost;
    }

    public void setConstraints(HashSet<C> constraints) {
        this.constraints = constraints;
    }

    public void setSolutions(ArrayList<P> solutions) {
        this.solutions = solutions;
    }

    public void updateCost() {
        cost = totalCostFn.apply(solutions);
    }
}
