/*
* Mustafa Bay
* Maximilian Törnqvist
* Fredrik Hammar
*/
import java.util.List;

/**
 * Artificial Intelligence A Modern Approach (3rd Edition): page 66.<br>
 * <br>
 * A problem can be defined formally by five components: <br>
 * <ul>
 * <li>The <b>initial state</b> that the agent starts in.</li>
 * <li>A description of the possible <b>actions</b> available to the agent.
 * Given a particular state s, ACTIONS(s) returns the set of actions that can be
 * executed in s.</li>
 * <li>A description of what each action does; the formal name for this is the
 * <b>transition model, specified by a function RESULT(s, a) that returns the
 * state that results from doing action a in state s.</b></li>
 * <li>The <b>goal test</b>, which determines whether a given state is a goal
 * state.</li>
 * <li>A <b>path cost</b> function that assigns a numeric cost to each path. The
 * problem-solving agent chooses a cost function that reflects its own
 * performance measure. The <b>step cost</b> of taking action a in state s to
 * reach state s' is denoted by c(s,a,s')</li>
 * </ul>
 *
 * This implementation provides an additional solution test. It can be used to
 * compute more than one solution or to formulate acceptance criteria for the
 * sequence of actions.
 *
 * @param <S> The type used to represent states
 * @param <A> The type of the actions to be used to navigate through the state space
 *
 * @author Ruediger Lunde
 * @author Mike Stampone
 */
public interface Problem<S, A> {

    /**
     * Returns the initial state of the agent.
     */
    S getInitialState();

    /**
     * Returns the set of actions that can be executed in the given state.
     * We say that each of these actions is <b>applicable</b> in the state.
     */
    List<A> getActions(S state);

    /**
     * Returns the description of what each action does.
     */
    S getResult(S state, A action);

    /**
     * Determines whether a given state is a goal state.
     */
    boolean testGoal(S state);

    /**
     * Returns the <b>step cost</b> of taking action <code>action</code> in state <code>state</code> to reach state
     * <code>stateDelta</code> denoted by c(s, a, s').
     */
    double getStepCosts(S state, A action, S stateDelta);

    /**
     * The cost of a path that arrives at state2 from state1 via action,
     * assuming cost c to get up to state1. If the problem is such that
     * the path doesn't matter, this function will only look at state2.
     * If the path does matter, it will consider c and maybe state1 and
     * action. The default method costs 1 for every step in the path.
     *
     * @param c The path-cost from the initial state to state1
     * @return The cost of a path that arrives at state2 from state1 via action.
     */
    default float getPathCost(float c, S state1, A action, S state2) {
        return c+1;
    }

    /**
     * Tests whether a node represents an acceptable solution. The default implementation
     * delegates the check to the goal test. Other implementations could make use of the additional
     * information given by the node (e.g. the sequence of actions leading to the node). To compute
     * all or the five best solutions (not just the best), tester implementations could return false
     * and internally collect the paths of all nodes whose state passes the goal test until enough
     * solutions have been collected.
     * Search implementations should always access the goal test via this method to support
     * solution acceptance testing.
     */
    default boolean testSolution(Nod node) {
        return testGoal((S) node.getState());
    }
}
