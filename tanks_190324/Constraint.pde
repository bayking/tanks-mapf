/*
* Mustafa Bay
* Maximilian Törnqvist
* Fredrik Hammar
*/

/**
* A constraint for a given agent a1 is a tuple (a1, v, t) 
* where agent a1 is prohibited from occupying vertex v at time step t. 
* During the course of the CBS algorithm agents are associated with constraints.
*
*/
public class Constraint {
    final Tank4 t;
    final PVector p;
    final int step;
    final int hash;

    /**
    * Constructs a Constraint with the specified components
    *
    * @param t      The given agent.
    * @param pos    PVector that represents vertex v
    * @param time   Time point t
    */
    public Constraint(Tank4 t, PVector p, int step) {
        this.t = t;
        this.p = p;
        this.step = step;
        this.hash = Objects.hash(this.t, this.p, this.step);
    }
    
    public Constraint() {
      this.t = null;
      this.p = null;
      this.step = 0;
      this.hash = Objects.hash(this.t, this.p, this.step);
    }
    
    @Override
        public boolean equals(Object o) {
            if (o == this) return true;
            if (o instanceof Constraint) {
                Constraint other = (Constraint) o;
                return Objects.equals(this.t, other.t) && 
                        Objects.equals(this.p, other.p) &&
                        Objects.equals(this.step, other.step);
            }
            return false;
        }
        
        @Override
        public int hashCode() {
            return hash;
        }
}
