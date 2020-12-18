
import java.util.Arrays;
import java.util.Set;
import java.util.HashSet;
import dk.brics.automaton.Automaton;
import dk.brics.automaton.BasicAutomata;
import dk.brics.automaton.BasicOperations;
import dk.brics.automaton.MinimizationOperations;
import dk.brics.automaton.RegExp;
import dk.brics.automaton.ShuffleOperations;
import dk.brics.automaton.State;
import dk.brics.automaton.StatePair;
import dk.brics.automaton.Transition;

class autDecomp{

	//{
		//Automaton SpecialOperations.subst(Automaton a, char c, String s);
		//addEpsilons(Automaton a, Collection<StatePair> pairs);
	//}
	
	/**
	 * Returns an automaton where all transitions of the given char are replaced by a string.
	 * @param c char
	 * @param s string
	 * @return new automaton
	 */
	public static Automaton subst(Automaton a, char c, String s) {
		a = a.cloneExpandedIfRequired();
		Set<StatePair> epsilons = new HashSet<StatePair>();
		for (State p : a.getStates()) {
			Set<Transition> st = p.transitions;
			p.resetTransitions();
			for (Transition t : st)
				if (t.max < c || t.min > c)
					p.transitions.add(t);
				else {
					if (t.min < c)
						p.transitions.add(new Transition(t.min, (char)(c - 1), t.to));
					if (t.max > c)
						p.transitions.add(new Transition((char)(c + 1), t.max, t.to));
					if (s.length() == 0)
						epsilons.add(new StatePair(p, t.to));
					else {
						State q = p;
						for (int i = 0; i < s.length(); i++) {
							State r;
							if (i + 1 == s.length())
								r = t.to;
							else
								r = new State();
							q.transitions.add(new Transition(s.charAt(i), r));
							q = r;
						}
					}
				}
		}
		a.addEpsilons(epsilons);
		a.deterministic = false;
		a.removeDeadTransitions();
		a.checkMinimizeAlways();
		return a;
	}
	
  public static void main(String args[]){
    
	  RegExp regExpr = new RegExp("a(b|h|d)");
	  Automaton gloAut = regExpr.toAutomaton();
	  //BasicOperations.determinize(gloAut);
	  System.out.println(gloAut.getStates());
	  
	  Character gES[] = {'a', 'b'};
	  Character ES[] = {'a'};
	  var gEventSet = Set.of(gES);
	  var eventSet = Set.of(ES);
	  //gEventSet.removeAll(eventSet);
	  //String s = "abcccdc";
	  //public Automaton();
	  //public void setInitialState(State s);
	  //public class State;
	  //public void addTransition(Transition t);
	  //public class Transition;
	  //Transition(char min, char max, State to);
	  //Transition(char c, State to);

	  //Project
	  //Automaton proj1 = projectChar(gloAut, eventSet);
	  Automaton proj1_ = projectChar(gloAut, gEventSet);

	  //Parallel composition
	  Automaton invProj = ShuffleOperations.shuffle(gloAut, gloAut);

	  //Equvalance
          System.out.println(eventSet);
          System.out.println(gEventSet);
          //System.out.println(proj1_);
          //System.out.println(gloAut);

  }
}
