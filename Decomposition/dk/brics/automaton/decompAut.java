
package dk.brics.automaton;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import dk.brics.automaton.Automaton;
import dk.brics.automaton.RegExp;
import dk.brics.automaton.ShuffleOperations;
import dk.brics.automaton.SpecialOperations;
import dk.brics.automaton.State;
import dk.brics.automaton.Transition;

class decompAut{
	
	/***Get the event set of automaton******/
	public static Set<Character> getEventSet(Automaton aut) {
		Set<Character> globalEventSet = new HashSet<Character>();
		
		for(State sta : aut.getStates())
			for(Transition tra : sta.getTransitions())
				for(int event = (int)tra.getMin(); event <= (int)tra.getMax(); event++)
					globalEventSet.add((char)event);
		
		return globalEventSet;
	}
	
	/***powerSet generation******/
	public static Set<Set<Character>> powerSet(Set<Character> originalSet) {
        Set<Set<Character>> sets = new HashSet<Set<Character>>();
        
        if (originalSet.isEmpty()) {
            sets.add(new HashSet<Character>());
            return sets;
        }
        
        List<Character> list = new ArrayList<Character>(originalSet);
        Character head = list.get(0);
        Set<Character> rest = new HashSet<Character>(list.subList(1, list.size()));
        for (Set<Character> set : powerSet(rest)) {
            Set<Character> newSet = new HashSet<Character>();
            newSet.add(head);
            newSet.addAll(set);
            sets.add(newSet);
            sets.add(set);
        }
        
        return sets;
    }
	
	/***PowerSet to Sorted ArrayList******/
	public static List<Set<Character>> powerSetToArrayList(Set<Set<Character>> powerSet){	  
		List<Set<Character>> powerArrayList = new ArrayList<Set<Character>>();
		powerArrayList.addAll(powerSet);
		
		Collections.sort(powerArrayList, new Comparator<Set<Character>>(){
				@Override
				public int compare(Set<Character> a, Set<Character> b) {
					return a.size() - b.size();// Comparison
					}
				});
		//System.out.println(powerArrayList);
		
		return powerArrayList;
	}
	
	/***Sorted PowerSet(ArrayList) to Binary Event Pairs******/
	public static List<List<Set<Character>>> getEventSetPair(List<Set<Character>> powerArrayList, 
			Set<Character> eventSet){
		List<List<Set<Character>>> PairArrayList = new ArrayList<List<Set<Character>>>();
		int powerIndxCnt = 0;
		
		for (Set<Character> eventSubset : powerArrayList) {
			if (powerIndxCnt++ >= powerArrayList.size()/2)
				break;
			
			Set<Character> eventSetPairs = new HashSet<Character>();
			eventSetPairs.addAll(eventSet);
			eventSetPairs.removeAll(eventSubset);

			@SuppressWarnings("serial")
			List<Set<Character>> eventPairArray = new ArrayList<Set<Character>>() {{
				add(eventSubset); 
				add(eventSetPairs);}};
				  
			//System.out.println(eventPairArray);
			PairArrayList.add(eventPairArray);
		}
		//System.out.println(PairArrayList);
		
		return PairArrayList;
	}
	
	/***Iterative Parallel Decomposition*********************/
	public static List<Automaton> paraDecomp(RegExp regExpr){
		///***Initial Regular Expression and Automaton******///
		//RegExp regExpr = new RegExp("a(b|(cd)*)");
		//RegExp regExpr = new RegExp("a(bd|db)|b(ad|da)|d(ab|ba)");
		//RegExp regExpr = new RegExp("(bd)|(db)");
		
		Automaton gloAut = regExpr.toAutomaton();
		System.out.println("The global automaton is:");
		System.out.println(gloAut);
		//System.out.println(getEventSet(gloAut));
	  
		///***get Automaton Event Set******///
		Set<Character> gEventSet = new HashSet<Character>();
		gEventSet.addAll(getEventSet(gloAut));
	  
		/////////////////////***Begin Iterative Decomposition******//////////////////////
		Set<Character> IterGEventSet = new HashSet<Character>();
		IterGEventSet.addAll(gEventSet);		
		Automaton IterGloAut = gloAut.clone();
		boolean DECOMFLAG = false;
		List<Automaton> subAutSet = new ArrayList<Automaton>();
		
		while (IterGEventSet.size() > 1) {	
			///***get the PowerSet of Automaton Event Set******///
			Set<Set<Character>> gEventSetPowerSet = powerSet(IterGEventSet);
			gEventSetPowerSet.remove(IterGEventSet);
			gEventSetPowerSet.remove(Collections.emptySet());
			//System.out.println(gEventSetPowerSet);
			
			///***PowerSet to Sorted ArrayList******///
			List<Set<Character>> gEventPowerArrayList = powerSetToArrayList(gEventSetPowerSet);
		  
			///***Sorted PowerSet(ArrayList) to Binary Event Pairs******///
			List<List<Set<Character>>> gEventPairArrayList = getEventSetPair(gEventPowerArrayList, 
					IterGEventSet);
	  
			///***(One Step Iteration)Automaton Decomposition with Binary Event Pairs******///
			for (List<Set<Character>> gEventPair : gEventPairArrayList) {
				char eventPair1Array[] = new char[(gEventPair.get(0)).size()];
				char eventPair2Array[] = new char[(gEventPair.get(1)).size()];
				
				///***Event Set Pairs to Array******///
				int i = 0;
				for(Character gEvent : gEventPair.get(0))
					eventPair1Array[i++] = gEvent;
				i = 0;
				for(Character gEvent : gEventPair.get(1))
				eventPair2Array[i++] = gEvent;
		  
				///***Project******///
				Automaton proj1 = SpecialOperations.project(IterGloAut, eventPair1Array);
				Automaton proj1_ = SpecialOperations.project(IterGloAut, eventPair2Array);
		  
				///***Parallel composition******///
				Automaton invProj = ShuffleOperations.shuffle(proj1, proj1_);
				
				///***Equivalence Verification******///
				//System.out.println(invProj);
			
				if (IterGloAut.equals(invProj)) {
					DECOMFLAG = true;
					
					subAutSet.add(proj1);
					IterGEventSet.removeAll(gEventPair.get(0));
					IterGloAut = proj1_;
				
					//System.out.println(proj1_);
					break;
				}
			}
			
			if (!DECOMFLAG)
				break;
		}
		
		subAutSet.add(IterGloAut);
		///***print out automata******//////
		if (!subAutSet.isEmpty()) {
			System.out.println("The decomposed subautomata are:");
			for (Automaton subAut : subAutSet)
				System.out.println(subAut);
		}
		
		return subAutSet;
		//System.out.println(gloAut.toDot());
		/////////////////////***End Iterative Decomposition******//////////////////////

	}
}
