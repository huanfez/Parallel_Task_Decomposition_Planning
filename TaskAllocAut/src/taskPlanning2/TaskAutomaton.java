package taskPlanning2;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.DirectedPseudograph;

import dk.brics.automaton.Automaton;
import dk.brics.automaton.State;
import dk.brics.automaton.Transition;
import dk.brics.automaton.decompAut;

public class TaskAutomaton {
	
	public String initialState;
	public ArrayList<String> acceptedStates = new ArrayList<>();
	public DirectedPseudograph<String,AutomatonEdge> AutomatonGraph;
	public Set<String> eventSet = new HashSet<>();
	
	public TaskAutomaton() {};
	
	public TaskAutomaton(String initialState, ArrayList<String> acceptedStates, DirectedPseudograph<String, 
			AutomatonEdge> AutomatonGraph, Set<String> eventSet) {
		this.initialState = initialState;
		this.acceptedStates = acceptedStates;
		this.AutomatonGraph = AutomatonGraph;
		this.eventSet = eventSet;
	}
	
	/** Initialization: task automaton described by new class**********/
	public TaskAutomaton(Automaton taskAutomaton) {
		
		System.out.println("Original automaton" + taskAutomaton);
		
		DirectedPseudograph<String,AutomatonEdge> taskAutomatonGraph = new DirectedPseudograph<>(
				AutomatonEdge.class);
		String autoStateInx, autoStateInx_;
		
		Set<State> autoStateSet= taskAutomaton.getStates();
		
		/* * Convert automaton states into graph vertices**/
		for (State autoState : autoStateSet) {
			autoStateInx = extractInteger(autoState.toString());
			taskAutomatonGraph.addVertex(autoStateInx);
		}
		
		for (State autoState : autoStateSet) {
			autoStateInx = extractInteger(autoState.toString());
			
			for (Transition transition : autoState.getTransitions()) {
				autoStateInx_ = extractInteger(transition.getDest().toString());
				
				for (int event = (int)transition.getMin(); event <= (int)transition.getMax(); event++) {
					AutomatonEdge edge = new AutomatonEdge(Character.toString((char)event));
					taskAutomatonGraph.addEdge(autoStateInx, autoStateInx_, edge);
				}
			}
		}
		
		/* * Add initial vertex for automaton graph**/
		this.initialState = extractInteger(taskAutomaton.getInitialState().toString());
		
		/* * Add accepted vertices for automaton graph**/
		for (State state : taskAutomaton.getAcceptStates())
			this.acceptedStates.add(extractInteger(state.toString()));
		
		this.AutomatonGraph = taskAutomatonGraph;
		
		this.eventSet = charSet2String(taskAutomaton);
		
		System.out.println("The converted task automaton init state:" + this.initialState);
		System.out.println("The converted task automaton graph:" + this.AutomatonGraph);
	}
	
	/** Task automaton described in graph**********/
	public DirectedPseudograph<String, AutomatonEdge> GraphTaskAut(Automaton taskAutomaton) {
		//System.out.println("Original automaton" + taskAutomaton);
		
		DirectedPseudograph<String,AutomatonEdge> taskAutomatonGraph = new DirectedPseudograph<>(
				AutomatonEdge.class);
//		String autStateNum, autNextStateNum;
//		State autState;
//		boolean notRepeated;
//		State initState = taskAutomaton.getInitialState();
//		
//		Queue<State> openList = new LinkedList<>();
//		openList.add(initState);
		
//		autStateNum = extractInteger(initState.toString());
//		taskAutomatonGraph.addVertex(autStateNum);

		
//		/* * Convert automaton states into graph vertices**/
//		for (State autoState : ) {
//			autState = openList.poll();
//			//autoStateNum = extractInteger(autoState.toString());
//			
//			for (Transition transition : autoState.getTransitions()) {
//				openList.add(transition.getDest());
//				autNextStateNum = extractInteger(transition.getDest().toString());
//				taskAutomatonGraph.addVertex(autNextStateNum);
//				
//				for (int event = (int)transition.getMin(); event <= (int)transition.getMax(); event++) {
//					
//					notRepeated = true;
//					Set<AutomatonEdge> oldEdgeSet = taskAutomatonGraph.getAllEdges(autStateNum, autNextStateNum);
//					
//					if (!oldEdgeSet.isEmpty())
//						for (AutomatonEdge oldEdge : oldEdgeSet)
//							if (oldEdge.Event.equals(Character.toString((char)event))) {
//								notRepeated = false;
//								break;
//							}
//					
//					if (notRepeated == true) {
//						AutomatonEdge edge = new AutomatonEdge(Character.toString((char)event));
//						taskAutomatonGraph.addEdge(autStateNum, autNextStateNum, edge);
//					}
//				}
//			}
//		}
		
		//System.out.println("The converted automaton graph" + taskAutomatonGraph);
		
		return taskAutomatonGraph;
	}
	
	/** Extract first occurrence of integer from string*****/
	public static String extractInteger(String state) {
		Pattern p = Pattern.compile("([\\d]+)");
		Matcher m = p.matcher(state);
		if (m.find()) {
			return m.group(0);
		}
		
		return null;
	}
	
	/** String format of event set of automaton*/
	public static Set<String> charSet2String(Automaton automaton) {
		Set<String> eventSetOfAutInString = new HashSet<>();
		Set<Character> eventSetOfAut = decompAut.getEventSet(automaton);
		
		for (Character event: eventSetOfAut)
			eventSetOfAutInString.add(Character.toString(event));
		
		return eventSetOfAutInString;
	}
	
	/** String format of event set */
	public static ArrayList<String> charSet2StringList(Set<Character> eventSet) {
		ArrayList<String> eventSetOfAutInString = new ArrayList<>();
		
		for (Character event: eventSet)
			eventSetOfAutInString.add(Character.toString(event));
		
		return eventSetOfAutInString;
	}
}

/** Define the labeled edge, allowing same name */
class AutomatonEdge 
	extends
	DefaultEdge	{
	
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	public String Event;
	
	public AutomatonEdge(String automatonEvent) {
		this.Event = automatonEvent;
	}
}