package taskPlanning2;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;


public class RobotConfig {
	public TaskAutomaton automaton;
	public Set<Robot> robotSet;
	
	public RobotConfig(TaskAutomaton automaton, Set<Robot> autRobotSet) {
		this.automaton = automaton;
		this.robotSet = autRobotSet;
	}
	
	/** Maximum robot configuration set: */
	/** given a set of robots and a task automaton, output the set of all the robots
	 ** that can perform the events of task automaton */
	public static Set<Robot> maxConfigOfRobotset(TaskAutomaton automaton, Set<Robot> providedRobots) {
		
		Set<Robot> maximumRobotSet = new HashSet<>();
		
		for (Robot robot : providedRobots)
			for (String robotEvent : robot.eventList())
				if (automaton.eventSet.contains(robotEvent))
				{
					maximumRobotSet.add(robot);
					break;
				}
		
		return maximumRobotSet;
	}
	
	/** List of capable robot configuration set: */
	/** given a set of robots and a task automaton, output all the sets of robot sets
	 ** that can perform the events of task automaton */
	public static List<Set<Robot>> qualifiedRobotsetList(TaskAutomaton automaton, Set<Robot> providedRobots) {
		
		List<Set<Robot>> RobotSetList = new ArrayList<Set<Robot>>();
		List<String> subsetSumOfRobotAction = new ArrayList<>();
		
		Set<Robot> maxRobotSet = maxConfigOfRobotset(automaton, providedRobots);
		
		/** truncate the minimum robot set**/
		for (Set<Robot> robotSet : powerSet(maxRobotSet)) {
			
			subsetSumOfRobotAction.clear();
			for (Robot robot : robotSet)
				subsetSumOfRobotAction.addAll(robot.eventList());
			
			if (subsetSumOfRobotAction.containsAll(automaton.eventSet)) {
				if (automaton.eventSet.contains("g")) {
					if (robotSet.size()>1)
						RobotSetList.add(robotSet);
				}
				else
					RobotSetList.add(robotSet);
			}
		}
		
		Collections.sort(RobotSetList, new Comparator<Set<Robot>>(){
			@Override
			public int compare(Set<Robot> a, Set<Robot> b) {
				return a.size() - b.size();// Comparison
				}
			});
		
		return RobotSetList;
	}
	
	/** List of minimum robot configuration set: */
	/** given a list of robot configurations, output all the sets of robot sets
	 ** that can perform the events of task automaton */
	public static Set<Set<Robot>> minRobotConfigSet(TaskAutomaton automaton, Set<Robot> providedRobots, 
			ArrayList<String> CoopEventSet) {
		
		Set<Set<Robot>> minRobotSetList = new HashSet<Set<Robot>>();
		int MINIMALSIZE;
//		int breakPoint = 0;
		
		List<Set<Robot>> robotSetList = qualifiedRobotsetList(automaton, providedRobots);
		MINIMALSIZE = robotSetList.get(0).size();
		
		for (Set<Robot> robotSet : robotSetList) {
//			RobotTransitionSystem composedSubTS = RobotTransitionSystem.transitionSystemComposition(robotSet,
//				CoopEventSet);
			for (Robot _robot : robotSet)
				System.out.print(_robot.getID() + ",");
			
			System.out.println("next pair");
			
			if (robotSet.size() <= MINIMALSIZE)
				minRobotSetList.add(robotSet);
			else
				break;
			
//			TaskAllocAutomaton subtaskAllocAutomatonT = new TaskAllocAutomaton(composedSubTS, automaton, 
//				providedRobots);
//			breakPoint += 1;
			
//			if (!subtaskAllocAutomatonT.acceptStates.isEmpty()) {
//				MINIMALSIZE = robotSet.size();
//				minRobotSetList.add(robotSet);
//				
////				for (Robot _robot : robotSet)
////					System.out.print(_robot.getID());
////				
////				System.out.println("Initial state:" + subtaskAllocAutomatonT.initState.TsState + "," + 
////						subtaskAllocAutomatonT.initState.AutState);
////				System.out.println("Final accepted states:");
////				for (VtxOfProdAut finalAcceptState: subtaskAllocAutomatonT.acceptStates)
////					System.out.println(finalAcceptState.TsState + "," + finalAcceptState.AutState);
////				
////				System.out.println("Task automaton:" + automaton.AutomatonGraph);
//				
//				break;
//			}
		}
		
//		for (int index = breakPoint; index < robotSetList.size(); index++)
//			if (robotSetList.get(index).size() <= MINIMALSIZE) {
//				
//				for (Robot _robot : robotSetList.get(index))
//					System.out.print(_robot.getID() + ",");
//				
//				RobotTransitionSystem composedSubTS = RobotTransitionSystem.transitionSystemComposition(
//						robotSetList.get(index), CoopEventSet);
//				TaskAllocAutomaton subtaskAllocAutomatonT = new TaskAllocAutomaton(composedSubTS, automaton, 
//						providedRobots);
//				if (!subtaskAllocAutomatonT.acceptStates.isEmpty()) {
//					minRobotSetList.add(robotSetList.get(index));
//				
//////					for (Robot _robot : robotSetList.get(index))
//////						System.out.print(_robot.getID());
//////					
//////					System.out.println("Initial state:" + subtaskAllocAutomatonT.initState.TsState + "," + 
//////							subtaskAllocAutomatonT.initState.AutState);
//////					System.out.println("Final accepted states:");
//////					for (VtxOfProdAut finalAcceptState: subtaskAllocAutomatonT.acceptStates)
//////						System.out.println(finalAcceptState.TsState + "," + finalAcceptState.AutState);
//////					
//////					System.out.println("Task automaton:" + automaton.AutomatonGraph);
//				}
//			}
		
		//System.out.println("robot minimal configuration size:" + minRobotSetList.size());
		if (automaton.eventSet.contains("g"))
			for (Set<Robot> robotset : minRobotSetList) {
				List<Robot> robotSubList = new ArrayList<>();
				robotSubList.addAll(robotset);
				if (robotSubList.get(0).getID() == 2 && robotSubList.get(1).getID() == 3
						|| robotSubList.get(0).getID() == 3 && robotSubList.get(1).getID() == 2) {
					minRobotSetList.remove(robotset);
					break;
				}
			}
			
		return minRobotSetList;
	}
	
	/***powerSet generation******/
	private static Set<Set<Robot>> powerSet(Set<Robot> maxRobotSet) {
		Set<Set<Robot>> sets = new HashSet<Set<Robot>>();
		
		if (maxRobotSet.isEmpty()) {
			sets.add(new HashSet<Robot>());
			return sets;
		}
		
		List<Robot> list = new ArrayList<>(maxRobotSet);
		Robot head = list.get(0);
		Set<Robot> rest = new HashSet<>(list.subList(1, list.size()));
		for (Set<Robot> set : powerSet(rest)) {
			Set<Robot> newSet = new HashSet<>();
			newSet.add(head);
			newSet.addAll(set);
			sets.add(newSet);
			sets.add(set);
		}
		
		return sets;
	}
	
}
