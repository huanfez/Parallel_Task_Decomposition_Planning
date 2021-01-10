package taskPlanning2;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.io.ExportException;

import dk.brics.automaton.Automaton;
import dk.brics.automaton.BasicOperations;
import dk.brics.automaton.RegExp;
import dk.brics.automaton.SubAutomataExtract;
import dk.brics.automaton.decompAut;

/** Experiment set for journal paper*/
public class Experiment_case2 {
	
	/** Robot Action Set */
	public static List<RobotAction> robot0ActionList = new ArrayList<>(Arrays.asList(
			new RobotAction(0, "a", 5.0), new RobotAction(0, "b", 10.0), new RobotAction(0, "c", 10.0), 
			new RobotAction(0, "g", 3.0)));
		
	public static List<RobotAction> robot1ActionList = new ArrayList<>(Arrays.asList(
			new RobotAction(1, "a", 5.0), new RobotAction(1, "b", 10.0), new RobotAction(1, "c", 10.0), 
			new RobotAction(1, "g", 5.0)));
		
	public static List<RobotAction> robot2ActionList = new ArrayList<>(Arrays.asList(
			new RobotAction(2, "a", 7.0), new RobotAction(2, "b", 12.0), new RobotAction(2, "c", 12.0), 
			new RobotAction(2, "d", 12.0), new RobotAction(2, "e", 12.0), new RobotAction(2, "f", 12.0),
			new RobotAction(2, "g", 5.0)));
	
	public static List<RobotAction> robot3ActionList = new ArrayList<>(Arrays.asList(
			new RobotAction(3, "a", 7.0), new RobotAction(3, "b", 12.0), new RobotAction(3, "c", 12.0), 
			new RobotAction(3, "d", 12.0), new RobotAction(3, "e", 12.0), new RobotAction(3, "f", 12.0),
			new RobotAction(3, "g", 5.0)));
	

	/** Each Robot's Transition System Graph */
	/** Graph 1*/
	private static Graph<ArrayList<String>, TSEdge> ts0Graph = new DefaultDirectedGraph<>(TSEdge.class);
	static
	{
		ArrayList<String> nodea = new ArrayList<String>(Arrays.asList("a"));
		ArrayList<String> nodeb = new ArrayList<String>(Arrays.asList("b"));
		ArrayList<String> nodec = new ArrayList<String>(Arrays.asList("c"));
		ArrayList<String> nodeg = new ArrayList<String>(Arrays.asList("g"));
		
		ts0Graph.addVertex(nodea);
		ts0Graph.addVertex(nodeb);
		ts0Graph.addVertex(nodec);
		ts0Graph.addVertex(nodeg);
		
		ts0Graph.addEdge(nodea, nodeb, new TSEdge("a"));
		ts0Graph.addEdge(nodea, nodec, new TSEdge("a"));
		ts0Graph.addEdge(nodea, nodeg, new TSEdge("a"));
		
		ts0Graph.addEdge(nodeb, nodea, new TSEdge("b"));
		ts0Graph.addEdge(nodeb, nodec, new TSEdge("b"));
		ts0Graph.addEdge(nodeb, nodeg, new TSEdge("b"));
		
		ts0Graph.addEdge(nodec, nodea, new TSEdge("c"));
		ts0Graph.addEdge(nodec, nodeb, new TSEdge("c"));
		ts0Graph.addEdge(nodec, nodeg, new TSEdge("c"));
		
		ts0Graph.addEdge(nodeg, nodea, new TSEdge("g"));
		ts0Graph.addEdge(nodeg, nodeb, new TSEdge("g"));
		ts0Graph.addEdge(nodeg, nodec, new TSEdge("g"));
	};
	/** Robot initial state of transition system*/
	private static ArrayList<String> initNode0 = new ArrayList<String>(Arrays.asList("a"));
	
	/** Graph 1*/
	private static Graph<ArrayList<String>, TSEdge> ts1Graph = new DefaultDirectedGraph<>(TSEdge.class);
	static
	{
		ArrayList<String> nodea = new ArrayList<String>(Arrays.asList("a"));
		ArrayList<String> nodeb = new ArrayList<String>(Arrays.asList("b"));
		ArrayList<String> nodec = new ArrayList<String>(Arrays.asList("c"));
		ArrayList<String> nodeg = new ArrayList<String>(Arrays.asList("g"));
		
		ts1Graph.addVertex(nodea);
		ts1Graph.addVertex(nodeb);
		ts1Graph.addVertex(nodec);
		ts1Graph.addVertex(nodeg);
		
		ts1Graph.addEdge(nodea, nodeb, new TSEdge("a"));
		ts1Graph.addEdge(nodea, nodec, new TSEdge("a"));
		ts1Graph.addEdge(nodea, nodeg, new TSEdge("a"));
		
		ts1Graph.addEdge(nodeb, nodea, new TSEdge("b"));
		ts1Graph.addEdge(nodeb, nodec, new TSEdge("b"));
		ts1Graph.addEdge(nodeb, nodeg, new TSEdge("b"));
		
		ts1Graph.addEdge(nodec, nodea, new TSEdge("c"));
		ts1Graph.addEdge(nodec, nodeb, new TSEdge("c"));
		ts1Graph.addEdge(nodec, nodeg, new TSEdge("c"));
		
		ts1Graph.addEdge(nodeg, nodea, new TSEdge("g"));
		ts1Graph.addEdge(nodeg, nodeb, new TSEdge("g"));
		ts1Graph.addEdge(nodeg, nodec, new TSEdge("g"));
	};
	private static ArrayList<String> initNode1 = new ArrayList<String>(Arrays.asList("a"));
	
	/** Transition System Graph 2*/
	private static Graph<ArrayList<String>, TSEdge> ts2Graph = new DefaultDirectedGraph<>(TSEdge.class);
	static
	{
		ArrayList<String> nodea = new ArrayList<String>(Arrays.asList("a"));
		ArrayList<String> nodeb = new ArrayList<String>(Arrays.asList("b"));
		ArrayList<String> nodec = new ArrayList<String>(Arrays.asList("c"));
		ArrayList<String> noded = new ArrayList<String>(Arrays.asList("d"));
		ArrayList<String> nodee = new ArrayList<String>(Arrays.asList("e"));
		ArrayList<String> nodef = new ArrayList<String>(Arrays.asList("f"));
		ArrayList<String> nodeg = new ArrayList<String>(Arrays.asList("g"));
		
		ts2Graph.addVertex(nodea);
		ts2Graph.addVertex(nodeb);
		ts2Graph.addVertex(nodec);
		ts2Graph.addVertex(noded);
		ts2Graph.addVertex(nodee);
		ts2Graph.addVertex(nodef);
		ts2Graph.addVertex(nodeg);
		
		ts2Graph.addEdge(nodea, nodeb, new TSEdge("a"));
		ts2Graph.addEdge(nodea, nodec, new TSEdge("a"));
		
		ts2Graph.addEdge(nodeb, nodea, new TSEdge("b"));
		ts2Graph.addEdge(nodeb, nodec, new TSEdge("b"));
		ts2Graph.addEdge(nodeb, noded, new TSEdge("b"));
		ts2Graph.addEdge(nodeb, nodee, new TSEdge("b"));
		
		ts2Graph.addEdge(nodec, nodea, new TSEdge("c"));
		ts2Graph.addEdge(nodec, nodeb, new TSEdge("c"));
		ts2Graph.addEdge(nodec, nodee, new TSEdge("c"));
		ts2Graph.addEdge(nodec, nodef, new TSEdge("c"));
		
		ts2Graph.addEdge(noded, nodeb, new TSEdge("d"));
		ts2Graph.addEdge(noded, nodee, new TSEdge("d"));
		ts2Graph.addEdge(noded, nodeg, new TSEdge("d"));
		
		ts2Graph.addEdge(nodee, nodeb, new TSEdge("e"));
		ts2Graph.addEdge(nodee, nodec, new TSEdge("e"));
		ts2Graph.addEdge(nodee, noded, new TSEdge("e"));
		ts2Graph.addEdge(nodee, nodef, new TSEdge("e"));
		ts2Graph.addEdge(nodee, nodeg, new TSEdge("e"));
		
		ts2Graph.addEdge(nodef, nodec, new TSEdge("f"));
		ts2Graph.addEdge(nodef, nodee, new TSEdge("f"));
		ts2Graph.addEdge(nodef, nodeg, new TSEdge("f"));
		
		ts2Graph.addEdge(nodeg, noded, new TSEdge("g"));
		ts2Graph.addEdge(nodeg, nodee, new TSEdge("g"));
		ts2Graph.addEdge(nodeg, nodef, new TSEdge("g"));
	};
	/** Robot initial state of transition system*/
	private static ArrayList<String> initNode2 = new ArrayList<String>(Arrays.asList("b"));
	
	private static Graph<ArrayList<String>, TSEdge> ts3Graph = new DefaultDirectedGraph<>(TSEdge.class);
	static
	{
		ArrayList<String> nodea = new ArrayList<String>(Arrays.asList("a"));
		ArrayList<String> nodeb = new ArrayList<String>(Arrays.asList("b"));
		ArrayList<String> nodec = new ArrayList<String>(Arrays.asList("c"));
		ArrayList<String> noded = new ArrayList<String>(Arrays.asList("d"));
		ArrayList<String> nodee = new ArrayList<String>(Arrays.asList("e"));
		ArrayList<String> nodef = new ArrayList<String>(Arrays.asList("f"));
		ArrayList<String> nodeg = new ArrayList<String>(Arrays.asList("g"));
		
		ts3Graph.addVertex(nodea);
		ts3Graph.addVertex(nodeb);
		ts3Graph.addVertex(nodec);
		ts3Graph.addVertex(noded);
		ts3Graph.addVertex(nodee);
		ts3Graph.addVertex(nodef);
		ts3Graph.addVertex(nodeg);
		
		ts3Graph.addEdge(nodea, nodeb, new TSEdge("a"));
		ts3Graph.addEdge(nodea, nodec, new TSEdge("a"));
		
		ts3Graph.addEdge(nodeb, nodea, new TSEdge("b"));
		ts3Graph.addEdge(nodeb, nodec, new TSEdge("b"));
		ts3Graph.addEdge(nodeb, noded, new TSEdge("b"));
		ts3Graph.addEdge(nodeb, nodee, new TSEdge("b"));
		
		ts3Graph.addEdge(nodec, nodea, new TSEdge("c"));
		ts3Graph.addEdge(nodec, nodeb, new TSEdge("c"));
		ts3Graph.addEdge(nodec, nodee, new TSEdge("c"));
		ts3Graph.addEdge(nodec, nodef, new TSEdge("c"));
		
		ts3Graph.addEdge(noded, nodeb, new TSEdge("d"));
		ts3Graph.addEdge(noded, nodee, new TSEdge("d"));
		ts3Graph.addEdge(noded, nodeg, new TSEdge("d"));
		
		ts3Graph.addEdge(nodee, nodeb, new TSEdge("e"));
		ts3Graph.addEdge(nodee, nodec, new TSEdge("e"));
		ts3Graph.addEdge(nodee, noded, new TSEdge("e"));
		ts3Graph.addEdge(nodee, nodef, new TSEdge("e"));
		ts3Graph.addEdge(nodee, nodeg, new TSEdge("e"));
		
		ts3Graph.addEdge(nodef, nodec, new TSEdge("f"));
		ts3Graph.addEdge(nodef, nodee, new TSEdge("f"));
		ts3Graph.addEdge(nodef, nodeg, new TSEdge("f"));
		
		ts3Graph.addEdge(nodeg, noded, new TSEdge("g"));
		ts3Graph.addEdge(nodeg, nodee, new TSEdge("g"));
		ts3Graph.addEdge(nodeg, nodef, new TSEdge("g"));
	};
	private static ArrayList<String> initNode3 = new ArrayList<String>(Arrays.asList("e"));
	
	/** Main function **/
	/** ideal input: 
	 *  (1. Automaton(R.E.) described task specification, 
	 *  2. Cooperative event set, 
	 *  3. Initial State of Robot) */
	public static void main(String args[]) throws ExportException, IOException {
		
		////////////////////////////Initial Decomposition and Allocation///////////////////////////////
		/** 0. Definition of robots and their transition systems **/		
		Robot robot0 = new Robot(robot0ActionList, ts0Graph, initNode0);
		Robot robot1 = new Robot(robot1ActionList, ts1Graph, initNode1);
		Robot robot2 = new Robot(robot2ActionList, ts2Graph, initNode2);
		Robot robot3 = new Robot(robot3ActionList, ts3Graph, initNode3);
		Set<Robot> providedRobots = new HashSet<>(Arrays.asList(robot0,robot1,robot2,robot3));
		
		/** 1.1 give a set of regular expressions, output automata*/
		RegExp regExpr_1 = new RegExp("abc((d((ef)|(fe)))|(e((df)|(fd)))|(f((de)|(ed))))g");
		
		Set<Automaton> originalAutomata = new HashSet<>() {/**
			 * 
			 */
			private static final long serialVersionUID = 1L;

		{
			add(regExpr_1.toAutomaton());
//			add(regExpr_2.toAutomaton());
//			add(regExpr_3.toAutomaton());
//			add(regExpr_4.toAutomaton());
		}};
		
		Automaton globalTaskAutomaton = BasicOperations.union(originalAutomata);
		
		/** 1.2 Declare cooperative events **/
		Set<Character> CoopEventSet = new HashSet<>() {/**
			 * 
			 */
			private static final long serialVersionUID = 1L;

		{
			add('g');
		}};
		
		/** 2.1. Obtain sub-automaton, of which all paths have common events **/
		Set<Automaton> SubautomatonSet= SubAutomataExtract.commonEventsAut(globalTaskAutomaton);
		
		/** 2.2. Generate parallel decompositions for each sub-automaton**/
		List<List<Automaton>> decompositionSet = new ArrayList<List<Automaton>>();
		for (Automaton automaton : SubautomatonSet)
			decompositionSet.add(decompAut.paraDecompC(automaton, CoopEventSet));
		
		/** 2.3. Convert each decomposition into graph form**/
		List<List<TaskAutomaton>> decompositionSetConvert = new ArrayList<List<TaskAutomaton>>();
		for (List<Automaton> decompositions : decompositionSet) {
			
			List<TaskAutomaton> decompositionsConvert = new ArrayList<TaskAutomaton>();
			for (Automaton singleAutomaton : decompositions) {
				decompositionsConvert.add(new TaskAutomaton(singleAutomaton));
			}
			
			decompositionSetConvert.add(decompositionsConvert);
		}
		
		/////////////////////////////////////////////////////////////////////////////////////////////////////
		/** 3. Obtain automaton decompositions - robots configuration **/
		List<RobotSetSwitch> robotSwitchConfigs = new ArrayList<>();
		for (int index = 0; index < decompositionSetConvert.size(); index++)
			robotSwitchConfigs.add(new RobotSetSwitch(decompositionSetConvert.get(index), CoopEventSet, 
					providedRobots));
//		System.out.println(robotSwitchConfigs.get(0).autRobotsPair.size());
		
		/** 4. For each decomposition set, take product automaton with transition system to 
		/* get the subtask allocation automaton **/
		List<List<TaskAllocAutomaton>> ListOfsubtaskAllocAutomatonLists = new ArrayList<>();
		List<List<GraphPath<VtxOfProdAut,EdgOfProdAut>>> ListOfShortestPathOfSubAllocationLists=new ArrayList<>();
		
		for (RobotSetSwitch robotSetConfig : robotSwitchConfigs) {
			List<TaskAllocAutomaton> subtaskAllocAutomatonList = new ArrayList<TaskAllocAutomaton>();
			List<GraphPath<VtxOfProdAut, EdgOfProdAut>> shortestPathList = new ArrayList<>();
			
			for (RobotConfig singleConfig : robotSetConfig.autRobotsPair) {
				System.out.println("---------------------------");
				System.out.println("Subtask automaton is:" + singleConfig.automaton.AutomatonGraph.vertexSet());
				
				RobotTransitionSystem composedTS = RobotTransitionSystem.transitionSystemComposition(
						singleConfig.robotSet, TaskAutomaton.charSet2StringList(CoopEventSet));
				TaskAllocAutomaton subtaskAllocAutomatonT = new TaskAllocAutomaton(composedTS, 
						singleConfig.automaton, providedRobots);
				subtaskAllocAutomatonList.add(subtaskAllocAutomatonT);
				
				//System.out.println("Subtask allocation automaton vertex set is:");
				//for (VtxOfProdAut vortex : subtaskAllocAutomatonT.taskAllocAutGraph.vertexSet())
					//System.out.println(vortex.TsState + "," + vortex.AutState + "," + vortex.robotState + " ");
				String fileName = "subtaskAllocAutomaton" + ListOfsubtaskAllocAutomatonLists.size()
					+ subtaskAllocAutomatonList.size();
				TaskAllocAutomaton.renderProdAutGraph(subtaskAllocAutomatonT.taskAllocAutGraph, fileName);
				
				/** 4.2 optimal path search**/
				GraphPath<VtxOfProdAut, EdgOfProdAut> shortestPath = subtaskAllocAutomatonT.lowestCostPath();
				String fileNameOfPath = "opt" + fileName;				
				TaskAllocAutomaton.outputSuboptimalPath(shortestPath, fileNameOfPath);
				
				shortestPathList.add(shortestPath);
			}
			
			ListOfShortestPathOfSubAllocationLists.add(shortestPathList);
			ListOfsubtaskAllocAutomatonLists.add(subtaskAllocAutomatonList);
		}
		
		TaskAllocAutomaton.outputFinalOptimalPath(ListOfShortestPathOfSubAllocationLists);
		
	}

}
