package taskPlanning2;

import java.io.BufferedWriter;
import java.io.FileWriter;
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
import dk.brics.automaton.RegExp;
import dk.brics.automaton.SubAutomataExtract;
import dk.brics.automaton.decompAut;

public class Example {
	
	//examples 6 of journal paper
	public static List<RobotAction> robot1ActionList = new ArrayList<>(Arrays.asList(
			new RobotAction(0, "e", 0.3), new RobotAction(0, "a", 1.0), new RobotAction(0, "b", 1.0)));
	
	public static List<RobotAction> robot2ActionList = new ArrayList<>(Arrays.asList(
			new RobotAction(1, "e", 0.3), new RobotAction(1, "a", 1.2), new RobotAction(1, "c", 0.8)));
	
	public static List<RobotAction> robot3ActionList = new ArrayList<>(Arrays.asList(
			new RobotAction(2, "e", 0.3), new RobotAction(2, "a", 0.7), new RobotAction(2, "d", 1.3)));
	
//	//examples 7 of journal paper
//	public static List<RobotAction> robot2TActionList = new ArrayList<>(Arrays.asList(
//			new RobotAction(1, "e", 0.3), new RobotAction(1, "c", 1.2), new RobotAction(1, "c", 0.8)));
	
	/** Individual Transition System*/
	/** Transition System 1*/
	private static Graph<ArrayList<String>, TSEdge> transitionSystem1 = new DefaultDirectedGraph<>(TSEdge.class);
	static
	{
		ArrayList<String> node0 = new ArrayList<String>(Arrays.asList("e"));
		ArrayList<String> node1 = new ArrayList<String>(Arrays.asList("a"));
		ArrayList<String> node2 = new ArrayList<String>(Arrays.asList("b"));
	    
		transitionSystem1.addVertex(node0);
		transitionSystem1.addVertex(node1);
		transitionSystem1.addVertex(node2);
		
		
		transitionSystem1.addEdge(node0, node0, new TSEdge("e"));
		transitionSystem1.addEdge(node0, node1, new TSEdge("a1"));
		transitionSystem1.addEdge(node0, node2, new TSEdge("b1"));
		transitionSystem1.addEdge(node1, node2, new TSEdge("b1"));
		transitionSystem1.addEdge(node2, node0, new TSEdge("e"));
	};
	private static ArrayList<String> initNode1 = new ArrayList<String>(Arrays.asList("e"));
//	private static List<Integer> idList1 = new ArrayList<>(Arrays.asList(0));
//	public static RobotTransitionSystem TsOfR1 = new RobotTransitionSystem(idList1, initNode1, transitionSystem1);
	
	/** Individual Transition System*/
	/** Transition System 2*/
	public static Graph<ArrayList<String>, TSEdge> transitionSystem2 = new DefaultDirectedGraph<>(TSEdge.class);
	static
	{
		ArrayList<String> node0 = new ArrayList<String>(Arrays.asList("e"));
		ArrayList<String> node1 = new ArrayList<String>(Arrays.asList("a"));
		ArrayList<String> node2 = new ArrayList<String>(Arrays.asList("c"));
	    
		transitionSystem2.addVertex(node0);
		transitionSystem2.addVertex(node1);
		transitionSystem2.addVertex(node2);
		
		transitionSystem2.addEdge(node0, node0, new TSEdge("e"));
		transitionSystem2.addEdge(node0, node1, new TSEdge("a2"));
		transitionSystem2.addEdge(node0, node2, new TSEdge("c2"));
		transitionSystem2.addEdge(node1, node2, new TSEdge("c2"));
		transitionSystem2.addEdge(node2, node0, new TSEdge("e"));
	};
	private static ArrayList<String> initNode2 = new ArrayList<String>(Arrays.asList("e"));
//	private static List<Integer> idList2 = new ArrayList<>(Arrays.asList(1));
//	public static RobotTransitionSystem TsOfR2 = new RobotTransitionSystem(idList2, initNode2, transitionSystem2);
	
//	/** Individual Transition System*/
//	/** Transition System 2T*/
//	public static Graph<ArrayList<String>, TSEdge> transitionSystem2T = new DefaultDirectedGraph<>(TSEdge.class);
//	static
//	{
//		ArrayList<String> node0 = new ArrayList<String>(Arrays.asList("e"));
//		ArrayList<String> node1 = new ArrayList<String>(Arrays.asList("c"));
//		ArrayList<String> node2 = new ArrayList<String>(Arrays.asList("c"));
//	    
//		transitionSystem2.addVertex(node0);
//		transitionSystem2.addVertex(node1);
//		transitionSystem2.addVertex(node2);
//		
//		transitionSystem2.addEdge(node0, node0, new TSEdge("e"));
//		transitionSystem2.addEdge(node0, node1, new TSEdge("e"));
//		transitionSystem2.addEdge(node0, node2, new TSEdge("e"));
//		transitionSystem2.addEdge(node1, node2, new TSEdge("c"));
//		transitionSystem2.addEdge(node2, node0, new TSEdge("c"));
//	};
//	//private static ArrayList<String> initNode2T = new ArrayList<String>(Arrays.asList("e"));
	
	/** Individual Transition System*/
	/** Transition System 3*/
	private static Graph<ArrayList<String>, TSEdge> transitionSystem3 = new DefaultDirectedGraph<>(TSEdge.class);
	static
	{
		ArrayList<String> node0 = new ArrayList<String>(Arrays.asList("e"));
		ArrayList<String> node1 = new ArrayList<String>(Arrays.asList("a"));
		ArrayList<String> node2 = new ArrayList<String>(Arrays.asList("d"));
		
		transitionSystem3.addVertex(node0);
		transitionSystem3.addVertex(node1);
		transitionSystem3.addVertex(node2);
		
		transitionSystem3.addEdge(node0, node0, new TSEdge("e"));
		transitionSystem3.addEdge(node0, node1, new TSEdge("a3"));
		transitionSystem3.addEdge(node0, node2, new TSEdge("d3"));
		transitionSystem3.addEdge(node1, node2, new TSEdge("d3"));
		transitionSystem3.addEdge(node2, node0, new TSEdge("e"));
	};
	private static ArrayList<String> initNode3 = new ArrayList<String>(Arrays.asList("e"));
//	private static List<Integer> idList3 = new ArrayList<>(Arrays.asList(2));
//	public static RobotTransitionSystem TsOfR3 = new RobotTransitionSystem(idList3, initNode3, transitionSystem3);
	
	/** Main function for trial**/
	public static void main(String args[]) throws ExportException, IOException {
		/** 0. Definition of robots and their transition systems **/
		//For example use
		Robot robot1 = new Robot(robot1ActionList, transitionSystem1, initNode1);
		Robot robot2 = new Robot(robot2ActionList, transitionSystem2, initNode2);
		Robot robot3 = new Robot(robot3ActionList, transitionSystem3, initNode3);
		Set<Robot> providedRobots = new HashSet<>(Arrays.asList(robot1, robot2, robot3));
		
		/**1. give a regular expression, output automata*/
		/** Declare cooperative events:
		 * choose one to work with**/
		RegExp regExpr = new RegExp("e(abd|b(ad|da))");//example in example4
		@SuppressWarnings("serial")
		Set<Character> CoopEventSet = new HashSet<>() {{
			add('e');
		}};
		
		Automaton globalTaskAutomaton = regExpr.toAutomaton();
		
		/** 3. Obtain sub-automaton, of which all paths have common events **/
		/** Generate parallel decompositions for each sub-automaton**/
		Set<Automaton> SubautomatonSet= SubAutomataExtract.commonEventsAut(globalTaskAutomaton);
		//System.out.print(AutomatonSet);
		List<List<Automaton>> decompositionSet = new ArrayList<List<Automaton>>();
		for (Automaton automaton : SubautomatonSet)
			decompositionSet.add(decompAut.paraDecompC(automaton, CoopEventSet));
		
		List<List<TaskAutomaton>> decompositionSetConvert = new ArrayList<List<TaskAutomaton>>();
		for (List<Automaton> decompositions : decompositionSet) {
			
			List<TaskAutomaton> decompositionsConvert = new ArrayList<TaskAutomaton>();
			
			for (Automaton singleAutomaton : decompositions) {
//				System.out.println(singleAutomaton);
				decompositionsConvert.add(new TaskAutomaton(singleAutomaton));
			}
			
			decompositionSetConvert.add(decompositionsConvert);
		}
		
		/////////////////////////////////////////////////////////////////////////////////////////////////////
		/** 4. Obtain automaton decompositions - robots configuration **/
		List<RobotSetSwitch> robotSwitchConfigs = new ArrayList<>();
		for (int index = 0; index < decompositionSetConvert.size(); index++)
			robotSwitchConfigs.add(new RobotSetSwitch(decompositionSetConvert.get(index), CoopEventSet, 
					providedRobots));
//		System.out.println(robotSwitchConfigs.get(0).autRobotsPair.size());
		
		/** 5. For each decomposition set, take product automaton with transition system to 
		/* get the subtask allocation automaton **/
		List<List<TaskAllocAutomaton>> ListOfsubtaskAllocAutomatonLists = new ArrayList<>();
		List<List<GraphPath<VtxOfProdAut,EdgOfProdAut>>> ListOfShortestPathOfSubAllocationLists=new ArrayList<>();
		
		for (RobotSetSwitch robotSetConfig : robotSwitchConfigs) {
			List<TaskAllocAutomaton> subtaskAllocAutomatonList = new ArrayList<TaskAllocAutomaton>();
			List<GraphPath<VtxOfProdAut, EdgOfProdAut>> shortestPathList = new ArrayList<>();
			
			for (RobotConfig singleConfig : robotSetConfig.autRobotsPair) {
				System.out.println("---------------------------");
				System.out.println("Subtask automaton is:" + singleConfig.automaton.AutomatonGraph.vertexSet());
				
				RobotTransitionSystem composedTS = RobotTransitionSystem.transitionSystemComposition(singleConfig.robotSet, 
						TaskAutomaton.charSet2StringList(CoopEventSet));
				
				TaskAllocAutomaton subtaskAllocAutomatonT = new TaskAllocAutomaton(composedTS, 
						singleConfig.automaton, providedRobots);
				subtaskAllocAutomatonList.add(subtaskAllocAutomatonT);
				
				System.out.println("Subtask allocation automaton vertex set is:");
				for (VtxOfProdAut vortex : subtaskAllocAutomatonT.taskAllocAutGraph.vertexSet())
					System.out.println(vortex.TsState + "," + vortex.AutState + "," + vortex.robotState);
				String fileName = "subtaskAllocAutomaton" + ListOfsubtaskAllocAutomatonLists.size()
					+ subtaskAllocAutomatonList.size();
				TaskAllocAutomaton.renderProdAutGraph(subtaskAllocAutomatonT.taskAllocAutGraph, fileName);
				//break;
				
				/** 5.2 optimal path search**/
				GraphPath<VtxOfProdAut, EdgOfProdAut> shortestPath = subtaskAllocAutomatonT.lowestCostPath();
				shortestPathList.add(shortestPath);
				
				String shortestPathString = new String();
				for (EdgOfProdAut edge : shortestPath.getEdgeList()) {
					System.out.print(edge.action + "," + edge.robotID + ";");
					shortestPathString += edge.action + "," + edge.robotID + ";";
					shortestPathString += "\n";
				}
		
				String fileNameOfPath = "opt" + fileName;
				BufferedWriter pathWriter = new BufferedWriter(new FileWriter(fileNameOfPath));
				pathWriter.write(shortestPathString);
				pathWriter.close();
			}
			
			ListOfShortestPathOfSubAllocationLists.add(shortestPathList);
			ListOfsubtaskAllocAutomatonLists.add(subtaskAllocAutomatonList);
		}
		
		double lowestCosts = 1000.0;
		List<GraphPath<VtxOfProdAut, EdgOfProdAut>> optimalShorestPath = new ArrayList<>();
		
		for (int index = 0; index < ListOfShortestPathOfSubAllocationLists.size(); index++) {
			
			double sumCostOfPath = 0;
			
			for (GraphPath<VtxOfProdAut, EdgOfProdAut> shortestPath : 
				ListOfShortestPathOfSubAllocationLists.get(index))
				sumCostOfPath += shortestPath.getWeight();
			
			if (sumCostOfPath < lowestCosts) {
				lowestCosts = sumCostOfPath;
				optimalShorestPath = ListOfShortestPathOfSubAllocationLists.get(index);
			}
		}
		
		String fileNameOfShortestPath = "FinalOptimalPath";
		String shortestOptPathString = new String();
		for (GraphPath<VtxOfProdAut, EdgOfProdAut> path : optimalShorestPath) {
			for (EdgOfProdAut edge : path.getEdgeList()) {
				System.out.print(edge.action + "," + edge.robotID + ";");
				shortestOptPathString += edge.action + "," + edge.robotID + ";";
				shortestOptPathString += "\n";
			}
			shortestOptPathString += "\n";
		}
		BufferedWriter pathWriter = new BufferedWriter(new FileWriter(fileNameOfShortestPath));
		pathWriter.write(shortestOptPathString);
		pathWriter.close();
		
	}
}
