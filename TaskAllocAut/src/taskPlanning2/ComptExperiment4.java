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
import dk.brics.automaton.ShuffleOperations;
import dk.brics.automaton.SubAutomataExtract;
import dk.brics.automaton.decompAut;

/** Experiment set for journal paper*/
public class ComptExperiment4 {
	
	/** Robot Action Set */		
	public static List<RobotAction> buildRobotActionList(int index){
		List<RobotAction> robotActionList = new ArrayList<>(Arrays.asList(
				new RobotAction(index, "a", 7.0), new RobotAction(index, "b", 12.0), new RobotAction(index, "c", 12.0), 
				new RobotAction(index, "d", 12.0), new RobotAction(index, "e", 12.0), new RobotAction(index, "f", 12.0),
				new RobotAction(index, "g", 5.0)));
		
		return robotActionList;
	}
	
	/** Each Robot's Transition System Graph */
	private static ArrayList<String> initNode = new ArrayList<String>(Arrays.asList("a"));
	private static Graph<ArrayList<String>, TSEdge> buildTsGraph(int index) {
		ArrayList<String> nodea = new ArrayList<String>(Arrays.asList("a"));
		ArrayList<String> nodeb = new ArrayList<String>(Arrays.asList("b"));
		ArrayList<String> nodec = new ArrayList<String>(Arrays.asList("c"));
		ArrayList<String> noded = new ArrayList<String>(Arrays.asList("d"));
		ArrayList<String> nodee = new ArrayList<String>(Arrays.asList("e"));
		ArrayList<String> nodef = new ArrayList<String>(Arrays.asList("f"));
		ArrayList<String> nodeg = new ArrayList<String>(Arrays.asList("g"));
		
		ArrayList<String> nodeh = new ArrayList<String>(Arrays.asList("h"));
		ArrayList<String> nodei = new ArrayList<String>(Arrays.asList("i"));
		ArrayList<String> nodej = new ArrayList<String>(Arrays.asList("j"));
		ArrayList<String> nodek = new ArrayList<String>(Arrays.asList("k"));
		ArrayList<String> nodel = new ArrayList<String>(Arrays.asList("l"));
		ArrayList<String> nodem = new ArrayList<String>(Arrays.asList("m"));
		ArrayList<String> noden = new ArrayList<String>(Arrays.asList("n"));
		
		ArrayList<String> nodeo = new ArrayList<String>(Arrays.asList("o"));
		ArrayList<String> nodep = new ArrayList<String>(Arrays.asList("p"));
		ArrayList<String> nodeq = new ArrayList<String>(Arrays.asList("q"));
		ArrayList<String> noder = new ArrayList<String>(Arrays.asList("r"));
		ArrayList<String> nodes = new ArrayList<String>(Arrays.asList("s"));
		
		ArrayList<String> nodet = new ArrayList<String>(Arrays.asList("t"));
		ArrayList<String> nodeu = new ArrayList<String>(Arrays.asList("u"));
		
		Graph<ArrayList<String>, TSEdge> tsGraph = new DefaultDirectedGraph<>(TSEdge.class);
		tsGraph.addVertex(nodea);
		tsGraph.addVertex(nodeb);
		tsGraph.addVertex(nodec);
		tsGraph.addVertex(noded);
		tsGraph.addVertex(nodee);
		tsGraph.addVertex(nodef);
		tsGraph.addVertex(nodeg);
		
		tsGraph.addVertex(nodeh);
		tsGraph.addVertex(nodei);
		tsGraph.addVertex(nodej);
		tsGraph.addVertex(nodek);
		tsGraph.addVertex(nodel);
		tsGraph.addVertex(nodem);
		tsGraph.addVertex(noden);
		
		tsGraph.addVertex(nodeo);
		tsGraph.addVertex(nodep);
		tsGraph.addVertex(nodeq);
		tsGraph.addVertex(noder);
		tsGraph.addVertex(nodes);
		
		tsGraph.addVertex(nodet);
		tsGraph.addVertex(nodeu);
		///////////////////////////////////////////////
		tsGraph.addEdge(nodea, nodeb, new TSEdge("a"));
		tsGraph.addEdge(nodea, nodec, new TSEdge("a"));
		tsGraph.addEdge(nodea, noded, new TSEdge("a"));
		tsGraph.addEdge(nodea, nodee, new TSEdge("a"));
		
		tsGraph.addEdge(nodeb, nodef, new TSEdge("b"));
		tsGraph.addEdge(nodeb, nodeg, new TSEdge("b"));
		tsGraph.addEdge(nodeb, nodeh, new TSEdge("b"));
		
		tsGraph.addEdge(nodec, nodei, new TSEdge("c"));
		tsGraph.addEdge(nodec, nodej, new TSEdge("c"));
		tsGraph.addEdge(nodec, nodek, new TSEdge("c"));
		
		tsGraph.addEdge(noded, nodel, new TSEdge("d"));
		tsGraph.addEdge(noded, nodem, new TSEdge("d"));
		tsGraph.addEdge(noded, noden, new TSEdge("d"));
		tsGraph.addEdge(noded, nodeo, new TSEdge("d"));
		
		tsGraph.addEdge(nodee, nodep, new TSEdge("e"));
		tsGraph.addEdge(nodee, nodeq, new TSEdge("e"));
		tsGraph.addEdge(nodee, noder, new TSEdge("e"));
		
		tsGraph.addEdge(nodef, nodei, new TSEdge("f"));
		tsGraph.addEdge(nodef, nodej, new TSEdge("f"));
		tsGraph.addEdge(nodef, nodek, new TSEdge("f"));
		
		tsGraph.addEdge(nodeg, nodel, new TSEdge("g"));
		tsGraph.addEdge(nodeg, nodem, new TSEdge("g"));
		tsGraph.addEdge(nodeg, noden, new TSEdge("g"));
		
		tsGraph.addEdge(nodeh, nodeo, new TSEdge("h"));
		tsGraph.addEdge(nodeh, nodep, new TSEdge("h"));
		tsGraph.addEdge(nodeh, nodeq, new TSEdge("h"));
		tsGraph.addEdge(nodeh, noder, new TSEdge("h"));
		
		tsGraph.addEdge(nodei, nodef, new TSEdge("i"));
		tsGraph.addEdge(nodei, nodeg, new TSEdge("i"));
		tsGraph.addEdge(nodei, nodeh, new TSEdge("i"));
		
		tsGraph.addEdge(nodej, nodel, new TSEdge("j"));
		tsGraph.addEdge(nodej, nodem, new TSEdge("j"));
		tsGraph.addEdge(nodej, noden, new TSEdge("j"));
		
		tsGraph.addEdge(nodek, nodes, new TSEdge("k"));
		
		tsGraph.addEdge(nodee, nodep, new TSEdge("e"));
		tsGraph.addEdge(nodee, nodeq, new TSEdge("e"));
		tsGraph.addEdge(nodee, noder, new TSEdge("e"));
		
		return tsGraph;
	}

	
	/** Main function **/
	/** ideal input: 
	 *  (1. Automaton(R.E.) described task specification, 
	 *  2. Cooperative event set, 
	 *  3. Initial State of Robot) */
	public static void main(String args[]) throws ExportException, IOException {
		
		////////////////////////////Initial Decomposition and Allocation///////////////////////////////
		/** 0. Definition of robots and their transition systems **/		
		List<Graph<ArrayList<String>, TSEdge>> tsGraphList = new ArrayList<>();
		Robot robot0 = new Robot(buildRobotActionList(0), buildTsGraph(0), initNode);
		Set<Robot> providedRobots = new HashSet<>(Arrays.asList(robot0));
		
		/** 1.1 give a set of regular expressions, output automata*/
		RegExp regExpr_1 = new RegExp("(c((khr)|(hkr)|(hrk)))|(h((rck)|(ckr)|(crk)))");
		RegExp regExpr_2 = new RegExp("(C((KHR)|(HKR)|(HRK)))|(H((RCK)|(CKR)|(CRK)))");
		Automaton globalTaskAutomaton1 = ShuffleOperations.shuffle(regExpr_1.toAutomaton(), regExpr_2.toAutomaton());
		
		RegExp regExpr_3 = new RegExp("(v((xwy)|(wxy)|(wyx)))|(w((yvx)|(vxy)|(vyx)))");
		RegExp regExpr_4 = new RegExp("(V((XWY)|(WXY)|(WYX)))|(W((YVX)|(VXY)|(VYX)))");
		Automaton globalTaskAutomaton2 = ShuffleOperations.shuffle(regExpr_3.toAutomaton(), globalTaskAutomaton1);
		
		Automaton globalTaskAutomaton = ShuffleOperations.shuffle(regExpr_4.toAutomaton(), globalTaskAutomaton2);
		
		/** 1.2 Declare cooperative events **/
		Set<Character> CoopEventSet = new HashSet<>() {/**
			 * 
			 */
			private static final long serialVersionUID = 1L;

		{
//			add('g');
		}};
		
		/** 2.1. Obtain sub-automaton, of which all paths have common events **/
		//Set<Automaton> SubautomatonSet= SubAutomataExtract.commonEventsAut(globalTaskAutomaton);
		Set<Automaton> SubautomatonSet = new HashSet<>();
		SubautomatonSet.add(globalTaskAutomaton);
		/** 2.2. Generate parallel decompositions for each sub-automaton**/
		long start = System.nanoTime();
		 // some time passes
		List<List<Automaton>> decompositionSet = new ArrayList<List<Automaton>>();
		for (Automaton automaton : SubautomatonSet)
			decompositionSet.add(decompAut.paraDecompC(automaton, CoopEventSet));
		
		long end = System.nanoTime();
		long elapsedTime = end - start; 
		System.out.println("Decomposition time:" + elapsedTime);
		
//		/** 2.3. Convert each decomposition into graph form**/
//		List<List<TaskAutomaton>> decompositionSetConvert = new ArrayList<List<TaskAutomaton>>();
//		for (List<Automaton> decompositions : decompositionSet) {
//			
//			List<TaskAutomaton> decompositionsConvert = new ArrayList<TaskAutomaton>();
//			for (Automaton singleAutomaton : decompositions) {
//				decompositionsConvert.add(new TaskAutomaton(singleAutomaton));
//			}
//			
//			decompositionSetConvert.add(decompositionsConvert);
//		}
		
//		/////////////////////////////////////////////////////////////////////////////////////////////////////
//		/** 3. manually assign automaton decompositions - robots configuration **/
//		List<RobotSetSwitch> robotSwitchConfigs = new ArrayList<>();
//		for (int index = 0; index < decompositionSetConvert.size(); index++) {
//			RobotSetSwitch robotSetSwitchManned = new RobotSetSwitch();
//			robotSetSwitchManned.autRobotsPair.add(
//					new RobotConfig(decompositionSetConvert.get(index).get(0), providedRobots)
//					);
//			robotSwitchConfigs.add(robotSetSwitchManned);
//		}
//		System.out.println(robotSwitchConfigs.get(0).autRobotsPair.size());
		
//		/** 4. For each decomposition set, take product automaton with transition system to 
//		/* get the subtask allocation automaton **/
//		List<List<TaskAllocAutomaton>> ListOfsubtaskAllocAutomatonLists = new ArrayList<>();
//		List<List<GraphPath<VtxOfProdAut,EdgOfProdAut>>> ListOfShortestPathOfSubAllocationLists=new ArrayList<>();
//		
//		for (RobotSetSwitch robotSetConfig : robotSwitchConfigs) {
//			List<TaskAllocAutomaton> subtaskAllocAutomatonList = new ArrayList<TaskAllocAutomaton>();
//			List<GraphPath<VtxOfProdAut, EdgOfProdAut>> shortestPathList = new ArrayList<>();
//			
//			for (RobotConfig singleConfig : robotSetConfig.autRobotsPair) {
//				System.out.println("---------------------------");
//				System.out.println("Subtask automaton is:" + singleConfig.automaton.AutomatonGraph.vertexSet());
//				
//				RobotTransitionSystem composedTS = RobotTransitionSystem.transitionSystemComposition(
//						singleConfig.robotSet, TaskAutomaton.charSet2StringList(CoopEventSet));
//				TaskAllocAutomaton subtaskAllocAutomatonT = new TaskAllocAutomaton(composedTS, 
//						singleConfig.automaton, providedRobots);
//				subtaskAllocAutomatonList.add(subtaskAllocAutomatonT);
//				
//				//System.out.println("Subtask allocation automaton vertex set is:");
//				//for (VtxOfProdAut vortex : subtaskAllocAutomatonT.taskAllocAutGraph.vertexSet())
//					//System.out.println(vortex.TsState + "," + vortex.AutState + "," + vortex.robotState + " ");
//				String fileName = "subtaskAllocAutomaton" + ListOfsubtaskAllocAutomatonLists.size()
//					+ subtaskAllocAutomatonList.size();
//				TaskAllocAutomaton.renderProdAutGraph(subtaskAllocAutomatonT.taskAllocAutGraph, fileName);
//				
//				/** 4.2 optimal path search**/
//				GraphPath<VtxOfProdAut, EdgOfProdAut> shortestPath = subtaskAllocAutomatonT.lowestCostPath();
//				String fileNameOfPath = "opt" + fileName;				
//				TaskAllocAutomaton.outputSuboptimalPath(shortestPath, fileNameOfPath);
//				
//				shortestPathList.add(shortestPath);
//			}
//			
//			ListOfShortestPathOfSubAllocationLists.add(shortestPathList);
//			ListOfsubtaskAllocAutomatonLists.add(subtaskAllocAutomatonList);
//		}
//		
//		TaskAllocAutomaton.outputFinalOptimalPath(ListOfShortestPathOfSubAllocationLists);
		
	}

}
