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
public class ComptExperiment3_2 {
	
	/** Robot Action Set */		
	public static List<RobotAction> buildRobotActionList(int index){
		List<RobotAction> robotActionList = new ArrayList<>(Arrays.asList(
				new RobotAction(index, "a", 7.0), new RobotAction(index, "b", 12.0), new RobotAction(index, "c", 12.0), 
				new RobotAction(index, "d", 12.0), new RobotAction(index, "e", 12.0), new RobotAction(index, "f", 12.0),
				new RobotAction(index, "g", 5.0)));
		
		return robotActionList;
	}
	
	/** Each Robot's Transition System Graph */
	private static Graph<ArrayList<String>, TSEdge> buildTsGraph(int index) {
//		ArrayList<String> nodef = new ArrayList<String>(Arrays.asList("f"));
		ArrayList<String> nodeg = new ArrayList<String>(Arrays.asList("g"));
		
		ArrayList<String> nodeh = new ArrayList<String>(Arrays.asList("h"));
		ArrayList<String> nodei = new ArrayList<String>(Arrays.asList("i"));
		ArrayList<String> nodej = new ArrayList<String>(Arrays.asList("j"));
//		ArrayList<String> nodel = new ArrayList<String>(Arrays.asList("l"));
		ArrayList<String> nodem = new ArrayList<String>(Arrays.asList("m"));
		ArrayList<String> noden = new ArrayList<String>(Arrays.asList("n"));
		
//		ArrayList<String> nodep = new ArrayList<String>(Arrays.asList("p"));
		ArrayList<String> nodeq = new ArrayList<String>(Arrays.asList("q"));
		
		ArrayList<String> nodet = new ArrayList<String>(Arrays.asList("t"));
		
		Graph<ArrayList<String>, TSEdge> tsGraph = new DefaultDirectedGraph<>(TSEdge.class);
//		tsGraph.addVertex(nodef);
		tsGraph.addVertex(nodeg);
		
		tsGraph.addVertex(nodeh);
		tsGraph.addVertex(nodei);
		tsGraph.addVertex(nodej);
//		tsGraph.addVertex(nodel);
		tsGraph.addVertex(nodem);
		tsGraph.addVertex(noden);
		
//		tsGraph.addVertex(nodep);
		tsGraph.addVertex(nodeq);
		
		tsGraph.addVertex(nodet);
		///////////////////////////////////////////////
		
//		tsGraph.addEdge(nodef, nodei, new TSEdge("i"));
//		tsGraph.addEdge(nodef, nodej, new TSEdge("j"));
		
//		tsGraph.addEdge(nodeg, nodel, new TSEdge("l"));
		tsGraph.addEdge(nodeg, nodem, new TSEdge("m"));
		tsGraph.addEdge(nodeg, noden, new TSEdge("n"));
		
//		tsGraph.addEdge(nodeh, nodep, new TSEdge("p"));
		tsGraph.addEdge(nodeh, nodeq, new TSEdge("q"));
		
//		tsGraph.addEdge(nodei, nodef, new TSEdge("f"));
		tsGraph.addEdge(nodei, nodeg, new TSEdge("g"));
		tsGraph.addEdge(nodei, nodeh, new TSEdge("h"));
		
//		tsGraph.addEdge(nodej, nodel, new TSEdge("l"));
		tsGraph.addEdge(nodej, nodem, new TSEdge("m"));
		tsGraph.addEdge(nodej, noden, new TSEdge("n"));
		
//		tsGraph.addEdge(nodel, nodef, new TSEdge("f"));
//		tsGraph.addEdge(nodel, nodeg, new TSEdge("g"));
//		tsGraph.addEdge(nodel, nodeh, new TSEdge("h"));
		
		tsGraph.addEdge(nodem, nodei, new TSEdge("i"));
		tsGraph.addEdge(nodem, nodej, new TSEdge("j"));
		
//		tsGraph.addEdge(noden, nodep, new TSEdge("p"));
		tsGraph.addEdge(noden, nodeq, new TSEdge("q"));
		
//		tsGraph.addEdge(nodep, nodef, new TSEdge("f"));
//		tsGraph.addEdge(nodep, nodeg, new TSEdge("g"));
//		tsGraph.addEdge(nodep, nodeh, new TSEdge("h"));
		
//		tsGraph.addEdge(nodeq, nodel, new TSEdge("l"));
		tsGraph.addEdge(nodeq, nodem, new TSEdge("m"));
		tsGraph.addEdge(nodeq, noden, new TSEdge("n"));
		
//		tsGraph.addEdge(nodet, nodef, new TSEdge("f"));
		tsGraph.addEdge(nodet, nodeg, new TSEdge("g"));
		tsGraph.addEdge(nodet, nodeh, new TSEdge("h"));
		tsGraph.addEdge(nodet, nodei, new TSEdge("i"));
		tsGraph.addEdge(nodet, nodej, new TSEdge("j"));
//		tsGraph.addEdge(nodet, nodel, new TSEdge("l"));
		tsGraph.addEdge(nodet, nodem, new TSEdge("m"));
		tsGraph.addEdge(nodet, noden, new TSEdge("n"));
//		tsGraph.addEdge(nodet, nodep, new TSEdge("p"));
		tsGraph.addEdge(nodet, nodeq, new TSEdge("q"));
		tsGraph.addEdge(nodet, nodet, new TSEdge("t"));
		
		return tsGraph;
	}

	
	private static Graph<ArrayList<String>, TSEdge> buildTsGraph2(int index) {
//		ArrayList<String> nodef = new ArrayList<String>(Arrays.asList("F"));
		ArrayList<String> nodeg = new ArrayList<String>(Arrays.asList("G"));
		
		ArrayList<String> nodeh = new ArrayList<String>(Arrays.asList("H"));
		ArrayList<String> nodei = new ArrayList<String>(Arrays.asList("I"));
		ArrayList<String> nodej = new ArrayList<String>(Arrays.asList("J"));
//		ArrayList<String> nodel = new ArrayList<String>(Arrays.asList("L"));
		ArrayList<String> nodem = new ArrayList<String>(Arrays.asList("M"));
		ArrayList<String> noden = new ArrayList<String>(Arrays.asList("N"));
		
//		ArrayList<String> nodep = new ArrayList<String>(Arrays.asList("P"));
		ArrayList<String> nodeq = new ArrayList<String>(Arrays.asList("Q"));
		
		ArrayList<String> nodet = new ArrayList<String>(Arrays.asList("T"));
		
		Graph<ArrayList<String>, TSEdge> tsGraph = new DefaultDirectedGraph<>(TSEdge.class);
//		tsGraph.addVertex(nodef);
		tsGraph.addVertex(nodeg);
		
		tsGraph.addVertex(nodeh);
		tsGraph.addVertex(nodei);
		tsGraph.addVertex(nodej);
//		tsGraph.addVertex(nodel);
		tsGraph.addVertex(nodem);
		tsGraph.addVertex(noden);
		
//		tsGraph.addVertex(nodep);
		tsGraph.addVertex(nodeq);
		
		tsGraph.addVertex(nodet);
		///////////////////////////////////////////////
		
//		tsGraph.addEdge(nodef, nodei, new TSEdge("I"));
//		tsGraph.addEdge(nodef, nodej, new TSEdge("J"));
		
//		tsGraph.addEdge(nodeg, nodel, new TSEdge("L"));
		tsGraph.addEdge(nodeg, nodem, new TSEdge("M"));
		tsGraph.addEdge(nodeg, noden, new TSEdge("N"));
		
//		tsGraph.addEdge(nodeh, nodep, new TSEdge("P"));
		tsGraph.addEdge(nodeh, nodeq, new TSEdge("Q"));
		
//		tsGraph.addEdge(nodei, nodef, new TSEdge("F"));
		tsGraph.addEdge(nodei, nodeg, new TSEdge("G"));
		tsGraph.addEdge(nodei, nodeh, new TSEdge("H"));
		
//		tsGraph.addEdge(nodej, nodel, new TSEdge("L"));
		tsGraph.addEdge(nodej, nodem, new TSEdge("M"));
		tsGraph.addEdge(nodej, noden, new TSEdge("N"));
		
//		tsGraph.addEdge(nodel, nodef, new TSEdge("F"));
//		tsGraph.addEdge(nodel, nodeg, new TSEdge("G"));
//		tsGraph.addEdge(nodel, nodeh, new TSEdge("H"));
		
		tsGraph.addEdge(nodem, nodei, new TSEdge("I"));
		tsGraph.addEdge(nodem, nodej, new TSEdge("J"));
		
//		tsGraph.addEdge(noden, nodep, new TSEdge("P"));
		tsGraph.addEdge(noden, nodeq, new TSEdge("Q"));
		
//		tsGraph.addEdge(nodep, nodef, new TSEdge("F"));
//		tsGraph.addEdge(nodep, nodeg, new TSEdge("G"));
//		tsGraph.addEdge(nodep, nodeh, new TSEdge("H"));
		
//		tsGraph.addEdge(nodeq, nodel, new TSEdge("l"));
		tsGraph.addEdge(nodeq, nodem, new TSEdge("M"));
		tsGraph.addEdge(nodeq, noden, new TSEdge("N"));
		
//		tsGraph.addEdge(nodet, nodef, new TSEdge("F"));
		tsGraph.addEdge(nodet, nodeg, new TSEdge("G"));
		tsGraph.addEdge(nodet, nodeh, new TSEdge("H"));
		tsGraph.addEdge(nodet, nodei, new TSEdge("I"));
		tsGraph.addEdge(nodet, nodej, new TSEdge("J"));
//		tsGraph.addEdge(nodet, nodel, new TSEdge("L"));
		tsGraph.addEdge(nodet, nodem, new TSEdge("M"));
		tsGraph.addEdge(nodet, noden, new TSEdge("N"));
//		tsGraph.addEdge(nodet, nodep, new TSEdge("P"));
		tsGraph.addEdge(nodet, nodeq, new TSEdge("Q"));
		tsGraph.addEdge(nodet, nodet, new TSEdge("T"));
		
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
		ArrayList<String> initNode0 = new ArrayList<String>(Arrays.asList("t"));
		Robot robot0 = new Robot(buildRobotActionList(0), buildTsGraph(0), initNode0);
		Set<Robot> providedRobots0 = new HashSet<>(Arrays.asList(robot0));
		
		ArrayList<String> initNode1 = new ArrayList<String>(Arrays.asList("T"));
		Robot robot1 = new Robot(buildRobotActionList(1), buildTsGraph2(1), initNode1);
		Set<Robot> providedRobots1 = new HashSet<>(Arrays.asList(robot1));
		
		/** 1.1 give a set of regular expressions, output automata*/
//		RegExp regExpr_1 = new RegExp("c((khr)|(hkr)|(hrk))");
//		RegExp regExpr_2 = new RegExp("C((KHR)|(HKR)|(HRK))");
		RegExp regExpr_1 = new RegExp("(m((ihq)|(hiq)|(hqi)))|(h((qmi)|(miq)|(mqi)))");
		RegExp regExpr_2 = new RegExp("(M((IHQ)|(HIQ)|(HQI)))|(H((QMI)|(MIQ)|(MQI)))");
		Automaton globalTaskAutomaton1 = regExpr_2.toAutomaton();
		
		RegExp regExpr_3 = new RegExp("(n((jtg)|(tjg)|(tgj)))|(t((gnj)|(njg)|(ngj)))");
		RegExp regExpr_4 = new RegExp("(N((JTG)|(TJG)|(TGJ)))|(T((GNJ)|(NJG)|(NGJ)))");
		Automaton globalTaskAutomaton2 = regExpr_4.toAutomaton();
		//RegExp regExpr_4 = new RegExp("(V((XWY)|(WXY)|(WYX)))|(W((YVX)|(VXY)|(VYX)))");
			
		/** 1.2 Declare cooperative events **/
		Set<Character> CoopEventSet = new HashSet<>() {/**
			 * 
			 */
			private static final long serialVersionUID = 1L;

		{
//			add('g');
		}};
		
		/** 2.1. Obtain sub-automaton, of which all paths have common events **/
		Set<Automaton> SubautomatonSet1= SubAutomataExtract.commonEventsAut(globalTaskAutomaton1);
		Set<Automaton> SubautomatonSet2= SubAutomataExtract.commonEventsAut(globalTaskAutomaton2);
		
		/** 2.2. Generate parallel decompositions for each sub-automaton**/
		long start = System.nanoTime();
		 // some time passes
		List<List<Automaton>> decompositionSet1 = new ArrayList<List<Automaton>>();
		for (Automaton automaton : SubautomatonSet1)
			decompositionSet1.add(decompAut.paraDecompC(automaton, CoopEventSet));
		
		List<List<Automaton>> decompositionSet2 = new ArrayList<List<Automaton>>();
		for (Automaton automaton : SubautomatonSet2)
			decompositionSet2.add(decompAut.paraDecompC(automaton, CoopEventSet));
		
		long end = System.nanoTime();
		long elapsedTime = end - start; 
		System.out.println("Decomposition time:" + elapsedTime);
		
		/** 2.3. Convert each decomposition into graph form**/
		List<List<TaskAutomaton>> decompositionSetConvert = new ArrayList<List<TaskAutomaton>>();
		for (List<Automaton> decompositions : decompositionSet1) {
			
			List<TaskAutomaton> decompositionsConvert = new ArrayList<TaskAutomaton>();
			for (Automaton singleAutomaton : decompositions) {
				decompositionsConvert.add(new TaskAutomaton(singleAutomaton));
			}
			
			decompositionSetConvert.add(decompositionsConvert);
		}
		
		Automaton globalTaskAutomaton1_ = ShuffleOperations.shuffle(regExpr_1.toAutomaton(), decompositionSet1.get(0).get(0));
		Automaton globalTaskAutomaton2_ = ShuffleOperations.shuffle(regExpr_3.toAutomaton(), decompositionSet2.get(0).get(0));
		/////////////////////////////////////////////////////////////////////////////////////////////////////
		/** 3. manually assign automaton decompositions - robots configuration **/
		List<RobotSetSwitch> robotSwitchConfigs = new ArrayList<>();
		for (int index = 0; index < 1; index++) {
			RobotSetSwitch robotSetSwitchManned = new RobotSetSwitch();
			
			robotSetSwitchManned.autRobotsPair.add(
					new RobotConfig(new TaskAutomaton(globalTaskAutomaton1_), providedRobots0));
			robotSetSwitchManned.autRobotsPair.add(
					new RobotConfig(new TaskAutomaton(globalTaskAutomaton2_), providedRobots1));
			robotSwitchConfigs.add(robotSetSwitchManned);
		}
		System.out.println(robotSwitchConfigs.get(0).autRobotsPair.size());
		
		/** 4. For each decomposition set, take product automaton with transition system to 
		/* get the subtask allocation automaton **/
		long start1 = System.nanoTime();
		
		List<List<TaskAllocAutomaton>> ListOfsubtaskAllocAutomatonLists = new ArrayList<>();
		List<List<GraphPath<VtxOfProdAut,EdgOfProdAut>>> ListOfShortestPathOfSubAllocationLists=new ArrayList<>();
		
		for (RobotSetSwitch robotSetConfig : robotSwitchConfigs) {
			List<TaskAllocAutomaton> subtaskAllocAutomatonList = new ArrayList<TaskAllocAutomaton>();
			List<GraphPath<VtxOfProdAut, EdgOfProdAut>> shortestPathList = new ArrayList<>();
			
			for (RobotConfig singleConfig : robotSetConfig.autRobotsPair) {
				//System.out.println("---------------------------");
				//System.out.println("Subtask automaton is:" + singleConfig.automaton.AutomatonGraph.vertexSet());
				
				RobotTransitionSystem composedTS = RobotTransitionSystem.transitionSystemComposition(
						singleConfig.robotSet, TaskAutomaton.charSet2StringList(CoopEventSet));
				TaskAllocAutomaton subtaskAllocAutomatonT = new TaskAllocAutomaton(composedTS, 
						singleConfig.automaton, singleConfig.robotSet);
				subtaskAllocAutomatonList.add(subtaskAllocAutomatonT);
				
				//System.out.println("Subtask allocation automaton vertex set is:");
				//for (VtxOfProdAut vortex : subtaskAllocAutomatonT.taskAllocAutGraph.vertexSet())
					//System.out.println(vortex.TsState + "," + vortex.AutState + "," + vortex.robotState + " ");
				String fileName = "subtaskAllocAutomaton" + ListOfsubtaskAllocAutomatonLists.size()
					+ subtaskAllocAutomatonList.size();
				//TaskAllocAutomaton.renderProdAutGraph(subtaskAllocAutomatonT.taskAllocAutGraph, fileName);
				
				/** 4.2 optimal path search**/
				GraphPath<VtxOfProdAut, EdgOfProdAut> shortestPath = subtaskAllocAutomatonT.lowestCostPath();
				String fileNameOfPath = "opt" + fileName;				
				//TaskAllocAutomaton.outputSuboptimalPath(shortestPath, fileNameOfPath);
				
				shortestPathList.add(shortestPath);
			}
			
			ListOfShortestPathOfSubAllocationLists.add(shortestPathList);
			ListOfsubtaskAllocAutomatonLists.add(subtaskAllocAutomatonList);
		}
		
		long end1 = System.nanoTime();
		long elapsedTime1 = end1 - start1; 
		System.out.println("Comput. time:" + elapsedTime1);
//		TaskAllocAutomaton.outputFinalOptimalPath(ListOfShortestPathOfSubAllocationLists);
		
//		//***Centralized computation***//
//		RobotConfig centralRobotConfig = new RobotConfig(new TaskAutomaton(globalTaskAutomaton), providedRobots);
//		
//		long startc = System.nanoTime();
//		
//		RobotTransitionSystem composedTS = RobotTransitionSystem.transitionSystemComposition(
//				centralRobotConfig.robotSet, TaskAutomaton.charSet2StringList(CoopEventSet));
//		TaskAllocAutomaton subtaskAllocAutomatonT = new TaskAllocAutomaton(composedTS, 
//				centralRobotConfig.automaton, centralRobotConfig.robotSet);
//		
//		/** 4.2 optimal path search**/
//		GraphPath<VtxOfProdAut, EdgOfProdAut> shortestPath = subtaskAllocAutomatonT.lowestCostPath();
//		
//		long endc = System.nanoTime();
//		long elapsedTimec = endc - startc; 
//		System.out.println("Centralized comput. time:" + elapsedTimec);
	}

}
