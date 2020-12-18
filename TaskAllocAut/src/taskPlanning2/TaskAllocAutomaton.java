package taskPlanning2;

/** 
 * Synthesize a subtask allocation automaton: 
 * 1. input the parallel transition system of several robots;
 * 2. convert task automaton into a JGraphT library described automaton;
 * 3. utilize the product of transition system and subtask automaton, which
 *    has a weight associated with each edge. 
 *    */

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.connectivity.BiconnectivityInspector;
import org.jgrapht.alg.interfaces.ShortestPathAlgorithm.SingleSourcePaths;
import org.jgrapht.alg.shortestpath.DijkstraShortestPath;
import org.jgrapht.alg.util.NeighborCache;
import org.jgrapht.graph.DirectedWeightedPseudograph;
import org.jgrapht.io.*;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

public class TaskAllocAutomaton {
	
	public VtxOfProdAut initState;
	public List<VtxOfProdAut> acceptStates = new ArrayList<>();
	public DirectedWeightedPseudograph<VtxOfProdAut, EdgOfProdAut> taskAllocAutGraph = new 
			DirectedWeightedPseudograph<>(EdgOfProdAut.class);
	
	public TaskAllocAutomaton() {};
	
	public TaskAllocAutomaton(VtxOfProdAut initState, List<VtxOfProdAut> acceptStates, 
			DirectedWeightedPseudograph<VtxOfProdAut, EdgOfProdAut> taskAllocAutGraph) {
		this.initState = initState;
		this.acceptStates = acceptStates;
		this.taskAllocAutGraph = taskAllocAutGraph;
	}
	
	/** 
	 * The process is the product of transition system and 
	 * subtask automaton. It removes unqualified states,
	 * which are also called unreachable states (usually 
	 * was called minimization). Finally, it generates subtask 
	 * allocation automaton
	 * **/
	public TaskAllocAutomaton(RobotTransitionSystem transitionSystem, TaskAutomaton taskAutomatonT, 
			Set<Robot> providedRobots) {
		/* *Claim for task allocation automaton* */
		//TaskAllocAutomaton taskAllocationAut = new TaskAllocAutomaton();
		DirectedWeightedPseudograph<VtxOfProdAut, EdgOfProdAut> productAutomaton = 
				new DirectedWeightedPseudograph<>(EdgOfProdAut.class);
		DirectedWeightedPseudograph<VtxOfProdAut, EdgOfProdAut> taskAllocAutGraphOfPA = 
				new DirectedWeightedPseudograph<>(EdgOfProdAut.class);
		boolean TsTransition, AutTransition;

		/* *Claim for task automaton described by java graph topology**/
		String taskAutInitStat;
		ArrayList<String> taskAutAcceptStates;
		//DirectedPseudograph<String,AutomatonEdge> taskAutomatonGraph;
		Set<String> taskAutomatonVertexSet;
		
		/* *Claim for successors of transition system and task automaton with graph topology**/
		NeighborCache<String, AutomatonEdge> neighborsOfAut;
		NeighborCache<ArrayList<String>, TSEdge> neighborsOfTS;
		
		//double weightOfAction; // action weight of transition
		List<Boolean> defaultRobotStates;
		
		/* *Claim for connectivity of product automaton**/
		BiconnectivityInspector<VtxOfProdAut,EdgOfProdAut> connectedTaskAllocAut;
		Set<Graph<VtxOfProdAut, EdgOfProdAut>> subGraphSet;
		
		/* * 0. Add epsilon state to transition system**/
//		ArrayList<String> Epsilon = new ArrayList<String>(Arrays.asList("Init"));
//		transitionSystem.tsGraph.addVertex(Epsilon);
//		ArrayList<String> tempState = transitionSystem.initSate;
//		transitionSystem.tsGraph.addEdge(Epsilon, tempState);
//		transitionSystem.initSate = Epsilon;
		
		/* *1. Generate states of product automaton**/
		//convert Automaton (class) to TaskAutomaton (class) 
		taskAutomatonVertexSet = taskAutomatonT.AutomatonGraph.vertexSet();
		taskAutInitStat = taskAutomatonT.initialState;
		taskAutAcceptStates = taskAutomatonT.acceptedStates;
		
		// successor set for task automaton's vertices and transition system's vertices
		neighborsOfAut = new NeighborCache<String,AutomatonEdge>(taskAutomatonT.AutomatonGraph);
		neighborsOfTS = new NeighborCache<ArrayList<String>, TSEdge>(transitionSystem.tsGraph);
		
//		Set<String> AutInitSuccessors = neighborsOfAut.successorsOf(taskAutInitStat);
//		taskAutInitSuccessorStat = new String();
//		for (String InitSuccessor : AutInitSuccessors) {/* better to guarantee it has one successor */
//			AutomatonEdge AutInitEdge = taskAutomatonT.AutomatonGraph.getEdge(taskAutInitStat, InitSuccessor);
//			if (transitionSystem.initSate.contains(AutInitEdge.Event))
//				taskAutInitSuccessorStat = InitSuccessor;
//		}
		
//		VtxOfProdAut[] vertexSetOfPA = new VtxOfProdAut[transitionSystem.tsGraph.vertexSet().size() * 
//		                                                taskAutomatonVertexSet.size()];
//		int num = 0;
		//Synthesize the composite vertex of product automaton (subtask allocation automaton)
		for (String automatonVertex : taskAutomatonVertexSet)
			for (ArrayList<String> tranSystemVertex : transitionSystem.tsGraph.vertexSet()) {
				VtxOfProdAut tmpVertexOfPA = new VtxOfProdAut(tranSystemVertex, automatonVertex);
				defaultRobotStates = Collections.nCopies(tranSystemVertex.size(), false);
				tmpVertexOfPA.robotState.addAll(defaultRobotStates);
				
				productAutomaton.addVertex(tmpVertexOfPA);
				////System.out.println("Current vertex is:" + vertexSetOfPA[num].TsState + vertexSetOfPA[num].AutState);
				
				
				//Identify the initial state
				if (tranSystemVertex.equals(transitionSystem.initSate) && automatonVertex.equals(taskAutInitStat))
					this.initState = tmpVertexOfPA;
				
				//Identify the final state
				if (taskAutAcceptStates.contains(automatonVertex))
					this.acceptStates.add(tmpVertexOfPA);
				
//				num++;
			}
		System.out.println("Initial state exists:" + this.initState.TsState + "," + this.initState.AutState);
		
		/* *2. Generate the edges of product automaton (subtask allocation automaton)****/
		
		//Verify the transition relation between two composite vertices: strtVertex, destVertex.
		//1.if a transition relation exists between strtVertex's transition system vertex and destVertex's
		//transition system vertex, AND a transition relation (event) exists between strtVertex's automaton
		//vertex and destVertex's automaton vertex;
		//2. if the above event is contained in destVertex's transition system vertex;
		//Only both 1 and 2 are satisfied, strtVertex and destVertex have a transition relation (event)
		for (VtxOfProdAut strtVertex : productAutomaton.vertexSet())
			for (VtxOfProdAut destVertex : productAutomaton.vertexSet())
			{
				TsTransition = neighborsOfTS.successorsOf(strtVertex.TsState).contains(destVertex.TsState); 
				AutTransition = neighborsOfAut.successorsOf(strtVertex.AutState).contains(destVertex.AutState);
				
				if (TsTransition && AutTransition) 
				{
					TSEdge tsEdge = transitionSystem.tsGraph.getEdge(strtVertex.TsState, destVertex.TsState);
					Set<AutomatonEdge> tmpAutEdgeSet = taskAutomatonT.AutomatonGraph.getAllEdges(
							strtVertex.AutState, destVertex.AutState);
					for (AutomatonEdge autTransEdge : tmpAutEdgeSet)
//						if (destVertex.TsState.contains(autTransEdge.Event)) {
						if (tsEdge.process.equals(autTransEdge.Event)) {
							EdgOfProdAut edgeOfProdAut = new EdgOfProdAut();
							edgeOfProdAut.action = autTransEdge.Event;
							edgeOfProdAut.weight = 0;
							
							for (int index = 0; index < strtVertex.TsState.size(); index++) {
								if (strtVertex.TsState.get(index).equals(autTransEdge.Event)) {
									edgeOfProdAut.robotID.add(transitionSystem.idList.get(index));
									edgeOfProdAut.weight += findWeightOfRobotAction(edgeOfProdAut.action, 
											transitionSystem.idList.get(index), providedRobots);
									strtVertex.robotState.set(index, true);
								}
							}
							
							productAutomaton.addEdge(strtVertex, destVertex, edgeOfProdAut);
							productAutomaton.setEdgeWeight(edgeOfProdAut, edgeOfProdAut.weight);
						}
				}
			}
		
		
		/* *3. remove the unqualified vertex **/
		/* * 3.0 Identify the connectivity between vertices;**/
		/* * select the subgraph containing initial state **/
//		connectedTaskAllocAut = new BiconnectivityInspector<VtxOfProdAut,EdgOfProdAut>(productAutomaton);
//		subGraphSet = connectedTaskAllocAut.getConnectedComponents();
		
	    //System.out.println("Initial state:" + this.initState.TsState + "," + this.initState.AutState);
		
	    /* *3.1 Remove the disconnected states**/
//	    for (Graph<VtxOfProdAut,EdgOfProdAut> subGraph : subGraphSet) {
//	    	if (!subGraph.vertexSet().contains(this.initState)) {
//	    		productAutomaton.removeAllVertices(subGraph.vertexSet());
//	    	}
//	    }
	    
	    /*need to label the accepted states such that Dijkstra's search can find the optimal allocation*/ 
	    taskAllocAutGraphOfPA = minimizedGraphOfTaskAlloc(productAutomaton, this.initState);
	    
	    /* 3.2 Remove the unreachable states **/
	    this.acceptStates.retainAll(taskAllocAutGraphOfPA.vertexSet()); /*all reachable and accepted states */
//	    System.out.println("Final accepted states:");
//	    for (VtxOfProdAut finalAcceptState: this.acceptStates)
//	    	System.out.println(finalAcceptState.TsState + "," + finalAcceptState.AutState);
	    /* 3.3 if accepted states are empty, then the PA doesn't exist*/
	    if (!this.acceptStates.isEmpty())
	    	this.taskAllocAutGraph = taskAllocAutGraphOfPA;
	}
	
	/** Find cost of robot's certain action**/
	public static double findWeightOfRobotAction(String action, int rID, Set<Robot> providedRobots) {
		
		for (Robot robot : providedRobots)
			if (robot.getID() == rID)
				for (RobotAction robotAction : robot.ActionList)
					if (robotAction.event.equals(action) )
						return robotAction.weight;
		return 0;
	}
	
	/** Minimize the vertices in graph of task allocation automaton*/
	public DirectedWeightedPseudograph<VtxOfProdAut,EdgOfProdAut> minimizedGraphOfTaskAlloc(
			DirectedWeightedPseudograph<VtxOfProdAut,EdgOfProdAut> productAutomaton, VtxOfProdAut initState) {
		/* * Successor set of each vertex in non-minimized 
		 * product automaton graph (subtask allocation automaton) **/
		NeighborCache<VtxOfProdAut,EdgOfProdAut> neighborsOfPA = new NeighborCache<>(productAutomaton);
		/* * Initialize a to be removed vertex set**/
		Set<VtxOfProdAut> tobeRemovedVertexSet = new HashSet<>(productAutomaton.vertexSet());
		
		/* * open list queue (for traverse) **/
		Queue<VtxOfProdAut> openList = new LinkedList<>();
		openList.add(initState);
		
		/* * Closed set (the desired set)**/
		Set<VtxOfProdAut> closedSet = new HashSet<VtxOfProdAut>();
		VtxOfProdAut currentVertex;
		
		while (!openList.isEmpty()) {
			currentVertex = openList.poll();
			closedSet.add(currentVertex);
			
			for (VtxOfProdAut successor : neighborsOfPA.successorsOf(currentVertex))
				if (!closedSet.contains(successor))
					openList.add(successor);
		}
		
		/* * Remove undesired vertices**/
		tobeRemovedVertexSet.removeAll(closedSet);
		//System.out.println(productAutomaton.vertexSet().size() + " " + tobeRemovedVertexSet.size());
		productAutomaton.removeAllVertices(tobeRemovedVertexSet);
		System.out.println("Task allocation automaton final size:" + productAutomaton.vertexSet().size());
		return productAutomaton;
	}
	
	/** Output the subtask allocation automaton graph in .dot format */
    public static void renderProdAutGraph(Graph<VtxOfProdAut, EdgOfProdAut> PAGraph, String fileName)
            throws ExportException, IOException
    {
		ComponentNameProvider<VtxOfProdAut> vertexLabelProvider = new ComponentNameProvider<VtxOfProdAut>(){
			public String getName(VtxOfProdAut vertex) 
				{
					return vertex.TsState.toString() + "," + vertex.AutState;
				}
		};
		
		ComponentNameProvider<VtxOfProdAut> vertexIdProvider = new ComponentNameProvider<VtxOfProdAut>() {
			public String getName(VtxOfProdAut vertex) {
				String listString = "";
				for (String s : vertex.TsState)
				{
					listString += s;
				}
				return listString + vertex.AutState;
			}
		};
		
		ComponentNameProvider<EdgOfProdAut> edgeLabelProvider = new ComponentNameProvider<EdgOfProdAut>() {
			public String getName(EdgOfProdAut edge) {
				return edge.action + "," + edge.robotID.toString();
			}
		};
		
		GraphExporter<VtxOfProdAut,EdgOfProdAut> exporter = new DOTExporter<>(vertexIdProvider, 
				vertexLabelProvider, edgeLabelProvider);
		BufferedWriter writer = new BufferedWriter(new FileWriter(fileName));
		exporter.exportGraph(PAGraph, writer);
		//System.out.println(writer.toString());
	}
    
	/** Dijkstra algorithm search the lowest cost path**/
	public GraphPath<VtxOfProdAut, EdgOfProdAut> lowestCostPath () {
		GraphPath<VtxOfProdAut, EdgOfProdAut> lowestPath = null;
		double minimalWeight = 1000;
		SingleSourcePaths<VtxOfProdAut, EdgOfProdAut> shortestPaths;
		
		if (!taskAllocAutGraph.vertexSet().isEmpty()) {		
			DijkstraShortestPath<VtxOfProdAut, EdgOfProdAut> dijkstraAlg =
		            new DijkstraShortestPath<>(taskAllocAutGraph);
			
			shortestPaths = dijkstraAlg.getPaths(initState);
			
			for (VtxOfProdAut dest : acceptStates)
				if (shortestPaths.getPath(dest).getWeight() < minimalWeight)
				{
					lowestPath = shortestPaths.getPath(dest);
					minimalWeight = lowestPath.getWeight();
				}
		}
		
		return lowestPath;
	}
	
	/** Output the lowest cost path
	 * @throws IOException */
	public static void outputSuboptimalPath(GraphPath<VtxOfProdAut, EdgOfProdAut> shortestPath, String fileNameOfPath) 
			throws IOException {
		String shortestPathString = new String();
		for (EdgOfProdAut edge : shortestPath.getEdgeList()) {
			System.out.print(edge.action + "," + edge.robotID + ";");
			
			shortestPathString += edge.action + "," + edge.robotID + ";";
			shortestPathString += "\n";
		}
		System.out.print("\n");
		
		BufferedWriter pathWriter = new BufferedWriter(new FileWriter(fileNameOfPath));
		pathWriter.write(shortestPathString);
		pathWriter.close();
	}
	
	/** Output the lowest cost path
	 * @throws IOException */
	public static void outputFinalOptimalPath(List<List<GraphPath<VtxOfProdAut,EdgOfProdAut>>> 
		ListOfShortestPathOfSubAllocationLists) 
			throws IOException {
		
		double lowestCosts = 1000.0;
		List<GraphPath<VtxOfProdAut, EdgOfProdAut>> optimalShorestPath = new ArrayList<>();
		
		for (int index = 0; index < ListOfShortestPathOfSubAllocationLists.size(); index++) {
			double sumCostOfPath = 0;
			
			for (GraphPath<VtxOfProdAut, EdgOfProdAut> shortestPath : ListOfShortestPathOfSubAllocationLists.get(index))
				sumCostOfPath += shortestPath.getWeight();
			
			if (sumCostOfPath < lowestCosts) {
				lowestCosts = sumCostOfPath;
				optimalShorestPath = ListOfShortestPathOfSubAllocationLists.get(index);
			}
		}
		
		System.out.println("The optimal paths:");
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

/** Define the class of product automaton vertex*/
class VtxOfProdAut{
	ArrayList<String> TsState = new ArrayList<>();
	String AutState;
	List<Boolean> robotState = new ArrayList<>();
	
	public VtxOfProdAut() {};
	public VtxOfProdAut(ArrayList<String> TsState, String AutState){
		this.TsState = TsState;
		this.AutState = AutState;
	}
	
}

/** Define the class of product automaton edge */
class EdgOfProdAut {
	ArrayList<Integer> robotID;
	String action;
	double weight;
	
	public EdgOfProdAut() {
		robotID = new ArrayList<Integer>();
	};
	
	public EdgOfProdAut(String action, ArrayList<Integer> robotID){
		this.action = action;
		this.robotID = robotID;
	}
	
}

/** Define the robot action class */
class RobotAction {
	String event;
	int robotID;
	double weight;
	
	public RobotAction(int robotID, String event, double weight){
		this.robotID = robotID;
		this.event = event;
		this.weight = weight;
	}
	
	public RobotAction() {};
	
}
