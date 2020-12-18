package taskPlanning2;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Concurrently execute all the subtask allocation automata,
 * present all the paths, and select the optimal one */

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Set;

import org.jgrapht.GraphPath;
import org.jgrapht.alg.interfaces.ShortestPathAlgorithm.SingleSourcePaths;
import org.jgrapht.alg.shortestpath.DijkstraShortestPath;
import org.jgrapht.alg.util.NeighborCache;
import org.jgrapht.graph.DefaultDirectedWeightedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.io.ComponentNameProvider;
import org.jgrapht.io.DOTExporter;
import org.jgrapht.io.ExportException;
import org.jgrapht.io.GraphExporter;

public class ParallelExecAutomaton {
	public VtxOfProdAut InitParaState;
	public Set<VtxOfProdAut> AcceptParaStates = new HashSet<>();
	public DefaultDirectedWeightedGraph<VtxOfProdAut, EdgeOfParaExec> paraTaskAllocAutGraph = new 
			DefaultDirectedWeightedGraph<>(EdgeOfParaExec.class);
	
	/** Initialization 1*/
	public ParallelExecAutomaton(){}
	
	/** Initialization 2*/
	public ParallelExecAutomaton(VtxOfProdAut InitParaState, Set<VtxOfProdAut> AcceptParaStates,
			DefaultDirectedWeightedGraph<VtxOfProdAut, EdgeOfParaExec> paraTaskAllocAutGraph)
	{
		this.InitParaState = InitParaState;
		this.AcceptParaStates = AcceptParaStates;
		this.paraTaskAllocAutGraph = paraTaskAllocAutGraph;
	}
	
	public static double alpha = 0.5;//Concurrent execution coefficients
	
	/** merge robot parallel transition states**/
	private VtxOfProdAut mergeState(VtxOfProdAut vertexPA1, VtxOfProdAut vertexPA2){
		VtxOfProdAut mergedState = new VtxOfProdAut(); 
		mergedState.AutState = vertexPA1.AutState + "_" + vertexPA2.AutState;
		
		for (int index = 0; index < vertexPA1.TsState.size(); index++) {
			if (vertexPA1.TsState.get(index).equals(vertexPA2.TsState.get(index))) {
				mergedState.TsState.add(vertexPA1.TsState.get(index));
				mergedState.robotState.add(vertexPA1.robotState.get(index));
			}
			else
			{
				if (vertexPA1.robotState.get(index) == true && vertexPA2.robotState.get(index) == false) {
					mergedState.TsState.add(vertexPA1.TsState.get(index));
					mergedState.robotState.add(true);
				}
				else if (vertexPA1.robotState.get(index) == false && vertexPA2.robotState.get(index) == true) {
					mergedState.TsState.add(vertexPA2.TsState.get(index));
					mergedState.robotState.add(true);
				}	
				else {
					mergedState = null;
					break;
				}
			}
		}
		
		return mergedState;
	}
	
	/**
	 * Synthesize the parallel execution of TWO subtask allocation automata
	 * @param subtaskAllocAut1 - subtask allocation automaton
	 * @param subtaskAllocAut2 - subtask allocation automaton
	 * @return paraExecAut - parallel executed task allocation automata
	 */
	public ParallelExecAutomaton(TaskAllocAutomaton subtaskAllocAut1, TaskAllocAutomaton 
			subtaskAllocAut2) {
		/* 0.Initialization */
		Queue<ArrayList<VtxOfProdAut>> openList = new LinkedList<>();
		Queue<VtxOfProdAut> openListOfSate = new LinkedList<>();
		NeighborCache<VtxOfProdAut,EdgOfProdAut> NeighborSetOfS1 = new NeighborCache<>(
				subtaskAllocAut1.taskAllocAutGraph);//set of neighbors
		NeighborCache<VtxOfProdAut,EdgOfProdAut> NeighborSetOfS2 = new NeighborCache<>(
				subtaskAllocAut2.taskAllocAutGraph);//set of neighbors 
		
		/* Add initial state pair into the open list */
		ArrayList<VtxOfProdAut> InitStatePair = new ArrayList<>(Arrays.asList(subtaskAllocAut1.initState,
				subtaskAllocAut2.initState));
		openList.add(InitStatePair);
		
		/* Add initial state */
		this.InitParaState = mergeState(subtaskAllocAut1.initState,subtaskAllocAut2.initState);
		this.paraTaskAllocAutGraph.addVertex(InitParaState);
		openListOfSate.add(InitParaState);
		
		while (!openList.isEmpty()) {
			/* 1.1 obtain the current state of open list*/
			ArrayList<VtxOfProdAut> currentStatePair = openList.poll();
			VtxOfProdAut pair1 = currentStatePair.get(0);
			VtxOfProdAut pair2 = currentStatePair.get(1);
			VtxOfProdAut currentState = openListOfSate.poll();
			
			/* obtain the successors of current state pair*/
			Set<VtxOfProdAut> pair1SuccessorSet = new HashSet<>(NeighborSetOfS1.successorsOf(pair1));
			Set<VtxOfProdAut> pair2SuccessorSet = new HashSet<>(NeighborSetOfS2.successorsOf(pair2));
			
			/* 1.2 Add accepted states*/
			if (subtaskAllocAut1.acceptStates.contains(pair1) && subtaskAllocAut2.acceptStates.contains(pair2))
				this.AcceptParaStates.add(currentState);
			
			/* 3 different situations to be deal with:
			 * a. - both the two elements of state pair have successors */
			if (!pair1SuccessorSet.isEmpty() && !pair2SuccessorSet.isEmpty())
				for (VtxOfProdAut successor1 : pair1SuccessorSet) {
					EdgOfProdAut pair1Edge = subtaskAllocAut1.taskAllocAutGraph.getEdge(pair1, successor1);
					
					for (VtxOfProdAut successor2 : pair2SuccessorSet) {
						EdgOfProdAut pair2Edge = subtaskAllocAut2.taskAllocAutGraph.getEdge(pair2,successor2);
						
						/* list all the edges with robots for subtask allocation aut 1*/ 
						/* keep all the edges with the same robot in subtask allocation aut 2, 
						 * these are the edges that need to be removed in the overall task 
						 * allocation automaton structure */
						ArrayList<Integer> edgeIntersectionRId = new ArrayList<>(pair1Edge.robotID);
						edgeIntersectionRId.retainAll(pair2Edge.robotID); 
						
						/* Satisfy the situation that the transition is parallel executable,
						 * or it is a cooperative process*/
						if (edgeIntersectionRId.isEmpty() || pair1Edge.action.equals(pair2Edge.action)) {
							/* Synthesize and add successor composite vertex*/
							VtxOfProdAut nextState = mergeState(successor1, successor2);
							if (nextState == null)
								continue;
							openListOfSate.add(nextState);
							
							ArrayList<VtxOfProdAut> nextStatePair = new ArrayList<>(Arrays.asList(successor1,
									successor2));
							openList.add(nextStatePair);
							this.paraTaskAllocAutGraph.addVertex(nextState);
							
							/* Synthesize and add successor composite edges*/
							EdgeOfParaExec compositeEdge = new EdgeOfParaExec(new ArrayList<>(Arrays.asList(
									pair1Edge, pair2Edge)), (pair1Edge.weight + pair2Edge.weight) * alpha);
							System.out.println(currentState.TsState + "," + currentState.AutState);
							System.out.println(nextState.TsState + "," + nextState.AutState);
							this.paraTaskAllocAutGraph.addEdge(currentState, nextState, compositeEdge);
							this.paraTaskAllocAutGraph.setEdgeWeight(currentState, nextState, 
									compositeEdge.weight);
						}
						
					}
				}
			
			/* b. - the first element of state pair certainly has successors */
			if (!pair1SuccessorSet.isEmpty()) {
				for (VtxOfProdAut successor1 : pair1SuccessorSet) {
					EdgOfProdAut pair1Edge = subtaskAllocAut1.taskAllocAutGraph.getEdge(pair1, successor1);
					
					if (TaskAllocAutomaton.CooperativeEventSet.contains(pair1Edge.action))
						break;
					
					/* Synthesize and add successor composite vertex*/
					VtxOfProdAut nextState1 = mergeState(successor1, pair2);
					if (nextState1 == null)
						continue;
					openListOfSate.add(nextState1);
					
					ArrayList<VtxOfProdAut> nextStatePair1 = new ArrayList<>(Arrays.asList(successor1,pair2));
					openList.add(nextStatePair1);
					this.paraTaskAllocAutGraph.addVertex(nextState1);
					
					/* Synthesize and add successor composite edges*/
					EdgeOfParaExec compositeEdge1 = new EdgeOfParaExec(new ArrayList<>(Arrays.asList(pair1Edge)),
							pair1Edge.weight);
					this.paraTaskAllocAutGraph.addEdge(currentState, nextState1, compositeEdge1);
					this.paraTaskAllocAutGraph.setEdgeWeight(currentState, nextState1, 
							compositeEdge1.weight);
				}
			}
			
			/* c. - the second element of state pair certainly has successors */
			if (!pair2SuccessorSet.isEmpty()) {
				for (VtxOfProdAut successor2 : pair2SuccessorSet) {
					EdgOfProdAut pair2Edge = subtaskAllocAut2.taskAllocAutGraph.getEdge(pair2, successor2);
					
					if (TaskAllocAutomaton.CooperativeEventSet.contains(pair2Edge.action))
						break;
					
					/* Synthesize and add successor composite vertex*/
					VtxOfProdAut nextState2 = this.mergeState(pair1, successor2);
					if (nextState2 == null)
						continue;
					openListOfSate.add(nextState2);
					
					ArrayList<VtxOfProdAut> nextStatePair2 = new ArrayList<>(Arrays.asList(pair1,successor2));
					openList.add(nextStatePair2);
					this.paraTaskAllocAutGraph.addVertex(nextState2);
					
					/* Synthesize and add successor composite edges*/
					EdgeOfParaExec compositeEdge2 = new EdgeOfParaExec(new ArrayList<>(Arrays.asList(pair2Edge)),
							pair2Edge.weight);
					this.paraTaskAllocAutGraph.addEdge(currentState, nextState2, compositeEdge2);
					this.paraTaskAllocAutGraph.setEdgeWeight(currentState, nextState2, 
							compositeEdge2.weight);
				}
			}
		}
		
		//paraExecAut.paraTaskAllocAutGraph = paraExecAutGraph;
	}
	
	/**
	 * Synthesize the parallel execution of more than two subtask allocation automata
	 * @param paraAllocAut - parallel executed subtask allocation automaton
	 * @param subtaskAllocAut3 - subtask allocation automaton
	 * @return paraExecAut - parallel executed task allocation automata
	 */
	public static ParallelExecAutomaton parallelExec(ParallelExecAutomaton paraAllocAut, TaskAllocAutomaton 
			subtaskAllocAut3) {
		/* 0.Initialization */
		ParallelExecAutomaton paraExecAut = new ParallelExecAutomaton();		
		Queue<ArrayList<VtxOfProdAut>> openList = new LinkedList<>();
		Queue<VtxOfProdAut> openListOfSate = new LinkedList<>();
		/* neighbors and successors*/
		NeighborCache<VtxOfProdAut, EdgeOfParaExec> NeighborSetOfPA = new NeighborCache<>(
				paraAllocAut.paraTaskAllocAutGraph);
		NeighborCache<VtxOfProdAut,EdgOfProdAut> NeighborSetOfS3 = new NeighborCache<>(
				subtaskAllocAut3.taskAllocAutGraph);
		
		/* Synthesize initial state */
		ArrayList<VtxOfProdAut> InitStatePair = new ArrayList<>(Arrays.asList(paraAllocAut.InitParaState,
				subtaskAllocAut3.initState));
		openList.add(InitStatePair);
		
		paraExecAut.InitParaState = paraExecAut.mergeState(paraAllocAut.InitParaState,subtaskAllocAut3.initState);
		openListOfSate.add(paraExecAut.InitParaState);
		paraExecAut.paraTaskAllocAutGraph.addVertex(paraExecAut.InitParaState);
		
		while (!openList.isEmpty()) {
			ArrayList<VtxOfProdAut> currentStatePair = openList.poll();
			//ArrayList<VtxOfProdAut> pair1 = get1stN(currentStatePair, currentStatePair.size()-2);
			VtxOfProdAut pair1 = currentStatePair.get(0);
			VtxOfProdAut pair2 = currentStatePair.get(1);
			VtxOfProdAut currentState = openListOfSate.poll();
			
			Set<VtxOfProdAut> pair1SuccessorSet = new HashSet<>(NeighborSetOfPA.successorsOf(pair1));
			Set<VtxOfProdAut> pair2SuccessorSet = new HashSet<>(NeighborSetOfS3.successorsOf(pair2));
			
			/* Add accepted states*/
			if (paraAllocAut.AcceptParaStates.contains(pair1)&&subtaskAllocAut3.acceptStates.contains(pair2))
				paraExecAut.AcceptParaStates.add(currentState);
			
			if (!pair1SuccessorSet.isEmpty() && !pair2SuccessorSet.isEmpty())
				for (VtxOfProdAut successor1 : pair1SuccessorSet) {
					EdgeOfParaExec pair1Edge = paraAllocAut.paraTaskAllocAutGraph.getEdge(pair1, successor1);
					
					for (VtxOfProdAut successor2 : pair2SuccessorSet) {
						EdgOfProdAut pair2Edge = subtaskAllocAut3.taskAllocAutGraph.getEdge(pair2,successor2);
						
						/* list all the edges with robots for subtask allocation aut 1*/ 
						/*keep all the edges with the same robot in subtask allocation aut 2, 
						 * these are the edges that need to be removed in the overall task 
						 * allocation automaton structure */
						ArrayList<Integer> edgeIntersectionRId = new ArrayList<>();
						for (EdgOfProdAut edge : pair1Edge.transEdge)
							edgeIntersectionRId.addAll(edge.robotID);
						edgeIntersectionRId.retainAll(pair2Edge.robotID);
						
						/* Satisfy the situation that the transition is parallel executable,
						 * or it is a cooperative process*/
						if (edgeIntersectionRId.isEmpty() || pair1Edge.transEdge.get(0).action.equals(
								pair2Edge.action)) {
							/* Synthesize and add successor composite vertex*/
							VtxOfProdAut nextState = paraExecAut.mergeState(successor1,successor2);
							if (nextState == null)
								continue;
							openListOfSate.add(nextState);
							
							ArrayList<VtxOfProdAut> nextStatePair = new ArrayList<>(Arrays.asList(successor1,
									successor2));
							openList.add(nextStatePair);
							paraExecAut.paraTaskAllocAutGraph.addVertex(nextState);
							
							/* Synthesize and add successor composite edges*/
							ArrayList<EdgOfProdAut> newEdges= new ArrayList<>(pair1Edge.transEdge);
							newEdges.add(pair2Edge);
							EdgeOfParaExec compositeEdge = new EdgeOfParaExec(newEdges,
									(pair1Edge.weight + pair2Edge.weight) * alpha);
							paraExecAut.paraTaskAllocAutGraph.addEdge(currentState, nextState, compositeEdge);
							paraExecAut.paraTaskAllocAutGraph.setEdgeWeight(currentState, nextState, 
									compositeEdge.weight);
						}
						
					}
				}
			
			if (!pair1SuccessorSet.isEmpty()) {
				for (VtxOfProdAut successor1 : pair1SuccessorSet) {
					EdgeOfParaExec pair1Edge = paraAllocAut.paraTaskAllocAutGraph.getEdge(pair1, successor1);
					if (TaskAllocAutomaton.CooperativeEventSet.contains(pair1Edge.transEdge.get(0).action))
						break;
					
					/* Synthesize and add successor composite vertex*/
					VtxOfProdAut nextState1 = paraExecAut.mergeState(successor1,pair2);
					if (nextState1 == null)
						continue;
					openListOfSate.add(nextState1);
					
					ArrayList<VtxOfProdAut> nextStatePair1 = new ArrayList<>(Arrays.asList(successor1, pair2));
					openList.add(nextStatePair1);
					paraExecAut.paraTaskAllocAutGraph.addVertex(nextState1);
					
					/* Synthesize and add successor composite edges*/
					EdgeOfParaExec compositeEdge1 = new EdgeOfParaExec(pair1Edge.transEdge, pair1Edge.weight);
					//System.out.println(pair1Edge.action + pair1Edge.robotID);
					paraExecAut.paraTaskAllocAutGraph.addEdge(currentState, nextState1, compositeEdge1);
					paraExecAut.paraTaskAllocAutGraph.setEdgeWeight(currentState, nextState1, 
							compositeEdge1.weight);
				}
			}
			
			if (!pair2SuccessorSet.isEmpty()) {
				for (VtxOfProdAut successor2 : pair2SuccessorSet) {
					EdgOfProdAut pair2Edge = subtaskAllocAut3.taskAllocAutGraph.getEdge(pair2, successor2);
					if (TaskAllocAutomaton.CooperativeEventSet.contains(pair2Edge.action))
						break;
					/* Synthesize and add successor composite vertex*/
					VtxOfProdAut nextState2 = paraExecAut.mergeState(pair1,successor2);
					if (nextState2 == null)
						continue;
					openListOfSate.add(nextState2);
					
					ArrayList<VtxOfProdAut> nextStatePair2 = new ArrayList<>(Arrays.asList(pair1, successor2));
					openList.add(nextStatePair2);
					paraExecAut.paraTaskAllocAutGraph.addVertex(nextState2);
					
					/* Synthesize and add successor composite edges*/
					EdgeOfParaExec compositeEdge2 = new EdgeOfParaExec(new ArrayList<>(Arrays.asList(
							pair2Edge)), pair2Edge.weight);
					paraExecAut.paraTaskAllocAutGraph.addEdge(currentState, nextState2, compositeEdge2);
					paraExecAut.paraTaskAllocAutGraph.setEdgeWeight(currentState, nextState2, 
							compositeEdge2.weight);
				}
			}
		}
		
//		paraExecAut.paraTaskAllocAutGraph = paraExecAutGraph;
		
		return paraExecAut;
	}
	
	/** 
	 * obtain the first N elements of an array list
	 * @param list - list to be checked
	 * @param n - the number of elements needed
	 * @return firstNelements - an array list with N elements
	 */
	@SuppressWarnings("unused")
	private ArrayList<VtxOfProdAut> get1stN(ArrayList<VtxOfProdAut> list, int n) {
		ArrayList<VtxOfProdAut> firstNelements = new ArrayList<>();
		
		for (int index = 0; index <= n; index++)
			firstNelements.add(list.get(index));
		
		return firstNelements;
	}
	
	/** Dijkstra algorithm search the lowest cost path**/
	public GraphPath<VtxOfProdAut, EdgeOfParaExec> lowestCostPath (ParallelExecAutomaton paraExecAut) {
		GraphPath<VtxOfProdAut, EdgeOfParaExec> lowestPath = null;
		double minimalWeight = 1000;
		SingleSourcePaths<VtxOfProdAut, EdgeOfParaExec> shortestPaths;
		
		DijkstraShortestPath<VtxOfProdAut, EdgeOfParaExec> dijkstraAlg =
	            new DijkstraShortestPath<>(paraExecAut.paraTaskAllocAutGraph);
		
		shortestPaths = dijkstraAlg.getPaths(paraExecAut.InitParaState);
		
		for (VtxOfProdAut dest : paraExecAut.AcceptParaStates)
			if (shortestPaths.getPath(dest).getWeight() < minimalWeight)
			{
				lowestPath = shortestPaths.getPath(dest);
				minimalWeight = lowestPath.getWeight();
			}
		
		return lowestPath;
	}
	
	/** Output the parallel execution of subtask allocation automaton graph in .dot format */
	public static void renderParaExecGraph(DefaultDirectedWeightedGraph<VtxOfProdAut, EdgeOfParaExec> 
			paraExecAutomaton, String fileName) 
					throws ExportException, IOException
    {
		ComponentNameProvider<VtxOfProdAut> vertexLabelProvider = new ComponentNameProvider<>(){
					public String getName(VtxOfProdAut vertex)
		            {
						 return vertex.TsState.toString() + "," + vertex.AutState;
		            }
		        };
		        
		ComponentNameProvider<VtxOfProdAut> vertexIdProvider = new ComponentNameProvider<>() {
            public String getName(VtxOfProdAut vertex)
            {
            	String listString = "";
            	for (String s : vertex.TsState)
            	{
            	    listString += s;
            	}
            	
                return listString + vertex.AutState;
            }
        };
        
        ComponentNameProvider<EdgeOfParaExec> edgeLabelProvider = new ComponentNameProvider<>(){
		            public String getName(EdgeOfParaExec edge)
		            {   
		            	String listString = "";
		            	
		            	for (EdgOfProdAut element : edge.transEdge) {
		            		listString = listString + element.action + "," + element.robotID.toString();
		            	}
		            	
		                return listString;
		            }
		        };
		
		GraphExporter<VtxOfProdAut, EdgeOfParaExec> exporter =
	            new DOTExporter<>(vertexIdProvider, vertexLabelProvider, edgeLabelProvider);
		BufferedWriter writer = new BufferedWriter(new FileWriter(fileName));
		exporter.exportGraph(paraExecAutomaton, writer);

		//System.out.println(writer.toString());
    }
	
	/** Main function for trial*/
	public static void main(String args[]) throws ExportException {
		
	}
}

/**Define a class for the edge of parallel execution of subtask 
 * allocation automata**/
class EdgeOfParaExec 
	extends
	DefaultEdge {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	public ArrayList<EdgOfProdAut> transEdge = new ArrayList<EdgOfProdAut>();
	public double weight;
	
	public EdgeOfParaExec() {};
	
	public EdgeOfParaExec(ArrayList<EdgOfProdAut> transEdge, double weight){
		this.transEdge = transEdge;
		this.weight = weight;
	}
}
