package taskPlanning2;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.io.ComponentNameProvider;
import org.jgrapht.io.DOTExporter;
import org.jgrapht.io.ExportException;
import org.jgrapht.io.GraphExporter;

public class RobotTransitionSystem {
	
	public List<Integer> idList;
	public ArrayList<String> initSate = new ArrayList<>();
	public Graph<ArrayList<String>, TSEdge> tsGraph = new DefaultDirectedGraph<>(TSEdge.class);
	
	public RobotTransitionSystem() {}
	public RobotTransitionSystem(List<Integer> idList, ArrayList<String> initSate, 
			Graph<ArrayList<String>, TSEdge> tsGraph) 
	{
		this.idList = idList;
		this.initSate = initSate;
		this.tsGraph = tsGraph;
	}

	/** Parallel composition of transition system**/
	public static RobotTransitionSystem transitionSystemComposition(Set<Robot> robotSubSets,
			ArrayList<String> CoopEventSet) {
		
		List<Robot> robotSubList = new ArrayList<>();
		robotSubList.addAll(robotSubSets);
		
		Robot robot0 = robotSubList.get(0);
		//System.out.println("robot is:" + robot0.tsGraph.vertexSet() + "," + robotSubList.get(1).tsGraph.vertexSet());
		RobotTransitionSystem composedTranSystem = new RobotTransitionSystem(Arrays.asList(
				robot0.getID()), robot0.initSate, robot0.tsGraph);
		
		if (robotSubList.size() > 1)
			for (int index = 1; index < robotSubList.size(); index++) {
				
				Graph<ArrayList<String>, TSEdge> graphOfComposedTS = RobotTransitionSystem.ParallelComposTS(
						composedTranSystem.tsGraph, robotSubList.get(index).tsGraph, CoopEventSet);
				
				ArrayList<String> initOfComposedTS = new ArrayList<>();
				initOfComposedTS.addAll(composedTranSystem.initSate);
				initOfComposedTS.addAll(robotSubList.get(index).initSate);
				
				List<Integer> tsIdList = new ArrayList<>(composedTranSystem.idList);
				tsIdList.add(robotSubList.get(index).getID());
				
				composedTranSystem.tsGraph = graphOfComposedTS;
				composedTranSystem.initSate = initOfComposedTS;
				composedTranSystem.idList = tsIdList;
			}
		//System.out.println("Composed transition system is" + composedTranSystem.tsGraph);
		
		return composedTranSystem;
	}
	
	/** Parallel compose graph of transition system */
	@SuppressWarnings("serial")
	public static Graph<ArrayList<String>, TSEdge> ParallelComposTS(Graph<ArrayList<String>, TSEdge> 
			TS1, Graph<ArrayList<String>, TSEdge> TS2, ArrayList<String> CooperativeEventSet){
		
		Graph<ArrayList<String>, TSEdge> parallelTS = new DefaultDirectedGraph<>(TSEdge.class);
		
		/* * Successor set for all vertices of each transition system **/
		////NeighborCache<ArrayList<String>,String> neighborsOfTS1 = new NeighborCache<>(TS1);
		////NeighborCache<ArrayList<String>,String> neighborsOfTS2 = new NeighborCache<>(TS2);
		
		/* * Synthesize parallel composed vertices set and, 
		 * add transition edges at the same time**/
		for (ArrayList<String> vertexOfTS1 : TS1.vertexSet())
			for (ArrayList<String> vertexOfTS2 : TS2.vertexSet()) {
				
				/* * Can't allow the composite state that has 
				 * the mutual exclusion **/
				if (!IsQualifiedVertex(vertexOfTS1, vertexOfTS2, CooperativeEventSet))
					continue;
				
				/* * Add vertex into graph **/
				ArrayList<String> currentVertex = new ArrayList<String>() {{
					addAll(vertexOfTS1);
					addAll(vertexOfTS2);
					}};
				
				parallelTS.addVertex(currentVertex);
			}
				
		for (ArrayList<String> strtVertex : parallelTS.vertexSet()) {
			
			ArrayList<String> strtVertexOfTS1 = new ArrayList<String>(strtVertex.subList(0, strtVertex.size()-1));
			ArrayList<String> strtVertexOfTS2 = new ArrayList<String>(strtVertex.subList(strtVertex.size() - 1, 
					strtVertex.size()));
		
			for (ArrayList<String> destVertex : parallelTS.vertexSet()) {
				/* * Verify all the possible successors of current 
				 * composite state**/
	
				ArrayList<String> destVertexOfTS1 = new ArrayList<String>(destVertex.subList(0, destVertex.size()-1));
				ArrayList<String> destVertexOfTS2 = new ArrayList<String>(destVertex.subList(destVertex.size() - 1, 
						destVertex.size()));
				
				TSEdge ts1Edge = TS1.getEdge(strtVertexOfTS1, destVertexOfTS1);
				TSEdge ts2Edge = TS2.getEdge(strtVertexOfTS2, destVertexOfTS2);
				
//				System.out.println("edge+1");
				// For successors of transition system 1,2,1 & 2
				if (ts1Edge != null && ts2Edge != null && ts1Edge.process.equals(ts2Edge.process)) {
					if (CooperativeEventSet.contains(ts1Edge.process))
						parallelTS.addEdge(strtVertex, destVertex, new TSEdge(ts1Edge.process));
				}
				else if (ts1Edge != null && strtVertexOfTS2.equals(destVertexOfTS2)) {
					if (!CooperativeEventSet.contains(ts1Edge.process))
						parallelTS.addEdge(strtVertex, destVertex, new TSEdge(ts1Edge.process));
				}
				else if (ts2Edge != null && strtVertexOfTS1.equals(destVertexOfTS1)) {
					if (!CooperativeEventSet.contains(ts2Edge.process)) 
						parallelTS.addEdge(strtVertex, destVertex, new TSEdge(ts2Edge.process));
				}
			}
		}
		
		return parallelTS;
	}
	
	/** Boolean - evaluate if a vertex is qualified in a parallel composition */
	public static Boolean IsQualifiedVertex(ArrayList<String> vertexOfTS1, ArrayList<String> vertexOfTS2, 
			ArrayList<String> CooperativeEventSet) {
		ArrayList<String> commonAction = new ArrayList<String>();
		
		/* * interaction of states of transition system 1 and 2**/
		commonAction.addAll(vertexOfTS1);
		commonAction.retainAll(vertexOfTS2);
		
		/* * not qualified - if interactions exist and, are not cooperative action **/
		if (!commonAction.isEmpty() && !CooperativeEventSet.containsAll(commonAction))
			return false;
		else
			return true;
	}
	
	/** Output the parallel transition system graph in .dot format */
    public static void renderTSGraph(Graph<ArrayList<String>, TSEdge> TS, String fileName)
    		throws ExportException, IOException
    {
		ComponentNameProvider<ArrayList<String>> vertexLabelProvider = new ComponentNameProvider<>(){
		    public String getName(ArrayList<String> vertex)
		    {
		        return vertex.toString();
		    }
		};
		        
		ComponentNameProvider<ArrayList<String>> vertexIdProvider = new ComponentNameProvider<>() {
            public String getName(ArrayList<String> vertex)
            {
            	String listString = "";

            	for (String s : vertex)
            	{
            	    listString += s;
            	}
                return listString;
            }
        };
		GraphExporter<ArrayList<String>,TSEdge> exporter = new DOTExporter<>(vertexIdProvider, 
				vertexLabelProvider, null);
		BufferedWriter writer = new BufferedWriter(new FileWriter(fileName));
		exporter.exportGraph(TS, writer);
		//System.out.println(writer.toString());
	}
	
}

/** Define the labeled edge, allowing same name */
class TSEdge 
	extends
	DefaultEdge	{
	
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	public String process;
	
	public TSEdge(String Event) {
		this.process = Event;
	}
}
