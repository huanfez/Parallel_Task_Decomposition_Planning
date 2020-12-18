//package taskPlanning2;
//
//import com.mxgraph.layout.*;
//import com.mxgraph.swing.*;
//import com.mxgraph.util.mxConstants;
//
//import dk.brics.automaton.Automaton;
//import dk.brics.automaton.RegExp;
//import dk.brics.automaton.SubAutomataExtract;
//import dk.brics.automaton.decompAut;
//import taskPlanning2.EdgOfProdAut;
//import taskPlanning2.EdgeOfParaExec;
//import taskPlanning2.ParallelExecutionAutomaton;
//import taskPlanning2.ParallelTaskAllocAut;
//import taskPlanning2.SubtaskAllocationAut;
//import taskPlanning2.TaskAllocAutomaton;
//import taskPlanning2.VtxOfProdAut;
//
//import org.jgrapht.*;
//import org.jgrapht.ext.*;
//import org.jgrapht.graph.*;
//
//import javax.swing.*;
//import java.awt.*;
//import java.util.ArrayList;
//import java.util.HashSet;
//import java.util.List;
//import java.util.Set;
//
///**
// * A demo applet that shows how to use JGraphX to visualize JGraphT graphs. Applet based on
// * JGraphAdapterDemo.
// *
// */
//public class JGraphXAdapterDemo
//    extends
//    JApplet
//{
//    private static final long serialVersionUID = 2202072534703043194L;
//
//    private static final Dimension DEFAULT_SIZE = new Dimension(530, 320);
//
////    private JGraphXAdapter<String, DefaultEdge> jgxAdapter;
//    private JGraphXAdapter<ArrayList<String>, DefaultEdge> jgxAdapter;
//    private JGraphXAdapter<String,AutomatonEdge> jgxAdapter1;
//    /**
//     * An alternative starting point for this demo, to also allow running this applet as an
//     * application.
//     *
//     * @param args command line arguments
//     */
//    public static void main(String[] args)
//    {
//        JGraphXAdapterDemo applet = new JGraphXAdapterDemo();
//        applet.init();
//        
//        JFrame frame = new JFrame();
//        frame.getContentPane().add(applet);
//        frame.setTitle("JGraphT Adapter to JGraphX Demo");
//        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
//        frame.pack();
//        frame.setVisible(true);
//    }
//
//    @Override
//    public void init()
//    {
//        // create a JGraphT graph       
//		Graph<ArrayList<String>, DefaultEdge> parallelTS12 = SubtaskAllocationAut.ParallelComposTS(
//				SubtaskAllocationAut.transitionSystem1, SubtaskAllocationAut.transitionSystem2);
//		Graph<ArrayList<String>, DefaultEdge> parallelTS123 = 
//				SubtaskAllocationAut.ParallelComposTS(parallelTS12, SubtaskAllocationAut.transitionSystem3);
//
//        // create a visualization using JGraph, via an adapter
//		jgxAdapter = new JGraphXAdapter<>(parallelTS123);
//        setPreferredSize(DEFAULT_SIZE);
//        mxGraphComponent component = new mxGraphComponent(jgxAdapter);
//        component.setConnectable(false);
//        component.getGraph().setAllowDanglingEdges(false);
//        component.getGraph().getStylesheet().getDefaultEdgeStyle().put(mxConstants.STYLE_NOLABEL, "1");
//        
//        ///////////////////////////////////////////////////////////////////////////////////////////////
//		/*1. give a regular expression, output sub-automata*/
//		RegExp regExpr = new RegExp("e(a(cb|b(c|d))|b(da|a(c|d)))");
//		DirectedPseudograph<String,AutomatonEdge> gTaskAutomatonGraph = SubtaskAllocationAut.GraphTaskAut(
//				regExpr.toAutomaton());
//		jgxAdapter1 = new JGraphXAdapter<>(gTaskAutomatonGraph);
//        mxGraphComponent component1 = new mxGraphComponent(jgxAdapter1);
//        component1.setConnectable(false);
//        component1.getGraph().setAllowDanglingEdges(false);
//		
//		/*2. Declare cooperative events**/
//		Set<Character> CoopEventSet = new HashSet<Character>();
//		CoopEventSet.add('e');
//
//		/* 3. Obtain sub-automaton, of which all paths have common events **/
//		Set<Automaton> SubautomatonSet= SubAutomataExtract.commonEventsAut(regExpr.toAutomaton());
////		//System.out.print(AutomatonSet);
//		
//		/* 4. Generate parallel decompositions for each sub-automaton**/
//		List<List<Automaton>> decompositionSet = new ArrayList<List<Automaton>>();
//		for (Automaton automaton : SubautomatonSet)
//			decompositionSet.add(decompAut.paraDecompC(automaton, CoopEventSet));
//		
//		/** 5. For each decomposition set, take product automaton with transition system to 
//		 * get the subtask allocation automaton **/
//		//TaskAllocAutomaton subtaskAllocAutomaton = new TaskAllocAutomaton();
//		List<TaskAllocAutomaton> subtaskAllocAutomatonList;
//		TaskAllocAutomaton subtaskAllocAutomatonT;
//		
//		ParallelExecAutomaton paraExecAutomaton;
//		GraphPath<ArrayList<VtxOfProdAut>, EdgeOfParaExec> shortPathOfParaExecAutomaton;
////		DefaultDirectedWeightedGraph<ArrayList<VtxOfProdAut>, EdgeOfParaExec> paraExecAutomatonGraph;
//		
//		for (List<Automaton> automatonDecomps : decompositionSet) {
//			subtaskAllocAutomatonList = new ArrayList<TaskAllocAutomaton>();
//			
//			for (Automaton subtaskAutomaton : automatonDecomps) {
//				System.out.println("---------------------------");
//				System.out.println("Subtask automaton is:" + subtaskAutomaton);
//				subtaskAllocAutomatonT = new TaskAllocAutomaton();
//				//subtaskAllocAutomatonT = genTaskAllocationAutomaton(transitionSystemP, subtaskAutomaton, 
//				//		robotActionSets);
////				subtaskAllocAutomatonT = SubtaskAllocationAut.genTaskAllocationAutomaton(parallelTS123, 
////						subtaskAutomaton, SubtaskAllocationAut.robotActionSets);
//				subtaskAllocAutomatonList.add(subtaskAllocAutomatonT);
//				
//				System.out.println("Subtask allocation automaton vertex set is:");
//				for (VtxOfProdAut vortex : subtaskAllocAutomatonT.taskAllocAutGraph.vertexSet())
//					System.out.println(vortex.TsState + "," + vortex.AutState);
//				
//				//break;
//			}
//			
//			/**6. Parallel execution of subtask allocation automaton**/
//			System.out.println("Parallel execution of subtask allocation automata: ");
////			paraExecAutomaton = ParallelExecutionAutomaton.parallelExec(subtaskAllocAutomatonList.get(0), 
////					subtaskAllocAutomatonList.get(1));
//			
//			shortPathOfParaExecAutomaton = ParallelExecutionAutomaton.lowestCostPath(paraExecAutomaton);
//			for (EdgeOfParaExec edge : shortPathOfParaExecAutomaton.getEdgeList()) {
//				for (EdgOfProdAut element : edge.transEdge)
//					System.out.print(element.action + "," + element.robotID + ";");
//				System.out.println("end");
//			}
//		}
//        
//        
//        getContentPane().setLayout(new GridLayout(1,2));
//        getContentPane().add(component);
//        getContentPane().add(component1);
//        resize(DEFAULT_SIZE);
//        
//        // positioning via jgraphx layouts
//        mxCircleLayout layout = new mxCircleLayout(jgxAdapter);
//        // center the circle
//        int radius = 100;
//        layout.setX0((DEFAULT_SIZE.width / 2.0) - radius);
//        layout.setY0((DEFAULT_SIZE.height / 2.0) - radius);
//        layout.setRadius(radius);
//        layout.setMoveCircle(true);
//        layout.execute(jgxAdapter.getDefaultParent());
//        
//        mxCircleLayout layout1 = new mxCircleLayout(jgxAdapter1);
//        layout1.setX0((DEFAULT_SIZE.width / 2.0) - radius);
//        layout1.setY0((DEFAULT_SIZE.height / 2.0) - radius);
//        layout1.setRadius(radius);
//        layout1.setMoveCircle(true);
//        layout1.execute(jgxAdapter1.getDefaultParent());
//    }
//    
//}
