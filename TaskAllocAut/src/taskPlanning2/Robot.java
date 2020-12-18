package taskPlanning2;

import java.util.ArrayList;
import java.util.List;

import org.jgrapht.Graph;

public class Robot 
	extends RobotTransitionSystem {
	
	List<RobotAction> ActionList;
	
	public Robot(List<RobotAction> ActionList, Graph<ArrayList<String>, TSEdge> tsGraph, 
			ArrayList<String> initState){
		this.ActionList = ActionList;
		
		this.tsGraph = tsGraph;
		this.initSate = initState;
	}
	
	public Robot() {};
	
	public List<String> eventList(){
		List<String> eventList = new ArrayList<>();
		
		for (RobotAction robotAction : ActionList)
			eventList.add(robotAction.event);
		
		return eventList;
	}
	
	public int getID(){
		return ActionList.get(0).robotID;
	}
}
