# Parallel_Task_Decomposition_Planning

The repository is for "Parallel Task Decomposition and Concurrent Specification Satisfaction for Heterogeneous Multi-Robot Symbolic Motion Planning". It contains the packages 
- **Decomposition** (Parallel decompose the finite words of deterministic automaton)
- **TaskAllocAut** (Assign robots to atomic tasks of task automaton; generate the subtask planning automaton (SPA) and search for the optimal task planning solution)

Note: These packages need JAVA 8.0+ library. It compiles and runs well with Eclipse IDE. 

- **mrs_taskplan** (ROS package. Deploy robots to the generated task plans; search for the paths in the discrete environment so that the task plans are satisfied.)
