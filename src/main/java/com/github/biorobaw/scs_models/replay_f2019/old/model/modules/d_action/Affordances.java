package com.github.biorobaw.scs_models.replay_f2019.old.model.modules.d_action;



import com.github.biorobaw.scs.robot.Robot;

/**
 * Gets
 * @author biorob
 * 
 */
public class Affordances  {
	
	// array to store affordances
	public float[] affordances;
	float threshold_distance;
	private int numActions;
	

	public Affordances(Robot robot, int numActions, float threshold_distance ){
		affordances =new float[numActions];
		this.numActions = numActions;
		this.threshold_distance = threshold_distance;		
	}
	
	
	public float[] calculateAffordances(float[] distances) {
		for(int i=0;i<numActions;i++) 
			affordances[i] = distances[i] > threshold_distance ? distances[i] : 0;
		return affordances;
	}
	
	
	
	
	
}
