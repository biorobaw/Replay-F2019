package com.github.biorobaw.scs_models.replay_f2019.old.model.modules.d_action;

import java.util.Arrays;

import com.github.biorobaw.scs.utils.math.Floats;

public class MotionBias {
	int bias_method = 0;
	int numActions;	
	float[] biases;
	float[] probabilities;	
	
	
	float[][][] precalculatedBias;
	
	
	// parameters
	float exponent;
	float uniformProbability;
	
	int previous_action;
	
	private float uniform_value;
	
	
	/**
	 * Add bias based on the concept of congruent actions 
	 * (actions whose angles defer by less than 90 degrees).
	 * If last 2 actions were congruent, then actions not congruent with any get minimum probability.
	 * Actions congruent with both get additional probability based on its angle angle bisector 
	 * between the last two actions.
	 * If last 2 actions were not congruent, then the process is the same but considering only the last action. 
	 * @param numActions
	 * @param exponent Defines the exponent in equation (1 - relativeAngle/pi)^exponent
	 * @param uniformProbability minimum probability of each action
	 */
	public MotionBias(int numActions,float exponent, float uniformProbability) {
		this.numActions = numActions;
		uniform_value = 1f/numActions;
		probabilities = new float[numActions];
		biases = new float[numActions];
		precalculatedBias = new float[numActions][numActions][numActions];
		

		this.exponent = exponent;
		this.uniformProbability = uniformProbability;
		
		float base = uniformProbability/numActions;
		float complement = 1-uniformProbability;
		
		
		// PRECALCULATE BIAS (b[[j] = precalc[a(t-1)][a(t-2)][j])
		float dTheta = 2*(float)Math.PI/numActions;
		float theta = 0;
		for(int i=0;i<numActions;i++){
			
			float theta2 = 0;
			for(int j=0;j<numActions;j++){
				
				
				
				float sum=0;
								
				// obs: by substracting pi in and out of the modulo operation the relative angle remains the same,
				// except the result is now between -pi and pi
				float relAngle = relativeAngle(theta2, theta);
//				System.out.println("theta,theta2,rel: "+ theta + " " + theta2 + " " + relAngle);
				
				if ( relAngle < -Math.PI*0.5  || Math.PI*0.5 < relAngle  ){
					//If new action defers from old one by more than 90 degrees
					float ang = theta; 
					
					float theta3 = 0;
					
					for(int k=0;k<numActions;k++){
						
						float rel = (float)Math.abs(relativeAngle(theta3, ang));
						if(rel > Math.PI*0.5) precalculatedBias[i][j][k]=0;
						else {
							precalculatedBias[i][j][k] = (float)Math.pow(1-rel/(float)Math.PI,exponent);
							sum+=precalculatedBias[i][j][k];
						}
						theta3+=dTheta;
					}
					
				}else{
					float ang = theta + relAngle/2;
					float theta3 = 0;
					
					for(int k=0;k<numActions;k++){
						
						float rel = Math.abs(relativeAngle(theta3, ang));
						float rel2= Math.abs(relativeAngle(theta3, theta));
						float rel3= Math.abs(relativeAngle(theta3, theta2));

						if(rel2 > Math.PI*0.5 || rel3 > Math.PI*0.5 ) precalculatedBias[i][j][k]=0;
						else {
							precalculatedBias[i][j][k] = (float)Math.pow(1-rel/(float)Math.PI,exponent);
							sum+=precalculatedBias[i][j][k];
						}
						
						theta3+=dTheta;
					}
					
				}
				
				for (int k=0;k<numActions;k++)
					precalculatedBias[i][j][k] = base + complement*precalculatedBias[i][j][k]/sum;
				
				theta2+=dTheta;
			}
			
			
			theta += dTheta;
		}
		
//		for(int i=0; i<numActions; i++)
//			for(int j=0; j<numActions; j++)
//				System.out.println("bias ("+i+","+j+"): " + Arrays.toString(precalculatedBias[i][j]));
				
		
		
		

		
		
	}
	
	
	public float[] calculateBias(int action) {
		if(bias_method == 2) {
			for(int i=0;i<numActions;i++)
				biases[i] = precalculatedBias[action][previous_action][i];
			previous_action = action;
		} else if(bias_method == 1) {
			for(int i=0;i<numActions;i++)
				biases[i] = precalculatedBias[action][action][i];
			previous_action = action;
			bias_method = 2;
		} else {
			for(int i=0; i<numActions; i++) {
				biases[i] = uniform_value;
			}
			bias_method = 1;
		}
		return biases;
	}
	
	
	public float[] addBias(int lastAction, float[] p_input) {
				
		calculateBias(lastAction);

		Floats.mul(biases, p_input,probabilities);
		var sum = Floats.sum(probabilities);
		
		if(sum!=0) Floats.div(probabilities, sum, probabilities);
		else {
			System.err.println("WARNING: Probability sum is 0, setting uniform distribution (MotionBias.java)");
			for(int i=0; i<numActions; i++) probabilities[i] = uniform_value;
		}
				
		return probabilities;
	}
	
	
	public void newEpisode(){
		bias_method = 0;
	}
	
	public void newTrial(){
	}
	
	public float[] getBias() {
		return biases;
	}
	
	public float[] getProbabilities() {
		return probabilities;
	}
	

	/**
	 * Returns the directed angle in radians between the angle and the base
	 * @param angle
	 * @param base
	 * @return
	 */
	static final float pi2 = (float)(2*Math.PI);
	public float relativeAngle(float angle, float base) {
		// note modulo returns a value between 0 and 2pi
		// thus, we subtract pi outside the modulo operation to shift the result range to [-pi, pi]
		// and we subtract inside the modulo operation so that the result remain unchanged via modulo
		float delta = (float)((angle - base)%pi2);
		if(delta <= -Math.PI) return delta + pi2;
		if(delta > Math.PI) return delta - pi2;
		return delta;
	}
	
	
}


