package com.github.biorobaw.scs_models.replay_f2019.old.model;

import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;


import com.github.biorobaw.scs.experiment.Experiment;
import com.github.biorobaw.scs.experiment.Subject;
import com.github.biorobaw.scs.maze.mazes.MultipleT;
import com.github.biorobaw.scs.robot.commands.TranslateXY;
import com.github.biorobaw.scs.robot.modules.FeederModule;
import com.github.biorobaw.scs.robot.modules.distance_sensing.DistanceSensingModule;
import com.github.biorobaw.scs.robot.modules.localization.SlamModule;
import com.github.biorobaw.scs.utils.files.XML;
import com.github.biorobaw.scs.utils.math.DiscreteDistribution;
import com.github.biorobaw.scs.utils.math.Floats;
import com.github.biorobaw.scs.utils.math.RandomSingleton;
import com.github.biorobaw.scs_models.replay_f2019.old.gui.GUI;
import com.github.biorobaw.scs_models.replay_f2019.old.model.modules.b_state.PlaceCellBins;
import com.github.biorobaw.scs_models.replay_f2019.old.model.modules.b_state.PlaceCells;
import com.github.biorobaw.scs_models.replay_f2019.old.model.modules.d_action.Affordances;
import com.github.biorobaw.scs_models.replay_f2019.old.model.modules.d_action.MotionBias;
import com.github.biorobaw.scs_models.replay_f2019.old.model.modules.utils.SparseMatrix;


public class Model extends Subject{

	static final public float foodReward = 1;

	// Model Parameters
	// action space params
	public int num_actions;

	// state space params
	public int num_cells;
	public float pcRadius;
	public float pc_bin_size;

	// RL params
	public float discountFactor;
	public float learningRate;

	// action selection params
	public float minDistance; // min distance to consider an action as possible

	// replay params
	public float wTransitionLR; // learning rate of the transition matrix
	public int num_replay; // number of replay episodes per task episode
	public float propagation_threshold;
	public float asleep_reward_distance;
	
	
	// Required Robot Modules
	public SlamModule slam;
	public FeederModule feederModule;
	public DistanceSensingModule distance_sensors;
	
	// Model Variables: State
	public PlaceCells place_cells;
	public PlaceCellBins pc_bins;
	
	public PlaceCells active;
	public float[] oldActivations;
	public int[] oldIds;
	HashMap<Integer, Integer> oldIdHash;
	
//	public EligibilityTraces vTraces;
//	public EligibilityTraces qTraces;
	
	// Model Variables: RL
	public float[] vTable;
	public float[][] qTable;
	public float[] qValues;
	public Float oldStateValue = null;
	
	
	
	// Model Variables: Action Selection
	public float[] softmax;
	public Affordances affordances;
	public float[] possible;
	public MotionBias motionBias;
	public int chosenAction;
	public boolean actionWasOptimal = false;
	
	// Model Variables: Replay
	SparseMatrix wTable;
	boolean doReplay = false;
	boolean foodInReplay =false;
	float feeding_x;
	float feeding_y;
	
	
	// GUI
	GUI gui;
	
	
	
	public Model(XML xml) {
		super(xml);
		
		// ======== PARAMETERS ===============
		
		// Get parameters frorm xml file
		num_actions    = xml.getIntAttribute("num_actions");

		num_cells	   = xml.getIntAttribute("num_cells");
		pcRadius 	   = xml.getFloatAttribute("PCRadius");
		pc_bin_size = xml.getFloatAttribute("pc_bin_size");

		discountFactor = xml.getFloatAttribute("discountFactor");
		learningRate   = xml.getFloatAttribute("v_learningRate");

		minDistance	 	= xml.getFloatAttribute("step");
		
		
		
		wTransitionLR  = xml.getFloatAttribute("wTransitionLR");
		num_replay 	   = xml.getIntAttribute("num_replay");
		propagation_threshold = xml.getFloatAttribute("propagation_threshold");
		asleep_reward_distance= xml.getFloatAttribute("asleep_reward_distance");
		// create place cells
		
		// ======== Model Input ====================
		
		// get robot modules
		slam = robot.getModule("slam");
		feederModule = robot.getModule("FeederModule");
		distance_sensors = robot.getModule("distance_sensors");
		
		// ======== Model Variables: STATE AND RL ==========
		var maze = (MultipleT)Experiment.get().getMaze();
		place_cells = new PlaceCells(num_cells, pcRadius,  ()->maze.getRandomPosition());
		pc_bins = new PlaceCellBins(place_cells, pc_bin_size);
		
		vTable = new float[num_cells];
		qTable = new float[num_cells][num_actions];
		
		// ======= Model Variables: ACTION SELECTION =======
		affordances = new Affordances(robot, num_actions, minDistance);
		motionBias = new MotionBias(num_actions, 1.5f, 0.001f);
		softmax = new float[num_actions];
		possible = new float[num_actions];
		
		// ======= Model Variables: REPLAY =================
		wTable = new SparseMatrix(num_cells);
		
		// ======= GUI =====================================
		gui = new GUI(this);
	}
	
	
	@Override
	public long runModel() {
		
		// get inputs
		var pos = slam.getPosition();
		float[] distances= distance_sensors.getDistances();
		boolean robot_ate = feederModule.ate();
		float reward =  robot_ate ? foodReward : 0f;
		
		// set flag if robot found food:
		foodInReplay |= robot_ate;
		
		// calculate state:
		calculateActivePCs((float)pos.getX(), (float)pos.getY());
		
		// perform rl
		updateRL(reward);
		
		// update transition matrix
		updateW();
		
		// perform action selection
		performAwakeActionSelection(distances);
		
		return 0;
	}
	
	
	@Override
	public void newEpisode() {
		super.newEpisode();
		motionBias.newEpisode();		
		clearState();
				
	}
	
	public void clearState() {
		active=null;
		oldIdHash = null;
		pc_bins.clear();
		oldStateValue = null;
		actionWasOptimal = false;
		chosenAction = -1;
	}
	
	@Override
	public void endEpisode() {
		super.endEpisode();
		MultipleT maze = (MultipleT)Experiment.get().getMaze();
		if(maze.feeders.size()>0 && doReplay) {
			System.out.println("foodInReplay: " + foodInReplay);
			var f = maze.feeders.get(1);
//			System.out.println("feeders: "+maze.feeders.size());
//			System.out.println("f: " + f);
			var x = (float)f.pos.getX();
			var y = (float)f.pos.getY();
			
			for(int i=0; i<num_replay; i++) {
				replayEpisode(x,y);
			}
			
		}
	}
	
	@Override
	public void newTrial() {
		super.newTrial();
		motionBias.newTrial();
	}
	
	@Override
	public void endTrial() {
		super.endTrial();
	}
	
	@Override
	public void newExperiment() {
		super.newExperiment();
	}
	
	@Override
	public void endExperiment() {
		super.endExperiment();
		try {
			System.in.read();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public void calculateActivePCs(float x, float y) {
		
		// first store old values:
		if(active!=null) {
			oldIds = active.ids;
			oldActivations = Floats.copy(active.ns);
			oldIdHash = active.idHash;
		}
		
		// now calculate pcs for current time
		float activity = pc_bins.activateBin(x,y);
		active = pc_bins.active_pcs;
		if(activity != 0) active.normalize(activity);
		else {
			var e = Experiment.get();
			System.err.println("WARNING: no active pcs Trial-Episode-cycle: " 
					+ e.getGlobal("trial") + " " 
					+ e.getGlobal("episode") + " " 
					+ e.getGlobal("cycle"));
		}
	}
	
	public float calculateV(PlaceCells pcs) {
		float value = 0;
		for(int i=0; i<pcs.num_cells; i++)
			value+=vTable[pcs.ids[i]]*pcs.ns[i];
		// TODO: old code thresholded the value since value diverged
//		System.out.println("V: " + value);
		value = threshold(value, -10000, 10000);
//		System.out.println("VT: " + value);
		return value;
	}
	
	
	public float[] calculateQ(PlaceCells pcs, float[] res) {
//		System.out.println("pcs: " + Arrays.toString(pcs.ns));
		for(int i=0; i< pcs.num_cells; i++) {
			var activation = pcs.ns[i];
			var q_pc = qTable[pcs.ids[i]];
//			System.out.println("qpc: " + Arrays.toString(q_pc));

			for(int j=0; j<num_actions; j++)
				res[j]+= q_pc[j]*activation;
				
		}
		// TODO: old code thresholded q values since they diverged
		for(int i=0; i< num_actions; i++)
			res[i] = threshold(res[i], -10000, 10000);
		return res;
	}
	

	public float[] calculateQ(PlaceCells pcs) {
		return calculateQ(pcs, new float[num_actions]);
	}
	
	
	/**
	 * Performs the RL update to vTable and qTable.
	 * Requires the following global variables to be set accordingly:
	 *  -oldStateValue
	 *  -actionWasOptimal
	 *  - ( oldIds and old activation ) or TODO: traces
	 * @param reward
	 */
	public void updateRL(float reward) {
		
		if(oldStateValue!=null) {
			//calculate bootstrap:
			// TODO: old model calculates next state value even in terminal states
			float bootstrap = reward;
//			if(reward == 0) 
				bootstrap += discountFactor * calculateV(active); 
			// TODO: old code forced bootstrap to range [-10000, 10000]
			bootstrap = threshold(bootstrap, -10000, 10000);
			
			// calculate rl error:
//			System.out.println("boot: " + bootstrap + " " + oldStateValue);
			float error = bootstrap - oldStateValue;
			float errorLR = error*learningRate;
			
			// update V
			// TODO: old code didn't update V if action was not optimal or error < 0
			var update = actionWasOptimal || error > 0;
			update = true;
//			System.out.println("update: " + actionWasOptimal + " " + error);
			if(update) {
				for(int i=0; i < oldIds.length; i++){
					vTable[oldIds[i]]+= errorLR*oldActivations[i];
				}
			}
			
			// update Q
			// TODO: old code didn't update Q if action was not optimal or error < 0
			if(update)
				for(int i=0; i < oldIds.length; i++){
					qTable[oldIds[i]][chosenAction]+=errorLR*oldActivations[i];
				}
		}
		oldStateValue = calculateV(active);			
		
	}
	

	
	/**
	 * Updates transition matrix W
	 * Requires the following global variables to be set accordingly:
	 *   -active
	 *   -oldIdHash
	 *   -oldActivations
	 */
	public void updateW() {
		
		// if oldIdHash has not been set there is no previous state, thus return
		if(oldIdHash == null) return;
		
		// update wTable
		HashSet<Integer> nonZeroPcs = new HashSet<>(oldIdHash.size()+active.num_cells);
		nonZeroPcs.addAll(oldIdHash.keySet());
		nonZeroPcs.addAll(active.idHash.keySet());
		
		for(var id_i : nonZeroPcs) {
			Integer old_i = oldIdHash.get(id_i);
			Integer new_i = active.idHash.get(id_i);

			var old_pci = old_i == null ? 0 : oldActivations[old_i];
			var new_pci = new_i == null ? 0 : active.ns[new_i];
			
			
			for(var id_j : nonZeroPcs) {
				if(id_j == id_i) continue;
				Integer old_j = oldIdHash.get(id_j);
				Integer new_j = active.idHash.get(id_j);
				
				var old_pcj = old_j == null ? 0 : oldActivations[old_j];
				var new_pcj = new_j == null ? 0 : active.ns[new_j];
				
				
				float val = wTable.get(id_i, id_j);
				val += Math.atan((old_pci + new_pci)/2 * (new_pcj - old_pcj));
				wTable.set(id_i, id_j, val);
				
			}
		}
	}

	
	
	
	public void performAwakeActionSelection(float[] distances) {
		// first calculate probability of each action
		qValues = calculateQ(active);
		Floats.softmax(qValues, softmax);
		
//		System.out.println("awake action selection");
//		System.out.println("Q: " + Arrays.toString(qValues));
		
		// TODO: using old affordance system
		var aff_values = affordances.calculateAffordances(distances);
//		System.out.println("AFF: " + Arrays.toString(aff_values));
		
		Floats.mul(softmax, aff_values, possible);
		var p_sum = Floats.sum(possible);
		if(p_sum == 0 ) Floats.div(aff_values, Floats.sum(aff_values), possible);
		else Floats.div(possible, p_sum, possible);
		
//		System.out.println("possible: " + Arrays.toString(possible));
		
		// TODO: using old bias system
		var biased = motionBias.addBias(chosenAction, possible);
		chosenAction = DiscreteDistribution.sample(biased);
		actionWasOptimal = biased[chosenAction] == Floats.max(biased);
		
//		System.out.println("bias: " + Arrays.toString(motionBias.getBias()));
//		System.out.println("biased: " + Arrays.toString(biased));
		
		// perform action
		double tita = 2*Math.PI/num_actions*chosenAction;
		robot.getRobotProxy().send_command(new TranslateXY(0.08f*(float)Math.cos(tita), 0.08f*(float)Math.sin(tita)));
		feederModule.eatAfterMotion();
		
		
	}
	
	
	
	
	public float threshold(float value, float min, float max) {
		return value < min ? min : value > max ? max : value;
	}
	
	
	
	
	public void setDoReplay(boolean value) {
		doReplay = value;
	}
	
	public void replayEpisode(float feeding_x, float feeding_y) {
		
		int propagations = 0;
		
		final double base = (float)Math.PI/num_actions;
		final double pi2 = 2*(float)Math.PI;
		final double angle_between_actions = pi2/num_actions;
		
		// clear state before starting a new replay episode
		clearState();
		
		// activate a random place cell
		int activePC = RandomSingleton.getInstance().nextInt(num_cells);
		HashSet<Integer> visited = new HashSet<>();
		visited.add(activePC);
		float old_x = place_cells.xs[activePC];
		float old_y = place_cells.ys[activePC];
		calculateActivePCs(old_x, old_y);
		
		// propagate activation and update RL until cant propagate any more
		// or food site is reached
		while( (activePC = propagateActivationToMax(activePC, visited)) != -1 ) {
			
			propagations++;
			
			float new_x = place_cells.xs[activePC];
			float new_y = place_cells.ys[activePC]; 
			
			double angle = Math.atan2(new_y-old_y, new_x-old_x) + base;
			if(angle < 0) angle+=pi2;
			
			chosenAction = (int)Math.floor(angle / angle_between_actions);
			
			calculateActivePCs(place_cells.xs[activePC], place_cells.ys[activePC]);
			
			// check reward
			float reward = 0;
			if(foodInReplay) {
				float dx = new_x-feeding_x;
				float dy = new_y-feeding_y;
				float d2 = dx*dx + dy*dy;
				if( d2 < asleep_reward_distance) {
					reward = foodReward;
					System.out.println("Reward!!!");
				}
			}
			
			// TODO: old model always sets "actionWasOptimal" to true during replay
			actionWasOptimal = true;
			
			// need to decide if give reward or not
			updateRL(reward);
			
			old_x = new_x;
			old_y = new_y;
			
			if(reward!=0) break;
		}
		
		System.out.println("Propagations: " + propagations);
		
	}
	
	public int propagateActivationToMax(int activePC, HashSet<Integer> visited) {
		var row = wTable.getRow(activePC);
		
		float maxVal = propagation_threshold;
		int maxID = -1;
		for(var e : row.entrySet())
			if(e.getValue() > maxVal &&  e.getKey() != activePC)
				maxID = e.getKey();

		return visited.add(maxID) ? maxID : -1;
	}
	
	public int propagateActivationRandom(int activePC, HashSet<Integer> visited) {
		var row = wTable.getRow(activePC);
		
		float sum = 0;
		for(var v : row.values()) 
			if( v > propagation_threshold)
				sum+=v;
		
		int res = -1;
		if(sum > 0) {
			float r = sum*RandomSingleton.getInstance().nextFloat();
			
			float partial = 0;
			for(var e : row.entrySet())
				if(e.getValue() > propagation_threshold && e.getKey() != activePC) {
					partial += e.getValue();
					if(r <= partial) {
						res = e.getKey();
						break;
					}
				}
		}
		
		return visited.add(res) ? res : -1;
	}
	
}
