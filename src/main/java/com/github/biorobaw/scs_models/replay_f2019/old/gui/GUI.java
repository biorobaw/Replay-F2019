package com.github.biorobaw.scs_models.replay_f2019.old.gui;

import java.awt.Color;

import com.github.biorobaw.scs.experiment.Experiment;
import com.github.biorobaw.scs.gui.Display;
import com.github.biorobaw.scs.gui.DrawPanel;
import com.github.biorobaw.scs.gui.displays.scs_swing.drawer.CycleDataDrawer;
import com.github.biorobaw.scs.gui.displays.scs_swing.drawer.FeederDrawer;
import com.github.biorobaw.scs.gui.displays.scs_swing.drawer.PathDrawer;
import com.github.biorobaw.scs.gui.displays.scs_swing.drawer.RobotDrawer;
import com.github.biorobaw.scs.gui.displays.scs_swing.drawer.RuntimesDrawer;
import com.github.biorobaw.scs.gui.displays.scs_swing.drawer.WallDrawer;
import com.github.biorobaw.scs.gui.utils.GuiUtils;
import com.github.biorobaw.scs_models.replay_f2019.old.gui.drawers.PCDrawer;
import com.github.biorobaw.scs_models.replay_f2019.old.gui.drawers.PolarDataDrawer;
import com.github.biorobaw.scs_models.replay_f2019.old.gui.drawers.VDrawer;
import com.github.biorobaw.scs_models.replay_f2019.old.model.Model;

public class GUI {

	// =========== PARAMETERS =====================
	static final int   wall_thickness = 1;
	static final Color wall_color 	  = GuiUtils.getHSBAColor(0f, 0f, 0f, 1);
	static final Color path_color 	  = Color.RED;
	
	// ============ VARIABLES =====================
	Display d = Experiment.get().display;
	Model model;
	
	// reference to drawers
	public WallDrawer wallDrawer;
	public PathDrawer pathDrawer;
	public RobotDrawer rDrawer;
	public FeederDrawer fDrawer;

	int numScales;
	public PCDrawer pcDrawers;
	public VDrawer VDrawers;
	
	
	public PolarDataDrawer qDrawer;
	public PolarDataDrawer affDrawer;
	public PolarDataDrawer biasDrawer;
	public PolarDataDrawer probDrawer;
	
	
	public RuntimesDrawer runtimes;
	
	// ============== CONSTRUCTOR ===================
	
	public GUI(Model model) {
		this.model = model;
		createPanels();
		createDrawers();
		addDrawersToPanels();
	}
	
	
	private void createPanels() {
		// =========== CREATE PANELS =================
		// PC PANELS
		d.addPanel(new DrawPanel(300, 300), "pcPanel", 0, 0, 1, 1);
		
		// VALUE PANELS
		d.addPanel(new DrawPanel(300, 300), "VPanel", 1, 0, 1, 1);
		
		// ACTION SELECTION PANELS
		d.addPanel(new DrawPanel(300,300), "Aff", 0, 1, 1, 1);
		d.addPanel(new DrawPanel(300,300), "bias", 1, 1, 1, 1);
		d.addPanel(new DrawPanel(300,300), "Q", 0, 2, 1, 1);
		d.addPanel(new DrawPanel(300,300), "actions", 1, 2, 1, 1);
		
		d.addPanel(new DrawPanel(300,300), "runtimes", 0, 3, 1, 1);
	}
	
	private void createDrawers() {
		// =========== CREATE DRAWERS ===============
		

		// Maze related drawers
		wallDrawer = new WallDrawer( wall_thickness);
		wallDrawer.setColor(wall_color);
		
		pathDrawer = new PathDrawer(model.getRobot().getRobotProxy());
		pathDrawer.setColor(path_color);

		rDrawer = new RobotDrawer(model.getRobot().getRobotProxy());
		
		fDrawer = new FeederDrawer(0.1f);
		
		// PC drawers
		var pc_bin = model.pc_bins;
		var pcs = model.place_cells;
		pcDrawers = new PCDrawer(pcs.xs, pcs.ys, pcs.rs,
						 				() -> pc_bin.active_pcs.as,
						 				() -> pc_bin.active_pcs.ids);
		
		// V drawers
		VDrawers =  new VDrawer(pcs.xs, pcs.ys, model.vTable);
		VDrawers.distanceOption = 0; // use fixed size to draw value of pcs
//		VDrawers.setRadius(1);
		
		
		// RL and action selection drawers
		
		qDrawer = new PolarDataDrawer("Q softmax",model.num_actions ,() -> model.softmax );
		affDrawer = new PolarDataDrawer("Affordances",model.num_actions , ()->model.affordances.affordances);
		biasDrawer = new PolarDataDrawer("Bias",model.num_actions, ()->model.motionBias.getBias());
		probDrawer = new PolarDataDrawer("Probs",model.num_actions, ()->model.motionBias.getProbabilities());
		probDrawer.setGetArrowFunction(()->model.chosenAction);
		
		runtimes = new RuntimesDrawer(50, 0, 800);
		runtimes.doLines = false;
		
		
	}
	
	private void addDrawersToPanels() {
		// ======== ADD DRAWERS TO PANELS ============
		
		// UNIVERSE PANEL
//		d.addDrawer("universe", "pcs", VDrawers);
		d.addDrawer("universe", "pcs", pcDrawers);
		d.addDrawer("universe", "maze", wallDrawer );
		d.addDrawer("universe", "feeders", fDrawer);
		d.addDrawer("universe", "path", pathDrawer);
		d.addDrawer("universe", "robot", rDrawer);
		d.addDrawer("universe", "cycle info", new CycleDataDrawer());
		
		// RUNTIMES
		d.addDrawer("runtimes", "runtimes", runtimes);
		
		// PC PANELS
		d.addDrawer("pcPanel", "PC layer", pcDrawers);
		d.addDrawer("pcPanel", "maze", wallDrawer);
		d.addDrawer("pcPanel", "robot other", rDrawer);
		
		
		// VALUE PANELS:
		d.addDrawer("VPanel", "maze", wallDrawer);
		d.addDrawer("VPanel", "robot other", rDrawer);
		d.addDrawer("VPanel", "V_layer", VDrawers);
		
		
		// ACTION SELECTION PANELS:
		d.addDrawer("Q", "qDrawer", qDrawer);
		d.addDrawer("Aff", "affDrawer", affDrawer);
		d.addDrawer("bias", "biasDrawer", biasDrawer);
		d.addDrawer("actions", "probDrawer", probDrawer);
		
		
		
	}
	
	
	
}
