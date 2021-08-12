package com.github.biorobaw.scs_models.replay_f2019.old.gui.drawers;


import java.awt.Color;
import java.awt.Graphics;
import java.util.Arrays;

import com.github.biorobaw.scs.gui.Drawer;
import com.github.biorobaw.scs.gui.displays.scs_swing.DrawerSwing;
import com.github.biorobaw.scs.gui.utils.GuiUtils;
import com.github.biorobaw.scs.gui.utils.Scaler;
import com.github.biorobaw.scs.gui.utils.Window;
import com.github.biorobaw.scs.utils.math.Floats;
import com.github.biorobaw.scs.utils.math.Integers;



public class PCDrawer extends DrawerSwing {

	float[] pc_x;
	float[] pc_y;
	float[] pc_r;
	int num_cells;
	
	float[] activations;
	int[] ids;
	
	GetActiveValues getActiveValues;
	GetActiveIds getActiveIds;
	
	float radius = -1; // used when all pcs have the same radius
	boolean constant = false;
	
//	HashMap<Integer, Float> values = new HashMap<>();
	

	public PCDrawer(float[] pc_x, float[] pc_y, float[] pc_r, GetActiveValues getActiveValues, GetActiveIds getActiveIds) {
		this.getActiveValues = getActiveValues;
		this.getActiveIds = getActiveIds;
		this.pc_x = pc_x;
		this.pc_y = pc_y;
		this.pc_r  = pc_r;
		
		num_cells = pc_x.length;
		radius = Floats.max(pc_r);
		constant = radius == Floats.min(pc_r);

	}


	@Override
	public void draw(Graphics g, Window panelCoordinates) {
		if(!doDraw) return;
		
		if(constant) drawConstant(g, panelCoordinates);
		else drawMultipleR(g, panelCoordinates);
		
		
		

	}

	@Override
	public void newEpisode() {
		super.newEpisode();
		ids = null;
	}

	@Override
	public void updateData() {
		ids = getActiveIds.get();
		if(ids!=null) {
			activations = Floats.copy(getActiveValues.get());
		}
		
	}
	
	
	public Color getColor(float val){
		float i = 0.4f;
		float alpha = i+(1f-i)*(val-0.2f)/(1f-0.2f);
		alpha =0.8f;
		float m = 0.2f;
		float beta  = m+(0.73f-m)*(val-0.2f)/(1f-0.2f);
//		max = (float)Math.max(max, alpha);
//		alpha = (float)Math.sqrt(alpha);
		return  GuiUtils.getHSBAColor(0f,0.9f,alpha,beta);
	}
	
	public  interface GetActiveValues {
		float[] get();
	}
	
	public interface GetActiveIds {
		int[] get();
	}
	
	
	void drawConstant(Graphics g, Window panelCoordinates) {
		
		// Scale coordinates
		Scaler s = new Scaler(worldCoordinates, panelCoordinates, true);
		
		var r = s.scaleDistanceX(radius);
		var r2 = Math.round(2*r);
		
		var scaled_x = s.scaleX(pc_x);
		var scaled_y = s.scaleY(pc_y);
		
		var all_top_x = Floats.round(Floats.sub(scaled_x, r, scaled_x));
		var all_top_y = Floats.round(Floats.sub(scaled_y, r, scaled_y));
		
		
		//draw base rings
		int grayLevel = 180;
		g.setColor(new Color(grayLevel,grayLevel,grayLevel));

		
		for(int i=0; i<num_cells; i++)
			g.drawOval(all_top_x[i], all_top_y[i], r2, r2);
		
		
		// proceed only if drawer has been updated at least once
		if(ids==null) return;
		
		var active_top_x = Integers.getElements(all_top_x, ids);
		var active_top_y = Integers.getElements(all_top_y, ids);
		int num_active = ids.length;
		
		//fill background of nonzero place cells
		grayLevel = 210;
		g.setColor(new Color(grayLevel,grayLevel,grayLevel));
		for(int i=0; i<num_active; i++)
			if(activations[i] >0)
				g.fillOval(active_top_x[i], active_top_y[i],r2, r2);


		//give hue to non zero place cells
		for(int i=0; i<num_active; i++) 
			if(activations[i] >0) {
				g.setColor(getColor(activations[i]));
				g.fillOval(active_top_x[i], active_top_y[i],r2, r2);
			}
		
	}
	
	void drawMultipleR(Graphics g, Window panelCoordinates) {
				
		// Scale coordinates
		Scaler s = new Scaler(worldCoordinates, panelCoordinates, true);
		
		var scaled_x = s.scaleX(pc_x);
		var scaled_y = s.scaleY(pc_y);
		var r = s.scaleDistanceX(pc_r);
		
		var all_top_x = Floats.round(Floats.sub(scaled_x, r, scaled_x));
		var all_top_y = Floats.round(Floats.sub(scaled_y, r, scaled_y));
		var r2 = Floats.round(Floats.mul(r, 2, r));  
		
		
		//draw base rings
		int grayLevel = 180;
		g.setColor(new Color(grayLevel,grayLevel,grayLevel));
		for(int i=0; i<num_cells; i++)
			g.drawOval(all_top_x[i], all_top_y[i],
					   r2[i], r2[i]);
		
		
		// proceed only if drawer has been updated at least once
		if(ids==null) return;
		
		var active_top_x = Integers.getElements(all_top_x, ids);
		var active_top_y = Integers.getElements(all_top_x, ids);
		var active_2r	 = Integers.getElements(r2, ids);
		int num_active = ids.length;
		
		//fill background of nonzero place cells
		grayLevel = 210;
		g.setColor(new Color(grayLevel,grayLevel,grayLevel));
		for(int i=0; i<num_active; i++)
			if(activations[i]>0)
				g.fillOval(active_top_x[i], active_top_y[i],
						   active_2r[i], active_2r[i]);


		
		//give hue to non zero place cells
		for(int i=0; i<num_active; i++) 
			if(activations[i]>0) {
				g.setColor(getColor(activations[i]));
				g.fillOval(active_top_x[i], active_top_y[i],
						   active_2r[i], active_2r[i]);
			}
		
	}
	
}
