package com.github.biorobaw.scs_models.replay_f2019.old.model.modules.b_state;


import java.util.HashMap;

import com.github.biorobaw.scs.experiment.Experiment;
import com.github.biorobaw.scs.utils.math.Floats;

/**
 * Class to operate with place cells.
 * The class creates a set of place cells, 
 * and distributes them into bins to allow quick computation of non zero activation values.
 * @author bucef
 *
 */
public class PlaceCells {
	
	// minimum activation of a place cell (value at its border)
	static final float min_activation = 0.07f;
	static final float numerical_error_tolerance = 0.01f; // 1cm
	
	// total number of cells in the set
	public int num_cells;
	
	
	public float[] xs; // the x coordinates in the set
	public float[] ys; // the y coordinates in the set
	public float[] rs; // the radii of the cells
	public float[] ks; // the constant part of the pc equation e^(k*d^2) for each cell
	public int[]  ids; // the id associated to each cell in the set
	
	public HashMap<Integer,Integer> idHash;

	public float[] r2s;// precalculated r squared values
	public float[] as; // the activation of each cell in the set
	public float[] ns; // normalized activation
	
	public float total_a; // the total activation of the layer (the sum)
	
	
	/**
	 * Creates a random cell of place cells
	 * @param num_cells Number of cells to create
	 * @param radius Radius of cells
	 * @param generator A pc generator, the generate function must return
	 * an array with the x and y coordinates.
	 */
	public PlaceCells(int num_cells, float radius ,PCGenerator generator) {
		this.num_cells = num_cells;
		idHash = new HashMap<>(num_cells);
		xs = new float[num_cells];
		ys = new float[num_cells];
		rs = new float[num_cells];
		ks = new float[num_cells];
		ids = new int[num_cells];
		
		as = new float[num_cells];
		r2s = new float[num_cells];
		ns = new float[num_cells];
		
		var r2 = radius*radius;
		for(int i=0; i<num_cells; i++) {
			var xy  = generator.generate();
			xs[i] = xy[0];
			ys[i] = xy[1];
			rs[i] = radius;
			ks[i] = (float)Math.log(min_activation)/r2;
			ids[i]= i;
			
			r2s[i] = r2;
			
			idHash.put(i, i);
		}
		
	}
	
	
	/**
	 * Constructor that creates a wrapper around the function arguments and completes remaining fields.
	 * @param xs x coordinate of each pc
	 * @param ys y coordinate of each pc 
	 * @param rs radius of each pc
	 * @param ks constant of the activation equation of each pc
	 * @param ids id of each pc
	 */
	public PlaceCells(float[] xs, float[] ys, float[] rs, float[] ks, int[] ids) {
		this.xs = xs;
		this.ys = ys;
		this.rs = rs;
		this.ks = ks;
		this.ids = ids;
		
		num_cells = xs.length;
		idHash = new HashMap<>(num_cells);
		as  = new float[num_cells];
		r2s = new float[num_cells];
		ns  = new float[num_cells];
		for(int i=0; i<num_cells; i++) {
			r2s[i] = (rs[i]+numerical_error_tolerance)*(rs[i]+numerical_error_tolerance);
			idHash.put(ids[i], i);
		}
		
	}
	
	/**
	 * A shallow copy of a set of place cells
	 * @param from
	 * @return
	 */
	public PlaceCells copyShallow(PlaceCells from) {
		this.num_cells = from.num_cells;
		
		this.xs  = from.xs;
		this.ys  = from.ys;
		this.rs  = from.rs;
		this.ks  = from.ks;
		this.ids = from.ids;
		
		this.as  = from.as;
		this.r2s = from.r2s;
		this.ns = from.ns;
		this.total_a = from.total_a;
		this.idHash = from.idHash;
		return this;
		
	}
	
	/**
	 * Computes the activation of each place cell.
	 * The result is stored in the field `as` 
	 * @param x The robot's current x coordinate
	 * @param y The robot's current y coordinate
	 * @return Returns the pointer 'this' to allow chaining functions
	 */
	public float activate(float x, float y) {
		total_a = 0;
		for(int i=0; i<num_cells; i++) {
			var dx = xs[i] - x;
			var dy = ys[i] - y;
			var r2 = dx*dx + dy*dy;
//			System.out.println("r2: " +r2 + " " + r2s[i] );
			if(r2 <= r2s[i]) {
				as[i] = (float)Math.exp(ks[i]*r2);
				total_a +=as[i];
			}
			else as[i] = 0;
		}
		return total_a;
	}
	
	/**
	 * Normalizes the activations of the pcs by the given value
	 * @param value
	 */
	public void normalize(float value) {
		if(value==0) {
			System.err.println("ERROR: Division by 0, place cells normalization failed");
			var e = Experiment.get();
			System.err.println("Trial-Episode-cycle: " 
								+ e.getGlobal("tria") + " " 
								+ e.getGlobal("episode") + " " 
								+ e.getGlobal("cycle"));
//			e.display.updateData();
//			e.display.repaint();
//			try {
//				System.in.read();
//			} catch (IOException e1) {
//				// TODO Auto-generated catch block
//				e1.printStackTrace();
//			}
			System.exit(-1);
		}
		Floats.div(as, value, ns);
	}
	
	
	/**
	 * Interface to create random cells
	 * @author bucef
	 *
	 */
	public interface PCGenerator {
		/**
		 * Generate a random PC
		 * @return returns an array of floats representing a pc
		 */
		float[] generate();
	}
	
	
}
