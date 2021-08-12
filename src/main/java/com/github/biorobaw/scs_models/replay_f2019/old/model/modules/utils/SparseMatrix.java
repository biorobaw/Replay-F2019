package com.github.biorobaw.scs_models.replay_f2019.old.model.modules.utils;

import java.util.HashMap;

public class SparseMatrix {

	HashMap<Integer,Float>[] matrix;
	
	HashMap<Integer,HashMap<Integer,Float>> nonZeroRows;
	
	@SuppressWarnings("unchecked")
	public SparseMatrix(int numRows) {
		matrix = new HashMap[numRows];
		for(int i=0; i<numRows; i++) {
			matrix[i] = new HashMap<>();
		}
		
		nonZeroRows = new HashMap<>(numRows);
	}
	
	public void set(int i, int j, float val) {
		if(val==0) {
			matrix[i].remove(j);
			if(matrix[i].size()==0) nonZeroRows.remove(i);
		} else {
			if(matrix[i].size() == 0) nonZeroRows.put(i, matrix[i]);
			matrix[i].put(j, val);
		}
	}
	
	public float get(int i, int j) {
		var val = matrix[i].get(j);
		return val == null ? 0 : val;
	}
	
	public HashMap<Integer,HashMap<Integer,Float>> getNonZeroRows(){
		return nonZeroRows;
	}
	
	public HashMap<Integer,Float> getRow(int row){
		return matrix[row];
	}
	
	
}
