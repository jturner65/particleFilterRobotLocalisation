package cs8803Lab1;

import java.util.concurrent.Callable;
import java.util.concurrent.ConcurrentNavigableMap;

import processing.core.PApplet;

public class myRayCaster implements Callable<Boolean> {
	private myMap map;
	//public ConcurrentNavigableMap<Tuple<Integer, Integer>, Float[]>  subMap;
	public Object[] subMapkeySet;
	
	public myRayCaster(myMap _map, Object[] _subMapkeySet){
		map=_map;
		subMapkeySet = _subMapkeySet;
	}

	//precomput 360 degree ray casts from each cell that can be potentially occupied
	public void rayCastFromCell(int cell_Col, int cell_Row, float[] calcs){
		float colMult,rowMult,dist = 0, radians;//displacement in x and y dir for each raycast	
		int col = 0, row = 0;
		for(int idx = 0; idx<calcs.length; ++idx){
			radians = idx * map.p.degPerC2Rad;
			//below is orientation for non-neg-y-scaled (swapped) configuration
			colMult = PApplet.cos(radians);			//how much to move in the x direction
			rowMult = PApplet.sin(radians);			//how much to move in the y direction
			//below is for swapped-orientation casts
//			colMult = PApplet.cos(radians);			//how much to move in the x direction
//			rowMult = PApplet.sin(-radians);			//how much to move in the y direction
			
			for(float i = .01f; i<map.maxDim; i+=.01f){//iterate through cell until hit highly believed to-be wall
				col = (int)(cell_Col + colMult*i);
				row = (int)(cell_Row + rowMult*i);	
				float xdist = col - cell_Col, ydist = row - cell_Row;
				dist = (float)Math.sqrt(((xdist)*(xdist)) + ((ydist)*(ydist)));
				if((row >= map.occupancyMap.length) || (col >= map.occupancyMap[0].length) ||(dist>map.maxDim)){calcs[idx]=(float)map.maxDim;break;}
				if(map.occupancyMap[row][col] <= 0){calcs[idx] = dist;break;}//if map is occupied	->means hit a barrier - use mapProbsRaw <= mapRCastThresh instead?			
			}//i		
			
		}//for each calc		
	}//rayCastFromCell
	//initialize ray casts for every occupiable map cell
	@SuppressWarnings("unchecked")
	public void initRayCasts(){
		//p.outStr2Scr("Start init ray casts : " + mapPreScanRayCasts.keySet().size());
		for(Object keyTest : subMapkeySet){
			Tuple<Integer,Integer> key = (Tuple<Integer,Integer>)keyTest;
			rayCastFromCell(key.x, key.y, map.mapPreScanRayCasts.get(key));			//y is row, x is col	\
		}	
		//p.outStr2Scr("Done init ray casts : " + mapPreScanRayCasts.keySet().size());
	}//initRayCasts
	
	
	@Override
	public Boolean call() throws Exception {
		// TODO Auto-generated method stub
		initRayCasts();
		return true;
	}

}
