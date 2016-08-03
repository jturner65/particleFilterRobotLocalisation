package cs8803Lab1;

import java.util.NavigableSet;
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
	public void rayCastFromCell(int cell_x, int cell_y, Float[] calcs){
		float xmult,ymult,dist = 0, radians;//displacement in x and y dir for each raycast	
		int x = 0, y = 0;
		for(int idx = 0; idx<calcs.length; ++idx){
			radians = idx * map.p.degPerC2Rad;
//			xmult = PApplet.cos(radians);			//how much to move in the x direction
//			ymult = PApplet.sin(radians);			//how much to move in the y direction
			xmult = PApplet.sin(-radians);			//how much to move in the x direction
			ymult = PApplet.cos(radians);			//how much to move in the y direction
			for(float i = .01f; i<map.maxDim; i+=.01f){//iterate through cell until hit highly believed to-be wall
				x = (int)(cell_x + xmult*i);
				y = (int)(cell_y + ymult*i);	
				float xdist = x - cell_x, ydist = y - cell_y;
				dist = (float)Math.sqrt(((xdist)*(xdist)) + ((ydist)*(ydist)));
				if((y >= map.occupancyMap.length) || (x >= map.occupancyMap[0].length) ||(dist>map.maxDim)){calcs[idx]=(float)map.maxDim;break;}
				if(map.occupancyMap[y][(int)x] <= 0){calcs[idx] = dist;break;}//if map is occupied	->means hit a barrier - use mapProbsRaw <= mapRCastThresh instead?			
			}//i		
//			if((cell_x == 371) && (cell_y==574)){
//				p.outStr2Scr(""+idx+" =  "+radians+" rad | "+ (idx*p.degPerCasts)+" deg |xmult :"+xmult+" ymult :"+ymult+" len of cast "+dist+" coords of col block :" + x +","+y );
//			}				
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
