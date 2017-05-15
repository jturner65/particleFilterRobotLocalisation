package cs8803Lab1;
import java.io.*;
import java.util.*;
import java.util.concurrent.ConcurrentSkipListMap;
import java.util.concurrent.Future;
import java.util.concurrent.ThreadLocalRandom;

import processing.core.*;

public class myMap {
	public Lab1Localization p;
	public String name;
	
	public float[][] mapProbsRaw;					//map, holding probabilities of emptiness (1-occ), from dat file, still has -1's in unknown regions
	public float[][] occupancyMap;						//map, holding probabilities of emptiness (1-occ), from dat file, all -1's turned to 0, all occupancy thresholded to 0/1
	
	public ConcurrentSkipListMap<Tuple<Integer,Integer>, float[]>mapPreScanRayCasts;			//360 degree ray casts for every occupiable grid cell (cells with prob 1) - 1 for every degPerCast
	//use LOG PROBS
	public TreeMap<Float, mySample> smpMapProbToLoc;		//map of cum probs to locs - cdf of locations, to sample map. use floor (first value is non-zero prob, which needs to be less than or equal to get this cell)
	public float sampMapToTProb;							//total  PROB of smpMapProbToLoc map
	public TreeMap<Float, mySample> smpMapProbToKnwnLoc;	//map of samples to prob 1 locations (known to be occupiable) - cdf of locations, to sample map. use floor (first value is non-zero prob, which needs to be less than or equal to get this cell)
	public float sampMapToTKnwnProb;					    //total  PROB of smpMapProbToKnwnLoc map
	
	public float cellDim;						//cell size in cm
	public float mapSizeX, mapSizeY;			//map size in cm
	public float mapOffX, mapOffY;				//map offset in cm
	public int mapWidth, mapHeight;				//map size in cells
	public int maxDim;							//max of width and height
	
	public String rcFileName;					//file name for ray cast info
	
	public List<Future<Boolean>> callRCPreCompFutures;
	public List<myRayCaster> callRCCalcs;
	
	public final int xOffset, yOffset;	
	//pregen files for 1.0f, .95f, .75f, and .5f
	public final float mapProbOccThresh = .95f;	//.75f; //.5f;	//threshold prob in map for cell to be occupiable - map given in file is density map
	public PImage[] mapImgs;// mapMainImg, mapDemoImg;
	public float mapTotProb;
		
	public final int numCastsForCircle;
	//load map and either precast ray casts from all locations, or precalc these raycasts and then save them to a file to reuse
	public myMap(Lab1Localization _p, String _name) {
		p=_p;name=_name;
		numCastsForCircle = (int)(360/p.degPerCasts);
		loadMap();	
		xOffset = mapWidth/2; yOffset=mapHeight/2;
		maxDim = PApplet.max(mapWidth, mapHeight);
		initMapImages();
		rcFileName = "RayCastsInfo_DegPC_Neg1"+p.degPerCasts+"_MinOccProb_"+mapProbOccThresh+".csv";
		p.outStr2Scr("Attempt to load precalced Ray Casts from Ray Cast file name : "+rcFileName);		
		boolean[] tryLoadRayCasts = new boolean[]{false};
		String[] fileData = loadRayCasts(rcFileName,tryLoadRayCasts);
		if(!tryLoadRayCasts[0]){	
			p.outStr2Scr("Failed to find and load file : " +rcFileName + " holding precast rays - precasting rays for every occupiable cell in grid, at "+p.degPerCasts+" degree incrmements");
			//initRayCasts();
			buildMTRayCasts();
			saveRayCasts(rcFileName);
			p.outStr2Scr("Saved raycast info to file : '"+rcFileName+"'");
		} else {
			readRayCasts(fileData);
			p.outStr2Scr("Finished loading pre-calced casts from file : " + rcFileName);			
		}
		p.flags[p.initialLoad]=false;				//set true upon initial load, to display data pertaining to program startup
		p.outStr2Scr("Finished initializing map from data");
	}
	
	
	public void buildMTRayCasts(){
		callRCCalcs = new ArrayList<myRayCaster>();
		callRCPreCompFutures = new ArrayList<Future<Boolean>>();
		int preScanRCSize =  mapPreScanRayCasts.keySet().size();
		p.outStr2Scr("Start init ray casts : " + preScanRCSize);
		
		int frameSize = p.max(preScanRCSize/50, 10);
		//ConcurrentNavigableMap<Tuple<Integer, Integer>, Float[]> subMap = new ConcurrentSkipListMap<Tuple<Integer,Integer>, Float[]>();
		@SuppressWarnings("unchecked")
		Object[] keySetAra = mapPreScanRayCasts.keySet().toArray(), tmpAra = new Object[frameSize];
		for (int i = 0; i < keySetAra.length; i += frameSize) {
			int cpyLen = (i + frameSize >= keySetAra.length ?  keySetAra.length - i - 1 : frameSize);
			tmpAra = new Object[cpyLen];
			System.arraycopy(keySetAra, i, tmpAra, 0, cpyLen);
			//subMap = mapPreScanRayCasts.subMap((Tuple<Integer,Integer>)keySetAra[i], (Tuple<Integer,Integer>)keySetAra[endIdx]);
			// p.outStr2Scr("i :" + i + " final len "+finalLen +" x length "+X.length);			
			callRCCalcs.add(new myRayCaster(this, tmpAra));
		} // sample and eval each sample
		try {callRCPreCompFutures = p.th_exec.invokeAll(callRCCalcs);for(Future<Boolean> f: callRCPreCompFutures) { f.get(); }} catch (Exception e) { e.printStackTrace(); }		
		p.outStr2Scr("Done init ray casts : " + mapPreScanRayCasts.keySet().size());
	}
	
	
//	//initialize ray casts for every occupiable map cell
//	public void initRayCasts(){
//		p.outStr2Scr("Start init ray casts : " + mapPreScanRayCasts.keySet().size());
//		Tuple<Integer,Integer> keyLast = new Tuple<Integer,Integer>(0,0);
//		for(Tuple<Integer,Integer> key : mapPreScanRayCasts.keySet()){
//			rayCastFromCell(key.x, key.y, mapPreScanRayCasts.get(key));			//y is row, x is col	\
//			//keyLast = key;
//		}	
//		p.outStr2Scr("Last Key : " + keyLast);
//		p.outStr2Scr("Done init ray casts : " + mapPreScanRayCasts.keySet().size());
//	}//initRayCasts
//	
//	//precompute 360 degree ray casts from each cell that can be potentially occupied
//	public void rayCastFromCell(int cell_x, int cell_y, float[] calcs){
//		float xmult, ymult, dist=0, radians;
//		int x=0, y=0;										//displacement in x and y dir for each raycast
//		for(int idx = 0; idx<calcs.length; ++idx){
//			radians = idx * p.degPerC2Rad;
//			xmult = PApplet.cos(radians);			//how much to move in the x direction
//			ymult = PApplet.sin(radians);			//how much to move in the y direction
//			for(float i = .01f; i<maxDim; i+=.01f){//iterate through cells until hit highly believed to-be wall
//				x = (int)(cell_x + xmult*i);
//				y = (int)(cell_y + ymult*i);	
//				float xdist = x - cell_x, ydist = y - cell_y;
//				dist = (float)Math.sqrt(((xdist)*(xdist)) + ((ydist)*(ydist)));
//				if(( y >= occupancyMap.length) || ((int)x >= occupancyMap[0].length) ||(dist>maxDim)){calcs[idx]=(float)maxDim;break;}
//				if(occupancyMap[y][x] <= 0){calcs[idx] = dist;break;}//if map is occupied	->means hit a barrier - use mapProbsRaw <= mapRCastThresh instead?			
//			}//i		
////			if((cell_x == 371) && (cell_y==574)){
////				p.outStr2Scr(""+idx+" =  "+radians+" rad | "+ (idx*p.degPerCasts)+" deg |xmult :"+xmult+" ymult :"+ymult+" len of cast "+dist+" coords of col block :" + x +","+y );
////			}				
//		}//for each calc		
//	}//rayCastFromCell
	
	//load pre-saved ray cast info for every cell in map
	public String[] loadRayCasts(String fileName, boolean[] success){
		success[0] = false;
		String[] rayCastFileData = null;
		try{
			//p.outStr2Scr("Trying to load : " + fileName);
			rayCastFileData = p.loadStrings(fileName);
			success[0] = (rayCastFileData.length != 0);
		}
		catch (Exception e){
			//p.outStr2Scr("Failed to load : " + fileName);
			success[0] = false;
		}
		finally{}		
		return rayCastFileData;
	}//loadRayCasts
	//read ray casts from file data
	public void readRayCasts(String[] rayCastFileData){	
		Tuple<Integer,Integer> tmpKey;
		String line;
		mapPreScanRayCasts.clear();
		for(int lidx=0;lidx<rayCastFileData.length;++lidx){
			line = rayCastFileData[lidx];
			String[]splitRes = line.split("[,\\s]+");	
			tmpKey = new Tuple<Integer,Integer>(Integer.parseInt(splitRes[0]),Integer.parseInt(splitRes[1]));//x and y are cols 1 and 2
			int fidx = 0;			
			float[] scanSweep = new float[splitRes.length-2];
			for(int i=2;i<splitRes.length;++i){scanSweep[fidx++]=Float.parseFloat(splitRes[i]);}
			mapPreScanRayCasts.put(tmpKey,  scanSweep);
		}		
		p.outStr2Scr("Done Reading precalced ray casts : " + mapPreScanRayCasts.values().size());		
	}//	readRayCasts	
	//save pre-calced raycasts to file
	public void saveRayCasts(String fileName) {
		try{
			PrintWriter writer = new PrintWriter(fileName);
			for(Tuple<Integer,Integer> key : mapPreScanRayCasts.keySet()){
				float[] res = mapPreScanRayCasts.get(key);
				String out = ""+key.x+","+key.y+",";
				for(int i = 0; i<res.length; ++i){out+=""+res[i]+",";}
				writer.println(out);		
			}		
			writer.close();				
		}
		catch (Exception e){} 
		finally {}
	}
	
	public void loadMap(){
		String[] fileData = p.loadStrings(name);
		int idx = 0;
		boolean[] headerDone = new boolean[]{false};
		for(String line : fileData){
			if(null==line){break;}
			if(!headerDone[0]){	loadHeaderMap(line, headerDone); } else {	break;}
			idx++;
		}
//		mapProbsRaw = new float[mapWidth][mapHeight];
//		occupancyMap = new float[mapWidth][mapHeight];
		mapProbsRaw = new float[mapHeight][mapWidth];
		occupancyMap = new float[mapHeight][mapWidth];		
		
		smpMapProbToLoc = new TreeMap<Float, mySample>();			//idx'ed by cumulative probability, value is idx in sampleMap
		sampMapToTProb = 0;
		smpMapProbToKnwnLoc = new TreeMap<Float, mySample>();	//map of samples to prob 1 locations (known to be occupiable) - cdf of locations, to sample map. use floor (first value is non-zero prob, which needs to be less than or equal to get this cell)
		sampMapToTKnwnProb = 0;					    //total  PROB of smpMapProbToKnwnLoc map
		loadDataMap(fileData, idx);	
		p.outStr2Scr("myMap::loadMap : Finished with loading map data found in file " + name);	 
	}//loadMap
	
	public void loadHeaderMap(String line, boolean[] done){
	    String[] tarStr = new String[]{"robot_specifications->global_mapsize_x", "robot_specifications->global_mapsize_y", "robot_specifications->resolution","robot_specifications->autoshifted_x", "robot_specifications->autoshifted_y", "global_map"};
	    for(int i=0; i<tarStr.length; ++i){	if(line.contains(tarStr[i])){loadHeaderMapStr(line, i,done); return;} }		
	}//loadHeaderMap	
	
	//MAP data is given in column-major order
	public void loadDataMap(String[] fileData, int startRow){
		String line;
		float prob;
		int rowIdx=0;
		Tuple<Integer,Integer> tmpTup;		
		float[] casts;
		mapPreScanRayCasts = new ConcurrentSkipListMap<Tuple<Integer,Integer>, float[]>();
		for(int fileRow = startRow; fileRow<fileData.length; ++fileRow){
			if(null==fileData[fileRow]){return;}
			line = fileData[fileRow];
			String[]splitRes = line.split("\\s+");	
			for(int col=0; col<mapProbsRaw[rowIdx].length; ++col){	
				mapProbsRaw[rowIdx][col]=Float.parseFloat(splitRes[col]); 								//holds chance map could be occupied at this particular location - density map
				occupancyMap[rowIdx][col] = (mapProbsRaw[rowIdx][col] >= mapProbOccThresh ? 1 : 0 ) ;
				if(occupancyMap[rowIdx][col] == 1){ 													//set up prior
					casts = new float[numCastsForCircle];
					tmpTup = new Tuple<Integer,Integer>(col,rowIdx);			//x is col, y is row			
					mapPreScanRayCasts.put(tmpTup, casts);
					//prob = mapProbsRaw[rowIdx][col]*mapProbsRaw[rowIdx][col]*mapProbsRaw[rowIdx][col];//p.map(mapProbsRaw[rowIdx][col], .5f, 1,0,1);
					prob = (mapProbsRaw[rowIdx][col] - mapProbOccThresh)/(1.0f - mapProbOccThresh);
					//{System.out.println(" -- prob : "+prob+ " row : "+rowIdx+" col : "+col);}
					mySample tmp =  new mySample(p,this,new myVector(col, rowIdx, 0),prob);
					smpMapProbToLoc.put(sampMapToTProb, tmp);	//key is floor of prob, above which is necessary to get this entry.  next key is ceiling.  access by "smpMapProbToLoc.lowerKey(randVal)"		
					sampMapToTProb += prob;																	//add this loc's prob to total, to be used for next loc's key, access by lowerKey(randVal)
					if(prob==1){
						smpMapProbToKnwnLoc.put(sampMapToTKnwnProb, new mySample(p,this,new myVector(col, rowIdx, 0),1));	//map of samples to prob 1 locations (known to be occupiable) - cdf of locations, to sample map. use floor (first value is non-zero prob, which needs to be less than or equal to get this cell)
						sampMapToTKnwnProb +=1;					    //total  PROB of smpMapProbToKnwnLoc map
					}				
				}
			}			//for each col
			rowIdx++;
		}				//for each row
	}//loadDataMap
	
	///end load map section
	//return a random spot from the occupancy map based to use as map prior
	public mySample sampleProbMap(float[] prob, int numSamp){
		float randVal = (float) ThreadLocalRandom.current().nextDouble(sampMapToTProb);
		prob[0]=smpMapProbToLoc.lowerKey(randVal); mySample tmp = smpMapProbToLoc.get(prob[0]); tmp.setOrient((float)ThreadLocalRandom.current().nextDouble(-PConstants.PI,PConstants.PI)); tmp.w /= (1.0f*numSamp); prob[1]=tmp.w; return tmp;}		//returns the highest key less than 
	//sample only from map of known-to-be occupiable grid cells
	public mySample sampleMap(float[] prob, int numSamp){float randVal = (float) ThreadLocalRandom.current().nextDouble(sampMapToTKnwnProb);prob[0]=smpMapProbToKnwnLoc.lowerKey(randVal); mySample tmp = smpMapProbToKnwnLoc.get(prob[0]);tmp.setOrient((float)ThreadLocalRandom.current().nextDouble(-PConstants.PI,PConstants.PI)); tmp.w /= (1.0f*numSamp); prob[1]=tmp.w; return tmp;}		//returns the highest key less than 
	
	public void loadHeaderMapStr(String line, int tarIdx, boolean[] done){
		String[] splitRes = line.split("\\s+");
		switch (tarIdx){
			case 0 : {		mapSizeX = Float.parseFloat(splitRes[splitRes.length-1]);break;}							//string : "robot_specifications->global_mapsize_x"
			case 1 : {		mapSizeY = Float.parseFloat(splitRes[splitRes.length-1]);break;}							//string : "robot_specifications->global_mapsize_y"
			case 2 : {		cellDim = Float.parseFloat(splitRes[splitRes.length-1]);break;}								//string : "robot_specifications->resolution"             
			case 3 : {		mapOffX = Float.parseFloat(splitRes[splitRes.length-1]);break;}								//string : "robot_specifications->autoshifted_x"
			case 4 : {		mapOffY = Float.parseFloat(splitRes[splitRes.length-1]);break;}								//string : "robot_specifications->autoshifted_y"
			case 5 : {
				mapWidth = Integer.parseInt(splitRes[splitRes.length-2]);
				mapHeight = Integer.parseInt(splitRes[splitRes.length-1]);				
				done[0]=true;
				break;}							//string : "global_map"				
		}	
	}//loadHeaderMapStr
	//two maps - raw probs (with -1 for unknown), and occupancy (thresholded to 0-1) probs
	public void initMapImages(){
		mapImgs = new PImage[2];
		mapImgs[0] = p.createImage( mapProbsRaw[0].length, mapProbsRaw.length, PConstants.RGB);		
		buildDispMap(mapProbsRaw, mapImgs[0], 0, "Raw Map");														//left map display with neg probs
		mapImgs[1] = p.createImage( occupancyMap[0].length, occupancyMap.length, PConstants.RGB);		
		buildDispMap(occupancyMap, mapImgs[1], 0, "Occ Map");															//right map display with thresholded probs		
	}

	//display the map on the screen
	public void buildDispMap(float[][] mapData, PImage mapImg, int clrOffset, String mapName){
		mapImg.loadPixels();
		int clr = p.color(0,0,0),idx = 0;
		mapTotProb = 0.0f;
		float mp,value;
		for(int row =0; row<mapData.length;++row){
			for(int col =0; col<mapData[row].length; ++col){
				mp = mapData[row][col];
				if(mp == -1.0f){clr = p.color(0,0,255,255);} 		//first map, unknown regions shown as blue
				else {
					value = mp*256 + clrOffset;		//higher MP means higher chance bot can be there, 0 mp means wall
					mapTotProb += mp;
//					if(mp == 1.0f-mapKnwnOcc){clr = p.color(100,0,100,255);}						//known to be occupiable
//					else if(mp == mapKnwnOcc){clr = p.color(0,100,100,255);}							//known to wall
//					else 
					clr = p.color(value,value,value, 255);
				}				
				mapImg.pixels[idx++]=clr;				
			}
		}
		System.out.println("" +mapName + " Map Tot Prob : "+ mapTotProb);
		mapImg.updatePixels();
	}//buildMap		
	
	public void draw(){
		p.pushMatrix();p.pushStyle();//debug only - moves to be in view
		//p.translate(-mapWidth/2.0f, 0,0);//debug only
		drawAxisLegend();
		for(int i=0;i<mapImgs.length;++i){	drawMap(i);}
		drawCasts(new Tuple<Integer,Integer>(650,647), mapPreScanRayCasts, p.degPerC2Rad);//draw precalcuated raycasts at a particular location in the map
		drawCasts(new Tuple<Integer,Integer>(455,647), mapPreScanRayCasts, p.degPerC2Rad);//draw precalcuated raycasts at a particular location in the map
		drawCasts(new Tuple<Integer,Integer>(455,585), mapPreScanRayCasts, p.degPerC2Rad);//draw precalcuated raycasts at a particular location in the map
		//uncomment to see if precalced ray casts match map layout
//		for(Tuple<Integer,Integer> key : mapPreScanRayCasts.keySet()){	//if((key.x==650)&&(key.y==647))  //705,647
//			drawCasts(key,mapPreScanRayCasts, p.degPerC2Rad);}
		p.popStyle();p.popMatrix();		//debug only	
	}
	
	public void drawMap(int idx){
		p.pushMatrix();p.pushStyle();//set up as row-col 2x array, not as x-y, so  need to swap width and height to y and x
		translateMap(idx);
		p.beginShape();
		p.texture(mapImgs[idx]);
		p.vertex(0,0,0,0);
		p.vertex(mapWidth, 0,0,1);
		p.vertex(mapWidth,-mapHeight,1,1);
		p.vertex(0,-mapHeight,1,0);
		p.endShape(PConstants.CLOSE);
		p.popStyle();p.popMatrix();			
	}
	public void drawAxisLegend(){
		p.pushMatrix();//set up as row-col 2x array, not as x-y, so  need to swap width and height to y and x
			moveToMapCtr();
			p.translate(810,0,1);
			p.pushMatrix();p.pushStyle();
				p.strokeWeight(1f);
				p.stroke(255,0,0,255);
				p.fill(100,0,0,255);
				p.line(0, 0, 0, 20,0,0);
				p.translate(20,0,0);
				p.drawSphere(new int[]{255,0,0,255},true, 1.0f, new int[]{255,0,0,255}, 2.0f);//p.sphere(2);
				p.fill(0,0,0,255);
				p.text("Pos X", 5,5);
			p.popStyle();p.popMatrix();
			p.pushMatrix();p.pushStyle();
				p.stroke(0,255,0,255);
				p.fill(0,100,0,255);
				p.line(0, 0, 0, 0,20,0);			
				p.translate(0,20,0);
				p.drawSphere(new int[]{0,255,0,255},true, 1.0f, new int[]{0,255,0,255}, 2.0f);//p.sphere(2);
				p.fill(0,0,0,255);
				p.text("Pos Y", 5,5);
			p.popStyle();p.popMatrix();
		p.popMatrix();				
	}
	//for debug purposes - display pre-calced casts for each cell
	public void drawCasts(Tuple<Integer,Integer>key, ConcurrentSkipListMap<Tuple<Integer,Integer>, float[]>mapCasts, float radPerCast){
		p.pushMatrix();p.pushStyle();//set up as row-col 2x array, not as x-y, so  need to swap width and height to y and x
		moveToMapCtr();
		p.translate(key.x, key.y, 1);
		p.drawSphere(new int[]{255,0,0,255},true, 1.0f, new int[]{255,0,0,255}, 2.0f);
	//	p.scale(1.0f,-1.0f,1.0f);
		//sweeps ccw cyan to yellow
		if(null != mapCasts.get(key)){
			p.drawCastAra(mapCasts.get(key), 255, radPerCast, 1);
		}
		p.popStyle();p.popMatrix();		
	}
	
	public void translateMap(int idx){
		switch (idx){
		case 0 :{translateToDemoMapCtr(); break;}//left "demo" map with bot moving on his own coords - uses raw prob
		case 1 :{translateToMapCtr(); break;}//right particle map
		case 2 :{translateToSampleMapCtr(); break;}//right lower prob map  - debug
		default : {translateToDemoMapCtr(); break;}
		}	
	}
	//--use this to set sample map loc (debug)
	public void translateToSampleMapCtr(){p.translate(-mapWidth, 3*mapHeight/2.0f, 0);	}
	//--use this to set data map loc 
	public void translateToDemoMapCtr(){p.translate(-mapWidth, mapHeight/2.0f, 0);	}
	//--use this to map all results 
	public void translateToMapCtr(){p.translate(0, mapHeight/2.0f, 0);	}
	
	//align drawn samples to prob map
	public void moveToMapCtr(){
		translateToMapCtr();
		p.rotate(-p.HALF_PI, 0,0,p.rotZ);
	}
	
	//move the bot drawn from the data file to lie on the map at approx correct location (from data)
	public void translateBotToDemoMapCtr(){	p.translate(-mapWidth/2.0f,0, 0);}//	p.scale(1.0f,-1.0f,1.0f);}//scale to flip orientation - distances are opposite sign in y
	
}//
