package cs8803Lab1;

import java.awt.event.KeyEvent;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import processing.core.PApplet;
import processing.core.PConstants;
import processing.core.PMatrix3D;
import processing.opengl.PGL;
import processing.opengl.PGraphics3D;
/**
 * cs8803 Lab 1 localization via particle filter
 * 
 * @author john turner
 *
 */
public class Lab1Localization extends PApplet {
	//implement a particle filter to help a robot find itself on a map based on its observations and movements
	//map is grid of 800 x 800 cells, with each cell being 10 cm on a side
	//
	//globals
	//
	public float rotZ = 1;			//for all rotation displays, to maintain appropriate orientation
	//map file name
	public String mapName = "wean.dat";
	public myMap map;
	// degrees between pre-calced ray casts, and radians equivalent
	public final int degPerCasts = 5;
	public final float degPerC2Rad = degPerCasts * PConstants.DEG_TO_RAD;
	public static int dp_incrID = 0;	
	public int camInitialDist = -800;		//initial distance camera is from scene - needs to be negative
	
	public myLogData[] dataLogs;				//all 5 files of data logs - put in localizer
	public String[] logNames = new String[]{"robotdata1.log","ascii-robotdata2.log","ascii-robotdata3.log","ascii-robotdata4.log","ascii-robotdata5.log"};
	
	public ExecutorService th_exec;
	//how many frames to wait to actually refresh/draw
	public final int drawModVal = 1;	
	public int logToSolve = 0;				//which log to solve for	
	public float mapTotProb;				//total prob measure across entire map
	
	public int numSimRuns = 1;				//# of sim calculations for every draw
	public int botIdx = 0;					//idx of current bot reading being simulated
	
	public static final float invSqrt2Pi = 1.0f/sqrt(2.0f*PI);			//used for calculations of normal prob dist
	
	public myVector focusTar;											//target of focus - used in translate to set where the camera is looking
	
	public myLocalizer[] MCL_locs;
	public void settings(){
		size((int)(displayWidth*.95f), (int)(displayHeight*.9f),P3D);
	}
	public void setup(){		
		colorMode(RGB, 255, 255, 255, 255);
		frameRate(60);
		sphereDetail(4);
		initOnce();
		background(bground[0],bground[1],bground[2],bground[3]);		
	}//setup
	
	public void draw(){
		if(flags[initialLoad]){
			draw3DInitLoad();
		} else {
			draw3D();
		}
		drawUI();		
	}//draw
	
	public void draw3DInitLoad(){
		pushMatrix();pushStyle();
		drawSetup();																//initialize camera, lights and scene orientation and set up eye movement
		background(bground[0],bground[1],bground[2],bground[3]);	
		
		buildCanvas();																//build drawing canvas based upon eye-to-scene vector		
		popStyle();popMatrix(); 		
	}
	
	public void draw3D(){
		botIdx = drawCount%(this.dataLogs[0].data.length-1);
		if(flags[runSim]){drawCount++;if(drawCount%scrMsgTime==0){if(consoleStrings.size() != 0){consoleStrings.pollFirst();}}}
		pushMatrix();pushStyle();
		drawSetup();																//initialize camera, lights and scene orientation and set up eye movement
		background(bground[0],bground[1],bground[2],bground[3]);	
		translate(focusTar.x,focusTar.y,focusTar.z);				//center of screen
		map.draw();
		//exec localization simulation
		if(flags[runSim]){for(int i =0; i<numSimRuns;++i){MCL_locs[logToSolve].MCL(botIdx);}}
		//draw results of simulation
		MCL_locs[logToSolve].draw(botIdx);
		drawAxes(100,3, new myPoint(-viewDimW/2.0f+40,0.0f,0.0f), 200, false); 		//for visualisation purposes and to show movement and location in otherwise empty scene
		if(flags[stepSim]){flags[stepSim] = false; flags[runSim]=false;}
		buildCanvas();																//build drawing canvas based upon eye-to-scene vector		
		popStyle();popMatrix(); 
	}//draw3D	
	
	public void drawUI(){
		drawSideBar();					//draw clickable side menu	
		if((!flags[runSim]) || (drawCount % drawModVal == 0)){
			drawOnScreenData();
		}
	}//drawUI
	
	//called once at start of program
	public void initOnce(){
		initVisOnce();
		//flags[initialLoad]=true;				//set true upon initial load, to display data pertaining to program startup
		//flags[runSim]=true;					//start with simulator running
		map = new myMap(this, mapName);		//map from data
		dataLogs = new myLogData[logNames.length];
		MCL_locs = new myLocalizer[logNames.length];
		for(int i=0; i<logNames.length;++i){	
			dataLogs[i] = new myLogData(this,(logNames[i])); 
			System.out.println("Loaded data from file : "+logNames[i]);
			MCL_locs[i] = new myLocalizer(this,dataLogs[i],map);
			System.out.println("Initialized localization calculator for log file : "+ logNames[i]);
		}		
		logToSolve = 0;		//set this one up in 
		focusTar = new myVector(0,-map.mapHeight/2.0f,0);
		initProgram();
	}//initOnce
	
	public void initProgram(){
		initVisProg();
		drawCount = 0;
		MCL_locs[logToSolve] = new myLocalizer(this,dataLogs[logToSolve],map);
		System.out.println("Initialized localization calculator for log file : "+ logNames[logToSolve]);
	}//initProgram

	//debug observation data
	public void debugObs(){	for(int i = 0; i<dataLogs.length; ++i){	debugOneObs(i);}}
	public void debugOneObs(int i){for(int j = 0; j<dataLogs[i].data.length; j+=10){	System.out.println("dataLog["+i+"].data["+j+"] = "+dataLogs[i].data[j]);	}}
	//get string array for onscreen display of debug info for each observation
	public String[] getDebugObs(){
		ArrayList<String> res = new ArrayList<String>();
		List<String>tmp;
		for(int i = 0; i<dataLogs.length; ++i){for(int j = botIdx; j<min(botIdx+20,dataLogs[i].data.length); j++){tmp = Arrays.asList(dataLogs[i].data[j].getInfoStrAra());res.addAll(tmp);}}
		return res.toArray(new String[0]);	
	}
	
	public String[] getDebugObsSamps(){
		ArrayList<String> res = this.MCL_locs[logToSolve].getSampleDataAra();
		return res.toArray(new String[0]);	
	}

	public void setFocus(int tar){
		switch (tar){
		case 0 : {focusTar.set(0,-map.mapHeight/2.0f,0);initCamView();break;} //refocus camera on center
		case 1 : {focusTar.set(map.mapWidth/2.0f,0,0);break;} //refocus camera on bot log data map     
		case 2 : {focusTar.set(-map.mapWidth/2.0f,0,0);break;} //refocus camera on samples              
		case 3 : {focusTar.set(map.mapWidth/2.0f,-map.mapHeight,0);break;} //refocus camera on brf model            
		case 4 : {focusTar.set(-map.mapWidth/2.0f,-map.mapHeight,0);break;} //refocus camera on histogram of lrf data		
		}
	}

	//////////////////////////////////////////////////////
	/// user interaction
	//////////////////////////////////////////////////////	
	
	public void keyPressed(){
		switch (key){
			case '1' : {logToSolve = 0;initProgram();break;}		//re-start program and solve for log 0
			case '2' : {logToSolve = 1;initProgram();break;}		//re-start program and solve for log 1
			case '3' : {logToSolve = 2;initProgram();break;}		//re-start program and solve for log 2
			case '4' : {logToSolve = 3;initProgram();break;}		//re-start program and solve for log 3
			case '5' : {logToSolve = 4;initProgram();break;}		//re-start program and solve for log 4
			case '6' : {setFocus(1);break;}		//refocus camera on bot log data map
			case '7' : {setFocus(2);break;}		//refocus camera on samples
			case '8' : {setFocus(3);break;}		//refocus camera on brf model
			case '9' : {setFocus(4);break;}		//refocus camera on histogram of lrf data
			case '0' : {setFocus(0);break;}		//refocus camera on center
			case 'r' : 
			case 'R' : {MCL_locs[logToSolve].initProposal();break;}
			case ' ' : {flags[runSim] = !flags[runSim]; break;}
			case 'i' : 
			case 'I' : {initProgram();break;}		//re-start program
		}
		if((!flags[shiftKeyPressed])&&(key==CODED)){flags[shiftKeyPressed] = (keyCode  == KeyEvent.VK_SHIFT);}
	}
	public void keyReleased(){
		if((flags[shiftKeyPressed])&&(key==CODED)){ if(keyCode == KeyEvent.VK_SHIFT){clearFlags(new int []{shiftKeyPressed, modView});}}
	}

	//2d range checking of point
	public boolean ptInRange(float x, float y, float minX, float minY, float maxX, float maxY){return ((x > minX)&&(x < maxX)&&(y > minY)&&(y < maxY));}	
	/**
	 * handle mouse presses - print out to console value of particular cell
	 */
	public void mousePressed() {
		if(mouseX<(menuWidth)){//check where mouse is - if in range of side menu, process clicks for ui input	
			if(mouseX>(menuWidth-15)&&(mouseY<15)){showInfo =!showInfo; return;}			//turn on/off info header
			if(mouseY<20){return;}
			int i = (int)((mouseY-(yOff))/(yOff));
			if(clkyFlgs.contains(i)){setFlags(i,!flags[i]);}
		}//handle menu interaction
		else {
			if(!flags[shiftKeyPressed]){flags[mouseClicked] = true;}
		}	
	}// mousepressed	
	
	public void mouseDragged(){
		if(mouseX<(width * menuWidthMult)){	//handle menu interaction
		}
		else {
			if(flags[shiftKeyPressed]){
				flags[modView]=true;
				if(mouseButton == LEFT){			rx-=PI*(mouseY-pmouseY)/height; ry+=PI*(mouseX-pmouseX)/width;} 
				else if (mouseButton == RIGHT) {	dz-=(double)(mouseY-pmouseY);}
			}
		}
	}//mouseDragged()
	
	public void mouseReleased(){
		clearFlags(new int[]{mouseClicked, modView});			
	}
	
	public void clearFlags(int[] idxs){		for(int idx : idxs){flags[idx]=false;}	}	

	//////////////////////////////////////////
	/// graphics and base functionality utilities and variables
	//////////////////////////////////////////
	//visualization variables
	// boolean flags used to control various elements of the program 
	public boolean[] flags;
	
	//dev/debug flags
	public final int initialLoad		= 0;			//set to true on start, turned false after data loaded
	public final int debugMode 			= 1;			//whether we are in debug mode or not	
	public final int saveAnim 			= 2;			//whether we are in debug mode or not	
	//interface flags	
	public final int shiftKeyPressed 	= 3;			//shift pressed
	public final int mouseClicked 		= 4;			//mouse left button is held down	
	public final int modView	 		= 5;			//shift+mouse click+mouse move being used to modify the view		
	public final int runSim				= 6;			//run simulation (if off localization progresses on single pose
	public final int stepSim			= 7;			//run simulation a single step
	public final int showDataSampDB		= 8;			//whether to show data points or samples during debug
	public final int showSampleRCs		= 9;			//show pre-computed sample ray-casts
	public final int sweepBRFCast		= 10;			//sweep through possible BRF raycast values
	
	public final int numFlags = 11;
	
	public boolean showInfo;										//whether or not to show start up instructions for code
	
	public final int[] bground = new int[]{240,240,240,255};		//bground color
	
	public final String[] flagNames = {
			"Loading Initial Data",
			"Debug Mode",		
			"Save Anim", 		
			"Shift-Key Pressed",
			"Click interact", 	
			"Changing View",	
			"Execute Simulation",
			"Single step",
			"Show DataPoints DB",
			"Show Precomputed Sample RCs",
			"Sweep BRF Ray Lengths"
			};
	
	public final String[] altFlagNames = {
			"Loading Initial Data",
			"Debug Mode",		
			"Save Anim", 		
			"Shift-Key Pressed",
			"Click interact", 	
			"Changing View",	 	
			"Execute Simulation",
			"Single step",
			"Show Samples DB",
			"Show Precomputed Sample RCs",
			"Sweep BRF Ray Lengths"
			};
	
	public int[][] flagColors;
	//List<String> places = Arrays.asList
	//flags that can be modified by clicking on screen
	public List<Integer> clkyFlgs = Arrays.asList(
			debugMode, saveAnim,runSim,stepSim,showDataSampDB,showSampleRCs,sweepBRFCast
			);			
	float xOff = 20 , yOff = 20;			//offset values to render boolean menu on side of screen	
	
	// path and filename to save pictures for animation
	public String animPath, animFileName;
	public int animCounter;	
	
	public final int scrMsgTime = 50;									//5 seconds to delay a message 60 fps (used against draw count)
	public ArrayDeque<String> consoleStrings;							//data being printed to console - show on screen
	
	public myVector eyeToMse, 
			eyeToCtr,													//vector from eye to center of cube, to be used to determine which panels of bounding box to show or hide
			canvasNorm, 												//normal of eye-to-mouse toward scene, current drawn object's normal to canvas
			drawSNorm;													//current normal of viewport/screen
	public edge camEdge;												//denotes line perp to cam eye, to use for intersections for mouse selection
	public final int viewDim = 900;
	public int viewDimW, viewDimH;
	public int drawCount;												// counter for draw cycles
	public int simCycles;
	
	public float menuWidthMult = .15f;									//side menu is 15% of screen width
	public float menuWidth;

	public ArrayList<String> DebugInfoAra;								//enable drawing dbug info onto screen
	public String debugInfoString;
	
	//animation control variables	
	public float animCntr, animModMult;
	public final float maxAnimCntr = PI*1000.0f, baseAnimSpd = 1.0f;
	
	private float dz=0, 												// distance to camera. Manipulated with wheel or when
	rx=-0.06f*TWO_PI, ry=-0.04f*TWO_PI;									// view angles manipulated when space pressed but not mouse	
	public myPoint drawEyeLoc,													//rx,ry,dz coords where eye was when drawing - set when first drawing and return eye to this location whenever trying to draw again - rx,ry,dz
			scrCtrInWorld = new myPoint(),									//
			mseLoc = new myPoint(),
			eyeInWorld = new myPoint(),
			oldDfCtr  = new myPoint(),
			dfCtr = new myPoint();											//mouse location projected onto current drawing canvas
	
	public float canvasDim = 1500; 									//canvas dimension for "virtual" 3d		
	public myPoint[] canvas3D;									//3d plane, normal to camera eye, to be used for drawing - need to be in "view space" not in "world space", so that if camera moves they don't change

	public float[] camVals;// = new float[]{width/2.0f, height/2.0f, (height/2.0f) / tan(PI*30.0f / 180.0f), width/2.0f, height/2.0f, 0, 0, 1, 0};
	///////////////////////////////////
	/// generic graphics functions and classes
	///////////////////////////////////
		//1 time initialization of things that won't change
	public void initVisOnce(){		
		initBoolFlags();
		camVals = new float[]{width/2.0f, height/2.0f, (height/2.0f) / tan(PI/6.0f), width/2.0f, height/2.0f, 0, 0, 1, 0};
		showInfo = true;
		//thread executor for multithreading
		th_exec = Executors.newCachedThreadPool();// new ForkJoinPool();
		println("sketchPath " + sketchPath());
		textureMode(NORMAL);	
		menuWidth = width * menuWidthMult;						//width of menu region		
		viewDimW = width;
		viewDimH = height;
		canvas3D = new myPoint[4];		//3 points to define canvas
		canvas3D[0]=new myPoint();canvas3D[1]=new myPoint();canvas3D[2]=new myPoint();canvas3D[3]=new myPoint();
		initCamView();
		drawEyeLoc = new myPoint(-1, -1, -1000);
		eyeInWorld = new myPoint();		
		simCycles = 0;
		eyeToMse = new myVector();		eyeToCtr = new myVector();	drawSNorm = new myVector();	canvasNorm = new myVector(); 						//normal of eye-to-mouse toward scene, current drawn object's normal to canvas
		animPath = sketchPath() + "\\robotMotion_" + (int) random(1000);
		animFileName = "\\" + "botMotionAnim";
		consoleStrings = new ArrayDeque<String>();				//data being printed to console
		
	}
		//init boolean state machine flags for program
	public void initBoolFlags(){
		flags = new boolean[numFlags];
		flagColors = new int[numFlags][3];
		for (int i = 0; i < numFlags; ++i) { flags[i] = false; flagColors[i] = new int[]{(int) random(150),(int) random(100),(int) random(150)}; }	

	}
		//reinit every time reinit
	public void initVisProg(){	
		drawCount = 0;
		debugInfoString = "";
		reInitInfoStr();
	}
	
	public void initCamView(){
		camEdge = new edge();
		dz=camInitialDist; 											// distance to camera. Manipulated with wheel or when 
		ry=-0.0f;rx=-PI/2.0f - ry;				// view angles manipulated when space pressed but not mouse			
	}	

	public void reInitInfoStr(){		DebugInfoAra = new ArrayList<String>();		DebugInfoAra.add("");	}	
	public int addInfoStr(String str){return addInfoStr(DebugInfoAra.size(), str);}
	public int addInfoStr(int idx, String str){	
		int lstIdx = DebugInfoAra.size();
		if(idx >= lstIdx){		for(int i = lstIdx; i <= idx; ++i){	DebugInfoAra.add(i,"");	}}
		setInfoStr(idx,str);	return idx;
	}
	public void setInfoStr(int idx, String str){DebugInfoAra.set(idx,str);	}
	public void drawInfoStr(float sc){//draw text on main part of screen
		pushMatrix();		pushStyle();
		fill(0,0,0,100);
		translate((menuWidth),0);
		scale(sc,sc);
		for(int i = 0; i < DebugInfoAra.size(); ++i){		text((flags[debugMode]?(i<10?"0":"")+i+":     " : "") +"     "+DebugInfoAra.get(i)+"\n\n",0,(10+(12*i)));	}
		popStyle();	popMatrix();
	}		
	//vector and point functions to be compatible with earlier code from jarek's class or previous projects	
	
	//address all flag-setting here, so that if any special cases need to be addressed they can be
	public void setFlags(int idx, boolean val ){
		flags[idx] = val;
		switch (idx){
			case debugMode : {  break;}//anything special for attractMode
			case stepSim : {if(val){setFlags(runSim, true);}break;}
		}		
	}//setFlags	
	
	//drawsInitial setup for each draw
	public void drawSetup(){
		camera(camVals[0],camVals[1],camVals[2],camVals[3],camVals[4],camVals[5],camVals[6],camVals[7],camVals[8]);       // sets a standard perspective
		translate((float)width/2.0f,(float)height/2.0f,(float)dz); // puts origin of model at screen center and moves forward/away by dz
	    setCamOrient();
	    shininess(.1f);
	    ambientLight(55, 55, 55);
	    lightSpecular(222, 222, 222);
	    directionalLight(111, 111, 111, -1,1,-1);
		specular(222, 222, 222);
	}//drawSetup	
	public void setCamOrient(){rotateX((float)rx);rotateY((float)ry); rotateX((float)PI/(2.0f));		}//sets the rx, ry, pi/2 orientation of the camera eye	
	public void unSetCamOrient(){rotateX((float)-PI/(2.0f)); rotateY((float)-ry);   rotateX((float)-rx); }//reverses the rx,ry,pi/2 orientation of the camera eye - paints on screen and is unaffected by camera movement
	public void drawAxes(float len, float stW, myPoint ctr, int alpha, boolean centered){
		pushMatrix();pushStyle();
			strokeWeight((float)stW);
			stroke(255,0,0,alpha);
			if(centered){line(ctr.x-len*.5f,ctr.y,ctr.z,ctr.x+len*.5f,ctr.y,ctr.z);stroke(0,255,0,alpha);line(ctr.x,ctr.y-len*.5f,ctr.z,ctr.x,ctr.y+len*.5f,ctr.z);stroke(0,0,255,alpha);line(ctr.x,ctr.y,ctr.z-len*.5f,ctr.x,ctr.y,ctr.z+len*.5f);} 
			else {		line(ctr.x,ctr.y,ctr.z,ctr.x+len,ctr.y,ctr.z);stroke(0,255,0,alpha);line(ctr.x,ctr.y,ctr.z,ctr.x,ctr.y+len,ctr.z);stroke(0,0,255,alpha);line(ctr.x,ctr.y,ctr.z,ctr.x,ctr.y,ctr.z+len);}
		popStyle();	popMatrix();	
	}//	drawAxes
	public void drawAxes(float len, float stW, myPoint ctr, myVector[] _axis, int alpha){
		pushMatrix();pushStyle();
			strokeWeight((float)stW);stroke(255,0,0,alpha);line(ctr.x,ctr.y,ctr.z,ctr.x+(_axis[0].x)*len,ctr.y+(_axis[0].y)*len,ctr.z+(_axis[0].z)*len);stroke(0,255,0,alpha);line(ctr.x,ctr.y,ctr.z,ctr.x+(_axis[1].x)*len,ctr.y+(_axis[1].y)*len,ctr.z+(_axis[1].z)*len);	stroke(0,0,255,alpha);	line(ctr.x,ctr.y,ctr.z,ctr.x+(_axis[2].x)*len,ctr.y+(_axis[2].y)*len,ctr.z+(_axis[2].z)*len);
		popStyle();	popMatrix();	
	}//	drawAxes
	public void drawText(String str, float x, float y, float z, int clr){
		int[] c = getClr(clr);
		pushMatrix();	pushStyle();
			fill(c[0],c[1],c[2],c[3]);
			translate((float)x,(float)y,(float)z);
			unSetCamOrient();
			text(str,0,0,0);		
		popStyle();	popMatrix();	
	}//drawText	
	public void savePic(){		save(animPath + animFileName + ((animCounter < 10) ? "000" : ((animCounter < 100) ? "00" : ((animCounter < 1000) ? "0" : ""))) + animCounter + ".jpg");		animCounter++;		}
	public void line(double x1, double y1, double z1, double x2, double y2, double z2){line((float)x1,(float)y1,(float)z1,(float)x2,(float)y2,(float)z2 );}
	public void drawOnScreenData(){
		if(flags[debugMode]){
			pushMatrix();pushStyle();			
			reInitInfoStr();
			addInfoStr(0,"mse loc on screen : " + new myPoint(mouseX, mouseY,0) + " mse loc in world :"+mseLoc +"  Eye loc in world :"+ eyeInWorld); 
			String[] res = flags[showDataSampDB] ?  getDebugObs() : getDebugObsSamps();
			//for(int s=0;s<res.length;++s) {	addInfoStr(res[s]);}				//add info to string to be displayed for debug
			int numToPrint = min(res.length,80);
			for(int s=0;s<numToPrint;++s) {	addInfoStr(res[s]);}				//add info to string to be displayed for debug
			drawInfoStr(1.0f); 	
			popStyle();	popMatrix();		
		}
		else {
			pushMatrix();pushStyle();			
			reInitInfoStr();	
			if(showInfo){
		      addInfoStr(0,"Click the light green box to the left to toggle showing this message.");
		      addInfoStr(1,"--Shift-Click-Drag to change view.  Shift-RClick-Drag to zoom.");
			}
			String[] res = consoleStrings.toArray(new String[0]);
			int dispNum = min(res.length, 80);
			for(int i=0;i<dispNum;++i){addInfoStr(res[i]);}
		    drawInfoStr(1.1f); 
			popStyle();	popMatrix();		
		}
	}
	//print informational string data to console, and to screen
	public void outStr2Scr(String str){
		System.out.println(str);
		consoleStrings.add(str);		//add console string output to screen display- decays over time
	}
	
	public void dispFlagTxt(String txt, int[] clrAra, boolean showSphere){
		setFill(clrAra, 255); 
		if(showSphere){setStroke(clrAra, 255);		sphere(5);	} 
		else {	noStroke();		}
		translate(-xOff*.5f,yOff*.5f);
		text(""+txt,xOff,-yOff*.25f);	
	}
	
	public void drawMouseBox(){
		pushMatrix();pushStyle();			
		translate((width * menuWidthMult-10),0);
	    setColorValFill(this.showInfo ? gui_LightGreen : gui_DarkRed);
		rect(0,0,10, 10);
		popStyle();	popMatrix();		
	}
	public void setFill(int[] clr, int alpha){fill(clr[0],clr[1],clr[2], alpha);}
	public void setStroke(int[] clr, int alpha){stroke(clr[0],clr[1],clr[2], alpha);}
	//draw side bar on left side of screen to enable interaction with booleans
	public void drawSideBar(){
		pushMatrix();pushStyle();
		hint(DISABLE_DEPTH_TEST);
		noLights();
		setColorValFill(gui_White);
		rect(0,0,width*menuWidthMult, height);
		drawMouseBox();
		//draw booleans and their state		
		translate(10,yOff);
		setColorValFill(gui_Black);
		text("Boolean Flags",0,-yOff*.25f);
		for(int i =0; i<numFlags; ++i){
			translate(xOff*.5f,yOff*.5f);
			if(flags[i] ){													dispFlagTxt(flagNames[i],flagColors[i], true);			}
			else {	if(flagNames[i].equals(altFlagNames[i])) {	dispFlagTxt(flagNames[i],new int[]{180,180,180}, false);}	
					else {													dispFlagTxt(altFlagNames[i],new int[]{0,255-flagColors[i][1],255-flagColors[i][2]}, true);}		
			}
		}		
		
		hint(ENABLE_DEPTH_TEST);
		popStyle();	popMatrix();	
	}//drawSideBar		
	
	public void drawMseEdge(){//draw mouse sphere and edge normal to cam eye through mouse sphere 
		pushMatrix();
		pushStyle();
			strokeWeight(1f);
			stroke(255,0,0,100);
			camEdge.set(1000, eyeToMse, dfCtr);		//build edge through mouse point normal to camera eye	
			camEdge.drawMe();
			translate((float)dfCtr.x, (float)dfCtr.y, (float)dfCtr.z);
			//project mouse point on bounding box walls
			drawAxes(10000,1f, myPoint.ZEROPT, 100, true);//
			//draw intercept with box
			stroke(0,0,0,255);
			show(myPoint.ZEROPT,3);
			drawText(""+dfCtr,4, 4, 4,0);
		popStyle();
		popMatrix();		
	}//drawMseEdge	
	//find points to define plane normal to camera eye, at set distance from camera, to use drawing canvas 	
	public void buildCanvas(){
		mseLoc = MouseScr();		
		scrCtrInWorld = pick(viewDimW/2, viewDimH/2);		
		myVector A = new myVector(scrCtrInWorld, pick(viewDimW, -viewDimH)),	B = new myVector(scrCtrInWorld, pick(viewDimW, 0));	//ctr to upper right, ctr to lower right		
		drawSNorm = U(myVector._cross(A,B));				 													//normal to canvas that is colinear with view normal to ctr of screen
		eyeInWorld = myPoint._add(new myPoint(scrCtrInWorld), myPoint._dist(pick(0,0), scrCtrInWorld), drawSNorm);								//location of "eye" in world space
		eyeToCtr = new myVector(eyeInWorld, new myPoint(0,0,0));
		eyeToMse = U(eyeInWorld, mseLoc);		//unit vector in world coords of "eye" to mouse location
		myVector planeTan = U(myVector._cross(drawSNorm, U(drawSNorm.x+10,drawSNorm.y+10,drawSNorm.z+10)));			//result of vector crossed with normal will be in plane described by normal
     	for(int i =0;i<canvas3D.length;++i){
     		canvas3D[i] = new myPoint(myVector._mult(planeTan, canvasDim));
     		planeTan = U(myVector._cross(drawSNorm, planeTan));												//this effectively rotates around center point by 90 degrees -builds a square
     	}
     	oldDfCtr = new myPoint(dfCtr);
     	dfCtr = getPlInterSect(mseLoc,eyeToMse);
     	drawMseEdge();
	}//buildCanvas()
	
	//returns unit vector in world coords of "eye" to point location
	public myVector getUnitEyeToPt(myPoint p){	return U(eyeInWorld, p);}
	
	//find pt in drawing plane that corresponds with mouse location and camera eye normal
	public myPoint getPlInterSect(myPoint p, myVector camEyeNorm){
		myPoint dctr = new myPoint(0,0,0);	//actual click location on visible plane
		 // if ray from E along T intersects triangle (A,B,C), return true and set proposal to the intersection point
		intersectPl(p, camEyeNorm, canvas3D[0],canvas3D[1],canvas3D[2],  dctr);//find point where mouse ray intersects canvas
		return dctr;		
	}//getPlInterSect	
	
	//line bounded by verts - from a to b new myPoint(x,y,z); 
	public class edge{ public myPoint a, b;
		public edge (){a=new myPoint(0,0,0); b=new myPoint(0,0,0);}
		public edge (myPoint _a, myPoint _b){a=new myPoint(_a); b=new myPoint(_b);}
		public void set(float d, myVector dir, myPoint _p){	set( myPoint._add(_p,-d,new myVector(dir)), myPoint._add(_p,d,new myVector(dir)));} 
		public void set(myPoint _a, myPoint _b){a=new myPoint(_a); b=new myPoint(_b);}
		public myVector v(){return new myVector(b.x-a.x, b.y-a.y, b.z-a.z);}			//vector from a to b
		public myVector dir(){return U(v());}
		public double len(){return  myPoint._dist(a,b);}
		public double distFromPt(myPoint P) {return myVector._det3(dir(),new myVector(a,P)); };
		public void drawMe(){line(a.x,a.y,a.z,b.x,b.y,b.z); }
	    public String toString(){return "a:"+a+" to b:"+b+" len:"+len();}
	}
	public myPoint pick(int mX, int mY){
		PGL pgl = beginPGL();
		FloatBuffer depthBuffer = ByteBuffer.allocateDirect(1 << 2).order(ByteOrder.nativeOrder()).asFloatBuffer();
		int newMy = viewDimW - mY;
		//pgl.readPixels(mX, gridDimDp - mY - 1, 1, 1, PGL.DEPTH_COMPONENT, PGL.FLOAT, depthBuffer);
		pgl.readPixels(mX, newMy, 1, 1, PGL.DEPTH_COMPONENT, PGL.FLOAT, depthBuffer);
		float depthValue = depthBuffer.get(0);
		depthBuffer.clear();
		endPGL();
	
		//get 3d matrices
		PGraphics3D p3d = (PGraphics3D)g;
		PMatrix3D proj = p3d.projection.get();
		PMatrix3D modelView = p3d.modelview.get();
		PMatrix3D modelViewProjInv = proj; modelViewProjInv.apply( modelView ); modelViewProjInv.invert();
	  
		float[] viewport = {0, 0, viewDimW, viewDimH};
		//float[] viewport = {0, 0, p3d.width, p3d.height};
		  
		float[] normalized = new float[4];
		normalized[0] = ((mX - viewport[0]) / viewport[2]) * 2.0f - 1.0f;
		normalized[1] = ((newMy - viewport[1]) / viewport[3]) * 2.0f - 1.0f;
		normalized[2] = depthValue * 2.0f - 1.0f;
		normalized[3] = 1.0f;
	  
		float[] unprojected = new float[4];
	  
		modelViewProjInv.mult( normalized, unprojected );
		return new myPoint( unprojected[0]/unprojected[3], unprojected[1]/unprojected[3], unprojected[2]/unprojected[3] );
	}		
		
	public myPoint MouseScr() {return pick(mouseX,mouseY);}                                          		// current mouse location
	 
	public myPoint Mouse() {return new myPoint(mouseX, mouseY,0);}                                          			// current mouse location
	public myVector MouseDrag() {return new myVector(mouseX-pmouseX,mouseY-pmouseY,0);};                     			// vector representing recent mouse displacement
	
	public myVector U(myVector v){myVector u = new myVector(v); return u._normalize(); }
	public myVector U(myPoint a, myPoint b){myVector u = new myVector(a,b); return u._normalize(); }
	public myVector U(float x, float y, float z) {myVector u = new myVector(x,y,z); return u._normalize();}
	
	public myVector normToPlane(myPoint A, myPoint B, myPoint C) {return myVector._cross(new myVector(A,B),new myVector(A,C)); };   // normal to triangle (A,B,C), not normalized (proportional to area)
	public void circle(myPoint P, float r) {ellipse(P.x, P.y, 2*r, 2*r);}
	public void line(myPoint p, myPoint q){line(p.x,p.y,p.z, q.x,q.y,q.z);}
	public void show(myPoint P, float r, int clr) {pushMatrix(); pushStyle(); setColorValFill(clr); setColorValStroke(clr);sphereDetail(5);translate((float)P.x,(float)P.y,(float)P.z); sphere((float)r); popStyle(); popMatrix();} // render sphere of radius r and center P)
	public void show(myPoint P, float r){show(P,r, gui_Black);}
	public void show(myPoint P, String s) {text(s, (float)P.x, (float)P.y, (float)P.z); } // prints string s in 3D at P
	public void show(myPoint P, String s, myVector D) {text(s, (float)(P.x+D.x), (float)(P.y+D.y), (float)(P.z+D.z));  } // prints string s in 3D at P+D
	public boolean intersectPl(myPoint E, myVector T, myPoint A, myPoint B, myPoint C, myPoint X) { // if ray from E along T intersects triangle (A,B,C), return true and set proposal to the intersection point
		myVector EA=new myVector(E,A), AB=new myVector(A,B), AC=new myVector(A,C); 		float t = (float)(myVector._mixProd(EA,AC,AB) / myVector._mixProd(T,AC,AB));		X.set(myPoint._add(E,t,T));		return true;
	}	

	
	public void setColorValFill(int colorVal){
		switch (colorVal){
			case gui_rnd				: { fill(random(255),random(255),random(255),255); ambient(120,120,120);break;}
	    	case gui_White  			: { fill(255,255,255,255); ambient(255,255,255); break; }
	    	case gui_Gray   			: { fill(120,120,120,255); ambient(120,120,120); break;}
	    	case gui_Yellow 			: { fill(255,255,0,255); ambient(255,255,0); break; }
	    	case gui_Cyan   			: { fill(0,255,255,255); ambient(0,255,255); break; }
	    	case gui_Magenta			: { fill(255,0,255,255); ambient(255,0,255); break; }
	    	case gui_Red    			: { fill(255,0,0,255); ambient(255,0,0); break; }
	    	case gui_Blue				: { fill(0,0,255,255); ambient(0,0,255); break; }
	    	case gui_Green				: { fill(0,255,0,255); ambient(0,255,0); break; } 
	    	case gui_DarkGray   		: { fill(80,80,80,255); ambient(80,80,80); break;}
	    	case gui_DarkRed    		: { fill(120,0,0,255); ambient(120,0,0); break;}
	    	case gui_DarkBlue   		: { fill(0,0,120,255); ambient(0,0,120); break;}
	    	case gui_DarkGreen  		: { fill(0,120,0,255); ambient(0,120,0); break;}
	    	case gui_DarkYellow 		: { fill(120,120,0,255); ambient(120,120,0); break;}
	    	case gui_DarkMagenta		: { fill(120,0,120,255); ambient(120,0,120); break;}
	    	case gui_DarkCyan   		: { fill(0,120,120,255); ambient(0,120,120); break;}		   
	    	case gui_LightGray   		: { fill(200,200,200,255); ambient(200,200,200); break;}
	    	case gui_LightRed    		: { fill(255,110,110,255); ambient(255,110,110); break;}
	    	case gui_LightBlue   		: { fill(110,110,255,255); ambient(110,110,255); break;}
	    	case gui_LightGreen  		: { fill(110,255,110,255); ambient(110,255,110); break;}
	    	case gui_LightYellow 		: { fill(255,255,110,255); ambient(255,255,110); break;}
	    	case gui_LightMagenta		: { fill(255,110,255,255); ambient(255,110,255); break;}
	    	case gui_LightCyan   		: { fill(110,255,255,255); ambient(110,255,255); break;}	    	
	    	case gui_Black			 	: { fill(0,0,0,255); ambient(0,0,0); break;}//
	    	case gui_TransBlack  	 	: { fill(0x00010100); ambient(0,0,0); break;}//	have to use hex so that alpha val is not lost    	
	    	case gui_FaintGray 		 	: { fill(77,77,77,77); ambient(77,77,77); break;}//
	    	case gui_FaintRed 	 	 	: { fill(110,0,0,100); ambient(110,0,0); break;}//
	    	case gui_FaintBlue 	 	 	: { fill(0,0,110,100); ambient(0,0,110); break;}//
	    	case gui_FaintGreen 	 	: { fill(0,110,0,100); ambient(0,110,0); break;}//
	    	case gui_FaintYellow 	 	: { fill(110,110,0,100); ambient(110,110,0); break;}//
	    	case gui_FaintCyan  	 	: { fill(0,110,110,100); ambient(0,110,110); break;}//
	    	case gui_FaintMagenta  	 	: { fill(110,0,110,100); ambient(110,0,110); break;}//
	    	case gui_TransGray 	 	 	: { fill(120,120,120,30); ambient(120,120,120); break;}//
	    	case gui_TransRed 	 	 	: { fill(255,0,0,150); ambient(255,0,0); break;}//
	    	case gui_TransBlue 	 	 	: { fill(0,0,255,150); ambient(0,0,255); break;}//
	    	case gui_TransGreen 	 	: { fill(0,255,0,150); ambient(0,255,0); break;}//
	    	case gui_TransYellow 	 	: { fill(255,255,0,150); ambient(255,255,0); break;}//
	    	case gui_TransCyan  	 	: { fill(0,255,255,150); ambient(0,255,255); break;}//
	    	case gui_TransMagenta  	 	: { fill(255,0,255,150); ambient(255,0,255); break;}//   	
	    	default         			: { fill(255,255,255,255); ambient(255,255,255); break; }
	    	    	
		}//switch	
	}//setcolorValFill
	public void setColorValStroke(int colorVal){
		switch (colorVal){
	    	case gui_White  	 	    : { stroke(255,255,255,255); break; }
 	    	case gui_Gray   	 	    : { stroke(120,120,120,255); break;}
	    	case gui_Yellow      	    : { stroke(255,255,0,255); break; }
	    	case gui_Cyan   	 	    : { stroke(0,255,255,255); break; }
	    	case gui_Magenta	 	    : { stroke(255,0,255,255);  break; }
	    	case gui_Red    	 	    : { stroke(255,120,120,255); break; }
	    	case gui_Blue		 	    : { stroke(120,120,255,255); break; }
	    	case gui_Green		 	    : { stroke(120,255,120,255); break; }
	    	case gui_DarkGray    	    : { stroke(80,80,80,255); break; }
	    	case gui_DarkRed     	    : { stroke(120,0,0,255); break; }
	    	case gui_DarkBlue    	    : { stroke(0,0,120,255); break; }
	    	case gui_DarkGreen   	    : { stroke(0,120,0,255); break; }
	    	case gui_DarkYellow  	    : { stroke(120,120,0,255); break; }
	    	case gui_DarkMagenta 	    : { stroke(120,0,120,255); break; }
	    	case gui_DarkCyan    	    : { stroke(0,120,120,255); break; }	   
	    	case gui_LightGray   	    : { stroke(200,200,200,255); break;}
	    	case gui_LightRed    	    : { stroke(255,110,110,255); break;}
	    	case gui_LightBlue   	    : { stroke(110,110,255,255); break;}
	    	case gui_LightGreen  	    : { stroke(110,255,110,255); break;}
	    	case gui_LightYellow 	    : { stroke(255,255,110,255); break;}
	    	case gui_LightMagenta	    : { stroke(255,110,255,255); break;}
	    	case gui_LightCyan   		: { stroke(110,255,255,255); break;}		   
	    	case gui_Black				: { stroke(0,0,0,255); break;}
	    	case gui_TransBlack  		: { stroke(1,1,1,1); break;}	    	
	    	case gui_FaintGray 			: { stroke(120,120,120,250); break;}
	    	case gui_FaintRed 	 		: { stroke(110,0,0,250); break;}
	    	case gui_FaintBlue 	 		: { stroke(0,0,110,250); break;}
	    	case gui_FaintGreen 		: { stroke(0,110,0,250); break;}
	    	case gui_FaintYellow 		: { stroke(110,110,0,250); break;}
	    	case gui_FaintCyan  		: { stroke(0,110,110,250); break;}
	    	case gui_FaintMagenta  		: { stroke(110,0,110,250); break;}
	    	case gui_TransGray 	 		: { stroke(150,150,150,60); break;}
	    	case gui_TransRed 	 		: { stroke(255,0,0,120); break;}
	    	case gui_TransBlue 	 		: { stroke(0,0,255,120); break;}
	    	case gui_TransGreen 		: { stroke(0,255,0,120); break;}
	    	case gui_TransYellow 		: { stroke(255,255,0,120); break;}
	    	case gui_TransCyan  		: { stroke(0,255,255,120); break;}
	    	case gui_TransMagenta  		: { stroke(255,0,255,120); break;}
	    	default         			: { stroke(55,55,255,255); break; }
		}//switch	
	}//setcolorValStroke	
	
	//returns one of 30 predefined colors as an array (to support alpha)
	public int[] getClr(int colorVal){
		switch (colorVal){
    	case gui_Gray   		         : { return new int[] {120,120,120,255}; }
    	case gui_White  		         : { return new int[] {255,255,255,255}; }
    	case gui_Yellow 		         : { return new int[] {255,255,0,255}; }
    	case gui_Cyan   		         : { return new int[] {0,255,255,255};} 
    	case gui_Magenta		         : { return new int[] {255,0,255,255};}  
    	case gui_Red    		         : { return new int[] {255,0,0,255};} 
    	case gui_Blue			         : { return new int[] {0,0,255,255};}
    	case gui_Green			         : { return new int[] {0,255,0,255};}  
    	case gui_DarkGray   	         : { return new int[] {80,80,80,255};}
    	case gui_DarkRed    	         : { return new int[] {120,0,0,255};}
    	case gui_DarkBlue  	 	         : { return new int[] {0,0,120,255};}
    	case gui_DarkGreen  	         : { return new int[] {0,120,0,255};}
    	case gui_DarkYellow 	         : { return new int[] {120,120,0,255};}
    	case gui_DarkMagenta	         : { return new int[] {120,0,120,255};}
    	case gui_DarkCyan   	         : { return new int[] {0,120,120,255};}	   
    	case gui_LightGray   	         : { return new int[] {200,200,200,255};}
    	case gui_LightRed    	         : { return new int[] {255,110,110,255};}
    	case gui_LightBlue   	         : { return new int[] {110,110,255,255};}
    	case gui_LightGreen  	         : { return new int[] {110,255,110,255};}
    	case gui_LightYellow 	         : { return new int[] {255,255,110,255};}
    	case gui_LightMagenta	         : { return new int[] {255,110,255,255};}
    	case gui_LightCyan   	         : { return new int[] {110,255,255,255};}
    	case gui_Black			         : { return new int[] {0,0,0,255};}
    	case gui_FaintGray 		         : { return new int[] {110,110,110,255};}
    	case gui_FaintRed 	 	         : { return new int[] {110,0,0,255};}
    	case gui_FaintBlue 	 	         : { return new int[] {0,0,110,255};}
    	case gui_FaintGreen 	         : { return new int[] {0,110,0,255};}
    	case gui_FaintYellow 	         : { return new int[] {110,110,0,255};}
    	case gui_FaintCyan  	         : { return new int[] {0,110,110,255};}
    	case gui_FaintMagenta  	         : { return new int[] {110,0,110,255};}
    	
    	case gui_TransBlack  	         : { return new int[] {1,1,1,100};}  	
    	case gui_TransGray  	         : { return new int[] {110,110,110,100};}
    	case gui_TransLtGray  	         : { return new int[] {180,180,180,100};}
    	case gui_TransRed  	         	 : { return new int[] {110,0,0,100};}
    	case gui_TransBlue  	         : { return new int[] {0,0,110,100};}
    	case gui_TransGreen  	         : { return new int[] {0,110,0,100};}
    	case gui_TransYellow  	         : { return new int[] {110,110,0,100};}
    	case gui_TransCyan  	         : { return new int[] {0,110,110,100};}
    	case gui_TransMagenta  	         : { return new int[] {110,0,110,100};}	
    	case gui_TransWhite  	         : { return new int[] {220,220,220,150};}	
    	default         		         : { return new int[] {255,255,255,255};}    
		}//switch
	}//getClr	
	
	public int[] getRndClr(int alpha){return new int[]{(int)random(0,250),(int)random(0,250),(int)random(0,250),alpha};	}
	public int[] getRndClr(){return getRndClr(255);	}		
	public Integer[] getClrMorph(int a, int b, float t){return getClrMorph(getClr(a), getClr(b), t);}    
	public Integer[] getClrMorph(int[] a, int[] b, float t){
		if(t==0){return new Integer[]{a[0],a[1],a[2],a[3]};} else if(t==1){return new Integer[]{b[0],b[1],b[2],b[3]};}
		return new Integer[]{(int)(((1.0f-t)*a[0])+t*b[0]),(int)(((1.0f-t)*a[1])+t*b[1]),(int)(((1.0f-t)*a[2])+t*b[2]),(int)(((1.0f-t)*a[3])+t*b[3])};
	}

	//used to generate random color
	public static final int gui_rnd = -1;
	//color indexes
	public static final int gui_Black 	= 0;
	public static final int gui_White 	= 1;	
	public static final int gui_Gray 	= 2;
	
	public static final int gui_Red 	= 3;
	public static final int gui_Blue 	= 4;
	public static final int gui_Green 	= 5;
	public static final int gui_Yellow 	= 6;
	public static final int gui_Cyan 	= 7;
	public static final int gui_Magenta = 8;
	
	public static final int gui_LightRed = 9;
	public static final int gui_LightBlue = 10;
	public static final int gui_LightGreen = 11;
	public static final int gui_LightYellow = 12;
	public static final int gui_LightCyan = 13;
	public static final int gui_LightMagenta = 14;
	public static final int gui_LightGray = 15;

	public static final int gui_DarkCyan = 16;
	public static final int gui_DarkYellow = 17;
	public static final int gui_DarkGreen = 18;
	public static final int gui_DarkBlue = 19;
	public static final int gui_DarkRed = 20;
	public static final int gui_DarkGray = 21;
	public static final int gui_DarkMagenta = 22;
	
	public static final int gui_FaintGray = 23;
	public static final int gui_FaintRed = 24;
	public static final int gui_FaintBlue = 25;
	public static final int gui_FaintGreen = 26;
	public static final int gui_FaintYellow = 27;
	public static final int gui_FaintCyan = 28;
	public static final int gui_FaintMagenta = 29;
	
	public static final int gui_TransBlack = 30;
	public static final int gui_TransGray = 31;
	public static final int gui_TransMagenta = 32;	
	public static final int gui_TransLtGray = 33;
	public static final int gui_TransRed = 34;
	public static final int gui_TransBlue = 35;
	public static final int gui_TransGreen = 36;
	public static final int gui_TransYellow = 37;
	public static final int gui_TransCyan = 38;	
	public static final int gui_TransWhite = 39;	
	
}
