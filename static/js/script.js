// From STS-PiLot-MarkOrion
//

function makeHttpObject() {
    try {return new XMLHttpRequest();}
    catch (error) {}
    try {return new ActiveXObject("Msxml2.XMLHTTP");}
    catch (error) {}
    try {return new ActiveXObject("Microsoft.XMLHTTP");}
    catch (error) {}
    throw new Error("Could not create HTTP request object.");
}
var request = makeHttpObject();
var getSensors = makeHttpObject();
var doubleClickTimer = 300;

//jwc o var heartbeatTimer = 1000;
// y too long, restore o: var heartbeatTimer = 1000000;  // 1000sec = 16.67min
// y var heartbeatTimer = 1000;  // 1sec
// y var heartbeatTimer = 10000;  // 10sec
// y var heartbeatTimer = 1000000;  // 1000sec = 16.67min
// jwc improve real-time stats
// jwc var heartbeatTimer = 3000;  // 3sec
var heartbeatTimer = 1000;  // 1sec

//jwc o var linkCheckTimer = 5000;
var linkCheckTimer = 1000000;  // 1000sec = 16.67min

var newSpeedL = 0;
var newSpeedR = 0;
var actSpeedL = 0;
var actSpeedR = 0;
var doubleClick = false;
var showHUD = true;
var video = false;
var framerate = 0;
var linkOn = true;

// jwc o var videoimg = '<img src="/video_feed.mjpg" alt="Connecting to Live Video">'
var videoimg = '<img style="transform:rotate(181deg);" "src="/video_feed" alt="Connecting to Live Video">'

var frameimg = '<img id="sframe" src="/single_frame.jpg" alt="Connecting to Live Video">'
var framesrc = '/single_frame.jpg'
var inactive = "0px";
var active = "10px solid black";

var clickTimer = setTimeout(reset_doubleclick, doubleClickTimer);

// jwc: every 'linkCheckTimer' (5sec), 'linkCheck' times out and calls 'linkLost()'
var linkCheck = setTimeout(linkLost, linkCheckTimer);

// jwc: every 'heartbeatTimer' (1sec), 'heartbeat()' called
setInterval(heartbeat, heartbeatTimer);

getSensors.onreadystatechange = updateHUD;

setTimeout(checkVideo, 500);

function linkLost() {
	var status = 'Status<br>Link: DOWN!';
	var sensors = 'Sensors<br>No Data!';
	document.getElementById("status").innerHTML = status;
    	document.getElementById("sensors").innerHTML = sensors;
    	document.getElementById("video").innerHTML = '&nbsp';
    	linkOn = false;
}

function checkVideo() {
	if (video) {
		if (framerate > 0) {
			document.getElementById("video").innerHTML = frameimg;
			setInterval(reloadFrame, framerate);
		} else {
			document.getElementById("video").innerHTML = videoimg;
		}
	}
}

function reloadFrame() {
	document.getElementById("sframe").src = framesrc + '?' + new Date().getTime();
}
function heartbeat() {
    var heartbeat_url = "/heartbeat";
    getSensors.open("GET", heartbeat_url, true);
    getSensors.send(null);
}
function updateHUD(e) {
	var status;
    var sensors;
    var score;

    // jwc: https://www.w3schools.com/xml/xml_http.asp explains following logic:
	if (getSensors.readyState == 4 && getSensors.status == 200) {
        var response = JSON.parse(getSensors.responseText);
        video = response.v;
        framerate = response.f;
        
        // jwc multiple-spacing gets reduced to one-space, so use '.' for column justification
        // jwc o status = "STS-PiLot Status<br>Link: online<br>Videolink: ";
        // status  = "*** HUD: Primary ***";
        status  = "";

        status += "<br>BatteryUps_Volts_Input_V ..(3.8 min): ";
        status += response.bvi;
        status += "<br>BatteryUps_Volts_Output_V .(5.0 avg): ";
        status += response.bvo;
        status += "<br>BatteryUps_Volts_Battery_V (3.1-4.3): ";
        status += response.bvb;
        
        status += "<br>";
        status += "<br>Motor-L: ";
        status += response.l;
        status += "<br>Motor-R: ";
        status += response.r;

        status += "<br>";
        status += "<br>BatteryUps_Temp_C ( 60max): ";
        status += response.btc;
        status += "<br>BatteryUps_Temp_F (140max): ";
        status += response.btf;

        
        // jwc o sensors = "*** Sensors ***<br>Digital 1: ";
        // sensors = "*** HUD: Secondary ***<br>Digital 1: ";
        sensors  = "";
        sensors += "<br>Cam_Servo_02_Tilt_Degrees: ";
        sensors += response.s2;
        sensors += "<br>Cam_Servo_01_Pan_Degrees : ";
        sensors += response.s1;

        sensors += "<br>";
        sensors += "<br>Arm_Servo_03_Degrees: ";
        sensors += response.s3;

        sensors += "<br>";
        sensors += "<br>Web-Link .: online";    
        sensors += "<br>Video-Link: ";
        if (framerate > 0) {
		    sensors += (Math.ceil(1000 / framerate) + " FPS");
	    } else {
        	sensors += video;
        }

        sensors += "<br>";
        sensors += "<br>Arm_Servo_04_Degrees: ";
        sensors += response.s4;

        sensors += "<br>";
        sensors += "<br>Motor-L-AntiDrift: ";
        sensors += response.lt;
        sensors += "<br>Motor-R-AntiDrift: ";
        sensors += response.rt;

        //jwc not needed yet:
        //
        // sensors += "<br>Digital 1: ";
        // sensors += response.i1;
        // sensors += "<br>Digital 2: ";
        // sensors += response.i2;
        // sensors += "<br>Digital 3: ";
        // sensors += response.i3;
        // sensors += "<br>Digital 4: ";
        // sensors += response.i4;

        //jwc not needed yet
        //
        // sensors += "<br>";
        // sensors += "<br>Analog 1: ";
        // sensors += response.a1;
        // sensors += "<br>Analog 2: ";
        // sensors += response.a2;
        // sensors += "<br>Analog 3: ";
        // sensors += response.a3;
        // sensors += "<br>Analog 4: ";
        // sensors += response.a4;

        score  = "";
        score += "<br>Score_Targeted_Dict: ";
        score += response.sc;

        document.getElementById("status").innerHTML = status;
        document.getElementById("sensors").innerHTML = sensors;
        document.getElementById("score").innerHTML = score;
        // button_status('blue', response.b);
        // button_status('yellow', response.y);
        // button_status('red', response.c);
        // button_status('green', response.g);

        $('#lmtrim-val').html(response.lt);
        $('#rmtrim-val').html(response.rt);
        $('#speed-input-val').html(response.l);
        $('#heading-input-val').html(response.l);

        $('#servo_Cam_01_Pan_Degrees_FrontEnd_Response_Id').html(response.s1);
        $('#servo_Cam_02_Tilt_Degrees_FrontEnd_Response_Id').html(response.s2);
        $('#servo_Arm_03_Degrees_FrontEnd_Response_Id').html(response.s3);

        // jwc: reset 'linkCheck' timeout since currently processing heartbeat, which is good
        //
        clearTimeout(linkCheck);
		linkCheck = setTimeout(linkLost, linkCheckTimer);
        // **jwc: disable this, seems destructive non-helping, 
        // ** perhaps let user manually refresh/reload browser
        //jwc o n // jwc: if 'linkOn' false, then reload browser
		//jwc o n if (!linkOn) {
		//jwc o n 	location.reload();
		//jwc o n }
        // jwc: since processing heartbeat, 'linkOn' is true
        linkOn = true;
	}
}
function toggle_hud() {
	button_status('hud', showHUD);
	showHUD = !showHUD;
	if (showHUD) {
		document.getElementById("overlay").style.display = 'block';
	} else {
		document.getElementById("overlay").style.display = 'none';
	}
}
function button_status(button, status) {
	if (status) {
		document.getElementById(button).style.opacity = '0.5';
	} else {
		document.getElementById(button).style.opacity = '1';
	}
}
function reset_doubleclick() {
    doubleClick = false;
}
function set_doubleclick() {
    clearTimeout(clickTimer);
    clickTimer = setTimeout(reset_doubleclick, doubleClickTimer);
    doubleClick = true;
}
function motor_l(lspeed) {
    newSpeedL = lspeed;
    if (lspeed == actSpeedL && lspeed != actSpeedR && doubleClick) {
	newSpeedR = lspeed;
	doubleClick = false;
    } else {
	set_doubleclick();
    }
    set_motor();
}
function motor_r(rspeed) {
    newSpeedR = rspeed;
    if (rspeed == actSpeedR && rspeed != actSpeedL && doubleClick) {
	newSpeedL = rspeed;
	doubleClick = false;
    } else {
	set_doubleclick();
    }
    set_motor();
}
function set_motor() {
    var motor_url = "/motor?l=" + newSpeedL.toString() + '&r=' + newSpeedR.toString();
    request.open("GET", motor_url, true);
    request.send(null);
    var oldId = "l" + actSpeedL.toString();
    var newId = "l" + newSpeedL.toString();
    document.getElementById(oldId).style.outline = inactive;
    document.getElementById(newId).style.outline = active;
    oldId = "r" + actSpeedR.toString();
    newId = "r" + newSpeedR.toString();
    document.getElementById(oldId).style.outline = inactive;
    document.getElementById(newId).style.outline = active;
    actSpeedL = newSpeedL;
    actSpeedR = newSpeedR;
}
function touchpad(pad) {
    var touchpad_url = "/touchpad?pad=" + pad.toString();
    request.open("GET", touchpad_url, true);
    request.send(null);
}
function brake() {
    newSpeedR = 0;
    newSpeedL = 0;
    set_motor();
}
heartbeat();


/* jwc y 

function makeHttpObject() {
    try {return new XMLHttpRequest();}
    catch (error) {}
    try {return new ActiveXObject("Msxml2.XMLHTTP");}
    catch (error) {}
    try {return new ActiveXObject("Microsoft.XMLHTTP");}
    catch (error) {}
    throw new Error("Could not create HTTP request object.");
}

var request = makeHttpObject();

var newSpeedL = 0;
var newSpeedR = 0;
var actSpeedL = 0;
var actSpeedR = 0;

var inactive = "0px";
var active = "10px solid black";

var showHUD = true;

function toggle_hud() {
	button_status('hud', showHUD);
	showHUD = !showHUD;
	if (showHUD) {
		//o document.getElementById("overlay").style.display = 'block';
	} else {
		//o document.getElementById("overlay").style.display = 'none';
	}
}

function button_status(button, status) {
	if (status) {
		document.getElementById(button).style.opacity = '0.5';
	} else {
		document.getElementById(button).style.opacity = '1';
	}
}

function motor_l(lspeed) {
    newSpeedL = lspeed;
    // if (lspeed == actSpeedL && lspeed != actSpeedR && doubleClick) {
	// newSpeedR = lspeed;
	// doubleClick = false;
    // } else {
	// set_doubleclick();
    // }
    set_motor();
}
function set_motor() {
    //o var motor_url = "/motor?l=" + newSpeedL.toString() + '&r=' + newSpeedR.toString();
    var motor_url = "/motor?l=" + newSpeedL.toString();
    request.open("GET", motor_url, true);
    request.send(null);
    var oldId = "l" + actSpeedL.toString();
    var newId = "l" + newSpeedL.toString();
 //n   document.getElementById(oldId).style.outline = inactive;
    document.getElementById(newId).style.outline = active;
    // oldId = "r" + actSpeedR.toString();
    // newId = "r" + newSpeedR.toString();
    // document.getElementById(oldId).style.outline = inactive;
    // document.getElementById(newId).style.outline = active;
    actSpeedL = newSpeedL;
    // actSpeedR = newSpeedR;
}
 */