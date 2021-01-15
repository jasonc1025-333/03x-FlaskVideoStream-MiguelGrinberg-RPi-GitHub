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
var httpRequest_Cl_Ob = makeHttpObject();
var getSensors = makeHttpObject();
var doubleClickTimer = 300;

//jwc o var heartbeat_TimerMax_Now_Global = 1000;
// y too long, restore o: var heartbeat_TimerMax_Now_Global = 1000000;  // 1000sec = 16.67min
// y var heartbeat_TimerMax_Now_Global = 1000;  // 1sec
// y var heartbeat_TimerMax_Now_Global = 10000;  // 10sec
// y var heartbeat_TimerMax_Now_Global = 1000000;  // 1000sec = 16.67min
// jwc improve real-time stats
// jwc var heartbeat_TimerMax_Now_Global = 3000;  // 3sec
//jwc n 2021-0109 TTFB  var heartbeat_TimerMax_Now_Global = 1000;  // 1sec
//jwc y TTFB 854ms var heartbeat_TimerMax_Now_Global = 10000;  // 10sec
// *KEY NOTE: Important to throttle timer slow enough for enough time for feedback packet to be generated and returned
// ** 2021-0109 TTFB (Time To First Byte): 900ms
//jwc y but sometimes (10%) not long enough:  var heartbeat_TimerMax_Now_Global = 2000;  // 2sec  TYJ
var heartbeat_TimerMax_Default_GLOBAL = 3000;  // 3sec  TYJ
var heartbeat_TimerMax_Now_Global = heartbeat_TimerMax_Default_GLOBAL;  // 3sec  TYJ
var heartbeat_TimerMax_IncDec_Max = 500;  // 500msec

var _ping_RoundTrip_Start_mSec_Int = 0;
var _ping_RoundTrip_End_mSec_Int = 0;
var _ping_RoundTrip_Total_mSec_Int = 0;

//jwc o var linkCheckTimer = 5000;
///jwc y too long: var linkCheckTimer = 1000000;  // 1000sec = 16.67min
///jwc y too short: var linkCheckTimer = 5000;  // 5000msec = 5sec
var linkCheckTimer = 10000;  // 10000msec = 10sec since 3sec is avg lag


var newSpeedL = 0;
var newSpeedR = 0;
var actSpeedL = 0;
var actSpeedR = 0;
var doubleClick = false;
var showHUD = true;
var video = false;
var framerate = 0;
var webServerLink_On_Bool_Global = true;

// jwc o var videoimg = '<img src="/video_feed.mjpg" alt="Connecting to Live Video">'
var videoimg = '<img style="transform:rotate(181deg);" "src="/video_feed" alt="Connecting to Live Video">'

var frameimg = '<img id="sframe" src="/single_frame.jpg" alt="Connecting to Live Video">'
var framesrc = '/single_frame.jpg'
var inactive = "0px";
var active = "10px solid black";

var clickTimer = setTimeout(reset_doubleclick, doubleClickTimer);

// jwc: every 'linkCheckTimer' (5sec), 'linkCheck' times out and calls 'webServerLink_Lost_Fn()'
var linkCheck = setTimeout(webServerLink_Lost_Fn, linkCheckTimer);

// jwc: every 'heartbeat_TimerMax_Now_Global' (1sec), 'heartbeat_Request_Fn()' called
var heartbeat_Request_SetInterval_TaskId_Global = setInterval(heartbeat_Request_Fn, heartbeat_TimerMax_Now_Global);

getSensors.onreadystatechange = updateHUD;

setTimeout(checkVideo, 500);

function webServerLink_Lost_Fn() {
	///jwc o var status = 'Status<br>Link: DOWN!';
	var status = 'WebServerLink: Lost';

    ///jwc o var sensors = 'Sensors<br>No Data!';
    var sensors = 'WebServerLink: Lost';
	document.getElementById("status").innerHTML = status;
    document.getElementById("sensors").innerHTML = sensors;
    ///jwc o ocument.getElementById("video").innerHTML = '&nbsp';
    ///jwc n 'uncaught TypeError: Cannot set property 'innerHTML' of null: document.getElementById("video").innerHTML = 'WebServerLink: Lost';
    ///jwc n '    Uncaught TypeError: Cannot set property 'innerHTML' of null: document.getElementById("video").innerHTML = '&nbsp';
    webServerLink_On_Bool_Global = false;
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
function heartbeat_Request_Fn() {
    var heartbeat_url = "/heartbeat";
    getSensors.open("GET", heartbeat_url, true);
    getSensors.send(null);
    _ping_RoundTrip_Start_mSec_Int = Date.now();    
}
function updateHUD(e) {
	var status;
    var sensors;
    var score;

    // jwc: https://www.w3schools.com/xml/xml_http.asp explains following logic:
    // https://www.w3schools.com/tags/ref_httpmessages.asp : HTTP Status Messsages
    // https://www.w3schools.com/js/js_ajax_http_response.asp    
    //    readyState: Holds the status of the XMLHttpRequest
    //       0: request not initialized
    //       1: server connection established
    //       2: request received
    //       3: processing request
    //       4: httpRequest_Cl_Ob finished and response is ready
    //    status	
    //       200: "OK"
    //       403: "Forbidden"
    //       404: "Page not found"
    //    For a complete list go to the Http Messages Reference    
	if (getSensors.readyState == 4 && getSensors.status == 200) {
        _ping_RoundTrip_End_mSec_Int = Date.now();    
        _ping_RoundTrip_Total_mSec_Int = _ping_RoundTrip_End_mSec_Int - _ping_RoundTrip_Start_mSec_Int;
        console.log('*** _ping_RoundTrip_Total_mSec_Int: ' + _ping_RoundTrip_Total_mSec_Int);

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
        sensors += "<br>Ping_RoundTrip_Total_mSec_Int: ";
        sensors += _ping_RoundTrip_Total_mSec_Int;
        sensors += "<br>Heartbeat_Freq_Mod: ";
        sensors += response.hf;

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

        $('#heartbeat_Freq_Mod_Id').html(response.hf);

        if((heartbeat_TimerMax_Default_GLOBAL + (parseInt(response.hf)*heartbeat_TimerMax_IncDec_Max)) !== heartbeat_TimerMax_Now_Global){
            heartbeat_TimerMax_Now_Global = heartbeat_TimerMax_Default_GLOBAL + (parseInt(response.hf)*heartbeat_TimerMax_IncDec_Max)
            clearTimeout( heartbeat_Request_SetInterval_TaskId_Global)
            heartbeat_Request_SetInterval_TaskId_Global = setInterval(heartbeat_Request_Fn, heartbeat_TimerMax_Now_Global);
			console.log('*** *** heartbeat_TimerMax_Now_Global:' + heartbeat_TimerMax_Now_Global + ' | heartbeat_TimerMax_Mod: ' + response.hf);
        }


        ////jwc y  $('#speed-input-val').html(response.l);
        ////jwc y  $('#heading-input-val').html(response.l);

        ////jwc y  $('#servo_Cam_01_Pan_Degrees_FrontEnd_Response_Id').html(response.s1);
        ////jwc y  $('#servo_Cam_02_Tilt_Degrees_FrontEnd_Response_Id').html(response.s2);
        ////jwc y  $('#servo_Arm_03_Degrees_FrontEnd_Response_Id').html(response.s3);

        // jwc: reset 'linkCheck' timeout since currently processing heartbeat, which is good
        //
        clearTimeout(linkCheck);
		linkCheck = setTimeout(webServerLink_Lost_Fn, linkCheckTimer);
        // **jwc: disable this, seems destructive non-helping, 
        // ** perhaps let user manually refresh/reload browser
        //jwc o n // jwc: if 'webServerLink_On_Bool_Global' false, then reload browser
		//jwc o n if (!webServerLink_On_Bool_Global) {
		//jwc o n 	location.reload();
		//jwc o n }
		if (!webServerLink_On_Bool_Global) {
			location.reload();
		}
        // jwc: since processing heartbeat, 'webServerLink_On_Bool_Global' is true
        webServerLink_On_Bool_Global = true;
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
    httpRequest_Cl_Ob.open("GET", motor_url, true);
    httpRequest_Cl_Ob.send(null);
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
    httpRequest_Cl_Ob.open("GET", touchpad_url, true);
    httpRequest_Cl_Ob.send(null);
}
function brake() {
    newSpeedR = 0;
    newSpeedL = 0;
    set_motor();
}

///jwc o seems redundant since 'setInterval': //  * Request server's ('I'm Alive') heartbeat_Request_Fn with its vital-stats
///jwc o seems redundant since 'setInterval': heartbeat_Request_Fn();
