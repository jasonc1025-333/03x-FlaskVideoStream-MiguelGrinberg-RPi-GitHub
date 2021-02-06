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

//jwc o var _serverHeartbeat_Req_Interval_Total_Msec = 1000;
// y too long, restore o: var _serverHeartbeat_Req_Interval_Total_Msec = 1000000;  // 1000sec = 16.67min
// y var _serverHeartbeat_Req_Interval_Total_Msec = 1000;  // 1sec
// y var _serverHeartbeat_Req_Interval_Total_Msec = 10000;  // 10sec
// y var _serverHeartbeat_Req_Interval_Total_Msec = 1000000;  // 1000sec = 16.67min
// jwc improve real-time stats
// jwc var _serverHeartbeat_Req_Interval_Total_Msec = 3000;  // 3sec
//jwc n 2021-0109 TTFB  var _serverHeartbeat_Req_Interval_Total_Msec = 1000;  // 1sec
//jwc y TTFB 854ms var _serverHeartbeat_Req_Interval_Total_Msec = 10000;  // 10sec
// *KEY NOTE: Important to throttle timer slow enough for enough time for feedback packet to be generated and returned
// ** 2021-0109 TTFB (Time To First Byte): 900ms
//jwc y but sometimes (10%) not long enough:  var _serverHeartbeat_Req_Interval_Total_Msec = 2000;  // 2sec  TYJ
var _serverHeartbeat_Req_Interval_BASE_MSEC = 3000;  // 3sec  TYJ
var _serverHeartbeat_Req_Interval_Total_Msec = _serverHeartbeat_Req_Interval_BASE_MSEC;  // 3sec  TYJ
var _serverHeartbeat_Req_Interval_INCDECMAX_MSEC = 500;  // 500msec

var _ping_RoundTrip_Start_mSec_Int = 0;
var _ping_RoundTrip_End_mSec_Int = 0;
var _ping_RoundTrip_Total_Msec_Int = 0;

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
var activate_Ipad_Tap_Bool = false;

// Default to false
// Need for iPad since '.button:active' not work
//
var trigger_Client_01_Opacity_Bool = false;
var trigger_Client_02_Opacity_Bool = false;
var trigger_Client_03_Opacity_Bool = false;

var timer_Mission_Refresh_Opacity_Bool = false;

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

// jwc: every '_serverHeartbeat_Req_Interval_Total_Msec' (1sec), 'heartbeat_Request_Fn()' called
var heartbeat_Request_SetInterval_TaskId_Global = setInterval(heartbeat_Request_Fn, _serverHeartbeat_Req_Interval_Total_Msec);

var servo_Cam_01_Pan_ControlsUsed_Bool = false;
var servo_Cam_02_Tilt_ControlsUsed_Bool = false;
var servo_Arm_03_ControlsUsed_Bool = false;

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
        _ping_RoundTrip_Total_Msec_Int = _ping_RoundTrip_End_mSec_Int - _ping_RoundTrip_Start_mSec_Int;
        console.log('*** _ping_RoundTrip_Total_Msec_Int: ' + _ping_RoundTrip_Total_Msec_Int);

        var response = JSON.parse(getSensors.responseText);
        video = response.v;
        framerate = response.f;
        
        // jwc multiple-spacing gets reduced to one-space, so use '.' for column justification
        // jwc o status = "STS-PiLot Status<br>Link: online<br>Videolink: ";
        // status  = "*** HUD: Primary ***";
        status  = "";

        status += "<br>_ping_RoundTrip_Total_Msec_Int: ";
        status += _ping_RoundTrip_Total_Msec_Int;

        status += "<br>";
        status += "<br>_serverHeartbeat_Req_Interval_BASE_MSEC: ";
        status += _serverHeartbeat_Req_Interval_BASE_MSEC;
        status += "<br>_serverHeartbeat_Req_Interval_INCDECMAX_MSEC: ";
        status += _serverHeartbeat_Req_Interval_INCDECMAX_MSEC;
        status += "<br>_serverHeartbeat_Req_Interval_IncDecMax_Factor_Msec: ";
        status += response.hf;
        status += "<br>";
        status += "<br>* Insure following > Network:Time (RTT) *";
        status += "<br>_serverHeartbeat_Req_Interval_Total_Msec: ";
        status += _serverHeartbeat_Req_Interval_Total_Msec;

        status += "<br>";
        status += "<br>Web-Link .: online";    
        status += "<br>Video-Link: ";
        if (framerate > 0) {
		    status += (Math.ceil(1000 / framerate) + " FPS");
	    } else {
        	status += video;
        }

        
        // jwc o sensors = "*** Sensors ***<br>Digital 1: ";
        // sensors = "*** HUD: Secondary ***<br>Digital 1: ";
        sensors  = "";
        sensors += "<br>_batteryUps_Input_V ..(3.8 min): ";
        sensors += response.bvi;
        sensors += "<br>_batteryUps_Output_V .(5.0 avg): ";
        sensors += response.bvo;
        sensors += "<br>_batteryUps_Battery_V (3.1-4.3): ";
        sensors += response.bvb;
        
        sensors += "<br>";
        sensors += "<br>Motor-L: ";
        sensors += response.l;
        sensors += "<br>Motor-R: ";
        sensors += response.r;

        sensors += "<br>Cam_Servo_02_Tilt_Degrees: ";
        sensors += response.s2;
        sensors += "<br>Cam_Servo_01_Pan_Degrees : ";
        sensors += response.s1;

        sensors += "<br>Arm_Servo_03_Degrees: ";
        sensors += response.s3;

        sensors += "<br>";
        sensors += "<br>_batteryUps_Temp_C ( 60max): ";
        sensors += response.btc;
        sensors += "<br>_batteryUps_Temp_F (140max): ";
        sensors += response.btf;


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
        score += "<br>_timer_Mission_Countdown_Sec: ";
        score += response.tmc;
        score += "<br>_timer_Mission_Expired_Bool: ";
        score += response.tme;
        score += "<br>_timer_Mission_Reserves_Sec_Int: ";
        score += response.tmr;
        score += "<br>";
        score += "<br>_timer_Mission_Recharge_Sec_Int: ";
        score += response.tmres;
        score += "<br>_timer_Mission_Recharge_THRESHOLD_DEC: ";
        score += response.tmreth;        
        score += "<br>_timer_Mission_Now_Sec: ";
        score += response.tmn;
        score += "<br>_timer_Mission_Recharge_Timestamp_Int: ";
        score += response.tmreti;
        
        score += "<br>";
        score += "<br>score_Targeted_Dict: ";
        score += response.sc;
        score += "<br>score_Targeted_WeightedToVideoCenter_Dict: ";
        score += response.sctw;
        score += "<br>";
        score += "<br>score_Targeted_WeightedToVideoCenter_TriggerClient_01_Dict: ";
        score += response.sctwtc1;
        score += "<br>score_Targeted_WeightedToVideoCenter_TriggerClient_02_Dict: ";
        score += response.sctwtc2;
        score += "<br>score_Targeted_WeightedToVideoCenter_TriggerClient_03_Dict: ";
        score += response.sctwtc3;

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
        if((_serverHeartbeat_Req_Interval_BASE_MSEC + (parseInt(response.hf)*_serverHeartbeat_Req_Interval_INCDECMAX_MSEC)) !== _serverHeartbeat_Req_Interval_Total_Msec){
            _serverHeartbeat_Req_Interval_Total_Msec = _serverHeartbeat_Req_Interval_BASE_MSEC + (parseInt(response.hf)*_serverHeartbeat_Req_Interval_INCDECMAX_MSEC)
            clearTimeout( heartbeat_Request_SetInterval_TaskId_Global)
            heartbeat_Request_SetInterval_TaskId_Global = setInterval(heartbeat_Request_Fn, _serverHeartbeat_Req_Interval_Total_Msec);
			console.log('*** *** _serverHeartbeat_Req_Interval_Total_Msec:' + _serverHeartbeat_Req_Interval_Total_Msec + ' | heartbeat_TimerMax_Mod: ' + response.hf);
        }

        // Provide auto-feedback for any remote-player's activity on any controls (not used by local-player)
        if(!servo_Cam_01_Pan_ControlsUsed_Bool){
            if(response.s1 !== parseInt($('#servo_Cam_01_Pan_Degrees_FrontEnd_Id').val())){
                $('#servo_Cam_01_Pan_Degrees_FrontEnd_Id').val(parseInt(response.s1));
                $('#servo_Cam_01_Pan_Degrees_FrontEnd_Response_Id').html(response.s1);
                ///jwc n console.log('*** *** !servo_Cam_01_Pan_ControlsUsed_Bool: ' + response.s1 + '>>' + $('#servo_Cam_01_Pan_Degrees_FrontEnd_Id').val());
                ///jwc y console.log('*** *** !servo_Cam_01_Pan_ControlsUsed_Bool: ' + parseInt(response.s1) + '>>' + $('#servo_Cam_01_Pan_Degrees_FrontEnd_Id').val());
                console.log('*** *** !servo_Cam_01_Pan_ControlsUsed_Bool: ' + response.s1 + '>>' + $('#servo_Cam_01_Pan_Degrees_FrontEnd_Id').val());
            }
        }

        if(!servo_Cam_02_Tilt_ControlsUsed_Bool){
            if(response.s2 !== parseInt($('#servo_Cam_02_Tilt_Degrees_FrontEnd_Id').val())){
                $('#servo_Cam_02_Tilt_Degrees_FrontEnd_Id').val(parseInt(response.s2));
                $('#servo_Cam_02_Tilt_Degrees_FrontEnd_Response_Id').html(response.s2);
                console.log('*** *** !servo_Cam_02_Tilt_ControlsUsed_Bool: ' + response.s2 + '>>' + $('#servo_Cam_02_Tilt_Degrees_FrontEnd_Id').val());
            }
        }

        if(!servo_Arm_03_ControlsUsed_Bool){
            if(response.s3 !== parseInt($('#servo_Arm_03_Degrees_FrontEnd_Id').val())){
                $('#servo_Arm_03_Degrees_FrontEnd_Id').val(parseInt(response.s3));
                $('#servo_Arm_03_Degrees_FrontEnd_Response_Id').html(response.s3);
                console.log('*** *** !servo_Arm_03_ControlsUsed_Bool: ' + response.s3 + '>>' + $('#servo_Arm_03_Degrees_FrontEnd_Id').val());
            }
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
		// document.getElementById(button).style.opacity = '0.5';
		document.getElementById(button).style.opacity = '0.75';
	} else {
		document.getElementById(button).style.opacity = '1';
	}
}

function activate_Ipad_Tap_Fn() {
	activate_Ipad_Tap_Bool = !activate_Ipad_Tap_Bool;
	activate_Ipad_Tap_Button_Fn('activate_Ipad_Tap_Id', activate_Ipad_Tap_Bool);
	if (activate_Ipad_Tap_Bool) {
        // document.getElementById("overlay").style.display = 'block';
        $('.sliderTapForIpadFix_Not_CssCl').removeClass('visible_CssCl')
        $('.sliderTapForIpadFix_Not_CssCl').addClass('visible_Not_CssCl')

        $('.sliderTapForIpadFix_CssCl').removeClass('visible_Not_CssCl')
        $('.sliderTapForIpadFix_CssCl').addClass('visible_CssCl')
	} else {
        // document.getElementById("overlay").style.display = 'none';
        $('.sliderTapForIpadFix_Not_CssCl').removeClass('visible_Not_CssCl')
        $('.sliderTapForIpadFix_Not_CssCl').addClass('visible_CssCl')

        $('.sliderTapForIpadFix_CssCl').removeClass('visible_CssCl')
        $('.sliderTapForIpadFix_CssCl').addClass('visible_Not_CssCl')
    }
}
function activate_Ipad_Tap_Button_Fn(button, status) {
	if (status) {
		//jwc o  document.getElementById(button).style.opacity = '0.5';
		document.getElementById(button).style.opacity = '0.25';
	} else {
		document.getElementById(button).style.opacity = '1';
	}
}

function timer_Mission_Refresh_Fn(pad) {
    servo_Cam_01_Pan_ControlsUsed_Bool = false;
    servo_Cam_02_Tilt_ControlsUsed_Bool = false;
    servo_Arm_03_ControlsUsed_Bool = false;
    
    timer_Mission_Refresh_Opacity_Bool = !timer_Mission_Refresh_Opacity_Bool;
    button_status('timer_Mission_Refresh_Id', timer_Mission_Refresh_Opacity_Bool)

    var url_Str = "/timer_Mission_Refresh_Fn";
    httpRequest_Cl_Ob.open("GET", url_Str, true);
    httpRequest_Cl_Ob.send(null);
}
function trigger_Client_01_Fn(pad) {    
    trigger_Client_01_Opacity_Bool = !trigger_Client_01_Opacity_Bool;
    button_status('trigger_Client_01_Id', trigger_Client_01_Opacity_Bool)

    var url_Str = "/trigger_Client_01_Fn";
    httpRequest_Cl_Ob.open("GET", url_Str, true);
    httpRequest_Cl_Ob.send(null);
}
function trigger_Client_02_Fn(pad) {
    trigger_Client_02_Opacity_Bool = !trigger_Client_02_Opacity_Bool;
    button_status('trigger_Client_02_Id', trigger_Client_02_Opacity_Bool)

    var url_Str = "/trigger_Client_02_Fn";
    httpRequest_Cl_Ob.open("GET", url_Str, true);
    httpRequest_Cl_Ob.send(null);
}
function trigger_Client_03_Fn(pad) {
    trigger_Client_03_Opacity_Bool = !trigger_Client_03_Opacity_Bool;
    button_status('trigger_Client_03_Id', trigger_Client_03_Opacity_Bool)

    var url_Str = "/trigger_Client_03_Fn";
    httpRequest_Cl_Ob.open("GET", url_Str, true);
    httpRequest_Cl_Ob.send(null);
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
