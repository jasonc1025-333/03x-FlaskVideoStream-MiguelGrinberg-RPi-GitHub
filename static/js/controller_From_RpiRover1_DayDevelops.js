var socket, connected = false;

// jwc n
var httpRequest_Cl_Ob = makeHttpObject();

///jwc n 'setUpControllerEvents' undefined, so remove
///jwc n addEventListener('load',setUpControllerEvents);

function powerOn() {
	console.log('powering on');
	openSocket();
	controllerOn();
}

function powerOff() {
	console.log('powering down');
	closeSocket();
	controllerOff();
}

function openSocket() {
    console.log('opening socket connection');
    socket = io.connect('http://' + document.domain + ':' + location.port);

    socket.on('connect', function() {
        console.log('We are connected!')
		connected = true;
    });

	socket.on('initTrim', (data) => {
		console.log(data)
		data = JSON.parse(data);
		$('#lmtrim-val').html(data['L']);
		$('#rmtrim-val').html(data['R']);
	});

}

function closeSocket() {
	// send disconnect msg to server
	socket.emit('disconnect');
	socket.disconnect();
	console.log('We have disconnected');
	connected = false;
}


function controllerOn() {
	$('#not-connected').css('display','none');
	$('#connected').css('display','block');
}

function controllerOff() {
	$('#connected').css('display','none');
	$('#not-connected').css('display','block');
}


function sleep(milliseconds) {
	const date = Date.now();
	let currentDate = null;
	do {
	  currentDate = Date.now();
	} while (currentDate - date < milliseconds);
}

// * https://stackoverflow.com/questions/951021/what-is-the-javascript-version-of-sleep
function sleep2(ms) {
	return new Promise(resolve => setTimeout(resolve, ms));
}
  
  async function demo() {
	console.log('Taking a break...');
	await sleep2(2000);
	console.log('Two seconds later, showing sleep in a loop...');
  
	// Sleep in loop
	for (let i = 0; i < 5; i++) {
	  if (i === 3)
		await sleep2(2000);
	  console.log(i);
	}
  }
  
  function sleep3(millis)
  {
	  var date = new Date();
	  var curDate = null;
	  do { curDate = new Date(); }
	  while(curDate-date < millis);
  }
  
async function updateMotorSpeeds_Fn() {
	var url_Str = '/motor?l=' + $('#dcMotors_FwdOrRev_Power_FrontEnd_Id').val() + '&r=' + $('#dcMotors_FwdOrRev_Power_FrontEnd_Id').val();
    httpRequest_Cl_Ob.open("GET", url_Str, true);
	httpRequest_Cl_Ob.send(null);
	document.getElementById("speed-input-val").innerHTML = $('#dcMotors_FwdOrRev_Power_FrontEnd_Id').val();
	console.log('*** url: ' + url_Str)

	///jwc y sleep(3000);
	///ywc y  await sleep2(3000);
	///jwc y await sleep2(1000);
	///jwc m sleep3(3000)
	///jwc y hard refresh neededfor( let i=0; i<10000; i++){ 
	///jwc y hard refresh needed	document.getElementById("servo_Arm_03_Degrees_FrontEnd_Response_Id").innerHTML = i;
	///jwc y hard refresh needed}

	///jwc y document.getElementById("dcMotors_FwdOrRev_Power_FrontEnd_Id").value = 0;
	///jwc y url_Str = '/motor?l=' + $('#dcMotors_FwdOrRev_Power_FrontEnd_Id').val() + '&r=' + $('#dcMotors_FwdOrRev_Power_FrontEnd_Id').val();

	// If Power == 0, then resend two more times (for total of 3 times) to insure at least one packet survives transmission (not get dropped in network) for critical stop
	if ($('#dcMotors_FwdOrRev_Power_FrontEnd_Id').val()==0) {
		for (i=0; i<2; i++) { 
			httpRequest_Cl_Ob.open("GET", url_Str, true);
			httpRequest_Cl_Ob.send(null);
			console.log('*** *** ' + i + ' url: ' + url_Str);
			//jwc n  await new Promise(r => setTimeout(r, 2000));
			//y  sleep(2000);
			//n  sleep(1);
			//y seems msec, TYJ server end receives at least one loop  sleep(10);
			//y 3 loops  sleep(100);
			//y 3 TYJ seems most reliable 2-3 (90% 3) loops  sleep(50);
			//n 1-3 not reliable 3  sleep(25);
			sleep(50);
		}
	}
}
function updateMotorSpeeds_ForTurn_Fn() {
	var url_Str = '/motor_for_turn?l=' + $('#dcMotors_Turn_Power_FrontEnd_Id').val() + '&r=' + $('#dcMotors_Turn_Power_FrontEnd_Id').val();
    httpRequest_Cl_Ob.open("GET", url_Str, true);
    httpRequest_Cl_Ob.send(null);
	console.log('*** url: ' + url_Str)

	document.getElementById("heading-input-val").innerHTML = $('#dcMotors_Turn_Power_FrontEnd_Id').val();
	// If Power == 0, then resend two more times (for total of 3 times) to insure at least one packet survives transmission (not get dropped in network) for critical stop
	if ($('#dcMotors_Turn_Power_FrontEnd_Id').val()==0) {
		for (i=0; i<2; i++) { 
			httpRequest_Cl_Ob.open("GET", url_Str, true);
			httpRequest_Cl_Ob.send(null);
			console.log('*** *** ' + i + ' url: ' + url_Str);
			sleep(50);
		}
	}
}
async function updateMotorSpeeds_ForHalt_Fn() {
	var url_Str = '/motor?l=0' + '&r=0';
	httpRequest_Cl_Ob.open("GET", url_Str, true);
	httpRequest_Cl_Ob.send(null);
	console.log('*** url: ' + url_Str)

	///jwc debug  document.getElementById("servo_Arm_03_Degrees_FrontEnd_Response_Id").innerHTML = 1;
	await sleep2(500);
	///jwc debug document.getElementById("servo_Arm_03_Degrees_FrontEnd_Response_Id").innerHTML = 2;
	
	var url_Str = '/motor?l=0' + '&r=0';
	httpRequest_Cl_Ob.open("GET", url_Str, true);
	httpRequest_Cl_Ob.send(null);
	console.log('*** url: ' + url_Str)
	sleep2(50);

	// Since Power == 0, then resend two more times (for total of 3 times) to insure at least one packet survives transmission (not get dropped in network) for critical stop
	for (i=0; i<2; i++) { 
		httpRequest_Cl_Ob.open("GET", url_Str, true);
		httpRequest_Cl_Ob.send(null);
		console.log('*** *** ' + i + ' url: ' + url_Str);
		//jwc o sleep(50);
		sleep2(50);
	}
}
function touchpad2(pad) {
    var touchpad_url = "/touchpad?pad=" + pad.toString() + pad.to;
    httpRequest_Cl_Ob.open("GET", touchpad_url, true);
    httpRequest_Cl_Ob.send(null);
}

async function updateMotorSpeeds_ForBurst_Fn_OLD_AAA_TYJ() {
	var url_Str = '/motor?l=' + $('#servo_Arm_03_Degrees_FrontEnd_Id').val() + '&r=' + $('#servo_Arm_03_Degrees_FrontEnd_Id').val();
	///jwc var url_Str = "/motor?l=" + power_In.toString + '&r=100';
	httpRequest_Cl_Ob.open("GET", url_Str, true);
	httpRequest_Cl_Ob.send(null);
	console.log('*** url: ' + url_Str)

	document.getElementById("servo_Arm_03_Degrees_FrontEnd_Response_Id").innerHTML = 1;
	///jwc y await sleep2(500);
	///jwc n await sleep2($('#servo_Arm_03_Degrees_FrontEnd_Id').val()*50);
	var sleep_duration = $('#servo_Arm_03_Degrees_FrontEnd_Id').val()*50;
	await sleep2(sleep_duration);
	document.getElementById("servo_Arm_03_Degrees_FrontEnd_Response_Id").innerHTML = 2;
	
	var url_Str = '/motor?l=0' + '&r=0';
	httpRequest_Cl_Ob.open("GET", url_Str, true);
	httpRequest_Cl_Ob.send(null);
	console.log('*** url: ' + url_Str)
	///jwc n sleep2(50);
	sleep(50);

	// Since Power == 0, then resend two more times (for total of 3 times) to insure at least one packet survives transmission (not get dropped in network) for critical stop
	for (i=0; i<2; i++) { 
		httpRequest_Cl_Ob.open("GET", url_Str, true);
		httpRequest_Cl_Ob.send(null);
		console.log('*** *** ' + i + ' url: ' + url_Str);
		//jwc o sleep(50);
		//jwc n sleep2(50);
		///jwc n sleep2(100);
		///jwc n sleep2(1000);
		sleep(50);

	}
}
async function updateMotorSpeeds_ForBurst_Fn() {
	///jwc var url_Str = "/motor?l=" + power_In.toString + '&r=100';
	///jwc y var url_Str = '/motor?l=' + $('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val() + '&r=' + $('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val();
	///jwc y var url_Str = '/motor?l=100' + '&r=100';
	///jwc y var url_Str = '/motor?l=' + $('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val() + '&r=' + $('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val();
	
	if ($('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val() >= 0) {
		var url_Str = '/motor?l=100' + '&r=100';
	} else {
		var url_Str = '/motor?l=-100' + '&r=-100';
	}
	httpRequest_Cl_Ob.open("GET", url_Str, true);
	httpRequest_Cl_Ob.send(null);
	console.log('*** url: ' + url_Str)

	//jwc n document.getElementById("dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id").innerHTML = $('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val();
	// * Provide Real-Time Feedback
	document.getElementById("speed-input-val").innerHTML = $('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val();

	///jwc y await sleep2(500);
	///jwc n await sleep2($('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val()*50);
	///jwc ? var sleep_duration = Math.abs($('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val()) * 50;
	///jwc y  var sleep_duration = Math.abs($('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val()) * 1;
	var sleep_duration = Math.abs($('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val()) * (1 + parseInt($('#lmtrim-val').text()));
	// * Clear input value
	$('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val(0);
	console.log('*** updateMotorSpeeds_ForBurst_Fn: lmtrim-val: ' +$('#lmtrim-val').text()+ ' sleep_duration: ' + sleep_duration);

	await sleep2(sleep_duration);
	
	var url_Str = '/motor?l=0' + '&r=0';
	httpRequest_Cl_Ob.open("GET", url_Str, true);
	httpRequest_Cl_Ob.send(null);
	console.log('*** url: ' + url_Str)
	///jwc n sleep2(50);
	sleep(50);

	// Since Power == 0, then resend two more times (for total of 3 times) to insure at least one packet survives transmission (not get dropped in network) for critical stop
	for (i=0; i<2; i++) { 
		httpRequest_Cl_Ob.open("GET", url_Str, true);
		httpRequest_Cl_Ob.send(null);
		console.log('*** *** ' + i + ' url: ' + url_Str);
		//jwc o sleep(50);
		//jwc n sleep2(50);
		///jwc n sleep2(100);
		///jwc n sleep2(1000);
		sleep(50);
	}
}
async function updateMotorSpeeds_ForTurn_ForBurst_Fn() {
	//jwc o  var url_Str = '/motor_for_turn?l=' + $('#dcMotors_Turn_Power_FrontEnd_Id').val() + '&r=' + $('#dcMotors_Turn_Power_FrontEnd_Id').val();

	if ($('#dcMotors_Turn_Power_FrontEnd_SliderTapForIpadFix_Id').val() >= 0) {
		var url_Str = '/motor_for_turn?l=100' + '&r=100';
	} else {
		var url_Str = '/motor_for_turn?l=-100' + '&r=-100';
	}
	httpRequest_Cl_Ob.open("GET", url_Str, true);
    httpRequest_Cl_Ob.send(null);
	console.log('*** url: ' + url_Str)

	// * Provide Real-Time Feedback
	document.getElementById("heading-input-val").innerHTML = $('#dcMotors_Turn_Power_FrontEnd_SliderTapForIpadFix_Id').val();

	var sleep_duration = Math.abs($('#dcMotors_Turn_Power_FrontEnd_SliderTapForIpadFix_Id').val()) * (1 + parseInt($('#lmtrim-val').text()));
	// * Clear input value
	$('#dcMotors_Turn_Power_FrontEnd_SliderTapForIpadFix_Id').val(0);
	console.log('*** updateMotorSpeeds_ForTurn_ForBurst_Fn: lmtrim-val: ' +$('#lmtrim-val').text()+ ' sleep_duration: ' + sleep_duration);

	await sleep2(sleep_duration);

	var url_Str = '/motor?l=0' + '&r=0';
	httpRequest_Cl_Ob.open("GET", url_Str, true);
	httpRequest_Cl_Ob.send(null);
	console.log('*** url: ' + url_Str)
	///jwc n sleep2(50);
	sleep(50);

	// Since Power == 0, then resend two more times (for total of 3 times) to insure at least one packet survives transmission (not get dropped in network) for critical stop
	for (i=0; i<2; i++) { 
		httpRequest_Cl_Ob.open("GET", url_Str, true);
		httpRequest_Cl_Ob.send(null);
		console.log('*** *** ' + i + ' url: ' + url_Str);
		//jwc o sleep(50);
		//jwc n sleep2(50);
		///jwc n sleep2(100);
		///jwc n sleep2(1000);
		sleep(50);
	}
}

function servo_Cam_01_Pan_Degrees_FrontEnd_Fn() {
	var url_Str = '/servo_Cam_01_Pan_Degrees_FrontEnd_Fn?servo_Cam_01_Pan_Degrees_FrontEnd_Id=' + $('#servo_Cam_01_Pan_Degrees_FrontEnd_Id').val();
    httpRequest_Cl_Ob.open('GET', url_Str, true);
    httpRequest_Cl_Ob.send(null);
	document.getElementById("servo_Cam_01_Pan_Degrees_FrontEnd_Response_Id").innerHTML = $('#servo_Cam_01_Pan_Degrees_FrontEnd_Id').val();
	console.log('*** url: ' + url_Str)
}

function servo_Cam_02_Tilt_Degrees_FrontEnd_Fn() {
	var url_Str = '/servo_Cam_02_Tilt_Degrees_FrontEnd_Fn?servo_Cam_02_Tilt_Degrees_FrontEnd_Id=' + $('#servo_Cam_02_Tilt_Degrees_FrontEnd_Id').val();
    httpRequest_Cl_Ob.open('GET', url_Str, true);
    httpRequest_Cl_Ob.send(null);
	document.getElementById("servo_Cam_02_Tilt_Degrees_FrontEnd_Response_Id").innerHTML = $('#servo_Cam_02_Tilt_Degrees_FrontEnd_Id').val();
	console.log('*** url: ' + url_Str)
}

function servo_Arm_03_Degrees_FrontEnd_Fn() {
	var url_Str = '/servo_Arm_03_Degrees_FrontEnd_Fn?servo_Arm_03_Degrees_FrontEnd_Id=' + $('#servo_Arm_03_Degrees_FrontEnd_Id').val();
    httpRequest_Cl_Ob.open('GET', url_Str, true);
    httpRequest_Cl_Ob.send(null);
	document.getElementById("servo_Arm_03_Degrees_FrontEnd_Response_Id").innerHTML = $('#servo_Arm_03_Degrees_FrontEnd_Id').val();
	console.log('*** url: ' + url_Str)
}

function changeTrim(data) {
	///jwc o socket.emit('updateTrim',{'L':data.L, 'R':data.R});
	var motorTrim_url = "/motorTrim?l=" + data.L + '&r=' + data.R;
    httpRequest_Cl_Ob.open("GET", motorTrim_url, true);
    httpRequest_Cl_Ob.send(null);	
	console.log('### DEBUG: asking server to update motorTrim');
}

function heartbeat_Freq_Mod_IncDec_Fn(data) {
	///jwc o socket.emit('updateTrim',{'L':data.L, 'R':data.R});
	var url_Str = "/heartbeat_Freq_Mod_IncDec_Fn?incdec=" + data.incdec;
	httpRequest_Cl_Ob.open("GET", url_Str, true);
	httpRequest_Cl_Ob.send(null);		
	console.log('### DEBUG: heartbeat_Freq_Mod_IncDec_Fn');
}

function turnCam(data) {
	// send change in camera heading left
	console.log(JSON.stringify(data,undefined,4));
}
