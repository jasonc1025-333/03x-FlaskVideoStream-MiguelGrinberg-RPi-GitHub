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
		$('#networkThrottleDelay_FactorOfOneCpuCycle_20msec-val').html(data['L']);
		$('#unusedUiVal-val').html(data['R']);
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
  
async function updateMotorSpeeds_ForStraight_Fn() {
	// Network Throttle Delay achieved by waiting on this semaphore UI/UX variable to resume inactive state (0 value) 
	if (document.getElementById("speed-input-val").innerHTML == 0){
		var url_Str = '/motor_for_straight?l=' + $('#dcMotors_FwdOrRev_Power_FrontEnd_Id').val() + '&r=' + $('#dcMotors_FwdOrRev_Power_FrontEnd_Id').val();
    	httpRequest_Cl_Ob.open("GET", url_Str, true);
		httpRequest_Cl_Ob.send(null);
		document.getElementById("speed-input-val").innerHTML = $('#dcMotors_FwdOrRev_Power_FrontEnd_Id').val();
		console.log('*** url: ' + url_Str)

		///jwc y await sleep2(500);
		///jwc n await sleep2($('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val()*50);
		///jwc ? var sleep_duration_msec = Math.abs($('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val()) * 50;
		///jwc y  var sleep_duration_msec = Math.abs($('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val()) * 1;
		///jwc yy var sleep_duration_msec = Math.abs($('#dcMotors_FwdOrRev_Power_FrontEnd_Id').val()) * (1 + parseInt($('#networkThrottleDelay_FactorOfOneCpuCycle_20msec-val').text()));
		var sleep_duration_msec = 20 * (parseInt($('#networkThrottleDelay_FactorOfOneCpuCycle_20msec-val').text()));
		/// jwc o await sleep2(sleep_duration_msec);
		/// jwc o await sleep2(50);
		// Significant delay to see screen updates
		///jwc y, just right: try 'sleep_duration_msec' since real-time adjustable: await sleep2(100);
		await sleep2(sleep_duration_msec);

		// * Clear input value
		$('#dcMotors_FwdOrRev_Power_FrontEnd_Id').val(0);
		document.getElementById("speed-input-val").innerHTML = 0;
		console.log('*** dcMotors_FwdOrRev_Power_FrontEnd_Id: networkThrottleDelay_FactorOfOneCpuCycle_20msec-val: ' +$('#networkThrottleDelay_FactorOfOneCpuCycle_20msec-val').text()+ ' sleep_duration_msec: ' + sleep_duration_msec);


		///jwc y sleep(3000);
		///ywc y  await sleep2(3000);
		///jwc y await sleep2(1000);
		///jwc m sleep3(3000)
		///jwc y hard refresh neededfor( let i=0; i<10000; i++){ 
		///jwc y hard refresh needed	document.getElementById("servo_Arm_03_Degrees_FrontEnd_Response_Id").innerHTML = i;
		///jwc y hard refresh needed}

		///jwc y document.getElementById("dcMotors_FwdOrRev_Power_FrontEnd_Id").value = 0;
		///jwc y url_Str = '/motor_for_straight?l=' + $('#dcMotors_FwdOrRev_Power_FrontEnd_Id').val() + '&r=' + $('#dcMotors_FwdOrRev_Power_FrontEnd_Id').val();

		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: // If Power == 0, then resend two more times (for total of 3 times) to insure at least one packet survives transmission (not get dropped in network) for critical stop
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: if ($('#dcMotors_FwdOrRev_Power_FrontEnd_Id').val()==0) {
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: 	for (i=0; i<2; i++) { 
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: 		httpRequest_Cl_Ob.open("GET", url_Str, true);
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: 		httpRequest_Cl_Ob.send(null);
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: 		console.log('*** *** ' + i + ' url: ' + url_Str);
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: 		//jwc n  await new Promise(r => setTimeout(r, 2000));
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: 		//y  sleep(2000);
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: 		//n  sleep(1);
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: 		//y seems msec, TYJ server end receives at least one loop  sleep(10);
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: 		//y 3 loops  sleep(100);
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: 		//y 3 TYJ seems most reliable 2-3 (90% 3) loops  sleep(50);
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: 		//n 1-3 not reliable 3  sleep(25);
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: 		sleep(50);
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: 	}
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: }
	}
}
async function updateMotorSpeeds_ForTurn_Fn() {
	// Network Throttle Delay achieved by waiting on this semaphore UI/UX variable to resume inactive state (0 value) 
	if (document.getElementById("heading-input-val").innerHTML == 0){
		var url_Str = '/motor_for_turn?l=' + $('#dcMotors_Turn_Power_FrontEnd_Id').val() + '&r=' + $('#dcMotors_Turn_Power_FrontEnd_Id').val();
    	httpRequest_Cl_Ob.open("GET", url_Str, true);
    	httpRequest_Cl_Ob.send(null);
		document.getElementById("heading-input-val").innerHTML = $('#dcMotors_Turn_Power_FrontEnd_Id').val();
		console.log('*** url: ' + url_Str)

		
		///jwc y await sleep2(500);
		///jwc n await sleep2($('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val()*50);
		///jwc ? var sleep_duration_msec = Math.abs($('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val()) * 50;
		///jwc y  var sleep_duration_msec = Math.abs($('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val()) * 1;
		///jwc yy var sleep_duration_msec = Math.abs($('#dcMotors_Turn_Power_FrontEnd_Id').val()) * (1 + parseInt($('#networkThrottleDelay_FactorOfOneCpuCycle_20msec-val').text()));
		var sleep_duration_msec = 20 * (parseInt($('#networkThrottleDelay_FactorOfOneCpuCycle_20msec-val').text()));
		/// jwc o await sleep2(sleep_duration_msec);
		/// jwc o await sleep2(50);
		// Significant delay to see screen updates
		///jwc y, just right: try 'sleep_duration_msec' since real-time adjustable: await sleep2(100);
		await sleep2(sleep_duration_msec);

		// * Clear input value
		$('#dcMotors_Turn_Power_FrontEnd_Id').val(0);
		document.getElementById("heading-input-val").innerHTML = 0;
		console.log('*** dcMotors_Turn_Power_FrontEnd_Id: networkThrottleDelay_FactorOfOneCpuCycle_20msec-val: ' +$('#networkThrottleDelay_FactorOfOneCpuCycle_20msec-val').text()+ ' sleep_duration_msec: ' + sleep_duration_msec);


		// If Power == 0, then resend two more times (for total of 3 times) to insure at least one packet survives transmission (not get dropped in network) for critical stop
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: if ($('#dcMotors_Turn_Power_FrontEnd_Id').val()==0) {
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: 	for (i=0; i<2; i++) { 
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: 		httpRequest_Cl_Ob.open("GET", url_Str, true);
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: 		httpRequest_Cl_Ob.send(null);
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: 		console.log('*** *** ' + i + ' url: ' + url_Str);
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: 		sleep(50);
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: 	}
		///jwc y try remove 'safety stops' and hardcode them on server-bot side for faster/reliable: }
	}
}

async function updateMotorSpeeds_ForStraight_ForIpad_Fn() {
	// Network Throttle Delay achieved by waiting on this semaphore UI/UX variable to resume inactive state (0 value) 
	if (document.getElementById("speed-input-val").innerHTML == 0){
		///jwc var url_Str = "/motor_for_straight?l=" + power_In.toString + '&r=100';
		///jwc y var url_Str = '/motor_for_straight?l=' + $('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val() + '&r=' + $('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val();
		///jwc y var url_Str = '/motor_for_straight?l=100' + '&r=100';
		///jwc y var url_Str = '/motor_for_straight?l=' + $('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val() + '&r=' + $('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val();
		
		if ($('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val() >= 0) {
			var url_Str = '/motor_for_straight?l=100' + '&r=100';
		} else {
			var url_Str = '/motor_for_straight?l=-100' + '&r=-100';
		}
		httpRequest_Cl_Ob.open("GET", url_Str, true);
		httpRequest_Cl_Ob.send(null);
		console.log('*** url: ' + url_Str)

		//jwc n document.getElementById("dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id").innerHTML = $('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val();
		// * Provide Real-Time Feedback
		document.getElementById("speed-input-val").innerHTML = $('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val();

		///jwc y await sleep2(500);
		///jwc n await sleep2($('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val()*50);
		///jwc ? var sleep_duration_msec = Math.abs($('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val()) * 50;
		///jwc y  var sleep_duration_msec = Math.abs($('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val()) * 1;
		///jwc yy var sleep_duration_msec = Math.abs($('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val()) * (1 + parseInt($('#networkThrottleDelay_FactorOfOneCpuCycle_20msec-val').text()));
		var sleep_duration_msec = 20 * (parseInt($('#networkThrottleDelay_FactorOfOneCpuCycle_20msec-val').text()));
		await sleep2(sleep_duration_msec);

		// * Clear input value
		$('#dcMotors_FwdOrRev_Power_FrontEnd_SliderTapForIpadFix_Id').val(0);
		document.getElementById("speed-input-val").innerHTML = 0;
		console.log('*** updateMotorSpeeds_ForStraight_ForIpad_Fn: networkThrottleDelay_FactorOfOneCpuCycle_20msec-val: ' +$('#networkThrottleDelay_FactorOfOneCpuCycle_20msec-val').text()+ ' sleep_duration_msec: ' + sleep_duration_msec);

		///jwc not needed anymore: var url_Str = '/motor_for_straight?l=0' + '&r=0';
		///jwc not needed anymore: httpRequest_Cl_Ob.open("GET", url_Str, true);
		///jwc not needed anymore: httpRequest_Cl_Ob.send(null);
		///jwc not needed anymore: console.log('*** url: ' + url_Str)
		///jwc not needed anymore: ///jwc n sleep2(50);
		///jwc not needed anymore: sleep(50);
		///jwc not needed anymore: 
		///jwc not needed anymore: // Since Power == 0, then resend two more times (for total of 3 times) to insure at least one packet survives transmission (not get dropped in network) for critical stop
		///jwc not needed anymore: for (i=0; i<2; i++) { 
		///jwc not needed anymore: 	httpRequest_Cl_Ob.open("GET", url_Str, true);
		///jwc not needed anymore: 	httpRequest_Cl_Ob.send(null);
		///jwc not needed anymore: 	console.log('*** *** ' + i + ' url: ' + url_Str);
		///jwc not needed anymore: 	//jwc o sleep(50);
		///jwc not needed anymore: 	//jwc n sleep2(50);
		///jwc not needed anymore: 	///jwc n sleep2(100);
		///jwc not needed anymore: 	///jwc n sleep2(1000);
		///jwc not needed anymore: 	sleep(50);
		///jwc not needed anymore: }
	}
}
async function updateMotorSpeeds_ForTurn_ForIpad_Fn() {
	// Network Throttle Delay achieved by waiting on this semaphore UI/UX variable to resume inactive state (0 value) 
	if (document.getElementById("heading-input-val").innerHTML == 0){
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

		///jwc yy var sleep_duration_msec = Math.abs($('#dcMotors_Turn_Power_FrontEnd_SliderTapForIpadFix_Id').val()) * (1 + parseInt($('#networkThrottleDelay_FactorOfOneCpuCycle_20msec-val').text()));
		var sleep_duration_msec = 20 * (parseInt($('#networkThrottleDelay_FactorOfOneCpuCycle_20msec-val').text()));
		await sleep2(sleep_duration_msec);

		// * Clear input value
		$('#dcMotors_Turn_Power_FrontEnd_SliderTapForIpadFix_Id').val(0);
		document.getElementById("heading-input-val").innerHTML = 0;
		console.log('*** updateMotorSpeeds_ForTurn_ForIpad_Fn: networkThrottleDelay_FactorOfOneCpuCycle_20msec-val: ' +$('#networkThrottleDelay_FactorOfOneCpuCycle_20msec-val').text()+ ' sleep_duration_msec: ' + sleep_duration_msec);

		///jwc not needed anymore: var url_Str = '/motor_for_straight?l=0' + '&r=0';
		///jwc not needed anymore: httpRequest_Cl_Ob.open("GET", url_Str, true);
		///jwc not needed anymore: httpRequest_Cl_Ob.send(null);
		///jwc not needed anymore: console.log('*** url: ' + url_Str)
		///jwc not needed anymore: ///jwc n sleep2(50);
		///jwc not needed anymore: sleep(50);
		///jwc not needed anymore: 
		///jwc not needed anymore: // Since Power == 0, then resend two more times (for total of 3 times) to insure at least one packet survives transmission (not get dropped in network) for critical stop
		///jwc not needed anymore: for (i=0; i<2; i++) { 
		///jwc not needed anymore: 	httpRequest_Cl_Ob.open("GET", url_Str, true);
		///jwc not needed anymore: 	httpRequest_Cl_Ob.send(null);
		///jwc not needed anymore: 	console.log('*** *** ' + i + ' url: ' + url_Str);
		///jwc not needed anymore: 	//jwc o sleep(50);
		///jwc not needed anymore: 	//jwc n sleep2(50);
		///jwc not needed anymore: 	///jwc n sleep2(100);
		///jwc not needed anymore: 	///jwc n sleep2(1000);
		///jwc not needed anymore: 	sleep(50);
		///jwc not needed anymore: }
	}
}

// Apears Obsolete
//
function touchpad2(pad) {
    var touchpad_url = "/touchpad?pad=" + pad.toString() + pad.to;
    httpRequest_Cl_Ob.open("GET", touchpad_url, true);
    httpRequest_Cl_Ob.send(null);
}
// Apears Obsolete
//
async function updateMotorSpeeds_ForHalt_Fn() {
	var url_Str = '/motor_for_straight?l=0' + '&r=0';
	httpRequest_Cl_Ob.open("GET", url_Str, true);
	httpRequest_Cl_Ob.send(null);
	console.log('*** url: ' + url_Str)

	///jwc yy ///jwc debug  document.getElementById("servo_Arm_03_Degrees_FrontEnd_Response_Id").innerHTML = 1;
	///jwc yy await sleep2(500);
	///jwc yy ///jwc debug document.getElementById("servo_Arm_03_Degrees_FrontEnd_Response_Id").innerHTML = 2;
	var sleep_duration_msec = 20 * (parseInt($('#networkThrottleDelay_FactorOfOneCpuCycle_20msec-val').text()));
	await sleep2(sleep_duration_msec);
	
	var url_Str = '/motor_for_straight?l=0' + '&r=0';
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
// Apears Obsolete
//
async function updateMotorSpeeds_ForBurst_Fn_OLD_AAA_TYJ() {
	var url_Str = '/motor_for_straight?l=' + $('#servo_Arm_03_Degrees_FrontEnd_Id').val() + '&r=' + $('#servo_Arm_03_Degrees_FrontEnd_Id').val();
	///jwc var url_Str = "/motor_for_straight?l=" + power_In.toString + '&r=100';
	httpRequest_Cl_Ob.open("GET", url_Str, true);
	httpRequest_Cl_Ob.send(null);
	console.log('*** url: ' + url_Str)

	document.getElementById("servo_Arm_03_Degrees_FrontEnd_Response_Id").innerHTML = 1;
	///jwc y await sleep2(500);
	///jwc n await sleep2($('#servo_Arm_03_Degrees_FrontEnd_Id').val()*50);
	var sleep_duration_msec = $('#servo_Arm_03_Degrees_FrontEnd_Id').val()*50;
	await sleep2(sleep_duration_msec);
	document.getElementById("servo_Arm_03_Degrees_FrontEnd_Response_Id").innerHTML = 2;
	
	var url_Str = '/motor_for_straight?l=0' + '&r=0';
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
	if(!servo_Cam_01_Pan_ControlsUsed_Bool){servo_Cam_01_Pan_ControlsUsed_Bool = true;}
	httpRequest_Cl_Ob.open('GET', url_Str, true);
    httpRequest_Cl_Ob.send(null);
	document.getElementById("servo_Cam_01_Pan_Degrees_FrontEnd_Response_Id").innerHTML = $('#servo_Cam_01_Pan_Degrees_FrontEnd_Id').val();
	console.log('*** url: ' + url_Str)
}

function servo_Cam_02_Tilt_Degrees_FrontEnd_Fn() {
	var url_Str = '/servo_Cam_02_Tilt_Degrees_FrontEnd_Fn?servo_Cam_02_Tilt_Degrees_FrontEnd_Id=' + $('#servo_Cam_02_Tilt_Degrees_FrontEnd_Id').val();
	if(!servo_Cam_02_Tilt_ControlsUsed_Bool){servo_Cam_02_Tilt_ControlsUsed_Bool = true;}
    httpRequest_Cl_Ob.open('GET', url_Str, true);
    httpRequest_Cl_Ob.send(null);
	document.getElementById("servo_Cam_02_Tilt_Degrees_FrontEnd_Response_Id").innerHTML = $('#servo_Cam_02_Tilt_Degrees_FrontEnd_Id').val();
	console.log('*** url: ' + url_Str)
}

function servo_Arm_03_Degrees_FrontEnd_Fn() {
	var url_Str = '/servo_Arm_03_Degrees_FrontEnd_Fn?servo_Arm_03_Degrees_FrontEnd_Id=' + $('#servo_Arm_03_Degrees_FrontEnd_Id').val();
	if(!servo_Arm_03_ControlsUsed_Bool){servo_Arm_03_ControlsUsed_Bool = true}
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
