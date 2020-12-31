var socket, connected = false;

// jwc n
var request = makeHttpObject();

addEventListener('load',setUpControllerEvents);

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
  

function updateMotorSpeeds() {
	var url_Str = "/motor?l=" + $('#dcMotors_FwdOrRev_Power_FrontEnd_Id').val() + '&r=' + $('#dcMotors_FwdOrRev_Power_FrontEnd_Id').val();
    request.open("GET", url_Str, true);
    request.send(null);
	console.log('*** url: ' + url_Str)
	// If Power == 0, then resend multiple times to insure at least one packet survives transmission (not get dropped in network) for critical stop
	if ($('#dcMotors_FwdOrRev_Power_FrontEnd_Id').val()==0) {
		for (i=0; i<3; i++) { 
			request.open("GET", url_Str, true);
			request.send(null);
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
	var url_Str = "/motor_for_turn?l=" + $('#dcMotors_Turn_Power_FrontEnd_Id').val() + '&r=' + $('#dcMotors_Turn_Power_FrontEnd_Id').val();
    request.open("GET", url_Str, true);
    request.send(null);
	console.log('*** url: ' + url_Str)
	// If Power == 0, then resend multiple times to insure at least one packet survives transmission (not get dropped in network) for critical stop
	if ($('#dcMotors_Turn_Power_FrontEnd_Id').val()==0) {
		for (i=0; i<3; i++) { 
			request.open("GET", url_Str, true);
			request.send(null);
			console.log('*** *** ' + i + ' url: ' + url_Str);
			sleep(50);
		}
	}

}

function servo_Cam_01_Pan_Degrees_FrontEnd_Fn() {
	var url_Str = '/servo_Cam_01_Pan_Degrees_FrontEnd_Fn?servo_Cam_01_Pan_Degrees_FrontEnd_Id=' + $('#servo_Cam_01_Pan_Degrees_FrontEnd_Id').val();
    request.open('GET', url_Str, true);
    request.send(null);
	console.log('*** url: ' + url_Str)
}

function servo_Cam_02_Tilt_Degrees_FrontEnd_Fn() {
	var url_Str = '/servo_Cam_02_Tilt_Degrees_FrontEnd_Fn?servo_Cam_02_Tilt_Degrees_FrontEnd_Id=' + $('#servo_Cam_02_Tilt_Degrees_FrontEnd_Id').val();
    request.open('GET', url_Str, true);
    request.send(null);
	console.log('*** url: ' + url_Str)
}

function servo_Arm_03_Degrees_FrontEnd_Fn() {
	var url_Str = '/servo_Arm_03_Degrees_FrontEnd_Fn?servo_Arm_03_Degrees_FrontEnd_Id=' + $('#servo_Arm_03_Degrees_FrontEnd_Id').val();
    request.open('GET', url_Str, true);
    request.send(null);
	console.log('*** url: ' + url_Str)
}

function changeTrim(data) {
	///jwc o socket.emit('updateTrim',{'L':data.L, 'R':data.R});
	var motorTrim_url = "/motorTrim?l=" + data.L + '&r=' + data.R;
    request.open("GET", motorTrim_url, true);
    request.send(null);
	
	console.log('### DEBUG: asking server to update motorTrim');
}

function turnCam(data) {
	// send change in camera heading left
	console.log(JSON.stringify(data,undefined,4));
}
