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


function updateMotorSpeeds() {
	// jwc n
	//
	//jwc o  var motor_url = "/motor?l=" + $('#speed-input').val() + '&r=' + $('#heading-input').val();
	//jwc o var motor_url = "/motor?l=" + $('#speed-input').val() + '&r=' + $('#speed-input').val();
	var url_Str = "/motor?l=" + $('#speed-input').val() + '&r=' + $('#speed-input').val();
    request.open("GET", url_Str, true);
    request.send(null);
	// console.log('speed: '+$("#speed-input").val()+'    heading: '+$("#heading-input").val());
	// console.log("/motor?l=" + $('#speed-input').val() + '&r=' + $('#speed-input').val());
	console.log('*** url: ' + url_Str)

}

function update_Servo_Fn(servo_ChannelNum_Int_In) {
	var url_Str = "/update_Servo_Fn?servo_ChannelNum_Int_In=" + servo_ChannelNum_Int_In.toString() + '&cam_Tilt_Degrees_SliderInput=' + $('#cam_Tilt_Degrees_SliderInput').val();
    request.open("GET", url_Str, true);
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
