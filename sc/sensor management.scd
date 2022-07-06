// set IPs of all accelerometers on the network

~allSensors = ();
(

~setAllIPsAndPorts = {|ipRangeLo = 100, ipRangeHi = 200, networkDevice = "en0", destPort = 8000, numRetries = 4, receivePort = (NetAddr.langPort), actionWhenDone = ({})|
	var myIP, myIParr, nameResponder;
	var waitTime = 0.05;
	myIP = NetAddr.myIP(device: networkDevice); // uses netlib
	// myIP = format("ipconfig getifaddr %", networkDevice).unixCmdGetStdOut; //macos only
	myIParr = myIP.split($.);//.asInteger; //actually we don't need integer there
	"myIP: ".post; myIP.postln;

	~allSensors ?? {~allSensors = ()};

	nameResponder = OSCdef(\accname, {|msg, time, addr|
		var name = msg[1];

		// [msg, time, addr].postln;
		"received name from sensor %".format(name).postln;
		~allSensors[addr.asCompileString.asSymbol] = name;

	}, '/name');

	Routine.run({
		(ipRangeLo..ipRangeHi).do({|thisLastNumber, inc|
			var numTriesForThisIP = 0;
			var thisDestIP, thisNetAddr;
			var thisIPisReachable = true;
			thisDestIP = "".catArgs(myIParr[0] ++ "." ++myIParr[1] ++ "." ++ myIParr[2] ++ "." ++ thisLastNumber.asString);
			thisNetAddr = NetAddr(thisDestIP, destPort);
			Routine.run({
				numRetries.do({
					// ("IP " ++ thisDestIP ++ ", ").post;
					if(thisIPisReachable, {
						try {
							"sending IP to ".post; thisDestIP.postln;
							thisNetAddr.sendMsg('/ip', *myIP.split($.).asInteger);
						} {
							thisIPisReachable = false;
							// ("Couldn't send my IP to " ++ thisDestIP).warn;
						};
					}, {
						thisDestIP.post; " not reachable ".post;
					});
					if(thisIPisReachable, {
						0.1.wait;
						try {
							thisNetAddr.sendMsg('/port', receivePort);
						} {
							thisIPisReachable = false;
							// ("Couldn't send my port to " ++ thisDestIP).warn;
						};
						0.1.wait;
						// ask for name:
						try {
							thisNetAddr.sendMsg('/name');
						}
					});
					0.1.wait;
				});
			});
			waitTime.wait;
		});
		actionWhenDone.();
		"asking for names done.".postln;
	});
}
)

// see what network devices are available
"ifconfig -l".unixCmd;
"ifconfig".unixCmd;
"ifconfig en0 | grep 'inet '".unixCmd; // check for IP of a particular device

// set all sensor devices on the network to our IP (change the networkDevice accordingly!):
~setAllIPsAndPorts.(ipRangeLo: 1, ipRangeHi: 254, networkDevice: "en0", numRetries: 1);

// ~setAllIPsAndPorts.(ipRangeLo: 100, ipRangeHi: 200, networkDevice: "en11", numRetries: 1);

// check names
~allSensors.keys
~allSensors[\acc21];

// after running ~setAllIPsAndPorts, you can see status messages
(
~statusDef = OSCdef(\status, {|msg, time, addr|
	var name;
	// msg.postln;
	name = ~allSensors[addr.asCompileString.asSymbol];
	name !? {
		"%: battery %V".format(name, msg[1].round(0.01)).postln;
	}
}, '/status')
)

~statusDef.free;




