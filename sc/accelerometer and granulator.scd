// OSC message tracing
OSCFunc.trace(true, true)
OSCFunc.trace(false)


// this requires NetLib


// sound test
s.boot;
x.free; x = {PinkNoise.ar(-12.dbamp ! 2)}.play(outbus: 0);
x.free;


// start here
// run blocks in order
// setup

(
Server.default.boot;
)

// setup buses for acceleromters
(
~buses !? {~buses.do(_.free)};
~buses = ();

[\accX, \accY, \accZ].do({|name|
	~buses[name] = Bus.control(Server.default, 1);
})
)


(
~resp.free;
~resp = OSCdef(\acc, {|msg|
	// msg.postln;
	var x, y, z;
	#x, y, z = msg[1..3]; // this assumes that we get accelerometer data first
	~buses[\accX].set(x);
	~buses[\accY].set(y);
	~buses[\accZ].set(z);
}, '/acc/acc20') // <<<--- change the sensor name/OSC path here!
)

~buses[\accX].scope; // see if accelerometer data is affecting the bus's value

// create a short buffer for recording
(
~buffer.free;
~buffer = Buffer.alloc(Server.default, Server.default.sampleRate * 2, 1) // 2 seconds
)

(
// one shot
~recorder = {RecordBuf.ar(SoundIn.ar(0), ~buffer, loop: 0, doneAction: Done.freeSelf; DC.ar(0))}.play
)
~recorder.free; // we need to free it manually regardless
~buffer.plot;

(
// loop
~recorder = {RecordBuf.ar(SoundIn.ar(0), ~buffer, loop: 1); DC.ar(0)}.play
)
~recorder.free;

~player = {PlayBuf.ar(1, ~buffer, BufRateScale.kr(~buffer) * 1, doneAction: Done.freeSelf) ! 2 * -12.dbamp}.play;

~player = {PlayBuf.ar(1, ~buffer, BufRateScale.kr(~buffer) * 2, doneAction: Done.freeSelf) ! 2 * -12.dbamp}.play;

~player = {PlayBuf.ar(1, ~buffer, BufRateScale.kr(~buffer) * 1, loop: 1, doneAction: Done.freeSelf) ! 2 * -12.dbamp}.play;

~player.free;



// granulate
(
~grain.free;
~grainFreq = 30;
~overlap = 4;
~grain = {
	var trig = Impulse.ar(~grainFreq);
	var rate = TRand.ar(1, 1, trig);
	GrainBuf.ar(2, trig, ~grainFreq.max(0.001).reciprocal * ~overlap, ~buffer, rate, LFSaw.ar(2.8).range(0, 1), pan: 0)
}.play
)

(
~grain.free;
~grainFreq = 30;
~overlap = 4;
~grain = {
	var trig = Impulse.ar(~grainFreq);
	var rate = TRand.ar(1, 1, trig);
	GrainBuf.ar(2, trig, ~grainFreq.max(0.001).reciprocal * ~overlap, ~buffer, rate, LFDNoise1.ar(12.8).range(0, 1), pan: 0)
}.play
)

(
~grain.free;
~grainFreq = 30;
~overlap = 4;
~grain = {
	var trig = Impulse.ar(~grainFreq);
	var rate = TRand.ar(1, 1, trig);
	GrainBuf.ar(2, trig, ~grainFreq.max(0.001).reciprocal * ~overlap, ~buffer, rate, ~buses[\accX].kr.linlin(-1, 1, 0, 1), pan: 0)
}.play
)


(
~grain.free;
~grainFreq = 30;
~overlap = 4;
~grain = {
	var trig = Impulse.ar(~grainFreq);
	var rate = ~buses[\accY].kr.linlin(-1, 1, -12, 12).midiratio;
	GrainBuf.ar(2, trig, ~grainFreq.max(0.001).reciprocal * ~overlap, ~buffer, rate, ~buses[\accX].kr.linlin(-1, 1, 0, 1), pan: 0) * -6.dbamp
}.play
)


(
~grain.free;
~grainFreq = 30;
~overlap = 4;
~grain = {
	var trig = Impulse.ar(~grainFreq);
	var rate = ~buses[\accY].kr.linlin(0, 1, -12, 12);
	var rateDispersion = ~buses[\accZ].kr.linlin(0, 1, 8, 0);
	var amp = ~buses[\accY].kr.linlin(-1, -0.2, -30, -3).dbamp;
	rate = rate + TRand.ar(rateDispersion.neg, rateDispersion, trig);
	rate = rate.midiratio;
	GrainBuf.ar(2, trig, ~grainFreq.max(0.001).reciprocal * ~overlap, ~buffer, rate, ~buses[\accX].kr.linlin(-1, 1, 0, 1), pan: 0) * amp
}.play
)


~sine.free; ~sine = {SinOsc.ar( ~buses[\accX].kr.linlin(-1, 1, 400, 1000).lag3(0.1), 0, -18.dbamp ! 2)}.play; // test