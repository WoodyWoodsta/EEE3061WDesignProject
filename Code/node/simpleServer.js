/*
In the node.js intro tutorial (http://nodejs.org/), they show a basic tcp
server, but for some reason omit a client connecting to it.  I added an
example at the bottom.
Save the following server in example.js:
*/

var net = require('net');

var server = net.createServer(function(socket) {
	// Set the correct encoding
	socket.setEncoding('utf8');

	// Data receive callback
	socket.on('data', function(data) {
		// Just log the data received and echo it back
		console.log('Received from client: ' + data);
		socket.write(data);
	});
});

// Set the server to listen on the correct ip and port
server.listen(1338, '192.168.137.1');

// Callback for new connections
server.on('connection', function(){
	// Notify that there is a new connection
	console.log('New client connected');
});

/*
And connect with a tcp client from the command line using netcat, the *nix
utility for reading and writing across tcp/udp network connections.  I've only
used it for debugging myself.
$ netcat 127.0.0.1 1337
You should see:
> Echo server
*/

/* Or use this example tcp client written in node.js.  (Originated with
example code from
http://www.hacksparrow.com/tcp-socket-programming-in-node-js.html.) */

// var net = require('net');
//
// var client = new net.Socket();
// client.connect(1338, '127.0.0.1', function() {
// 	console.log('Client connected');
// 	client.write('Hello, server! Love, Client.');
// });
//
// client.on('data', function(data) {
// 	console.log('Received: ' + data);
// 	client.destroy(); // kill client after server's response
// });
//
// client.on('close', function() {
// 	console.log('Connection closed');
// });
