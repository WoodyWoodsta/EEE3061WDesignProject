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
