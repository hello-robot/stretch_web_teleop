const http = require('http');
const socket = require('socket.io');
const server = http.createServer();
const port = 5000;
const io = socket(server, {
    allowEIO3: true
});

server.listen(port, () => {
    console.log('listening on *:' + port);
});

io.on("connect_error", (err) => {
    console.log(`connect_error due to ${err.message}`);
});

io.on('connection', function(socket) {
    console.log('new socket.io connection');
    console.log('socket.handshake = ');
    console.log(socket.handshake);

    socket.on('join', function (room) {
        console.log('Received request to join room ' + room);
        io.sockets.in(room).emit('join', room);
        socket.join(room);
        socket.emit('joined', room, socket.id);
        io.sockets.in(room).emit('ready');
        socket.emit('full', room);
    });
});