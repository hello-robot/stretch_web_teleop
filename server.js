var fs = require('fs');

var options = {
	key: fs.readFileSync('certificates/slinky.hcrlab.cs.washington.edu+6-key.pem'),
	cert: fs.readFileSync('certificates/slinky.hcrlab.cs.washington.edu+6.pem')
};

// const http = require('http');
// const socket = require('socket.io');
// const server = http.createServer(options);
// const port = 5000;
// const io = socket(server, {
//     allowEIO3: true
// });

// server.listen(port, () => {
//     console.log('listening on *:' + port);
// });

const socket = require('socket.io');
var express = require('express')
var app = express();
app.all('*', ensureSecure); // at top of routing calls

function ensureSecure(req, res, next){
    // console.log('https://' + req.hostname + req.url)
    if(!req.secure){
        // handle port numbers if you need non defaults
        res.redirect('https://' + req.hostname + req.url); 
    }

    return next();
};

var server = require('http').Server(app);
var secure_server = require('https').Server(options, app);
const io = socket(secure_server, {
    allowEIO3: true
});
app.enable('trust proxy')
app.set('port', 443);
server.listen(80);
secure_server.listen(443);

var path = require('path')
app.use('/', express.static(path.join(__dirname, 'dist')));

app.listen(process.env.port)

io.on("connect_error", (err) => {
    console.log(`connect_error due to ${err.message}`);
});

io.on('connection', function(socket) {
    console.log('new socket.io connection');
    // console.log('socket.handshake = ');
    // console.log(socket.handshake);

    socket.on('join', function (room) {
        console.log('Received request to join room ' + room);
        if (!io.sockets.adapter.rooms.get(room) || io.sockets.adapter.rooms.get(room).size < 2) { 
            socket.join(room);
            socket.emit('joined', room, socket.id);
        } else {
            console.log('room full')
            socket.emit('full', room)
        }
    });

    socket.on('is robot available', () => {
        if (io.sockets.adapter.rooms.get('robot')) {
            // console.log(io.sockets.adapter.rooms.get('operator'))
            console.log('robot is available')
            io.in('operator').emit('robot available', true)
            io.in('robot').emit('join', 'robot');
        } else {
            console.log('robot not available')
            io.in('operator').emit('robot available', false)
        }
    })

    socket.on('signalling', function (message) {
        if (io.sockets.adapter.rooms.get('robot')) {
            socket.to('robot').emit('signalling', message);
        } else {
            console.log('robot_operator_room is none, so there is nobody to send the WebRTC message to');
        }
    });

    socket.on('disconnect', function(){
        socket.to('robot').emit('bye');
    });
});