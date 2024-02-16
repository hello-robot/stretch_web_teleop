var fs = require('fs');
require('dotenv').config();

var options = {
    key: fs.readFileSync(`certificates/${process.env.keyfile}`),
    cert: fs.readFileSync(`certificates/${process.env.certfile}`)
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

var pm2 = require('pm2');
const socket = require('socket.io');
var express = require('express')
var app = express();
app.all('*', ensureSecure); // at top of routing calls

function ensureSecure(req, res, next) {
    if (!req.secure) {
        // handle port numbers if you need non defaults
        console.log('redirecting insecure request')
        return res.redirect('https://' + req.hostname + req.url);
        // res.redirect(`https://${req.hostname}${process.env.NGROK_URL}`);
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

io.on('connection', function (socket) {
    console.log('new socket.io connection');
    // console.log('socket.handshake = ');
    // console.log(socket.handshake);

    socket.on('join', function (room) {
        console.log('Received request to join room ' + room);
        // A room can have atmost two clients
        if (!io.sockets.adapter.rooms.get(room) || io.sockets.adapter.rooms.get(room).size < 2) {
            socket.join(room);
            socket.emit('join', room, socket.id);
        } else {
            console.log('room full')
            socket.emit('full', room)
        }
    });

    socket.on('is robot available', () => {
        // The robot room is only available if another operator is not connected to it
        if (io.sockets.adapter.rooms.get('robot')) {
            if (io.sockets.adapter.rooms.get('robot').size < 2) {
                console.log('robot is available')
                socket.join('robot');
                socket.in('robot').emit('joined', 'robot');
                socket.emit('robot available', true)
            } else {
                console.log('robot not available because room is full')
                socket.emit('robot available', false)
            }
        } else {
            console.log('robot not available')
            socket.emit('robot available', false)
        }
    })

    socket.on('signalling', function (message) {
        if (io.sockets.adapter.rooms.get('robot')) {
            socket.to('robot').emit('signalling', message);
        } else {
            console.log('robot_operator_room is none, so there is nobody to send the WebRTC message to');
        }
    });

    socket.on('bye', (role) => {
        console.log(role, socket.rooms)
        if (socket.rooms.has('robot')) {
            socket.to('robot').emit('bye');
            console.log('Attempting to have the ' + role + ' leave the robot room.');
            console.log('');
            socket.leave('robot');
        }
    })
});