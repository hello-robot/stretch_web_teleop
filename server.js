var fs = require('fs');
require('dotenv').config();

var options = {
    key: fs.readFileSync(`certificates/${process.env.keyfile}`),
    cert: fs.readFileSync(`certificates/${process.env.certfile}`),
};

const socket = require('socket.io');
var express = require('express');
var app = express();
app.all('*', ensureSecure); // at top of routing calls

function ensureSecure(req, res, next) {
    if (!req.secure) {
        // handle port numbers if you need non defaults
        console.log('redirecting insecure request');
        return res.redirect('https://' + req.hostname + req.url);
        // res.redirect(`https://${req.hostname}${process.env.NGROK_URL}`);
    }

    return next();
}

var server = require('http').Server(app);
var secure_server = require('https').Server(options, app);
const io = socket(secure_server, {
    allowEIO3: true,
});
app.enable('trust proxy');
app.set('port', 443);
server.listen(80);
secure_server.listen(443);

var path = require('path');
app.use('/', express.static(path.join(__dirname, 'dist')));

app.listen(process.env.port);

io.on('connect_error', (err) => {
    console.log(`connect_error due to ${err.message}`);
});

const ROOM = 'default';
let robo_sock = undefined;
let oper_sock = undefined;
let protocol = undefined; // TODO(binit): ensure robot/operator protocol match
let status = 'offline'; // ["online", "offline", "occupied"]
function updateRooms() {
    io.emit('update_rooms', {
        robot_id: {
            name: process.env.HELLO_FLEET_ID,
            protocol: protocol,
            status: status,
        },
    });
}

io.on('connection', function (socket) {
    console.log('new socket.io connection');
    // console.log('socket.handshake = ');
    // console.log(socket.handshake);

    socket.on('join_as_robot', (callback) => {
        console.log('Received join_as_robot request');
        if (!robo_sock) {
            socket.join(ROOM);
            robo_sock = socket.id;
            status = 'online';
            console.log('join_as_robot SUCCESS');
            callback({ success: true });
        } else {
            status = 'occupied';
            console.log('join_as_robot FAILURE');
            callback({ success: false });
        }
        updateRooms();
    });

    socket.on('list_rooms', () => {
        updateRooms();
    });

    socket.on('join_as_operator', (callback) => {
        console.log('Received join_as_operator request');
        if (robo_sock) {
            status = 'occupied';
            if (!oper_sock) {
                socket.join(ROOM);
                socket.in(ROOM).emit('joined');
                oper_sock = socket.id;
                console.log('join_as_operator SUCCESS');
                callback({ success: true });
            } else {
                console.log(
                    'join_as_operator FAILURE: occupied by another operator'
                );
                callback({ success: false });
            }
        } else {
            status = 'offline';
            console.log('join_as_operator FAILURE: robot is not available');
            callback({ success: false });
        }
        updateRooms();
    });

    socket.on('signalling', (message) => {
        if (robo_sock && oper_sock && io.sockets.adapter.rooms.get(ROOM)) {
            socket.to(ROOM).emit('signalling', message);
        } else {
            console.log(
                `signaling FAILURE: robo_sock=${robo_sock} oper_sock=${oper_sock} room=${io.sockets.adapter.rooms.get(ROOM)}`
            );
        }
    });

    socket.on('bye', (role) => {
        console.log(`Received bye from ${role}`);
        if (socket.rooms.has(ROOM)) {
            socket.to(ROOM).emit('bye');
            if (socket.id == robo_sock) {
                status = 'offline';
                robo_sock = undefined;
                console.log('Robot disconnected');
            }
            if (socket.id == oper_sock) {
                status = 'online';
                oper_sock = undefined;
                console.log('Operator disconnected');
            }
            socket.leave(ROOM);
        }
        updateRooms();
    });

    socket.on('disconnect', () => {
        if (socket.id == robo_sock) {
            status = 'offline';
            robo_sock = undefined;
            console.log('Robot disconnected');
        }
        if (socket.id == oper_sock) {
            status = 'online';
            oper_sock = undefined;
            console.log('Operator disconnected');
        }
        updateRooms();
    });
});

const { spawn } = require('child_process');
let rosbagProcess = null;

app.post('/start_rosbag', (req, res) => {
    if (rosbagProcess) {
        return res.status(400).json({ error: 'Rosbag recording already in progress.' });
    }

    // Check for flash drive
    const fs = require('fs');
    const path = require('path');
    
    let outputDir;
    let flashDrivePath = '/media/HCRLAB';
    
    // Check if flash drive is mounted
    if (fs.existsSync(flashDrivePath)) {
        console.log('Flash drive found at /media/HCRLAB');
        outputDir = path.join(flashDrivePath, 'rosbags');
    } else {
        console.log('Flash drive not found, using local storage');
        outputDir = 'rosbags';
    }
    
    // Create rosbags directory if it doesn't exist
    if (!fs.existsSync(outputDir)) {
        fs.mkdirSync(outputDir, { recursive: true });
    }
    
    // Generate descriptive filename
    const now = new Date();
    const dateStr = now.toISOString().slice(0, 19).replace(/:/g, '-');
    const userLabel = req.body.label || 'recording';
    const safeLabel = userLabel.replace(/[^a-zA-Z0-9_-]/g, '_');
    
    const bagName = `${safeLabel}_${dateStr}`;
    const fullOutputDir = path.join(outputDir, bagName);
    
    console.log(`Starting rosbag recording to: ${fullOutputDir}`);
    
    rosbagProcess = spawn('ros2', [
        'bag', 'record',
        '-a',
        '-s', 'mcap',
        '-o', fullOutputDir
    ], {
        detached: true,
        stdio: ['ignore', 'pipe', 'pipe']
    });

    rosbagProcess.stdout.on('data', (data) => {
        console.log(`[rosbag stdout]: ${data}`);
    });
    rosbagProcess.stderr.on('data', (data) => {
        console.error(`[rosbag stderr]: ${data}`);
    });
    rosbagProcess.on('exit', (code, signal) => {
        console.log(`ros2 bag record exited with code ${code}, signal ${signal}`);
        rosbagProcess = null; 
    });
    
    res.json({ 
        status: 'started', 
        dir: fullOutputDir,
        bagName: bagName,
        flashDrive: fs.existsSync(flashDrivePath)
    });
});

app.post('/stop_rosbag', (req, res) => {
    if (!rosbagProcess) {
        return res.status(400).json({ error: 'No rosbag recording in progress.' });
    }
    try {
        process.kill(-rosbagProcess.pid, 'SIGINT');
        res.json({ status: 'stopped' });
    } catch (e) {
        console.error('Error stopping rosbag process:', e);
        return res.status(500).json({ error: 'Failed to stop rosbag process.' });
    }
});

app.get('/list_rosbags', (req, res) => {
    const fs = require('fs');
    const path = require('path');
    
    let rosbagDirs = [];
    
    // Check local rosbags
    const localRosbagsPath = 'rosbags';
    if (fs.existsSync(localRosbagsPath)) {
        try {
            const localBags = fs.readdirSync(localRosbagsPath)
                .filter(item => fs.statSync(path.join(localRosbagsPath, item)).isDirectory())
                .map(bag => ({
                    name: bag,
                    path: path.join(localRosbagsPath, bag),
                    location: 'Local Storage',
                    size: getDirectorySize(path.join(localRosbagsPath, bag)),
                    modified: fs.statSync(path.join(localRosbagsPath, bag)).mtime
                }));
            rosbagDirs.push(...localBags);
        } catch (e) {
            console.error('Error reading local rosbags:', e);
        }
    }
    
    // Check flash drive rosbags
    const flashDrivePath = '/media/HCRLAB/rosbags';
    if (fs.existsSync(flashDrivePath)) {
        try {
            const flashBags = fs.readdirSync(flashDrivePath)
                .filter(item => fs.statSync(path.join(flashDrivePath, item)).isDirectory())
                .map(bag => ({
                    name: bag,
                    path: path.join(flashDrivePath, bag),
                    location: 'Flash Drive',
                    size: getDirectorySize(path.join(flashDrivePath, bag)),
                    modified: fs.statSync(path.join(flashDrivePath, bag)).mtime
                }));
            rosbagDirs.push(...flashBags);
        } catch (e) {
            console.error('Error reading flash drive rosbags:', e);
        }
    }
    
    // Sort by modification time (newest first)
    rosbagDirs.sort((a, b) => b.modified.getTime() - a.modified.getTime());
    
    res.json({ rosbags: rosbagDirs });
});

function getDirectorySize(dirPath) {
    try {
        const fs = require('fs');
        const path = require('path');
        
        let totalSize = 0;
        const files = fs.readdirSync(dirPath);
        
        for (const file of files) {
            const filePath = path.join(dirPath, file);
            const stats = fs.statSync(filePath);
            
            if (stats.isDirectory()) {
                totalSize += getDirectorySize(filePath);
            } else {
                totalSize += stats.size;
            }
        }
        
        return totalSize;
    } catch (e) {
        return 0;
    }
}
