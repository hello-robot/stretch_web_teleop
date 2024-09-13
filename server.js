var fs = require("fs");
require("dotenv").config();

var options = {
    key: fs.readFileSync(`certificates/${process.env.keyfile}`),
    cert: fs.readFileSync(`certificates/${process.env.certfile}`),
};

const socket = require("socket.io");
var express = require("express");
var app = express();
app.all("*", ensureSecure); // at top of routing calls

function ensureSecure(req, res, next) {
    if (!req.secure) {
        // handle port numbers if you need non defaults
        console.log("redirecting insecure request");
        return res.redirect("https://" + req.hostname + req.url);
        // res.redirect(`https://${req.hostname}${process.env.NGROK_URL}`);
    }

    return next();
}

var server = require("http").Server(app);
var secure_server = require("https").Server(options, app);
const io = socket(secure_server, {
    allowEIO3: true,
});
app.enable("trust proxy");
app.set("port", 443);
server.listen(80);
secure_server.listen(443);

var path = require("path");
app.use("/", express.static(path.join(__dirname, "dist")));

app.listen(process.env.port);

io.on("connect_error", (err) => {
    console.log(`connect_error due to ${err.message}`);
});

ROOM = 'default';
is_robot_in_room = false;
is_operator_in_room = false;

io.on("connection", function (socket) {
    console.log("new socket.io connection");

    socket.on("join_as_robot", (callback) => {
        console.log("Received join_as_robot request...");
        if (!is_robot_in_room) {
            socket.join(ROOM);
            console.log("join_as_robot SUCCESS");
            is_robot_in_room = true;
            callback({ success: true });
        } else {
            console.log("join_as_robot FAILURE: there's already a robot in the room");
            callback({ success: false });
        }
    });

    // TODO: The browsers shouldnt have to call this. Can socketio tell us when the clients have left?
    socket.on("leave_as_robot", (callback) => {
        console.log("Received leave_as_robot request...");
        if (is_robot_in_room) {
            socket.to(ROOM).emit("bye");
            socket.leave(ROOM);
            console.log("leave_as_robot SUCCESS");
            is_robot_in_room = false;
            callback({ success: true });
        } else {
            callback({ success: false });
        }
    });

    socket.on("join_as_operator", (callback) => {
        console.log("Received join_as_operator request...");
        if (is_robot_in_room) {
            if (!is_operator_in_room) {
                socket.join(ROOM);
                socket.in(ROOM).emit("joined", ROOM);
                console.log("join_as_operator SUCCESS");
                is_operator_in_room = true;
                callback({ success: true });
            } else {
                console.log("join_as_operator FAILURE: there's already a operator in the room");
                callback({ success: false });
            }
        } else {
            console.log("join_as_operator FAILURE: could not join because robot is not available");
            callback({ success: false });
        }
    });

    // TODO: The browsers shouldnt have to call this. Can socketio tell us when the clients have left?
    socket.on("leave_as_operator", (callback) => {
        console.log("Received leave_as_operator request...");
        if (is_operator_in_room) {
            socket.to(ROOM).emit("bye");
            socket.leave(ROOM);
            console.log("leave_as_operator SUCCESS");
            is_operator_in_room = false;
            callback({ success: true });
        } else {
            callback({ success: false });
        }
    });

    socket.on("signaling", (message) => {
        if (is_robot_in_room && is_operator_in_room && io.sockets.adapter.rooms.get(ROOM)) {
            socket.to(ROOM).emit("signaling", message);
        } else {
            console.log("signaling FAILURE: ", is_robot_in_room, is_operator_in_room, io.sockets.adapter.rooms.get(ROOM));
        }
    });
});
