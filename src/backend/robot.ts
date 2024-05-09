// TODOs:
//  1. Add support for occupancyGridCallback, moveBaseResultCallback, amclPoseCallback, hasBetaTeleopKitCallback

import * as fs from 'fs';
import * as os from 'os';
import * as yaml from 'js-yaml';
import * as zmq from 'zeromq';
import {
    RobotPose, ValidJointStateDict, BatteryVoltageMessage, // Interfaces
    statustoRobotPose, statustoInJointLimits, statustoInCollision, statustoBatteryVoltageMessage, disallowConcurrency, // Methods
} from '../shared/util_node';


export class Robot {
    private proto_sock: zmq.Request
    private status_sock: zmq.Subscriber
    private moveby_sock: zmq.Request
    private basevel_sock: zmq.Request
    private hncb64_sock: zmq.Subscriber
    private jointStateCallback: (robotPose: RobotPose, jointValues: ValidJointStateDict, effortValues: ValidJointStateDict) => void
    private batteryStateCallback: (batteryState: BatteryVoltageMessage) => void
    // private occupancyGridCallback: (occupancyGrid: ROSOccupancyGrid) => void
    // private moveBaseResultCallback: (goalState: MoveBaseState) => void
    // private amclPoseCallback: (pose: ROSLIB.Transform) => void
    private isRunStoppedCallback: (isRunStopped: boolean) => void
    // private hasBetaTeleopKitCallback: (value: boolean) => void
    private headNavCamStreamUncompressAndSendCallback: (compressed_image: Buffer) => void
    syncStopTrajectory

    constructor(
        jointStateCallback: (robotPose: RobotPose, jointValues: ValidJointStateDict, effortValues: ValidJointStateDict) => void,
        batteryStateCallback: (batteryState: BatteryVoltageMessage) => void,
        // occupancyGridCallback: (occupancyGrid: ROSOccupancyGrid) => void,
        // moveBaseResultCallback: (goalState: MoveBaseState) => void,
        // amclPoseCallback: (pose: ROSLIB.Transform) => void,
        isRunStoppedCallback: (isRunStopped: boolean) => void,
        // hasBetaTeleopKitCallback: (value: boolean) => void
        headNavCamStreamUncompressAndSendCallback: (compressed_image: Buffer) => void
    ) {
        this.jointStateCallback = jointStateCallback
        this.batteryStateCallback = batteryStateCallback
        // this.occupancyGridCallback = occupancyGridCallback
        // this.moveBaseResultCallback = moveBaseResultCallback
        // this.amclPoseCallback = amclPoseCallback
        this.isRunStoppedCallback = isRunStoppedCallback
        // this.hasBetaTeleopKitCallback = hasBetaTeleopKitCallback
        this.headNavCamStreamUncompressAndSendCallback = headNavCamStreamUncompressAndSendCallback
    }

    async connect(): Promise<void> {
        let ip_addr: string;
        let port: number;
        try {
            const config = yaml.load(fs.readFileSync(`${os.homedir()}/.stretch/config.yaml`, 'utf8'));
            ip_addr = config.robots[0].ip_addr;
            port = config.robots[0].port;
        } catch (error) {
            console.error("Failed to connect to Stretch. Confirm ~/.stretch/config.yaml exists.")
        }

        // Verify protocol
        let proto_port = port;
        this.proto_sock = new zmq.Request();
        this.proto_sock.connect(`tcp://${ip_addr}:${proto_port}`);
        this.proto_sock.send("requesting_stretchpy_protocol");
        const [server_protocol] = await this.proto_sock.receive();
        if (server_protocol.toString() !== "spp0") {
            throw new Error("Mismatched protocol error");
        }

        // Connect to body
        let status_port = port + 1;
        this.status_sock = new zmq.Subscriber();
        this.status_sock.connect(`tcp://${ip_addr}:${status_port}`);
        this.status_sock.subscribe("");
        let moveby_port = port + 2;
        this.moveby_sock = new zmq.Request();
        this.moveby_sock.connect(`tcp://${ip_addr}:${moveby_port}`);
        let basevel_port = port + 3;
        this.basevel_sock = new zmq.Request();
        this.basevel_sock.connect(`tcp://${ip_addr}:${basevel_port}`);

        // Connect to head nav camera
        let hncb64_port = port + 5
        this.hncb64_sock = new zmq.Subscriber();
        this.hncb64_sock.connect(`tcp://${ip_addr}:${hncb64_port}`);
        this.hncb64_sock.subscribe("");

        // Establish subscriptions
        this.subscribeToStatus();
        this.subscribeToHeadNavCam();

        // Bind sync methods
        this.syncStopTrajectory = disallowConcurrency(this.stopTrajectory);
    }

    subscribeToStatus() {
        (async function() {
            for await (const [status_buff] of this.status_sock) {
                // joint state
                let status = JSON.parse(JSON.parse(status_buff));
                let robotPose: RobotPose = statustoRobotPose(status);
                let jointValues: ValidJointStateDict = statustoInJointLimits(status);
                let effortValues: ValidJointStateDict = statustoInCollision(status);
                if (this.jointStateCallback) { this.jointStateCallback(robotPose, jointValues, effortValues) }

                // battery state
                let batteryState: BatteryVoltageMessage = statustoBatteryVoltageMessage(status);
                if (this.batteryStateCallback) { this.batteryStateCallback(batteryState) }

                // runstop state
                if (this.isRunStoppedCallback) { this.isRunStoppedCallback(status['other']['is_runstopped']) }
            }
        }).call(this);
    };

    subscribeToHeadNavCam() {
        (async function() {
            for await (const [hncb64_buff] of this.hncb64_sock) {
                if (this.headNavCamStreamUncompressAndSendCallback) { this.headNavCamStreamUncompressAndSendCallback(hncb64_buff) }
            }
        }).call(this);
    }

    async stopTrajectory() {
        let stop_cmd = {
            'joint_translate': 0.0,
            'joint_lift': 0.0,
            'joint_arm': 0.0,
            'joint_wrist_yaw': 0.0,
            'joint_wrist_pitch': 0.0,
            'joint_wrist_roll': 0.0,
            'joint_head_pan': 0.0,
            'joint_head_tilt': 0.0,
            'joint_gripper': 0.0,
        };
        await this.moveby_sock.send(JSON.stringify(stop_cmd));
        await this.moveby_sock.receive();
    }

    executeIncrementalMove(joint_name, delta) {
        let cmd = {};
        cmd[joint_name] = delta;
        this.moveby_sock.send(JSON.stringify(cmd));
        this.moveby_sock.receive();
    }

    executeBaseVelocity(linVel, angVel) {
        let cmd = {
            'translational_vel': linVel,
            'rotational_vel': angVel,
        };
        this.basevel_sock.send(JSON.stringify(cmd));
        this.basevel_sock.receive();
    }
}

