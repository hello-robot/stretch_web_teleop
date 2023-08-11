import { Robot } from '../src/pages/robot/tsx/robot';
import { ROSJointState, ROSCompressedImage, ValidJoints, VideoProps, ROSOccupancyGrid, ROSPose, FollowJointTrajectoryActionResult, MarkerArray, MoveBaseActionResult, ArucoMarkersInfo, ArucoNavigationState, MoveBaseState, ArucoNavigationFeedback } from 'shared/util';
import ROSLIB, { Goal, Message, Ros, Topic } from "roslib";
import { JOINT_LIMITS, RobotPose, generateUUID, waitUntil } from '../src/shared/util';

// Example function to demonstrate the usage of Jest
const add = (numbers: string): number => {
    let integers = numbers.split(',').map(x => parseInt(x));
    let negatives = integers.filter(x => x < 0);

    if (negatives.length > 0)
        throw new RangeError('Negatives are not allowed: ' + negatives.join(', '));

    return integers
        .filter(x => x <= 1000)
        .reduce((a, b) => a + b, 0);
}

// Test adding a negative number, used by test #2
const testNegative = () => {
    add('-1');
}

const testROSConnection = async (): Promise<Boolean> => {
    // Create a new ROS connection
    var ros = new ROSLIB.Ros({
        url : 'wss://localhost:9090'
    });
    let connected = false;
    let start = Date.now();
    
    ros.on('connection', function() {
        // Connection succeeded
        connected = true;
    });

    ros.on('error', function(error) {
        // Do nothing
    });
    
    ros.on('close', function() {
        // Do nothing
    });

    // Wait until we connect to the rosbridge server
    while (!connected && Date.now() - start <= 2000) {
        const sleep = (ms: number) => new Promise(r => setTimeout(r, ms));
        await sleep(100);
    }

    if (!connected)
        return false;

    return true;
};

const testTFListener = async (): Promise<Boolean> => {
    // Create a new ROS connection
    var ros = new ROSLIB.Ros({
        url : 'wss://localhost:9090'
    });
    let connected = false;
    let start = Date.now();

    ros.on('connection', function() {
        // Connection succeeded
        connected = true;
    });

    ros.on('error', function(error) {
        // Do nothing
    });
    
    ros.on('close', function() {
        // Do nothing
    });

    // Wait until we connect to the rosbridge server
    while (!connected && Date.now() - start <= 2000) {
        const sleep = (ms: number) => new Promise(r => setTimeout(r, ms));
        await sleep(100);
    }

    if (!connected)
        return false;

    // Initialize a new TF client object and pass in our ROS context.
    // <map> is the fixed frame and all transforms will be w.r.t to /map
    var tfClient = new ROSLIB.TFClient({
        ros : ros,
        fixedFrame : 'map',
        angularThres : 0.01,
        transThres : 0.01
    });

    var tfReceived = false;
    start = Date.now();

    // Create a new TF subscription between /map (fixed frame) and /base_link
    // If an invalid frame is passed, we still get a transform but populated with zero values
    tfClient.subscribe('base_link', function(tf) {
        // Print out TF quaternion values to check if they appear to be sane
        console.log('Received!', tf)
        tfReceived = true;
    });

    // Wait until we receive a TF message
    while (!tfReceived && Date.now() - start <= 2000) {
        const sleep = (ms: number) => new Promise(r => setTimeout(r, ms));
        await sleep(100);
    }

    if (!tfReceived)
        return false;

    return true;
};

const testCameraTopics = async (): Promise<Boolean> => {
    // Create a new ROS connection
    var ros = new ROSLIB.Ros({
        url : 'wss://localhost:9090'
    });
    let connected = false;
    let start = Date.now();

    ros.on('connection', function() {
        // Connection succeeded
        connected = true;
    });

    ros.on('error', function(error) {
        // Do nothing
    });
    
    ros.on('close', function() {
        // Do nothing
    });

    // Wait until we connect to the rosbridge server
    while (!connected && Date.now() - start <= 2000) {
        const sleep = (ms: number) => new Promise(r => setTimeout(r, ms));
        await sleep(100);
    }

    if (!connected)
        return false;

    // Create a topic object for all the three cameras
    var realsenseTopic = new ROSLIB.Topic({
        ros : ros,
        name : '/camera/color/image_raw',
        messageType : 'sensor_msgs/Image'
    });

    var navigationCameraTopic = new ROSLIB.Topic({
        ros : ros,
        name : '/navigation_camera/image_raw',
        messageType : 'sensor_msgs/Image'
    });

    var gripperCameraTopic = new ROSLIB.Topic({
        ros : ros,
        name : '/gripper_camera/image_raw',
        messageType : 'sensor_msgs/Image'
    });

    var receivedRealsense = false;
    var receivedNavigationCamera = false;
    var receivedGripperCamera = false;

    // Create topic subscriptions
    realsenseTopic.subscribe(function(message: ROSLIB.Message) {
        receivedRealsense = true;
    });

    navigationCameraTopic.subscribe(function(message: ROSLIB.Message) {
        receivedNavigationCamera = true;
    });

    gripperCameraTopic.subscribe(function(message: ROSLIB.Message) {
        receivedGripperCamera = true;
    });

    // Wait until we receive all messages
    start = Date.now();
    while ((!receivedRealsense || !receivedNavigationCamera || !receivedGripperCamera) && Date.now() - start <= 3000) {
        const sleep = (ms: number) => new Promise(r => setTimeout(r, ms));
        await sleep(100);
    }

    if (!receivedRealsense || !receivedNavigationCamera || !receivedGripperCamera)
        return false;

    return true;
};

const testPublishCmdVel = async (): Promise<Boolean> => {
    // Create a new ROS connection
    var ros = new ROSLIB.Ros({
        url : 'wss://localhost:9090'
    });
    let connected = false;
    let start = Date.now();

    ros.on('connection', function() {
        // Connection succeeded
        connected = true;
    });

    ros.on('error', function(error) {
        // Do nothing
    });
    
    ros.on('close', function() {
        // Do nothing
    });

    // Wait until we connect to the rosbridge server
    while (!connected && Date.now() - start <= 2000) {
        const sleep = (ms: number) => new Promise(r => setTimeout(r, ms));
        await sleep(100);
    }

    if (!connected)
        return false;

    // Create a ROS topic object
    var cmdVel = new ROSLIB.Topic({
        ros : ros,
        name : '/cmd_vel',
        messageType : 'geometry_msgs/Twist'
    });

    // Create twist message type
    var twist = new ROSLIB.Message({
        linear : {
          x : 0.1,
          y : 0.2,
          z : 0.3
        },
        angular : {
          x : -0.1,
          y : -0.2,
          z : -0.3
        }
    });

    // Publish the velocity command
    cmdVel.publish(twist);

    return true;
};

const testSwitchStretchDriverModes = async (): Promise<Boolean> => {
    // Create a new ROS connection
    var ros = new ROSLIB.Ros({
        url : 'wss://localhost:9090'
    });
    let connected = false;
    let start = Date.now();

    ros.on('connection', function() {
        // Connection succeeded
        connected = true;
    });

    ros.on('error', function(error) {
        // Do nothing
    });
    
    ros.on('close', function() {
        // Do nothing
    });

    // Wait until we connect to the rosbridge server
    while (!connected && Date.now() - start <= 2000) {
        const sleep = (ms: number) => new Promise(r => setTimeout(r, ms));
        await sleep(100);
    }

    if (!connected)
        return false;

    // Create ROS service objects for position and navigation modes
    var switchToPositionModeClient = new ROSLIB.Service({
        ros : ros,
        name : '/switch_to_position_mode',
        serviceType : 'std_srvs/Trigger'
    });

    var switchToNavigationModeClient = new ROSLIB.Service({
        ros : ros,
        name : '/switch_to_navigation_mode',
        serviceType : 'std_srvs/Trigger'
    });

    // Create a Trigger request
    var request = new ROSLIB.ServiceRequest({});

    // Call services
    var positionSucceeded = false;
    var navigationSucceeded = false;

    switchToNavigationModeClient.callService(request, function(result) {
        navigationSucceeded = result.success;
    });
    switchToPositionModeClient.callService(request, function(result) {
        positionSucceeded = result.success;
    });

    // Wait until the services finish execution
    start = Date.now();
    while ((!navigationSucceeded || !positionSucceeded) && Date.now() - start <= 2000) {
        const sleep = (ms: number) => new Promise(r => setTimeout(r, ms));
        await sleep(100);
    }

    if (!positionSucceeded || !navigationSucceeded)
        return false;

    return true;
};

// Main test function
describe('testing ROS functionalities', () => {
    test('empty string should result in zero', () => {
        expect(add('')).toBe(0);
    });

    test('negative number should throw RangeError', () => {
        expect(testNegative).toThrow(RangeError);
    });

    test('ROS Client must connect to wss://localhost:9090', async () => {
        expect(await testROSConnection()).toBe(true);
    });

    test('ROS TF client must be able to fetch a transform from /map to /base_link', async () => {
        expect(await testTFListener()).toBe(true);
    });

    test('RealSense and fisheye camera images must be published', async () => {
        expect(await testCameraTopics()).toBe(true);
    });

    test('Test publishing to /cmd_vel', async () => {
        expect(await testPublishCmdVel()).toBe(true);
    });

    test('Test switching between position and navigation modes', async () => {
        expect(await testSwitchStretchDriverModes()).toBe(true);
    });
});
