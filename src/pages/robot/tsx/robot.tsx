import React from "react";
import ROSLIB from "roslib";
import {
    ROSJointState,
    ROSCompressedImage,
    ValidJoints,
    VideoProps,
    ROSOccupancyGrid,
    ROSPose,
    ActionState,
    ActionStatusList,
    ROSBatteryState,
} from "shared/util";
import {
    rosJointStatetoRobotPose,
    ValidJointStateDict,
    RobotPose,
    IsRunStoppedMessage,
} from "../../../shared/util";

export var robotMode: "navigation" | "position" | "unknown" = "position";
export var rosConnected = false;

// Names of ROS actions
const moveBaseActionName = "/navigate_to_pose";
const moveToPregraspActionName = "/move_to_pregrasp";
const showTabletActionName = "/show_tablet";

export class Robot extends React.Component {
    private ros: ROSLIB.Ros;
    private readonly rosURL = "wss://localhost:9090";
    private rosReconnectTimerID?: ReturnType<typeof setTimeout>;
    private onRosConnectCallback?: () => Promise<void>;
    private jointLimits: { [key in ValidJoints]?: [number, number] } = {};
    private jointState?: ROSJointState;
    private poseGoal?: ROSLIB.ActionGoal;
    private poseGoalComplete?: boolean;
    private isRunStopped?: boolean;
    private moveBaseGoal?: ROSLIB.ActionGoal;
    private trajectoryClient?: ROSLIB.ActionClient;
    private moveBaseClient?: ROSLIB.ActionClient;
    private moveToPregraspGoal?: ROSLIB.ActionGoal;
    private showTabletGoal?: ROSLIB.ActionGoal;
    private moveToPregraspClient?: ROSLIB.ActionClient;
    private showTabletClient?: ROSLIB.ActionClient;
    private cmdVelTopic?: ROSLIB.Topic;
    private switchToNavigationService?: ROSLIB.Service;
    private switchToPositionService?: ROSLIB.Service;
    private setCameraPerspectiveService?: ROSLIB.Service;
    private setRealsenseDepthSensingService?: ROSLIB.Service;
    private setGripperDepthSensingService?: ROSLIB.Service;
    private setExpandedGripperService?: ROSLIB.Service;
    private setRealsenseShowBodyPoseService?: ROSLIB.Service;
    private setComputeBodyPoseService?: ROSLIB.Service;
    private setRunStopService?: ROSLIB.Service;
    private robotFrameTfClient?: ROSLIB.TFClient;
    private mapFrameTfClient?: ROSLIB.TFClient;
    private linkGripperFingerLeftTF?: ROSLIB.Transform;
    private linkTabletTF?: ROSLIB.Transform;
    private linkWristYawTF?: ROSLIB.Transform;
    private linkHeadTiltTF?: ROSLIB.Transform;
    private jointStateCallback: (
        robotPose: RobotPose,
        jointValues: ValidJointStateDict,
        effortValues: ValidJointStateDict,
    ) => void;
    private batteryStateCallback: (batteryState: ROSBatteryState) => void;
    private occupancyGridCallback: (occupancyGrid: ROSOccupancyGrid) => void;
    private moveBaseResultCallback: (goalState: ActionState) => void;
    private moveToPregraspResultCallback: (goalState: ActionState) => void;
    private showTabletResultCallback: (goalState: ActionState) => void;
    private amclPoseCallback: (pose: ROSLIB.Transform) => void;
    private modeCallback: (mode: string) => void;
    private isHomedCallback: (isHomed: boolean) => void;
    private isRunStoppedCallback: (isRunStopped: boolean) => void;
    private hasBetaTeleopKitCallback: (value: boolean) => void;
    private stretchToolCallback: (value: string) => void;
    private lookAtGripperInterval?: number; // ReturnType<typeof setInterval>
    private subscriptions: ROSLIB.Topic[] = [];
    private hasBetaTeleopKitParam: ROSLIB.Param;
    private stretchToolParam: ROSLIB.Param;
    private textToSpeechTopic?: ROSLIB.Topic;
    private homeTheRobotService?: ROSLIB.Service;

    constructor(props: {
        jointStateCallback: (
            robotPose: RobotPose,
            jointValues: ValidJointStateDict,
            effortValues: ValidJointStateDict,
        ) => void;
        batteryStateCallback: (batteryState: ROSBatteryState) => void;
        occupancyGridCallback: (occupancyGrid: ROSOccupancyGrid) => void;
        moveBaseResultCallback: (goalState: ActionState) => void;
        moveToPregraspResultCallback: (goalState: ActionState) => void;
        showTabletResultCallback: (goalState: ActionState) => void;
        amclPoseCallback: (pose: ROSLIB.Transform) => void;
        modeCallback: (mode: string) => void;
        isHomedCallback: (isHomed: boolean) => void;
        isRunStoppedCallback: (isRunStopped: boolean) => void;
        hasBetaTeleopKitCallback: (value: boolean) => void;
        stretchToolCallback: (value: string) => void;
    }) {
        super(props);
        this.jointStateCallback = props.jointStateCallback;
        this.batteryStateCallback = props.batteryStateCallback;
        this.occupancyGridCallback = props.occupancyGridCallback;
        this.moveBaseResultCallback = props.moveBaseResultCallback;
        this.moveToPregraspResultCallback = props.moveToPregraspResultCallback;
        this.showTabletResultCallback = props.showTabletResultCallback;
        this.amclPoseCallback = props.amclPoseCallback;
        this.modeCallback = props.modeCallback;
        this.isHomedCallback = props.isHomedCallback;
        this.isRunStoppedCallback = props.isRunStoppedCallback;
        this.hasBetaTeleopKitCallback = props.hasBetaTeleopKitCallback;
        this.stretchToolCallback = props.stretchToolCallback;
    }

    setOnRosConnectCallback(callback: () => Promise<void>) {
        this.onRosConnectCallback = callback;
    }

    async connect(): Promise<void> {
        console.log("Connecting to ROS...");
        this.ros = new ROSLIB.Ros({
            // set this to false to use the new service interface to
            // tf2_web_republisher. true is the default and means roslibjs
            // will use the action interface
            groovyCompatibility: false,
            url: this.rosURL,
        });

        this.ros.on("connection", async () => {
            console.log("Connected to ROS.");
            // We check that bidirectional communications with ROS are working, and
            // that some key topics have publishers (which are indicative of all
            // required nodes being loaded). This is because ROSbridge matches the
            // QoS of publishers, so without publishers there is likely to be a
            // QoS mismatch.
            let isConnected = await this.checkROSConnection();
            if (isConnected) {
                await this.onConnect();
                if (this.onRosConnectCallback)
                    await this.onRosConnectCallback();
            } else {
                console.log(
                    "Required ROS nodes are not yet loaded. Reconnecting.",
                );
                this.reconnect();
            }
        });
        this.ros.on("error", (error) => {
            console.log("Error connecting to ROS:", error);
            this.reconnect();
        });

        this.ros.on("close", () => {
            console.log("Connection to ROS has been closed.");
            this.reconnect();
        });
    }

    async reconnect(interval_ms: number = 1000) {
        if (!this.rosReconnectTimerID) {
            this.rosReconnectTimerID = setTimeout(() => {
                console.log("Reconnecting to ROS...");
                this.ros.close();
                this.ros.connect(this.rosURL);
                this.rosReconnectTimerID = undefined;
            }, interval_ms);
        }
    }

    async checkROSConnection(
        required_topics: string[] = [
            "/camera/color/image_raw/rotated/compressed",
            "/gripper_camera/image_raw/cropped/compressed",
            "/navigation_camera/image_raw/rotated/compressed",
            "/stretch/joint_states",
        ],
        timeout_ms: number = 5000,
    ): Promise<boolean> {
        // For backwards compatibility with older versions of roslibjs, use the
        // local copy of getPublishers if the ROS object does not have it.
        let getPublishers = this.getPublishers.bind(this);
        if (this.ros.getPublishers !== undefined) {
            getPublishers = this.ros.getPublishers.bind(this.ros);
        }

        let numRequiredTopicsWithPublisher = 0;
        let isResolved = false;
        console.log("Checking ROS connection...");
        return new Promise(async (resolve) => {
            if (this.ros.isConnected) {
                for (let topic of required_topics) {
                    // Verify that the topic has a publisher
                    getPublishers(
                        topic,
                        // Success callback
                        (publishers: string[]) => {
                            if (publishers.length > 0) {
                                console.log("Got a publisher on topic", topic);
                                numRequiredTopicsWithPublisher += 1;
                                if (
                                    numRequiredTopicsWithPublisher ===
                                    required_topics.length
                                ) {
                                    console.log(
                                        "Got publishers on all required topics.",
                                    );
                                    isResolved = true;
                                    resolve(true);
                                }
                            } else {
                                console.log("No publisher on topic", topic);
                                isResolved = true;
                                resolve(false);
                            }
                        },
                        // Failure callback
                        (error) => {
                            console.log(
                                "Error in getting publishers for topic",
                                topic,
                                error,
                            );
                            isResolved = true;
                            resolve(false);
                        },
                    );
                }
                resolve(
                    await new Promise<boolean>((resolve) =>
                        setTimeout(() => {
                            if (!isResolved) {
                                if (
                                    numRequiredTopicsWithPublisher <
                                    required_topics.length
                                ) {
                                    console.log(
                                        "Timed out with at least one required topic not having publishers.",
                                    );
                                    resolve(false);
                                }
                            }
                        }, timeout_ms),
                    ),
                );
            } else {
                console.log("ROS is not connected.");
                isResolved = true;
                resolve(false);
            }
        });
    }

    async onConnect() {
        console.log("onConnect");
        this.subscribeToJointState();
        this.subscribeToJointLimits();
        this.subscribeToBatteryState();
        this.subscribeToMode();
        this.subscribeToIsHomed();
        this.subscribeToIsRunStopped();
        this.subscribeToActionResult(
            moveBaseActionName,
            this.moveBaseResultCallback,
            "Navigation canceled!",
            "Navigation succeeded!",
            "Navigation failed!",
        );
        this.subscribeToActionResult(
            moveToPregraspActionName,
            this.moveToPregraspResultCallback,
            "Move To Pre-grasp canceled!",
            "Move To Pre-grasp succeeded!",
            "Move To Pre-grasp failed!",
        );
        this.subscribeToActionResult(
            showTabletActionName,
            this.showTabletResultCallback,
            "Show Tablet canceled!",
            "Show Tablet succeeded!",
            "Show Tablet failed!",
        );
        this.createTrajectoryClient();
        this.createMoveBaseClient();
        this.createMoveToPregraspClient();
        this.createShowTabletClient();
        this.createCmdVelTopic();
        this.createSwitchToNavigationService();
        this.createSwitchToPositionService();
        this.createRealsenseDepthSensingService();
        this.createGripperDepthSensingService();
        this.createExpandedGripperService();
        this.createRealsenseShowBodyPoseService();
        this.createComputeBodyPoseService();
        this.createRunStopService();
        this.createRobotFrameTFClient();
        this.createMapFrameTFClient();
        this.subscribeToHeadTiltTF();
        this.subscribeToMapTF();
        this.createTextToSpeechTopic();
        this.createHomeTheRobotService();

        return Promise.resolve();
    }

    closeROSConnection() {
        this.subscriptions.forEach((topic) => {
            topic.unsubscribe();
        });
        this.ros.close();
    }

    isROSConnected() {
        return this.ros.isConnected;
    }

    subscribeToJointState() {
        const jointStateTopic: ROSLIB.Topic<ROSJointState> = new ROSLIB.Topic({
            ros: this.ros,
            name: "/stretch/joint_states",
            messageType: "sensor_msgs/msg/JointState",
        });
        this.subscriptions.push(jointStateTopic);

        jointStateTopic.subscribe((msg: ROSJointState) => {
            this.jointState = msg;
            let robotPose: RobotPose = rosJointStatetoRobotPose(
                this.jointState,
            );
            let jointValues: ValidJointStateDict = {};
            let effortValues: ValidJointStateDict = {};
            this.jointState.name.forEach((name?: ValidJoints) => {
                let inLimits = this.inJointLimits(name);
                let collision = this.inCollision(name);
                if (inLimits) jointValues[name!] = inLimits;
                if (collision) effortValues[name!] = collision;
            });

            if (this.jointStateCallback)
                this.jointStateCallback(robotPose, jointValues, effortValues);
        });
    }

    subscribeToJointLimits() {
        const jointLimitsTopic: ROSLIB.Topic<ROSJointState> = new ROSLIB.Topic({
            ros: this.ros,
            name: "/joint_limits",
            messageType: "sensor_msgs/msg/JointState",
        });
        this.subscriptions.push(jointLimitsTopic);

        jointLimitsTopic.subscribe((msg: ROSJointState) => {
            msg.name.forEach((name: string, idx: number) => {
                console.log(
                    "Got joint limit for",
                    name,
                    msg.position[idx],
                    msg.velocity[idx],
                );
                if (name == "joint_arm") name = "wrist_extension";
                this.jointLimits[name] = [msg.position[idx], msg.velocity[idx]];
            });
        });
    }

    subscribeToBatteryState() {
        const batteryStateTopic: ROSLIB.Topic<ROSBatteryState> =
            new ROSLIB.Topic({
                ros: this.ros,
                name: "/battery",
                messageType: "sensor_msgs/msg/BatteryState",
            });
        this.subscriptions.push(batteryStateTopic);

        batteryStateTopic.subscribe((msg: ROSBatteryState) => {
            if (this.batteryStateCallback) this.batteryStateCallback(msg);
        });
    }

    subscribeToMode() {
        const modeTopic: ROSLIB.Topic = new ROSLIB.Topic({
            ros: this.ros,
            name: "/mode",
            messageType: "std_msgs/msg/String",
        });
        this.subscriptions.push(modeTopic);

        modeTopic.subscribe((msg) => {
            if (this.modeCallback) this.modeCallback(msg.data);
        });
    }

    subscribeToIsHomed() {
        const isHomedTopic: ROSLIB.Topic = new ROSLIB.Topic({
            ros: this.ros,
            name: "/is_homed",
            messageType: "std_msgs/msg/Bool",
        });
        this.subscriptions.push(isHomedTopic);

        isHomedTopic.subscribe((msg) => {
            if (this.isHomedCallback) this.isHomedCallback(msg.data);
        });
    }

    subscribeToIsRunStopped() {
        let topic: ROSLIB.Topic = new ROSLIB.Topic({
            ros: this.ros,
            name: "is_runstopped",
            messageType: "std_msgs/msg/Bool",
        });
        this.subscriptions.push(topic);

        topic.subscribe((msg) => {
            if (this.isRunStoppedCallback) this.isRunStoppedCallback(msg.data);
        });
    }

    subscribeToVideo(props: VideoProps) {
        let topic: ROSLIB.Topic<ROSCompressedImage> = new ROSLIB.Topic({
            ros: this.ros,
            name: props.topicName,
            messageType: "sensor_msgs/CompressedImage",
        });
        topic.subscribe(props.callback);
        this.subscriptions.push(topic);
    }

    getHasBetaTeleopKit() {
        this.hasBetaTeleopKitParam = new ROSLIB.Param({
            ros: this.ros,
            name: "/configure_video_streams_gripper:has_beta_teleop_kit",
        });

        this.hasBetaTeleopKitParam.get((value: boolean) => {
            console.log("has beta teleop kit: ", value);
            if (this.hasBetaTeleopKitCallback)
                this.hasBetaTeleopKitCallback(value);
        });
    }

    getStretchTool() {
        console.log("Getting stretch tool", this.ros.isConnected);
        // NOTE: This information can also come from the /tool topic.
        // However, we only need it once, so opt for a parameter.
        this.stretchToolParam = new ROSLIB.Param({
            ros: this.ros,
            name: "/configure_video_streams_gripper:stretch_tool",
        });

        this.stretchToolParam.get((value: string) => {
            console.log("stretch tool: ", value);
            if (value === "eoa_wrist_dw3_tool_tablet_12in") {
                this.subscribeToTabletTF();
            } else if (
                [
                    "eoa_wrist_dw3_tool_sg3",
                    "tool_stretch_dex_wrist",
                    "tool_stretch_gripper",
                ].includes(value)
            ) {
                this.subscribeToGripperFingerTF();
            } else {
                this.subscribeToWristYawTF();
            }
            if (this.stretchToolCallback) this.stretchToolCallback(value);
        });
    }

    getOccupancyGrid() {
        let getMapService = new ROSLIB.Service({
            ros: this.ros,
            name: "/map_server/map",
            serviceType: "nav2_msgs/srv/GetMap",
        });

        var request = new ROSLIB.ServiceRequest({});
        getMapService?.callService(
            request,
            (response: { map: ROSOccupancyGrid }) => {
                if (this.occupancyGridCallback)
                    this.occupancyGridCallback(response.map);
            },
        );
    }

    getJointLimits() {
        console.log("Getting joint limits");
        let getJointLimitsService = new ROSLIB.Service({
            ros: this.ros,
            name: "/get_joint_states",
            serviceType: "std_srvs/Trigger",
        });

        var request = new ROSLIB.ServiceRequest({});
        getJointLimitsService.callService(
            request,
            () => {
                console.log("Got joint limits service succeeded");
            },
            (error) => {
                console.log("Got joint limits service failed", error);
            },
        );
    }

    subscribeToActionResult(
        actionName: string,
        callback?: (goalState: ActionState) => void,
        cancelMsg?: string,
        successMsg?: string,
        failureMsg?: string,
    ) {
        // Get the messages
        if (!cancelMsg) {
            cancelMsg = "Action " + actionName + "canceled!";
        }
        if (!successMsg) {
            successMsg = "Action " + actionName + "succeeded!";
        }
        if (!failureMsg) {
            failureMsg = "Action " + actionName + "failed!";
        }

        // Create the topic
        let topic: ROSLIB.Topic<ActionStatusList> = new ROSLIB.Topic({
            ros: this.ros,
            name: actionName + "/_action/status",
            messageType: "action_msgs/msg/GoalStatusArray",
        });
        this.subscriptions.push(topic);

        // Subscribe to the topic
        topic.subscribe((msg: ActionStatusList) => {
            console.log("Got action status msg", msg);
            let status = msg.status_list.pop()?.status;
            console.log("For action ", actionName, "got status ", status);
            if (callback) {
                if (status == 5)
                    callback({
                        state: cancelMsg,
                        alert_type: "error",
                    });
                else if (status == 4)
                    callback({
                        state: successMsg,
                        alert_type: "success",
                    });
                else if (status == 6)
                    callback({
                        state: failureMsg,
                        alert_type: "error",
                    });
            }
        });
    }

    createTrajectoryClient() {
        this.trajectoryClient = new ROSLIB.ActionHandle({
            ros: this.ros,
            name: "/stretch_controller/follow_joint_trajectory",
            actionType: "control_msgs/action/FollowJointTrajectory",
        });
    }

    createMoveBaseClient() {
        this.moveBaseClient = new ROSLIB.ActionHandle({
            ros: this.ros,
            name: moveBaseActionName,
            actionType: "nav2_msgs/action/NavigateToPose",
            // timeout: 100
        });
    }

    createMoveToPregraspClient() {
        this.moveToPregraspClient = new ROSLIB.ActionHandle({
            ros: this.ros,
            name: moveToPregraspActionName,
            actionType: "stretch_web_teleop/action/MoveToPregrasp",
        });
    }

    createShowTabletClient() {
        this.showTabletClient = new ROSLIB.ActionHandle({
            ros: this.ros,
            name: showTabletActionName,
            actionType: "stretch_show_tablet_interfaces/action/ShowTablet",
        });
    }

    createCmdVelTopic() {
        this.cmdVelTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: "/stretch/cmd_vel",
            messageType: "geometry_msgs/Twist",
        });
    }

    createTextToSpeechTopic() {
        this.textToSpeechTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: "/text_to_speech",
            messageType: "stretch_web_teleop/msg/TextToSpeech",
        });
    }

    createSwitchToNavigationService() {
        this.switchToNavigationService = new ROSLIB.Service({
            ros: this.ros,
            name: "/switch_to_navigation_mode",
            serviceType: "std_srvs/Trigger",
        });
    }

    createSwitchToPositionService() {
        this.switchToPositionService = new ROSLIB.Service({
            ros: this.ros,
            name: "/switch_to_position_mode",
            serviceType: "std_srvs/Trigger",
        });
    }

    createHomeTheRobotService() {
        this.homeTheRobotService = new ROSLIB.Service({
            ros: this.ros,
            name: "/home_the_robot",
            serviceType: "std_srvs/Trigger",
        });
    }

    createRealsenseDepthSensingService() {
        this.setRealsenseDepthSensingService = new ROSLIB.Service({
            ros: this.ros,
            name: "/realsense_depth_ar",
            serviceType: "std_srvs/srv/SetBool",
        });
    }

    createGripperDepthSensingService() {
        this.setGripperDepthSensingService = new ROSLIB.Service({
            ros: this.ros,
            name: "/gripper_depth_ar",
            serviceType: "std_srvs/srv/SetBool",
        });
    }

    createExpandedGripperService() {
        this.setExpandedGripperService = new ROSLIB.Service({
            ros: this.ros,
            name: "/expanded_gripper",
            serviceType: "std_srvs/srv/SetBool",
        });
    }

    createRealsenseShowBodyPoseService() {
        this.setRealsenseShowBodyPoseService = new ROSLIB.Service({
            ros: this.ros,
            name: "/realsense_body_pose_ar",
            serviceType: "std_srvs/srv/SetBool",
        });
    }

    createComputeBodyPoseService() {
        this.setComputeBodyPoseService = new ROSLIB.Service({
            ros: this.ros,
            name: "/toggle_body_pose_estimator",
            serviceType: "std_srvs/srv/SetBool",
        });
    }

    createRunStopService() {
        this.setRunStopService = new ROSLIB.Service({
            ros: this.ros,
            name: "/runstop",
            serviceType: "std_srvs/srv/SetBool",
        });
    }

    createRobotFrameTFClient() {
        this.robotFrameTfClient = new ROSLIB.TFClient({
            ros: this.ros,
            fixedFrame: "base_link",
            angularThres: 0.001,
            transThres: 0.001,
            rate: 10,
        });
    }

    createMapFrameTFClient() {
        this.mapFrameTfClient = new ROSLIB.TFClient({
            ros: this.ros,
            fixedFrame: "map",
            angularThres: 0.001,
            transThres: 0.001,
            rate: 10,
        });
    }

    subscribeToGripperFingerTF() {
        this.robotFrameTfClient?.subscribe(
            "link_gripper_finger_left",
            (transform) => {
                this.linkGripperFingerLeftTF = transform;
            },
        );
    }

    subscribeToTabletTF() {
        this.robotFrameTfClient?.subscribe(
            "link_DW3_tablet_12in",
            (transform) => {
                this.linkTabletTF = transform;
            },
        );
    }

    subscribeToWristYawTF() {
        this.robotFrameTfClient?.subscribe("link_wrist_yaw", (transform) => {
            this.linkWristYawTF = transform;
        });
    }

    subscribeToHeadTiltTF() {
        this.robotFrameTfClient?.subscribe("link_head_tilt", (transform) => {
            this.linkHeadTiltTF = transform;
        });
    }

    subscribeToMapTF() {
        this.mapFrameTfClient?.subscribe("base_link", (transform) => {
            if (this.amclPoseCallback) this.amclPoseCallback(transform);
        });
    }

    setRealsenseDepthSensing(toggle: boolean) {
        var request = new ROSLIB.ServiceRequest({ data: toggle });
        this.setRealsenseDepthSensingService?.callService(
            request,
            (response: boolean) => {
                response
                    ? console.log(
                          "Successfully set realsense depth sensing to",
                          toggle,
                      )
                    : console.log(
                          "Failed to set realsense depth sensing to",
                          toggle,
                      );
            },
        );
    }

    setGripperDepthSensing(toggle: boolean) {
        var request = new ROSLIB.ServiceRequest({ data: toggle });
        this.setGripperDepthSensingService?.callService(
            request,
            (response: boolean) => {
                response
                    ? console.log(
                          "Successfully set gripper depth sensing to",
                          toggle,
                      )
                    : console.log(
                          "Failed to set gripper depth sensing to",
                          toggle,
                      );
            },
        );
    }

    setExpandedGripper(toggle: boolean) {
        var request = new ROSLIB.ServiceRequest({ data: toggle });
        this.setExpandedGripperService?.callService(
            request,
            (response: boolean) => {
                response
                    ? console.log(
                          "Successfully set expanded gripper to",
                          toggle,
                      )
                    : console.log("Failed to set expanded gripper to", toggle);
            },
        );
    }

    setRealsenseShowBodyPose(toggle: boolean) {
        var request = new ROSLIB.ServiceRequest({ data: toggle });
        this.setRealsenseShowBodyPoseService?.callService(
            request,
            (response: boolean) => {
                response
                    ? console.log(
                          "Successfully set realsense depth sensing to",
                          toggle,
                      )
                    : console.log(
                          "Failed to set realsense depth sensing to",
                          toggle,
                      );
            },
        );
    }

    setComputeBodyPose(toggle: boolean) {
        var request = new ROSLIB.ServiceRequest({ data: toggle });
        this.setComputeBodyPoseService?.callService(
            request,
            (response: boolean) => {
                response
                    ? console.log(
                          "Successfully set compute body pose to",
                          toggle,
                      )
                    : console.log("Failed to set compute body pose to", toggle);
            },
        );
    }

    setRunStop(toggle: boolean) {
        var request = new ROSLIB.ServiceRequest({ data: toggle });
        this.setRunStopService?.callService(request, (response: boolean) => {});
    }

    /**
     * In navigation mode, you can send position commands to the arm and
     * velocity commands to the base.
     */
    switchToNavigationMode() {
        var request = new ROSLIB.ServiceRequest({});
        if (robotMode !== "navigation") {
            this.switchToNavigationService!.callService(request, () => {
                robotMode = "navigation";
                console.log("Switched to navigation mode");
            });
        }
    }

    /**
     * In position mode, you can send position commands to the arm and
     * position commands to the base. This mode is no longer used in the
     * web interface.
     */
    switchToPositionMode = () => {
        var request = new ROSLIB.ServiceRequest({});
        if (robotMode !== "position") {
            this.switchToPositionService!.callService(request, () => {
                robotMode = "position";
                console.log("Switched to position mode");
            });
        }
    };

    /**
     * Ask the robot to home itself.
     */
    homeTheRobot() {
        var request = new ROSLIB.ServiceRequest({});
        this.homeTheRobotService!.callService(request, () => {
            robotMode = "unknown"; // returns to whatever mode the robot was in before this service was called
            console.log("Homing complete");
        });
    }

    executeBaseVelocity = (props: { linVel: number; angVel: number }): void => {
        this.switchToNavigationMode();
        this.stopExecution();
        let twist = new ROSLIB.Message({
            linear: {
                x: props.linVel,
                y: 0,
                z: 0,
            },
            angular: {
                x: 0,
                y: 0,
                z: props.angVel,
            },
        });
        if (!this.cmdVelTopic) throw "cmdVelTopic is undefined";
        console.log("Publishing base velocity twist message");
        this.cmdVelTopic.publish(twist);
    };

    makeIncrementalMoveGoal(
        jointName: ValidJoints,
        jointValueInc: number,
    ): ROSLIB.Goal | undefined {
        if (!this.jointState) throw "jointState is undefined";
        let newJointValue = this.getJointValue(jointName);
        // Paper over Hello's fake joints
        if (
            jointName === "translate_mobile_base" ||
            jointName === "rotate_mobile_base"
        ) {
            // These imaginary joints are floating, always have 0 as their reference
            newJointValue = 0;
        }

        let collision = this.inCollision({
            jointStateMessage: this.jointState,
            jointName: jointName,
        });
        let collisionIndex = jointValueInc <= 0 ? 0 : 1;
        if (jointName === "joint_wrist_yaw") {
            collisionIndex = jointValueInc <= 0 ? 1 : 0;
        }
        // Negative joint increment is for lower/retract/wrist out
        // Positive joint increment is for lift/extend/wrist in
        let index = jointValueInc <= 0 ? 0 : 1;
        // If request to move the joint in the direction of collision, cancel movement
        if (collision[collisionIndex]) return;

        newJointValue = newJointValue + jointValueInc;

        // Make sure new joint value is within limits
        if (jointName in this.jointLimits) {
            let inLimits = this.inJointLimitsHelper(newJointValue, jointName);
            if (!inLimits) throw "invalid joint name";
            // console.log(newJointValue, this.jointLimits[jointName]![index], inLimits[index])
            if (!inLimits[index])
                newJointValue = this.jointLimits[jointName]![index];
        }

        let pose = { [jointName]: newJointValue };
        if (!this.trajectoryClient) throw "trajectoryClient is undefined";
        return this.makePoseGoal(pose);
    }

    makeMoveBaseGoal(pose: ROSPose) {
        if (!this.moveBaseClient) throw "moveBaseClient is undefined";

        let newGoal = new ROSLIB.ActionGoal({
            pose: {
                header: {
                    frame_id: "map",
                },
                pose: pose,
            },
        });

        return newGoal;
    }

    makeMoveToPregraspGoal(
        scaled_x: number,
        scaled_y: number,
        horizontal: boolean,
    ) {
        if (!this.moveToPregraspClient)
            throw "moveToPregraspClient is undefined";

        let newGoal = new ROSLIB.ActionGoal({
            scaled_u: scaled_x,
            scaled_v: scaled_y,
            pregrasp_direction: horizontal ? 1 : 2,
        });

        return newGoal;
    }

    makeShowTabletGoal() {
        if (!this.showTabletClient) throw "showTabletClient is undefined";

        let newGoal = new ROSLIB.ActionGoal({
            // TODO: Update once we have a finalized interface!
            number_of_pose_estimates: 10,
        });

        return newGoal;
    }

    makePoseGoal(pose: RobotPose) {
        let jointNames: ValidJoints[] = [];
        let jointPositions: number[] = [];
        for (let key in pose) {
            jointNames.push(key as ValidJoints);
            jointPositions.push(pose[key as ValidJoints]!);
        }

        console.log(this.trajectoryClient);

        if (!this.trajectoryClient) throw "trajectoryClient is undefined";
        let newGoal = new ROSLIB.ActionGoal({
            trajectory: {
                header: {
                    stamp: {
                        secs: 0,
                        nsecs: 0,
                    },
                },
                joint_names: jointNames,
                points: [
                    {
                        positions: jointPositions,
                        // The following might causing the jumpiness in continuous motions
                        time_from_start: {
                            secs: 1,
                            nsecs: 0,
                        },
                    },
                ],
            },
        });

        return newGoal;
    }

    makePoseGoals(poses: RobotPose[]) {
        let jointNames: ValidJoints[] = [];
        for (let key in poses[0]) {
            jointNames.push(key as ValidJoints);
        }

        let points: any = [];
        let jointPositions: number[] = [];
        poses.forEach((pose, index) => {
            jointPositions = [];
            for (let key in pose) {
                jointPositions.push(pose[key as ValidJoints]!);
            }
            points.push({
                positions: jointPositions,
                time_from_start: {
                    secs: 10,
                    nsecs: 0,
                },
            });
        });

        if (!this.trajectoryClient) throw "trajectoryClient is undefined";
        let newGoal = new ROSLIB.ActionGoal({
            trajectory: {
                header: {
                    stamp: {
                        secs: 0,
                        nsecs: 0,
                    },
                },
                joint_names: jointNames,
                points: points,
            },
        });

        return newGoal;
    }

    executePoseGoal(pose: RobotPose) {
        this.switchToNavigationMode();
        this.stopExecution();
        this.poseGoal = this.makePoseGoal(pose);
        this.trajectoryClient.createClient(this.poseGoal);
    }

    async executePoseGoals(poses: RobotPose[], index: number) {
        this.switchToNavigationMode();
        this.stopExecution();
        this.poseGoal = this.makePoseGoals(poses);
        this.trajectoryClient.createClient(this.poseGoal);
    }

    executeMoveBaseGoal(pose: ROSPose) {
        this.switchToNavigationMode();
        // this.stopExecution()
        this.moveBaseGoal = this.makeMoveBaseGoal(pose);
        this.moveBaseClient.createClient(this.moveBaseGoal);

        // An autonomous client may change the robot's mode.
        robotMode = "unknown";
    }

    executeIncrementalMove(jointName: ValidJoints, increment: number) {
        this.switchToNavigationMode();
        this.stopAutonomousClients();
        this.poseGoal = this.makeIncrementalMoveGoal(jointName, increment);
        this.trajectoryClient.createClient(this.poseGoal);
    }

    // NOTE: When we undo this temp fix (of not stopping the
    // trajectory client) we also need to undo it in FunctionProvider.jsx
    // `stopCurrentAction()`.
    // However, we should consider not stopping the trajectory client here,
    // regardless, because:
    //   (1) there is a race condition where ROS can receive the cancellation
    //       request *after* the new goal, and thus cancel the new goal (e.g.,
    //       this occurs often when toggling the tablet between portrait and
    //       landscape mode);
    //   (2) the trajectory client smoothly interpolates between goals anyway,
    //       so there is no need to stop it.
    // If we premanently don't stop the trajectory client here, then
    // `stopExecution` should just be replaced with `stopAutonomousClients`.
    stopExecution(stop_trajectory_client: boolean = false) {
        if (stop_trajectory_client) this.stopTrajectoryClient();
        this.stopAutonomousClients();
    }

    stopAutonomousClients() {
        this.stopMoveBaseClient();
        this.stopMoveToPregraspClient();
        this.stopShowTabletClient();
    }

    stopTrajectoryClient() {
        if (!this.trajectoryClient) throw "trajectoryClient is undefined";
        if (this.poseGoal) {
            this.trajectoryClient.cancelGoal();
            // this.poseGoal.cancel()
            this.poseGoal = undefined;
        }
    }

    stopMoveBaseClient() {
        if (!this.moveBaseClient) throw "moveBaseClient is undefined";
        if (this.moveBaseGoal) {
            this.moveBaseClient.cancelGoal();
            // this.moveBaseGoal.cancel()
            this.moveBaseGoal = undefined;
        }
    }

    /**
     * @param x The x coordinate of the click on the Realsense camera
     * @param y The y coordinate of the click on the Realsense camera
     * @param horizontal Whether the gripper should orient horizontally or vertically.
     */
    executeMoveToPregraspGoal(
        scaled_x?: number,
        scaled_y?: number,
        horizontal?: boolean,
    ) {
        if (
            scaled_x === undefined ||
            scaled_y === undefined ||
            horizontal === undefined
        ) {
            return;
        }
        console.log(
            "Got move to pregrasp goal",
            scaled_x,
            scaled_y,
            horizontal,
        );
        this.moveToPregraspGoal = this.makeMoveToPregraspGoal(
            scaled_x,
            scaled_y,
            horizontal,
        );
        this.moveToPregraspClient.createClient(this.moveToPregraspGoal);

        // An autonomous client may change the robot's mode.
        robotMode = "unknown";
    }

    stopMoveToPregraspClient() {
        if (!this.moveToPregraspClient)
            throw "moveToPregraspClient is undefined";
        if (this.moveToPregraspGoal) {
            this.moveToPregraspClient.cancelGoal();
            this.moveToPregraspGoal = undefined;
        }
    }

    executeShowTabletGoal() {
        this.showTabletGoal = this.makeShowTabletGoal();
        this.showTabletClient.createClient(this.showTabletGoal);

        // An autonomous client may change the robot's mode.
        robotMode = "unknown";
    }

    stopShowTabletClient() {
        if (!this.showTabletClient) throw "showTabletClient is undefined";
        if (this.showTabletGoal) {
            this.showTabletClient.cancelGoal();
            this.showTabletGoal = undefined;
        }
    }

    setPanTiltFollowGripper(followGripper: boolean) {
        if (this.lookAtGripperInterval && followGripper) return;

        if (followGripper) {
            let panOffset = 0;
            let tiltOffset = 0;
            let lookIfReadyAndRepeat = () => {
                if (
                    (this.linkGripperFingerLeftTF ||
                        this.linkTabletTF ||
                        this.linkWristYawTF) &&
                    this.linkHeadTiltTF
                ) {
                    this.lookAtGripper(panOffset, tiltOffset);
                }
                this.lookAtGripperInterval = window.setTimeout(
                    lookIfReadyAndRepeat,
                    500,
                );
            };
            lookIfReadyAndRepeat();
        } else {
            this.stopExecution();
            clearTimeout(this.lookAtGripperInterval);
            this.lookAtGripperInterval = undefined;
        }
    }

    lookAtGripper(panOffset: number, tiltOffset: number) {
        // If there is a gripper, follow its TF frame. Else, if there is a tablet, follow its TF frame.
        // Else, follow the quick connect TF frame.
        let transform: ROSLIB.Transform | undefined =
            this.linkGripperFingerLeftTF ||
            this.linkTabletTF ||
            this.linkWristYawTF;
        if (!transform)
            throw "linkGripperFingerLeftTF, linkTabletTF, and linkWristYawTF are all undefined";
        if (!this.linkHeadTiltTF) throw "linkHeadTiltTF is undefined";
        let posDifference = {
            x: transform.translation.x - this.linkHeadTiltTF.translation.x,
            y: transform.translation.y - this.linkHeadTiltTF.translation.y,
            z: transform.translation.z - this.linkHeadTiltTF.translation.z,
        };

        // Normalize posDifference
        const scalar = Math.sqrt(
            posDifference.x ** 2 + posDifference.y ** 2 + posDifference.z ** 2,
        );
        posDifference.x /= scalar;
        posDifference.y /= scalar;
        posDifference.z /= scalar;

        const pan = Math.atan2(posDifference.y, posDifference.x) + panOffset;
        const tilt = Math.atan2(posDifference.z, -posDifference.y) + tiltOffset;

        // Goals really close to current state cause some whiplash in these joints in simulation.
        // Ignoring small goals is a temporary fix
        if (!this.jointState) throw "jointState is undefined";
        let panDiff = Math.abs(this.getJointValue("joint_head_pan") - pan);
        let tiltDiff = Math.abs(this.getJointValue("joint_head_tilt") - tilt);
        if (panDiff < 0.02 && tiltDiff < 0.02) {
            return;
        }

        this.executePoseGoal({
            joint_head_pan: pan + panOffset,
            joint_head_tilt: tilt + tiltOffset,
        });
    }

    getJointValue(jointName: ValidJoints): number {
        // Paper over Hello's fake joint implementation
        if (jointName === "joint_arm" || jointName === "wrist_extension") {
            return (
                this.getJointValue("joint_arm_l0") +
                this.getJointValue("joint_arm_l1") +
                this.getJointValue("joint_arm_l2") +
                this.getJointValue("joint_arm_l3")
            );
        } else if (
            jointName === "translate_mobile_base" ||
            jointName === "rotate_mobile_base"
        ) {
            return 0;
        }

        let jointIndex = this.jointState.name.indexOf(jointName);
        return this.jointState.position[jointIndex];
    }

    inJointLimits(jointName: ValidJoints) {
        let jointValue = this.getJointValue(jointName);
        return this.inJointLimitsHelper(jointValue, jointName);
    }

    inJointLimitsHelper(jointValue: number, jointName: ValidJoints) {
        let jointLimits = this.jointLimits[jointName];
        if (!jointLimits) return;

        var eps = 0.03;
        let inLimits: [boolean, boolean] = [true, true];
        inLimits[0] = jointValue - eps >= jointLimits[0]; // Lower joint limit
        inLimits[1] = jointValue + eps <= jointLimits[1]; // Upper joint limit
        return inLimits;
    }

    inCollision(jointName: ValidJoints) {
        let inCollision: [boolean, boolean] = [false, false];
        // TODO: This formulation needs to be changed, because effort values are
        // robot-specific and change based on whether the robot is plugged in or not,
        // mechanical factors (e.g., an old cable), etc. Thus, a single threshold
        // will not work across all robots.
        const MAX_EFFORTS: { [key in ValidJoints]?: [number, number] } = {
            joint_head_tilt: [-50, 50],
            joint_head_pan: [-50, 50],
            wrist_extension: [-40, 40],
            joint_lift: [0, 70],
            // "joint_wrist_yaw": [-10, 10],
            // "joint_wrist_pitch": [-10, 10],
            // "joint_wrist_roll": [-10, 10],
        };

        if (!(jointName in MAX_EFFORTS)) return inCollision;

        let jointIndex = this.jointState.name.indexOf(jointName);
        // In collision if joint is applying more than 50% effort when moving downward/inward/backward
        inCollision[0] =
            this.jointState.effort[jointIndex] < MAX_EFFORTS[jointName]![0];
        // In collision if joint is applying more than 50% effort when moving upward/outward/forward
        inCollision[1] =
            this.jointState.effort[jointIndex] > MAX_EFFORTS[jointName]![1];

        return inCollision;
    }

    playTextToSpeech(
        text: string,
        override_behavior: number = 0,
        is_slow: boolean = false,
    ) {
        if (!this.textToSpeechTopic) throw "textToSpeechTopic is undefined";
        if (override_behavior != 0 && override_behavior != 1) {
            console.log(
                "override behavior must be 0 (queue) or 1 (interrupt). Setting to 0.",
            );
            override_behavior = 0;
        }
        let message = new ROSLIB.Message({
            text: text,
            is_slow: is_slow,
            override_behavior: override_behavior,
        });
        this.textToSpeechTopic.publish(message);
    }

    stopTextToSpeech() {
        // Send an empty string and override behavior 1 to interrupt the current speech
        this.playTextToSpeech("", 1);
    }

    /**
     * Copied from https://github.com/hello-vinitha/roslibjs/pull/1 and
     * https://github.com/RobotWebTools/roslibjs/pull/760 , included here for
     * backwards compatibility.
     */
    getPublishers(
        topic: string,
        callback: (publishers: string[]) => void,
        failedCallback: (message: any) => void,
    ) {
        var publishersClient = new ROSLIB.Service({
            ros: this.ros,
            name: "/rosapi/publishers",
            serviceType: "rosapi_msgs/srv/Publishers",
        });

        var request = new ROSLIB.ServiceRequest({
            topic: topic,
        });
        if (typeof failedCallback === "function") {
            publishersClient.callService(
                request,
                function (result: any) {
                    callback(result.publishers);
                },
                failedCallback,
            );
        } else {
            publishersClient.callService(request, function (result) {
                callback(result.publishers);
            });
        }
    }
}
