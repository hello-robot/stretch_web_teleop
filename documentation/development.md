# Developing on the Web Interface

This document captures the organization of Web Teleop's codebase and best practices for adding new development. It takes the example of adding a "homing" button to the web interface. Homing is a 30 second sequence the robot must go through every time it wakes up to ascertain its zero positions. This document is meant to be read from top to bottom.

## `src/shared`

Within the `src/shared` folder, there is:

 - `commands.tsx`
 - `remoterobot.tsx`
 - `util.tsx`
 - `webrtcconnections.tsx`

### `commands.tsx`

This file defines the common language between the operator and robot browsers. It contains the set of "commands" the interface can take. For example, you can ask Nav2 to navigate the robot using the `MoveBaseCommand`, or you can ask the driver for the battery's current voltage using `GetBatteryVoltageCommand`. Every action taken in the web interface is converted into a command and transmitted through WebRTC's data channel to the robot browser, where it interpreted and further transmitted to Stretch's ROS2 packages.

Adding new capabilities to the web interface will involve creating new commands in this file. Let's take the `HomeTheRobotCommand` for example. We define an interface like so:

```js
export interface HomeTheRobotCommand {
    type: "homeTheRobot";
}
```

Then, we add `HomeTheRobotCommand` as a `cmd` type at the top of the file:

```js
export type cmd =
    ...
    | HomeTheRobotCommand;
```

### `remoterobot.tsx`

This file wraps the logic for receiving and transmitting commands, sensor streams, etc. over the WebRTC channels in a nice API for the operator browser. The interface creates a single instance of `RemoteRobot` and uses it for the duration of the session that it is connected to the robot. On the other end, the robot browser is listening for commands from / transmitting data to `RemoteRobot`. The robot-side logic can be found in `src/pages/robot/tsx/index.tsx`, which is described [at this section](#indextsx).

Adding new capabilities to the web interface will involve adding functionality to the `RemoteRobot` class. Let's take homing the robot for example. We add a method to the class like so:

```js
homeTheRobot() {
    let cmd: HomeTheRobotCommand = {
        type: "homeTheRobot",
    };
    this.robotChannel(cmd);
}
```

At the top of the file, be sure to import `HomeTheRobotCommand` from [`commands.tsx`](#commandstsx).

### `utils.tsx` and `webrtcconnections.tsx`

These files are not needed to add homing functionality. TODO: document these files elsewhere and link to it.

## `src/pages/robot`

The `src/pages/robot` folder contains the website being run on the robot browser on the Stretch robot. The website is organized into `css`, `html`, and `tsx` folders, and the `tsx` folder contains the important logic. It contains:

 - `index.tsx`
 - `robot.tsx`
 - `audiostreams.tsx`
 - `videostreams.tsx`

### `index.tsx`

The logic for the robot browser starts executing here. We create an instance of `Robot` from the [`robot.tsx` file](#robottsx).

```js
export const robot = new Robot({
    batteryStateCallback: forwardBatteryState,
    ...
});
```

We create an instance of `WebRTCConnection` from the [`webrtcconnections.tsx` file](#utilstsx-and-webrtcconnectionstsx).

```js
connection = new WebRTCConnection({
    peerRole: "robot",
    onRobotConnectionStart: handleSessionStart,
    ...
});
```

We define a series of callbacks:

 - The robot gets `forwardBatteryState()`, `forwardStretchTool()`, and many other `forward<Something>()`, all of which forward information from the robot to the operator using the WebRTC connection.
 - The WebRTC connection gets `handleSessionStart()`, which handles setting up the WebRTC streams and channels, and `handleMessage()`, which receives the commands defined in [`command.tsx`](#commandstsx) and were sent by [`remoterobot.tsx`](#remoterobottsx). `handleMessage()` is a big switch statement which interprets the commands and calls into `robot`:
    ```js
    function handleMessage(message: WebRTCMessage) {
        switch (message.type) {
            case "driveBase":
                robot.executeBaseVelocity(message.modifier);
                break;
            case "incrementalMove":
                robot.executeIncrementalMove(message.jointName, message.increment);
                break;
            case "stopTrajectory":
                ...
        }
    }
    ```

Adding new capabilities to the web interface may involve interpreting new types of commands in `handleMessage()`. Let's take homing the robot for example. We add a switch case like so:

```js
case "homeTheRobot":
    robot.homeTheRobot();
    break;
```

We will define `robot.homeTheRobot()` in [`robot.tsx`](#robottsx). Adding your new capability may also involve writing a new `forward<Something>()` function.

Lastly, at the end of the `index.tsx` file, all of the videos streams are rendered to the robot browser (which is hidden because it is launched headless). This is a necessary hack to transmit the video streams over WebRTC's video channels.

### `robot.tsx`

The file wraps the logic for communicating with Stretch's ROS2 packages. The communication is done using the ROSLib library, which establishes a websocket connection between the robot browser and a rosbridge ROS2 package. The rosbridge is responsible for converting and relaying between the ROS2 topics, services, actions, parameters, etc. messages and the JSON objects which can be transmitted over websockets. The robot browser creates a single instance of `Robot` and uses it for the duration that it is alive (i.e. over multiple sessions with operators).

The `Robot` class is defined:

```js
export class Robot extends React.Component {
    private ros: ROSLIB.Ros;
    private moveBaseClient?: ROSLIB.ActionClient;
    ...

    constructor(props: {
        moveBaseResultCallback: (goalState: ActionState) => void;
        ...
    }) {
        super(props);
        this.moveBaseResultCallback = props.moveBaseResultCallback;
    }

    async onConnect() {
        this.moveBaseClient = new ROSLIB.ActionHandle({
            ros: this.ros,
            name: "/navigate_to_pose",
            actionType: "nav2_msgs/action/NavigateToPose",
            // timeout: 100
        });
        this.subscribeToActionResult(
            "/navigate_to_pose",
            this.moveBaseResultCallback,
            "Navigation canceled!",
            "Navigation succeeded!",
            "Navigation failed!",
        );
    }

    executeMoveBaseGoal(pose: ROSPose) {
        this.switchToNavigationMode();
        this.moveBaseClient.sendGoal(pose);
    }
}
```

The above code is greatly abridged, but it gives you the general idea. Create clients of type `ROSLIB.ActionClient` for each ROS action, subscribers of type `ROSLIB.Topic` for each topic, and callers of type `ROSLIB.Service` for each service. Then, connect them in the `onConnect()` method. Lastly, define execution logic in `execute<Something>()` methods. Let's take homing the robot for example. We add a caller for the `/home_the_robot` service and connect it like so:

```js
export class Robot extends React.Component {
    private homeTheRobotService?: ROSLIB.Service;
    ...

    async onConnect() {
        this.createHomeTheRobotService();
    }

    createHomeTheRobotService() {
        this.homeTheRobotService = new ROSLIB.Service({
            ros: this.ros,
            name: "/home_the_robot",
            serviceType: "std_srvs/Trigger",
        });
    }

    homeTheRobot() {
        var request = new ROSLIB.ServiceRequest({});
        this.homeTheRobotService!.callService(request, () => {
            console.log("Homing complete");
        });
    }
}
```

### `audiostreams.tsx` and `videostreams.tsx`

These files are not needed to add homing functionality. TODO: document these files elsewhere and link to it.

## `src/pages/operator`

The `src/pages/operator` folder contains the website being run on the operator browser on the user's device. The website is organized into `css`, `html`, `icons`, and `tsx` folders, and the `tsx` folder contains the important logic. It contains:

 - `index.tsx`
 - `Operator.tsx`
 - `MobileOperator.tsx`
 - `function_providers/`
 - other folders

### `index.tsx`

The logic for the operator interface starts executing here. We create an instance of `RemoteRobot` from the [`remoterobot.tsx` file](#remoterobottsx).

```js
remoteRobot = new RemoteRobot({
    robotChannel: (message: cmd) => connection.sendData(message),
});
remoteRobot.sensors.setBatteryFunctionProviderCallback(
    batteryVoltageFunctionProvider.updateVoltage,
);
```

We set up the "function providers" from the `src/pages/operator/tsx/function_providers` folder. Function providers connect the information from `RemoteRobot` with the UI components being rendered on the screen. Some function providers require a storage handler, which gives those components a way to persist data/preferences across sessions (e.g. if I change the layout, it should remember my preferred layout next time I launch the web interface).

```js
export var batteryVoltageFunctionProvider = new BatteryVoltageFunctionProvider();
export var textToSpeechFunctionProvider: TextToSpeechFunctionProvider;
...
const storageHandlerReadyCallback = () => {
    textToSpeechFunctionProvider = new TextToSpeechFunctionProvider(
        storageHandler,
    );
};
storageHandler = createStorageHandler(storageHandlerReadyCallback);
```

We create an instance of `WebRTCConnection` from the [`webrtcconnections.tsx` file](#utilstsx-and-webrtcconnectionstsx).

```js
connection = new WebRTCConnection({
    peerRole: "operator",
    onMessage: handleWebRTCMessage,
    ...
});
```

We define two callbacks for the WebRTC connection: `handleRemoteTrackAdded()`, which subscribes to the video/audio WebRTC channels so they can be rendered to the interface's camera feeds, and `handleWebRTCMessage()`, which receives messages sent by the robot browser. `handleWebRTCMessage()` is a big switch statement which interprets the commands and calls into `RemoteRobot`:

```js
function handleWebRTCMessage(message: WebRTCMessage | WebRTCMessage[]) {
    switch (message.type) {
        case "isRunStopped":
            remoteRobot.sensors.setRunStopState(message.enabled);
            break;
        case "amclPose":
            remoteRobot.setMapPose(message.message);
            break;
        case "moveBaseState":
            console.log("moveBaseState", message.message);
            underMapFunctionProvider.setMoveBaseState(message.message);
            break;
        ...
    }
}
```

Lastly, we render the interface:

```js
function renderOperator(storageHandler: StorageHandler) {
    const layout = storageHandler.loadCurrentLayoutOrDefault();

    !isMobile
        ? root.render(
              <Operator
                  remoteStreams={allRemoteStreams}
                  layout={layout}
                  storageHandler={storageHandler}
              />,
          )
        : root.render(
              <MobileOperator
                  remoteStreams={allRemoteStreams}
                  storageHandler={storageHandler}
              />,
          );
}
```

### `Operator.tsx`

