// Adapted from ros2djs and nav2djs

import React from "react";
import createjs from "createjs-module";
import { ROSOccupancyGrid, ROSPoint, ROSPose } from "shared/util";
import ROSLIB from "roslib";
import { MapFunctions } from "../layout_components/AutoNav";
import hexToRgbArray from "../utils/hex-to-rgb-array";

/**
 * OccupancyGrid is a React component that manages the display and interaction
 * with a ROS occupancy grid map using createjs for rendering. It provides
 * methods for drawing pose markers, navigation arrows, and handling user
 * interactions for navigation goals.
 */
export class OccupancyGrid extends React.Component {
    // The createjs stage (canvas root)
    private rootObject: createjs.Stage;
    // The origin of the map in ROS coordinates
    private origin?: ROSLIB.Pose;
    // The bitmap image of the occupancy grid
    private bitmap?: createjs.Bitmap;
    // Map dimensions in cells
    public width: number;
    public height: number;
    // Scaling factors for the map
    private scaleX?: number;
    private scaleY?: number;
    // The ROS occupancy grid data
    private map: ROSOccupancyGrid;
    // The createjs shape for the goal marker
    private goalMarker?: createjs.Shape;
    // Interval for checking if the goal is reached
    private getGoalReached?: NodeJS.Timeout;
    // List of saved pose markers (shapes and labels)
    private savedPoseMarkers: {
        circle: createjs.Shape;
        label: createjs.Text;
    }[];
    // List of saved pose marker labels
    private savedPoseMarkersLabels: string[];
    // Map functions for interacting with the robot and map
    private functs: MapFunctions;
    // The current goal position in ROS coordinates
    private goal_position?: ROSPoint;

    // Listeners for goal_position changes
    private goalPositionListeners: ((pos: ROSPoint | undefined) => void)[] = [];

    /**
     * Constructor initializes the occupancy grid, sets up the canvas, and enables mouse/touch events.
     */
    constructor(props: { functs: MapFunctions; rootObject: createjs.Stage }) {
        super(props);
        this.rootObject = props.rootObject;
        this.rootObject.enableMouseOver();
        createjs.Touch.enable(this.rootObject);
        this.width = 0;
        this.height = 0;
        this.map = props.functs.GetMap;
        this.functs = props.functs;
        this.savedPoseMarkers = [];
        this.savedPoseMarkersLabels = [];
        this.createOccupancyGridClient();
    }

    /**
     * Draws a saved pose marker (circle and label) at the given coordinates.
     * @param x X coordinate
     * @param y Y coordinate
     * @param color RGB color array
     * @param text Label text
     */
    drawSavedPoseMarker(x: number, y: number, color: number[], text: string) {
        var circle = new createjs.Shape();
        var radius = 30;

        var graphics = new createjs.Graphics();
        graphics.beginFill(
            createjs.Graphics.getRGB(color[0], color[1], color[2], 0.5),
        );
        graphics.drawCircle(0, 0, radius);

        createjs.Shape.call(circle, graphics);

        circle.x = x;
        circle.y = y;
        circle.scaleX = 1.0 / this.rootObject.scaleX;
        circle.scaleY = 1.0 / this.rootObject.scaleY;

        var label = new createjs.Text(text, "bold 40px Arial", "#ff7700");
        label.x = x;
        label.y = y - 10;
        label.textAlign = "center";
        label.scaleX = 1.0 / this.rootObject.scaleX;
        label.scaleY = 1.0 / this.rootObject.scaleY;
        label.textBaseline = "alphabetic";

        circle.on("mouseover", (event) => {
            label.visible = true;
        });
        circle.on("mouseout", (event) => {
            label.visible = false;
        });
        return { circle, label };
    }

    /**
     * Draws a navigation arrow at the given location, optionally pulsing.
     * @param pulse Whether the arrow should pulse
     * @param color RGB color array
     */
    drawNavigationArrow(pulse: boolean, color: number[]) {
        var arrow = new createjs.Shape();
        var size = 40;
        var strokeSize = 0;
        var strokeColor = createjs.Graphics.getRGB(
            color[0],
            color[1],
            color[2],
            0.85,
        );
        var fillColor = createjs.Graphics.getRGB(
            color[0],
            color[1],
            color[2],
            0.85,
        );

        // draw the arrow
        var graphics = new createjs.Graphics();

        // line width
        graphics.setStrokeStyle(strokeSize);
        graphics.moveTo(0.0, size / 1.5);
        graphics.beginStroke(strokeColor);
        graphics.beginFill(fillColor);
        graphics.lineTo(-size / 2.0, -size / 2.0);
        graphics.lineTo(size / 2.0, -size / 2.0);
        graphics.lineTo(0.0, size / 1.5);
        graphics.closePath();
        graphics.endFill();
        graphics.endStroke();

        // create the shape
        createjs.Shape.call(arrow, graphics);

        // check if we are pulsing
        if (pulse) {
            // have the model "pulse"
            var growCount = 0;
            var growing = true;
            createjs.Ticker.addEventListener("tick", () => {
                if (growing) {
                    arrow.scaleX *= 1.035;
                    arrow.scaleY *= 1.035;
                    growing = ++growCount < 10;
                } else {
                    arrow.scaleX /= 1.035;
                    arrow.scaleY /= 1.035;
                    growing = --growCount < 0;
                }
            });
        }
        return arrow;
    }

    /**
     * Creates the occupancy grid by drawing the map data onto a canvas.
     * It sets the origin, dimensions, and scaling factors for the map.
     */
    createOccupancyGrid() {

        // Create an internal drawing canvas for the occupancy grid image
        var canvas = document.createElement("canvas");
        // Get the 2D drawing context, with willReadFrequently for performance
        var context = canvas!.getContext("2d", { willReadFrequently: true });

        // If the map is not available, display a placeholder rectangle and error text
        if (!this.map) {
            var rect = new createjs.Shape();
            rect.graphics.beginStroke("#000000");
            rect.graphics.setStrokeStyle(3);
            rect.graphics.drawRect(0, 0, 300, 500);
            rect.graphics.endStroke();
            var text = new createjs.Text("Could not load map", "30px Arial");
            text.x = 20;
            text.y = 250;
            this.rootObject.addChild(rect);
            this.rootObject.addChild(text);
            return;
        }

        // Save the map origin (position and orientation) from ROS map metadata
        this.origin = new ROSLIB.Pose({
            position: this.map.info.origin.position,
            orientation: this.map.info.origin.orientation,
        });

        // Set the canvas size to match the map dimensions (in cells)
        this.width = this.map.info.width;
        this.height = this.map.info.height;
        canvas.width = this.width;
        canvas.height = this.height;

        // Create an ImageData object to hold the pixel data for the map
        var imageData = context!.createImageData(this.width, this.height);

        // Loop through each row of the map...
        for (var row = 0; row < this.height; row++) {
            // ...and for each column in the row
            for (var col = 0; col < this.width; col++) {

                // Calc index into the map data array.
                // NOTE: ROS maps are bottom-left origin.
                var mapI = col + (this.height - row - 1) * this.width;

                // Get the occupancy value for this cell
                var data = this.map.data[mapI];

                // Init RGB vars...
                var r: number;
                var g: number;
                var b: number;

                // Calc pixel color based on...
                if (data === 100) {
                    // ...occupied cells: rgb(71, 95, 111)
                    r = 71;
                    g = 95;
                    b = 111;
                } else if (data === 0) {
                    // ...free cells: rgb(241, 248, 253)
                    r = 241;
                    g = 248;
                    b = 253;
                } else {
                    // ...unknown cells: rgb(157, 197, 191)
                    r = 157;
                    g = 197;
                    b = 191;
                }

                // Calculate the index into the image data array (RGBA)
                var i = (col + row * this.width) * 4;

                // Set R, G, B channels to respective values, and alpha to 255 (opaque)
                imageData.data[i] = r;
                imageData.data[++i] = g;
                imageData.data[++i] = b;
                imageData.data[++i] = 255;
            }
        }

        // Draw the generated image data onto the canvas
        context!.putImageData(imageData, 0, 0);

        // Create a createjs.Bitmap from the canvas and add it to the stage
        this.bitmap = new createjs.Bitmap(canvas);
        this.rootObject.addChild(this.bitmap);

        // Set the scaling factors for converting between map and world coordinates
        this.scaleX = this.map.info.resolution;
        this.scaleY = this.map.info.resolution;
    }

    /**
     * Converts a ROS translation (Vector3) to global canvas coordinates.
     */
    rosToGlobal(translation: ROSLIB.Vector3) {
        var x =
            (this.width * this.scaleX! -
                (-translation.x +
                    this.width * this.scaleX! +
                    this.origin!.position.x)) /
            this.scaleX!;
        var y =
            (-translation.y +
                this.height * this.scaleY! +
                this.origin!.position.y) /
            this.scaleY!;
        return {
            x: x,
            y: y,
        };
    }

    /**
     * Converts a ROS quaternion to a global theta (angle in degrees).
     * See: https://github.com/RobotWebTools/ros2djs/blob/develop/src/Ros2D.js#L34C1-L44C3
     */
    rosQuaternionToGlobalTheta(orientation: ROSLIB.Quaternion) {
        // See https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Rotation_matrices
        // here we use [x y z] = R * [1 0 0]
        var w = orientation.w;
        var x = orientation.x;
        var y = orientation.y;
        var z = orientation.z;
        // Canvas rotation is clock wise and in degrees
        return (
            (-Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) *
                180.0) /
            Math.PI
        );
    }

    /**
     * Converts global canvas coordinates to ROS coordinates.
     */
    globalToRos(x: number, y: number) {
        var rosX = (x / 5) * this.scaleX! + this.origin!.position.x;
        var rosY =
            (this.height - y / 5) * this.scaleY! + this.origin!.position.y;
        return {
            x: rosX,
            y: rosY,
            z: 0,
        } as ROSPoint;
    }

    /**
     * Adds a marker for the robot's current pose and updates it periodically.
     */
    addCurrentPoseMarker() {
        const color = hexToRgbArray('#008AE5');
        var robotMarker = this.drawNavigationArrow(false, color);
        this.rootObject.addChild(robotMarker);

        const setPoseInterval = setInterval(() => {
            let pose = this.functs.GetPose();
            let globalCoord = this.rosToGlobal(pose.translation);
            robotMarker.x = globalCoord.x;
            robotMarker.y = globalCoord.y;
            let theta = this.rosQuaternionToGlobalTheta(pose.rotation);
            robotMarker.rotation = theta - 90.0;
            robotMarker.scaleX = 1.0 / this.rootObject.scaleX;
            robotMarker.scaleY = 1.0 / this.rootObject.scaleY;
            robotMarker.visible = true;
            this.rootObject.update();
        }, 1000);
    }

    /**
     * Displays or hides pose markers for saved navigation goals.
     * @param display Whether to display the markers
     * @param poses Array of ROS transforms for each pose
     * @param poseNames Array of pose names
     * @param poseTypes Array of pose types
     */
    public displayPoseMarkers(
        display: boolean,
        poses: ROSLIB.Transform[],
        poseNames: string[],
        poseTypes: string[],
    ) {
        if (!display) {
            this.savedPoseMarkers.forEach((marker) => {
                marker.circle.visible = false;
                marker.label.visible = false;
            });
        } else {
            // Re-draw or add pose markers
            poses.forEach((pose, index) => {
                // Recreate marker
                let globalCoord = this.rosToGlobal(pose.translation);
                let color = poseTypes[index] == "MAP" ? [0, 0, 255] : [255, 0, 0];
                var poseMarker = this.drawSavedPoseMarker(
                    globalCoord.x,
                    globalCoord.y,
                    color,
                    poseNames[index],
                );
                poseMarker.circle.visible = true;
                poseMarker.label.visible = false;

                var label_idx = this.savedPoseMarkersLabels.indexOf(
                    poseNames[index],
                );
                // If old pose marker label exists, overwrite marker
                if (label_idx !== -1) {
                    var oldPoseMarker = this.savedPoseMarkers[label_idx];
                    this.rootObject.removeChild(oldPoseMarker.circle);
                    this.rootObject.removeChild(oldPoseMarker.label);
                    this.savedPoseMarkers[label_idx] = poseMarker;
                    this.savedPoseMarkersLabels[label_idx] = poseNames[index];
                } else {
                    this.savedPoseMarkers.push(poseMarker);
                    this.savedPoseMarkersLabels.push(poseNames[index]);
                }

                this.rootObject.addChild(poseMarker.circle);
                this.rootObject.addChild(poseMarker.label);
            });
        }
        this.rootObject.update();
    }

    /**
     * Creates a goal marker at the given coordinates (in ROS or global space).
     * @param x X coordinate
     * @param y Y coordinate
     * @param ros Whether the coordinates are in ROS space
     */
    public createGoalMarker(x: number, y: number, ros: boolean) {
        const color = hexToRgbArray('#2EE4C8'); // Orange color for goal marker
        let globalCoord = { x: x, y: y };
        if (ros)
            globalCoord = this.rosToGlobal({
                x: x,
                y: y,
                z: 0,
            } as ROSLIB.Vector3);
        if (this.getGoalReached) clearInterval(this.getGoalReached);
        if (this.goalMarker) this.rootObject.removeChild(this.goalMarker);
        this.goalMarker = this.drawNavigationArrow(true, color);
        this.goalMarker.x = globalCoord.x;
        this.goalMarker.y = globalCoord.y;
        this.goalMarker.scaleX = 1.0 / this.rootObject.scaleX;
        this.goalMarker.scaleY = 1.0 / this.rootObject.scaleY;
        this.goalMarker.visible = true;
        this.rootObject.addChild(this.goalMarker);
        this.getGoalReached = setInterval(() => {
            if (this.functs.GoalReached()) {
                this.rootObject.removeChild(this.goalMarker!);
                clearInterval(this.getGoalReached);
            }
        }, 1000);
    }

    /**
     * Getter for the current goal_position.
     */
    goalPositionGet(): ROSPoint | undefined {
        return this.goal_position;
    }

    /**
     * Subscribe to changes in goal_position.
     * @param callback Function to call when goal_position changes
     */
    goalPositionSubscribe(callback: (pos: ROSPoint | undefined) => void): () => void {
        this.goalPositionListeners.push(callback);
        // Return an unsubscribe function
        // that's used in useEffect cleanup
        return () => {
            this.goalPositionListeners = this.goalPositionListeners.filter(cb => cb !== callback);
        };
    }

    /**
     * Setter for the current goal position.
     */
    goalPositionSet(pos: ROSPoint | undefined): void {
        this.goal_position = pos;
        this.goalPositionListeners.forEach(cb => cb(pos));
    }

    /**
     * Sends the robot to the current goal position, if set.
     */
    play() {
        if (this.goal_position) {
            this.functs.MoveBase({
                position: this.goal_position,
                orientation: { x: 0, y: 0, z: -0.45, w: 0.893 },
            } as ROSPose);
        }
        this.goalPositionSet(undefined);
        this.functs.SetSelectGoal(false);
    }

    /**
     * Removes the current goal marker from the map.
     */
    removeGoalMarker() {
        this.goalPositionSet(undefined);
        if (this.goalMarker) this.rootObject.removeChild(this.goalMarker);
    }

    /**
     * Initializes the occupancy grid, adds the robot pose marker, and sets up mouse event handlers.
     */
    createOccupancyGridClient() {
        this.createOccupancyGrid();

        if (!this.map) return;

        this.addCurrentPoseMarker();

        this.rootObject.on("mousedown", (event) => {
            if (!this.functs.SelectGoal()) return
            else {
                let evt = event as createjs.MouseEvent;
                this.goalPositionSet(this.globalToRos(evt.stageX, evt.stageY));
                this.createGoalMarker(evt.stageX / 5, evt.stageY / 5, false);
            }
        });
    }
}