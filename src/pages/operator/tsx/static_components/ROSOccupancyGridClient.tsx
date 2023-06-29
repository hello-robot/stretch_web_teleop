import React from "react";
import createjs, { Bitmap } from "createjs-module";
import { AMCLPose, ROSOccupancyGrid, ROSPoint, ROSPose } from "shared/util";
import ROSLIB from "roslib";

export class ROSOccupancyGridClient extends React.Component {
    private rootObject: createjs.Stage
    private origin?: ROSLIB.Pose
    private bitmap?: createjs.Bitmap
    public width: number
    public height: number
    private scaleX?: number
    private scaleY?: number
    private map: ROSOccupancyGrid
    private getPoseFn: () => ROSLIB.Transform
    private moveBaseFn: (pose: ROSPose) => void
    constructor(props: { 
        map: ROSOccupancyGrid,
        moveBaseFn: (pose: ROSPose) => void,
        getPoseFn: () => ROSLIB.Transform,
        rootObject: createjs.Stage 
    }) {
        super(props);
        this.rootObject = props.rootObject
        this.width = 0
        this.height = 0
        this.map = props.map
        this.getPoseFn = props.getPoseFn
        this.moveBaseFn = props.moveBaseFn
        this.createOccupancyGridClient()
    }

    NavigationArrow() {
        var arrow = new createjs.Shape()
        var size = 10;
        var strokeSize = 0;
        var strokeColor = createjs.Graphics.getRGB(0, 0, 0);
        var fillColor = createjs.Graphics.getRGB(255, 0, 0);
        var pulse = false;
        
        // draw the arrow
        var graphics = new createjs.Graphics();
        
        // line width
        graphics.setStrokeStyle(strokeSize);
        graphics.moveTo(0.0, size / 2.0);
        graphics.beginStroke(strokeColor);
        graphics.beginFill(fillColor);
        graphics.lineTo(-size / 2.0, -size / 2.0);
        graphics.lineTo(size / 2.0, -size / 2.0);
        graphics.lineTo(0.0, size / 2.0);
        graphics.closePath();
        graphics.endFill();
        graphics.endStroke();
        
        // create the shape
        createjs.Shape.call(arrow, graphics);

        // check if we are pulsing
        // if (pulse) {
        //     // have the model "pulse"
        //     var growCount = 0;
        //     var growing = true;
        //     createjs.Ticker.addEventListener('tick', function () {
        //         if (growing) {
        //             this.scaleX *= 1.035;
        //             this.scaleY *= 1.035;
        //             growing = (++growCount < 10);
        //         }
        //         else {
        //             this.scaleX /= 1.035;
        //             this.scaleY /= 1.035;
        //             growing = (--growCount < 0);
        //         }
        //     });
        // }
        return arrow
    };

    createOccupancyGrid() {
        // internal drawing canvas
        var canvas = document.createElement('canvas')
        var context = canvas!.getContext('2d');
        
        // save the metadata we need
        this.origin = new ROSLIB.Pose({
            position: this.map.info.origin.position,
            orientation: this.map.info.origin.orientation
        });
        // set the size
        this.width = this.map.info.width;
        this.height = this.map.info.height;
        canvas.width = this.width
        canvas.height = this.height
         
        var imageData = context!.createImageData(this.width, this.height);
        for (var row = 0; row < this.height; row++) {
            for (var col = 0; col < this.width; col++) {
                // determine the index into the map data
                var mapI = col + ((this.height - row - 1) * this.width);
                // determine the value
                var data = this.map.data[mapI];
                var val;
                if (data === 100) {
                    val = 0;
                }
                else if (data === 0) {
                    val = 255;
                }
                else {
                    val = 127;
                }
                // determine the index into the image data array
                var i = (col + (row * this.width)) * 4;
                // r
                imageData.data[i] = val;
                // g
                imageData.data[++i] = val;
                // b
                imageData.data[++i] = val;
                // a
                imageData.data[++i] = 255;
            }
        }
        context!.putImageData(imageData, 0, 0);

        // create the bitmap
        this.bitmap = new createjs.Bitmap(canvas);
        this.rootObject.addChild(this.bitmap)

        // scale the image
        this.scaleX = this.map.info.resolution;
        this.scaleY = this.map.info.resolution;
    }

    rosToGlobal(translation: ROSLIB.Vector3) {
        var x = (translation.x + this.width * this.scaleX! + this.origin!.position.x) / this.scaleX!;
        var y = (-translation.y + this.height * this.scaleY! + this.origin!.position.y) / this.scaleY!;
        console.log(translation.x, x, translation.y, y)
        return {
            x: x,
            y: y
        };
    }

    globalToRos(x: number, y: number) {
        var rosX = (x - this.width) * this.scaleX! - this.origin!.position.x
        var rosY = (this.height - y) * this.scaleY! + this.origin!.position.y
        return {
            x: rosX,
            y: rosY,
            z: 0
        } as ROSPoint
    }

    addCurrenPoseMarker() {
        var robotMarker = this.NavigationArrow()
        this.rootObject.addChild(robotMarker)

        const setPoseInterval = setInterval(() => {
            let pose = this.getPoseFn()
            let globalCoord = this.rosToGlobal(pose.translation)
            robotMarker.x = globalCoord.x
            robotMarker.y = globalCoord.y
            robotMarker.scaleX = 1.0 / this.rootObject.scaleX
            robotMarker.scaleY = 1.0 / this.rootObject.scaleY
            robotMarker.visible = true
            this.rootObject.update()
        }, 1000);
    }

    createOccupancyGridClient() {
        this.createOccupancyGrid()
        this.addCurrenPoseMarker()

        this.rootObject.on('click', (event) => {
            let evt = event as createjs.MouseEvent
            // convert to ROS coordinates
            var position: ROSPoint = this.globalToRos(evt.stageX, evt.stageY);

            this.moveBaseFn({
                position: position,
                orientation: {x: 0, y: 0, z: -0.45, w: 0.893}
            } as ROSPose)
            
            console.log(position)
        });
    }
}