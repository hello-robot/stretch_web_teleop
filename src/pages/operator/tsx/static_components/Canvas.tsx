import React from "react";
import createjs from "createjs-module";

/**
 * Canvas.tsx creates a canvas element
 * and initializes a CreateJS stage.
 * This component used for rendering graphics
 * in <canvas>.
 */
export class Canvas extends React.Component {
    // Unique ID of the div where the canvas will be rendered
    private divID: string;
    // CSS class name for the canvas element
    private className: string;
    // Width of the canvas in pixels
    private width: number;
    // Height of the canvas in pixels
    private height: number;
    // Reference to the CreateJS stage (scene)
    public scene?: createjs.Stage;

    constructor(props: {
        divID: string;
        className: string;
        width: number;
        height: number;
    }) {
        super(props);
        this.divID = props.divID;
        this.className = props.className;
        this.width = props.width;
        this.height = props.height;
        // Initialize the canvas and CreateJS stage
        this.createCanvas();
    }

    // Creates the canvas element, attaches it to the DOM, and sets up the CreateJS stage
    createCanvas() {
        // create the canvas to render to
        var canvas = document.createElement("canvas");
        canvas.setAttribute("class", this.className);
        canvas.width = this.width;
        canvas.height = this.height;

        // create the easel to use
        this.scene = new createjs.Stage(canvas);

        // add the renderer to the page
        document.getElementById(this.divID)!.replaceChildren(canvas);

        // update at 30fps
        createjs.Ticker.framerate = 30;
        createjs.Ticker.addEventListener("tick", this.scene);
    }

    // Scales the scene to fit the specified width and height
    scaleToDimensions(width: number, height: number) {
        if (!this.scene) throw "Canvas scene is undefined!";

        // save scene scaling
        this.scene.scaleX = this.width / width;
        this.scene.scaleY = this.height / height;
        this.scene.update();
    }
}
