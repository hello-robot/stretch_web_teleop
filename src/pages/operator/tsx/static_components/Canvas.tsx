import React from "react";
import createjs from "createjs-module";

export class Canvas extends React.Component {
    private divID: string;
    private className: string;
    private width: number;
    private height: number;
    public scene?: createjs.Stage;
    private x_prev_shift?: number;
    private y_prev_shift?: number;

    constructor(props: {
        divID: string,
        className: string,
        width: number,
        height: number
    }) {
        super(props);
        this.divID = props.divID
        this.className = props.className
        this.width = props.width
        this.height = props.height
        this.createCanvas()
    }

    createCanvas() {
        var background = '#111111';

        // create the canvas to render to
        var canvas = document.createElement('canvas');
        canvas.setAttribute("class", this.className);
        canvas.width = this.width;
        canvas.height = this.height
        
        // create the easel to use
        this.scene = new createjs.Stage(canvas);
        
        // add the renderer to the page
        document.getElementById(this.divID)!.appendChild(canvas);
        
        // update at 30fps
        createjs.Ticker.framerate = 30;
        createjs.Ticker.addEventListener('tick', this.scene);
    }

    scaleToDimensions(width: number, height: number) {
        if (!this.scene) throw 'ROSViewer scene is undefined!'
        
        // save scene scaling
        this.scene.scaleX = this.width / width;
        this.scene.scaleY = this.height / height;
        this.scene.update()
    }

    shift(x: number, y: number) {
        if (!this.scene) throw 'ROSViewer scene is undefined!'
        
        this.x_prev_shift = this.scene.x;
        this.y_prev_shift = this.scene.y;

        // shift scene by scaling the desired offset
        this.scene.x -= (x * this.scene.scaleX);
        this.scene.y += (y * this.scene.scaleY);
    }
}