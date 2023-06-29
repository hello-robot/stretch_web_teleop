import React from "react";
import createjs from "createjs-module";

export class ROSViewer extends React.Component {
    private divID: string;
    private width: number;
    private height: number;
    public scene: createjs.Stage;
    private x_prev_shift?: number;
    private y_prev_shift?: number;

    constructor(props: {
        divID: string,
        width: number,
        height: number
    }) {
        super(props);
        this.divID = props.divID
        this.width = props.width
        this.height = props.height
        this.createViewer()
    }

    createViewer() {
        var divID = this.divID;
        this.width = this.width;
        this.height = this.height;
        var background = '#111111';

        // create the canvas to render to
        var canvas = document.createElement('canvas');
        canvas.setAttribute("class", "mapCanvas");
        canvas.width = this.width;
        canvas.height = this.height
        // canvas.style.background = background;
        // document.getElementById(divID)!.appendChild(canvas);
        
        // create the easel to use
        this.scene = new createjs.Stage(canvas);
        
        // change Y axis center
        // this.scene.y = this.height;
        
        // add the renderer to the page
        document.getElementById(divID)!.appendChild(canvas);
        
        // update at 30fps
        createjs.Ticker.framerate = 30;
        createjs.Ticker.addEventListener('tick', this.scene);
    }

    scaleToDimensions(width: number, height: number) {
        if (!this.scene) throw 'ROSViewer scene is undefined!'
        
        // save scene scaling
        this.scene.scaleX = this.width / width;
        this.scene.scaleY = this.height / height;
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