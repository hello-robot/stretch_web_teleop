import * as React from "react";
import {
    GridContextProvider,
    GridDropZone,
    GridItem,
    swap
} from "react-grid-dnd";
import {useRef} from 'react';
import { OverheadNavActionOverlay } from './overlays'
import { ROSCompressedImage } from "../util/util";
import "../css/operator.css"

/** Displays a single camera view, contains some of the more complex backend
 * logic for retrieving the stream.
 */
export class VideoStream extends React.Component {
    canvas = React.createRef<HTMLCanvasElement>();
    img: HTMLImageElement;
    video: HTMLVideoElement;
    width: number;
    height: number;
    fps: number;

    constructor(props) {
        super(props);
        this.width = props.width;
        this.height = props.height;
        this.fps = props.fps;
        this.img = document.createElement("img");
        this.video = document.createElement("video");
        this.video.style.display = "block";
        this.video.setAttribute("width", this.width.toString());
        this.video.setAttribute("height", this.height.toString());
    }

    get imageReceived() {
        return this.img.src != null;
    }

    renderVideo() {
        if (!this.imageReceived) {
            return;
        }
        this.canvas.current?.getContext('2d')?.drawImage(this.img, 0, 0, this.width, this.height)
    }

    updateImage(message: ROSCompressedImage) {
        this.img.src = 'data:image/jpg;base64,' + message.data;
    }

    drawVideo() {
        this.renderVideo();
        requestAnimationFrame(this.drawVideo.bind(this));
    }

    start() {
        let outputVideoStream = this.canvas.current?.captureStream(this.fps);
        this.video.srcObject = outputVideoStream as MediaProvider;
        this.drawVideo();
    }

    render() {
        return (
            <canvas ref={this.canvas!} width={this.width} height={this.height} style={{width: "100%"}}></canvas>
        )
    }
}

/*
 * Initialize the video stream objects.
 */
const navigationProps = {
    width: 1024,
    height: 768,
    fps: 6.0
}
export const navigationVideoStream = new VideoStream(navigationProps)

const realsenseProps = {
    width: 640,
    height: 360,
    fps: 6.0
}
export const realsenseVideoStream = new VideoStream(realsenseProps)

const gripperProps = {
    width: 1024,
    height: 768,
    fps: 6.0
}
export const gripperVideoStream = new VideoStream(gripperProps)

/** Displays a single videostream and the button pad */
const VideoViewer = (props: any) => {
    /** The video stream to display */
    const videoStream: VideoStream = props.videoStream;

    /** Refrence to the outermost parent div */
    const parentDivRef = useRef<HTMLDivElement>(null);

    /** Style object for the child div containing the stream element */
    const [streamStyle, setStreamStyle] = React.useState({});

    // This handles resizing the video stream object to fit inside of the 
    if (props.rotate) {
        const heightRatio = videoStream.props.width / videoStream.props.height;
        React.useEffect(() => {
            const observer = new ResizeObserver(entries => {
                const { width, height } = entries[0].contentRect;
                const calculatedHeight = width * heightRatio;
                let newHeight: number;
                let newWidth: number;
                if (calculatedHeight < height) {
                    newHeight = calculatedHeight;
                    newWidth = width;
                } else {
                    newHeight = height;
                    newWidth = height / heightRatio;
                }
                const down = (newHeight - newWidth) / 2;
                const left = -(newHeight - width) / 2;
                const transform = `translate(${left}px, ${down}px) rotate(90deg)`
                const newStyle = props.rotate ? {
                    width: newHeight,
                    height: newWidth,
                    border: "1px dotted blue",
                    transform: transform,
                } : {};
                setStreamStyle(newStyle)
            });
        
            observer.observe(parentDivRef.current);
        
            return () => observer.disconnect();
        }, []);
    }
    // end of rotated resizing logic

    return (
        <div className="video-viewer" ref={parentDivRef} >
            {props.buttonPad}
            <div style={streamStyle}>
                {videoStream.render()}
            </div>
        </div>
    )
}

/** Displays the video streams arranges horizontally. */
export const VideoStreams = () => {
    const streams = [realsenseVideoStream, navigationVideoStream, gripperVideoStream]
    streams.forEach((value) => value.start())
    return (
        <div id="video-stream-container">
            {streams.map((stream, i) => (
                <VideoViewer key={i} videoStream={stream} rotate={stream ==navigationVideoStream || stream==realsenseVideoStream}/>
            ))}
        </div>
    )
}