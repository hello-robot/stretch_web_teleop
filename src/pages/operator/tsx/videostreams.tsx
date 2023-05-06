import React from "react";
import { ROSCompressedImage } from "utils/util";
import * as Bp from "operator/tsx/buttonpads"
import { PredictiveDisplay } from "operator/tsx/predictivedisplay";
import "operator/css/videostreams.css"

type VideoStreamProps = {
    width: number,
    height: number,
    fps: number
}

export class VideoStream extends React.Component<VideoStreamProps> {
    canvas = React.createRef<HTMLCanvasElement>();
    img: HTMLImageElement;
    video: HTMLVideoElement;
    width: number;
    height: number;
    fps: number;
    outputVideoStream?: MediaStream

    constructor(props: VideoStreamProps) {
        super(props);
        this.width = props.width;
        this.height = props.height;
        this.fps = props.fps;
        this.img = document.createElement("img");
        this.video = document.createElement("video");
        this.video.style.display = "block";
        this.video.setAttribute("width", this.width.toString());
        this.video.setAttribute("height", this.height.toString());

        this.updateImage = this.updateImage.bind(this);
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
        this.outputVideoStream = this.canvas.current?.captureStream(this.fps);
        this.video.srcObject = this.outputVideoStream as MediaProvider;
        this.drawVideo();
    }

    render() {
        return (
            <canvas ref={this.canvas!} width={this.width} height={this.height}></canvas>
        )
    }
}

/**
 * Displays a video stream with an optional button pad overlay
 * @param props properties
 * @returns a video stream component
 */
export const VideoStreamComponent = (props: { stream: VideoStream, buttonPad?: React.ReactNode }) => {
    const [streamHeight, setStreamHeight] = React.useState(0);
    const streamRef = props.stream.canvas;  // refrence to the canvas element from the stream
    const resizeObserver = new ResizeObserver(entries => {
        const { height } = entries[0].contentRect;
        setStreamHeight(height);
    });
    React.useEffect(() => {
        if (!streamRef?.current) return;
        resizeObserver.observe(streamRef.current);
        return () => resizeObserver.disconnect();
    }, []);
    return (
        <div className="video-stream">
            {
                props.buttonPad ?
                    <div
                        className="video-button-pad"
                        style={{ height: streamHeight }}
                    >
                        {props.buttonPad}
                    </div> : undefined
            }
            {props.stream.render()}
        </div>
    );
}

// Gripper video stream
export const AllVideoStreamComponent = (props: { streams: VideoStream[] }) => {
    console.log(props.streams)
    let buttonPads = Bp.ExampleButtonPads;
    // let buttonPads = [undefined, undefined, undefined];
    // Replace the overhead button pad with predictive display
    buttonPads[0] = <PredictiveDisplay onClick={(len, ang) => console.log(`Length: ${len}, Angle: ${ang}`)} />;
    const widths = ["30%", "22.5%", "45%"];
    return (
        <div id="video-stream-container">
            {props.streams.map((stream, i) => (
                <div key={i} className="video-stream" style={{ width: widths[i] }}>
                    <div className="video-button-pad">
                        {buttonPads[i]}
                    </div>
                    {stream.render()}
                </div>
            )
            )}
        </div>
    );
};