import React from "react";
import { ROSCompressedImage } from "shared/util";
var jpeg = require("jpeg-js");

type VideoStreamProps = {
    width: number;
    height: number;
    scale: number;
    fps: number;
    streamName: string;
};

// Adapted from https://github.com/personalrobotics/feeding_web_interface/blob/2dc2dfe3a67b840b619fa9b00ef2f2ee66c46962/feedingwebapp/src/robot/VideoStream.jsx#L10C1-L31C2
function dataURItoBlob(dataURI: string) {
    // convert base64 to raw binary data held in a string
    // doesn't handle URLEncoded DataURIs - see SO answer #6850276 for code that does this
    var splitString = dataURI.split(",");
    var byteString = atob(splitString[1]);

    // separate out the mime component
    var mimeString = splitString[0].split(":")[1].split(";")[0];

    // write the bytes of the string to an ArrayBuffer
    var ab = new ArrayBuffer(byteString.length);
    var ia = new Uint8Array(ab);
    for (var i = 0; i < byteString.length; i++) {
        ia[i] = byteString.charCodeAt(i);
    }

    // write the ArrayBuffer to a blob, and you're done
    var blob = new Blob([ab], { type: mimeString });
    return blob;
}

export class VideoStream extends React.Component<VideoStreamProps> {
    canvas = React.createRef<HTMLCanvasElement>();
    img: HTMLImageElement;
    video: HTMLVideoElement;
    width: number;
    height: number;
    scale: number;
    fps: number;
    streamName: string;
    className?: string;
    outputVideoStream?: MediaStream;
    aspectRatio: any;
    started: boolean;

    constructor(props: VideoStreamProps) {
        super(props);
        this.width = props.width;
        this.height = props.height;
        this.scale = props.scale;
        this.fps = props.fps;
        this.streamName = props.streamName;
        this.img = document.createElement("img");
        this.video = document.createElement("video");
        this.video.style.display = "block";
        this.video.setAttribute("width", this.width.toString());
        this.video.setAttribute("height", this.height.toString());
        this.outputVideoStream = new MediaStream();
        this.started = false;

        this.updateImage = this.updateImage.bind(this);
    }

    get imageReceived() {
        return this.img.src !== "";
    }

    renderVideo() {
        if (!this.imageReceived) {
            return;
        }
        this.canvas.current
            ?.getContext("2d")
            ?.drawImage(this.img, 0, 0, this.width, this.height);
    }

    updateImage(message: ROSCompressedImage, verbose: boolean = false) {
        if (verbose) {
            console.log(
                this.streamName,
                "stream got image with latency",
                Date.now() / 1.0e3 -
                    (message.header.stamp.sec +
                        message.header.stamp.nanosec / 1.0e9),
            );
        }
        if (!this.imageReceived) {
            let { width, height } = jpeg.decode(
                Uint8Array.from(atob(message.data), (c) => c.charCodeAt(0)),
                true,
            );
            this.aspectRatio = width / height;
            this.height = Math.max(height * this.aspectRatio, 1000);
            this.width = this.height * this.aspectRatio;
            this.canvas.current!.width = this.width;
            this.canvas.current!.height = this.height;
        }
        if (this.img.src) {
            URL.revokeObjectURL(this.img.src);
        }
        this.img.src = URL.createObjectURL(
            dataURItoBlob("data:image/jpg;base64," + message.data),
        );
    }

    drawVideo() {
        this.renderVideo();
        requestAnimationFrame(this.drawVideo.bind(this));
    }

    start() {
        if (!this.started) {
            console.log("Starting video stream", this.streamName);
            if (!this.canvas.current) throw "Video stream canvas null";
            this.outputVideoStream = this.canvas.current.captureStream(
                this.fps,
            );
            this.video.srcObject = this.outputVideoStream;
            this.drawVideo();
            this.started = true;
        } else {
            console.log("Video stream already started", this.streamName);
        }
    }

    render() {
        return (
            <canvas
                ref={this.canvas!}
                width={this.width}
                height={this.height}
                className={this.className}
            />
        );
    }
}

/** Renders all three video streams side by side */
export const AllVideoStreamComponent = (props: { streams: VideoStream[] }) => {
    console.log(props.streams);
    // let buttonPads = Bp.ExampleButtonPads;
    // let buttonPads = [undefined, undefined, undefined];
    // Replace the overhead button pad with predictive display
    // buttonPads[0] = <PredictiveDisplay onPointerDown={(len, ang) => console.log(`Length: ${len}, Angle: ${ang}`)} />;
    const widths = ["30%", "22.5%", "45%"];
    return (
        <div id="video-stream-container">
            {props.streams.map((stream, i) => (
                <div
                    key={i}
                    className="video-container"
                    style={{ width: widths[i] }}
                >
                    {stream.render()}
                </div>
            ))}
        </div>
    );
};
