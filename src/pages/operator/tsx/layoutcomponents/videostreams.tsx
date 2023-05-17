import React from "react";
import { ROSCompressedImage, className } from "shared/util";
import "operator/css/videostreams.css"
import { CustomizableComponentProps } from "./customizablecomponent";
import { DropZone } from "./dropzone";

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
    className?: string;
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
        this.outputVideoStream = new MediaStream();

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
        if (!this.canvas.current) throw 'Video stream canvas null'
        this.outputVideoStream = this.canvas.current.captureStream(this.fps);
        this.video.srcObject = this.outputVideoStream;
        this.drawVideo();
    }

    render() {
        return (
            <canvas
                ref={this.canvas!}
                width={this.width}
                height={this.height}
                className={this.className}
            />
        )
    }
}

export type VideoStreamComponentProps = CustomizableComponentProps & {
    stream: MediaStream,
    buttonPad: React.ReactNode,
}

/**
 * Displays a video stream with an optional button pad overlay
 * @param props properties
 * @returns a video stream component
 */
export const VideoStreamComponent = (props: VideoStreamComponentProps) => {
    const [streamStyle, setStreamStyle] = React.useState({});
    const videoRef = React.useRef<HTMLVideoElement>(null);

    // Record the height and width of the video component on resize
    const resizeObserver = new ResizeObserver(entries => {
        const { height, width } = entries[0].contentRect;
        setStreamStyle({ height, width });
    });
    React.useEffect(() => {
        if (!videoRef?.current) return;
        videoRef.current.srcObject = props.stream;
        resizeObserver.observe(videoRef.current);
        return () => resizeObserver.disconnect();
    }, [props.stream]);

    const { customizing } = props.sharedState;
    const selected = props.path === props.sharedState.activePath;
    const videoClass = className("video-canvas", { customizing, selected })

    function handleClick(e: React.MouseEvent<HTMLDivElement>) {
        console.log('click on video stream')
        e.stopPropagation();
        props.sharedState.onSelect(props.definition, props.path);
    }

    return (
        <div className='video-stream'>
            <div className="video-button-pad" style={streamStyle} onClick={handleClick}>
                {props.buttonPad ? props.buttonPad :
                    <DropZone
                        path={props.path + "-0"}
                        sharedState={props.sharedState}
                        parentDef={props.definition}
                    />}
            </div>

            <video ref={videoRef} autoPlay muted={true} className={videoClass}
                
            />
        </div>
    );
}

/** Renders all three video streams side by side */
export const AllVideoStreamComponent = (props: { streams: VideoStream[] }) => {
    console.log(props.streams)
    // let buttonPads = Bp.ExampleButtonPads;
    // let buttonPads = [undefined, undefined, undefined];
    // Replace the overhead button pad with predictive display
    // buttonPads[0] = <PredictiveDisplay onClick={(len, ang) => console.log(`Length: ${len}, Angle: ${ang}`)} />;
    const widths = ["30%", "22.5%", "45%"];
    return (
        <div id="video-stream-container">
            {props.streams.map((stream, i) => (
                <div key={i} className="video-stream" style={{ width: widths[i] }}>
                    {stream.render()}
                </div>
            )
            )}
        </div>
    );
};