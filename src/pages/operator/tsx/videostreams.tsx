import React from "react";
import { ROSCompressedImage, className } from "utils/util";
import "operator/css/videostreams.css"
import { CustomizableComponentProps } from "./customizablecomponent";

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


// Dead code
export class VideoControl extends React.Component<{ stream: MediaStream }, {}> {
    videoRef: React.RefObject<HTMLVideoElement>;

    constructor(props: { stream: MediaStream }) {
        super(props);
        this.videoRef = React.createRef<HTMLVideoElement>();
        // this.state = {
        //     stream: null
        // }
    }

    componentDidMount() {
        if (this.videoRef.current) {
            this.videoRef.current.srcObject = this.props.stream;
            // this.setState({stream: this.props.stream})
        }
    }

    // componentDidUpdate() {
    //     if (this.videoRef.current) {
    //         this.videoRef.current.srcObject = this.props.stream;
    //     }
    // }

    render() {
        // console.log("videocontrol ", this.videoRef.current?.srcObject)
        return (
            <video ref={this.videoRef} autoPlay muted={true} className="video-canvas" />
        )
    }
}


// type VideoControlPropsType = VideoHTMLAttributes<HTMLVideoElement> & {
//     srcObject: MediaStream;
//   };

//   export default function Video({ srcObject, ...props }: VideoControlPropsType) {
//     const refVideo = useRef<HTMLVideoElement>(null);
//     const [srcObjectSet, setSrcObjectSet] = React.useState(false);

//     useEffect(() => {
//       if (refVideo.current && srcObject) {
//         refVideo.current.srcObject = srcObject;
//         refVideo.current.onloadedmetadata = function(e) {
//             refVideo.current!.play();
//         };
//         setSrcObjectSet(true);
//       }
//     }, [srcObject]);

//     console.log("Video element:", refVideo.current);
//     return <video ref={refVideo} {...props} autoPlay  playsInline muted />;
//   }

//   export const VideoControlComponent = (props: { streams: MediaStream[] }) => {
//     let test= <Video key={0} srcObject={props.streams[0]} />
//     console.log(test)
//     return (
//         <div>
//             <button>test</button>
//             {props.streams.map((stream, i) => (
//                 <Video key={i} srcObject={stream} />
//             ))}
//         </div>
//     );
//   };

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
    // const streamRef = props.stream.props.videoRef;  // refrence to the canvas element from the stream
    const resizeObserver = new ResizeObserver(entries => {
        const { height, width } = entries[0].contentRect;
        setStreamStyle({ height, width });
    });
    React.useEffect(() => {
        // console.log("hook stream", props.stream)
        if (!videoRef?.current) return;
        videoRef.current.srcObject = props.stream;
        resizeObserver.observe(videoRef.current);
        return () => resizeObserver.disconnect();
    }, [props.stream]);
    // console.log('stream comp id', props.definition.id, "stream", props.stream)

    const { customizing, onSelect } = props.sharedState;

    const active = props.path === props.sharedState.activePath;
    const videoClass = className("video-canvas", { customizing, active })

    return (
        <div
            className='video-stream'
            onClick={() => onSelect(props.definition, props.path)}
        >
            {
                props.buttonPad ?
                    <div
                        className="video-button-pad"
                        style={streamStyle}
                    >
                        {props.buttonPad}
                    </div> : undefined
            }
            <video ref={videoRef} autoPlay muted={true} className={videoClass} />
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