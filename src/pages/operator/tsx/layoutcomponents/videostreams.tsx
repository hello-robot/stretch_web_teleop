import React from "react";
import { ROSCompressedImage, className } from "shared/util";
import "operator/css/videostreams.css"
import { CustomizableComponentProps } from "./customizablecomponent";
import { DropZone } from "./dropzone";
import { VideoStreamDef, VideoStreamId } from "../utils/componentdefinitions";
import { RemoteStream } from "shared/util"
import { ButtonPad } from "./buttonpads";

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

/**
 * Displays a video stream with an optional button pad overlay
 * @param props properties
 * @returns a video stream component
 */
export const VideoStreamComponent = (props: CustomizableComponentProps) => {
    const [streamStyle, setStreamStyle] = React.useState({});
    const videoRef = React.useRef<HTMLVideoElement>(null);

    const [clickXY, setClickXY] = React.useState<[number, number] | null>(null);

    const definition = props.definition as VideoStreamDef;
    const stream: MediaStream = getStream(definition.id, props.sharedState.remoteStreams);

    // Create the button pad overlay
    const buttonPadDef = definition.children.length > 0 ? definition.children[0] : undefined;
    const buttonPadProps = buttonPadDef ? {
        definition: buttonPadDef,
        path: props.path + "-0",
        sharedState: props.sharedState
    } as CustomizableComponentProps : undefined;
    const buttonPad = buttonPadProps ?
        <ButtonPad {...buttonPadProps} overlay={true} /> : undefined;


    // Record the height and width of the video component on resize
    const resizeObserver = new ResizeObserver(entries => {
        const { height, width } = entries[0].contentRect;
        setStreamStyle({ height, width });
    });

    React.useEffect(() => {
        if (!videoRef?.current) return;
        videoRef.current.srcObject = stream;
        resizeObserver.observe(videoRef.current);
        return () => resizeObserver.disconnect();
    }, [stream]);

    const { customizing } = props.sharedState;
    const selected = props.path === props.sharedState.activePath;
    const videoClass = className("video-canvas", { customizing, selected })

    /** Mark this video stream as selected */
    function selectSelf() {
        props.sharedState.onSelect(props.definition, props.path);
        setClickXY(null);
    }

    /** Mark the button pad child as selected */
    function selectChild() {
        props.sharedState.onSelect(buttonPadDef!, props.path + '-0');
        setClickXY(null);
    }

    /** Opens a popup  */
    function handleClick(event: React.MouseEvent<HTMLDivElement>) {
        event.stopPropagation();

        // If no button pad overlay then select self and return
        if (!buttonPadDef) {
            selectSelf();
            return;
        }

        // Create context menu popup where user can choose between selecting 
        // the button pad or the video stream
        const { clientX, clientY } = event;
        const { left, top } = videoRef.current!.getBoundingClientRect();
        const x = clientX - left;
        const y = clientY - top;
        setClickXY([x, y]);
    }

    return (
        <div className='video-stream'>
            <div
                className={className("video-button-pad", { customizing, selected })}
                style={streamStyle}
                onClick={customizing ? handleClick : undefined}
            >
                {buttonPad ? buttonPad :
                    <DropZone
                        path={props.path + "-0"}
                        sharedState={props.sharedState}
                        parentDef={props.definition}
                    />}
                {
                    clickXY ? <SelectContexMenu
                        clickXY={clickXY}
                        selectSelf={selectSelf}
                        selectChild={selectChild}
                        clickOut={() => setClickXY(null)}
                    /> : undefined
                }
            </div>
            <video ref={videoRef} autoPlay muted={true} className={videoClass} />
        </div>
    );
}

/**
 * Gets the stream based on the identifier
 * 
 * @param id identifier for the video stream
 * @param remoteStreams map of {@link RemoteStream}
 * @returns the corresponding stream
 */
function getStream(id: VideoStreamId, remoteStreams: Map<string, RemoteStream>): MediaStream {
    let streamName: string;
    switch (id) {
        case VideoStreamId.overhead:
            streamName = "overhead";
            break;
        case VideoStreamId.realsense:
            streamName = "realsense";
            break;
        case VideoStreamId.gripper:
            streamName = "gripper";
            break;
        default:
            throw Error(`unknow video stream id: ${id}`);
    }
    return remoteStreams.get(streamName)!.stream;
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


/** Props for {@link SelectContexMenu} */
type SelectContexMenuProps = {
    /** x and y location to render the context menu popup */
    clickXY: [number, number];
    /** Callback to select the video stream */
    selectSelf: () => void;
    /** Callback to select the child button pad */
    selectChild: () => void;
    /** Callback to hide the context menu popup when click outside */
    clickOut: () => void;
}

/**
 * Creates a context menu popup when user clicks during 
 * customization mode so the user can choose between the button pad and its 
 * parent video stream.
 * 
 * @param props {@link SelectContexMenuProps}
 */
const SelectContexMenu = (props: SelectContexMenuProps) => {
    const ref = React.useRef<HTMLUListElement>(null);
    const [x, y] = props.clickXY;

    // Handler to close dropdown when click outside
    React.useEffect(() => {

        /** Closes context menu if user clicks outside */
        const handler = (e: any) => {
            // If didn't click inside the context menu or the existing SVG, then
            // hide the popup
            if (ref.current && !ref.current.contains(e.target)) {
                props.clickOut();
                console.log('clicked')
            }
        };
        window.addEventListener("click", handler, true);
        return () => {
            window.removeEventListener("click", handler);
        };
    }, []);

    /**
     * Handles when the user clicks on one of the context menu options
     * @param e mouse event of the click
     * @param self if true selects itself (a button pad), if false selects its 
     * parent (the video stream)
     */
    function handleClick(e: React.MouseEvent<HTMLLIElement>, self: boolean) {
        self ? props.selectSelf() : props.selectChild();

        // Make sure background elements don't receive a click
        e.stopPropagation();
    }

    return (

        <ul aria-label="Select"
            ref={ref}
            className="button-pad-context-menu"
            style={{ top: `${y}px`, left: `${x}px` }}
        >
            <li onClick={(e) => handleClick(e, false)}>Button Pad</li>
            <li onClick={(e) => handleClick(e, true)}>Video Stream</li>
        </ul>
    );
}