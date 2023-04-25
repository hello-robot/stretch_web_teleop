import * as React from "react";
import { Card, CardContent } from '@mui/material';
import Grid from '@mui/material/Grid'
import {
    GridContextProvider,
    GridDropZone,
    GridItem,
    swap
} from "react-grid-dnd";
import {useRef} from 'react';
import { OverheadNavActionOverlay } from './overlays'
import { ROSCompressedImage } from "../util/util";

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
            <canvas ref={this.canvas!} width={this.width} height={this.height} style={{width: "100%", paddingTop: "10px"}}></canvas>
        )
    }
}

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

// Navigation overhead fisheye videostream
export const OverheadComponent = () => {
    const style = {
        width: "100%",
        height: "auto",
        paddingTop: "10px"
    }

    const contentRef = useRef<HTMLDivElement>(null);
    const [height, setHeight] = React.useState(0)
    const [width, setWidth] = React.useState(0)

    // Update height and width variables as window is resized 
    React.useEffect(() => {
        if (contentRef.current) {
            const observer = new ResizeObserver(entries => {
                setWidth(entries[0].contentRect.height)
                setHeight(entries[0].contentRect.width)
            })
            observer.observe(contentRef.current)
            return () => contentRef.current && observer.unobserve(contentRef.current)
        }
    }, []);

    return (
        <Card sx={{ maxWidth: 1000, transform: "rotate(90deg)" }}>
            <CardContent>
                    <div ref={contentRef} className="imageViewer" >
                        <OverheadNavActionOverlay width={width} height={height}/>
                        {navigationVideoStream.render()}
                    </div>
            </CardContent>
        </Card>
    );
};

// Realsense video stream 
export const RealsenseComponent = () => {
    const style = {
        width: "100%",
        height: "auto",
        paddingTop: "10px"
    }

    return (
        <Card sx={{ maxWidth: 1000, maxHeight: 1000, transform: "rotate(90deg)" }}>
            <CardContent>
                {realsenseVideoStream.render()}
            </CardContent>
        </Card>
    );
};

// Gripper video stream
export const GripperComponent = () => {
    const style = {
        width: "100%",
        height: "auto",
    }

    return (
        <Card sx={{ maxWidth: 1000 }}>
            <CardContent>
                {gripperVideoStream.render()}
            </CardContent>
        </Card>
    );
};

// Creates a grid of video streams
export const VideoStreams = () => {
    realsenseVideoStream.start()
    navigationVideoStream.start()
    gripperVideoStream.start()
    return (
        <Grid container alignItems="stretch">
            <Grid item xs>
                <OverheadComponent/>
            </Grid>
            <Grid item xs>
                <RealsenseComponent/>
            </Grid>
            <Grid item xs>
                <GripperComponent/>
            </Grid>
        </Grid>
    );
}

// Creates a grid of rearrangeable video streams
export const VideoStreamGrid = () => {
    const [items, setItems] = React.useState([
        {position: 1, component: <OverheadComponent/>},
        {position: 2, component: <RealsenseComponent/>},
        {position: 3, component: <GripperComponent/>},
    ]);

    function onChange(sourceId: string, sourceIndex: any, targetIndex: any, targetId?: string) {
        const nextState = swap(items, sourceIndex, targetIndex);
        setItems(nextState);
    }

    return (
        <GridContextProvider onChange={onChange}>
            <GridDropZone
                id="items"
                boxesPerRow={3}
                rowHeight={500}
                style={{ width: "100%", height: "100%" }}>
                {items.map((item) => (
                    <GridItem key={item.position}>
                        {item.component}
                    </GridItem>
                ))}
            </GridDropZone>
        </GridContextProvider>
    )
}