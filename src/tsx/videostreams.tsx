import * as React from "react";
import { Card, CardContent } from '@mui/material';
import Grid from '@mui/material/Grid'
import {
    GridContextProvider,
    GridDropZone,
    GridItem,
    swap
} from "react-grid-dnd";
import { ImageViewer, Encoding, TransportLayer } from 'rosreact';
import {useRef} from 'react';
import { NavDriveForward, NavDriveBackward, NavRotateLeft, NavRotateRight } from './overlays'

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
                        <NavDriveForward width={width} height={height}/>
                        <NavDriveBackward width={width} height={height}/>
                        <NavRotateLeft width={width} height={height}/>
                        <NavRotateRight width={width} height={height}/>
                        <ImageViewer 
                            topic="/navigation_camera/image_raw" 
                            encoding={Encoding.ros} 
                            transportLayer={TransportLayer.compressed}
                            imageStyle={style}
                        />   
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
                <ImageViewer 
                    topic="/camera/color/image_raw" 
                    encoding={Encoding.ros} 
                    transportLayer={TransportLayer.compressed} 
                    imageStyle={style}
                />
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
                <ImageViewer 
                    topic="/gripper_camera/image_raw" 
                    encoding={Encoding.ros} 
                    transportLayer={TransportLayer.compressed}
                    imageStyle={style}
                />
            </CardContent>
        </Card>
    );
};

// Creates a grid of video streams
export const VideoStreams = () => {
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

