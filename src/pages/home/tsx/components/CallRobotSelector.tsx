import "home/css/CallRobotSelector.css";
import React, { useEffect, useState } from "react";
import Box from '@mui/material/Box';
import Grid from '@mui/material/Grid2';
import Card from '@mui/material/Card';
import CardContent from '@mui/material/CardContent';
import CardActions from '@mui/material/CardActions';
import Button from '@mui/material/Button';
import Typography from '@mui/material/Typography';
import CircleIcon from "@mui/icons-material/Circle";
import { green, red, yellow, grey } from '@mui/material/colors';

function get_indicator_text(status_str) {
    switch (status_str) {
        case "online":
            return "Online"
        case "offline":
            return "Offline"
        case "occupied":
            return "Occupied"
        default:
            return "Unknown"
    }
}


function get_indicator(status_str) {
    let statusui;
    switch (status_str) {
        case "online":
            statusui = {
                "color_name": "green",
                "color": green,
            };
            break;
        case "offline":
            statusui = {
                "color_name": "red",
                "color": red,
            };
            break;
        case "occupied":
            statusui = {
                "color_name": "yellow",
                "color": yellow,
            };
            break;
        default:
            statusui = {
                "color_name": "grey",
                "color": grey,
            };
    }
    let indicator_css = {
        fontSize: 12,
        color: statusui['color']["A400"],
        animation: `glowing_${statusui['color_name']} 3s linear infinite`
    };
    indicator_css[`@keyframes glowing_${statusui['color_name']}`] = {
        "0%": {
            color: statusui['color']["A400"]
        },
        "50%": {
            color: statusui['color']["A200"]
        },
        "100%": {
            color: statusui['color']["A400"]
        }
    };
    return <CircleIcon sx={indicator_css} />
}



function get_action(status_str) {
    switch (status_str) {
        case "online":
            return <Button variant="contained" size="small">Call Robot</Button>
        case "offline":
            return <Button variant="contained" size="small" disabled>Call Robot</Button>
        case "occupied":
            return <Button variant="contained" size="small" disabled>Call Robot</Button>
        default:
            return <Button variant="contained" size="small" disabled>Call Robot</Button>
    }
}

const CallRobotItem = (props: {
    name: String;
    status: String;
}) => {

    return (
        <Card sx={{ minWidth: 275 }}>
            <CardContent>
                <Typography variant="h5" component="div">
                    {props.name}
                </Typography>
                <Typography sx={{ color: 'text.secondary', mb: 1.5 }}>
                    {get_indicator(props.status)} {get_indicator_text(props.status)}
                </Typography>
            </CardContent>
            <CardActions>
                {get_action(props.status)}
            </CardActions>
        </Card>
    );
};


export const CallRobotSelector = (props: {
    style?: React.CSSProperties;
}) => {

    return (
        <Box sx={{ flexGrow: 1 }}>
            <h2>Robots:</h2>
            <Grid container spacing={2} className='rs-container' style={props.style}>
                <Grid size={{md: 12, lg: 6}}>
                    <CallRobotItem name="stretch-se3-3004" status="online" />
                </Grid>
                <Grid size={{md: 12, lg: 6}}>
                    <CallRobotItem name="stretch-re1-1002" status="offline" />
                </Grid>
                <Grid size={{md: 12, lg: 6}}>
                    <CallRobotItem name="stretch-re1-1002" status="occupied" />
                </Grid>
                <Grid size={{md: 12, lg: 6}}>
                    <CallRobotItem name="stretch-re1-1002" status="whatever" />
                </Grid>
            </Grid>
        </Box>
    );
};