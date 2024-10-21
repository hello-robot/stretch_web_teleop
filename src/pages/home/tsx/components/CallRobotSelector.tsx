import "home/css/CallRobotSelector.css";
import React, { useEffect, useState } from "react";
import Box from '@mui/material/Box';
import Grid from '@mui/material/Grid2';
import Card from '@mui/material/Card';
import CardActionArea from "@mui/material/CardActionArea";
import CardContent from '@mui/material/CardContent';
import Typography from '@mui/material/Typography';
import CircleIcon from "@mui/icons-material/Circle";
import { green, red } from '@mui/material/colors';


const CallRobotItem = (props: {
    name: String;
    isOnline: Boolean;
}) => {

    return (
        <Card sx={{ minWidth: 275 }}>
            {props.isOnline ? (
                <CardActionArea onClick={() => console.log("CardActionArea clicked")}>
                    <CardContent>
                        <Typography variant="h5" component="div">
                            {props.name}
                        </Typography>
                        <Typography sx={{ color: 'text.secondary', mb: 1.5 }}>
                            <CircleIcon sx={{
                                fontSize: 12,
                                "@keyframes glowing_green": {
                                    "0%": {
                                        color: green["A400"]
                                    },
                                    "50%": {
                                        color: green["A200"]
                                    },
                                    "100%": {
                                        color: green["A400"]
                                    }
                                },
                                color: green["A400"],
                                animation: "glowing_green 3s linear infinite" }} /> Online
                        </Typography>
                    </CardContent>
                </CardActionArea>
            ) : (
                <CardContent>
                    <Typography variant="h5" component="div">
                        {props.name}
                    </Typography>
                    <Typography sx={{ color: 'text.secondary', mb: 1.5 }}>
                        <CircleIcon sx={{
                            fontSize: 12,
                            "@keyframes glowing_red": {
                                "0%": {
                                    color: red["A400"]
                                },
                                "50%": {
                                    color: red["A200"]
                                },
                                "100%": {
                                    color: red["A400"]
                                }
                            },
                            color: red["A400"],
                            animation: "glowing_red 3s linear infinite" }} /> Offline
                    </Typography>
                </CardContent>
            )}
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
                    <CallRobotItem name="stretch-se3-3004" isOnline={true} />
                </Grid>
                <Grid size={{md: 12, lg: 6}}>
                    <CallRobotItem name="stretch-re1-1002" isOnline={false} />
                </Grid>
            </Grid>
        </Box>
    );
};