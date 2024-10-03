import "home/css/SideBySideView.css";
import React, { useEffect, useState } from "react";
import { isTablet, isBrowser } from "react-device-detect";
import Box from '@mui/material/Box';
import Grid from '@mui/material/Grid2';
import { Changelog } from "./Changelog";
import { CallRobotSelector } from "./CallRobotSelector";


export const SideBySideView = (props) => {

    return isTablet || isBrowser ? (
        <Box sx={{ flexGrow: 1 }} className='sbs-container'>
            <Grid container rowSpacing={1} columnSpacing={{ lg: 4, xl: 5 }}>
                <Grid size={12}>
                    <h1>Stretch Web Teleop</h1>
                </Grid>
                <Grid size={{ md: 12, lg: 6 }}>
                    <Changelog style={{ height: "500px", maxHeight: "500px" }} />
                </Grid>
                <Grid size={{ md: 12, lg: 6 }}>
                    <CallRobotSelector style={{ height: "500px", maxHeight: "500px" }} />
                </Grid>
            </Grid>
        </Box>
    ) : (
        <Box sx={{ flexGrow: 1 }} className='sbs-container'>
            <Grid container spacing={2}>
                <Grid size={12}>
                    <h1>Stretch Web Teleop</h1>
                </Grid>
                <Grid size={12}>
                    <CallRobotSelector style={{ height: "500px", maxHeight: "500px" }} />
                </Grid>
                <Grid size={12}>
                    <Changelog style={{ height: "500px", maxHeight: "500px" }} />
                </Grid>
            </Grid>
        </Box>
    );
};
