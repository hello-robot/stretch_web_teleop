import "home/css/Changelog.css";
import React, { useEffect, useState } from "react";
import Box from '@mui/material/Box';
import Grid from '@mui/material/Grid2';
import { styled } from '@mui/material/styles';
import Paper from '@mui/material/Paper';

const LogItem = styled(Paper)(({ theme }) => ({
    backgroundColor: '#fff',
    ...theme.typography.body2,
    padding: theme.spacing(1),
    color: theme.palette.text.secondary,
    ...theme.applyStyles('dark', {
      backgroundColor: '#1A2027',
    }),
}));


export const Changelog = (props: {
    style?: React.CSSProperties;
}) => {

    return (
        <Box sx={{ flexGrow: 1 }}>
            <h2>What's new?</h2>
            <Grid container spacing={2} className='cv-container' style={props.style}>
                <Grid size={12}>
                    <LogItem>
                        <span><b style={{fontWeight: "bolder"}}>Login Page</b> - Oct 9th, 2024</span>
                        <p>The new homepage shows the robots you can call.</p>
                    </LogItem>
                </Grid>
                <Grid size={12}>
                    <LogItem>
                        <span><b style={{fontWeight: "bolder"}}>Homing Button</b> - Sept 21st, 2024</span>
                        <p>A button appears if your robot needs to be homed.</p>
                    </LogItem>
                </Grid>
                <Grid size={12}>
                    <LogItem>
                        <span><b style={{fontWeight: "bolder"}}>Blah blah</b> - May 18st, 2024</span>
                        <p>Lorem ipsum dolor et.</p>
                    </LogItem>
                </Grid>
                <Grid size={12}>
                    <LogItem>
                        <span><b style={{fontWeight: "bolder"}}>Blah blah</b> - May 18st, 2024</span>
                        <p>Lorem ipsum dolor et.</p>
                    </LogItem>
                </Grid>
                <Grid size={12}>
                    <LogItem>
                        <span><b style={{fontWeight: "bolder"}}>Blah blah</b> - May 18st, 2024</span>
                        <p>Lorem ipsum dolor et.</p>
                    </LogItem>
                </Grid>
                <Grid size={12}>
                    <LogItem>
                        <span><b style={{fontWeight: "bolder"}}>Blah blah</b> - May 18st, 2024</span>
                        <p>Lorem ipsum dolor et.</p>
                    </LogItem>
                </Grid>
            </Grid>
        </Box>
    );
};
