import "home/css/CallRobotSelector.css";
import React, { useEffect, useState } from "react";
import Box from '@mui/material/Box';
import Grid from '@mui/material/Grid2';
import Card from '@mui/material/Card';
import CardActions from '@mui/material/CardActions';
import CardContent from '@mui/material/CardContent';
import Button from '@mui/material/Button';
import Typography from '@mui/material/Typography';
import { styled } from '@mui/material/styles';
import Paper from '@mui/material/Paper';


const Item = styled(Paper)(({ theme }) => ({
    backgroundColor: '#fff',
    ...theme.typography.body2,
    padding: theme.spacing(1),
    textAlign: 'center',
    color: theme.palette.text.secondary,
    ...theme.applyStyles('dark', {
      backgroundColor: '#1A2027',
    }),
}));

const CallRobotItem = () => {

    return (
        <Card sx={{ minWidth: 275 }}>
            <CardContent>
                <Typography gutterBottom sx={{ color: 'text.secondary', fontSize: 14 }}>
                    Word of the Day
                </Typography>
                <Typography variant="h5" component="div">
                    benevolent
                </Typography>
                <Typography sx={{ color: 'text.secondary', mb: 1.5 }}>adjective</Typography>
                <Typography variant="body2">
                    well meaning and kindly.
                    <br />
                    {'"a benevolent smile"'}
                </Typography>
                <CardActions>
                    <Button size="small">Learn More</Button>
                </CardActions>
            </CardContent>
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
                    <CallRobotItem />
                </Grid>
                <Grid size={{md: 12, lg: 6}}>
                    <CallRobotItem />
                </Grid>
                <Grid size={{md: 12, lg: 6}}>
                    <CallRobotItem />
                </Grid>
            </Grid>
        </Box>
    );
};