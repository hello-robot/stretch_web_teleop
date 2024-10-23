import "home/css/SideBySideView.css";
import React, { useEffect, useState } from "react";
import { isTablet, isBrowser } from "react-device-detect";
import Box from '@mui/material/Box';
import Grid from '@mui/material/Grid2';
import AppBar from '@mui/material/AppBar';
import Toolbar from '@mui/material/Toolbar';
import Typography from '@mui/material/Typography';
import Button from '@mui/material/Button';
import Snackbar from '@mui/material/Snackbar';
import { Changelog } from "./Changelog";
import { CallRobotSelector } from "./CallRobotSelector";
import { loginHandler } from "../index";


export const SideBySideView = (props) => {
    const [openFailureToast, setOpenFailureToast] = useState(false);
    const [failureToastMessage, setfailureToastMessage] = useState('');

    const handleLogout = () => {
        loginHandler.logout()
            .catch((error) => {
                setfailureToastMessage(`Please contact Hello Robot Support. ERROR ${error.code}: ${error.message}`);
                setOpenFailureToast(true);
            });
    };

    return isTablet || isBrowser ? (
        <Box sx={{ flexGrow: 1 }}>
            <AppBar position="static" color="transparent" elevation={0}>
                <Toolbar>
                    <Typography variant="h4" component="div" sx={{ flexGrow: 1 }}>
                        Stretch Web Teleop
                    </Typography>
                    <Button color="inherit" onClick={handleLogout}>Logout</Button>
                </Toolbar>
            </AppBar>
            <Grid container rowSpacing={1} columnSpacing={{ lg: 4, xl: 5 }} className='sbs-container'>
                <Grid size={{ md: 12, lg: 6 }}>
                    <Changelog style={{ height: "500px", maxHeight: "500px" }} />
                </Grid>
                <Grid size={{ md: 12, lg: 6 }}>
                    <CallRobotSelector style={{ height: "500px", maxHeight: "500px" }} />
                </Grid>
            </Grid>
            <Snackbar
              anchorOrigin={{ vertical: "bottom", horizontal: "center" }}
              open={openFailureToast}
              message={failureToastMessage}
              ContentProps={{
                sx: {
                  background: "red"
                }
              }} />
        </Box>
    ) : (
        <Box sx={{ flexGrow: 1 }}>
            <AppBar position="static" color="transparent" elevation={0}>
                <Toolbar>
                    <Typography variant="h4" component="div" sx={{ flexGrow: 1 }}>
                        Stretch Web Teleop
                    </Typography>
                    <Button color="inherit" onClick={handleLogout}>Logout</Button>
                </Toolbar>
            </AppBar>
            <Grid container spacing={2} className='sbs-container'>
                <Grid size={12}>
                    <CallRobotSelector style={{ height: "500px", maxHeight: "500px" }} />
                </Grid>
                <Grid size={12}>
                    <Changelog style={{ height: "500px", maxHeight: "500px" }} />
                </Grid>
            </Grid>
            <Snackbar
              anchorOrigin={{ vertical: "bottom", horizontal: "center" }}
              open={openFailureToast}
              message={failureToastMessage}
              ContentProps={{
                sx: {
                  background: "red"
                }
              }} />
        </Box>
    );
};
