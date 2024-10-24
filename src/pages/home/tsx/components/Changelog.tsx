import "home/css/Changelog.css";
import React, { useEffect, useState } from "react";
import Box from "@mui/material/Box";
import Grid from "@mui/material/Grid2";
import { styled } from "@mui/material/styles";

const LogItem = styled(Box)(({ theme }) => ({
    ...theme.typography.body2,
    paddingLeft: theme.spacing(1),
    color: theme.palette.text.secondary,
    fontSize: 17,
}));

export const Changelog = (props: { style?: React.CSSProperties }) => {
    return (
        <Box sx={{ flexGrow: 1 }}>
            <h2>What's new?</h2>
            <Grid
                container
                spacing={0}
                className="cv-container"
                style={props.style}
            >
                <Grid size={12}>
                    <LogItem>
                        <span>
                            <b style={{ fontWeight: "bolder" }}>Login Page</b> -
                            Oct 9th, 2024
                        </span>
                        <p>
                            The new homepage shows the robots you can call, or
                            as "unavailable" if the robot is powered off or
                            occupied by another operator. There's also a "What's
                            New" section with details about new features being
                            added to the web interface. By default, you are
                            logged in when running the interface locally, but
                            there's a login screen that can be accessed by
                            logging out. In the future, this login screen will
                            enable you to call your Stretch over the internet.
                        </p>
                    </LogItem>
                </Grid>
                <Grid size={12}>
                    <LogItem>
                        <span>
                            <b style={{ fontWeight: "bolder" }}>
                                Homing Button
                            </b>{" "}
                            - Sept 21st, 2024
                        </span>
                        <p>
                            The operator interface now shows an banner if your
                            robot needs to be homed. Since some of Stretch's
                            encoders are relative, there's a homing sequence to
                            find zero for those joints when Stretch wakes up.
                            Previously, developers had to use a terminal to
                            trigger Stretch's homing sequence, but now you can
                            do it through the web interface.
                        </p>
                    </LogItem>
                </Grid>
                <Grid size={12}>
                    <LogItem>
                        <span>
                            <b style={{ fontWeight: "bolder" }}>
                                Lorem ipsum dolor et.
                            </b>{" "}
                            - May 18st, 2024
                        </span>
                        <p>
                            Lorem ipsum odor amet, consectetuer adipiscing elit.
                            Mattis purus potenti orci per torquent scelerisque.
                            Feugiat fringilla tristique varius feugiat quis cras
                            magnis efficitur. Aptent curabitur mattis dui congue
                            porta cubilia. Lorem scelerisque convallis tempor
                            himenaeos donec inceptos ultricies dis. Efficitur
                            feugiat senectus nullam semper conubia risus mi
                            volutpat.
                        </p>
                    </LogItem>
                </Grid>
                <Grid size={12}>
                    <LogItem>
                        <span>
                            <b style={{ fontWeight: "bolder" }}>
                                Lorem ipsum dolor et.
                            </b>{" "}
                            - Mar 2nd, 2024
                        </span>
                        <p>
                            Lorem ipsum odor amet, consectetuer adipiscing elit.
                            Mattis purus potenti orci per torquent scelerisque.
                            Feugiat fringilla tristique varius feugiat quis cras
                            magnis efficitur. Aptent curabitur mattis dui congue
                            porta cubilia. Lorem scelerisque convallis tempor
                            himenaeos donec inceptos ultricies dis. Efficitur
                            feugiat senectus nullam semper conubia risus mi
                            volutpat.
                        </p>
                    </LogItem>
                </Grid>
                <Grid size={12}>
                    <LogItem>
                        <span>
                            <b style={{ fontWeight: "bolder" }}>
                                Lorem ipsum dolor et.
                            </b>{" "}
                            - Jan 31st, 2024
                        </span>
                        <p>
                            Lorem ipsum odor amet, consectetuer adipiscing elit.
                            Mattis purus potenti orci per torquent scelerisque.
                            Feugiat fringilla tristique varius feugiat quis cras
                            magnis efficitur. Aptent curabitur mattis dui congue
                            porta cubilia. Lorem scelerisque convallis tempor
                            himenaeos donec inceptos ultricies dis. Efficitur
                            feugiat senectus nullam semper conubia risus mi
                            volutpat.
                        </p>
                    </LogItem>
                </Grid>
            </Grid>
        </Box>
    );
};
