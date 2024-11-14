// This component comes from the template at:
// https://github.com/mui/material-ui/tree/v6.1.5/docs/data/material/getting-started/templates/sign-in

import React, { useEffect, useState } from "react";
import { isTablet, isBrowser } from "react-device-detect";
import Dialog from "@mui/material/Dialog";
import DialogTitle from "@mui/material/DialogTitle";
import DialogContent from "@mui/material/DialogContent";
import DialogContentText from "@mui/material/DialogContentText";
import DialogActions from "@mui/material/DialogActions";
import OutlinedInput from "@mui/material/OutlinedInput";
import Button from "@mui/material/Button";

export const ForgotPassword = (props: {
    open: boolean;
    handleClose: () => void;
    handleExecute: (email: string) => void;
}) => {
    return isTablet || isBrowser ? (
        <Dialog
            open={props.open}
            onClose={props.handleClose}
            PaperProps={{
                component: "form",
                onSubmit: (event: React.FormEvent<HTMLFormElement>) => {
                    event.preventDefault();
                    const data = new FormData(event.currentTarget);
                    props.handleExecute(data.get("email") as string);
                    props.handleClose();
                    event.stopPropagation();
                },
            }}
        >
            <DialogTitle>Reset password</DialogTitle>
            <DialogContent>
                <DialogContentText>
                    Enter your account&apos;s email address, and we&apos;ll send
                    you a link to reset your password.
                </DialogContentText>
                <OutlinedInput
                    autoFocus
                    required
                    margin="dense"
                    id="email"
                    name="email"
                    placeholder="Email address"
                    type="email"
                    fullWidth
                />
            </DialogContent>
            <DialogActions sx={{ pb: 3, px: 3 }}>
                <Button onClick={props.handleClose}>Cancel</Button>
                <Button variant="contained" type="submit">
                    Continue
                </Button>
            </DialogActions>
        </Dialog>
    ) : (
        <p>Not implemented</p>
    );
};
