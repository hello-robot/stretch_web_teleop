// This component comes from the template at:
// https://github.com/mui/material-ui/tree/v6.1.5/docs/data/material/getting-started/templates/sign-in

import "home/css/LoginView.css";
import React, { useEffect, useState } from "react";
import { isTablet, isBrowser } from "react-device-detect";
import Box from '@mui/material/Box';
import MuiCard from '@mui/material/Card';
import Typography from '@mui/material/Typography';
import FormControl from '@mui/material/FormControl';
import FormLabel from '@mui/material/FormLabel';
import FormControlLabel from '@mui/material/FormControlLabel';
import TextField from '@mui/material/TextField';
import Link from '@mui/material/Link';
import Checkbox from '@mui/material/Checkbox';
import Button from '@mui/material/Button';
import Snackbar from '@mui/material/Snackbar';
import { styled } from '@mui/material/styles';
import { ForgotPassword } from "./ForgotPassword";
import { loginHandler } from "../index";


const Card = styled(MuiCard)(({ theme }) => ({
    display: 'flex',
    flexDirection: 'column',
    alignSelf: 'center',
    width: '100%',
    padding: theme.spacing(4),
    gap: theme.spacing(2),
    margin: 'auto',
    [theme.breakpoints.up('sm')]: {
      maxWidth: '450px',
    },
    boxShadow:
      'hsla(220, 30%, 5%, 0.05) 0px 5px 15px 0px, hsla(220, 25%, 10%, 0.05) 0px 15px 35px -5px',
    ...theme.applyStyles('dark', {
      boxShadow:
        'hsla(220, 30%, 5%, 0.5) 0px 5px 15px 0px, hsla(220, 25%, 10%, 0.08) 0px 15px 35px -5px',
    }),
}));


const SignInError = styled(Box)(({ theme }) => ({
    backgroundColor: '#d32f2f',
    margin: `calc(-1 * ${theme.spacing(4)})`,
    marginBottom: 0,
    padding: 10,
    color: "white",
}));


export const LoginView = (props) => {
    const [emailError, setEmailError] = useState(false);
    const [emailErrorMessage, setEmailErrorMessage] = useState('');
    const [passwordError, setPasswordError] = useState(false);
    const [passwordErrorMessage, setPasswordErrorMessage] = useState('');
    const [open, setOpen] = useState(false);
    const [openToast, setOpenToast] = useState(false);
    const [openFailureToast, setOpenFailureToast] = useState(false);
    const [failureToastMessage, setfailureToastMessage] = useState('');
    const [failureLogin, setfailureLogin] = useState(false);

    const handleClickOpen = () => {
        setOpen(true);
    };

    const handleForgotPassword = (email: string) => {
        loginHandler.forgot_password(email)
            .then(() => {
                setOpenToast(true);
            })
            .catch((error) => {
                setfailureToastMessage(`Please contact Hello Robot Support. ERROR ${error.code}: ${error.message}`);
                setOpenFailureToast(true);
            });
    };

    const handleClose = () => {
        setOpen(false);
    };

    const handleToastClose = () => {
        setOpenToast(false);
    };

    const handleSubmit = (event: React.FormEvent<HTMLFormElement>) => {
        event.preventDefault();
        if (emailError || passwordError) {
            return;
        }

        const data = new FormData(event.currentTarget);
        let l = {
            email: data.get('email') as string,
            password: data.get('password') as string,
            remember: data.get('remember') ? true : false,
        };
        loginHandler.login(l['email'], l['password'], l['remember'])
            .then(() => {
                // reset failure messages
                setfailureLogin(false);
                setOpenFailureToast(false);
                setfailureToastMessage('');
            })
            .catch((error) => {
                if (error.code === 'auth/invalid-login-credentials') {
                    setfailureLogin(true);
                } else if (error.code === 'auth/user-not-found') {
                    setfailureLogin(true);
                } else if (error.code === 'auth/invalid-email') {
                    setfailureLogin(true);
                } else {
                    setfailureToastMessage(`Please contact Hello Robot Support. ERROR ${error.code}: ${error.message}`);
                    setOpenFailureToast(true);
                }
            });
    };

    const validateInputs = () => {
        const email = document.getElementById('email') as HTMLInputElement;
        const password = document.getElementById('password') as HTMLInputElement;
    
        let isValid = true;
    
        if (!email.value || !/\S+@\S+\.\S+/.test(email.value)) {
          setEmailError(true);
          setEmailErrorMessage('Please enter a valid email address.');
          isValid = false;
        } else {
          setEmailError(false);
          setEmailErrorMessage('');
        }
    
        if (!password.value || password.value.length < 6) {
          setPasswordError(true);
          setPasswordErrorMessage('Password must be at least 6 characters long.');
          isValid = false;
        } else {
          setPasswordError(false);
          setPasswordErrorMessage('');
        }
    
        return isValid;
    };

    return isTablet || isBrowser ? (
        <Box
          display="flex"
          justifyContent="center"
          alignItems="center"
          minHeight="100vh">
            <Card variant="outlined">
                { failureLogin ? (
                    <SignInError>
                        Incorrect email or password
                    </SignInError>
                ) : (
                    <></>
                )}
                <Typography
                  component="h1"
                  variant="h4"
                  sx={{ width: '100%', fontSize: 'clamp(2rem, 10vw, 2.15rem)' }}>
                    Sign in
                </Typography>
                <Box
                  component="form"
                  onSubmit={handleSubmit}
                  noValidate
                  sx={{
                      display: 'flex',
                      flexDirection: 'column',
                      width: '100%',
                      gap: 2,
                  }}>
                    <FormControl>
                        <FormLabel htmlFor="email">Email</FormLabel>
                        <TextField
                          error={emailError}
                          helperText={emailErrorMessage}
                          id="email"
                          type="email"
                          name="email"
                          placeholder="your@email.com"
                          autoComplete="email"
                          autoFocus
                          required
                          fullWidth
                          variant="outlined"
                          color={emailError ? 'error' : 'primary'}
                          sx={{ ariaLabel: 'email' }} />
                    </FormControl>
                    <FormControl>
                        <Box sx={{ display: 'flex', justifyContent: 'space-between' }}>
                            <FormLabel htmlFor="password">Password</FormLabel>
                            <Link
                              component="button"
                              type="button"
                              onClick={handleClickOpen}
                              variant="body2"
                              sx={{ alignSelf: 'baseline' }}>
                                Forgot your password?
                            </Link>
                        </Box>
                        <TextField
                          error={passwordError}
                          helperText={passwordErrorMessage}
                          name="password"
                          placeholder="••••••••••••"
                          type="password"
                          id="password"
                          autoComplete="current-password"
                          required
                          fullWidth
                          variant="outlined"
                          color={passwordError ? 'error' : 'primary'} />
                    </FormControl>
                    <FormControlLabel
                      control={<Checkbox name="remember" value="remember" color="primary" defaultChecked />}
                      label="Remember me" />
                    <ForgotPassword open={open} handleClose={handleClose} handleExecute={handleForgotPassword} />
                    <Button
                      type="submit"
                      fullWidth
                      variant="contained"
                      onClick={validateInputs}>
                        Sign in
                    </Button>
                </Box>
            </Card>
            <Snackbar
              anchorOrigin={{ vertical: "bottom", horizontal: "center" }}
              open={openToast}
              onClose={handleToastClose}
              autoHideDuration={6000}
              message="We'll send you a password reset email soon" />
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
        <p>Not implemented</p>
    );
};
