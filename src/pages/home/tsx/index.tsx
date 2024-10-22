import 'home/css/index.css';
import React from "react";
import { createRoot } from "react-dom/client";
import { createLoginHandler } from './utils';
import { LoginHandler } from './login_handler/LoginHandler';
import { SideBySideView } from './components/SideBySideView';

export let loginHandler: LoginHandler;
const container = document.getElementById("root");
const root = createRoot(container!);

const loginHandlerReadyCallback = () => {
    renderHomePage();
};
loginHandler = createLoginHandler(loginHandlerReadyCallback);


function renderHomePage() {
    loginHandler.loginState() == "authenticated"
        ? root.render(
              <SideBySideView />
          )
        : root.render(
              <div>Not Logged In</div>
          );
}
