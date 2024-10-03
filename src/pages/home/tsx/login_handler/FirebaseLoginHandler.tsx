import { LoginHandler } from "./LoginHandler";

export class FirebaseLoginHandler extends LoginHandler {

    constructor(onLoginHandlerReadyCallback: () => void, config) {
        super(onLoginHandlerReadyCallback);

        // Allow the initialization process to complete before invoking the callback
        setTimeout(() => {
            this.onReadyCallback();
        }, 0);
    }

    public loginState(): string {
        return "authenticated";
    }
}
