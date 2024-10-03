

export abstract class LoginHandler {
    public onReadyCallback: () => void;

    constructor(onLoginHandlerReadyCallback: () => void) {
        this.onReadyCallback = onLoginHandlerReadyCallback;
    }

    public abstract loginState(): string;
}
