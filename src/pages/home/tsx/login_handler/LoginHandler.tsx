

export abstract class LoginHandler {
    public onReadyCallback: () => void;

    constructor(onLoginHandlerReadyCallback: () => void) {
        this.onReadyCallback = onLoginHandlerReadyCallback;
    }

    public abstract loginState(): string;

    public abstract listRooms(): Promise<boolean>;

    public abstract logout();

    public abstract login(username: string, password: string, remember_me: boolean);
}
