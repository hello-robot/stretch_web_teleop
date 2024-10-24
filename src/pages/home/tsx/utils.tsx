import { FirebaseOptions } from "firebase/app";
import { FirebaseLoginHandler } from "./login_handler/FirebaseLoginHandler";
import { LocalLoginHandler } from "./login_handler/LocalLoginHandler";


/**
 * Creates a login handler based on the `storage` property in the process
 * environment.
 *
 * @returns the login handler
 */
export function createLoginHandler(loginHandlerReadyCallback: () => void) {
    switch (process.env.storage) {
        case "firebase":
            const config: FirebaseOptions = {
                apiKey: process.env.apiKey,
                authDomain: process.env.authDomain,
                databaseURL: process.env.databaseURL,
                projectId: process.env.projectId,
                storageBucket: process.env.storageBucket,
                messagingSenderId: process.env.messagingSenderId,
                appId: process.env.appId,
                measurementId: process.env.measurementId,
            };
            return new FirebaseLoginHandler(loginHandlerReadyCallback, config);
        default:
            return new LocalLoginHandler(loginHandlerReadyCallback);
    }
}
