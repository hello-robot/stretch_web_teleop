import { FirebaseOptions } from "firebase/app";
import { FirebaseSignaling } from "./FirebaseSignaling";
import { LocalSignaling } from "./LocalSignaling";
import { SignalingProps } from "./Signaling";

/**
 * Creates a signaling handler based on the `storage` property in the process
 * environment.
 *
 * @returns the signaler
 */
export function createSignaler(props: SignalingProps) {
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
            return new FirebaseSignaling(props, config);
        default:
            return new LocalSignaling(props);
    }
}
