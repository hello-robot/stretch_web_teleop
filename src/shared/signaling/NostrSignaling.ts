import { BaseSignaling } from "./Signaling";


export class NostrSignaling extends BaseSignaling {

    constructor() {
        super();
    }

    /**
     * Connects to the signaling server
     */
    public connect(): void {
        return;
    }
}