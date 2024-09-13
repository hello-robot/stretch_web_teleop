import { SignallingMessage } from "shared/util";


export interface SignalingProps {
    onSignal: (SignallingMessage) => void;
    onGoodbye?: () => void;
}


export abstract class BaseSignaling {
    public onSignal: (SignallingMessage) => void;
    public onGoodbye?: () => void;

    constructor(props: SignalingProps) {
        this.onSignal = props.onSignal;
        this.onGoodbye = props.onGoodbye;
    }

    /**
     * Joins the signaling room as a robot
     */
    public abstract join_as_robot(): Promise<boolean>;

    /**
     * Joins the signaling room as an operator
     */
    public abstract join_as_operator(): Promise<boolean>;

    /**
     * Leaves the signaling room as a robot
     */
    public abstract leave_as_robot(): Promise<boolean>;

    /**
     * Leaves the signaling room as an operator
     */
    public abstract leave_as_operator(): Promise<boolean>;
}