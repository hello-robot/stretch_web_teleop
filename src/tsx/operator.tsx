import React from "react";
import { VelocityControl, DEFAULT_SPEED } from "./velocitycontrol"
import { VideoStreams } from "./videostreams";

/** Internal state of Operator */
interface OperatorState {
    /** Current speed of the robot */
    speed: number;
}

/** Operator internface webpage */
export class Operator extends React.Component<any, OperatorState> {
    constructor(props: {}) {
        super(props);
        this.state = {
            speed: DEFAULT_SPEED,
        }

        this.speedChange = this.speedChange.bind(this);
    }
    
    /**
     * Records the new speed
     * @param e the change event when a new speed is clicked
     */
    speedChange(e: React.ChangeEvent<HTMLInputElement>) {
        this.setState({speed: +e.target.value});
    }

    render() {
        return (
            <main style={{display: "flex", flexFlow: "column", height: "95%"}}>
                <div style={{width: "100%", flex: "0 1 auto"}}>
                    <VelocityControl 
                        currentSpeed={this.state.speed} 
                        onChange={this.speedChange}
                    />
                </div>
                <div style={{flex: "1 1 auto"}}>
                    <VideoStreams />
                </div>
            </main>
        )
    }
}