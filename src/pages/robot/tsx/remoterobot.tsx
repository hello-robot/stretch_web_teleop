import React from 'react'
import { cmd } from 'utils/util';

export type robotMessageChannel = (message: cmd) => void;

export class RemoteRobot extends React.Component {
    robotChannel: robotMessageChannel;

    constructor(props: {robotChannel: robotMessageChannel}) {
        super(props);
        this.robotChannel = props.robotChannel
    }
}