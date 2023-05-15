export type cmd = GeneralCommand | VelocityCommand

interface GeneralCommand {
    msg: string
}

export interface VelocityCommand {
    type: "driveBase",
    modifier: {
        linVel: number,
        angVel: number
    }
}