export type cmd = DriveCommand

export interface VelocityCommand { stop: () => void }

interface GeneralCommand {
    msg: string
}

export interface DriveCommand {
    type: "driveBase",
    modifier: {
        linVel: number,
        angVel: number
    }
}