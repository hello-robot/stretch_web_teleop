import ROSLIB from "roslib";
import { uuid } from "utils/util";

export type Responder = () => Promise<{}>;

export interface Request {
    type: "request",
    requestType: string,
    id: uuid,
    data: any,
}

export interface Response {
    type: "response",
    requestType: string,
    id: uuid,
    data?: any,
}

export type pgmArray = (-1 | 0 | 100)[];

export interface mapView {
    mapData: pgmArray,
    mapWidth: number,
    mapHeight: number,
    mapResolution: number,
    mapOrigin: ROSLIB.Pose
}