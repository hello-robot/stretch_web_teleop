import ROSLIB from "roslib";
import { ArucoMarkersInfo } from "shared/util";

export const ARUCO_MARKER_INFO: ArucoMarkersInfo = {
    aruco_marker_info: {
        '130': {
            length_mm: 47,
            use_rgb_only: false,
            name: 'base_left',
            link: 'link_aruco_left_base'
        },
        '131': {
            length_mm: 47,
            use_rgb_only: false,
            name: 'base_right',
            link: 'link_aruco_right_base'
        },
        '132': {
            length_mm: 23.5,
            use_rgb_only: false,
            name: 'wrist_inside',
            link: 'link_aruco_inner_wrist'
        },
        '133': {
            length_mm: 23.5,
            use_rgb_only: false,
            name: 'wrist_top',
            link: 'link_aruco_top_wrist'
        },
        '134': {
            length_mm: 31.4,
            use_rgb_only: false,
            name: 'shoulder_top',
            link: 'link_aruco_shoulder'
        },
        '245': {
            length_mm: 88.0, 
            use_rgb_only: false,
            name: 'docking_station',
            link: "None",
            pose: {
                transform: new ROSLIB.Transform({
                    translation: {
                        x: 0.0,
                        y: -0.5,
                        z: 0.47
                    },
                    rotation: {
                        x: -0.382,
                        y: -0.352,
                        z: -0.604,
                        w: 0.604
                    }
                })
            }
        },
        '246': {
            length_mm: 179.0,
            use_rgb_only: false,
            name: 'floor_0',
            link: "None"
        },
        '247': {
            length_mm: 179.0,
            use_rgb_only: false,
            name: 'floor_1',
            link: "None"
        },
        '248': {
            length_mm: 179.0,
            use_rgb_only: false,
            name: 'floor_2',
            link: "None"
        },
        '249': {
            length_mm: 179.0,
            use_rgb_only: false,
            name: 'floor_3',
            link: "None"
        },
        '10': {
            length_mm: 24,
            use_rgb_only: false,
            name: 'target_object',
            link: "None"
        },
        '21': {
            length_mm: 86,
            use_rgb_only: false,
            name: 'user_pointer',
            link: "None"
        },
        'default': {
            length_mm: 24,
            use_rgb_only: false,
            name: 'unknown',
            link: "None"
        },
        // "0": {
        //     length_mm: 40, 
        //     use_rgb_only: false, 
        //     name: "Brush",
        //     link: "None",
        //     pose: {
        //         transform: new ROSLIB.Transform({
        //             translation: {
        //                 x: 0.0,
        //                 y: -0.6165221897614768,
        //                 z:-0.8765156889436512
        //             },
        //             rotation: {
        //                 x: -0.03965766243238807,
        //                 y: 0.04420070564372583,
        //                 z: 0.9982143182308036,
        //                 w: 0.006462207990295707
        //             }
        //         })
        //     }
        // },
        // "1": {
        //     length_mm: 22,
        //     use_rgb_only: false,
        //     name: "Feeding Tool",
        //     link: "None",
        //     pose: {
        //         transform: new ROSLIB.Transform({
        //             translation: {
        //                 x: 0.0,
        //                 y: -0.6165221897614768,
        //                 z: -0.8765156889436512
        //             },
        //             rotation: {
        //                 x: -0.03965766243238807,
        //                 y: 0.04420070564372583,
        //                 z: 0.9982143182308036,
        //                 w: 0.006462207990295707
        //             }
        //         })
        //     }
        // },
        // "2": {
        //     length_mm: 40, 
        //     use_rgb_only: false, 
        //     name: "Button Pusher",
        //     link: "None",
        //     pose: {
        //         transform: new ROSLIB.Transform({
        //             translation: {
        //                 x: 0.0,
        //                 y: -0.6165221897614768,
        //                 z:-0.8765156889436512
        //             },
        //             rotation: {
        //                 x: -0.03965766243238807,
        //                 y: 0.04420070564372583,
        //                 z: 0.9982143182308036,
        //                 w: 0.006462207990295707
        //             }
        //         })
        //     }
        // },
        "20": {
            length_mm: 68, 
            use_rgb_only: false, 
            name: "Tool Shelf",
            link: "None",
            pose: {
                transform: new ROSLIB.Transform({
                    translation: {
                        x: 0.0,
                        y: -1.2395535657717278,
                        z: 0.655349797002288
                    },
                    rotation: {
                        x: -0.007660462539611933,
                        y: 0.720369323410184,
                        z: 0.6932368040571439,
                        w: -0.020787303947235967
                    }
                })
            }
        },
    }
}