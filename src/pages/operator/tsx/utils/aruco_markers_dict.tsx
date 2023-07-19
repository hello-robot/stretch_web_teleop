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
            link: null
        },
        '246': {
            length_mm: 179.0,
            use_rgb_only: false,
            name: 'floor_0',
            link: null
        },
        '247': {
            length_mm: 179.0,
            use_rgb_only: false,
            name: 'floor_1',
            link: null
        },
        '248': {
            length_mm: 179.0,
            use_rgb_only: false,
            name: 'floor_2',
            link: null
        },
        '249': {
            length_mm: 179.0,
            use_rgb_only: false,
            name: 'floor_3',
            link: null
        },
        '10': {
            length_mm: 24,
            use_rgb_only: false,
            name: 'target_object',
            link: null
        },
        '21': {
            length_mm: 86,
            use_rgb_only: false,
            name: 'user_pointer',
            link: null
        },
        'default': {
            length_mm: 24,
            use_rgb_only: false,
            name: 'unknown',
            link: null
        },
        // 0: {
        //     length_mm: 47,
        //     use_rgb_only: false,
        //     name: 'light_switch',
        //     link: null
        // }
    }
}