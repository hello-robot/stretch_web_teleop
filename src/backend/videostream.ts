import * as jpeg from 'jpeg-js';
const { RTCVideoSource, rgbaToI420 } = require('wrtc').nonstandard;


export class VideoStream {
    source
    track

    constructor() {
        this.source = new RTCVideoSource();
        this.track = this.source.createTrack();
    }

    uncompressAndSendImage(compressed_image) {
        if (this.source) {
            let {width, height, data} = jpeg.decode(compressed_image, {useTArray: true});
            let rgbaFrame = {
                width,
                height,
                data: data
            }
            let i420Frame = {
                width,
                height,
                data: new Uint8ClampedArray(1.5 * width * height)
            };
            rgbaToI420(rgbaFrame, i420Frame);
            this.source.onFrame(i420Frame);
        }
    }

}
