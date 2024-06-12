import * as jpeg from "jpeg-js";
// const { RTCVideoSource, rgbaToI420 } = require('wrtc').nonstandard;
import * as WebRTC from "wrtc";

export class VideoStream {
  source;
  track;
  stream;

  constructor() {
    this.source = new WebRTC.nonstandard.RTCVideoSource();
    this.track = this.source.createTrack();
    this.stream = new WebRTC.MediaStream([this.track]);
    console.log(this.stream);
  }

  uncompressAndSendImage(compressed_image) {
    if (this.source != undefined) {
      let { width, height, data } = jpeg.decode(compressed_image, {
        useTArray: true,
      });
      let rgbaFrame = {
        width,
        height,
        data: data,
      };
      let i420Frame = {
        width,
        height,
        data: new Uint8ClampedArray(1.5 * width * height),
      };
      WebRTC.nonstandard.rgbaToI420(rgbaFrame, i420Frame);
      this.source.onFrame(i420Frame);
    }
  }
}
