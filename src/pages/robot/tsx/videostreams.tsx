import React from "react";
import { ROSCompressedImage } from "shared/util";
var jpeg = require("jpeg-js");

type VideoStreamProps = {
  width: number;
  height: number;
  scale: number;
  fps: number;
};

export class VideoStream extends React.Component<VideoStreamProps> {
  canvas = React.createRef<HTMLCanvasElement>();
  img: HTMLImageElement;
  video: HTMLVideoElement;
  width: number;
  height: number;
  scale: number;
  fps: number;
  className?: string;
  outputVideoStream?: MediaStream;
  aspectRatio: any;

  constructor(props: VideoStreamProps) {
    super(props);
    this.width = props.width;
    this.height = props.height;
    this.scale = props.scale;
    this.fps = props.fps;
    this.img = document.createElement("img");
    this.video = document.createElement("video");
    this.video.style.display = "block";
    this.video.setAttribute("width", this.width.toString());
    this.video.setAttribute("height", this.height.toString());
    this.outputVideoStream = new MediaStream();

    this.updateImage = this.updateImage.bind(this);
  }

  get imageReceived() {
    return this.img.src !== "";
  }

  renderVideo() {
    if (!this.imageReceived) {
      return;
    }
    this.canvas.current
      ?.getContext("2d")
      ?.drawImage(this.img, 0, 0, this.width, this.height);
  }

  updateImage(message: ROSCompressedImage) {
    if (!this.imageReceived) {
      let { width, height, data } = jpeg.decode(
        Uint8Array.from(atob(message.data), (c) => c.charCodeAt(0)),
        true,
      );
      this.aspectRatio = width / height;
      this.height = Math.max(height * this.aspectRatio, 1000);
      this.width = this.height * this.aspectRatio;
      this.canvas.current!.width = this.width;
      this.canvas.current!.height = this.height;
    }
    this.img.src = "data:image/jpg;base64," + message.data;
  }

  drawVideo() {
    this.renderVideo();
    requestAnimationFrame(this.drawVideo.bind(this));
  }

  start() {
    if (!this.canvas.current) throw "Video stream canvas null";
    this.outputVideoStream = this.canvas.current.captureStream(this.fps);
    this.video.srcObject = this.outputVideoStream;
    this.drawVideo();
  }

  render() {
    return (
      <canvas
        ref={this.canvas!}
        width={this.width}
        height={this.height}
        className={this.className}
      />
    );
  }
}

/** Renders all three video streams side by side */
export const AllVideoStreamComponent = (props: { streams: VideoStream[] }) => {
  console.log(props.streams);
  // let buttonPads = Bp.ExampleButtonPads;
  // let buttonPads = [undefined, undefined, undefined];
  // Replace the overhead button pad with predictive display
  // buttonPads[0] = <PredictiveDisplay onClick={(len, ang) => console.log(`Length: ${len}, Angle: ${ang}`)} />;
  const widths = ["30%", "22.5%", "45%"];
  return (
    <div id="video-stream-container">
      {props.streams.map((stream, i) => (
        <div key={i} className="video-container" style={{ width: widths[i] }}>
          {stream.render()}
        </div>
      ))}
    </div>
  );
};
