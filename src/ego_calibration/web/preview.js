'use strict';

// Multipart boundaries and JPEG bodies can be split across arbitrary reads.
class MjpegFrames {
  constructor() {this.buffer = new Uint8Array(); this.length = null;}
  push(chunk) {
    const combined = new Uint8Array(this.buffer.length + chunk.length);
    combined.set(this.buffer); combined.set(chunk, this.buffer.length);
    this.buffer = combined;
    let latest = null;
    while (this.buffer.length) {
      if (this.length === null) {
        let end = -1;
        for (let i=0; i+3<this.buffer.length; i++) {
          if (this.buffer[i]===13 && this.buffer[i+1]===10 && this.buffer[i+2]===13 && this.buffer[i+3]===10) {end=i; break;}
        }
        if (end < 0) {
          if (this.buffer.length > 1024) throw new Error('预览响应头无效');
          break;
        }
        const header = new TextDecoder().decode(this.buffer.subarray(0,end));
        const match = header.match(/\r\nContent-Length:\s*(\d+)\s*$/im);
        const length = match ? Number(match[1]) : 0;
        if (!header.startsWith('--frame\r\n') || !Number.isSafeInteger(length) || length < 1 || length > 8*1024*1024) throw new Error('预览帧大小无效');
        this.length = length;
        this.buffer = this.buffer.subarray(end+4);
      }
      if (this.buffer.length < this.length+2) break;
      if (this.buffer[this.length]!==13 || this.buffer[this.length+1]!==10) throw new Error('预览帧格式无效');
      latest = this.buffer.subarray(0,this.length);
      this.buffer = this.buffer.subarray(this.length+2);
      this.length = null;
    }
    // Drop older complete frames if the browser received a backlog in one read.
    return latest;
  }
}

class CameraPreview {
  constructor(canvas, onFrame, onError) {
    this.canvas=canvas; this.onFrame=onFrame; this.onError=onError;
    this.controller=null; this.times=[]; this.retryAt=0;
  }
  get fps() {
    const now=performance.now();
    this.times=this.times.filter(time=>now-time<=2000);
    if (this.times.length<2 || now-this.times.at(-1)>1000) return 0;
    return (this.times.length-1)*1000/(this.times.at(-1)-this.times[0]);
  }
  update(wanted) {
    if (!wanted) {this.stop(); return;}
    if (!this.controller && performance.now()>=this.retryAt) this.start();
  }
  stop() {
    this.controller?.abort(); this.controller=null; this.times=[];
  }
  async start() {
    const controller=new AbortController();
    this.controller=controller;
    let reader;
    try {
      const response=await fetch('/api/stream.mjpg', {signal:controller.signal, cache:'no-store'});
      if (!response.ok || !response.body) throw new Error('预览连接失败');
      reader=response.body.getReader();
      const parser=new MjpegFrames();
      while (!controller.signal.aborted) {
        const {done,value}=await reader.read();
        if (done) break;
        const frame=parser.push(value);
        if (!frame) continue;
        const bitmap=await createImageBitmap(new Blob([frame], {type:'image/jpeg'}));
        try {
          if (controller.signal.aborted) break;
          await new Promise(resolve=>{
            const cancel=()=>{cancelAnimationFrame(id); resolve();};
            const id=requestAnimationFrame(time=>{
              controller.signal.removeEventListener('abort',cancel);
              if (!controller.signal.aborted) {
                if (this.canvas.width!==bitmap.width || this.canvas.height!==bitmap.height) {
                  this.canvas.width=bitmap.width; this.canvas.height=bitmap.height;
                }
                this.canvas.getContext('2d', {alpha:false}).drawImage(bitmap,0,0);
                this.times.push(time);
                this.onFrame(this.fps);
              }
              resolve();
            });
            controller.signal.addEventListener('abort',cancel,{once:true});
          });
        } finally {bitmap.close();}
      }
    } catch(error) {
      if (!controller.signal.aborted) this.onError(error.message);
    } finally {
      if (reader) await reader.cancel().catch(()=>{});
      if (this.controller===controller) {
        this.controller=null; this.retryAt=performance.now()+750;
      }
    }
  }
}

if (typeof module!=='undefined') module.exports={MjpegFrames};
