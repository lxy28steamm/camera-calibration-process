const {test}=require('node:test');
const assert=require('node:assert/strict');
const {MjpegFrames}=require('../src/ego_calibration/web/preview.js');

const part=data=>Buffer.concat([Buffer.from(`--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ${data.length}\r\n\r\n`),data,Buffer.from('\r\n')]);

test('multipart headers and JPEG bytes can arrive one byte at a time',()=>{
  const frame=Buffer.from([255,216,0,13,10,13,10,255,217]);
  const parser=new MjpegFrames(), frames=[];
  for (const byte of part(frame)) {
    const result=parser.push(Uint8Array.of(byte));
    if (result) frames.push(Buffer.from(result));
  }
  assert.deepEqual(frames,[frame]);
});

test('a backlog displays the newest complete frame and retains the partial next frame',()=>{
  const parser=new MjpegFrames();
  const next=part(Buffer.from('third'));
  assert.equal(Buffer.from(parser.push(Buffer.concat([part(Buffer.from('old')),part(Buffer.from('new')),next.subarray(0,next.length-3)]))).toString(),'new');
  assert.equal(Buffer.from(parser.push(next.subarray(next.length-3))).toString(),'third');
});

test('invalid sizes and malformed frames are rejected',()=>{
  for (const size of ['0','-1','999999999']) {
    assert.throws(()=>new MjpegFrames().push(Buffer.from(`--frame\r\nContent-Length: ${size}\r\n\r\n`)));
  }
  assert.throws(()=>new MjpegFrames().push(Buffer.from('--frame\r\nContent-Length: 1\r\n\r\nxZZ')));
  assert.throws(()=>new MjpegFrames().push(Buffer.alloc(2048,65)));
});
