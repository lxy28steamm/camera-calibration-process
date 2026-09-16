'use strict';
const dexForm = $('dex-settings');
let dexCatalog = {videos:[],results:[],flashes:[]}, dexYamlText='', dexProfileText='', dexReviewDirty=true;
let dexLastReview='', dexLastVideo='', dexResultKey='', dexFlashKey='', dexCatalogBusy=false, dexImageLoading=false, dexWasBusy=false;
const dexFileUrl = id => '/dex-files/'+id.split('/').map(encodeURIComponent).join('/');
const dexLinks = files => (files || []).map(file=>`<a class="button secondary" href="${dexFileUrl(file.id)}" target="_blank" rel="noopener">${esc(file.name)}</a>`).join('');
function dexSettings() {
  if(!dexForm.reportValidity()) return null;
  const values={};
  for(const input of dexForm.querySelectorAll('[name]')) values[input.name]=input.tagName==='SELECT'?input.value:Number(input.value);
  return values;
}
function dexInvalidateReview() {dexReviewDirty=true;$('dex-confirm-write').checked=false;renderDexControls();}
function renderDexControls() {
  const busy=!!state.busy || pending || uploading, dex=state.dex || {}, selected=!!state.capabilities?.mono, flash=!!state.capabilities?.flash;
  const recording=state.operation==='dex_record', solving=state.operation==='dex_solve', writing=state.operation==='dex_flash_write';
  for(const id of ['dex-preview','dex-record']) $(id).disabled=busy||!selected;
  for(const id of ['dex-flash-probe','dex-flash-read']) $(id).disabled=busy||!flash||!dex.environment?.flash_sdk;
  $('mono-verify').disabled=busy||!($('dex-video').value||dex.video_id)||!(dexYamlText||$('dex-yaml-source').value);
  for(const id of ['dex-import-video','dex-import-yaml','dex-import-profile','dex-reset-profile','dex-check-yaml']) $(id).disabled=busy;
  $('dex-settings-fields').disabled=busy;
  $('dex-video').disabled=busy;$('dex-yaml-source').disabled=busy;
  $('dex-solve').disabled=busy||!($('dex-video').value||dex.video_id)||!dex.environment?.project_ready;
  $('dex-stop-stream').disabled=uploading||pending||writing||(!recording&&(busy||!dex.stream?.running));
  $('dex-cancel').classList.toggle('hidden',!solving);$('dex-cancel').disabled=pending;
  $('dex-flash-write').disabled=busy||!flash||!dex.environment?.flash_sdk||dexReviewDirty||!dex.review?.ready_to_write||!dex.review_token||!$('dex-confirm-write').checked;
  $('dex-confirm-write').disabled=busy||!flash||!dex.environment?.flash_sdk||dexReviewDirty||!dex.review?.ready_to_write;
  $('dex-use-result').disabled=busy||!dex.result?.yaml_id;
  document.querySelectorAll('[data-dex-yaml]').forEach(button=>button.disabled=busy);
}
function dexCameraSummary(camera) {
  if(!camera) return '';
  return `<dl class="dex-values"><div><dt>模型</dt><dd>${esc(camera.camera_model)} / ${esc(camera.distortion_model)}</dd></div><div><dt>标定分辨率</dt><dd>${esc(camera.resolution?.join(' × '))}</dd></div><div><dt>内参</dt><dd>${esc(camera.intrinsics?.map(v=>number(v,6)).join(', '))}</dd></div><div><dt>畸变系数</dt><dd>${esc(camera.distortion_coeffs?.map(v=>number(v,8)).join(', ') || '无附加畸变项')}</dd></div></dl>`;
}
window.renderDex = function() {
  const dex=state.dex || {}, env=dex.environment || {}, stream=dex.stream;
  badge($('dex-env-status'),env.project_ready&&env.ffmpeg?'pass':'incomplete',env.project_ready&&env.ffmpeg?'内置后端已就绪':'后端配置待检查');
  $('dex-environment').classList.toggle('hidden',!!env.project_ready&&!!env.ffmpeg);
  $('dex-environment').textContent=`后端目录：${env.project||'尚未读取'}。${!env.project_ready?'内置后端文件不完整，请检查安装包。':''}${!env.ffmpeg?'需要系统 FFmpeg / ffprobe。':''}`;
  $('dex-stream-info').textContent=stream ? `${stream.resolution?.join('×')||'尺寸待测'} · ${stream.recording?'录制':'预览'} ${number(stream.elapsed_s,1)} 秒${stream.running?'':' · 已停止'}` : '等待连接';
  $('dex-detection').textContent=stream?.error || stream?.detection || '支持 AprilGrid 与棋盘格';
  $('dex-record-progress').max=Number(dexForm.elements.duration_s.value);
  $('dex-record-progress').value=stream?.recording?stream.elapsed_s:0;
  if(dex.video_id&&dex.video_id!==dexLastVideo){dexLastVideo=dex.video_id;if(!Array.from($('dex-video').options).some(option=>option.value===dex.video_id))$('dex-video').add(new Option('本次录制 / 导入：'+dex.video_id,dex.video_id));$('dex-video').value=dex.video_id;}
  const result=dex.result, resultKey=JSON.stringify(result);
  if(resultKey!==dexResultKey){dexResultKey=resultKey;
    badge($('dex-result-status'),result?.status==='complete'?'pass':result?.status==='running'?'running':result?'incomplete':'',({complete:'求解完成 · 待精度复核',running:'正在求解',failed:'求解失败',cancelled:'已停止'})[result?.status]||'尚未求解');
    $('dex-result').innerHTML=result ? `${result.error?`<p class="alert error">${esc(result.error)}</p>`:''}${dexCameraSummary(result.camera)}${result.verification?`<p>模式：${esc(result.verification.mode)} · 有效 ${result.verification.accepted} 帧 · 留出角点 RMS ${number(result.verification.holdout_error_px?.rms,3)} px · ${esc(({pass:'指标通过',fail:'误差超限',incomplete:'检查不完整',calibrated:'已求解，需独立复核'})[result.verification.status]||result.verification.status)}</p><p>${esc(result.verification.scope)}</p>`:''}${result.yaml_text?`<details><summary>查看本次 YAML</summary><pre>${esc(result.yaml_text)}</pre></details>`:''}<div class="dex-file-links">${dexLinks(result.files)}</div>${result.status==='complete'?'<p class="form-note">这是新求解的单目参数。检查 PDF 中的覆盖与误差，并用独立画面验证；不会自动写入相机。</p>':''}` : '求解后展示实际分辨率、内参、畸变和可下载报告。';
    $('dex-use-result-row').classList.toggle('hidden',!result?.yaml_id);
  }
  if(dex.review_token!==dexLastReview){dexLastReview=dex.review_token||'';dexReviewDirty=!dex.review_token;$('dex-confirm-write').checked=false;const review=dex.review,profile=review?.reservation;
    if(review?.yaml_id&&!dexYamlText&&!$('dex-yaml-source').value){$('dex-yaml-source').add(new Option('已校验的 YAML 快照',review.yaml_id));$('dex-yaml-source').value=review.yaml_id;}
    $('dex-review').innerHTML=review ? `${dexCameraSummary(review.camera)}<dl class="dex-values"><div><dt>当前目标设备</dt><dd>${esc(state.selected?.kind==='dex-mono'?state.selected.label:'未选择 Dex，相机读写不可用')}</dd></div><div><dt>YAML SHA-256</dt><dd>${esc(review.yaml_sha256||'未提供 YAML')}</dd></div><div><dt>存储范围</dt><dd>${profile?`sector ${profile.first_sector}–${profile.first_sector+profile.sector_count-1} · ${profile.sector_count*4096} 字节`:'未校验区域'}</dd></div><div><dt>本次记录占用</dt><dd>${review.storage_bytes?`${review.storage_bytes} 字节 / ${review.required_sectors} 扇区`:'仅读取'}</dd></div></dl>${review.reservation_error?`<p class="alert warning">${esc(review.reservation_error)}</p>`:''}${review.yaml_text?`<details><summary>核对待写 YAML 原文</summary><pre>${esc(review.yaml_text)}</pre></details>`:''}` : '请选择结果或导入 YAML 后校验；只读取时可保留 YAML 为空。';
    $('dex-trial-note').classList.toggle('hidden',profile?.selection_mode!=='user_selected_blank');
    badge($('dex-flash-status'),review?.ready_to_read?'pass':'',review?.ready_to_write?'格式与区域已校验':review?.ready_to_read?'区域已校验，可读取':'未校验');
  }
  const flashKey=JSON.stringify(dex.flash_result);
  if(flashKey!==dexFlashKey){dexFlashKey=flashKey;const result=dex.flash_result;$('dex-flash-result').innerHTML=result?`<strong>${esc(result.status||'设备信息已读取')}</strong>${dexCameraSummary(result.camera)}${result.device?`<pre>${esc(JSON.stringify(result.device,null,2))}</pre>`:''}${result.yaml_text?`<details><summary>查看相机读回 YAML</summary><pre>${esc(result.yaml_text)}</pre></details>`:''}<div class="dex-file-links">${dexLinks(result.files)}</div>`:'';}
  const log=dex.log||'尚无任务日志';if($('dex-log').textContent!==log){const follow=$('dex-log').scrollHeight-$('dex-log').scrollTop-$('dex-log').clientHeight<60;$('dex-log').textContent=log;if(follow)$('dex-log').scrollTop=$('dex-log').scrollHeight;}
  if(dexWasBusy&&!state.busy)loadDexCatalog();dexWasBusy=!!state.busy;
  renderDexControls();
};
window.loadDexCatalog = async function() {
  if(dexCatalogBusy)return;dexCatalogBusy=true;
  try {dexCatalog=await api('/api/dex/catalog');const video=$('dex-video').value||state.dex?.video_id||'',yaml=$('dex-yaml-source').value;
    $('dex-video').innerHTML='<option value="">录制、导入或选择历史视频</option>'+dexCatalog.videos.map(item=>`<option value="${esc(item.id)}">${esc(item.name)} · ${(item.bytes/1024**2).toFixed(0)} MB · ${esc(item.status)}</option>`).join('');
    if(video&&!Array.from($('dex-video').options).some(option=>option.value===video))$('dex-video').add(new Option('本次导入：'+video,video));$('dex-video').value=video;
    $('dex-yaml-source').innerHTML='<option value="">不提供 YAML，仅校验区域 / 读回</option>'+dexCatalog.results.filter(item=>item.yaml_id).map(item=>`<option value="${esc(item.yaml_id)}">${esc(item.name)} · ${esc(item.status)}</option>`).join('');
    if(yaml&&!Array.from($('dex-yaml-source').options).some(option=>option.value===yaml))$('dex-yaml-source').add(new Option('本次结果',yaml));$('dex-yaml-source').value=yaml;
    $('dex-history').innerHTML=dexCatalog.results.length?dexCatalog.results.map(item=>`<article class="record"><div><h3>${esc(item.name)}</h3><p>${esc(item.model)} · ${esc(item.status)}（历史状态，不代表验收）</p><div class="dex-file-links">${dexLinks(item.files)}</div></div>${item.yaml_id?`<button class="button secondary" data-dex-yaml="${esc(item.yaml_id)}">载入 YAML</button>`:''}</article>`).join(''):'<p class="empty-text">尚无单目标定结果</p>';
    $('dex-flash-history').innerHTML=dexCatalog.flashes.length?dexCatalog.flashes.map(item=>`<article class="record"><div><h3>${esc(item.name)}</h3><p>${esc(item.status)}</p><div class="dex-file-links">${dexLinks(item.files)}</div></div></article>`).join(''):'<p class="empty-text">尚无 Flash 操作记录</p>';
    document.querySelectorAll('[data-dex-yaml]').forEach(button=>button.onclick=()=>useDexYaml(button.dataset.dexYaml));renderDexControls();
  }catch(error){toast(error.message);}finally{dexCatalogBusy=false;}
};
function useDexYaml(id){dexYamlText='';$('dex-yaml-name').textContent='';if(!Array.from($('dex-yaml-source').options).some(option=>option.value===id))$('dex-yaml-source').add(new Option('本次结果',id));$('dex-yaml-source').value=id;dexInvalidateReview();$('dex-flash-panel').scrollIntoView({behavior:'smooth',block:'start'});}
window.openDexPreview = function(){const config=dexSettings();if(config){action('dex_preview',{settings:config});location.hash='dex';}};
$('dex-preview').onclick=openDexPreview;
$('dex-record').onclick=()=>{const config=dexSettings();if(config)action('dex_record',{settings:config});};
$('dex-stop-stream').onclick=()=>action(state.operation==='dex_record'?'stop':'dex_close');
$('dex-solve').onclick=()=>{const config=dexSettings();if(config)action('dex_solve',{settings:config,video_id:$('dex-video').value||state.dex?.video_id});};
$('dex-cancel').onclick=()=>action('stop');$('dex-refresh').onclick=loadDexCatalog;
$('dex-video').onchange=renderDexControls;
dexForm.onsubmit=event=>event.preventDefault();
$('dex-board').onchange=()=>{const checker=$('dex-board').value==='checkerboard';$('dex-size-label').textContent=checker?'棋盘格边长 / mm':'标签边长 / mm';$('dex-gap-label').classList.toggle('hidden',checker);$('dex-board-note').textContent=checker?'行、列填写内角点数量，不是方格数。边长填写实物格子的测量值。':'AprilGrid · tag36h11 · 连续 ID 从 0 开始；标签边长按外黑边测量，间隙单独填写。';};
$('dex-import-video').onclick=()=>$('dex-video-file').click();
$('dex-video-file').onchange=async event=>{const file=event.target.files[0];if(!file)return;if(file.size>8*1024**3){toast('单目视频最大 8 GB；已有视频可直接从列表选择');return;}uploading=true;updateControls();renderDexControls();
  try {const result=await new Promise((resolve,reject)=>{const xhr=new XMLHttpRequest();xhr.open('POST','/api/upload?purpose=dex&name='+encodeURIComponent(file.name));xhr.setRequestHeader('X-Camera-Token',token);xhr.upload.onprogress=e=>{$('dex-upload-progress').textContent=e.lengthComputable?`上传 ${Math.round(e.loaded/e.total*100)}%`:'正在上传';};xhr.onload=()=>{try{const data=JSON.parse(xhr.responseText);xhr.status>=400?reject(new Error(data.error)):resolve(data);}catch{reject(new Error('上传失败'));}};xhr.onerror=()=>reject(new Error('上传连接中断'));xhr.send(file);});uploading=false;await action('dex_import_video',{upload:result.upload});}
  catch(error){toast(error.message);}finally{uploading=false;event.target.value='';$('dex-upload-progress').textContent='';updateControls();renderDexControls();}
};
$('dex-import-yaml').onclick=()=>$('dex-yaml-file').click();$('dex-import-profile').onclick=()=>$('dex-profile-file').click();
$('dex-yaml-file').onchange=async event=>{const file=event.target.files[0];if(!file)return;if(file.size>1024**2){toast('YAML 不得超过 1 MB');return;}dexYamlText=await file.text();$('dex-yaml-name').textContent=file.name;$('dex-yaml-source').value='';dexInvalidateReview();event.target.value='';};
$('dex-profile-file').onchange=async event=>{const file=event.target.files[0];if(!file)return;if(file.size>128*1024){toast('区域配置不得超过 128 KB');return;}dexProfileText=await file.text();$('dex-profile-name').textContent=file.name;dexInvalidateReview();event.target.value='';};
$('dex-reset-profile').onclick=()=>{dexProfileText='';$('dex-profile-name').textContent='内置 sector 126 试写配置（未经厂家确认）';dexInvalidateReview();};
$('dex-yaml-source').onchange=()=>{dexYamlText='';$('dex-yaml-name').textContent='';dexInvalidateReview();};
$('dex-check-yaml').onclick=()=>{dexInvalidateReview();action('dex_inspect_yaml',{yaml_text:dexYamlText,yaml_id:$('dex-yaml-source').value,profile_text:dexProfileText});};
$('dex-use-result').onclick=()=>useDexYaml(state.dex.result.yaml_id);
$('dex-flash-probe').onclick=()=>action('dex_flash_probe');
$('dex-flash-read').onclick=()=>{if(dexReviewDirty&&(dexProfileText||dexYamlText||$('dex-yaml-source').value)){toast('输入已改变，请先点击“校验 YAML 与区域”');return;}action('dex_flash_read');};
$('dex-confirm-write').onchange=renderDexControls;
$('dex-flash-write').onclick=()=>{if(dexReviewDirty||!$('dex-confirm-write').checked)return;const reviewToken=state.dex.review_token;$('dex-confirm-write').checked=false;action('dex_flash_write',{confirmed:true,review_token:reviewToken});};
$('dex-image').onload=()=>{dexImageLoading=false;$('dex-image').classList.remove('hidden');$('dex-camera-empty').classList.add('hidden');};
$('dex-image').onerror=()=>{dexImageLoading=false;};
setInterval(()=>{if(document.hidden||location.hash!=='#dex'||dexImageLoading)return;if(state.dex?.stream?.preview_frames){dexImageLoading=true;$('dex-image').src='/api/dex/frame.jpg?t='+Date.now();}else{$('dex-image').classList.add('hidden');$('dex-camera-empty').classList.remove('hidden');}},250);
if(location.hash==='#dex')loadDexCatalog();

$('mono-verify').onclick=()=>{const config=dexSettings();if(config)action('dex_solve',{settings:config,video_id:$('dex-video').value||state.dex?.video_id,verify:true,yaml_text:dexYamlText,yaml_id:$('dex-yaml-source').value});};
let healthKey='';
window.renderUniversal=function(){
  const caps=state.capabilities||{}, busy=!!state.busy||pending||uploading;
  $('camera-layout').disabled=busy||!caps.select_layout;
  $('camera-layout').value=caps.stereo?'stereo':'mono';
  $('capability-note').textContent=`${caps.mono?'单目采集 / 标定 / 视频复测':caps.stereo?'双目预览 / 几何复测':'请选择设备'} · ${caps.read_calibration?'支持设备标定读取':'通过文件导入标定'}${caps.flash?' · Dex 专用 Flash':''}`;
  $('health-start').disabled=busy||!caps.health;
  $('health-stop').classList.toggle('hidden',state.operation!=='health');
  const h=state.health,key=JSON.stringify(h);
  if(key!==healthKey){healthKey=key;$('health-result').innerHTML=h?`<strong>${esc(({complete:'检查结束',running:'检查中',failed:'检查失败',cancelled:'已停止'})[h.status]||h.status)}</strong> · 实际 ${esc((h.actual_resolutions?.[0]||h.resolution)?.join(' × ')||'待测')} · ${h.frames||0} 帧 · ${number(h.measured_fps,2)} FPS${h.read_errors!=null?' · 读帧错误 '+h.read_errors:''}<p>${esc(h.error||h.timing_basis||'')}</p>${(h.checks||[]).map(c=>`<span class="badge ${c.passed?'pass':'incomplete'}">${esc(c.name)}：${c.passed?'是':'否'}</span>`).join(' ')}${h.quality_samples?.length?`<p>平均亮度 ${number(h.quality_samples.at(-1).mean_brightness,1)} / 255 · 清晰度 ${number(h.quality_samples.at(-1).sharpness,1)}</p>`:''}${h.status!=='running'&&h.report_id?`<p><a href="${dexFileUrl(h.report_id)}">下载基础检查 JSON</a></p>`:''}`:'选择设备后开始。';}
};
$('camera-layout').onchange=()=>{const layout=$('camera-layout').value;action('layout',{layout});$('retest-mode').value=layout==='mono'?'mono':'stereo';location.hash='inspect';};
$('health-start').onclick=()=>{const config=window.cameraHealthSettings?.() || dexSettings();if(config)action('health',{settings:{...config,duration_s:10}});};
$('health-stop').onclick=()=>action('stop');
