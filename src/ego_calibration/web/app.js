'use strict';
const $ = id => document.getElementById(id);
const token = document.querySelector('meta[name="camera-token"]').content;
let state = {}, polling = false, uploading = false, pending = false, calibrationKey = '', deviceKey = '', sampleKey = '';
let showReadCalibration = false;
const titles = {inspect: '相机复测', calibration: '设备标定', records: '检测记录', dex: 'Dex / 单目标定', lite: 'Ego-Lite 标定', std: 'Ego-Std 标定'};
const labels = {pass: '通过', fail: '超出阈值', incomplete: '未完成', running: '检测中'};
const form = $('settings-form');
const esc = value => String(value ?? '').replace(/[&<>"']/g, char => ({'&':'&amp;', '<':'&lt;', '>':'&gt;', '"':'&quot;', "'":'&#39;'}[char]));
const number = (value, digits=2) => Number.isFinite(value) ? value.toFixed(digits) : '—';
const previewPlayer = new CameraPreview($('camera-image'), fps=>{
  $('camera-image').classList.remove('hidden'); $('camera-empty').classList.add('hidden');
  $('preview-fps').textContent=`网页预览 ${number(fps,1)} FPS`;
}, message=>{$('preview-fps').textContent='预览重连中'; $('preview-fps').title=message;});
function updatePreview() {
  const available=!!state.stream?.running;
  const wanted=available && !document.hidden && !$('page-inspect').classList.contains('hidden') && $('retest-mode').value==='stereo';
  previewPlayer.update(wanted);
  if (!wanted) $('preview-fps').textContent='网页预览 — FPS';
  else if (!previewPlayer.fps) $('preview-fps').textContent='网页预览 · 等待画面';
  if (!available) {$('camera-image').classList.add('hidden'); $('camera-empty').classList.remove('hidden');}
  updateSample();
}
function updateSample() {
  const sample=state.report?.latest_sample;
  const available=!!state.sample_available && !!sample;
  const interval=state.report?.settings?.interval_s ?? Number(form.elements.interval_s.value);
  const offline=['video','replay'].includes(state.operation) || state.report?.source?.mode==='video';
  $('sample-rate').textContent=`${offline?'视频时间每':'每'} ${number(interval,1)} 秒采样一次`;
  $('sample-detail').textContent=available ? `第 ${sample.index+1} 对 · ${number(sample.timestamp_s,1)} 秒 · ${sample.reason}` : '开始复测后显示最近一次检测结果';
  if (!available) {
    sampleKey=''; $('sample-image').classList.add('hidden'); $('sample-empty').classList.remove('hidden');
    return;
  }
  if (document.hidden || $('page-inspect').classList.contains('hidden')) return;
  const key=`${state.session_id}:${sample.index}`;
  if (key!==sampleKey) {
    sampleKey=key;
    $('sample-image').src='/api/sample.jpg?sample='+encodeURIComponent(key);
  }
}
function toast(message) {$('toast').textContent = message; $('toast').classList.remove('hidden'); clearTimeout(toast.timer); toast.timer = setTimeout(() => $('toast').classList.add('hidden'), 5500);}
async function api(path, options={}) {
  const response = await fetch(path, {...options, headers: {'X-Camera-Token': token, ...options.headers}});
  const type = response.headers.get('content-type') || '';
  const data = type.includes('json') ? await response.json() : {error: await response.text()};
  if (!response.ok) throw new Error(data.error || '请求失败');
  return data;
}
async function action(name, data={}) {
  if (pending || uploading) return;
  pending = true; updateControls();
  try {
    await api('/api/action', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({action:name, data})});
    if (name === 'read') showReadCalibration = true;
    await poll();
  }
  catch(error) {toast(error.message);}
  finally {pending = false; updateControls();}
}
function settings() {
  if (!form.reportValidity()) return null;
  const data = {};
  for (const input of form.querySelectorAll('input[name]')) {
    if (input.type === 'checkbox') data[input.name] = input.checked;
    else if (input.type === 'text') data[input.name] = input.value.trim();
    else data[input.name] = Number(input.value);
  }
  data.tag_size_m = data.tag_size_mm / 1000;
  data.tag_spacing_m = data.tag_spacing_mm / 1000;
  delete data.tag_size_mm; delete data.tag_spacing_mm;
  return data;
}
function navigate(page) {
  if (!titles[page]) page = 'inspect';
  for (const key of Object.keys(titles)) $('page-'+key).classList.toggle('hidden', key !== page);
  document.querySelectorAll('[data-page]').forEach(button => {button.classList.toggle('active', button.dataset.page === page); button.setAttribute('aria-current', button.dataset.page === page ? 'page' : 'false');});
  $('page-title').textContent = titles[page];
  for (const id of ['layout-controls', 'health-panel']) $(id).classList.toggle('hidden', !['inspect','dex'].includes(page));
  $('scope-note').textContent=page==='dex'?'单目标定与 Dex 专用存储':page==='lite'?'DepthAI 双目与 IMU 采集 / 联合标定':page==='std'?'YCTC H.264 / SEI 双目与 IMU 标定':'按单目 / 双目类型复测已有标定；受支持模型与输入详见各流程';
  if (page === 'records') loadRecords();
  if (page === 'dex') window.loadDexCatalog?.();
  if (page === 'lite' || page === 'std') window.loadWorkflowCatalog?.(page);
  if (page === 'inspect') window.loadMonoRetestCatalog?.();
  updatePreview();
}
function showCalibration() {
  location.hash = 'calibration';
  navigate('calibration');
  $('page-calibration').scrollIntoView({block:'start'});
}
document.querySelectorAll('[data-page]').forEach(button => button.onclick = () => {location.hash = button.dataset.page;});
window.addEventListener('hashchange', () => navigate(location.hash.slice(1)));
function updateControls() {
  const busy = !!state.busy || pending || uploading;
  const selected = !!state.selected, calibrated = !!state.calibration;
  const capturing = ['inspect','video','replay'].includes(state.operation);
  $('scan').disabled = busy; $('read').disabled = busy || !selected || !state.capabilities?.read_calibration || state.selected?.identifier === 'offline';
  $('device').disabled = busy; $('preview').disabled = busy || !selected || state.selected?.identifier === 'offline';
  $('close-preview').disabled = busy || !state.stream;
  $('start').disabled = busy || !calibrated || state.selected?.identifier === 'offline' || !state.capabilities?.stereo;
  $('settings-fields').disabled = busy;
  $('stop').classList.toggle('hidden', !capturing);
  $('start').classList.toggle('hidden', capturing);
  $('stop').disabled = pending;
  $('import-video').disabled = busy || !calibrated;
  $('load-json').disabled = busy;
  $('save-json').disabled = !calibrated;
  $('view-calibration').disabled = !calibrated;
  document.querySelectorAll('[data-replay]').forEach(button => {button.disabled = busy;});
}
function badge(element, status, text) {element.className = 'badge '+(status || ''); element.textContent = text;}
function calibrationResolution(payload) {
  return payload?.cameras?.left_mono?.calibration_resolution || payload?.kalibr_calibration?.cam0?.resolution || payload?.resolution;
}
function setDimensions(payload) {
  const c = calibrationResolution(payload);
  if (c) {form.elements.width.value=c[0]; form.elements.height.value=c[1];}
  else {form.elements.width.value=1920; form.elements.height.value=1080;}
  form.elements.resolution_confirmed.checked = false;
  form.elements.width.readOnly = !!c; form.elements.height.readOnly = !!c;
  $('resolution-confirm-row').classList.toggle('hidden', !!c);
  $('resolution-note').textContent = c ? `设备标定记录：${c[0]}×${c[1]}。采集尺寸必须与此一致。` : '设备未记录标定分辨率。可先试算；未核实尺寸时，结果不会判为通过。';
}
function renderCalibrationMatrices(payload) {
  let matrices = [];
  if (payload.cameras) {
    for (const [key, name] of [['left_mono','左目'], ['right_mono','右目'], ['rgb','RGB']]) {
      const camera = payload.cameras[key];
      if (camera) matrices.push([`${name}内参 K / px`,camera.K], [`${name}畸变 D`,camera.distortion_coefficients]);
    }
    matrices.push(['左目 → 右目变换（平移单位 m）',payload.camera_extrinsics?.T_right_mono_from_left_mono?.matrix_m]);
  } else if (payload.kalibr_calibration) {
    for (const [key, name] of [['cam0','左目'], ['cam1','右目']]) {
      const camera = payload.kalibr_calibration[key];
      if (camera) matrices.push([`${name}内参 fx, fy, cx, cy / px`,camera.intrinsics], [`${name}畸变（${camera.distortion_model || '未提供模型'}）`,camera.distortion_coeffs]);
    }
    matrices.push(['左目 → 右目变换（平移单位 m）',payload.kalibr_calibration.cam1?.T_cn_cnm1]);
  } else {
    const data = payload.calibration || payload;
    matrices = [['左目内参 K1 / px',data.K1], ['左目畸变 D1',data.D1], ['右目内参 K2 / px',data.K2], ['右目畸变 D2',data.D2], ['左目 → 右目旋转 R',data.R], [data.T_m ? '左目 → 右目平移 T / m' : '左目 → 右目平移 T / mm',data.T_m || data.T]];
  }
  $('calibration-matrices').innerHTML = matrices.filter(([,value])=>Array.isArray(value) && value.length).map(([name,value])=>{
    const rows = Array.isArray(value[0]) ? value : [value];
    return `<section class="matrix-card"><h3>${esc(name)}</h3><div class="table-scroll"><table aria-label="${esc(name)}"><tbody>${rows.map(row=>`<tr>${row.map(value=>`<td>${number(value,8)}</td>`).join('')}</tr>`).join('')}</tbody></table></div></section>`;
  }).join('');
}
function renderCalibration() {
  const payload = state.calibration;
  const key = JSON.stringify([payload,state.selected,state.validation]);
  if (key === calibrationKey) return;
  calibrationKey = key;
  if (!payload) {
    $('calibration-summary').textContent = '尚未读取设备标定'; $('calibration-matrices').innerHTML=''; $('validation-body').innerHTML=''; $('raw-json').textContent='{}'; return;
  }
  setDimensions(payload);
  const data = [
    ['设备', state.selected?.label || '—'], ['标定格式', payload.format || 'DepthAI EEPROM'],
    ['设备序列号', payload.device_identity?.usb_serial_number || payload.device?.mxid || '—'],
    ['标定 / 固件 SN', payload.header?.serial_number || '—'],
    ['标定样本数', payload.metrics?.sample_count ?? '未提供'],
    ['双目基线', payload.metrics?.baseline_mm ? number(payload.metrics.baseline_mm,3)+' mm' : '见复测报告'],
    ['左 / 右历史 RMS', `${number(payload.metrics?.left_calibrate_rms,3)} / ${number(payload.metrics?.right_calibrate_rms,3)} px（仅展示）`],
    ['设备保存的历史 RMS', payload.metrics?.stereo_rms != null ? number(payload.metrics.stereo_rms,3)+' px（仅展示）' : '未提供'],
  ];
  $('calibration-summary').innerHTML=data.map(([key,value])=>`<div><span class="key">${esc(key)}</span><span class="value">${esc(value)}</span></div>`).join('');
  $('raw-json').textContent=JSON.stringify(payload,null,2);
  renderCalibrationMatrices(payload);
  $('validation-body').innerHTML=(state.validation||[]).map(check=>`<tr><td>${esc(check.category)}</td><td>${esc(check.name)}</td><td><span class="status-label status-${check.passed===false?'fail':check.passed===null?'incomplete':'pass'}">${check.informational?'仅展示':check.passed===null?'无法评估':check.passed?'通过':'异常'}</span></td><td>${esc(check.detail)}</td></tr>`).join('');
}
function renderComparison(report) {
  const acceptance=report?.acceptance, comparison=report?.calibration_comparison;
  const config=report?.settings || Object.fromEntries(['mono_rms_limit','stereo_rms_limit','epipolar_p95_limit'].map(key=>[key,Number(form.elements[key].value)]));
  const defaults=config.mono_rms_limit===1 && config.stereo_rms_limit===1.5 && config.epipolar_p95_limit===1;
  const basis=acceptance?.label || (defaults?'软件默认参考阈值':'用户配置阈值');
  const reference=acceptance?.reference || (report?.settings?.criteria_reference ?? form.elements.criteria_reference.value).trim() || '未提供厂商或项目验收文件';
  $('criteria-basis').textContent=basis;
  $('criteria-summary').textContent=`${report?'本次报告':'待测设置'}：左 / 右 RMS ≤ ${number(config.mono_rms_limit)} px；跨目 RMS ≤ ${number(config.stereo_rms_limit)} px；极线 P95 ≤ ${number(config.epipolar_p95_limit)} px。依据备注：${reference}。`;
  $('comparison-table').classList.toggle('hidden',!comparison);
  $('comparison-empty').classList.toggle('hidden',!!comparison);
  const history=state.calibration?.metrics;
  $('comparison-empty').textContent=report && !comparison ? '此历史报告尚无对比表，可在检测记录中重新分析已保存样本。' : history ? `已有标定 RMS：左 ${number(history.left_calibrate_rms,3)} / 右 ${number(history.right_calibrate_rms,3)} / 双目联合 ${number(history.stereo_rms,3)} px。开始复测后逐项对比；联合 RMS 不能代替跨目预测 RMS。` : '读取或导入标定后，开始复测即可显示逐项对比；设备未保存的历史误差显示“未提供”。';
  $('comparison-body').innerHTML=(comparison?.rows || []).map(row=>{
    const old=Number.isFinite(row.historical_value)?number(row.historical_value,3):'未提供';
    const current=Number.isFinite(row.value)?number(row.value,3):row.status==='reference'?'未复算':'待采样';
    const delta=Number.isFinite(row.delta_px)?`${row.delta_px>=0?'+':''}${number(row.delta_px,3)}`:'—';
    const limit=Number.isFinite(row.limit)?`≤ ${number(row.limit)}`:'—';
    const result=row.status==='reference'?'历史记录':labels[row.status];
    return `<tr title="${esc(row.note)}"><td>${esc(row.name)} <small>${esc(row.statistic.toUpperCase())}</small></td><td>${old}</td><td>${current}</td><td>${delta}</td><td>${limit}</td><td><span class="status-label status-${esc(row.status)}">${esc(result)}</span></td></tr>`;
  }).join('');
  $('comparison-context').textContent=comparison ? `标定 SN：${comparison.calibration_serial || '未提供'} · 历史 ${comparison.historical_sample_count ?? '未知'} 组 · 本次有效 ${report.counts.accepted} 组 · 单目 ${report.resolution.join('×')}。差值 = 本次 − 历史，仅作参考。` : '左右历史拟合与本次复测条件不同，误差增加不直接判为标定退化。';
  const notes=acceptance ? [...acceptance.notes,...new Set(comparison.rows.map(row=>row.note)),comparison.note] : [
    '通过表示满足本次配置及采样条件，不代表厂商、国标或 ISO 认证。可在检测设置中调整阈值并填写依据备注。',
    '左右重投影和跨目预测使用原始单目像素；极线 P95 使用校正后的像素尺度。设备的联合标定 RMS 与跨目预测 RMS 不同，缺失的极线历史值不按零处理。',
    '本次固定已有 K、D、R、T 和基线，只检测现有参数的投影表现。样本数、覆盖范围、距离、倾角和分辨率确认也必须满足要求。',
  ];
  $('comparison-notes').innerHTML=notes.map(note=>`<li>${esc(note)}</li>`).join('');
}
function renderReport() {
  const report=state.report, metrics=report?.metrics || {}, counts=report?.counts;
  renderComparison(report);
  $('metric-views').innerHTML=`${counts?.accepted || 0} <small>/ ${report?.settings.min_views || form.elements.min_views.value}</small>`;
  $('metric-mono').innerHTML=`${number(metrics.left?.rms)} / ${number(metrics.right?.rms)} <small>px</small>`;
  const cross=[metrics.left_to_right?.rms,metrics.right_to_left?.rms].filter(Number.isFinite);
  $('metric-stereo').innerHTML=`${cross.length ? number(Math.max(...cross)) : '—'} <small>px</small>`;
  $('metric-epipolar').innerHTML=`${number(metrics.epipolar?.p95)} <small>px</small>`;
  [0,1].forEach(index=>{const grid=$(index ? 'coverage-right':'coverage-left'); grid.innerHTML=Array.from({length:9},(_,cell)=>`<i class="${report?.coverage[index]?.includes(cell)?'seen':''}" title="第 ${cell+1} 格"></i>`).join('');});
  $('sampling-tip').textContent=report?.latest_sample?.reason || '让标定板依次覆盖中心、四角和边缘';
  const quality=report?.latest_sample?.quality;
  $('quality-line').textContent=quality ? `共同标签 ${report.latest_sample.common_tags} · 清晰度 左 ${number(quality[0].sharpness,0)} / 右 ${number(quality[1].sharpness,0)} · 采样 ${counts.sampled} 对` : '建议采集 30 对以上不同姿态的样本';
  $('sample-clock').textContent=report ? `${Math.floor(state.elapsed_s || 0)} / ${report.settings.duration_s} 秒` : '尚未开始';
  $('capture-progress').max=report?.settings.duration_s || form.elements.duration_s.value;
  $('capture-progress').value=state.elapsed_s || 0;
  badge($('report-status'),report?.status,report?.summary || '等待采集');
  $('checks-empty').classList.toggle('hidden',!!report); $('checks-table').classList.toggle('hidden',!report);
  $('checks-body').innerHTML=(report?.checks || []).map(check=>`<tr><td>${esc(check.name)}</td><td><span class="status-label status-${esc(check.status)}">${labels[check.status]}</span></td><td>${esc(check.detail)}</td></tr>`).join('');
  const complete=report && report.status !== 'running';
  $('report-actions').classList.toggle('hidden',!complete);
  if (complete) {const base='/reports/'+encodeURIComponent(state.session_id)+'/'; $('report-html').href=base+'report.html'; $('report-json').href=base+'report.json'; $('report-csv').href=base+'samples.csv';}
}
function render() {
  const key=JSON.stringify(state.devices);
  if (key!==deviceKey) {deviceKey=key; $('device').innerHTML='<option value="">选择设备</option>'+(state.devices||[]).map(device=>`<option value="${esc(device.identifier)}">${esc(device.label)}</option>`).join('');}
  if (state.selected?.identifier!=='offline') $('device').value=state.selected?.identifier || '';
  $('device-detail').textContent=state.selected ? `${state.selected.model || state.selected.kind} · ${state.selected.serial || state.selected.identifier}` : '普通 UVC / 拼接双目 / Ego / DepthAI / Dex';
  $('notice').textContent=state.notice;
  const error=state.error || state.stream?.error || state.stream?.preview_error;
  $('error-banner').classList.toggle('hidden',!error); $('error-banner').textContent=error || '';
  const stream=state.stream;
  const actual=stream?.resolution || state.report?.source.frame_resolution;
  $('stream-resolution').textContent=actual ? `双目 ${actual[0]}×${actual[1]} · 单目 ${actual[0]/2}×${actual[1]}` : '尺寸待测';
  $('stream-fps').textContent=stream?.running ? '采集 '+number(stream.fps,1)+' FPS' : '采集 — FPS';
  $('frame-caption').textContent=stream?.sync || '实时画面用于调整标定板位置；检测叠加图在下方单独显示';
  const calibrationSize=calibrationResolution(state.calibration);
  const measured=actual || (state.health?.device?.identifier === state.selected?.identifier ? state.health?.resolution : null);
  const referenceSize=state.calibration?.common_calibration?.cam0?.resolution;
  $('resolution-summary').textContent=`${actual ? '当前画面' : '最近基础检查'}实测：${measured ? `单目 ${measured[0]/2}×${measured[1]}，双目 ${measured[0]}×${measured[1]}` : '尚未测量'}。当前标定参数对应的单目分辨率：${calibrationSize ? calibrationSize.join('×') : '未提供'}。${referenceSize ? `附带 common_calibration 参考配置：单目 ${referenceSize.join('×')}，不用于确认当前设备内参的分辨率。` : ''}`;
  const globalText=state.busy ? ({scan:'正在扫描',read:'正在读取',inspect:'现场检测中',video:'视频分析中',replay:'重新分析中'}[state.operation] || '正在处理') : state.report?.summary || (state.calibration?'标定已就绪':state.selected?'设备已选择':'等待设备');
  badge($('global-status'),state.busy?'running':state.report?.status,globalText);
  $('step-device').className=state.calibration?'complete':'current'; $('step-board').className=stream?.resolution?'complete':state.calibration?'current':'';
  $('step-sample').className=state.report?.status==='running'?'current':state.report?'complete':''; $('step-report').className=state.report && state.report.status!=='running'?'complete':'';
  renderCalibration(); renderReport(); updateControls(); window.renderDex?.(); window.renderUniversal?.(); window.renderWorkflows?.();
  updatePreview();
  if (showReadCalibration && !state.busy && !pending) {
    showReadCalibration = false;
    if (!state.error && state.calibration) showCalibration();
  }
}
async function poll() {
  if (polling) return;
  polling=true;
  try {state=await api('/api/state'); $('connection-dot').classList.add('online'); $('connection-label').textContent='本机服务已连接'; render();}
  catch(error) {$('connection-dot').classList.remove('online'); $('connection-label').textContent='本机服务已断开';}
  finally {polling=false;}
}
async function loadRecords() {
  try {const records=await api('/api/records');
    $('records-list').innerHTML=records.length ? records.map(record=>{const base='/reports/'+encodeURIComponent(record.id)+'/'; return `<article class="record"><div><h3>${esc(record.source.device?.label || '相机检测')} <span class="badge ${esc(record.status)}">${esc(record.summary)}</span></h3><p>${esc(new Date(record.created_at).toLocaleString())} · 单目 ${esc(record.resolution.join('×'))} · 有效 ${record.counts.accepted} / 采样 ${record.counts.sampled}</p></div><div class="inline-actions"><a class="button primary" href="${base}report.html" target="_blank" rel="noopener">查看报告</a><a class="button secondary" href="${base}report.json">JSON</a><button class="button secondary" data-replay="${esc(record.id)}">重新分析</button></div></article>`;}).join('') : '<p class="empty-text">暂无记录。完成检测后，报告和原始左右图会自动保存在本机。</p>';
    document.querySelectorAll('[data-replay]').forEach(button=>button.onclick=()=>{action('replay',{session:button.dataset.replay}); location.hash='inspect';}); updateControls();
  } catch(error) {toast(error.message);}
}
$('scan').onclick=()=>action('scan'); $('read').onclick=()=>{if(state.selected?.kind==='dex-mono'){location.hash='dex';action('dex_flash_read');}else action('read');};
$('view-calibration').onclick=showCalibration;
$('preview').onclick=()=>{if(state.capabilities?.mono){window.openInspectionPreview?.();return;}const config=settings(); if(config) {action('preview',{settings:config}); location.hash='inspect';}};
$('close-preview').onclick=()=>action('close_preview');
$('device').onchange=()=>{const identifier=$('device').value;action('select',{identifier});};
form.onsubmit=event=>{event.preventDefault(); const config=settings(); if(config) action('inspect',{settings:config});};
$('stop').onclick=()=>action('stop'); $('refresh-records').onclick=loadRecords;
$('save-json').onclick=()=>{const url=URL.createObjectURL(new Blob([JSON.stringify(state.calibration,null,2)],{type:'application/json'})); const a=document.createElement('a');a.href=url;a.download='ego-calibration.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
$('load-json').onclick=()=>$('json-file').click();
$('json-file').onchange=async event=>{const file=event.target.files[0]; if(!file)return;try {if(file.size>2*1024*1024)throw new Error('标定文件不得超过 2 MB');const text=await file.text();if(/\.ya?ml$/i.test(file.name))await action('load_calibration',{yaml_text:text});else{const payload=JSON.parse(text);await action('load_calibration',{payload,kind:payload.cameras?'ego-lite':'ego-std'});}}catch(error){toast(error.message);}event.target.value='';};
$('import-video').onclick=()=>{if(settings())$('video-file').click();};
$('video-file').onchange=async event=>{
  const file=event.target.files[0], config=settings(); if(!file||!config)return;
  if(file.size>2*1024**3){toast('视频不得超过 2 GB');return;}
  uploading=true;updateControls();$('upload-status').textContent='上传 0%';
  try {
    const upload=await new Promise((resolve,reject)=>{const xhr=new XMLHttpRequest();xhr.open('POST','/api/upload?name='+encodeURIComponent(file.name));xhr.setRequestHeader('X-Camera-Token',token);xhr.setRequestHeader('Content-Type','application/octet-stream');xhr.upload.onprogress=e=>{if(e.lengthComputable)$('upload-status').textContent=`上传 ${Math.round(e.loaded/e.total*100)}%`;};xhr.onload=()=>{try{const data=JSON.parse(xhr.responseText);if(xhr.status>=400)reject(new Error(data.error));else resolve(data);}catch{reject(new Error('视频上传失败'));}};xhr.onerror=()=>reject(new Error('上传连接中断'));xhr.send(file);});
    uploading=false;await action('video',{upload:upload.upload,settings:config});
  }catch(error){toast(error.message);}finally{uploading=false;$('upload-status').textContent='';event.target.value='';updateControls();}
};
document.addEventListener('visibilitychange',updatePreview);
$('sample-image').onload=()=>{if(state.sample_available){$('sample-image').classList.remove('hidden');$('sample-empty').classList.add('hidden');}};
$('sample-image').onerror=()=>{sampleKey='';$('sample-image').classList.add('hidden');$('sample-empty').classList.remove('hidden');};
window.addEventListener('pagehide',()=>previewPlayer.stop());
navigate(location.hash.slice(1)); poll(); setInterval(()=>{if(!document.hidden)poll();},750);
