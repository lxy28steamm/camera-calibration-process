'use strict';
let workflowBusy=false, workflowDevice='', liteYamlText='', stdCamchain='', monoRetestYaml='', monoRetestYamlId='', monoRetestVideo='';
let liteReviewToken='', monoResultKey='', liteImageLoading=false, monoImageLoading=false;
const workflowFile=(prefix,id)=>'/'+prefix+'-files/'+id.split('/').map(encodeURIComponent).join('/');
const workflowLinks=(prefix,files)=>(files||[]).map(file=>`<a href="${workflowFile(prefix,file.id)}" target="_blank" rel="noopener">${esc(file.name)}</a>`).join(' · ');
const formNumbers=id=>Object.fromEntries(Array.from($(id).querySelectorAll('input[name],select[name]'),input=>[input.name,input.type==='number'?Number(input.value):input.value]));
function liteBoard(){const values=formNumbers('lite-capture-settings');return Object.fromEntries(['rows','columns','size_mm','gap_mm'].map(key=>[key,values[key]]));}
function monoRetestSettings(){if(!$('mono-retest-settings').reportValidity())return null;return {...formNumbers('mono-retest-settings'),engine:'opencv'};}
function syncSelect(id,entries,preferred){const element=$(id),selected=preferred||element.value;element.innerHTML='<option value="">请选择</option>'+entries.map(([value,label])=>`<option value="${esc(value)}">${esc(label)}</option>`).join('');if(selected&&!entries.some(([value])=>value===selected))element.add(new Option('当前：'+selected,selected));element.value=selected||'';}
function workflowHistory(prefix,entries){$(prefix+'-history').innerHTML=entries.length?entries.map(entry=>`<article class="record"><div><h3>${esc(entry.id)} · ${esc(({import:'导入',capture:'动态采集',imu_noise:'静置采集',solve:'求解',flash:'标定写入',backup:'标定备份',review:'写入参数检查'})[entry.mode]||entry.mode)}</h3><p>${esc(({complete:'流程完成 · 待精度复核',failed:'失败',cancelled:'已停止',running:'处理中'})[entry.status]||entry.status)} ${esc(entry.error||'')}</p>${prefix==='std'&&entry.mode==='solve'?`<details><summary>继续处理使用的原配置（请先核对）</summary><pre>${esc(JSON.stringify({settings:entry.request?.settings,model:entry.request?.model||'pinhole-radtan',runtime:entry.request?.runtime||'native'},null,2))}</pre></details>`:''}<div class="dex-file-links">${workflowLinks(prefix,entry.files)}</div>${prefix==='std'&&entry.yaml_id?`<button type="button" class="button secondary std-history-use" data-yaml="${esc(entry.yaml_id)}">载入此结果复测</button>`:''}${prefix==='std'&&entry.mode==='solve'?`<button type="button" class="button secondary std-resume" data-session="${esc(entry.id)}">按原配置继续到所选阶段</button>`:''}</div></article>`).join(''):'<p class="empty-text">暂无此流程的数据</p>';}
window.loadWorkflowCatalog=async prefix=>{try{const entries=await api('/api/'+prefix+'/catalog');workflowHistory(prefix,entries);if(prefix==='lite'){
  syncSelect('lite-dataset',entries.filter(e=>['capture','import'].includes(e.mode)&&e.files.some(f=>f.name==='camchain.yaml')).map(e=>[e.id,e.id]));
  syncSelect('lite-result-yaml',entries.filter(e=>e.yaml_id).map(e=>[e.yaml_id,e.id]),state.lite?.result?.yaml_id);
}else {syncSelect('std-video',entries.filter(e=>e.mode==='import'&&e.video_id).map(e=>[e.video_id,e.request?.filename||e.id]),state.std?.video_id);syncSelect('std-result-yaml',entries.filter(e=>e.mode==='solve'&&e.yaml_id?.endsWith('-camchain-imucam.yaml')).map(e=>[e.yaml_id,e.id]));}
}catch(error){toast(error.message);}};
window.loadMonoRetestCatalog=async()=>{try{const data=await api('/api/dex/catalog');syncSelect('mono-retest-video',data.videos.map(v=>[v.id,v.name]),state.dex?.video_id);}catch(error){toast(error.message);}};
function modeChanged(){const mono=$('retest-mode').value==='mono';$('stereo-inspection').classList.toggle('hidden',mono);$('mono-inspection').classList.toggle('hidden',!mono);updatePreview();}
$('retest-mode').onchange=modeChanged;
window.cameraHealthSettings=()=>location.hash!=='#dex'&&$('retest-mode').value==='mono'?monoRetestSettings():null;
window.openInspectionPreview=()=>{const config=monoRetestSettings();if(config){$('retest-mode').value='mono';modeChanged();location.hash='inspect';action('dex_preview',{settings:config});}};
$('mono-retest-preview').onclick=openInspectionPreview;
$('mono-retest-record').onclick=()=>{const config=monoRetestSettings();if(config)action('dex_record',{settings:config});};
$('mono-retest-close').onclick=()=>action(state.operation==='dex_record'?'stop':'dex_close');
$('mono-retest-stop').onclick=()=>action('stop');
$('mono-retest-refresh').onclick=loadMonoRetestCatalog;
$('mono-retest-start').onclick=()=>{const config=monoRetestSettings();if(config)action('mono_verify',{settings:config,video_id:$('mono-retest-video').value,yaml_text:monoRetestYaml,yaml_id:monoRetestYamlId});};
$('mono-retest-settings').onsubmit=event=>event.preventDefault();
async function readYaml(input){const file=input.files[0];if(!file)return null;if(file.size>1024**2)throw new Error('YAML 不得超过 1 MB');const text=await file.text();input.value='';return {text,name:file.name};}
$('mono-retest-yaml-file').onchange=async event=>{try{const value=await readYaml(event.target);if(value){monoRetestYaml=value.text;monoRetestYamlId='';$('mono-retest-yaml-name').textContent='复测标定：'+value.name;}}catch(error){toast(error.message);}};
// Reuse a solved/read monocular YAML explicitly; no Dex storage operation is triggered.
const useMono=document.createElement('button');useMono.type='button';useMono.className='button secondary';useMono.textContent='载入当前单目求解 / 读回的 YAML';useMono.id='mono-retest-use-yaml';$('mono-retest-yaml-name').after(useMono);
useMono.onclick=()=>{const text=state.dex?.flash_result?.yaml_text||state.dex?.result?.yaml_text;if(!text){toast('暂无单目 YAML，请先导入、求解或读取');return;}monoRetestYaml=text;monoRetestYamlId='';$('mono-retest-yaml-name').textContent='已载入当前单目 YAML 快照（尺寸与模型将由后端校验）';};
async function uploadWorkflow(file,purpose,statusId){if(file.size>8*1024**3)throw new Error('文件最大 8 GB，本机大文件可用路径导入');uploading=true;updateControls();renderWorkflows();try{return await new Promise((resolve,reject)=>{const xhr=new XMLHttpRequest();xhr.open('POST','/api/upload?purpose='+purpose+'&name='+encodeURIComponent(file.name));xhr.setRequestHeader('X-Camera-Token',token);xhr.upload.onprogress=e=>{$(statusId).textContent=e.lengthComputable?`上传 ${Math.round(e.loaded/e.total*100)}%`:'正在上传';};xhr.onload=()=>{try{const data=JSON.parse(xhr.responseText);xhr.status>=400?reject(new Error(data.error)):resolve(data);}catch{reject(new Error('上传失败'));}};xhr.onerror=()=>reject(new Error('上传连接中断'));xhr.send(file);});}finally{uploading=false;updateControls();renderWorkflows();}}
$('mono-retest-import').onclick=()=>$('mono-retest-video-file').click();
$('mono-retest-video-file').onchange=async event=>{try{const file=event.target.files[0];if(!file)return;const response=await uploadWorkflow(file,'dex','mono-retest-stream');await action('dex_import_video',{upload:response.upload});}catch(error){toast(error.message);}finally{event.target.value='';}};
$('lite-capture').onclick=()=>{if($('lite-capture-settings').reportValidity())action('lite_capture',{duration_s:Number($('lite-capture-settings').elements.duration_s.value),board:liteBoard()});};
$('lite-noise').onclick=()=>{if($('lite-capture-settings').reportValidity())action('lite_noise',{duration_s:Number($('lite-capture-settings').elements.noise_minutes.value)*60});};
$('lite-stop').onclick=()=>action('stop');$('lite-environment').onclick=()=>action('lite_environment');$('lite-refresh').onclick=()=>loadWorkflowCatalog('lite');
$('lite-import').onclick=()=>action('lite_import_dataset',{path:$('lite-import-path').value.trim()});
$('lite-solve').onclick=()=>{if($('lite-capture-settings').reportValidity())action('lite_solve',{dataset_id:$('lite-dataset').value,imu_yaml:$('lite-imu-yaml').value,noise_confirmed:$('lite-noise-confirm').checked,board:liteBoard()});};
$('lite-imu-file').onchange=async event=>{try{const value=await readYaml(event.target);if(value){$('lite-imu-yaml').value=value.text;$('lite-noise-confirm').checked=false;}}catch(error){toast(error.message);}};
function invalidateLiteReview(){liteReviewToken='';$('lite-confirm-write').checked=false;renderWorkflows();}
$('lite-result-file').onchange=async event=>{try{const value=await readYaml(event.target);if(value){liteYamlText=value.text;$('lite-result-yaml').value='';invalidateLiteReview();}}catch(error){toast(error.message);}};
$('lite-result-yaml').onchange=()=>{liteYamlText='';invalidateLiteReview();};
$('lite-check').onclick=()=>action('lite_inspect_result',{yaml_text:liteYamlText,yaml_id:$('lite-result-yaml').value});
$('lite-confirm-write').onchange=()=>renderWorkflows();
$('lite-write').onclick=()=>{if(!liteReviewToken||!$('lite-confirm-write').checked)return;const review_token=liteReviewToken;invalidateLiteReview();action('lite_flash_write',{confirmed:true,review_token});};
$('std-history').onclick=event=>{const use=event.target.closest('.std-history-use');if(use&&!state.busy){action('std_load_result',{yaml_id:use.dataset.yaml});$('retest-mode').value='stereo';modeChanged();location.hash='inspect';return;}const button=event.target.closest('.std-resume');if(button&&!state.busy)action('std_solve',{resume_id:button.dataset.session,stage:$('std-stage').value,parameters_confirmed:$('std-parameters-confirm').checked});};
$('std-environment').onclick=()=>action('std_environment');$('std-refresh').onclick=()=>loadWorkflowCatalog('std');$('std-stop').onclick=()=>action('stop');
$('std-import-local').onclick=()=>action('std_import_video',{path:$('std-import-path').value.trim()});
$('std-import').onclick=()=>$('std-video-file').click();
$('std-video-file').onchange=async event=>{try{const file=event.target.files[0];if(!file)return;const response=await uploadWorkflow(file,'std','std-upload-status');await action('std_import_video',{upload:response.upload});}catch(error){toast(error.message);}finally{event.target.value='';}};
$('std-camchain-file').onchange=async event=>{try{const value=await readYaml(event.target);if(value){stdCamchain=value.text;$('std-camchain-name').textContent='固定双目参数：'+value.name;}}catch(error){toast(error.message);}};
$('std-clear-camchain').onclick=()=>{stdCamchain='';$('std-camchain-name').textContent='默认重新求解双目内外参';};
$('std-settings').onsubmit=event=>event.preventDefault();
$('std-solve').onclick=()=>{if(!$('std-settings').reportValidity())return;const settings=formNumbers('std-settings');settings.tag_size=settings.tag_size_mm/1000;settings.tag_spacing=settings.tag_spacing_mm/1000;delete settings.tag_size_mm;delete settings.tag_spacing_mm;action('std_solve',{settings,video_id:$('std-video').value,runtime:$('std-runtime').value,stage:$('std-stage').value,model:$('std-model').value,camchain_yaml:stdCamchain,parameters_confirmed:$('std-parameters-confirm').checked});};
$('std-use-result').onclick=()=>{const yaml_id=state.std?.result?.yaml_id;if(yaml_id){action('std_load_result',{yaml_id});$('retest-mode').value='stereo';modeChanged();location.hash='inspect';}};
let lastLiteReview='', stdResultKey='', lastStdReview='', stdReviewToken='', stdYamlText='';
function invalidateStdReview(){stdReviewToken='';$('std-confirm-write').checked=false;renderWorkflows();}
$('std-backup').onclick=()=>action('std_flash_read');
$('std-result-file').onchange=async event=>{try{const value=await readYaml(event.target);if(value){if(new TextEncoder().encode(value.text).length>8124)throw new Error('Ego-Std YAML 最大为 8124 字节');stdYamlText=value.text;$('std-result-yaml').value='';$('std-write-filename').textContent='待写入：'+value.name;invalidateStdReview();}}catch(error){toast(error.message);}};
$('std-result-yaml').onchange=()=>{stdYamlText='';$('std-write-filename').textContent='待写入：'+($('std-result-yaml').value||'尚未选择');invalidateStdReview();};
$('std-write-sn').oninput=invalidateStdReview;
$('std-check').onclick=()=>action('std_inspect_result',{yaml_text:stdYamlText,yaml_id:$('std-result-yaml').value,serial_number:$('std-write-sn').value.trim()});
$('std-confirm-write').onchange=()=>renderWorkflows();
$('std-write').onclick=()=>{if(!stdReviewToken||!$('std-confirm-write').checked)return;const review_token=stdReviewToken;invalidateStdReview();action('std_flash_write',{confirmed:true,review_token});};

window.renderWorkflows=function(){const busy=!!state.busy||pending||uploading,operation=state.operation||'';
  const device=JSON.stringify([state.selected?.identifier,state.selected?.kind]);if(device!==workflowDevice){workflowDevice=device;if(state.capabilities?.mono)$('retest-mode').value='mono';else if(state.capabilities?.stereo)$('retest-mode').value='stereo';modeChanged();liteReviewToken='';$('lite-confirm-write').checked=false;}
  $('retest-mode').disabled=busy;
  for(const prefix of ['lite','std']){const item=state[prefix]||{};$(prefix+'-log').textContent=item.log||'尚无任务日志';const env=item.environment||{};$(prefix+'-environment-status').textContent=!env.checked?'尚未检查运行环境':prefix==='lite'?`${env.ready?'Kalibr 就绪':'环境未就绪'} · ${env.label||''} ${(env.missing||[]).join('、')}`:`本机 Kalibr：${env.native_ready?'就绪':'不可用'}；交付 Docker：${env.docker_ready?'就绪':'未配置'}；SEI 提取器：${env.dump_ready?'就绪':'缺失'}。${env.native_ready?'':env.detail||''}`;
    document.querySelectorAll('#page-'+prefix+' button').forEach(button=>{button.disabled=busy;});
    $(prefix+'-stop').disabled=!operation.startsWith(prefix+'_')||operation.endsWith('_flash_write')||pending;
  }
  const lite=state.lite||{},eligible=state.selected?.kind==='ego-lite'&&state.selected?.identifier!=='offline';
  for(const id of ['lite-capture','lite-noise','lite-check'])$(id).disabled=busy||!eligible;
  $('lite-solve').disabled=busy||!$('lite-noise-confirm').checked||!$('lite-dataset').value;
  const review=JSON.stringify([lite.review,lite.review_token]);if(review!==lastLiteReview){lastLiteReview=review;liteReviewToken=lite.review_token||'';$('lite-confirm-write').checked=false;$('lite-review').innerHTML=lite.review?`<p>目标：${esc(lite.review.device.label)} / ${esc(lite.review.device.identifier)}</p><p>YAML SHA-256：${esc(lite.review.sha256)}</p><pre>${esc(JSON.stringify(lite.review.result,null,2))}</pre><p>${esc(lite.review.note)}</p>`:'先选择 Ego-Lite 并检查结果';}
  $('lite-write').disabled=busy||!eligible||!liteReviewToken||!$('lite-confirm-write').checked;
  const p=lite.progress||{};$('lite-progress').textContent=p.elapsed_s!=null?`${number(p.elapsed_s,1)} / ${p.duration_s} 秒 · 左 ${p.left_frames??'—'} 帧 / 右 ${p.right_frames??'—'} 帧 · IMU ${p.imu_samples||0} 条`:'双目目标 20 Hz · IMU 200 Hz。静置采集只保存噪声分析数据。';
  const std=state.std?.result,key=JSON.stringify(std);if(key!==stdResultKey){stdResultKey=key;$('std-result').innerHTML=std?`<p>${esc(({complete:'处理完成 · 待精度复核',running:'处理中',failed:'失败',cancelled:'已停止'})[std.status]||std.status)} ${esc(std.error||'')}</p>${std.verified?'<p><strong>写入、激活及逐字节回读验证成功</strong></p>':''}${std.current?`<details><summary>写入前 / 备份标定</summary><pre>${esc(JSON.stringify(std.current,null,2))}</pre></details>`:''}${std.calibration?`<pre>${esc(JSON.stringify(std.calibration,null,2))}</pre>`:''}${std.conversion_summary?`<p>实际用于求解：单目 ${esc(std.conversion_summary.image_width)}×${esc(std.conversion_summary.image_height)}</p>`:''}<div class="dex-file-links">${workflowLinks('std',std.files)}</div>`:'尚无结果';}
  $('std-use-result').disabled=busy||!std?.yaml_id;
  const stdState=state.std||{},stdEligible=['ego-std','ego-std-235'].includes(state.selected?.kind)&&state.selected?.identifier?.startsWith('/dev/');
  $('std-write-credential').textContent=stdState.write_credential_configured?'本机写入凭据已配置':'尚未配置本机写入凭据：请参阅部署说明；仍可读取并备份。';
  $('std-backup').disabled=busy||!stdEligible;
  $('std-check').disabled=busy||!stdEligible||!(stdYamlText||$('std-result-yaml').value);
  const stdReview=JSON.stringify([stdState.review,stdState.review_token]);
  if(stdReview!==lastStdReview){lastStdReview=stdReview;stdReviewToken=stdState.review_token||'';$('std-confirm-write').checked=false;const r=stdState.review;$('std-write-review').innerHTML=r?`<p><strong>目标：${esc(r.device.label)}</strong><br>USB SN：${esc(r.device.serial)}<br>写入标定 SN：${esc(r.serial_number)}</p><p>写入尺寸（单目）：${esc(r.new.resolution.join(' × '))} · 基线 ${number(r.new.baseline_mm,3)} mm</p><p>YAML SHA-256：${esc(r.new.sha256)}<br>写前备份 SHA-256：${esc(r.current.sha256)}</p><details open><summary>新标定参数（请核对）</summary><pre>${esc(JSON.stringify(r.new.calibration,null,2))}</pre></details><details><summary>相机现有标定（对照）</summary><pre>${esc(JSON.stringify(r.current.calibration,null,2))}</pre></details><div class="dex-file-links">${workflowLinks('std',r.files)}</div>`:'先选择 Ego-Std，并检查待写入结果。';}
  $('std-write').disabled=busy||!stdEligible||!stdState.write_credential_configured||!stdReviewToken||!$('std-confirm-write').checked;
  for(const id of ['std-result-yaml','std-result-file','std-write-sn','std-confirm-write'])$(id).disabled=busy;
  $('std-write-progress').textContent=stdState.progress?.stage||'';

  for(const id of ['mono-retest-preview','mono-retest-record'])$(id).disabled=busy||!state.capabilities?.mono;
  $('mono-retest-close').disabled=(!state.dex?.stream)||pending||(busy&&operation!=='dex_record');$('mono-retest-stop').disabled=operation!=='mono_verify'||pending;
  $('mono-retest-start').disabled=busy||!$('mono-retest-video').value||!(monoRetestYaml||monoRetestYamlId);for(const id of ['mono-retest-import','mono-retest-refresh','mono-retest-use-yaml'])$(id).disabled=busy;
  const stream=state.dex?.stream;$('mono-retest-stream').textContent=stream?`${stream.resolution?.join('×')||'尺寸待测'} · ${stream.recording?'录制':'预览'} ${number(stream.elapsed_s,1)} 秒 · ${stream.error||stream.detection||''}`:'';
  const result=state.dex?.result,rkey=JSON.stringify(result);if(rkey!==monoResultKey){monoResultKey=rkey;const v=result?.verification;$('mono-retest-result').innerHTML=v?.mode==='fixed_parameter_verification'?`<p><strong>${esc(({pass:'通过本次要求',fail:'误差超限',incomplete:'条件不完整'})[v.status])}</strong> · 实际 ${v.resolution.join('×')} · 有效 ${v.accepted} 帧 · 留出角点 RMS ${number(v.holdout_error_px?.rms,3)} px</p>${v.checks.map(c=>`<p><span class="badge ${c.passed?'pass':'incomplete'}">${c.passed?'通过':'未通过'}</span> ${esc(c.name)}</p>`).join('')}<p>${esc(v.scope)}</p><div class="dex-file-links">${dexLinks(result.files)}</div>`:result?.error?esc(result.error):'尚无单目复测结果';}
  $('mono-retest-log').textContent=state.dex?.log||'尚无任务日志';
  if(state.dex?.video_id&&state.dex.video_id!==monoRetestVideo){monoRetestVideo=state.dex.video_id;loadMonoRetestCatalog();}
  if(workflowBusy&&!state.busy){loadWorkflowCatalog('lite');loadWorkflowCatalog('std');loadMonoRetestCatalog();}workflowBusy=!!state.busy;
};
$('lite-noise-confirm').onchange=renderWorkflows;$('lite-dataset').onchange=renderWorkflows;$('mono-retest-video').onchange=renderWorkflows;
$('lite-image').onload=$('lite-image').onerror=()=>{liteImageLoading=false;};$('mono-retest-image').onload=$('mono-retest-image').onerror=()=>{monoImageLoading=false;};
setInterval(()=>{if(document.hidden)return;if(location.hash==='#lite'&&state.operation==='lite_capture'&&!liteImageLoading){liteImageLoading=true;$('lite-image').classList.remove('hidden');$('lite-image').src='/api/lite/frame.jpg?t='+Date.now();}if((location.hash==='#inspect'||!location.hash)&&$('retest-mode').value==='mono'&&state.dex?.stream?.preview_frames&&!monoImageLoading){monoImageLoading=true;$('mono-retest-image').classList.remove('hidden');$('mono-retest-empty').classList.add('hidden');$('mono-retest-image').src='/api/dex/frame.jpg?t='+Date.now();}},100);
modeChanged();if(['#lite','#std'].includes(location.hash))loadWorkflowCatalog(location.hash.slice(1));else loadMonoRetestCatalog();
