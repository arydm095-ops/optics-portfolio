const works = {
  prism: {
    title: '三楔镜折射扫描仿真', category: '01 / REFRACTION & SCANNING',
    src: 'demos/risleyPrismSimulator_standalone.html?v=2',
    description: '基于逐面 Snell 折射计算光线传播，比较楔角、转速、相位与轴心位置变化对扫描轨迹的影响。二维轨迹、光路几何与三维视图在同一界面中显示。',
    focus: '观察重点：折射追迹 / 扫描覆盖 / 参数对比',
    steps: ['先查看默认参数下的扫描轨迹。', '修改楔角、转速比或相位，点击“运行仿真”比较变化。', '点击“动画预览”观察光线运动，也可导出 CSV。'], height: 930
  },
  coude: {
    title: '四镜库德指向误差补偿', category: '02 / REFLECTION & CALIBRATION',
    src: 'demos/coude/coude_debug_console.html',
    description: '虚拟四镜平台与调试控制台联合运行。通过多姿态扫描记录光斑脱靶量，依据反射几何计算调镜建议，再重新扫描比较补偿前后的指向误差。',
    focus: '观察重点：多姿态误差 / 调镜补偿 / RMS 对比',
    steps: ['先在虚拟平台中点击“随机固定误差”，为模型加入非零误差。', '选择“快速检查”，点击“一键自动扫描”，等待采集完成。', '依次点击“生成调镜建议”和“确认执行调镜”，再点击“验证扫描”比较 RMS/MAX。'], height: 1330
  },
  are: {
    title: 'ARE 运动与跟踪演示', category: '03 / KINEMATICS & TRACKING',
    src: 'demos/aer_demo.html',
    description: '在三维场景中联动观察 A/R/E 三轴姿态、目标视线与飞行轨迹。支持自动跟踪、KF-CV/IMM 轨迹预测对比，以及过顶区域的肩侧相机协同演示。',
    focus: '观察重点：轴系运动 / 视线关系 / 过顶协同',
    steps: ['拖动场景旋转视角，使用滚轮缩放。', '点击“开启自动追踪”，观察目标与三轴运动。', '展开“肩侧广角相机”并启用，查看过顶区域的协同过程。'], height: 820
  }
};
const requested = new URLSearchParams(location.search).get('project');
const key = Object.hasOwn(works, requested) ? requested : 'prism';
const work = works[key];
document.title = work.title + ' · 张杨阳';
for (const field of ['title', 'category', 'description', 'focus']) document.getElementById(field).textContent = work[field];
work.steps.forEach(text => { const li = document.createElement('li'); li.textContent = text; document.getElementById('steps').append(li); });
const frame = document.getElementById('demo-frame');
frame.title = work.title;
frame.src = work.src;
document.getElementById('demo-stage').style.height = work.height + 'px';
document.getElementById('direct-link').href = work.src;
document.querySelector('[data-project="' + key + '"]').setAttribute('aria-current', 'page');
document.getElementById('fullscreen').addEventListener('click', async () => {
  try { await document.getElementById('demo-stage').requestFullscreen(); }
  catch { showMessage('可点击“独立窗口打开”查看完整演示'); }
});
document.getElementById('copy').addEventListener('click', async () => {
  try { await navigator.clipboard.writeText(location.href); showMessage('本作品链接已复制'); }
  catch { showMessage('请复制浏览器地址栏中的链接'); }
});
function showMessage(text) {
  const box = document.getElementById('message'); box.textContent = text; box.style.display = 'block';
  clearTimeout(window.messageTimer); window.messageTimer = setTimeout(() => box.style.display = 'none', 3000);
}
