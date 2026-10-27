const { useState, useEffect } = React;

/* ---------------------------------------------------------------- slider */
function VerticalSlider({ label, hint, min, max, value, onChange, disabled }) {
  return (
    <div className="slider-col">
      <div className="title">{label}</div>
      <div className="value">{value.toFixed(2)}</div>
      <input
        type="range"
        className="vertical"
        orient="vertical"
        min={min}
        max={max}
        step={0.05}
        value={value}
        disabled={disabled}
        onChange={(e) => onChange(parseFloat(e.target.value))}
      />
      <div className="hint">{hint}<br />{min.toFixed(0)} … {max.toFixed(0)}</div>
    </div>
  );
}

/* ------------------------------------------------------------------ app */
function App() {
  const [image, setImage] = useState(null);
  const [connected, setConnected] = useState(false);
  const [mode, setMode] = useState('manual');
  const [state, setState] = useState({
    v: 0, w: 0, follow: '', damage: { state: 'unknown', x: 0, length: 0 },
  });
  const [linear, setLinear] = useState(0);
  const [angular, setAngular] = useState(0);

  // Server-Sent Events: camera frames + telemetry.
  useEffect(() => {
    const es = new EventSource('/stream');
    es.onopen = () => setConnected(true);
    es.onerror = () => setConnected(false);
    es.onmessage = (e) => {
      let d;
      try { d = JSON.parse(e.data); } catch (_) { return; }
      if (d.type === 'image') {
        setImage('data:image/jpeg;base64,' + d.jpeg);
      } else if (d.type === 'state') {
        setState(d);
        setMode(d.mode);
      }
    };
    return () => es.close();
  }, []);

  const send = (cmd) => {
    fetch('/cmd', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(cmd),
    }).catch(() => {});
  };

  const chooseMode = (m) => {
    setMode(m);
    send({ mode: m });
  };

  const onManual = (l, a) => {
    setLinear(l);
    setAngular(a);
    send({ linear: l, angular: a });
  };

  const onStop = () => {
    setLinear(0);
    setAngular(0);
    setMode('manual');
    send({ stop: true });
  };

  const dmg = state.damage || { state: 'unknown', x: 0 };
  let damageCls = 'status-line muted';
  let damageText = '护栏检测中…';
  if (dmg.state === 'damage') {
    damageCls = 'status-line damage';
    const where = dmg.x >= 0 ? '前方' : '后方';
    damageText = `⚠ 发现护栏破损（${where} ${Math.abs(dmg.x).toFixed(1)} m）`;
  } else if (dmg.state === 'ok') {
    damageCls = 'status-line ok';
    damageText = '护栏连续 ✓';
  }

  const followText = mode === 'follow'
    ? `寻迹模式：${state.follow || '等待摄像头…'}`
    : '手动控制模式：用摇杆控制车速与转向。';

  return (
    <React.Fragment>
      <div className="app">
        <div className="panel camera-panel">
          <h2>摄像头画面 / Camera</h2>
          <div className="camera-wrap">
            {image
              ? <img src={image} alt="camera" />
              : <div className="camera-placeholder">等待摄像头画面…</div>}
          </div>
        </div>

        <div className="side">
          <div className="panel">
            <h2>控制模式 / Mode</h2>
            <div className="radio-row">
              <button
                className={'mode-btn' + (mode === 'follow' ? ' active' : '')}
                onClick={() => chooseMode('follow')}>
                寻迹 (line following)
              </button>
              <button
                className={'mode-btn' + (mode === 'manual' ? ' active' : '')}
                onClick={() => chooseMode('manual')}>
                手动控制 (manual)
              </button>
            </div>
          </div>

          <div className="panel">
            <h2>护栏检测 / Guardrail</h2>
            <div className={damageCls}>{damageText}</div>
          </div>

          <div className="panel">
            <h2>手动摇杆 / Steering</h2>
            <div className="joystick">
              <VerticalSlider
                label="Linear x" hint="前 +  /  后 −"
                min={-4} max={12} value={linear}
                disabled={mode !== 'manual'}
                onChange={(v) => onManual(v, angular)}
              />
              <VerticalSlider
                label="Angular z" hint="左 +  /  右 −"
                min={-3} max={3} value={angular}
                disabled={mode !== 'manual'}
                onChange={(w) => onManual(linear, w)}
              />
            </div>
            <button className="stop-btn" onClick={onStop}>急停 / Stop</button>
          </div>

          <div className="panel">
            <h2>状态 / Status</h2>
            <div className="status-line muted">{followText}</div>
          </div>
        </div>
      </div>

      <div className="bar">
        <span>image: /camera/image_raw</span>
        <span>cmd: /cmd_vel</span>
        <span>scan: /scan/points</span>
        <span>
          模式 {mode === 'follow' ? '寻迹' : '手动'} ·
          v={state.v.toFixed(2)} m/s · w={state.w.toFixed(2)} rad/s
        </span>
        <span className={'conn ' + (connected ? 'on' : 'off')}>
          {connected ? '已连接' : '未连接'}
        </span>
      </div>
    </React.Fragment>
  );
}

ReactDOM.createRoot(document.getElementById('root')).render(<App />);
