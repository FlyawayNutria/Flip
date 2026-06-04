const char WEBPAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Balance Bot Tuning</title>
  <style>
    body { font-family: Arial; padding: 20px; max-width: 400px; margin: auto; text-align: center; }
    input { width: 100%; padding: 8px; margin-bottom: 10px; box-sizing: border-box; }
    .btn {
      padding: 15px;
      font-size: 16px;
      color: white;
      border: none;
      border-radius: 5px;
      cursor: pointer;
      width: 100%;
      transition: all 0.1s ease;
      box-shadow: 0 3px 0 rgba(0,0,0,0.2);
      margin-bottom: 10px;
    }

    .btn:active {
      transform: translateY(2px);
      box-shadow: 0 1px 0 rgba(0,0,0,0.2);
      filter: brightness(0.8);
    }

    .update-btn { background-color: #4CAF50; margin-bottom: 20px; }
    .stop-btn { background-color: #f44336; }
    .gyro-btn { background-color: #ff9800; }
    .dpad { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; margin: 20px 0; }
    .dpad .btn { margin-bottom: 0; }
    .status-box {
      background: #f2f2f2;
      padding: 10px;
      border-radius: 5px;
      margin-bottom: 15px;
      font-size: 16px;
      font-weight: bold;
    }
    .status-armed { color: #4CAF50; }
    .status-disarmed { color: #f44336; }
  </style>
  <script src='https://cdn.jsdelivr.net/npm/chart.js'></script>
</head>
<body>
  <h2>Balance Bot Tuning</h2>

  <div class="status-box">
    Status: <span id="statusText" class="status-disarmed">DISARMED</span>
  </div>

  <div style='width:100%; height:250px; margin-bottom: 20px;'>
    <canvas id='tiltChart'></canvas>
  </div>

  <button id="clearGraphBtn" class="btn" style="background:#607d8b;">
    Clear Graph
  </button>

  <div class="dpad">
    <button id="turn2" class="btn">&#8634;</button><button id="up" class="btn">&#9650;</button><button id="turn1" class="btn">&#8635;</button>
    <button id="left" class="btn">&#9664;</button><button id="stop" class="btn" style="background:#666;">&#9632;</button><button id="right" class="btn">&#9654;</button>
    <div class="empty"></div><button id="down" class="btn">&#9660;</button><div class="empty"></div>
  </div>

  <form action='/update' method='GET'>
    <b>Kp:</b> <input type='number' step='0.01' name='p' value='%KP%'>
    <b>Ki:</b> <input type='number' step='0.01' name='i' value='%KI%'>
    <b>Kd:</b> <input type='number' step='0.01' name='d' value='%KD%'>
    <b>Kv:</b> <input type='number' step='0.01' name='v' value='%KV%'>
    <b>Setpoint:</b> <input type='number' step='0.01' name='t' value='%SP%'>
    <b>Turning Kp:</b> <input type='number' step='0.01' name='tkp' value='%TKP%'>
    <b>Turning Kd:</b> <input type='number' step='0.01' name='tkd' value='%TKD%'>
    <b>Velocity Kp:</b> <input type='number' step='0.01' name='vp' value='%VKP%'>
    <b>Velocity Ki:</b> <input type='number' step='0.01' name='vi' value='%VKI%'>
    <input type='submit' class='btn update-btn' value='UPDATE PID'>
  </form>
  
  <hr style='margin: 20px 0;'>

  <button id="calibrateBtn" class="btn gyro-btn">
    Calibrate Gyro
  </button>
  
  <form action='/stop' method='GET'>
    <input type='submit' class='btn stop-btn' value='EMERGENCY STOP'>
  </form>

<script>
  const ctx = document.getElementById('tiltChart').getContext('2d');
  const chart = new Chart(ctx, {
    type: 'line',
    data: { labels: [], datasets: [
      { label: 'Actual Tilt', borderColor: '#f44336', data: [], pointRadius: 0, borderWidth: 2 },
      { label: 'Setpoint', borderColor: '#2196F3', borderDash: [5,5], data: [], pointRadius: 0, borderWidth: 2 }
    ]},
    options: { animation: false, responsive: true, maintainAspectRatio: false, scales: { x: { display: false } } }
  });
  
  setInterval(() => {
    fetch('/data').then(r => r.json()).then(d => {
      
      // Update Status Text
      const statusEl = document.getElementById('statusText');
      if (d.active) {
        statusEl.innerText = 'ARMED';
        statusEl.className = 'status-armed';
      } else {
        statusEl.innerText = 'DISARMED';
        statusEl.className = 'status-disarmed';
        return; // Don't draw on graph if disarmed
      }

      chart.data.labels.push('');
      chart.data.datasets[0].data.push(d.tilt);
      chart.data.datasets[1].data.push(d.set);

      if(chart.data.labels.length > 150) { 
        chart.data.labels.shift();
        chart.data.datasets[0].data.shift();
        chart.data.datasets[1].data.shift();
      }

      chart.update();

    }).catch(e => console.log('Data fetch failed'));
  }, 200);

  function bindBtn(id, cmd) {
    let el = document.getElementById(id);
    let press = (e) => { e.preventDefault(); fetch('/control?dir=' + cmd); };
    let release = (e) => { e.preventDefault(); fetch('/control?dir=S'); };

    el.addEventListener('mousedown', press);
    el.addEventListener('touchstart', press);
    el.addEventListener('mouseup', release);
    el.addEventListener('touchend', release);
    el.addEventListener('mouseleave', release);
    el.addEventListener('touchcancel', release);
  }

  window.onload = () => {
    bindBtn('up', 'F');
    bindBtn('down', 'B');
    bindBtn('left', 'L');
    bindBtn('right', 'R');
    
    document.getElementById('turn1').onclick = (e) => {
      e.preventDefault();
      fetch('/control?dir=T1');
    };

    document.getElementById('turn2').onclick = (e) => {
      e.preventDefault();
      fetch('/control?dir=T2');
    };

    document.getElementById('stop').onclick = (e) => {
      e.preventDefault();
      fetch('/control?dir=S');
    };

    document.getElementById('calibrateBtn').onclick = (e) => {
      e.preventDefault();
      fetch('/calibrate');
    };

    document.getElementById('clearGraphBtn').onclick = (e) => {
      e.preventDefault();
      chart.data.labels = [];
      chart.data.datasets[0].data = [];
      chart.data.datasets[1].data = [];
      chart.update();
    };
  };
</script>
</body>
</html>
)rawliteral";
