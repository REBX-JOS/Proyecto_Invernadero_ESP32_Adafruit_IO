// Sistema de Control de Invernadero - ESP32 (Solo WebSocket)
class GreenhouseController {
  constructor() {
    this.espIp = "10.225.254.5";
    this.ws = null;
    this.isConnected = false;
    this.lastUpdate = null;
    this.dataHistory = {
      temp: [],
      hum: [],
      soil: [],
      timestamps: []
    };
    this.maxHistoryPoints = 30;
    this.currentChart = 'temp';
    this.chart = null;

    this.deviceStates = { 
      foco: false, 
      vent: false, 
      bomba: false, 
      modo_auto: true 
    };

    this.initialize();
  }

  initialize() {
    this.initializeChart();
    this.loadSavedConfig();
    this.setupEventListeners();
    this.initializeWebSocket();
    
    setInterval(() => this.updateUI(), 1000);
  }

  initializeWebSocket() {
    const wsUrl = `ws://${this.espIp}:81/`;
    console.log(`🌐 Conectando WebSocket: ${wsUrl}`);

    this.ws = new WebSocket(wsUrl);

    this.ws.onopen = () => {
      console.log('✅ WebSocket conectado');
      this.isConnected = true;
      this.updateConnectionStatus();
      this.showNotification('Conectado al ESP32', 'success');
    };

    this.ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        console.log('📥 Datos WebSocket:', data);
        this.processData(data);
        this.updateLastUpdate();
      } catch (error) {
        console.error('❌ Error procesando WebSocket:', error, event.data);
      }
    };

    this.ws.onclose = () => {
      console.log('🔌 WebSocket desconectado');
      this.isConnected = false;
      this.updateConnectionStatus();
      this.showNotification('Desconectado del ESP32', 'warning');
      setTimeout(() => this.initializeWebSocket(), 5000);
    };

    this.ws.onerror = (err) => {
      console.error('⚠️ Error WebSocket:', err);
      this.isConnected = false;
      this.updateConnectionStatus();
    };
  }

  // ===== FUNCIONES PARA WEB SOCKET =====
  
  async saveConfigWS() {
    const config = {
      action: "config",
      temp_min: parseFloat(document.getElementById('tmin').value),
      temp_max: parseFloat(document.getElementById('tmax').value),
      soil_low: parseInt(document.getElementById('slow').value),
      soil_high: parseInt(document.getElementById('shigh').value)
    };
    
    // Validación
    if (config.temp_min >= config.temp_max) {
      this.showNotification('❌ Temp mínima debe ser menor que máxima', 'error');
      return;
    }
    
    if (config.soil_low >= config.soil_high) {
      this.showNotification('❌ Umbral bajo debe ser menor que alto', 'error');
      return;
    }
    
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(config));
      this.saveToLocalStorage('greenhouse_config', config);
      this.showNotification('✅ Configuración guardada por WebSocket', 'success');
    } else {
      this.showNotification('❌ No conectado al ESP32', 'error');
    }
  }

  toggleDeviceWS(device) {
    if (this.deviceStates.modo_auto) {
      this.showNotification('⚠️ Sistema en modo AUTOMÁTICO. Cambia a manual para controlar actuadores.', 'warning');
      return;
    }
    
    const newState = !this.deviceStates[device];
    
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      const command = {
        action: "control",
        device: device,
        state: newState ? 1 : 0
      };
      
      this.ws.send(JSON.stringify(command));
      console.log(`📤 Comando por WS: ${device} -> ${newState ? 'ON' : 'OFF'}`);
      
      // Actualizar UI inmediatamente
      this.deviceStates[device] = newState;
      this.updateActuatorStatus();
    } else {
      this.showNotification('❌ No conectado al ESP32', 'error');
    }
  }

  toggleModeWS() {
    const newMode = !this.deviceStates.modo_auto;
    
    // Actualizar localmente
    this.deviceStates.modo_auto = newMode;
    
    // Actualizar UI
    const modeButton = document.getElementById('mode-toggle');
    if (modeButton) {
      modeButton.textContent = newMode ? 'AUTO' : 'MANUAL';
      modeButton.className = newMode ? 'toggle-btn auto-mode' : 'toggle-btn manual-mode';
    }
    
    this.updateActuatorStatus();
    this.showNotification(`✅ Modo cambiado a ${newMode ? 'AUTOMÁTICO' : 'MANUAL'}`, 'success');
    
    // Enviar por WebSocket
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify({
        action: "setMode",
        mode: newMode ? 1 : 0
      }));
    }
  }

  // ===== PROCESAMIENTO DE DATOS =====
  processData(data) {
    // Actualizar estados de dispositivos desde datos recibidos
    if (data.foco !== undefined) {
      this.deviceStates.foco = data.foco === 1;
    }
    if (data.vent !== undefined) {
      this.deviceStates.vent = data.vent === 1;
    }
    if (data.bomba !== undefined) {
      this.deviceStates.bomba = data.bomba === 1;
    }
    if (data.modo_auto !== undefined) {
      this.deviceStates.modo_auto = data.modo_auto === 1;
    }
    
    // Actualizar datos de sensores
    if (data.temp_LM35 !== undefined) {
      this.updateSensorDisplay('temp', data.temp_LM35);
    }
    if (data.temp_amb !== undefined) {
      this.updateSensorDisplay('tempAmb', data.temp_amb);
    }
    if (data.hum_amb !== undefined) {
      this.updateSensorDisplay('hum', data.hum_amb);
    }
    if (data.soil_adc !== undefined) {
      this.updateSensorDisplay('soil', data.soil_adc);
      this.updateSoilBar(data.soil_adc);
    }
    
    // Actualizar historial para gráficas
    if (data.temp_LM35 !== undefined && data.hum_amb !== undefined && data.soil_adc !== undefined) {
      this.addToHistory({
        temp: data.temp_LM35,
        hum: data.hum_amb,
        soil: data.soil_adc,
        timestamp: new Date().toLocaleTimeString()
      });
      
      this.updateChart();
    }
    
    this.updateActuatorStatus();
  }

  updateSensorDisplay(sensorId, value) {
    const element = document.getElementById(sensorId);
    if (element) {
      const formattedValue = sensorId === 'temp' || sensorId === 'tempAmb' 
        ? value.toFixed(1) 
        : Math.round(value).toString();
      
      element.textContent = formattedValue;
      element.classList.add('value-updated');
      setTimeout(() => element.classList.remove('value-updated'), 500);
    }
  }

  updateSoilBar(soilValue) {
    const fillElement = document.getElementById('soil-fill');
    if (fillElement) {
      const invertedValue = 4095 - soilValue;
      const percentage = Math.max(0, Math.min(100, (invertedValue / 4095) * 100));
      fillElement.style.width = `${percentage}%`;
      
      if (percentage < 30) {
        fillElement.style.backgroundColor = 'rgba(211, 47, 47, 0.7)';
      } else if (percentage < 70) {
        fillElement.style.backgroundColor = 'rgba(255, 152, 0, 0.7)';
      } else {
        fillElement.style.backgroundColor = 'rgba(76, 175, 80, 0.7)';
      }
    }
  }

  addToHistory(data) {
    this.dataHistory.temp.push(data.temp);
    this.dataHistory.hum.push(data.hum);
    this.dataHistory.soil.push(data.soil);
    this.dataHistory.timestamps.push(data.timestamp);
    
    if (this.dataHistory.temp.length > this.maxHistoryPoints) {
      this.dataHistory.temp.shift();
      this.dataHistory.hum.shift();
      this.dataHistory.soil.shift();
      this.dataHistory.timestamps.shift();
    }
  }

  updateActuatorStatus() {
    const devices = ['foco', 'vent', 'bomba'];
    
    devices.forEach(device => {
      const isOn = this.deviceStates[device];
      
      const statusElement = document.getElementById(`${device}-status`);
      if (statusElement) {
        statusElement.textContent = isOn ? 'ON' : 'OFF';
        statusElement.className = isOn ? 'status-on' : 'status-off';
      }
      
      const ledElement = document.getElementById(`${device}-led`);
      if (ledElement) {
        ledElement.className = isOn ? 'led led-on' : 'led led-off';
      }
      
      const buttonElement = document.getElementById(`${device}-btn`);
      if (buttonElement) {
        buttonElement.textContent = isOn ? 'Apagar' : 'Encender';
        buttonElement.className = isOn ? 'control-btn btn-on' : 'control-btn btn-off';
        buttonElement.disabled = this.deviceStates.modo_auto;
      }
    });
    
    const modeButton = document.getElementById('mode-toggle');
    if (modeButton) {
      modeButton.textContent = this.deviceStates.modo_auto ? 'AUTO' : 'MANUAL';
      modeButton.className = this.deviceStates.modo_auto ? 'toggle-btn auto-mode' : 'toggle-btn manual-mode';
    }
  }

  // ===== CONFIGURACIÓN =====
  loadDefaultConfig() {
    const defaultConfig = {
      temp_min: 20,
      temp_max: 24,
      soil_low: 400,
      soil_high: 2000
    };
    
    document.getElementById('tmin').value = defaultConfig.temp_min;
    document.getElementById('tmax').value = defaultConfig.temp_max;
    document.getElementById('slow').value = defaultConfig.soil_low;
    document.getElementById('shigh').value = defaultConfig.soil_high;
    
    this.showNotification('🔄 Valores por defecto cargados', 'info');
  }

  loadSavedConfig() {
    const savedConfig = this.loadFromLocalStorage('greenhouse_config');
    if (savedConfig) {
      document.getElementById('tmin').value = savedConfig.temp_min || 20;
      document.getElementById('tmax').value = savedConfig.temp_max || 24;
      document.getElementById('slow').value = savedConfig.soil_low || 400;
      document.getElementById('shigh').value = savedConfig.soil_high || 2000;
    }
  }

  // ===== GRÁFICAS =====
  initializeChart() {
    const ctx = document.getElementById('chart').getContext('2d');
    
    this.chart = new Chart(ctx, {
      type: 'line',
      data: {
        labels: [],
        datasets: [{
          label: 'Temperatura (°C)',
          data: [],
          borderColor: '#d32f2f',
          backgroundColor: 'rgba(211, 47, 47, 0.1)',
          borderWidth: 2,
          fill: true,
          tension: 0.4
        }]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        plugins: {
          legend: {
            labels: { color: '#333', font: { size: 12 } }
          }
        },
        scales: {
          x: {
            grid: { color: 'rgba(0, 0, 0, 0.1)' },
            ticks: { color: '#666', maxTicksLimit: 10 }
          },
          y: {
            grid: { color: 'rgba(0, 0, 0, 0.1)' },
            ticks: { color: '#666' },
            beginAtZero: false
          }
        }
      }
    });
  }

  showChart(type) {
    this.currentChart = type;
    
    ['temp', 'hum', 'soil'].forEach(chartType => {
      const btn = document.getElementById(`${chartType}-chart-btn`);
      if (btn) {
        btn.classList.toggle('active', chartType === type);
      }
    });
    
    this.updateChart();
  }

  updateChart() {
    if (!this.chart) return;
    
    const labels = this.dataHistory.timestamps;
    let data, label, color;
    
    switch (this.currentChart) {
      case 'temp':
        data = this.dataHistory.temp;
        label = 'Temperatura (°C)';
        color = '#d32f2f';
        break;
      case 'hum':
        data = this.dataHistory.hum;
        label = 'Humedad (%)';
        color = '#1565c0';
        break;
      case 'soil':
        data = this.dataHistory.soil;
        label = 'Humedad Suelo (ADC)';
        color = '#2e7d32';
        break;
    }
    
    this.chart.data.labels = labels;
    this.chart.data.datasets[0].data = data;
    this.chart.data.datasets[0].label = label;
    this.chart.data.datasets[0].borderColor = color;
    this.chart.data.datasets[0].backgroundColor = this.hexToRgba(color, 0.1);
    
    this.chart.update();
  }

  hexToRgba(hex, alpha) {
    const r = parseInt(hex.slice(1, 3), 16);
    const g = parseInt(hex.slice(3, 5), 16);
    const b = parseInt(hex.slice(5, 7), 16);
    return `rgba(${r}, ${g}, ${b}, ${alpha})`;
  }

  // ===== UI =====
  updateConnectionStatus() {
    const statusElement = document.getElementById('connection-status');
    if (statusElement) {
      statusElement.textContent = this.isConnected ? 'Conectado' : 'Desconectado';
      statusElement.className = this.isConnected ? 'status connected' : 'status disconnected';
    }
  }

  updateLastUpdate() {
    this.lastUpdate = new Date();
    const timeString = this.lastUpdate.toLocaleTimeString();
    
    const updateElement = document.getElementById('last-update');
    if (updateElement) {
      updateElement.textContent = `Última actualización: ${timeString}`;
    }
  }

  updateUI() {
    this.updateConnectionStatus();
  }

  showNotification(message, type = 'info') {
    const notification = document.createElement('div');
    notification.className = `notification notification-${type}`;
    notification.innerHTML = `
      <span>${message}</span>
      <button onclick="this.parentElement.remove()">×</button>
    `;
    
    notification.style.cssText = `
      position: fixed;
      top: 20px;
      right: 20px;
      padding: 15px 20px;
      border-radius: 8px;
      color: white;
      font-weight: 500;
      z-index: 1000;
      display: flex;
      align-items: center;
      justify-content: space-between;
      min-width: 300px;
      max-width: 400px;
      animation: slideIn 0.3s ease;
      ${type === 'success' ? 'background: linear-gradient(135deg, #4caf50, #2e7d32);' : ''}
      ${type === 'error' ? 'background: linear-gradient(135deg, #f44336, #d32f2f);' : ''}
      ${type === 'warning' ? 'background: linear-gradient(135deg, #ff9800, #f57c00);' : ''}
      ${type === 'info' ? 'background: linear-gradient(135deg, #2196f3, #1565c0);' : ''}
    `;
    
    notification.querySelector('button').style.cssText = `
      background: none;
      border: none;
      color: white;
      font-size: 24px;
      cursor: pointer;
      margin-left: 15px;
      padding: 0 5px;
    `;
    
    document.body.appendChild(notification);
    
    setTimeout(() => {
      if (notification.parentElement) {
        notification.remove();
      }
    }, 5000);
  }

  // ===== LOCAL STORAGE =====
  saveToLocalStorage(key, data) {
    try {
      localStorage.setItem(key, JSON.stringify(data));
    } catch (error) {
      console.error('❌ Error guardando en localStorage:', error);
    }
  }

  loadFromLocalStorage(key) {
    try {
      const data = localStorage.getItem(key);
      return data ? JSON.parse(data) : null;
    } catch (error) {
      console.error('❌ Error cargando de localStorage:', error);
      return null;
    }
  }

  // ===== EVENT LISTENERS =====
  setupEventListeners() {
    const modeButton = document.getElementById('mode-toggle');
    if (modeButton) {
      modeButton.addEventListener('click', () => this.toggleModeWS());
    }
    
    ['temp', 'hum', 'soil'].forEach(chartType => {
      const btn = document.getElementById(`${chartType}-chart-btn`);
      if (btn) {
        btn.addEventListener('click', () => this.showChart(chartType));
      }
    });
  }
}

// ===== INICIALIZACIÓN GLOBAL =====
let greenhouseApp;

document.addEventListener('DOMContentLoaded', () => {
  greenhouseApp = new GreenhouseController();
  
  // Exponer funciones globales para onclick en HTML
  window.toggleDeviceWS = (device) => greenhouseApp.toggleDeviceWS(device);
  window.saveConfigWS = () => greenhouseApp.saveConfigWS();
  window.loadDefaultConfig = () => greenhouseApp.loadDefaultConfig();
  window.showChart = (type) => greenhouseApp.showChart(type);
  window.toggleModeWS = () => greenhouseApp.toggleModeWS();
});

window.addEventListener('online', () => {
  console.log('🌐 Aplicación en línea');
});

window.addEventListener('offline', () => {
  console.log('📴 Aplicación sin conexión');
});