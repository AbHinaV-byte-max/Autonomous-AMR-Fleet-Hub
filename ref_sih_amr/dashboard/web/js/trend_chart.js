/**
 * Mission Completion Time Trend Chart Renderer
 * Compares Naive Baseline vs. Decentralized DSS over runtime session.
 * SIH 26123 - Bharat Electronics Limited (BEL)
 */

class TrendChart {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext('2d');
    this.resize();
    window.addEventListener('resize', () => this.resize());
  }

  resize() {
    const rect = this.canvas.parentElement.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = rect.width * dpr;
    this.canvas.height = rect.height * dpr;
    this.ctx.scale(dpr, dpr);
    this.displayWidth = rect.width;
    this.displayHeight = rect.height;
  }

  render(trendData) {
    if (!trendData || trendData.length === 0) return;
    const ctx = this.ctx;
    const w = this.displayWidth || 600;
    const h = this.displayHeight || 180;
    const padding = { top: 16, right: 28, bottom: 22, left: 36 };

    ctx.clearRect(0, 0, w, h);

    const chartW = w - padding.left - padding.right;
    const chartH = h - padding.top - padding.bottom;

    // Determine max value for Y scale
    let maxVal = 10;
    trendData.forEach(d => {
      maxVal = Math.max(maxVal, d.baseline_duration_sec, d.decentralized_duration_sec);
    });
    maxVal = Math.ceil(maxVal * 1.15);

    // Axes
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(padding.left, padding.top);
    ctx.lineTo(padding.left, h - padding.bottom);
    ctx.lineTo(w - padding.right, h - padding.bottom);
    ctx.stroke();

    // Y Gridlines and ticks
    ctx.fillStyle = '#64748b';
    ctx.font = '500 10px "JetBrains Mono", monospace';
    ctx.textAlign = 'right';
    for (let i = 0; i <= 3; i++) {
      const val = Math.round((maxVal / 3) * i);
      const y = h - padding.bottom - (chartH / 3) * i;
      ctx.fillText(`${val}s`, padding.left - 6, y + 3);
      
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.04)';
      ctx.beginPath();
      ctx.moveTo(padding.left, y);
      ctx.lineTo(w - padding.right, y);
      ctx.stroke();
    }

    const n = trendData.length;
    const stepX = n > 1 ? chartW / (n - 1) : chartW;

    // 1. Baseline Line (Warm Amber)
    ctx.strokeStyle = '#f59e0b';
    ctx.lineWidth = 2;
    ctx.beginPath();
    trendData.forEach((d, i) => {
      const x = padding.left + i * stepX;
      const y = h - padding.bottom - (d.baseline_duration_sec / maxVal) * chartH;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();

    // 2. Decentralized DSS Line (BEL Signal Amber — Primary)
    ctx.strokeStyle = '#d97706';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    trendData.forEach((d, i) => {
      const x = padding.left + i * stepX;
      const y = h - padding.bottom - (d.decentralized_duration_sec / maxVal) * chartH;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();

    // Draw data point dots
    trendData.forEach((d, i) => {
      const x = padding.left + i * stepX;
      const yDec = h - padding.bottom - (d.decentralized_duration_sec / maxVal) * chartH;
      ctx.fillStyle = '#d97706';
      ctx.beginPath();
      ctx.arc(x, yDec, 3.5, 0, Math.PI * 2);
      ctx.fill();

      const yBase = h - padding.bottom - (d.baseline_duration_sec / maxVal) * chartH;
      ctx.fillStyle = '#f59e0b';
      ctx.beginPath();
      ctx.arc(x, yBase, 3.5, 0, Math.PI * 2);
      ctx.fill();
    });
  }
}
