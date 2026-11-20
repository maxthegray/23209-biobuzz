import {finite, indexAt, sonarReading, hasSonar, SONAR_TOO_FAR_IN, SONAR_MIN_IN} from './core.mjs';
import {$, state, fmt, node} from './ui.js';

// The Sonar view: both front ultrasonics as bars, 0 to 30 in (past that the robot won't trust them),
// with the skew between them and, when the op-mode logged one, the distance it was driving to.
const SCALE_IN = SONAR_TOO_FAR_IN;

export function renderSonar() {
  const root = $('sonar');
  root.replaceChildren();
  const bars = node('div', 'sonar-bars');
  state.sonar = {sides: {}, note: node('p', 'sonar-note'), summary: node('div', 'sonar-summary')};
  for (const side of ['left', 'right']) {
    const column = node('div', 'sonar-side'), track = node('div', 'sonar-track'), fill = node('i', 'sonar-fill'), target = node('b', 'sonar-target');
    const value = node('strong', 'sonar-value'), volts = node('small', 'sonar-volts');
    target.hidden = true;
    track.append(fill, target);
    column.append(node('span', 'sonar-name', side === 'left' ? 'LEFT' : 'RIGHT'), value, track, volts);
    bars.append(column);
    state.sonar.sides[side] = {fill, target, value, volts, column};
  }
  root.append(bars, state.sonar.summary, state.sonar.note);
}

// A distance on the bar, 0 at the bottom (touching) to the top (SCALE_IN).
const percent = valueIn => `${Math.max(0, Math.min(1, valueIn / SCALE_IN)) * 100}%`;

function latest(series, time) {
  const index = indexAt(series || [], time);
  return index >= 0 && finite(series[index][1]) ? series[index][1] : null;
}

export function updateSonar() {
  if (!state.sonar || !state.run) return;
  const playback = state.run.playback, logged = hasSonar(playback);
  const reading = logged ? sonarReading(playback, state.time) : null;
  const targetIn = latest(playback['DriveToWall/targetIn'], state.time);
  for (const side of ['left', 'right']) {
    const parts = state.sonar.sides[side], value = reading?.[`${side}In`] ?? null;
    const bad = value !== null && (!finite(value) || value < SONAR_MIN_IN), far = finite(value) && value > SONAR_TOO_FAR_IN;
    parts.column.classList.toggle('bad', bad);
    parts.column.classList.toggle('far', far);
    parts.fill.style.height = finite(value) && !bad ? percent(value) : '0%';
    parts.value.textContent = value === null ? '—' : bad ? '?' : `${fmt(value, 1)} in`;
    parts.value.title = bad ? 'Bad reading: no supply, or under 1 in (a loose signal wire)' : far ? `Past ${SONAR_TOO_FAR_IN} in: the robot won't drive on this` : '';
    parts.target.hidden = targetIn === null;
    if (targetIn !== null) parts.target.style.bottom = percent(targetIn);
    const signal = reading?.[`${side}SignalVolts`], vcc = reading?.[`${side}VccVolts`], raw = reading?.[`${side}RawIn`];
    parts.volts.textContent = finite(signal) || finite(vcc)
      ? `raw ${finite(raw) ? fmt(raw, 1) : '—'} in · ${finite(signal) ? fmt(signal, 3) : '—'} / ${finite(vcc) ? fmt(vcc, 2) : '—'} V`
      : '';
  }
  const wallIn = latest(playback['WallApproach/wallIn'], state.time);
  state.sonar.summary.textContent = !reading ? '' : [
    reading.skewIn === null ? null : `skew ${reading.skewIn >= 0 ? '+' : ''}${fmt(reading.skewIn, 1)} in (${fmt(Math.abs(reading.angleDeg), 1)}° ${reading.angleDeg >= 0 ? 'turned right' : 'turned left'})`,
    wallIn === null ? null : `wall ${fmt(wallIn, 1)} in`,
    targetIn === null ? null : `target ${fmt(targetIn, 1)} in`,
  ].filter(Boolean).join(' · ');
  state.sonar.note.textContent = !logged ? 'This run has no ultrasonic channels.' : !reading ? 'No ultrasonic sample at this time.'
    : reading.mount.logged ? '' : 'Sensor spacing not in this log: using 7.5 in.';
}
