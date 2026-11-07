const assert = require('node:assert/strict');
const {readFileSync} = require('node:fs');
const {join} = require('node:path');
const {test} = require('node:test');
const vm = require('node:vm');

const source = readFileSync(join(__dirname, '../web/root.js'), 'utf8');
const render = source.slice(source.indexOf('function renderExtrema('), source.indexOf('let faultsInFlight='));

test('Logs renders raw extrema with units and distinguishes missing values from zero', () => {
  const values = new Map();
  const ui = vm.createContext({txt: (id, value) => values.set(id, value)});
  vm.runInContext(render, ui);
  ui.renderExtrema({vin_min_v: 4.7, vin_max_v: 5.6, tps_temp_max_c: 110, asic_temp_max_c: 70});
  assert.equal(values.get('logs-vin-min'), '4.700 V');
  assert.equal(values.get('logs-vin-max'), '5.600 V');
  assert.equal(values.get('logs-tps-max'), '110.0 °C');
  assert.equal(values.get('logs-asic-max'), '70.0 °C');
  ui.renderExtrema({vin_min_v: 0, vin_max_v: null, tps_temp_max_c: NaN, asic_temp_max_c: Infinity});
  assert.equal(values.get('logs-vin-min'), '0.000 V');
  assert.equal(values.get('logs-vin-max'), '--');
  assert.equal(values.get('logs-tps-max'), '--');
  assert.equal(values.get('logs-asic-max'), '--');
  ui.renderExtrema(null);
  assert.ok([...values.values()].every(value => value === '--'));
});

test('manual clear commits before refreshing and preserves displayed values on failure', async () => {
  const button = {}, calls = [];
  let ok = true;
  const ui = vm.createContext({
    faultsRefreshGeneration: 0,
    $: () => button, confirm: () => true,
    fetch: async (url, options) => { calls.push([url, options.method]); return {ok, status: 500}; },
    renderExtrema: () => calls.push('clear display'),
    refreshFaults: async () => calls.push('refresh'),
    setLogStatus: text => calls.push(text),
  });
  vm.runInContext(source.slice(source.indexOf('async function clearExtrema('),
    source.indexOf('let errorsInFlight=')), ui);
  await button.onclick();
  assert.deepEqual(calls, [['/api/extrema/clear', 'POST'], 'clear display', 'refresh']);
  assert.equal(button.disabled, false);
  calls.length = 0; ok = false;
  await button.onclick();
  assert.deepEqual(calls, [['/api/extrema/clear', 'POST'], 'EXTREMES CLEAR FAILED']);
  assert.equal(button.disabled, false);
});

test('a status response started before Clear Extremes cannot restore stale peaks on screen', async () => {
  const shown = [], button = {};
  let finishOld, reads = 0;
  const ui = vm.createContext({
    $: () => button, confirm: () => true,
    fetch: async url => {
      if (url === '/api/extrema/clear') return {ok: true};
      if (++reads === 1) return new Promise(resolve => { finishOld = resolve; });
      return {ok: true, json: async () => ({extrema: {vin_min_v: 5.1}, faults: []})};
    },
    renderExtrema: e => shown.push(e?.vin_min_v ?? null),
    renderFaults() {}, setLogStatus() {},
  });
  vm.runInContext(source.slice(source.indexOf('let faultsInFlight='),
    source.indexOf('let errorsInFlight=')), ui);
  const pending = ui.refreshFaults();
  await ui.clearExtrema();
  finishOld({ok: true, json: async () => ({extrema: {vin_min_v: 4.7}, faults: []})});
  await pending;
  await new Promise(setImmediate);
  assert.deepEqual(shown, [null, 5.1]);
  assert.equal(reads, 2);
});
