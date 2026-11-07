const assert = require('node:assert/strict');
const {execFileSync} = require('node:child_process');
const {mkdtempSync, readFileSync, rmSync} = require('node:fs');
const {tmpdir} = require('node:os');
const {join, resolve} = require('node:path');
const {pathToFileURL} = require('node:url');
const {after, test} = require('node:test');
const vm = require('node:vm');
const {inflateSync} = require('node:zlib');

const output = mkdtempSync(join(tmpdir(), 'm45-flasher-test-'));
after(() => rmSync(output, {recursive: true, force: true}));
execFileSync('python3', ['-c', `
import importlib.util, pathlib, sys
spec = importlib.util.spec_from_file_location('packager', sys.argv[1])
packager = importlib.util.module_from_spec(spec)
spec.loader.exec_module(packager)
packager.write_index(pathlib.Path(sys.argv[2]), 'Gamma', 'test', 'factory.bin', '602')
`, resolve(__dirname, '../scripts/tools/package_web_flasher.py'), output]);
const html = readFileSync(join(output, 'index.html'), 'utf8');
const script = html.match(/<script type="module">([\s\S]*?)<\/script>/)[1];
const config = JSON.parse(script.match(/const CONFIG = (.*);/)[1]);
const functions = script.slice(script.indexOf('function flashParts('), script.indexOf('select.addEventListener('));
const image = Uint8Array.from({length: 0x30000}, (_, i) => (i * 71 + (i >>> 8)) & 255);

test('generated flash parts preserve NVS boundaries and every binary byte', () => {
  const ui = vm.createContext({CONFIG: config, NVS_END: config.nvsStart + config.nvsSize, Uint8Array});
  vm.runInContext(functions, ui);
  const parts = ui.flashParts(image, true);
  assert.equal(parts.length, 2);
  assert.ok(parts.every(part => part.data instanceof Uint8Array));
  assert.equal(parts[0].address, 0);
  assert.equal(parts[0].data.length, config.nvsStart);
  assert.equal(parts[1].address, config.nvsStart + config.nvsSize);
  assert.deepEqual(parts[0].data, image.slice(0, config.nvsStart));
  assert.deepEqual(parts[1].data, image.slice(parts[1].address));
  const full = ui.flashParts(image, false);
  assert.equal(full.length, 1);
  assert.equal(full[0].address, 0);
  assert.deepEqual(full[0].data, image);
});

test('actual esptool-js accepts and compresses generated flash data without touching preserved NVS',
  {skip: !process.env.M45_ESPTOOL_BUNDLE}, async () => {
  const {ESPLoader} = await import(pathToFileURL(resolve(process.env.M45_ESPTOOL_BUNDLE)).href);
  for (const eraseSettings of [false, true]) {
    const writes = [];
    let active, disconnected = false, reset = false, status;
    class Transport {
      getInfo() { return 'offline test transport'; }
      async disconnect() { disconnected = true; }
    }
    class OfflineLoader extends ESPLoader {
      constructor(options) {
        super(options);
        this.chip = {BOOTLOADER_FLASH_OFFSET: 0};
        this.IS_STUB = true;
      }
      async main() {}
      async flashDeflBegin(size, compressedSize, address) {
        active = {size, address, chunks: []}; writes.push(active);
        return Math.ceil(compressedSize / this.FLASH_WRITE_SIZE);
      }
      async flashDeflBlock(bytes) { active.chunks.push(Buffer.from(bytes)); }
      async flashDeflFinish() {}
      async hardReset() { reset = true; }
    }
    const button = {}, select = {value: '0'}, erase = {checked: eraseSettings}, progress = {};
    const ui = vm.createContext({
      CONFIG: config, NVS_END: config.nvsStart + config.nvsSize, Uint8Array,
      navigator: {serial: {requestPort: async () => ({})}},
      Transport, ESPLoader: OfflineLoader, firmwareOptions: [config.bundledRelease],
      flashButton: button, select, eraseSettings: erase, progress,
      fetchFirmware: async () => image.buffer,
      setStatus: (message, kind) => { status = {message, kind}; },
      console: {debug() {}, warn() {}, error() {}},
    });
    vm.runInContext(functions, ui);
    await ui.flashFirmware();
    assert.equal(status.kind, 'ok', status.message);
    assert.ok(reset && disconnected);
    assert.equal(button.disabled, false);
    assert.equal(progress.value, 100);
    assert.equal(writes.length, eraseSettings ? 1 : 2);
    for (const write of writes) {
      const bytes = inflateSync(Buffer.concat(write.chunks));
      assert.equal(bytes.length, write.size);
      assert.deepEqual(bytes, Buffer.from(image.slice(write.address, write.address + write.size)));
      if (!eraseSettings) {
        assert.ok(write.address + bytes.length <= config.nvsStart ||
          write.address >= config.nvsStart + config.nvsSize);
      }
    }
  }
});
