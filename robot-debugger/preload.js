const { contextBridge, ipcRenderer } = require('electron');
contextBridge.exposeInMainWorld('serial', {
  list: () => ipcRenderer.invoke('serial:list'),
  connect: (path, baudRate) => ipcRenderer.invoke('serial:connect', { path, baudRate }),
  disconnect: () => ipcRenderer.invoke('serial:disconnect'),
  onLine: callback => ipcRenderer.on('serial:line', (_event, line) => callback(line)),
  onStatus: callback => ipcRenderer.on('serial:status', (_event, status) => callback(status))
});
